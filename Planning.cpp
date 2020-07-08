#include "stdafx.h"
#include "Planning.h"
#include "GAC_Auotpilot_DP.h"

// 存储历史帧路点数据
GlobalPoint2D CPlanning::last_Bpoints[200];

CPlanning::CPlanning()
{
	his_behavior = 1;
}

CPlanning::~CPlanning()
{

}

CPlanning& CPlanning::Instance()
{
	static CPlanning thePlanning;
	return thePlanning;
}

BYTE CPlanning::startCPlanningThread()
{
	DWORD dwPlanningThreadId;
	if (CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)CPlanning::threadEntry,
		this, 0, &dwPlanningThreadId) != NULL)
	{
		return 1;
	}
	return 0;
}


DWORD CPlanning::threadEntry(LPVOID lpParam)
{
	return (((CPlanning*)lpParam)->CPlanningThread());
}


/***********************************************

************************************************/
DWORD CPlanning::CPlanningThread(void)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	double hisTime = 0;
	double period_max_tmp = 0;
	BYTE count = 0;                          // 循环记录
	static PlanningOut Planningresult;       // 规划结果
	static PlanningStatus Planningshow;      // 规划结果用于显示
	
	LARGE_INTEGER new_time;                  // 记录当前时刻
	new_time.QuadPart = 0;
	QueryPerformanceCounter(&new_time);
	hisTime = (double)new_time.QuadPart;

	BYTE start_flag = 0;

	memset(last_Bpoints, 0, sizeof(last_Bpoints));

	while (true)
	{
		memset(&Planningshow, 0, sizeof(Planningshow));     // 初始化规划线程结构体数据
		DWORD dw = WaitForSingleObject(app->x_DecisionEvent, 1000);
		if (dw != 0)
		{
			continue;
		}
				
		/*计算线程周期*/
		new_time.QuadPart = 0;
		QueryPerformanceCounter(&new_time);
		double curTime = (double)new_time.QuadPart;
		z_period_last = (curTime - hisTime) / SYS_Frequency * 1000;

		if (start_flag == 0)   // 第一次period_max等于period_last
		{
			period_max_tmp = z_period_last;
			start_flag = 1;
		}
		else
		{
			if (z_period_last > period_max_tmp)
			{
				period_max_tmp = z_period_last;
			}
		}
		z_period_max = period_max_tmp;
		hisTime = curTime;

		// 获取决策结果信息
		app->x_criticalDecision.Lock();
		z_DecisionOut = app->GetDesicionOut();        //读取决策结果
		app->x_criticalDecision.Unlock();

		// 获取车辆定位信息
		app->x_criticalLocation.Lock();
		z_LocationOut = app->GetLocationOut();        // 读取定位结果
		app->x_criticalLocation.Unlock();
		
		// 获取车辆位姿信息
		app->x_criticalVhclHisPos.Lock();
		z_VehStatus = app->GetVehStatus();            // 读取车辆状态,车辆位姿信息
		app->x_criticalVhclHisPos.Unlock();

		// 获取车辆周围障碍物信息
		app->x_criticalObstacle.Lock();
		z_Obs = app->GetObj();                        // 读取障碍物信息
		app->x_criticalObstacle.Unlock();

		// 定义规划路点
		GlobalPoint2D road_points[200] = {};

		// 计算预瞄距离
		Calculate_aim_dis(z_DecisionOut, z_LocationOut, z_VehStatus, faraim_dis, nearaim_dis);

		// 搜索预瞄点
		SearchAimPoint(z_DecisionOut, z_LocationOut, z_VehStatus, aimpoint_far, aimpoint_near);

		// 初始规划
		if (count == 0)
		{
			InitialPlanning(z_DecisionOut,z_LocationOut,z_VehStatus,aimpoint_far,aimpoint_near, road_points);	
			memcpy(last_Bpoints, road_points, sizeof(road_points));
		}

		// 判定车辆与局部路径相对距离
		GetVhclLocalState(z_LocationOut, last_Bpoints, path_lat_dis, path_dir_err, path_near_id, path_front_near_id, remain_dis);

		// 重规划判定
		afresh_planning = UpdatePlanJudge(z_DecisionOut, z_LocationOut, his_behavior, afresh_cause);

		// 路径规划 		
		if (afresh_planning)   // 重新规划局部路径
		{			
			PathPlanning(z_DecisionOut,afresh_cause, z_LocationOut, aimpoint_far, aimpoint_near, road_points);
			// memcpy(last_Bpoints, road_points, sizeof(road_points));
		}		
		else                  // 沿用上一帧局部路径
		{
			memset(road_points, 0, sizeof(road_points));
			memcpy(road_points, last_Bpoints, sizeof(last_Bpoints));
		}
		
		
		// 将路点结构体数组转化成vector向量
		// vector<GlobalPoint2D>road_points_ser(road_points, road_points + 200);

		// 存储规划路径的剩余路点
		vector<GlobalPoint2D>rem_path;
		rem_path.clear();
		for (int i = path_near_id;i < 200;i++)
		{
			rem_path.push_back(road_points[i]);
		}

		// 局部路径障碍物参数
		double mindist_lat=999;         // 局部路径障碍物横向距离
		double mindist_lon=999;         // 局部路径障碍物纵向距离
		ObPoint min_ob_pt;              // 局部路径障碍物坐标
		WORD mindist_pathid;            // 局部路径上障碍物路点ID
		bool ob_flag=false;             // 局部路径上障碍物标志位

		// 搜索局部路径障碍物
		ob_flag=SearchObstacle(rem_path,z_Obs,(float)(-1.1),(float)(1.1),mindist_lat,mindist_lon,min_ob_pt,mindist_pathid);

		// 车速规划
		SpeedPlanning(ob_flag, z_DecisionOut, z_LocationOut, mindist_lon, mindist_lat, faraim_dis, brakespeed, acc_flag, des_acc);	

		// 规划状态需显示在调试界面
		Planningshow.afresh_cause = afresh_cause;          // 重规划原因
		Planningshow.near_ob_dist = mindist_lon;           // 局部规划路径最近障碍物距离
		Planningshow.planspeed = brakespeed;               // 局部规划
		Planningshow.planacc = des_acc;                    // 规划结果
		Planningshow.trafficlight=z_DecisionOut.light;     // 交通灯

		for (UINT i = 0;i < 100;i++)
		{
			Planningshow.path_points[i] = road_points[2*i];		
		}

		
		app->SetPlanningStatus(Planningshow);
		
		// 输出规划结果
		Planningresult.cnt = count % 100;
		Planningresult.APA = 0;
		Planningresult.brakedis = mindist_lon;
		Planningresult.brake_speed = 0;
		Planningresult.desaccVd = acc_flag;
		Planningresult.desacc = des_acc;
		Planningresult.desspd = brakespeed;
		Planningresult.desstr = 0; 
		Planningresult.desstrVd = false;             // 定位精度可用
		Planningresult.light = z_DecisionOut.light;  
		Planningresult.radius = CalculateRadius();   // 计算曲率半径
		Planningresult.road_type = 0;
		Planningresult.sstop = TRUE;
		
		GPSPoint2D gps_points;
		GlobalPoint2D path_points;
		for (UINT i = 0;i < 100;i++)
		{
			path_points.x = road_points[2 * i].x;
			path_points.y = road_points[2 * i].y;
			gps_points = GlobalToWGS84(path_points);
			Planningresult.pnts[i].x = gps_points.lat;
			Planningresult.pnts[i].y = gps_points.lng;
		}
		// 存储规划结果数据
		app->SetUdpSendCtrl(Planningresult);
		// 存储上一帧数据
		his_behavior = z_DecisionOut.behavior;                      // 存储上一帧行为
		memcpy(last_Bpoints, road_points, sizeof(road_points));     // 存储上一帧

		count++;   // 每发送一次给控制，计数一次
		if (count % 100 == 1)
		{
			count = 1;
		}
		SetEvent(app->x_PlanningEvent);     // 规划线程完成
	
	}
	return 0;
}



/*********************************************************************************************************
功能：分别计算路上、预路口和路口场景预瞄距离
输入：
decision_result 决策输出结果
vhcl_location 车辆定位
vhcl_status  车辆位姿
输出：
faraim_dis 远预瞄点距离
nearaim_dis 近预瞄点距离
**********************************************************************************************************/
void CPlanning::Calculate_aim_dis(DecisionOut decision_result, LocationOut vhcl_location, VehStatus vhcl_status, FLOAT &faraim_dis, FLOAT &nearaim_dis)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	BYTE road_task = 0;                // 任务状态，0表示路上，1表示预路口，2表示路口
	road_task = vhcl_location.pos;     // 确定车辆所处任务状态
	
	// 远近预瞄距离初始化
	faraim_dis = 0;
	nearaim_dis = 0;
	
	switch (road_task)
	{
		// 路上预瞄距离计算
	case 0:   
		// 路上远预瞄点距离
		faraim_dis = (vhcl_location.velocity / 3.6) * 5 + 4;   
		// 路上远预瞄距离限制
		if (faraim_dis > ROAD_FARAIM_MAX)
		{
			faraim_dis = ROAD_FARAIM_MAX;
		}
		else if (faraim_dis < ROAD_FARAIM_MIN)
		{
			faraim_dis = ROAD_FARAIM_MIN;
		}
		// 路上近预瞄距离
		nearaim_dis = faraim_dis;
		break;

		// 预路口预瞄距离
	case 1:  
		// 预路口远预瞄点距离
		faraim_dis = PRE_INTER_FARAIM;
		// 预路口近预瞄点距离
		nearaim_dis = faraim_dis;
		break;

		// 路口预瞄距离
	case 2:
		// 路口远预瞄点距离
		faraim_dis = INTER_FARAIM;
		// 路口近预瞄点距离
		nearaim_dis = faraim_dis;
		break;
	default:
		break;		
	}
}


/*********************************************************************************************************
功能：搜索远近预瞄点
输入：
decision_result 决策输出结果
vhcl_location 车辆定位
vhcl_status  车辆位置状态
输出：
aimpoint_far 远预瞄点
aimpoint_near 近预瞄点
**********************************************************************************************************/
void CPlanning::SearchAimPoint(DecisionOut decision_result, LocationOut vhcl_location, VehStatus vhcl_status, AimPoint &aimpoint_far, AimPoint &aimpoint_near)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	BYTE road_task = 0;                // 任务状态，0表示路上，1表示预路口，2表示路口
	BOOL plan_divflag = false;         // 分段规划标志位，false表示不分段规划，true表示分段规划
	DOUBLE sum_dis = 0;                // 计算距离
	INT curpoint_sum=999;              // 当前车道路点总数
    // INT tarpoint_sum = 999;         // 目标车道路点总数
	INT curpoint_id = 0;               // 当前车道路点ID
	INT aimpoint_id = 0;               // 预瞄点ID
	INT leftpoint_id = 0;              // 当前车辆对应左侧车道路点ID
	INT rightpoint_id = 0;             // 当前车道对应右侧车道路点ID
	INT leftpoint_sum = 0;             // 左侧车道左侧车道路点总数
	INT rightpoint_sum = 0;            // 右侧车道右侧车道路点总数

	// 确定车辆所处任务状态
	road_task = vhcl_location.pos;   

	// 车辆在路口位置
	WORD LastRoadNum_Cur = vhcl_location.last_roadnum; // 路口上一段道路编号
	WORD NextRoadNum_Cur = vhcl_location.next_roadnum; // 路口下一段道路编号
	WORD LastLaneNum_Cur = vhcl_location.last_lanenum; // 路口上一段道路的车道编号
	WORD NextLaneNum_Cur = vhcl_location.next_lanenum; // 路口下一段道路的车道编号

	// 车辆在路上位置
	WORD RoadNum_Cur = vhcl_location.road_num;         // 路上道路编号
	WORD LaneNum_Cur = vhcl_location.lane_num;         // 路上车道编号
	WORD LaneSum = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][0].lane_sum;     // 车辆所在道路车道总数
	
	// 路上及路口车辆位置
	if (road_task == 0 || road_task == 1)     // 路上及预路口
	{
		BYTE RoadNum_Cur = vhcl_location.road_num;         // 车辆在当前道路编号		
		BYTE LaneNum_Cur = vhcl_location.lane_num;         // 车辆在当前车道编号
		int Id_Cur = vhcl_location.id[LaneNum_Cur - 1];    // 车辆在当前车道路点ID
		// 路上路点总数
		WORD IdSum_CurLane = app->planning_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1].size();
		// 路口路点总数
		WORD IdSum_Inter = app->planning_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1].size();
	}
	else                                      // 路口
	{
		BYTE RoadNum_Next = vhcl_location.road_num;         // 车辆下一段道路编号		
		BYTE LaneNum_Next = vhcl_location.lane_num;         // 车辆下一段车道编号
		int Id_Inter = vhcl_location.id[LastLaneNum_Cur - 1];  // 车辆在路口车道
	    // 路口路点总数
		WORD IdSum_Inter = app->planning_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1].size();
		// 下一段路上路点总数
		WORD IdSum_NextLane = app->planning_MapData[RoadNum_Next - 1][LaneNum_Next - 1].size();
	}	
		
	curpoint_id = vhcl_location.id[vhcl_location.lane_num - 1];                                            // 当前车道路点id		
	curpoint_sum = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1].size();   // 当前车道路点总数
	
	// 左侧车道路点和路点总数
	if (LaneNum_Cur > 1)
	{
		leftpoint_id = vhcl_location.id[LaneNum_Cur - 2];
		leftpoint_sum= app->planning_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2].size();
	}
	else
	{
		leftpoint_id = 0;
		leftpoint_sum = 0;
	}
	
	// 右侧车道路点和路点总数
	if (LaneNum_Cur < LaneSum)
	{
		rightpoint_id = vhcl_location.id[LaneNum_Cur];
		rightpoint_sum = app->planning_MapData[RoadNum_Cur - 1][LaneNum_Cur].size();
	}
	else
	{
		rightpoint_id = 0;
		rightpoint_sum = 0;
	}
	
	// 目标车道路点总数
    //tarpoint_sum=app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1].size();

	// 判定是否采用分段规划
	if (fabs(faraim_dis - nearaim_dis) < 1)
	{
		plan_divflag = false;         // 不分段规划
	}
	else
	{
		plan_divflag = true;          // 分段规划
	}

	// 分别确定预瞄点
	switch (road_task)
	{
		
	case 0:   // 路上预瞄点

		if (decision_result.target_lanenum == vhcl_location.lane_num)
		{
			// 当行为决策结果为不换道跟车行驶
			if (decision_result.behavior == 1)
			{
				// 不分段规划
				if (plan_divflag == false)
				{
					// 累加距离复位
					for (int i = curpoint_id;i < curpoint_sum - 1;i++)
					{
						// 累加计算
						sum_dis += sqrt(pow(app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1][i + 1].global_point.x - app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1][i].global_point.x, 2) +
							pow(app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1][i + 1].global_point.y - app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1][i].global_point.y, 2));

						if (sum_dis - 4 > faraim_dis)
						{
							aimpoint_far.Aim_point.x = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1][i].global_point.x;
							aimpoint_far.Aim_point.y = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1][i].global_point.y;
							aimpoint_far.Aim_point.dir = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1][i].global_point.dir;
							aimpoint_far.Aim_id = i;
							break;
						}
						else
						{
							aimpoint_far.Aim_point.x = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1][curpoint_sum - 1].global_point.x;
							aimpoint_far.Aim_point.y = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1][curpoint_sum - 1].global_point.y;
							aimpoint_far.Aim_point.dir = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1][curpoint_sum - 1].global_point.dir;
							aimpoint_far.Aim_id = curpoint_sum - 1;
						}

					}
					// 不用分段规划时，近预瞄点和远预瞄点重叠
					aimpoint_near = aimpoint_far;
				}
				
			}
		
		}
		else if (decision_result.target_lanenum != vhcl_location.lane_num)
		{
			// 向左换道行驶
			if (decision_result.behavior == 2)
			{
				// 不分段规划
				if (plan_divflag == false)
				{
					for (int i = leftpoint_id;i < leftpoint_sum - 1;i++)
					{
						// 累加计算
						sum_dis += sqrt(pow(app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 2][i + 1].global_point.x - app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 2][i].global_point.x, 2) +
							pow(app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 2][i + 1].global_point.y - app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 2][i].global_point.y, 2));

						if (sum_dis - 4 > faraim_dis)
						{
							aimpoint_far.Aim_point.x = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 2][i].global_point.x;
							aimpoint_far.Aim_point.y = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 2][i].global_point.y;
							aimpoint_far.Aim_point.dir = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 2][i].global_point.dir;
							aimpoint_far.Aim_id = i;
							break;
						}
						else
						{
							aimpoint_far.Aim_point.x = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1][leftpoint_sum - 2].global_point.x;
							aimpoint_far.Aim_point.y = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1][leftpoint_sum - 2].global_point.y;
							aimpoint_far.Aim_point.dir = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1][leftpoint_sum - 2].global_point.dir;
							aimpoint_far.Aim_id = leftpoint_sum - 1;
						}
					}
				}
			}
			// 向右换道行驶
			else if (decision_result.behavior == 3)
			{
				// 不分段规划
				if (plan_divflag == false)
				{
					for (int i = rightpoint_id;i < leftpoint_sum - 1;i++)
					{
						// 累加计算
						sum_dis += sqrt(pow(app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num][i + 1].global_point.x - app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num][i].global_point.x, 2) +
							pow(app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num][i + 1].global_point.y - app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num][i].global_point.y, 2));

						if (sum_dis - 4 > faraim_dis)
						{
							aimpoint_far.Aim_point.x = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num][i].global_point.x;
							aimpoint_far.Aim_point.y = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num][i].global_point.y;
							aimpoint_far.Aim_point.dir = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num][i].global_point.dir;
							aimpoint_far.Aim_id = i;
							break;
						}
						else
						{
							aimpoint_far.Aim_point.x = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num ][rightpoint_sum - 1].global_point.x;
							aimpoint_far.Aim_point.y = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num ][rightpoint_sum - 1].global_point.y;
							aimpoint_far.Aim_point.dir = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num ][rightpoint_sum - 1].global_point.dir;
							aimpoint_far.Aim_id = rightpoint_sum - 1;
						}
					}
				}
			}			
		}

		break;
	case 1:   // 预路口预瞄点
		sum_dis = 0;
		for (int i = 0;i < decision_result.refpath.size()-1;i++)
		{
			sum_dis += CalcDistance(decision_result.refpath[i], decision_result.refpath[i + 1]);

			// 确定预瞄点
			if ((sum_dis - 4 )> faraim_dis)
			{
				// 预瞄点赋值
				aimpoint_far.Aim_point.x = decision_result.refpath[i].x;
				aimpoint_far.Aim_point.y = decision_result.refpath[i].y;
				if (i < decision_result.refpath.size() - 4)
				{
					aimpoint_far.Aim_point.dir = CalcGlobalDir(decision_result.refpath[i], decision_result.refpath[i + 2]);
				}
				else
				{
					aimpoint_far.Aim_point.dir= CalcGlobalDir(decision_result.refpath[i-2], decision_result.refpath[i]);
				}		
				aimpoint_far.Aim_id = i;			
				break ;

			}
			else
			{
				aimpoint_far.Aim_point.x = decision_result.refpath[decision_result.refpath.size() - 1].x;
				aimpoint_far.Aim_point.y = decision_result.refpath[decision_result.refpath.size() - 1].y;
				aimpoint_far.Aim_point.dir = CalcGlobalDir(decision_result.refpath[decision_result.refpath.size() - 3], 
					decision_result.refpath[decision_result.refpath.size() - 1]);
				aimpoint_far.Aim_id = decision_result.refpath.size() - 1;				
			}
			
		}
		aimpoint_near = aimpoint_far;		
		break;
	case 2:   // 路口预瞄点
		sum_dis = 0;
		for (int i = 0;i < decision_result.refpath.size() - 1;i++)
		{
			sum_dis += CalcDistance(decision_result.refpath[i], decision_result.refpath[i + 1]);

			// 确定预瞄点
			if ((sum_dis - 4) > faraim_dis)
			{
				// 预瞄点赋值
				aimpoint_far.Aim_point.x = decision_result.refpath[i].x;
				aimpoint_far.Aim_point.y = decision_result.refpath[i].y;
				if (i < decision_result.refpath.size() - 4)
				{
					aimpoint_far.Aim_point.dir = CalcGlobalDir(decision_result.refpath[i], decision_result.refpath[i + 2]);
				}
				else
				{
					aimpoint_far.Aim_point.dir = CalcGlobalDir(decision_result.refpath[i - 2], decision_result.refpath[i]);
				}
				aimpoint_far.Aim_id = i;
				//aimpoint_id = i;
				break;

			}
			else
			{
				aimpoint_far.Aim_point.x = decision_result.refpath[decision_result.refpath.size() - 1].x;
				aimpoint_far.Aim_point.y = decision_result.refpath[decision_result.refpath.size() - 1].y;
				aimpoint_far.Aim_point.dir = CalcGlobalDir(decision_result.refpath[decision_result.refpath.size() - 3],
					decision_result.refpath[decision_result.refpath.size() - 1]);
				aimpoint_far.Aim_id= decision_result.refpath.size() - 1;
				//aimpoint_id = decision_result.refpath.size() - 1;
			}

		}
		aimpoint_near = aimpoint_far;
		break;
	default:
		break;
	}

}

/***************************************************************************************************************
初始规划
输入：
decision_result 决策输出结果
vhcl_location 车辆定位
vhcl_status 车辆位姿
aimpoint_far 远预瞄点
aimpoint_near 近预瞄点
输出：

****************************************************************************************************************/
void CPlanning::InitialPlanning(DecisionOut decision_result, LocationOut vhcl_location, VehStatus vhcl_status, const AimPoint aimpoint_far, const AimPoint aimpoint_near,GlobalPoint2D road_points[])
{
	GlobalPoint2D Bezier_points[200] = {};                    // 定义存储路点
	GlobalPoint3D vhcl_point = vhcl_location.globalpoint;   // 车辆当前坐标
	
	// 初始化存储路点
	memset(road_points, 0, sizeof(road_points));     
	memset(Bezier_points, 0, sizeof(Bezier_points)); 

	// 初始规划
	BezierPlanning(vhcl_point, aimpoint_far.Aim_point, Bezier_points, 200);

	// 拷贝初始化规划的路点
	memcpy(road_points,Bezier_points, sizeof(Bezier_points));

}

/***********************************************************************************************************************************
计算车辆相对于局部路径的位置
输入：
vhcl_location 车辆定位
last_Bpoints 上一周期的局部规划路点
输出：
mindist_lat 相对局部路径横向偏差
remain_dis 剩余距离
path_dir_err 航向误差
************************************************************************************************************************************/
void CPlanning::GetVhclLocalState(LocationOut vhcl_location, const GlobalPoint2D last_Bpoints[], double& mindist_lat, double& path_dir_err, int& mindist_id, int& front_mindist_id, double& remain_dis)
{	
	GlobalPoint2D vhcl_pose;           // 定义当前位置点坐标
	double dist;
	double front_min_dis = 9999;
	double middle_min_dis = 9999;
	GlobalPoint2D pt, pt_next;	
	
	// 信息初始化
	remain_dis = 0;
	mindist_lat = 9999;
	

	// 当前位置坐标赋值
	vhcl_pose.x = vhcl_location.globalpoint.x;
	vhcl_pose.y = vhcl_location.globalpoint.y;
	
	for (UINT i = 0; i < 200; i++) //车辆与已规划路径的横向距离，横向偏差大于一定值需要重规划
	{
		dist = sqrt(pow(vhcl_pose.x - last_Bpoints[i].x, 2) + pow(vhcl_pose.y - last_Bpoints[i].y, 2));
		if (dist < mindist_lat)
		{
			mindist_lat = dist;
			mindist_id = i;       
		}

		front_mindist_id = mindist_id + 8;	     // 车头最近路点	
	}

	
	// 车辆在局部路径上最近ID
	int index = mindist_id;  

	if (mindist_id == 199)
	{
		index = mindist_id - 1;
	}
	pt.x = last_Bpoints[index].x;
	pt.y = last_Bpoints[index].y;
	pt_next.x = last_Bpoints[index + 1].x;
	pt_next.y = last_Bpoints[index + 1].y;

	// 计算车辆 
	mindist_lat = GetLatDis(vhcl_pose, pt, pt_next);

	for (int i = front_mindist_id; i < 199; i++)
	{
		remain_dis += sqrt(pow(last_Bpoints[i + 1].x - last_Bpoints[i].x, 2) + pow(last_Bpoints[i + 1].y - last_Bpoints[i].y, 2));
	}
	// 计算局部路点航向
	double pt_dir = GetRoadAngle(pt, pt_next);
	// 计算航线误差
	path_dir_err = GetAngleErr(pt_dir, vhcl_location.globalpoint.dir);//车辆与已规划路径的方向夹角，夹角大于一定值需要重规划
}
/****************************************************************************************************
计算车辆当前位置与局部路径横向偏差
输入：
cur_pt 车辆当前位置坐标
pt 当前路点坐标
pt_next 当前路点下一个路点坐标
输出：
lat_dis 返回横向误差
*****************************************************************************************************/
double CPlanning::GetLatDis(GlobalPoint2D cur_pt, GlobalPoint2D pt, GlobalPoint2D pt_next)
{
	double lat_dis = 0;

	if (fabs(pt.x - pt_next.x) > EPSILON)
	{
		double k = (pt.y - pt_next.y) / (pt.x - pt_next.x);
		lat_dis = fabs((cur_pt.y - pt.y) - k*(cur_pt.x - pt.x)) / sqrt(1 + k*k);
	}
	else
	{
		lat_dis = fabs(pt.x - cur_pt.x);
	}
	if (lat_dis < EPSILON)
	{
		lat_dis = 0;
	}
	else
	{
		lat_dis = lat_dis*Sgn(((pt_next.x - pt.x)*(cur_pt.y - pt.y) - (pt_next.y - pt.y)*(cur_pt.x - pt.x)));
	}
	return lat_dis;

}

/*********************************************************************************************
根据两点相对路径起点坐标求方向，与东向夹角，逆时针为正，范围：[0,360°]
输入：
apoint:第一点全局坐标
bpoint:第二点全局坐标
输出：
apoint与bpoint连线与正东方向的夹角
********************************************************************************************/
double CPlanning::GetRoadAngle(GlobalPoint2D apoint, GlobalPoint2D bpoint)
{
	double angle;
	if (fabs(bpoint.x - apoint.x) < EPSILON && fabs(bpoint.y - apoint.y) < EPSILON)
		angle = 0;
	else if (fabs(bpoint.x - apoint.x) < EPSILON)
	{
		if (bpoint.y > apoint.y)
		{
			angle = PI / 2;
		}
		else
		{
			angle = 3 * PI / 2;
		}
	}
	else
	{
		angle = atan((bpoint.y - apoint.y) / (bpoint.x - apoint.x));

		if (bpoint.x < apoint.x)
		{
			angle = angle + PI;
		}
		else if ((bpoint.x > apoint.x) && (bpoint.y < apoint.y))
		{
			angle = angle + 2 * PI;
		}
	}
	angle = angle * 180 / PI;
	return angle;
}

/*********************************************************************************************
获取两个方向之间的夹角，逆时针为正，由dir1转向dir2
输入：
dir1:基准方向
dir2:待求夹角方向
输出：
待求夹角方向相对基准方向的夹角，有正负之分(-180°~ 180°]
********************************************************************************************/
double CPlanning::GetAngleErr(double dir1, double dir2)
{
	double angle_err = dir2 - dir1;
	if (dir1 < 180)
	{
		if (dir2 - dir1 <= 180)
		{
			angle_err = dir2 - dir1;
		}
		else
		{
			angle_err = dir2 - dir1 - 360;
		}
	}
	else if (dir1 >= 180)
	{
		if (dir2 - dir1 > -180)
		{
			angle_err = dir2 - dir1;
		}
		else
		{
			angle_err = dir2 - dir1 + 360;
		}
	}
	return angle_err;
}

/*********************************************************************************************
判断是否需要重规划
输入：
obpoints，ob_num:障碍物点和障碍物个数
nearid:局部路径上距离车头最近点的id
输出：
afresh_cause：重规划类型
是否需要重规划
********************************************************************************************/
bool CPlanning::UpdatePlanJudge(const DecisionOut decision_result,const LocationOut vhcl_location,const int last_behavior, int &afreshcause)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();	

	afreshcause = 0;

	
	if (last_behavior != decision_result.behavior) //状态切换造成重规划
	{
		afreshcause = 1;
		return true;
	}

	if (fabs(path_lat_dis) > 0.2)       //横向偏差过大造成重规划
	{
		afreshcause = 2;
		return true;
	}
	if (fabs(path_dir_err) > 45)         //方向偏差过大造成重规划
	{
		afreshcause = 3;
		return true;
	}

	if ((vhcl_location.pos == 0)  && remain_dis < ROAD_REMAIN_DISTANCE )    //路上剩余距离不足重规划
	{
		afreshcause = 4;
		return true;
	}
	else if (vhcl_location.pos != 0 && remain_dis < INTER_REMAIN_DISTANCE)  //预路口和路口剩余距离不足重规划
	{
		afreshcause = 4;
		return true;
	}
	return false;
}

/************************************************************************************************************************************
局部路径规划
输入：
afresh_planning 重规划标志位
afresh_cause 重规划原因
vhcl_location 车辆位姿
aimpoint_far 远预瞄点
aimpoint_near 近预瞄点
输出：
road_points[] 规划的局部路径
*************************************************************************************************************************************/
void CPlanning::PathPlanning(const DecisionOut DecisionResult ,const int afresh_cause, LocationOut vhcl_location, const AimPoint aimpoint_far, const AimPoint aimpoint_near, GlobalPoint2D road_points[])
{
	GlobalPoint2D Bezier_points[200] = {};                  // 定义存储路点
	GlobalPoint3D vhcl_point = vhcl_location.globalpoint;   // 车辆当前坐标
	BYTE vhcl_pos = vhcl_location.pos;                      // 车辆当前路段，0表示路上，1表示预路口，2表示路口
	//vector<GlobalPoint2D> ref_points;
	GlobalPoint2D Ref_points[200] = {};                     // 定义参考存储路点
	

	// 初始化存储路点
	memset(road_points, 0, sizeof(road_points));
	memset(Bezier_points, 0, sizeof(Bezier_points));

	// 初始规划
	// BezierPlanning(vhcl_point, aimpoint_far.Aim_point, Bezier_points, 200);

	if (vhcl_pos==0)
	{
		BezierPlanning(vhcl_point, aimpoint_far.Aim_point, Bezier_points, 200);
	}
	else if (vhcl_pos == 1 || vhcl_pos == 2)
	{
		for (int i = 0;i < aimpoint_far.Aim_id;i++)
		{
			Ref_points[i] = DecisionResult.refpath[i];
		}
		// 将路点均匀化
		MeanPoints(Ref_points, aimpoint_far.Aim_id, Bezier_points, 200);
	}

	// 拷贝初始化规划的路点
	memcpy(road_points, Bezier_points, sizeof(Bezier_points));
}

/****************************************************************************************************************************************
车速规划
输入：
ob_flag 局部路径障碍物标志位
decision_result 决策规划结果
vhcl_location 车辆定位
mindist_lon 局部路径障碍物纵向距离
mindist_lat 局部路径障碍物横向距离
*****************************************************************************************************************************************/
void CPlanning::SpeedPlanning(const bool ob_flag, const DecisionOut decision_result, const LocationOut vhcl_location,
	const double mindist_lon, const double mindist_lat,const FLOAT faraim_dis,double &brake_speed, bool &acc_flag, double &des_acc)
{
	switch (vhcl_location.pos)
	{
	case 0:     // 路上场景车速规划
		if (ob_flag)
		{
			if (mindist_lon-4 > 9)
			{
				brake_speed = 3 + (mindist_lon - 9) / (faraim_dis - 9)*(decision_result.velocity_expect - 3);
				acc_flag = false;
				des_acc = 0;
			}
			else if (mindist_lon-4 > 5)
			{
				brake_speed = 3;
				acc_flag = false;
				des_acc = 0;
			}
			else
			{
				brake_speed = 0;
				acc_flag = true;
				des_acc = -3;
			}			
		}
		else
		{
			brake_speed = decision_result.velocity_expect;
			acc_flag = false;
			des_acc = 0;
			/*int j = 0;
			j= j + 1;*/
		}
		break;
	case 1:     // 预路口场景车速规划
		if (ob_flag)
		{
			if (mindist_lon-4 > 9)
			{
				brake_speed = 3 + (mindist_lon - 9) / (faraim_dis - 9)*(decision_result.velocity_expect - 3);
				acc_flag = false;
				des_acc = 0;
			}
			else if (mindist_lon-4 > 5)
			{
				brake_speed = 3;
				acc_flag = false;
				des_acc = 0;
			}
			else
			{
				brake_speed = 0;
				acc_flag = true;
				des_acc = -3;
			}
		}
		else
		{
			brake_speed = decision_result.velocity_expect;
			acc_flag = false;
			des_acc = 0;
			/*int j = 0;
			j= j + 1;*/
		}
		break;
	case 2:
		if (ob_flag)
		{
			if (mindist_lon-4 > 9)
			{
				brake_speed = 3 + (mindist_lon - 9) / (faraim_dis - 9)*(decision_result.velocity_expect - 3);
				acc_flag = false;
				des_acc = 0;
			}
			else if (mindist_lon-4 > 5)
			{
				brake_speed = 3;
				acc_flag = false;
				des_acc = 0;
			}
			else
			{
				brake_speed = 0;
				acc_flag = true;
				des_acc = -3;
			}
		}
		else
		{
			brake_speed = decision_result.velocity_expect;
			acc_flag = false;
			des_acc = 0;
			/*int j = 0;
			j= j + 1;*/
		}
		break;  // 路口场景车速规划
	default:
		break;  
	}
	
}
/******************************************************************************************************************
计算局部规划路径曲率
输入：
last_Bpoints 上帧规划路点
path_front_near_id 车头在规划路径上id
path_near_id CPT位置在规划路径上id
输出：
radius 道路曲率
*******************************************************************************************************************/
double CPlanning::CalculateRadius()//计算规划路径曲率
{
	double radius;
	UINT midlle_num = static_cast<UINT>(round((path_near_id + path_front_near_id) / 2));
	double dis1 = sqrt(pow((last_Bpoints[path_near_id].x - last_Bpoints[midlle_num].x), 2) + pow((last_Bpoints[path_near_id].y - last_Bpoints[midlle_num].y), 2));
	double dis2 = sqrt(pow((last_Bpoints[midlle_num].x - last_Bpoints[path_front_near_id].x), 2) + pow((last_Bpoints[midlle_num].y - last_Bpoints[path_front_near_id].y), 2));
	double dis3 = sqrt(pow((last_Bpoints[path_near_id].x - last_Bpoints[path_front_near_id].x), 2) + pow((last_Bpoints[path_near_id].y - last_Bpoints[path_front_near_id].y), 2));
	double dis = dis1*dis1 + dis2*dis2 - dis3*dis3;
	double cosA = dis / (2 * dis1*dis2);
	double sinA = sqrt(1 - cosA*cosA);
	if (sinA < 0.001)
	{
		radius = 1000;
	}
	else
	{
		radius = 0.5*dis3 / sinA;
	}
	return radius;
}

