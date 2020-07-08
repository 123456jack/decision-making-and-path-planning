#include "stdafx.h"
#include "Decision.h"
#include "GAC_Auotpilot_DP.h"
#include "V2XTCP.h"
#include <queue>
#include "GAC_Auotpilot_DPDlg.h"

CDecision::CDecision()
{

	z_segment_lanechg_status = 0;
	z_segment_obsavoid_status = 0;
	z_target_lanenum = 0;
	z_light_status = 0;
	z_behavior = 1;               // 初始行为车道保持
	z_velocity_expect = 10;

	z_period_max = 0;
	z_period_last = 0;
	z_behavior_to_dlg = 0;

	leftlight_time = 0;          // 左转向灯时长
	rightlight_time = 0;         // 右转向灯时长

	his_behavior = 1;            // 初始化上一帧行为决策结果
	his_light_status = 0;        // 初始化上一帧转向灯状态
	his_target_lanenum = 0;
	
}

CDecision::~CDecision()
{

}

CDecision& CDecision::Instance()
{
	static CDecision theCDecision;
	return theCDecision;
}

BYTE CDecision::startCDecisionThread()
{
	DWORD dwSendThreadId;
	if (CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)CDecision::threadEntry,
		this, 0, &dwSendThreadId) != NULL)
	{
		return 1;
	}
	return 0;
}


DWORD CDecision::threadEntry(LPVOID lpParam)
{
	return (((CDecision*)lpParam)->CDecisionThread());
}


/***********************************************
决策线程：
0.读取静态数据：道路级导航，道路拓扑关系，地图数据直接调用app->decision_MapData[][][]；
while(1)
{
1.读取感知输入；
2.读取定位输入；
3.读取车辆信息（控制转发过来）；
4.读取规划路径；
5.Switch(Pos)
{
case 路上：
case 预路口：
case 路口：
}
6.发送相应信息给规划；
}
************************************************/
DWORD CDecision::CDecisionThread(void)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();


	z_RoadInfo = app->GetRoadInfo();             // 读取道路拓扑关系

	z_RoadNavi = app->GetRoadNavi();             // 读取道路级导航路径

	//app->x_criticalLocation.Lock();
	//z_LocationOut = app->GetLocationOut();     // 读取定位结果
	//app->x_criticalLocation.Unlock();


	//while (z_LocationOut.period_last == 0)     // 说明定位还未有结果输出；
	//{
	//	z_LocationOut = app->GetLocationOut();   // 读取定位结果
	//	Sleep(50);
	//}

	//z_target_lanenum = z_LocationOut.lane_num;


	double hisTime = 0;
	double period_max_tmp = 0;
	LARGE_INTEGER new_time;
	new_time.QuadPart = 0;
	QueryPerformanceCounter(&new_time);
	hisTime = (double)new_time.QuadPart;
	BYTE start_flag = 0;


	// 障碍物出现的时间及次数
	//UINT Obsavoid_time = 0;                               // 障碍物出现的次数
	//UINT No_obsavoid_time = 0;                            // 绕障过程中没有出现障碍物次数
	//static UINT obsavoid_time = 0;                        // 障碍物出现的次数
	//static UINT no_obsaviod_time = 0;                     // 绕障过程中障碍物消失次数

	static double leftlight_time = 0;                       // 左转向灯时间
	static double rightlight_time = 0;                      // 右转向灯时间   

	while (true)
	{
		DWORD dw = WaitForSingleObject(app->x_PercetionPreProcessingEvent, 1000);
		if (dw != 0)
		{
			continue;
		}	

		DWORD dwLocation = WaitForSingleObject(app->x_LocationEvent, 1000);
		if (dwLocation != 0)
		{
			continue;
		}

		/*计算线程周期*/
		new_time.QuadPart = 0;
		QueryPerformanceCounter(&new_time);
		double curTime = (double)new_time.QuadPart;
		z_period_last = (curTime - hisTime) / SYS_Frequency * 1000;

		if (start_flag == 0)   //第一次period_max等于period_last
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


		app->x_criticalLocation.Lock();
		z_LocationOut = app->GetLocationOut();        //读取定位结果
		app->x_criticalLocation.Unlock();

		z_VehStatus = app->GetVehStatus();            //读取车辆状态  
		z_Obs = app->GetObj();                        //读取障碍物信息

	    //z_DynaObs_front = app->GetDynaObj_front();   // 读取前方动态障碍物信息
	    //z_DynaObs_rear = app->GetDynaObj_rear();     // 读取后方动态障碍物信息
		//z_Person = app->GetPerson();				   // 读取行人信息
		//z_LinePoints = app->GetLinepoints();         // 读取车道线信息
		app->x_criticalV2X_Data.Lock();
		z_V2XData = app->GetV2XData();
		z_RSIWarningPointList = app->GetRSIWarningPointList();
		app->x_criticalV2X_Data.Unlock();

		// 不同场景决策
		switch (z_LocationOut.pos)
		{
		case 0:
			SegmentDecision();           // 路上路径规划
			break;
		case 1:
			PreStubDecision();           // 预路口路径规划
			break;
		case 2:
			StubDecision();              // 路口路径规划
			break;
		default:
			break;
		}

		DecisionOut z_decisionout;
		z_decisionout.period_max = z_period_max;
		z_decisionout.period_last = z_period_last;
		z_decisionout.behavior = z_behavior;
		z_decisionout.target_roadnum = z_target_roadnum;
		z_decisionout.target_lanenum = z_target_lanenum;
		z_decisionout.light = z_light_status;
		z_decisionout.velocity_expect = z_velocity_expect;
		z_decisionout.refpath = z_refpath;
		z_decisionout.behavior_to_dlg = z_behavior_to_dlg;

		// 存储历史帧数据		
		his_behavior = z_behavior;
		his_light_status = z_light_status;		
		his_target_lanenum = z_target_lanenum;

		app->SetDecisionOut(z_decisionout);

		SetEvent(app->x_DecisionEvent);
	}
	return 0;
}

/*************************************************************************************
函数名称：SegmentDecision() 
功能描述：路上行为决策
输入：
输出：
**************************************************************************************/
void CDecision::SegmentDecision()
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	vector<GlobalPoint2D> PathPoint_LongFront;           // 车辆正前方的路径100m
	vector<GlobalPoint2D> PathPoint_Front_Long;
	vector<GlobalPoint2D> PathPoint_LF_Long;
	vector<GlobalPoint2D> PathPoint_RF_Long;

	vector<GlobalPoint2D> PathPoint_Front;                // 车辆正前方的路径60m 
	vector<GlobalPoint2D> PathPoint_Rear;                 // 车辆正后方的路径20m,
	vector<GlobalPoint2D> PathPoint_LF;                   // 车辆左前方的路径60m,如果左侧五车道，向量大小为0	
	vector<GlobalPoint2D> PathPoint_LR;                   // 车辆左后方的路径20m,如果左侧无车道，向量大小为0
	vector<GlobalPoint2D> PathPoint_RF;	                  // 车辆右前方的路径60m，往前方向
	vector<GlobalPoint2D> PathPoint_RR;                   // 车辆右后方的路径20m，往后方向
	
	double Width_CurLane = 0; 	                          //车辆所在车道宽度

	UINT Navi_LaneChg=4;                                  // 导航要求的换道，0表示不要求换道，1表示向左换道，2表示向右换道
	UINT Navi_LaneChg_times=0;                            // 导航要求的换道次数

	Path_Obs Path_Obs_F;                                  // 当前车道正前方障碍物信息
	Path_Obs Path_Obs_R;                                  // 当前车道正后方障碍物信息
	Path_Obs Path_Obs_LF;                                 // 当前车道左前方障碍物信息
	Path_Obs Path_Obs_LR;                                 // 当前车道左后方障碍物信息
	Path_Obs Path_Obs_RF;                                 // 当前车道右前方障碍物信息
	Path_Obs Path_Obs_RR;                                 // 当前车道右后方障碍物信息	

 //   static WORD obsavoid_time = 0;                        // 障碍物出现次数
 //   static WORD no_obsaviod_time = 0;                     // 绕障中，原车道中心没有出现障碍物次数
 //   static WORD frontobs_time = 0;                        // 障碍物出现的次数

	Behavior_Dec Cur_Behavior;                            // 当前行为决策结果
	Behavior_Dec His_Behavior;                            // 历史帧行为决策结果
	
	// V2X标志位初始化
	bool pedestrian_flag = false;
	bool construction_flag = false;
	WORD V2XLight_flag = 0;

	// 参数初始化
	memset(&Path_Obs_F, 0, sizeof(Path_Obs_F));
	memset(&Path_Obs_R, 0, sizeof(Path_Obs_R));
	memset(&Path_Obs_LF, 0, sizeof(Path_Obs_LF));
	memset(&Path_Obs_LR, 0, sizeof(Path_Obs_LR));
	memset(&Path_Obs_RF, 0, sizeof(Path_Obs_RF));
	memset(&Path_Obs_RR, 0, sizeof(Path_Obs_RR));

	memset(&Cur_Behavior, 0, sizeof(Cur_Behavior));
	memset(&His_Behavior, 0, sizeof(His_Behavior));

	// 判定导航是否需要换道
	Nav_LaneChange(z_LocationOut, z_RoadNavi, Navi_LaneChg, Navi_LaneChg_times);

	// 加载车辆周围相对路径	
	LoadRefPath(z_LocationOut, PathPoint_Front, PathPoint_Rear, PathPoint_LF, PathPoint_LR, PathPoint_RF, PathPoint_RR, Width_CurLane);
	
	// 搜索车辆周围障碍物
	AroundObstacle(PathPoint_Front, PathPoint_Rear, PathPoint_LF, PathPoint_LR, PathPoint_RF, PathPoint_RR, Path_Obs_F, Path_Obs_R, Path_Obs_LF, Path_Obs_LR, Path_Obs_RF, Path_Obs_RR, Width_CurLane);
	
	// V2X场景判断函数，输出相应的标志位给行为判定函数
	//V2XPedestrianJudge(z_LocationOut, z_V2XData, PathPoint_LongFront, pedestrian_flag); //弱势行人检测预警

	//V2XSignalLight(z_V2XData, V2XLight_flag); //信号灯下发

	//V2XConstructionEvent(z_LocationOut, z_V2XData, z_RSIWarningPointList, PathPoint_LongFront, construction_flag); //施工道路预警

	V2XEventDecision(z_LocationOut, z_V2XData, z_RSIWarningPointList, PathPoint_Front_Long, PathPoint_LF_Long, PathPoint_RF_Long, pedestrian_flag, V2XLight_flag, construction_flag);

	// 行为决策结果赋值
	Cur_Behavior.behavior = z_behavior;
	Cur_Behavior.light_status = z_light_status;
	Cur_Behavior.target_lanenum = z_target_lanenum;
	Cur_Behavior.lanechg_status = z_segment_lanechg_status;        //  换道状态
	Cur_Behavior.obsavoid_status = z_segment_obsavoid_status;      //  绕障状态
	Cur_Behavior.behavior_to_dlg = z_behavior_to_dlg;   

	His_Behavior.behavior = his_behavior;
	His_Behavior.light_status = his_light_status;
	His_Behavior.target_lanenum = his_target_lanenum;	

	// 行为判定	
	BehaviorDecision(z_LocationOut, z_RoadNavi, Navi_LaneChg, Navi_LaneChg_times, Width_CurLane, PathPoint_Front, Path_Obs_F, Path_Obs_R, Path_Obs_LF, Path_Obs_LR, Path_Obs_RF, Path_Obs_RR,His_Behavior,Cur_Behavior);

	// 车速决策		
	SpeedDecision(Cur_Behavior, z_refpath, z_velocity_expect);

	// 参考路径
	RefPath(Cur_Behavior, PathPoint_Front, PathPoint_LF, PathPoint_RF, z_refpath);	

	// 当前行为决策结果
	z_behavior = Cur_Behavior.behavior;
	z_light_status = Cur_Behavior.light_status;
	z_target_lanenum = Cur_Behavior.target_lanenum;
	z_segment_lanechg_status = Cur_Behavior.lanechg_status;
	z_segment_obsavoid_status = Cur_Behavior.obsavoid_status;
	z_behavior_to_dlg = Cur_Behavior.behavior_to_dlg;	
	z_target_roadnum = z_LocationOut.road_num;

}

/*************************************************************************************
函数名称：PreStubDecision()
功能描述：预路口行为决策
输入：
输出：
**************************************************************************************/
void CDecision::PreStubDecision()
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	// 变量初始化
	WORD light_status;                                 // 转向灯
	vector<GlobalPoint2D> PathPoint_Front;             // 车辆前方路点
	// 当前车道状态
	BYTE RoadNum_Cur = z_LocationOut.road_num;         //车辆所在道路编号		
	BYTE LaneNum_Cur = z_LocationOut.lane_num;         //车辆所在车道编号
	BYTE LastRoadNum_Cur = z_LocationOut.last_roadnum; // 路口上一段道路编号
	BYTE NextRoadNum_Cur = z_LocationOut.next_roadnum; // 路口下一段道路编号
	BYTE LastLaneNum_Cur = z_LocationOut.last_lanenum; // 路口上一段道路的车道编号
	BYTE NextLaneNum_Cur = z_LocationOut.next_lanenum; // 路口下一段道路的车道编号	
	int Id_CurLane = z_LocationOut.id[LaneNum_Cur - 1];// 车辆在当前车道路点ID
	
	// 路况变量声明
	BOOL Obs_flag = false;                             // 预路口及路口障碍物标志位
	Obs_To_Veh Obs_Front;                              // 预路口及路口障碍物相车辆位置信息
	WORD Obs_Pathid_F;                                 // 障碍物在路径上ID
	ObPoint ObPoint_F;                                 // 障碍物坐标及类型
	
	// 路上路点总数
	WORD IdSum_CurLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1].size();   
	// 路口路点总数
	WORD IdSum_Inter = app->decision_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1].size();


	// 将预路口路点压入向量中
	for (WORD i = Id_CurLane; i < IdSum_CurLane; i++)   
	{
		GlobalPoint2D pt;
		pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
		pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
		PathPoint_Front.push_back(pt);
	}

	// 将路口路点压入向量中
	for (WORD j = 0;j < IdSum_Inter;j++)
	{
		GlobalPoint2D pt;
		pt.x = app->decision_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1][j].global_point.x;
		pt.y = app->decision_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1][j].global_point.y;
		PathPoint_Front.push_back(pt);
	}

	// 搜索预路口及路口障碍物
	Obs_flag = SearchObstacle(PathPoint_Front, z_Obs, -0.5*Vehicle_Width, 0.5*Vehicle_Width, Obs_Front.dis_lat, Obs_Front.dis_lng, ObPoint_F, Obs_Pathid_F);

	// 预路口车速规划
	if (Obs_Front.dis_lng < 13) 
	{		
		z_velocity_expect = max(Obs_Front.dis_lng - 3, 0);    // 遇障后预路口车速决策
		z_behavior_to_dlg = 13;                               // 不可变道，路径上有障碍物，不可绕障
	}
	else 
	{
		z_velocity_expect = 10;                   // 预路口限速10
		z_behavior_to_dlg = 1;                    // 不可变道，路径上无障碍物
	}

	// 转向灯
	if (z_RoadNavi[z_LocationOut.path_num].stub_attribute == 3) 
	{
		light_status = 1;                         // 左转灯
	}
	else 
	{ 
		light_status = z_RoadNavi[z_LocationOut.path_num].stub_attribute;
	}
	
	z_behavior = 1;                                 // 车辆道路行为,车道保持
	z_target_roadnum = RoadNum_Cur;                 // 当前目标道路
	z_target_lanenum = LaneNum_Cur;                 // 当前目标车道
	z_velocity_expect = z_velocity_expect;          // 期望车速
	z_refpath = PathPoint_Front;                    // 车辆前方参考路点
	z_light_status = light_status;                  // 车辆前方交通灯状态

	return;
}
/*************************************************************************************
函数名称：StubDecision()
功能描述：路口行为决策
输入：
输出：
**************************************************************************************/
void CDecision::StubDecision()
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	// 变量初始化
	WORD light_status;                                 // 转向灯
	vector<GlobalPoint2D> PathPoint_Front;             // 车辆前方路点
	// 当前车道状态
	BYTE RoadNum_Next = z_LocationOut.road_num;        // 车辆下一段道路编号		
	BYTE LaneNum_Next = z_LocationOut.lane_num;        // 车辆下一段车道编号
	BYTE LastRoadNum_Cur = z_LocationOut.last_roadnum; // 路口上一段道路编号
	BYTE NextRoadNum_Cur = z_LocationOut.next_roadnum; // 路口下一段道路编号
	BYTE LastLaneNum_Cur = z_LocationOut.last_lanenum; // 路口上一段道路的车道编号
	BYTE NextLaneNum_Cur = z_LocationOut.next_lanenum; // 路口下一段道路的车道编号
	int Id_Inter = z_LocationOut.id[LastLaneNum_Cur - 1];   // 路口车辆当前位置ID

	// 路况变量声明
	BOOL Obs_flag = false;                             // 预路口及路口障碍物标志位
	Obs_To_Veh Obs_Front;                              // 预路口及路口障碍物相车辆位置信息
	WORD Obs_Pathid_F;                                 // 障碍物在路径上ID
	ObPoint ObPoint_F;                                 // 障碍物坐标及类型

	// 路口路点总数
	WORD IdSum_Inter = app->decision_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1].size();
	// 下一段路上路点总数
	WORD IdSum_NextLane = app->decision_MapData[RoadNum_Next - 1][LaneNum_Next - 1].size();

	//// 将路口路点压入向量中
	//PathPoint_Front.clear();
	for (WORD j = Id_Inter;j < IdSum_Inter;j++)
	{
		GlobalPoint2D pt;
		pt.x = app->decision_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1][j].global_point.x;
		pt.y = app->decision_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1][j].global_point.y;
		PathPoint_Front.push_back(pt);
	}
	// 将出路口路点压入向量中
	for (WORD i = 0;i < min(60, IdSum_NextLane);i++)
	{
			GlobalPoint2D pt;
			pt.x = app->decision_MapData[RoadNum_Next - 1][LaneNum_Next - 1][i].global_point.x;
			pt.y = app->decision_MapData[RoadNum_Next - 1][LaneNum_Next - 1][i].global_point.y;
			PathPoint_Front.push_back(pt);
	}

	// 搜索预路口及路口障碍物
	Obs_flag = SearchObstacle(PathPoint_Front, z_Obs, -0.5*Vehicle_Width, 0.5*Vehicle_Width, Obs_Front.dis_lat, Obs_Front.dis_lng, ObPoint_F, Obs_Pathid_F);

	// 预路口车速规划
	if (Obs_Front.dis_lng < 13)
	{
		z_velocity_expect = max(Obs_Front.dis_lng - 3, 0);    // 遇障后预路口车速决策
		z_behavior_to_dlg = 13;                               // 不可变道，路径上有障碍物，不可绕障
	}
	else
	{
		z_velocity_expect = 10;                   // 预路口限速10
		z_behavior_to_dlg = 1;                    // 不可变道，路径上无障碍物
	}

	// 转向灯
	if (z_RoadNavi[z_LocationOut.path_num].stub_attribute == 3)
	{
		light_status = 1;                         // 左转灯
	}
	else
	{
		light_status = z_RoadNavi[z_LocationOut.path_num].stub_attribute;
	}

	z_behavior = 1;                                 // 车辆道路行为
	z_target_roadnum = RoadNum_Next;                // 当前目标道路
	z_target_lanenum = LaneNum_Next;                // 当前目标车道
	z_velocity_expect = z_velocity_expect;          // 期望车速
	z_refpath = PathPoint_Front;                    // 车辆前方参考路点
	z_light_status = light_status;                  // 车辆前方交通灯状态
	return;
}

/*************************************************
名称：CalcNaviLaneChgTimes
描述：计算导航要求的变道次数
输入：
roadnum_cur 当前车道编号
outlanenum[LANESUM]当前道路出道路可行车道
lanechgdir 换道类型，向右换道
输出：
times 换道次数
**************************************************/
BYTE CDecision::CalcNaviLaneChgTimes(const BYTE lanenum_cur, const WORD outlanenum[LANESUM], BYTE lanechgdir)
{
	int times = 5;
	int times_tmp = 0;

	if (lanechgdir == 1)
	{
		for (BYTE i = 0; i < LANESUM &&outlanenum[i] != 0; i++)
		{
			times_tmp = lanenum_cur - outlanenum[i];
			if (times_tmp < times)
			{
				times = times_tmp;
			}
			if (times_tmp < 0)
			{
				AfxMessageBox("CalcNaviLaneChgTimes参数1有误！");
			}
		}
	}
	else if (lanechgdir == 2)
	{
		for (BYTE i = 0; i < LANESUM &&outlanenum[i] != 0; i++)
		{
			times_tmp = outlanenum[i] - lanenum_cur;
			if (times_tmp < times)
			{
				times = times_tmp;
			}
			if (times_tmp < 0)
			{
				AfxMessageBox("CalcNaviLaneChgTimes参数2有误！");
			}
		}
	}
	else
	{
		AfxMessageBox("CalcNaviLaneChgTimes参数3有误！");
	}
	return times;
}

/******************************************************************************
名称：LoadRefPath
描述：加载车辆周围相对路径
输入:
     z_LocationOut 车辆定位信息
输出：
     PathPoint_Front 正前方路径
	 PathPoint_Rear  正后方路径
	 PathPoint_LF    左前方路径
	 PathPoint_LR    左后方路径
	 PathPoint_RF    右前方路径
	 PathPoint_RR    右后方路径
*******************************************************************************/
void CDecision::LoadRefPath(const LocationOut z_LocationOut, vector<GlobalPoint2D> &PathPoint_Front, vector<GlobalPoint2D> &PathPoint_Rear, vector<GlobalPoint2D> &PathPoint_LF, vector<GlobalPoint2D> &PathPoint_LR,
	vector<GlobalPoint2D> &PathPoint_RF, vector<GlobalPoint2D> &PathPoint_RR,double &Width_CurLane)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	WORD RoadNum_Cur = z_LocationOut.road_num;            // 车辆当前道路编号		
	WORD LaneNum_Cur = z_LocationOut.lane_num;            // 车辆当前车道编号

	WORD Id_CurLane = z_LocationOut.id[LaneNum_Cur - 1];  // 车辆在当前车道上的路点id编号
	WORD IdSum_CurLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1].size();   	// 车辆所在车道的路点id总数																							
	WORD LaneSum = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][0].lane_sum;     // 车辆所在道路车道总数

	// 地图可变道属性，0不可变道，1可左变道，2可右变道，3可左右变道
	WORD LaneChg = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][Id_CurLane].lanechg_attribute;

	
	// double Width_CurLane = 0; 	     //车辆所在车道宽度
	WORD Id_LeftLane = 0;            //车辆在车道左侧车道上的id编号，如果左侧无车道，则id为0 	
	WORD IdSum_LeftLane = 0;         //车辆在车道左侧车道的路点id总数，如果左侧无车道，则idsum为0 
	double Width_LeftLane = 0;		 //车辆所在车道左侧车道宽度，如果左侧无车道，向量大小为0 
	WORD Id_RightLane = 0;			 //车辆在右侧车道上的id编号，如果右侧无车道，则id为0 		
	WORD IdSum_RightLane = 0;        //车辆在右侧车道的路点id总数，如果右侧无车道，则idsum为0 
	double Width_RightLane = 0; 	 //车辆所在车道右侧车道宽度，如果右侧无车道，向量大小为0 	

	// 车道宽度
	Width_CurLane=(app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][Id_CurLane].lane_width) / 100.0;     

	// 加载正前方60m范围路点坐标
	for (WORD i = min(IdSum_CurLane, Id_CurLane + ID_MORE); i < min(IdSum_CurLane, Id_CurLane + 120 + ID_MORE); i++)   //120个点，差不多60m
	{
		GlobalPoint2D pt;
		pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
		pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
		PathPoint_Front.push_back(pt);                    // 将车前60m范围路点存储至车前路点数组
	}

	// 加载正后方20m范围路点坐标
	for (WORD i = min(IdSum_CurLane, Id_CurLane + ID_MORE);i > max(0, Id_CurLane + ID_MORE - 40);i--)
	{
		GlobalPoint2D pt;
		pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
		pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
		PathPoint_Rear.push_back(pt);                     // 将车后20m范围路点存储至车后路点数组
	}

	// 加载左前方及左后方路点坐标
	if (LaneChg == 1||LaneChg==3)
	{
		// 如果左侧同向有车道时
		if (LaneNum_Cur > 1)   // 要有左侧车道，则车道编号需要大于1
		{
			Id_LeftLane = z_LocationOut.id[LaneNum_Cur - 2];                                   // 车辆在当前车道左侧车道ID
			IdSum_LeftLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2].size();   // 左侧车道路点总数
			if ((Id_LeftLane > 0) && (Id_LeftLane < IdSum_LeftLane))                           // 避免左侧车道是新增车道和断头车道
			{
				Width_LeftLane = (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][Id_LeftLane].lane_width) / 100.0;

				GlobalPoint2D pt;
				for (WORD i = min(IdSum_LeftLane, Id_LeftLane + ID_MORE); i < min(IdSum_LeftLane, Id_LeftLane + 120 + ID_MORE); i++)   // 120个点，差不多60m
				{
					pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][i].global_point.x;
					pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][i].global_point.y;
					PathPoint_LF.push_back(pt);
				}
				for (int i = min(IdSum_LeftLane, Id_LeftLane + ID_MORE); i > max(0, Id_LeftLane + ID_MORE - 40); i--)                  // 后方40个点，差不多60m
				{
					pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][i].global_point.x;
					pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][i].global_point.y;
					PathPoint_LR.push_back(pt);
				}
			}
		}
		// 如果左侧同向没有车道，但是可以左变道，生成参考路点
		else
		{
			// 生成左前参考路点
			PathPoint_LF = CreateNewPath(PathPoint_Front, -1 * Width_CurLane);
			// 生成左后参考路点
			PathPoint_LR = CreateNewPath(PathPoint_Rear, -1 * Width_CurLane);			
		}
	}

	// 加载右前方和右后方路点坐标
	if (LaneChg == 2)
	{
		//右侧同向有车道
		if (LaneNum_Cur < LaneSum)       //要有左侧车道，则车道编号需要小于车道总数
		{
			Id_RightLane = z_LocationOut.id[LaneNum_Cur];                                    // 车辆在右侧车道对应的路点ID
			IdSum_RightLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur].size();    // 右侧车道路点总数

			if ((Id_RightLane > 0) && (Id_RightLane < IdSum_RightLane))                      // 避免右侧新增车道和断头车道
			{
				// 右侧车道宽度
				Width_RightLane = (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur][Id_RightLane].lane_width) / 100.0;
				GlobalPoint2D pt;
				for (WORD i = min(IdSum_RightLane, Id_RightLane + ID_MORE); i < min(IdSum_RightLane, Id_RightLane + 120 + ID_MORE); i++)   //120个点，差不多60m
				{
					pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur][i].global_point.x;
					pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur][i].global_point.y;
					PathPoint_RF.push_back(pt);
				}
				for (int i = min(IdSum_RightLane, Id_RightLane + ID_MORE); i > max(0, Id_RightLane + ID_MORE - 40); i--)   //120个点，差不多60m
				{
					pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur][i].global_point.x;
					pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur][i].global_point.y;
					PathPoint_RR.push_back(pt);
				}
			}
		}
		// 右侧同向没有车道
		else
		{
			// 生成右前参考路点
			PathPoint_RF = CreateNewPath(PathPoint_Front, Width_CurLane);
			// 生成右后参考路点
			PathPoint_RR = CreateNewPath(PathPoint_Rear,  Width_CurLane);
		}

	}
}

/*******************************************************************************
名称：Nav_LaneChange
描述：根据导航判定车辆是否需要换道
输入：
z_LocationOut 定位输出结果
z_RoadNavi    导航输出结果
输出：
Navi_LaneChg  导航换道标志位
Navi_LaneChg_Times 导航换道次数
*******************************************************************************/
void CDecision::Nav_LaneChange(const LocationOut z_LocationOut, const vector<PathInfo> z_RoadNavi, UINT &Navi_LaneChg, UINT &Navi_LaneChg_times)
{
	UINT RoadNum_Cur = z_LocationOut.road_num;            // 车辆当前道路编号		
	UINT LaneNum_Cur = z_LocationOut.lane_num;            // 车辆当前车道编号
	
	UINT PathNum = z_LocationOut.path_num;                // 车辆在导航路径上的编号，从0开始	
	//UINT Navi_LaneChg = 4;                                // 导航是否要求变道，0：不要求，1：要求左换道，2：要求右换道	
	//UINT Navi_LaneChg_times = 0;                          // 导航要求变道的次数
	 

	// 判定当前车道是匹配出路口车道
	for (UINT i = 0; ((i < LANESUM) && (z_RoadNavi[PathNum].out_lane_no[i] != 0)); i++)
	{
		if (LaneNum_Cur == z_RoadNavi[PathNum].out_lane_no[i])
		{
			Navi_LaneChg = 0;
			break;
		}
	}

	// edit 20191114
	WORD out_lane_min = 1;              // 当前可通行的最小车道编号
	WORD out_lane_max = 1;              // 当前可通行的最大车道编号
	out_lane_min = z_RoadNavi[PathNum].out_lane_no[0];
	for (int i = 0;i < LANESUM;i++)
	{
		if (z_RoadNavi[PathNum].out_lane_no[i] > out_lane_max)
		{
			out_lane_max = z_RoadNavi[PathNum].out_lane_no[i];         // 计算最大的可通行车道
		}
	}


	// 判定导航要求向左变道还是向右变道
	if (Navi_LaneChg == 4)   //导航要求换道方向
	{
		// if (LaneNum_Cur < z_RoadNavi[PathNum].out_lane_no[0])        // 前提是导航出路口的车道编号应该是连续
		if (LaneNum_Cur < out_lane_min)
		{
			Navi_LaneChg = 2;      //导航要求右换道    
			Navi_LaneChg_times = CalcNaviLaneChgTimes(LaneNum_Cur, z_RoadNavi[PathNum].out_lane_no, 2);
		}
		//else if (LaneNum_Cur > z_RoadNavi[PathNum].out_lane_no[0])   // 前提是导航出路口的车道编号应该是连续
		else if (LaneNum_Cur > out_lane_max)
		{
			Navi_LaneChg = 1;     //导航要求左换道
			Navi_LaneChg_times = CalcNaviLaneChgTimes(LaneNum_Cur, z_RoadNavi[PathNum].out_lane_no, 1);
		}
		else
		{
			Navi_LaneChg = 0;      //不换道，进入此处说明有问题
		}
	}
}

/********************************************************************
名称：AroundObstacle
描述：获取车辆周围参考路径障碍物信息
输入：
PathPoint_F           // 当前车道前方障碍物
PathPoint_R           // 当前车道后方障碍物
PathPoint_LF          // 当前车道左前方障碍物
PathPoint_LR          // 当前车道左后方障碍物
PathPoint_RF          // 当前车道右前方障碍物
PathPoint_RR          // 当前车道右后方障碍物
Width_CurLane         // 车道宽度
输出：
Path_Obs_F            // 当前车道前方障碍物信息
Path_Obs_R            // 当前车道后方障碍物信息
Path_Obs_LF           // 当前车道左前方障碍物信息
Path_Obs_LR           // 当前车道左后方障碍物信息
Path_Obs_RF           // 当前车道右前方障碍物信息
Path_Obs_RR           // 当前车道右后方障碍物信息
********************************************************************/
void CDecision::AroundObstacle(const vector<GlobalPoint2D>PathPoint_F, const vector<GlobalPoint2D>PathPoint_R, const vector<GlobalPoint2D>PathPoint_LF, const vector<GlobalPoint2D>PathPoint_LR,
	const vector<GlobalPoint2D>PathPoint_RF, const vector<GlobalPoint2D>PathPoint_RR, Path_Obs &Path_Obs_F, Path_Obs &Path_Obs_R, Path_Obs &Path_Obs_LF, Path_Obs &Path_Obs_LR,
	Path_Obs &Path_Obs_RF, Path_Obs &Path_Obs_RR, const double Width_CurLane)
{
	
	bool Obs_flag_F = false;         // 前方障碍物标志位
	Obs_To_Veh Obs_Pose_F;           // 车辆正前方障碍物位姿
	WORD Obs_Pathid_F=0;               // 前方障碍物ID
	ObPoint ObAttr_F;                // 前方障碍物属性

	bool Obs_flag_R = false;         // 后方障碍物标志位
	Obs_To_Veh Obs_Pose_R;           // 车辆正后方障碍物位姿
	WORD Obs_Pathid_R=0;               // 后方障碍物ID
	ObPoint ObAttr_R;                // 后方障碍物属性

	bool Obs_flag_LF = false;         // 左前方障碍物标志位
	Obs_To_Veh Obs_Pose_LF;           // 车辆左前方障碍物位姿
	WORD Obs_Pathid_LF=0;               // 左前方障碍物ID
	ObPoint ObAttr_LF;                // 左前方障碍物属性

	bool Obs_flag_LR = false;         // 左后方障碍物标志位
	Obs_To_Veh Obs_Pose_LR;           // 车辆左后方障碍物位姿
	WORD Obs_Pathid_LR=0;               // 左后方障碍物ID
	ObPoint ObAttr_LR;                // 左后方障碍物属性

	bool Obs_flag_RF = false;         // 右前方障碍物标志位
	Obs_To_Veh Obs_Pose_RF;           // 车辆右前方障碍物位姿
	WORD Obs_Pathid_RF=0;               // 右前方障碍物ID
	ObPoint ObAttr_RF;                // 右前方障碍物属性

	bool Obs_flag_RR = false;         // 右后方障碍物标志位
	Obs_To_Veh Obs_Pose_RR;           // 车辆右后方障碍物位姿
	WORD Obs_Pathid_RR=0;               // 右后方障碍物ID
	ObPoint ObAttr_RR;                // 右后方障碍物属性
	
	memset(&Obs_Pose_F, 0, sizeof(Obs_Pose_F));
	memset(&Obs_Pose_R, 0, sizeof(Obs_Pose_R));
	memset(&Obs_Pose_LF, 0, sizeof(Obs_Pose_LF));
	memset(&Obs_Pose_LR, 0, sizeof(Obs_Pose_LR));
	memset(&Obs_Pose_RF, 0, sizeof(Obs_Pose_RF));
	memset(&Obs_Pose_RR, 0, sizeof(Obs_Pose_RR));	

	memset(&ObAttr_F, 0, sizeof(ObAttr_F));
	memset(&ObAttr_R, 0, sizeof(ObAttr_R));
	memset(&ObAttr_LF, 0, sizeof(ObAttr_LF));
	memset(&ObAttr_LR, 0, sizeof(ObAttr_LF));
	memset(&ObAttr_RF, 0, sizeof(ObAttr_RF));
	memset(&ObAttr_RR, 0, sizeof(ObAttr_RR));

	// 计算正前方障碍物信息
	if (PathPoint_F.size() != 0)
	{
		Obs_flag_F = SearchObstacle(PathPoint_F, z_Obs, -0.5*Vehicle_Width, 0.5*Vehicle_Width, Obs_Pose_F.dis_lat, Obs_Pose_F.dis_lng, ObAttr_F, Obs_Pathid_F);
	}

	// 计算正后方障碍物信息
	if (PathPoint_R.size() != 0)
	{
		Obs_flag_R= SearchObstacle(PathPoint_R, z_Obs, -0.5*Vehicle_Width, 0.5*Vehicle_Width, Obs_Pose_R.dis_lat, Obs_Pose_R.dis_lng, ObAttr_R, Obs_Pathid_R);
	}

	// 计算左前方障碍物信息
	if (PathPoint_LF.size() != 0)
	{
		Obs_flag_LF= SearchObstacle(PathPoint_LF, z_Obs, -0.5*Vehicle_Width, 0.5*Width_CurLane, Obs_Pose_LF.dis_lat, Obs_Pose_LF.dis_lng, ObAttr_LF, Obs_Pathid_LF);
	}


	// 计算左后方障碍物信息
	if (PathPoint_LR.size() != 0)
	{
		Obs_flag_LR = SearchObstacle(PathPoint_LR, z_Obs, -0.5*Vehicle_Width, 0.5*Width_CurLane, Obs_Pose_LR.dis_lat, Obs_Pose_LR.dis_lng, ObAttr_LR, Obs_Pathid_LR);
	}

	// 计算右后方障碍物信息
	if (PathPoint_RF.size() != 0)
	{
		Obs_flag_RF = SearchObstacle(PathPoint_RF, z_Obs, -0.5*Width_CurLane, 0.5*Vehicle_Width, Obs_Pose_RF.dis_lat, Obs_Pose_RF.dis_lng, ObAttr_RF, Obs_Pathid_RF);
	}

	// 计算右后方障碍物信息
	if (PathPoint_RR.size() != 0)
	{
		Obs_flag_RR = SearchObstacle(PathPoint_RR, z_Obs, -0.5*Width_CurLane, 0.5*Vehicle_Width, Obs_Pose_RR.dis_lat, Obs_Pose_RR.dis_lng, ObAttr_RR, Obs_Pathid_RR);
	}


	// 车辆当前车道前方障碍物信息
	Path_Obs_F.Ob_Pose = Obs_Pose_F;
	Path_Obs_F.Obs_flag = Obs_flag_F;
	Path_Obs_F.Ob_Pathid = Obs_Pathid_F;
	Path_Obs_F.Ob_Attr = ObAttr_F;

	// 车辆当前车道后方障碍物信息
	Path_Obs_R.Ob_Pose = Obs_Pose_R;
	Path_Obs_R.Obs_flag = Obs_flag_R;
	Path_Obs_R.Ob_Pathid = Obs_Pathid_R;
	Path_Obs_R.Ob_Attr = ObAttr_R;

	// 车辆左侧车道前方障碍物信息
	Path_Obs_LF.Ob_Pose = Obs_Pose_LF;
	Path_Obs_LF.Obs_flag = Obs_flag_LF;
	Path_Obs_LF.Ob_Pathid = Obs_Pathid_LF;
	Path_Obs_LF.Ob_Attr = ObAttr_LF;

	// 车辆左侧车道后方障碍物信息
	Path_Obs_LR.Ob_Pose = Obs_Pose_LR;
	Path_Obs_LR.Obs_flag = Obs_flag_LR;
	Path_Obs_LR.Ob_Pathid = Obs_Pathid_LR;
	Path_Obs_LR.Ob_Attr = ObAttr_LR;

	// 车辆右侧车道前方障碍物信息
	Path_Obs_RF.Ob_Pose = Obs_Pose_RF;
	Path_Obs_RF.Obs_flag = Obs_flag_RF;
	Path_Obs_RF.Ob_Pathid = Obs_Pathid_RF;
	Path_Obs_RF.Ob_Attr = ObAttr_RF;

	// 车辆右侧车道后方障碍物信息
	Path_Obs_RR.Ob_Pose = Obs_Pose_RR;
	Path_Obs_RR.Obs_flag = Obs_flag_RR;
	Path_Obs_RR.Ob_Pathid = Obs_Pathid_RR;
	Path_Obs_RR.Ob_Attr = ObAttr_RR;
}

/*********************************************************************************************
名称：BehaviorDecision
描述：行为决策与判定
输入：
z_LocationOut   定位结果
z_RoadNavi      导航结果
Path_Obs_F      当前车道正前方障碍物信息
Path_Obs_R      当前车道正后方障碍物信息
Path_Obs_LF     当前车道左前方障碍物信息
Path_Obs_LR     当前车道左后方障碍物信息
Path_Obs_RF     当前车道右前方障碍物信息
Path_Obs_RR     当前车道右后方障碍物信息
输出：
behavior
**********************************************************************************************/
void CDecision::BehaviorDecision(const LocationOut z_LocationOut, const vector<PathInfo> z_RoadNavi, const UINT Navi_LaneChg, const UINT Navi_LaneChg_times,const double Width_CurLane, const vector<GlobalPoint2D>PathPoint_Front, const Path_Obs &Path_Obs_F, const Path_Obs &Path_Obs_R, const Path_Obs &Path_Obs_LF,
	const Path_Obs &Path_Obs_LR, const Path_Obs &Path_Obs_RF, const Path_Obs &Path_Obs_RR, const Behavior_Dec His_Behavior, Behavior_Dec &Cur_Behavior)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();


	WORD RoadNum_Cur = z_LocationOut.road_num;            // 车辆当前道路编号		
	WORD LaneNum_Cur = z_LocationOut.lane_num;            // 车辆当前车道编号														  
	WORD PathNum = z_LocationOut.path_num;                // 车辆在导航路径上的编号，从0开始

	WORD Id_CurLane = z_LocationOut.id[LaneNum_Cur - 1];  // 车辆在当前车道上的路点id编号
	WORD IdSum_CurLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1].size();   	// 车辆所在车道的路点id总数																							
	WORD LaneSum = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][0].lane_sum;     // 车辆所在道路车道总数

	// 地图可变道属性，0不可变道，1可左变道，2可右变道，3可左右变道
	WORD LaneChg_Map = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][Id_CurLane].lanechg_attribute;
	
	static UINT obsavoid_time = 0;                          // 障碍物出现的次数
	static UINT no_obsaviod_time = 0;                       // 绕障过程中障碍物消失次数
	static UINT frontobs_time = 0;                          // 障碍物出现的次数
	
	// 地图属性不可变道
	if (LaneChg_Map == 0)
	{		
		if (Path_Obs_F.Ob_Pose.dis_lng < 15)
		{
			no_obsaviod_time = 0;
			obsavoid_time++;
		
			Obs_To_Veh Path_Obs_F_L;        // 同车道前方左侧		 
			Obs_To_Veh Path_Obs_F_R;        // 同车道前方右侧
			WORD Obs_Pathid_F;              // 同车道前方两侧障碍物ID
			ObPoint ObPoint_F;              // 同车道前方两侧障碍物属性

			//static bool left_flag = false;     // 左绕障标志位
			//static bool right_flag = false;    // 右绕障标志位

			// 前方持续存在障碍物
			if (obsavoid_time > 2)
			{
				vector<GlobalPoint2D> newpath;
				bool left_flag = false;     // 左绕障标志位
				for (BYTE i = 0; i < (Width_CurLane - Vehicle_Width) / 0.6; i++)
				{
					newpath = CreateNewPath(PathPoint_Front, -0.3*i);
					SearchObstacle(newpath, z_Obs, -0.5*Vehicle_Width, 0.5*Vehicle_Width, Path_Obs_F_L.dis_lat, Path_Obs_F_L.dis_lng, ObPoint_F, Obs_Pathid_F);
					if (Path_Obs_F_L.dis_lng > 25)
					{
						Cur_Behavior.behavior = 4;                                  // 左绕障
						Cur_Behavior.target_lanenum= LaneNum_Cur;                   // 目标车道编号
						Cur_Behavior.light_status = 1;                              // 左转灯
						Cur_Behavior.obsavoid_status = 1;                           // 绕障过程中
						Cur_Behavior.behavior_to_dlg = 11;                          // 不可变道，路径上有障碍物，可左绕障，面板输出
						left_flag = true;                                           // 左绕障标志位至true
						break;
					}
				}
				
				bool right_flag = false;    // 右绕障标志位
				if (left_flag == false)
				{
					for (BYTE i = 0; i < (Width_CurLane - Vehicle_Width) / 0.6; i++)
					{
						newpath = CreateNewPath(PathPoint_Front, 0.3*i);
						SearchObstacle(newpath, z_Obs, -0.5*Vehicle_Width, 0.5*Vehicle_Width, Path_Obs_F_R.dis_lat, Path_Obs_F_R.dis_lng, ObPoint_F, Obs_Pathid_F);
						if (Path_Obs_F_R.dis_lng > 25)
						{							
							Cur_Behavior.behavior = 5;                                  // 右绕障
							Cur_Behavior.target_lanenum = LaneNum_Cur;                  // 目标车道编号
							Cur_Behavior.light_status = 2;                              // 左转灯
							Cur_Behavior.obsavoid_status = 1;                           // 绕障过程中
							Cur_Behavior.behavior_to_dlg = 12;                          // 不可变道，路径上有障碍物，可左绕障，面板输出
							right_flag = true;                                          // 左绕障标志位至true
							break;
						}
					}
				}	
			}		
			// 前方不存在障碍物时
			else
			{
				Cur_Behavior.behavior = 1;                                  // 车道保持
				Cur_Behavior.target_lanenum = LaneNum_Cur;                  // 目标车道编号
				Cur_Behavior.light_status = 0;                              // 关闭转向灯				
				Cur_Behavior.behavior_to_dlg = 1;                           // 不可变道，路径上有障碍物，可左绕障，面板输出				
			}
		}
		else
		{
			if (z_segment_obsavoid_status == 0)
			{

				Cur_Behavior.behavior = 1;                                  // 车道保持
				Cur_Behavior.target_lanenum = LaneNum_Cur;                  // 目标车道编号
				Cur_Behavior.light_status = 0;                              // 关闭转向灯				
				Cur_Behavior.behavior_to_dlg = 1;                           // 不可变道，路径上有障碍物，可左绕障，面板输出	
								
			}
			else
			{
				no_obsaviod_time++;
				if (no_obsaviod_time > 3)
				{
					Cur_Behavior.behavior = 1;
					Cur_Behavior.target_lanenum = LaneNum_Cur;                  // 目标车道编号
					Cur_Behavior.light_status = 0;                              // 关闭转向灯				
					Cur_Behavior.behavior_to_dlg = 1;                           // 不可变道，路径上有障碍物，可左绕障，面板输出
					Cur_Behavior.obsavoid_status = 0;
				}
			}
			Cur_Behavior.behavior_to_dlg = 1;                                   //不可变道，路径上无障碍物
		}
	}
	// 地图属性可变道
	else
	{
		no_obsaviod_time = 0;
		obsavoid_time = 0;

		// 非变道过程中
		if (Cur_Behavior.lanechg_status==0)
		{
			// 导航有换道需求
			if (Navi_LaneChg != 0)
			{
				// 导航要求向左侧换道
				if (Navi_LaneChg == 1)
				{
					// 地图允许向左侧换道
					if (LaneChg_Map == 1 || LaneChg_Map == 3)
					{
						Cur_Behavior.behavior_to_dlg = 2;
						if (Cur_Behavior.light_status != 1)
						{
							Cur_Behavior.light_status = 1;
							leftlight_time = 0;
						}
						leftlight_time += z_period_last;

						// 导航要求左侧换道，判定左侧换道条件是否允许
						if ((Path_Obs_LF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10) || (Path_Obs_LF.Ob_Pose.dis_lng > 40))
						{
							// 判定左侧后方来车的相对距离
							if (Path_Obs_LR.Ob_Pose.dis_lng > 15)
							{
								// 转向灯时长满足时
								if (leftlight_time > 2000)
								{
									Cur_Behavior.behavior = 2;                       // 向左换道
									Cur_Behavior.target_lanenum = LaneNum_Cur - 1;   // 目标车道
									Cur_Behavior.lanechg_status = 1;                 // 换道过程中
								}
								// 转向灯时长不满足时
								else
								{
									Cur_Behavior.behavior = 1;                     // 保持当前车道
									Cur_Behavior.target_lanenum = LaneNum_Cur;     // 目标车道
									Cur_Behavior.lanechg_status = 0;               // 非换道过程中
								}

							}
							// 左侧车道后方调价不允许
							else
							{
								Cur_Behavior.behavior = 1;                     // 保持当前车道
								Cur_Behavior.target_lanenum = LaneNum_Cur;     // 目标车道
								Cur_Behavior.lanechg_status = 0;               // 非换道过程中
							}
						}
						// 左侧车道不允许换道
						else
						{
							Cur_Behavior.behavior = 1;                     // 保持当前车道
							Cur_Behavior.target_lanenum = LaneNum_Cur;     // 目标车道
							Cur_Behavior.lanechg_status = 0;               // 非换道过程中
						}

					}
					// 地图不允许向左换道
					else
					{
						Cur_Behavior.behavior = 1;                     // 保持当前车道
						Cur_Behavior.target_lanenum = LaneNum_Cur;     // 目标车道
						Cur_Behavior.lanechg_status = 0;               // 非换道过程中
						Cur_Behavior.behavior_to_dlg = 4;              // 导航要求变道，但变道属性不匹配
					}
				}
				// 导航要求向右侧换道
				else if (Navi_LaneChg == 2)
				{
					if (LaneChg_Map == 2 || LaneChg_Map == 3)
					{
						Cur_Behavior.behavior_to_dlg = 3;    // 导航要求右变道，可右变道	
						// 右转灯计时
						if (Cur_Behavior.light_status != 2)
						{
							Cur_Behavior.lanechg_status = 2;
							rightlight_time = 0;
						}
						rightlight_time += z_period_last;

						// 向右侧换道
						if ((Path_Obs_RF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10) || (Path_Obs_RF.Ob_Pose.dis_lng > 40))
						{
							// 判断后方来车的相对距离
							if (Path_Obs_RR.Ob_Pose.dis_lng > 15)    //满足变道条件,左前方无障碍物，后方障碍物距离大于15
							{
								// 转向灯时长满足
								if (rightlight_time >= 2000)
								{
									Cur_Behavior.behavior = 3;                     // 右变道
									Cur_Behavior.target_lanenum = LaneNum_Cur + 1;   // 目标车道						
									Cur_Behavior.lanechg_status = 1;               // 变道状态标志位							
								}
								// 转向灯时长不满足
								else
								{
									Cur_Behavior.behavior = 1;                       // 车道保持
									Cur_Behavior.target_lanenum = LaneNum_Cur;       // 目标车道
									Cur_Behavior.lanechg_status = 0;                 // 非换道过程中

								}
							}
							else 
							{
								Cur_Behavior.behavior = 1;                        // 车道保持
								Cur_Behavior.target_lanenum = LaneNum_Cur;        // 目标车道
								Cur_Behavior.lanechg_status = 0;                  // 非换道过程中
							}
						}
						else
						{
							Cur_Behavior.behavior = 1;                       // 车道保持
							Cur_Behavior.target_lanenum = LaneNum_Cur;       // 目标车道
							Cur_Behavior.lanechg_status = 0;                 // 非换道过程中
						}
					}
					else
					{
						Cur_Behavior.behavior = 1;                       // 车道保持
						Cur_Behavior.target_lanenum = LaneNum_Cur;       // 目标车道
						Cur_Behavior.lanechg_status = 0;                 // 非换道过程中
						Cur_Behavior.behavior_to_dlg = 4;             // 导航要求变道，但变道属性不匹配
					}
				}
												
			}
			// 导航没有换道需求
			else
			{
				// 前方长时间存在障碍物
				if (Path_Obs_F.Ob_Pose.dis_lng < (2 * 10 + 5))
				{
					frontobs_time++;
					// 前方长时间存在障碍物
					if (frontobs_time > 2)
					{
						frontobs_time = 3;
						// 地图允许左变道
						if (LaneChg_Map == 1)
						{
							// 左侧有同向车道
							if(LaneNum_Cur>1)
							{
								Cur_Behavior.behavior_to_dlg = 5;  // 导航不要求变道，前方有障碍物，可左变道						

								// 判定是否需要返回
								bool no_back_flag = true;    // 换道后是否需要返回，true：需要变回来，false：不需要变回来
								for (BYTE i = 0; i < LANESUM && z_RoadNavi[PathNum].out_lane_no[i] != 0; i++)
								{
									if (z_RoadNavi[PathNum].out_lane_no[i] == LaneNum_Cur - 1)
									{
										no_back_flag = false;
									}
								}

								// 判定地图是否允许换道
								bool chg_condition_flag = false;          // false 表示变完道之后不能回来，true 表示有足够空间返回							
								if (no_back_flag == true)
								{
									double dis_chg1 = 0;
									for (WORD i = Id_CurLane; (i < IdSum_CurLane - 1) && (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].lanechg_attribute == 1); i++)
									{
										GlobalPoint2D pt, pt1;
										pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
										pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
										pt1.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.x;
										pt1.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.y;
										dis_chg1 += CalcDistance(pt, pt1);
									}

									// 当返回空间满足的情况下才会换道
									if (dis_chg1 > 60)     //后续，dis_chg2可以延伸到是否有下一段可变道区域，而不是到预路口了
									{
										chg_condition_flag = true;    // 距离充足表明能返回
										if (z_light_status != 1)
										{
											z_light_status = 1;
											leftlight_time = 0;
										}
										leftlight_time += z_period_last;
										if (leftlight_time > 2000)         //开左转灯2s
										{
											leftlight_time = 2000;
										}
									}
									else
									{
										z_light_status = 0;          // 关闭转向灯      
									}
								}
								else
								{
									double dis_chg1 = 0;
									for (WORD i = Id_CurLane; (i < IdSum_CurLane - 1) && (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].lanechg_attribute & 0x01 == 0x01); i++)
									{
										GlobalPoint2D pt, pt1;
										pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
										pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
										pt1.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.x;
										pt1.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.y;
										dis_chg1 += CalcDistance(pt, pt1);
									}
									if (dis_chg1 > 15)
									{
										chg_condition_flag = true;
										if (Cur_Behavior.light_status!=1)
										{
											Cur_Behavior.light_status = 1;
											leftlight_time = 0;
										}
										leftlight_time += z_period_last;
										if (leftlight_time > 2000)                          //开左转灯2s
										{
											leftlight_time = 2100;
										}
									}
									else
									{
										Cur_Behavior.light_status = 0; 
									}
								}

								// 判定目标车道是否允许换道
								if (chg_condition_flag == true)
								{
									// 左前方空间是否允许允许换道
									if (Path_Obs_LF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10)
									{
										if (Path_Obs_LR.Ob_Pose.dis_lng > 10)
										{
											// 状态灯时长满足换道
											if (leftlight_time > 1500)
											{
												frontobs_time = 0;                              // 障碍物出现次数至零
												Cur_Behavior.behavior = 2;                      // 左变道
												Cur_Behavior.target_lanenum = LaneNum_Cur - 1;  // 目标车道
												Cur_Behavior.lanechg_status = 1;                // 确定变道标志位
											}
											// 状态灯时长不满足换道
											else
											{
												Cur_Behavior.behavior = 1;                      // 车道保持
												Cur_Behavior.target_lanenum = LaneNum_Cur;      // 目标车道
												Cur_Behavior.lanechg_status = 0;                // 不发生变道，标志位
											}
										}
										else
										{
											Cur_Behavior.behavior = 1;                          // 车道保持
											Cur_Behavior.target_lanenum = LaneNum_Cur;          // 目标车道
											Cur_Behavior.lanechg_status = 0;                    // 不发生变道，标志位
										}
									}
									else
									{
										Cur_Behavior.behavior = 1;                          // 车道保持
										Cur_Behavior.target_lanenum = LaneNum_Cur;          // 目标车道
										Cur_Behavior.lanechg_status = 0;                    // 不发生变道，标志位
									}
								}
								else
								{
									Cur_Behavior.behavior = 1;                      // 车道保持
									Cur_Behavior.target_lanenum = LaneNum_Cur;      // 目标车道
									Cur_Behavior.lanechg_status = 0;                // 不发生变道，标志位
								}
							}
							// 左侧没有同向车道
							else
							{
								Cur_Behavior.behavior = 1;                      // 车道保持
								Cur_Behavior.target_lanenum = LaneNum_Cur;      // 目标车道
								Cur_Behavior.lanechg_status = 0;                // 不发生变道，标志位								
							}												

						}
						// 地图允许右变道且右侧存在同向车道
						else if (LaneChg_Map == 2)
						{
							// 右侧有同向车道
							if (LaneNum_Cur < LaneSum)
							{
								Cur_Behavior.behavior_to_dlg = 6;    //导航不要求变道，前方有障碍物，可右变道

								// 判定是否需要换回来
								bool no_back_flag = true;   //换道后是否需要返回，true：需要变回来，false：不需要变回来
								for (BYTE i = 0; i < LANESUM && z_RoadNavi[PathNum].out_lane_no[i] != 0; i++)
								{
									if (z_RoadNavi[PathNum].out_lane_no[i] == LaneNum_Cur - 1)
										no_back_flag = false;
								}

								// 判定地图剩余长度是否允许换道
								bool chg_condition_flag = false;  //如果向左变完道后还要变回来，则看还有没有足够的距离让变回来，没有的话不能变道
								if (no_back_flag == true)
								{
									double dis_chg1 = 0;
									for (WORD i = Id_CurLane; (i < IdSum_CurLane - 1) && (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].lanechg_attribute & 0x02 == 0x02); i++)
									{
										GlobalPoint2D pt, pt1;
										pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
										pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
										pt1.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.x;
										pt1.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.y;
										dis_chg1 += CalcDistance(pt, pt1);
									}
									// 剩余长度是否允许换道
									if (dis_chg1 > 50)
									{
										chg_condition_flag = true;
										if (Cur_Behavior.light_status != 2)
										{
											Cur_Behavior.light_status = 2;
											leftlight_time = 0;
										}
										leftlight_time += z_period_last;
										if (leftlight_time > 2000)                          //开左转灯2s
										{
											leftlight_time = 2000;
										}
									}
								}
								else
								{
									double dis_chg1 = 0;
									for (WORD i = Id_CurLane; (i < IdSum_CurLane - 1) && (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].lanechg_attribute & 0x02 == 0x02); i++)
									{
										GlobalPoint2D pt, pt1;
										pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
										pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
										pt1.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.x;
										pt1.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.y;
										dis_chg1 += CalcDistance(pt, pt1);
									}
									if (dis_chg1 > 10)
									{
										chg_condition_flag = true;
										if (Cur_Behavior.light_status != 1)
										{
											Cur_Behavior.light_status = 1;
											leftlight_time = 0;
										}
										leftlight_time += z_period_last;
										if (leftlight_time > 2000)                          //开左转灯2s
										{
											leftlight_time = 2000;
										}
									}
								}

								// 判定目标车道空间是否允许换道
								if (chg_condition_flag == true)
								{
									// 目标车道前方障碍物满足换道
									if (Path_Obs_RF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10)
									{
										// 目标车道后方距离满足换道
										if (Path_Obs_RR.Ob_Pose.dis_lng > 10)    //满足变道条件,右前方无障碍物，后方障碍物距离大于15
										{
											// 转向灯时长满足
											if (leftlight_time > 1500)
											{
												frontobs_time = 0;
												Cur_Behavior.behavior = 3;                       // 右变道
												Cur_Behavior.target_lanenum = LaneNum_Cur + 1;   // 目标车道
												Cur_Behavior.lanechg_status = 1;                 // 换道标志位										
											}
											// 转向灯时长不满足
											else
											{
												Cur_Behavior.behavior = 1;                       // 车道保持
												Cur_Behavior.target_lanenum = LaneNum_Cur;      // 目标车道
												Cur_Behavior.lanechg_status = 0;                 // 换道标志位	
											}
										}
										// 目标车道后方距离不满足变道
										else
										{
											Cur_Behavior.behavior = 1;                       // 车道保持
											Cur_Behavior.target_lanenum = LaneNum_Cur;       // 目标车道
											Cur_Behavior.lanechg_status = 0;                 // 换道标志位	
										}
									}
									// 目标车道前方障碍物不满足换道
									else
									{
										Cur_Behavior.behavior = 1;                       // 车道保持
										Cur_Behavior.target_lanenum = LaneNum_Cur;       // 目标车道
										Cur_Behavior.lanechg_status = 0;                 // 换道标志位	
									}
								}
								else
								{
									Cur_Behavior.behavior = 1;                       // 车道保持
									Cur_Behavior.target_lanenum = LaneNum_Cur;       // 目标车道
									Cur_Behavior.lanechg_status = 0;                 // 换道标志位
								}
							}
							else
							{
								Cur_Behavior.behavior = 1;                       // 车道保持
								Cur_Behavior.target_lanenum = LaneNum_Cur;       // 目标车道
								Cur_Behavior.lanechg_status = 0;                 // 换道标志位
							}
							
						}
						// 地图允许左右变道						
						else if (LaneChg_Map == 3)
						{
							z_behavior_to_dlg = 7;          //导航不要求变道，前方有障碍物，可左右变道

							// 判定向左侧换道是否需要返回原车道
							bool no_back_leftchg_flag = true;   //左换道后是否需要返回，true：需要变回来，false：不需要变回来
							for (BYTE i = 0; i < LANESUM && z_RoadNavi[PathNum].out_lane_no[i] != 0; i++)
							{
								if (z_RoadNavi[PathNum].out_lane_no[i] == LaneNum_Cur - 1)
								{
									no_back_leftchg_flag = false;
								}

							}

							// 判定向右侧换道是否需要返回原车道
							bool no_back_rightchg_flag = true;   //右换道后是否需要返回，true：需要变回来，false：不需要变回来
							for (BYTE i = 0; i < LANESUM && z_RoadNavi[PathNum].out_lane_no[i] != 0; i++)
							{
								if (z_RoadNavi[PathNum].out_lane_no[i] == LaneNum_Cur + 1)
								{
									no_back_rightchg_flag = false;
								}

							}

							// 判定向左侧换道剩余长度是否满足换道条件
							bool leftchg_condition_flag = false;  //如果向左变完道后还要变回来，则看还有没有足够的距离让变回来，没有的话不能变道
							if (LaneNum_Cur > 1)
							{
								if (no_back_leftchg_flag == true)
								{
									double dis_chg1 = 0;
									for (WORD i = Id_CurLane; (i < IdSum_CurLane - 1) && (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].lanechg_attribute & 0x01 == 0x01); i++)
									{
										GlobalPoint2D pt, pt1;
										pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
										pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
										pt1.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.x;
										pt1.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.y;
										dis_chg1 += CalcDistance(pt, pt1);
									}

									if (dis_chg1 > 50)     //后续，dis_chg2可以延伸到是否有下一段可变道区域，而不是到预路口了
									{
										leftchg_condition_flag = true;
									}
								}
								else
								{
									double dis_chg1 = 0;
									for (WORD i = Id_CurLane; (i < IdSum_CurLane - 1) && (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].lanechg_attribute & 0x01 != 0x01); i++)
									{
										GlobalPoint2D pt, pt1;
										pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
										pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
										pt1.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.x;
										pt1.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.y;
										dis_chg1 += CalcDistance(pt, pt1);
									}
									if (dis_chg1 > 10)
									{
										leftchg_condition_flag = true;
									}
								}
							}
							else
							{
								leftchg_condition_flag = false;
							}
							
							

							// 判定向右侧换道剩余长度是否满足换道条件
							bool rightchg_condition_flag = false;  //如果向右变完道后还要变回来，则看还有没有足够的距离让变回来，没有的话不能变道
							if (LaneNum_Cur < LaneSum)
							{
								if (no_back_rightchg_flag == true)
								{
									double dis_chg1 = 0;
									for (WORD i = Id_CurLane; (i < IdSum_CurLane - 1) && (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].lanechg_attribute & 0x01 == 0x01); i++)
									{
										GlobalPoint2D pt, pt1;
										pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
										pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
										pt1.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.x;
										pt1.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.y;
										dis_chg1 += CalcDistance(pt, pt1);
									}
									if (dis_chg1 > 50)     // 剩余距离
									{
										rightchg_condition_flag = true;
									}
								}
								else
								{
									double dis_chg1 = 0;
									for (WORD i = Id_CurLane; (i < IdSum_CurLane - 1) && (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].lanechg_attribute & 0x02 == 0x02); i++)
									{
										GlobalPoint2D pt, pt1;
										pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
										pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
										pt1.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.x;
										pt1.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i + 1].global_point.y;
										dis_chg1 += CalcDistance(pt, pt1);
									}
									if (dis_chg1 > 10)
									{
										rightchg_condition_flag = true;
									}
								}
							}
							else
							{
								rightchg_condition_flag = false;
							}
							

							// 不需要返回原车道且地图剩余距离满足换道
							if (leftchg_condition_flag == true && no_back_leftchg_flag == false)  //如果左变道不用回来且可以左变道，则左变道
							{
								if (Cur_Behavior.lanechg_status != 1)
								{
									Cur_Behavior.light_status = 1;
									leftlight_time = 0;
								}
								leftlight_time += z_period_last;
								if (leftlight_time > 2000)                          //开左转灯2s
								{
									leftlight_time = 2000;
								}

								if (Path_Obs_LF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10)
								{
									// 目标车道后方距离满足换道
									if (Path_Obs_LR.Ob_Pose.dis_lng > 10)    //满足变道条件,右前方无障碍物，后方障碍物距离大于10
									{
										// 转向灯时长满足换道条件
										if (leftlight_time > 2000)
										{
											frontobs_time = 0;
											Cur_Behavior.behavior = 2;                        // 左变道
											Cur_Behavior.target_lanenum = LaneNum_Cur - 1;    // 目标车道
											Cur_Behavior.lanechg_status = 1;                  // 换道状态

										}
										// 转向灯时长不满足换道条件
										else
										{
											Cur_Behavior.behavior = 1;                         // 车道保持
											Cur_Behavior.target_lanenum = LaneNum_Cur;         // 目标车道 
											Cur_Behavior.lanechg_status = 0;
										}
									}
									// 目标车道后方距离不满足换道
									else
									{
										Cur_Behavior.behavior = 1;                          // 车道保持
										Cur_Behavior.target_lanenum = LaneNum_Cur;          // 目标车道	
										Cur_Behavior.lanechg_status = 0;
									}
								}
								else
								{
									Cur_Behavior.behavior = 1;                              // 车道保持
									Cur_Behavior.target_lanenum = LaneNum_Cur;              // 目标车道		
									Cur_Behavior.lanechg_status = 0;
								}
							}
							// 不需要返回原车道且地图剩余距离满足换道
							else if (rightchg_condition_flag == true && no_back_rightchg_flag == false)  //如果右变道不用回来且左变道不满足，则右变道
							{
								if (Cur_Behavior.light_status != 2)
								{
									Cur_Behavior.light_status = 2;
									leftlight_time = 0;
								}
								leftlight_time += z_period_last;
								if (leftlight_time > 2000)                          //开左转灯2s
								{
									leftlight_time = 2000;
								}
								// 目标车道前方满足换道条件
								if (Path_Obs_RF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10)
								{
									// 目标车道后方满足换道条件
									if (Path_Obs_RR.Ob_Pose.dis_lng > 10)    //满足变道条件,右前方无障碍物，后方障碍物距离大于15
									{
										// 转向灯时长满足换道条件
										if (leftlight_time > 2000)
										{
											frontobs_time = 0;
											Cur_Behavior.behavior = 3;                      // 右变道
											Cur_Behavior.target_lanenum = LaneNum_Cur + 1;  // 目标车道
											Cur_Behavior.lanechg_status = 1;                // 换道过程中											
										}
										// 转向灯时长不满足换道条件
										else
										{
											Cur_Behavior.behavior = 1;
											Cur_Behavior.target_lanenum = LaneNum_Cur;
										}
									}
									// 目标车道后方不满足换道条件
									else
									{
										Cur_Behavior.behavior = 1;
										Cur_Behavior.target_lanenum = LaneNum_Cur;
									}
								}
							}
							// 地图剩余距离满足换道条件
							else if (leftchg_condition_flag == true)  //如果变道后要回来，优先左变道
							{
								if (Cur_Behavior.light_status != 1)
								{
									Cur_Behavior.lanechg_status = 1;
									leftlight_time = 0;
								}
								leftlight_time += z_period_last;
								if (leftlight_time > 2000)                          //开左转灯2s
								{
									leftlight_time = 2000;
								}


								// 目标车道前方空间满足换道条件
								if (Path_Obs_LF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10)
								{
									// 目标车道后方距离满足换道条件
									if (Path_Obs_LR.Ob_Pose.dis_lng > 10)    //满足变道条件,右前方无障碍物，后方障碍物距离大于15
									{
										// 转向灯时长满足换道条件
										if (leftlight_time > 2000)
										{
											frontobs_time = 0;
											Cur_Behavior.behavior = 2;                      // 左变道
											Cur_Behavior.target_lanenum = LaneNum_Cur - 1;  // 目标车道
											Cur_Behavior.lanechg_status = 1;                // 左变道中											
										}
										// 转向灯时长不满足换道条件
										else
										{
											Cur_Behavior.behavior = 1;                     // 车道保持中
											Cur_Behavior.target_lanenum = LaneNum_Cur;     // 目标车道
										}
									}
									// 目标车道后方不满足换道条件
									else
									{
										Cur_Behavior.behavior = 1;
										Cur_Behavior.target_lanenum = LaneNum_Cur;
									}
								}
								// 目标车道前方空间不满足换道条件
								else
								{
									Cur_Behavior.behavior = 1;
									Cur_Behavior.target_lanenum = LaneNum_Cur;
								}
							}
							// 地图剩余距离满足换道条件
							else if (rightchg_condition_flag == true)  //左变道不行，就看是否可以右变道
							{
								if (Cur_Behavior.light_status != 2)
								{
									Cur_Behavior.light_status = 2;
									leftlight_time = 0;
								}
								leftlight_time += z_period_last;
								if (leftlight_time > 2000)                          //开左转灯2s
								{
									leftlight_time = 2000;
								}

								// 
								if (Path_Obs_RF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10)
								{
									// 目标车道后方空间满足换道条件
									if (Path_Obs_RR.Ob_Pose.dis_lng > 10)    //满足变道条件,右前方无障碍物，后方障碍物距离大于10
									{
										// 转向灯时长满足换道条件
										if (leftlight_time > 2000)
										{
											frontobs_time = 0;
											Cur_Behavior.behavior = 3;                      // 右变道
											Cur_Behavior.target_lanenum = LaneNum_Cur = 1;  // 目标车道
											Cur_Behavior.lanechg_status = 1;                // 变道中

										}
										// 转向灯时长不满足换道条件
										else
										{
											Cur_Behavior.behavior = 1;
											Cur_Behavior.target_lanenum = LaneNum_Cur;
										}
									}
									// 目标车道后方空间不满足换道条件
									else
									{
										Cur_Behavior.behavior = 1;
										Cur_Behavior.target_lanenum = LaneNum_Cur;
									}
								}
							}
							else              //车道保持
							{
								Cur_Behavior.behavior = 1;
								Cur_Behavior.target_lanenum = LaneNum_Cur;
								Cur_Behavior.lanechg_status = 0;
							}

						}
					}
					// 前方障碍物消失
					else
					{
						Cur_Behavior.behavior = 1;                   // 车道保持
						Cur_Behavior.target_lanenum = LaneNum_Cur;   // 目标车道
						Cur_Behavior.lanechg_status = 0;						
					}
				}
				// 前方无障碍物
				else
				{
					frontobs_time = 0;
					Cur_Behavior.behavior_to_dlg = 8;            // 导航不要求换道，前方无障碍物
					Cur_Behavior.behavior = 1;                   // 车道保持
					Cur_Behavior.target_lanenum = LaneNum_Cur;   // 目标车道
					Cur_Behavior.lanechg_status = 0;             // 非换道状态
				}
			}			
		}
		// 变道过程中
		else if(Cur_Behavior.lanechg_status==1)
		{
			Cur_Behavior.behavior_to_dlg = 9;       // 变道中
			if (Cur_Behavior.target_lanenum==LaneNum_Cur)
			{
				Cur_Behavior.lanechg_status = 0;   
				Cur_Behavior.light_status = 0;				
			}
			Cur_Behavior.behavior = His_Behavior.behavior;
			Cur_Behavior.target_lanenum = His_Behavior.target_lanenum;
			Cur_Behavior.light_status = His_Behavior.light_status;
		}
	}
}

/********************************************************************
名称：SpeedDecision
描述：车速决策
输入：
输出：
*********************************************************************/
void CDecision::SpeedDecision(const Behavior_Dec Cur_Behavior, vector<GlobalPoint2D>&z_refpath, double &z_velocity_expect)
{
	// 避障过程中减速
	if (Cur_Behavior.behavior == 4 || Cur_Behavior.behavior == 5)
	{
		z_velocity_expect = 5;
	}
	else
	{
		z_velocity_expect = 10;
	}
	
}

/********************************************************************
名称：RefPath
描述：输出参考路径
输入：
输出：
*********************************************************************/
void CDecision::RefPath(const Behavior_Dec Cur_Behavior, const vector<GlobalPoint2D>PathPoint_Front, const vector<GlobalPoint2D>PathPoint_LF, const vector<GlobalPoint2D>PathPoint_RF, vector<GlobalPoint2D>&z_refpath)
{
	if (Cur_Behavior.behavior == 2)
	{
		z_refpath = PathPoint_LF;
	}
	else if (Cur_Behavior.behavior == 3)
	{
		z_refpath = PathPoint_RF;
	}
	else
	{
		z_refpath = PathPoint_Front;
	}

}

/****************************************************************************
名称：V2XPedestrianJudge
描述：当V2X路测设备检测到弱势行人时，向车辆发送报警信息，提醒车辆进行减速让行。
输入：行人位置信息，车辆位置信息和参考路径。
输出：行人标志位。
*****************************************************************************/
void CDecision::V2XPedestrianJudge(const LocationOut z_LocationOut, const V2X_Data z_V2XData, vector<GlobalPoint2D> &PathPoint_LongFront, bool &pedestrian_flag)
{

	// 加载前方100m路点
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	WORD RoadNum_Cur = z_LocationOut.road_num;            // 车辆当前道路编号		
	WORD LaneNum_Cur = z_LocationOut.lane_num;            // 车辆当前车道编号

	WORD Id_CurLane = z_LocationOut.id[LaneNum_Cur - 1];  // 车辆在当前车道上的路点id编号
	WORD IdSum_CurLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1].size();   	// 车辆所在车道的路点id总数																							

	// 加载正前方100m范围路点坐标
	for (WORD i = min(IdSum_CurLane, Id_CurLane); i < min(IdSum_CurLane, Id_CurLane + 200); i++)   //200个点，差不多100m
	{
		GlobalPoint2D p;
		p.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
		p.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
		PathPoint_LongFront.push_back(p);                    // 将车前100m范围路点存储至车前路点数组
	}

	// 局部变量初始化
	double distance_pedestrian = 9999; // 行人与车的相对距离
	double lat_pedestrian = 0; // 行人纬度
	double lng_pedestrian = 0; // 行人经度
	int direction_pedestrian = 0; // 行人方向
	double WIDTH_LANE = 3.75; // 车道宽

	double lng_distance = 9999; // 行人距离车的纵向距离
	double lat_distance = 9999; // 行人距离车的横向距离
	double lat_distance_last = 0; // 上一帧行人距离参考路径的横向距离
	int lat_distance_time = 0; // 行人横向距离减小的次数
	double width = WIDTH_LANE + WIDTH_LANE / 2; //一个半车道的宽度

	// 读取V2X行人信息
	distance_pedestrian = z_V2XData.PedesDistance;
	lat_pedestrian = z_V2XData.PedesLatitude;
	lng_pedestrian = z_V2XData.PedesLongitude;
	direction_pedestrian = z_V2XData.PedesDirection;

	// 把行人经纬度位置转换为全局坐标点位置
	GPSPoint2D pedestrian_gps;
	pedestrian_gps.lat = lat_pedestrian;
	pedestrian_gps.lng = lng_pedestrian;

	//对行人位置进行纠偏
	/*GPSPoint3D pedestrian_gps_3D;
	pedestrian_gps_3D.lat = lat_pedestrian;
	pedestrian_gps_3D.lng = lng_pedestrian;
	pedestrian_gps_3D.ang = 0;
	pedestrian_gps_3D = V2XOBUOffSetCorrect(z_LocationOut, z_V2XData, pedestrian_gps_3D);*/

	/*GPSPoint2D pedestrian_gps_2D;
	pedestrian_gps_2D.lat = pedestrian_gps_3D.lat;
	pedestrian_gps_2D.lng = pedestrian_gps_3D.lng;*/

	GlobalPoint2D pedestrian_global;
	pedestrian_global = WGS84ToGlobal(pedestrian_gps);

	if (distance_pedestrian >= 0 && distance_pedestrian <= 100)
	{
		// 行人距离车辆前方路径的横向距离
		GlobalPoint2D pt;
		GlobalPoint2D pt_next;
		int index = 0;
		index = NearestId(pedestrian_global, PathPoint_LongFront);
		pt.x = PathPoint_LongFront[index].x;
		pt.y = PathPoint_LongFront[index].y;
		pt_next.x = PathPoint_LongFront[index + 1].x;
		pt_next.y = PathPoint_LongFront[index + 1].y;

		lat_distance = LatDis(pedestrian_global, pt, pt_next);

		// 行人距离车辆的纵向距离
		if (index >= 2)
		{
			lng_distance = 0;
			for (int i = 0; i < index; i++)
			{
				lng_distance += sqrt(pow((PathPoint_LongFront[i].x - PathPoint_LongFront[i + 1].x), 2) + pow((PathPoint_LongFront[i].y - PathPoint_LongFront[i + 1].y), 2));
			}
		}
		else if (index == 1)
		{
			lng_distance = sqrt(pow((PathPoint_LongFront[index - 1].x - PathPoint_LongFront[index].x), 2) + pow((PathPoint_LongFront[index - 1].y - PathPoint_LongFront[index].y), 2));
		}
		else
		{
			if (distance_pedestrian > 5)
			{
				lng_distance = -9999;
			}
			else
			{
				lng_distance = 0;
			}
		}

		// 行人距离车辆路径横向距离变化方向，若逐渐变小，说明行人轨迹有可能与参考路径交叉，若逐渐变大，说明行人可能远离车辆前进
		if (lat_distance < lat_distance_last)
		{
			lat_distance_time++;
			lat_distance_last = lat_distance;
		}
		else
		{
			lat_distance_last = lat_distance;
			lat_distance_time = 0;
		}

		// 判断纵向距离和横向距离是否小于一定距离，是则减速，否则正常行驶
		if (lng_distance >= 0 && lng_distance <= 100)
		{
			if (lat_distance >= width)
			{
				pedestrian_flag = false;
			}
			else if (lat_distance > 1.5 && lat_distance < width)
			{
				if (lat_distance_time < 5)
					pedestrian_flag = false;
				else
					pedestrian_flag = true;
			}
			else if (lat_distance <= 1.5 || direction_pedestrian == 1)
			{
				pedestrian_flag = true;
			}
		}
		else
		{
			pedestrian_flag = false;
		}
	}
	else
	{
		pedestrian_flag = false;
	}
}

/****************************************************************************
名称：V2XSignalLight
描述：信号灯下发：参赛车辆到达路口前提前接收到路口RSU设备发送的信号灯
状态信息，遇到红灯时减速等待，绿灯时正常减速通过。
因为单车智能也有类似功能，故该函数仅提供V2X的红绿灯信息供Planning参考使用
输入：信号灯信息
输出：红绿灯状态信息
*****************************************************************************/
void CDecision::V2XSignalLight(const V2X_Data z_V2XData, WORD &V2XLight_flag)
{
	// 局部变量初始化
	int Lane_occupied = 999; //当前车道是否为本车所在车道
	int SignalLight_state = 999; // V2X信号灯状态（红、绿、黄）

	//读取信号灯下发信息
	Lane_occupied = z_V2XData.SPATLaneOccupied;
	SignalLight_state = z_V2XData.SPATState;

	if (Lane_occupied == 1) //当前车道是本车所在车道
	{
		if (SignalLight_state == 3 || SignalLight_state == 7) //3：红灯，7：黄灯
		{
			V2XLight_flag = 1;
		}
		else if (SignalLight_state == 6)//6：绿灯
		{
			V2XLight_flag = 2;
		}
		else
		{
			V2XLight_flag = 0;
		}
	}
	/*否则转到单车智能
	else
	{
	}
	*/

}//考虑情况：V2X信息有误，对于当前车道是否是本车所在车道的判断有误

/****************************************************************************
 名称：V2XConstructionEvent
 描述：施工车道预警：路侧设备广播施工信息，车辆提前换道或重新规划路线
 输入：施工事件信息
 输出：是否施工标志
 *****************************************************************************/
void CDecision::V2XConstructionEvent(const LocationOut z_LocationOut, const V2X_Data z_V2XData, const vector<WarningPoint> z_RSIWarningPointList, vector<GlobalPoint2D> &PathPoint_LongFront, bool &construction_flag)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();
	// 局部变量初始化
	double WIDTH_LANE = 3.75; // 车道宽 
	int constructionpoint_num = 0;
	int index = 0;//距离施工点最近的车辆路点id
	double min_distance_construction = 9999; // 车与最近的施工路点的距离
	float near_lat_constructionpoint = 0; // 距离车辆最近施工点纬度
	float near_lng_constructionpoint = 0; // 距离车辆最近施工点经度
	WORD RoadNum_Cur = z_LocationOut.road_num;            // 车辆当前道路编号		
	WORD LaneNum_Cur = z_LocationOut.lane_num;            // 车辆当前车道编号
	WORD Id_CurLane = z_LocationOut.id[LaneNum_Cur - 1];  // 车辆在当前车道上的路点id编号
	WORD IdSum_CurLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1].size();   	// 车辆所在车道的路点id总数	
	
	// 加载车辆正前方100m范围路点坐标
	for (WORD i = min(IdSum_CurLane, Id_CurLane + ID_MORE); i < min(IdSum_CurLane, Id_CurLane + 200 + ID_MORE); i++)   //200个点，差不多100m
	{
		GlobalPoint2D p;
		p.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
		p.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
		PathPoint_LongFront.push_back(p);                    // 将车前100m范围路点存储至车前路点数组
	}
	vector<WarningPoint> ConstructionPointList;
	//读取V2X施工车道预警信息
	ConstructionPointList = z_RSIWarningPointList;
	
	if (ConstructionPointList.size() == 0 && (z_V2XData.RSILatitude == 0 || z_V2XData.RSILongitude == 0))////没有施工点信息，判断无施工事件发生，退出函数
	{
		construction_flag = false;
		return;
	}
	else if (ConstructionPointList.size() == 0 && z_V2XData.RSILatitude != 0 && z_V2XData.RSILongitude != 0) //若WarningList为空，只有事件点，事件点作为施工点存入WarningPointList
	{
		WarningPoint ConstructionPoint;
		ConstructionPoint.latitude = z_V2XData.RSILatitude;
		ConstructionPoint.longitude = z_V2XData.RSILongitude;
		ConstructionPointList.push_back(ConstructionPoint);
	}

	//遍历WarningPointList，找到距离车辆纵向距离最近的施工点，并保存其经度、纬度、与车辆的纵向距离
	for (WORD num = 0; num < ConstructionPointList.size(); num++)
	{
		double lng_distance_temp = 9999;
		GPSPoint2D constructionpoint_gps_temp;

		//对施工点的经纬度纠偏
		/*GPSPoint3D construction_gps_3D;
		construction_gps_3D.lat = ConstructionPointList[num].latitude;
		construction_gps_3D.lng = ConstructionPointList[num].longitude;
		construction_gps_3D.ang = 0;
		construction_gps_3D = V2XOBUOffSetCorrect(z_LocationOut, z_V2XData, construction_gps_3D);

		GPSPoint2D constructionpoint_gps_temp;
		constructionpoint_gps_temp.lat = construction_gps_3D.lat;
		constructionpoint_gps_temp.lng = construction_gps_3D.lng;*/

		constructionpoint_gps_temp.lat = ConstructionPointList[num].latitude;
		constructionpoint_gps_temp.lng = ConstructionPointList[num].longitude;
		//把当前施工点的经纬度坐标转换为以路径起点为原点的全局坐标
		GlobalPoint2D construction_global_temp;
		construction_global_temp = WGS84ToGlobal(constructionpoint_gps_temp);
		//根据全局坐标计算车辆和当前施工事件点的横向、纵向距离
		index = NearestId(construction_global_temp, PathPoint_LongFront);//距离施工点最近的车辆路点id
																		 //求出车辆距离当前施工点的纵向距离lng_distance_temp
		if (index >= 2)
		{
			lng_distance_temp = 0;
			for (int i = 0; i < index; i++)
			{
				lng_distance_temp += sqrt(pow((PathPoint_LongFront[i].x - PathPoint_LongFront[i + 1].x), 2) + pow((PathPoint_LongFront[i].y - PathPoint_LongFront[i + 1].y), 2));
			}
		}
		else if (index == 1)
		{
			lng_distance_temp = sqrt(pow((PathPoint_LongFront[index - 1].x - PathPoint_LongFront[index].x), 2) + pow((PathPoint_LongFront[index - 1].y - PathPoint_LongFront[index].y), 2));
		}
		else
		{
			lng_distance_temp = -9999;
		}
		double lng_distance = lng_distance_temp;
		//找到距离车辆纵向距离最近的施工点的经度、纬度、纵向距离
		if (min_distance_construction >= lng_distance_temp)
		{
			min_distance_construction = lng_distance_temp;
			near_lat_constructionpoint = ConstructionPointList[num].latitude;
			near_lng_constructionpoint = ConstructionPointList[num].longitude;
		}
		else
		{
			min_distance_construction = min_distance_construction;
		}
	}

	GlobalPoint2D pt;
	GlobalPoint2D pt_next;
	GlobalPoint2D near_construction_globalpoint; //距离车辆纵向距离最近的施工点
	GPSPoint2D near_constructionpoint_gps;//距离车辆纵向距离最近的施工点
	near_constructionpoint_gps.lat = near_lat_constructionpoint;
	near_constructionpoint_gps.lng = near_lng_constructionpoint;
	near_construction_globalpoint = WGS84ToGlobal(near_constructionpoint_gps);
	double near_lat_distance = 0;//车辆与纵向最近施工点的横向距离
	pt.x = PathPoint_LongFront[index].x;
	pt.y = PathPoint_LongFront[index].y;
	pt_next.x = PathPoint_LongFront[index + 1].x;
	pt_next.y = PathPoint_LongFront[index + 1].y;
	near_lat_distance = LatDis(near_construction_globalpoint, pt, pt_next);//最近施工点与车辆的横向距离

	// 判断纵向距离和横向距离是否小于一定距离，是则当前车道前方有施工事件，否则正常行驶
	if (min_distance_construction >= 0 && min_distance_construction <= 100)//纵向距大于0，小于等于100m
	{
		if (near_lat_distance >= 0 && near_lat_distance < WIDTH_LANE / 2)//且横向距离小于路宽一半,施工事件发生
		{
			construction_flag = true;
		}
		else
		{
			construction_flag = false;
		}

	}
	else //纵向距离小于0m或大于100m施工事件未发生
	{
		construction_flag = false;
	}

}

/****************************************************************************
名称：V2XEventDecision
描述：综合判断信号灯下发，施工车道和弱势行人参与者的函数
输入：定位信息，V2X输入信息，前方路点信息
输出：信号灯标志位，施工标志位和行人标志位
*****************************************************************************/
void CDecision::V2XEventDecision(const LocationOut z_LocationOut, const V2X_Data z_V2XData, const vector<WarningPoint> z_RSIWarningPointList, vector<GlobalPoint2D> PathPoint_Front_Long, vector<GlobalPoint2D> PathPoint_LF_Long, vector<GlobalPoint2D> PathPoint_RF_Long, bool &pedestrian_flag, WORD &V2XLight_flag, bool &construction_flag)
{
	
	if (z_V2XData.V2XWarnStatus == 3)
	{
		V2XSignalLight(z_V2XData, V2XLight_flag); //信号灯下发
	}
	else if (z_V2XData.V2XWarnStatus == 4)
	{
		V2XConstructionEvent(z_LocationOut, z_V2XData, z_RSIWarningPointList, PathPoint_Front_Long, construction_flag); //施工车道预警
		//V2XConstructionEventTemporal(z_LocationOut, z_V2XData, z_RSIWarningPointList, PathPoint_Front_Long, PathPoint_LF_Long, PathPoint_RF_Long, construction_flag);
		int a = 0;
	}
	else if (z_V2XData.V2XWarnStatus == 5)
	{
		V2XPedestrianJudge(z_LocationOut, z_V2XData, PathPoint_Front_Long, pedestrian_flag); //弱势行人预警
	}
	else
	{
		return;
	}
}
/****************************************************************************
名称：V2XConstructionEventTemporal
描述：施工车道预警：路侧设备广播施工信息，车辆提前换道或重新规划路线
输入：施工事件信息
输出：是否施工标志
*****************************************************************************/
void CDecision::V2XConstructionEventTemporal(const LocationOut z_LocationOut, const V2X_Data z_V2XData, const vector<WarningPoint> z_RSIWarningPointList, vector<GlobalPoint2D> PathPoint_Front_Long, vector<GlobalPoint2D> PathPoint_LF_Long, vector<GlobalPoint2D> PathPoint_RF_Long, bool &construction_flag)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	WORD RoadNum_Cur = z_LocationOut.road_num;            // 车辆当前道路编号		
	WORD LaneNum_Cur = z_LocationOut.lane_num;            // 车辆当前车道编号

	WORD Id_CurLane = z_LocationOut.id[LaneNum_Cur - 1];  // 车辆在当前车道上的路点id编号
	WORD IdSum_CurLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1].size();   	// 车辆所在车道的路点id总数																							
	WORD LaneSum = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][0].lane_sum;     // 车辆所在道路车道总数

	// 地图可变道属性，0不可变道，1可左变道，2可右变道，3可左右变道
	WORD LaneChg = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][Id_CurLane].lanechg_attribute;

	double Width_CurLane = 0; 	     //车辆所在车道宽度
	WORD Id_LeftLane = 0;            //车辆在车道左侧车道上的id编号，如果左侧无车道，则id为0 	
	WORD IdSum_LeftLane = 0;         //车辆在车道左侧车道的路点id总数，如果左侧无车道，则idsum为0 
	double Width_LeftLane = 0;		 //车辆所在车道左侧车道宽度，如果左侧无车道，向量大小为0 
	WORD Id_RightLane = 0;			 //车辆在右侧车道上的id编号，如果右侧无车道，则id为0 		
	WORD IdSum_RightLane = 0;        //车辆在右侧车道的路点id总数，如果右侧无车道，则idsum为0 
	double Width_RightLane = 0; 	 //车辆所在车道右侧车道宽度，如果右侧无车道，向量大小为0 	

	Width_CurLane = (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][Id_CurLane].lane_width) / 100.0;

	// 加载正前方100m范围路点坐标
	for (WORD i = min(IdSum_CurLane, Id_CurLane + ID_MORE); i < min(IdSum_CurLane, Id_CurLane + 200 + ID_MORE); i++)
	{
		GlobalPoint2D pt;
		pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
		pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
		PathPoint_Front_Long.push_back(pt);                     // 将车后60m范围路点存储至车后路点数组
	}

	// 加载左前方路点坐标
	if (LaneChg == 1)
	{
		if (LaneNum_Cur > 1)   // 要有左侧车道，则车道编号需要大于1
		{
			Id_LeftLane = z_LocationOut.id[LaneNum_Cur - 2];                                   // 车辆在当前车道左侧车道ID
			IdSum_LeftLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2].size();   // 左侧车道路点总数
			if ((Id_LeftLane > 0) && (Id_LeftLane < IdSum_LeftLane))                           // 避免左侧车道是新增车道和断头车道
			{
				Width_LeftLane = (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][Id_LeftLane].lane_width) / 100.0;

				GlobalPoint2D pt;
				for (int i = min(IdSum_LeftLane, Id_LeftLane + ID_MORE); i < min(IdSum_LeftLane, Id_LeftLane + 200 + ID_MORE); i++)                  // 后方120个点，差不多60m
				{
					pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][i].global_point.x;
					pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][i].global_point.y;
					PathPoint_LF_Long.push_back(pt);
				}
			}
		}
	}

	// 加载右前方路点坐标
	if (LaneChg == 2)
	{
		if (LaneNum_Cur < LaneSum)       //要有右侧车道，则车道编号需要小于车道总数
		{
			Id_RightLane = z_LocationOut.id[LaneNum_Cur];                                    // 车辆在右侧车道对应的路点ID
			IdSum_RightLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur].size();    // 右侧车道路点总数

			if ((Id_RightLane > 0) && (Id_RightLane < IdSum_RightLane))                      // 避免右侧新增车道和断头车道
			{
				// 右侧车道宽度
				Width_RightLane = (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur][Id_RightLane].lane_width) / 100.0;
				GlobalPoint2D pt;
				for (int i = min(IdSum_RightLane, Id_RightLane + ID_MORE); i < min(IdSum_RightLane, Id_RightLane + 200 + ID_MORE); i++)   //120个点，差不多60m
				{
					pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur][i].global_point.x;
					pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur][i].global_point.y;
					PathPoint_RF_Long.push_back(pt);
				}
			}
		}
	}

	GPSPoint2D egovehicle_GPS2D;
	GlobalPoint2D egovehicle_Global2D;
	GPSPoint2D rsipoint_GPS2D;
	GlobalPoint2D rsipoint_Global2D;

	egovehicle_GPS2D.lat = z_LocationOut.gpspoint.lat;
	egovehicle_GPS2D.lng = z_LocationOut.gpspoint.lng;

	egovehicle_Global2D = WGS84ToGlobal(egovehicle_GPS2D);

	rsipoint_GPS2D.lat = z_V2XData.RSILatitude;
	rsipoint_GPS2D.lng = z_V2XData.RSILongitude;

	rsipoint_Global2D = WGS84ToGlobal(rsipoint_GPS2D);

	double rsi_distance = 9999;

	rsi_distance = CalcDistance(egovehicle_GPS2D, rsipoint_GPS2D);

	int l_size_Point_F = PathPoint_Front_Long.size();
	int l_size_Point_LF = PathPoint_LF_Long.size();
	int l_size_Point_RF = PathPoint_RF_Long.size();
	double l_min_distance_front = 9999;
	double l_min_distance_lf = 9999;
	double l_min_distance_rf = 9999;
	double l_distance = 9999;
	vector<double> l_distance_v2x_pathpoint_front;
	vector<double> l_distance_v2x_pathpoint_lf;
	vector<double> l_distance_v2x_pathpoint_rf;
	construction_flag = false;

	if (rsi_distance >= 0 && rsi_distance <= 100) //自车和v2x车辆距离小于100m，才进行判断
	{
		for (int i = 0; i < l_size_Point_F; i++)  //计算v2x车辆到自车正后方每个路点的距离，存储到l_distance_v2x_pathpoint_rear向量中
		{
			l_distance = CalcDistance(rsipoint_Global2D, PathPoint_Front_Long[i]);
			l_distance_v2x_pathpoint_front.push_back(l_distance);
		}
		for (int i = 0; i < l_size_Point_F; i++) //l_distance_v2x_pathpoint_rear向量中，找出v2x车辆到自车正后方路点的最小距离
		{
			if (l_distance_v2x_pathpoint_front[i] < l_min_distance_front)
			{
				l_min_distance_front = l_distance_v2x_pathpoint_front[i];
			}
		}

		for (int i = 0; i < l_size_Point_LF; i++)  //计算v2x车辆到自车左后方每个路点的距离，存储到l_distance_v2x_pathpoint_lr向量中
		{
			l_distance = CalcDistance(rsipoint_Global2D, PathPoint_LF_Long[i]);
			l_distance_v2x_pathpoint_lf.push_back(l_distance);
		}
		for (int i = 0; i < l_size_Point_LF; i++)  //l_distance_v2x_pathpoint_lr向量中，找出v2x车辆到自车左后方路点的最小距离
		{
			if (l_distance_v2x_pathpoint_lf[i] < l_min_distance_lf)
			{
				l_min_distance_lf = l_distance_v2x_pathpoint_lf[i];
			}
		}

		for (int i = 0; i < l_size_Point_RF; i++) //计算v2x车辆到自车右后方每个路点的距离，存储到l_distance_v2x_pathpoint_rr向量中
		{
			l_distance = CalcDistance(rsipoint_Global2D, PathPoint_RF_Long[i]);
			l_distance_v2x_pathpoint_rf.push_back(l_distance);
		}
		for (int i = 0; i < l_size_Point_RF; i++) //l_distance_v2x_pathpoint_rr向量中，找出v2x车辆到自车右后方路点的最小距离
		{
			if (l_distance_v2x_pathpoint_rf[i] < l_min_distance_rf)
			{
				l_min_distance_rf = l_distance_v2x_pathpoint_rf[i];
			}
		}

		if ((l_min_distance_lf < l_min_distance_rf) && (l_min_distance_lf < l_min_distance_front) && (l_min_distance_lf < 2)) //判断v2x车辆是否在自车左后方车道
		{
			construction_flag = false;
		}
		else if ((l_min_distance_rf < l_min_distance_lf) && (l_min_distance_rf < l_min_distance_front) && (l_min_distance_rf < 2)) //判断v2x车辆是否在自车右后方车道
		{
			construction_flag = false;
		}
		else if ((l_min_distance_front < l_min_distance_lf) && (l_min_distance_front < l_min_distance_rf) && (l_min_distance_front < 2)) //判断v2x车辆是否在自车正后方车道
		{

			// 局部变量初始化
			double WIDTH_LANE = 3.75; // 车道宽 
			int constructionpoint_num = 0;
			int index = 0;//距离施工点最近的车辆路点id
			double min_distance_construction = 9999; // 车与最近的施工路点的距离
			float near_lat_constructionpoint = 0; // 距离车辆最近施工点纬度
			float near_lng_constructionpoint = 0; // 距离车辆最近施工点经度
			
			vector<WarningPoint> ConstructionPointList;
			//读取V2X施工车道预警信息
			ConstructionPointList = z_RSIWarningPointList;

			if (ConstructionPointList.size() == 0 && (z_V2XData.RSILatitude == 0 || z_V2XData.RSILongitude == 0))////没有施工点信息，判断无施工事件发生，退出函数
			{
				construction_flag = false;
				return;
			}
			else if (ConstructionPointList.size() == 0 && z_V2XData.RSILatitude != 0 && z_V2XData.RSILongitude != 0) //若WarningList为空，只有事件点，事件点作为施工点存入WarningPointList
			{
				WarningPoint ConstructionPoint;
				ConstructionPoint.latitude = z_V2XData.RSILatitude;
				ConstructionPoint.longitude = z_V2XData.RSILongitude;
				ConstructionPointList.push_back(ConstructionPoint);
			}

			//遍历WarningPointList，找到距离车辆纵向距离最近的施工点，并保存其经度、纬度、与车辆的纵向距离
			for (WORD num = 0; num < ConstructionPointList.size(); num++)
			{
				double lng_distance_temp = 9999;
				GPSPoint2D constructionpoint_gps_temp;

				//对施工点的经纬度纠偏
				/*GPSPoint3D construction_gps_3D;
				construction_gps_3D.lat = ConstructionPointList[num].latitude;
				construction_gps_3D.lng = ConstructionPointList[num].longitude;
				construction_gps_3D.ang = 0;
				construction_gps_3D = V2XOBUOffSetCorrect(z_LocationOut, z_V2XData, construction_gps_3D);

				GPSPoint2D constructionpoint_gps_temp;
				constructionpoint_gps_temp.lat = construction_gps_3D.lat;
				constructionpoint_gps_temp.lng = construction_gps_3D.lng;*/

				constructionpoint_gps_temp.lat = ConstructionPointList[num].latitude;
				constructionpoint_gps_temp.lng = ConstructionPointList[num].longitude;
				//把当前施工点的经纬度坐标转换为以路径起点为原点的全局坐标
				GlobalPoint2D construction_global_temp;
				construction_global_temp = WGS84ToGlobal(constructionpoint_gps_temp);
				//根据全局坐标计算车辆和当前施工事件点的横向、纵向距离
				index = NearestId(construction_global_temp, PathPoint_Front_Long);//距离施工点最近的车辆路点id
																				 //求出车辆距离当前施工点的纵向距离lng_distance_temp
				if (index >= 2)
				{
					lng_distance_temp = 0;
					for (int i = 0; i < index; i++)
					{
						lng_distance_temp += sqrt(pow((PathPoint_Front_Long[i].x - PathPoint_Front_Long[i + 1].x), 2) + pow((PathPoint_Front_Long[i].y - PathPoint_Front_Long[i + 1].y), 2));
					}
				}
				else if (index == 1)
				{
					lng_distance_temp = sqrt(pow((PathPoint_Front_Long[index - 1].x - PathPoint_Front_Long[index].x), 2) + pow((PathPoint_Front_Long[index - 1].y - PathPoint_Front_Long[index].y), 2));
				}
				else
				{
					lng_distance_temp = -9999;
				}
				
				//找到距离车辆纵向距离最近的施工点的经度、纬度、纵向距离
				if (min_distance_construction >= lng_distance_temp)
				{
					min_distance_construction = lng_distance_temp;
					near_lat_constructionpoint = ConstructionPointList[num].latitude;
					near_lng_constructionpoint = ConstructionPointList[num].longitude;
				}
				else
				{
					min_distance_construction = min_distance_construction;
				}
			}

			// 判断纵向距离和横向距离是否小于一定距离，是则当前车道前方有施工事件，否则正常行驶
			if (min_distance_construction >= 0 && min_distance_construction <= 100)//纵向距大于0，小于等于100m
			{
				construction_flag = true;
			}
			else //纵向距离小于0m或大于100m施工事件未发生
			{
				construction_flag = false;
			}
		}
		else
		{
			construction_flag = false;
		}
	}
	else
	{
		construction_flag = false;
	}
}








