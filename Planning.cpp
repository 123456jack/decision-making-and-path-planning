#include "stdafx.h"
#include "Planning.h"
#include "GAC_Auotpilot_DP.h"

// �洢��ʷ֡·������
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
	BYTE count = 0;                          // ѭ����¼
	static PlanningOut Planningresult;       // �滮���
	static PlanningStatus Planningshow;      // �滮���������ʾ
	
	LARGE_INTEGER new_time;                  // ��¼��ǰʱ��
	new_time.QuadPart = 0;
	QueryPerformanceCounter(&new_time);
	hisTime = (double)new_time.QuadPart;

	BYTE start_flag = 0;

	memset(last_Bpoints, 0, sizeof(last_Bpoints));

	while (true)
	{
		memset(&Planningshow, 0, sizeof(Planningshow));     // ��ʼ���滮�߳̽ṹ������
		DWORD dw = WaitForSingleObject(app->x_DecisionEvent, 1000);
		if (dw != 0)
		{
			continue;
		}
				
		/*�����߳�����*/
		new_time.QuadPart = 0;
		QueryPerformanceCounter(&new_time);
		double curTime = (double)new_time.QuadPart;
		z_period_last = (curTime - hisTime) / SYS_Frequency * 1000;

		if (start_flag == 0)   // ��һ��period_max����period_last
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

		// ��ȡ���߽����Ϣ
		app->x_criticalDecision.Lock();
		z_DecisionOut = app->GetDesicionOut();        //��ȡ���߽��
		app->x_criticalDecision.Unlock();

		// ��ȡ������λ��Ϣ
		app->x_criticalLocation.Lock();
		z_LocationOut = app->GetLocationOut();        // ��ȡ��λ���
		app->x_criticalLocation.Unlock();
		
		// ��ȡ����λ����Ϣ
		app->x_criticalVhclHisPos.Lock();
		z_VehStatus = app->GetVehStatus();            // ��ȡ����״̬,����λ����Ϣ
		app->x_criticalVhclHisPos.Unlock();

		// ��ȡ������Χ�ϰ�����Ϣ
		app->x_criticalObstacle.Lock();
		z_Obs = app->GetObj();                        // ��ȡ�ϰ�����Ϣ
		app->x_criticalObstacle.Unlock();

		// ����滮·��
		GlobalPoint2D road_points[200] = {};

		// ����Ԥ�����
		Calculate_aim_dis(z_DecisionOut, z_LocationOut, z_VehStatus, faraim_dis, nearaim_dis);

		// ����Ԥ���
		SearchAimPoint(z_DecisionOut, z_LocationOut, z_VehStatus, aimpoint_far, aimpoint_near);

		// ��ʼ�滮
		if (count == 0)
		{
			InitialPlanning(z_DecisionOut,z_LocationOut,z_VehStatus,aimpoint_far,aimpoint_near, road_points);	
			memcpy(last_Bpoints, road_points, sizeof(road_points));
		}

		// �ж�������ֲ�·����Ծ���
		GetVhclLocalState(z_LocationOut, last_Bpoints, path_lat_dis, path_dir_err, path_near_id, path_front_near_id, remain_dis);

		// �ع滮�ж�
		afresh_planning = UpdatePlanJudge(z_DecisionOut, z_LocationOut, his_behavior, afresh_cause);

		// ·���滮 		
		if (afresh_planning)   // ���¹滮�ֲ�·��
		{			
			PathPlanning(z_DecisionOut,afresh_cause, z_LocationOut, aimpoint_far, aimpoint_near, road_points);
			// memcpy(last_Bpoints, road_points, sizeof(road_points));
		}		
		else                  // ������һ֡�ֲ�·��
		{
			memset(road_points, 0, sizeof(road_points));
			memcpy(road_points, last_Bpoints, sizeof(last_Bpoints));
		}
		
		
		// ��·��ṹ������ת����vector����
		// vector<GlobalPoint2D>road_points_ser(road_points, road_points + 200);

		// �洢�滮·����ʣ��·��
		vector<GlobalPoint2D>rem_path;
		rem_path.clear();
		for (int i = path_near_id;i < 200;i++)
		{
			rem_path.push_back(road_points[i]);
		}

		// �ֲ�·���ϰ������
		double mindist_lat=999;         // �ֲ�·���ϰ���������
		double mindist_lon=999;         // �ֲ�·���ϰ����������
		ObPoint min_ob_pt;              // �ֲ�·���ϰ�������
		WORD mindist_pathid;            // �ֲ�·�����ϰ���·��ID
		bool ob_flag=false;             // �ֲ�·�����ϰ����־λ

		// �����ֲ�·���ϰ���
		ob_flag=SearchObstacle(rem_path,z_Obs,(float)(-1.1),(float)(1.1),mindist_lat,mindist_lon,min_ob_pt,mindist_pathid);

		// ���ٹ滮
		SpeedPlanning(ob_flag, z_DecisionOut, z_LocationOut, mindist_lon, mindist_lat, faraim_dis, brakespeed, acc_flag, des_acc);	

		// �滮״̬����ʾ�ڵ��Խ���
		Planningshow.afresh_cause = afresh_cause;          // �ع滮ԭ��
		Planningshow.near_ob_dist = mindist_lon;           // �ֲ��滮·������ϰ������
		Planningshow.planspeed = brakespeed;               // �ֲ��滮
		Planningshow.planacc = des_acc;                    // �滮���
		Planningshow.trafficlight=z_DecisionOut.light;     // ��ͨ��

		for (UINT i = 0;i < 100;i++)
		{
			Planningshow.path_points[i] = road_points[2*i];		
		}

		
		app->SetPlanningStatus(Planningshow);
		
		// ����滮���
		Planningresult.cnt = count % 100;
		Planningresult.APA = 0;
		Planningresult.brakedis = mindist_lon;
		Planningresult.brake_speed = 0;
		Planningresult.desaccVd = acc_flag;
		Planningresult.desacc = des_acc;
		Planningresult.desspd = brakespeed;
		Planningresult.desstr = 0; 
		Planningresult.desstrVd = false;             // ��λ���ȿ���
		Planningresult.light = z_DecisionOut.light;  
		Planningresult.radius = CalculateRadius();   // �������ʰ뾶
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
		// �洢�滮�������
		app->SetUdpSendCtrl(Planningresult);
		// �洢��һ֡����
		his_behavior = z_DecisionOut.behavior;                      // �洢��һ֡��Ϊ
		memcpy(last_Bpoints, road_points, sizeof(road_points));     // �洢��һ֡

		count++;   // ÿ����һ�θ����ƣ�����һ��
		if (count % 100 == 1)
		{
			count = 1;
		}
		SetEvent(app->x_PlanningEvent);     // �滮�߳����
	
	}
	return 0;
}



/*********************************************************************************************************
���ܣ��ֱ����·�ϡ�Ԥ·�ں�·�ڳ���Ԥ�����
���룺
decision_result ����������
vhcl_location ������λ
vhcl_status  ����λ��
�����
faraim_dis ԶԤ������
nearaim_dis ��Ԥ������
**********************************************************************************************************/
void CPlanning::Calculate_aim_dis(DecisionOut decision_result, LocationOut vhcl_location, VehStatus vhcl_status, FLOAT &faraim_dis, FLOAT &nearaim_dis)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	BYTE road_task = 0;                // ����״̬��0��ʾ·�ϣ�1��ʾԤ·�ڣ�2��ʾ·��
	road_task = vhcl_location.pos;     // ȷ��������������״̬
	
	// Զ��Ԥ������ʼ��
	faraim_dis = 0;
	nearaim_dis = 0;
	
	switch (road_task)
	{
		// ·��Ԥ��������
	case 0:   
		// ·��ԶԤ������
		faraim_dis = (vhcl_location.velocity / 3.6) * 5 + 4;   
		// ·��ԶԤ���������
		if (faraim_dis > ROAD_FARAIM_MAX)
		{
			faraim_dis = ROAD_FARAIM_MAX;
		}
		else if (faraim_dis < ROAD_FARAIM_MIN)
		{
			faraim_dis = ROAD_FARAIM_MIN;
		}
		// ·�Ͻ�Ԥ�����
		nearaim_dis = faraim_dis;
		break;

		// Ԥ·��Ԥ�����
	case 1:  
		// Ԥ·��ԶԤ������
		faraim_dis = PRE_INTER_FARAIM;
		// Ԥ·�ڽ�Ԥ������
		nearaim_dis = faraim_dis;
		break;

		// ·��Ԥ�����
	case 2:
		// ·��ԶԤ������
		faraim_dis = INTER_FARAIM;
		// ·�ڽ�Ԥ������
		nearaim_dis = faraim_dis;
		break;
	default:
		break;		
	}
}


/*********************************************************************************************************
���ܣ�����Զ��Ԥ���
���룺
decision_result ����������
vhcl_location ������λ
vhcl_status  ����λ��״̬
�����
aimpoint_far ԶԤ���
aimpoint_near ��Ԥ���
**********************************************************************************************************/
void CPlanning::SearchAimPoint(DecisionOut decision_result, LocationOut vhcl_location, VehStatus vhcl_status, AimPoint &aimpoint_far, AimPoint &aimpoint_near)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	BYTE road_task = 0;                // ����״̬��0��ʾ·�ϣ�1��ʾԤ·�ڣ�2��ʾ·��
	BOOL plan_divflag = false;         // �ֶι滮��־λ��false��ʾ���ֶι滮��true��ʾ�ֶι滮
	DOUBLE sum_dis = 0;                // �������
	INT curpoint_sum=999;              // ��ǰ����·������
    // INT tarpoint_sum = 999;         // Ŀ�공��·������
	INT curpoint_id = 0;               // ��ǰ����·��ID
	INT aimpoint_id = 0;               // Ԥ���ID
	INT leftpoint_id = 0;              // ��ǰ������Ӧ��೵��·��ID
	INT rightpoint_id = 0;             // ��ǰ������Ӧ�Ҳ೵��·��ID
	INT leftpoint_sum = 0;             // ��೵����೵��·������
	INT rightpoint_sum = 0;            // �Ҳ೵���Ҳ೵��·������

	// ȷ��������������״̬
	road_task = vhcl_location.pos;   

	// ������·��λ��
	WORD LastRoadNum_Cur = vhcl_location.last_roadnum; // ·����һ�ε�·���
	WORD NextRoadNum_Cur = vhcl_location.next_roadnum; // ·����һ�ε�·���
	WORD LastLaneNum_Cur = vhcl_location.last_lanenum; // ·����һ�ε�·�ĳ������
	WORD NextLaneNum_Cur = vhcl_location.next_lanenum; // ·����һ�ε�·�ĳ������

	// ������·��λ��
	WORD RoadNum_Cur = vhcl_location.road_num;         // ·�ϵ�·���
	WORD LaneNum_Cur = vhcl_location.lane_num;         // ·�ϳ������
	WORD LaneSum = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][0].lane_sum;     // �������ڵ�·��������
	
	// ·�ϼ�·�ڳ���λ��
	if (road_task == 0 || road_task == 1)     // ·�ϼ�Ԥ·��
	{
		BYTE RoadNum_Cur = vhcl_location.road_num;         // �����ڵ�ǰ��·���		
		BYTE LaneNum_Cur = vhcl_location.lane_num;         // �����ڵ�ǰ�������
		int Id_Cur = vhcl_location.id[LaneNum_Cur - 1];    // �����ڵ�ǰ����·��ID
		// ·��·������
		WORD IdSum_CurLane = app->planning_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1].size();
		// ·��·������
		WORD IdSum_Inter = app->planning_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1].size();
	}
	else                                      // ·��
	{
		BYTE RoadNum_Next = vhcl_location.road_num;         // ������һ�ε�·���		
		BYTE LaneNum_Next = vhcl_location.lane_num;         // ������һ�γ������
		int Id_Inter = vhcl_location.id[LastLaneNum_Cur - 1];  // ������·�ڳ���
	    // ·��·������
		WORD IdSum_Inter = app->planning_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1].size();
		// ��һ��·��·������
		WORD IdSum_NextLane = app->planning_MapData[RoadNum_Next - 1][LaneNum_Next - 1].size();
	}	
		
	curpoint_id = vhcl_location.id[vhcl_location.lane_num - 1];                                            // ��ǰ����·��id		
	curpoint_sum = app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1].size();   // ��ǰ����·������
	
	// ��೵��·���·������
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
	
	// �Ҳ೵��·���·������
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
	
	// Ŀ�공��·������
    //tarpoint_sum=app->planning_MapData[vhcl_location.road_num - 1][vhcl_location.lane_num - 1].size();

	// �ж��Ƿ���÷ֶι滮
	if (fabs(faraim_dis - nearaim_dis) < 1)
	{
		plan_divflag = false;         // ���ֶι滮
	}
	else
	{
		plan_divflag = true;          // �ֶι滮
	}

	// �ֱ�ȷ��Ԥ���
	switch (road_task)
	{
		
	case 0:   // ·��Ԥ���

		if (decision_result.target_lanenum == vhcl_location.lane_num)
		{
			// ����Ϊ���߽��Ϊ������������ʻ
			if (decision_result.behavior == 1)
			{
				// ���ֶι滮
				if (plan_divflag == false)
				{
					// �ۼӾ��븴λ
					for (int i = curpoint_id;i < curpoint_sum - 1;i++)
					{
						// �ۼӼ���
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
					// ���÷ֶι滮ʱ����Ԥ����ԶԤ����ص�
					aimpoint_near = aimpoint_far;
				}
				
			}
		
		}
		else if (decision_result.target_lanenum != vhcl_location.lane_num)
		{
			// ���󻻵���ʻ
			if (decision_result.behavior == 2)
			{
				// ���ֶι滮
				if (plan_divflag == false)
				{
					for (int i = leftpoint_id;i < leftpoint_sum - 1;i++)
					{
						// �ۼӼ���
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
			// ���һ�����ʻ
			else if (decision_result.behavior == 3)
			{
				// ���ֶι滮
				if (plan_divflag == false)
				{
					for (int i = rightpoint_id;i < leftpoint_sum - 1;i++)
					{
						// �ۼӼ���
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
	case 1:   // Ԥ·��Ԥ���
		sum_dis = 0;
		for (int i = 0;i < decision_result.refpath.size()-1;i++)
		{
			sum_dis += CalcDistance(decision_result.refpath[i], decision_result.refpath[i + 1]);

			// ȷ��Ԥ���
			if ((sum_dis - 4 )> faraim_dis)
			{
				// Ԥ��㸳ֵ
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
	case 2:   // ·��Ԥ���
		sum_dis = 0;
		for (int i = 0;i < decision_result.refpath.size() - 1;i++)
		{
			sum_dis += CalcDistance(decision_result.refpath[i], decision_result.refpath[i + 1]);

			// ȷ��Ԥ���
			if ((sum_dis - 4) > faraim_dis)
			{
				// Ԥ��㸳ֵ
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
��ʼ�滮
���룺
decision_result ����������
vhcl_location ������λ
vhcl_status ����λ��
aimpoint_far ԶԤ���
aimpoint_near ��Ԥ���
�����

****************************************************************************************************************/
void CPlanning::InitialPlanning(DecisionOut decision_result, LocationOut vhcl_location, VehStatus vhcl_status, const AimPoint aimpoint_far, const AimPoint aimpoint_near,GlobalPoint2D road_points[])
{
	GlobalPoint2D Bezier_points[200] = {};                    // ����洢·��
	GlobalPoint3D vhcl_point = vhcl_location.globalpoint;   // ������ǰ����
	
	// ��ʼ���洢·��
	memset(road_points, 0, sizeof(road_points));     
	memset(Bezier_points, 0, sizeof(Bezier_points)); 

	// ��ʼ�滮
	BezierPlanning(vhcl_point, aimpoint_far.Aim_point, Bezier_points, 200);

	// ������ʼ���滮��·��
	memcpy(road_points,Bezier_points, sizeof(Bezier_points));

}

/***********************************************************************************************************************************
���㳵������ھֲ�·����λ��
���룺
vhcl_location ������λ
last_Bpoints ��һ���ڵľֲ��滮·��
�����
mindist_lat ��Ծֲ�·������ƫ��
remain_dis ʣ�����
path_dir_err �������
************************************************************************************************************************************/
void CPlanning::GetVhclLocalState(LocationOut vhcl_location, const GlobalPoint2D last_Bpoints[], double& mindist_lat, double& path_dir_err, int& mindist_id, int& front_mindist_id, double& remain_dis)
{	
	GlobalPoint2D vhcl_pose;           // ���嵱ǰλ�õ�����
	double dist;
	double front_min_dis = 9999;
	double middle_min_dis = 9999;
	GlobalPoint2D pt, pt_next;	
	
	// ��Ϣ��ʼ��
	remain_dis = 0;
	mindist_lat = 9999;
	

	// ��ǰλ�����긳ֵ
	vhcl_pose.x = vhcl_location.globalpoint.x;
	vhcl_pose.y = vhcl_location.globalpoint.y;
	
	for (UINT i = 0; i < 200; i++) //�������ѹ滮·���ĺ�����룬����ƫ�����һ��ֵ��Ҫ�ع滮
	{
		dist = sqrt(pow(vhcl_pose.x - last_Bpoints[i].x, 2) + pow(vhcl_pose.y - last_Bpoints[i].y, 2));
		if (dist < mindist_lat)
		{
			mindist_lat = dist;
			mindist_id = i;       
		}

		front_mindist_id = mindist_id + 8;	     // ��ͷ���·��	
	}

	
	// �����ھֲ�·�������ID
	int index = mindist_id;  

	if (mindist_id == 199)
	{
		index = mindist_id - 1;
	}
	pt.x = last_Bpoints[index].x;
	pt.y = last_Bpoints[index].y;
	pt_next.x = last_Bpoints[index + 1].x;
	pt_next.y = last_Bpoints[index + 1].y;

	// ���㳵�� 
	mindist_lat = GetLatDis(vhcl_pose, pt, pt_next);

	for (int i = front_mindist_id; i < 199; i++)
	{
		remain_dis += sqrt(pow(last_Bpoints[i + 1].x - last_Bpoints[i].x, 2) + pow(last_Bpoints[i + 1].y - last_Bpoints[i].y, 2));
	}
	// ����ֲ�·�㺽��
	double pt_dir = GetRoadAngle(pt, pt_next);
	// ���㺽�����
	path_dir_err = GetAngleErr(pt_dir, vhcl_location.globalpoint.dir);//�������ѹ滮·���ķ���нǣ��нǴ���һ��ֵ��Ҫ�ع滮
}
/****************************************************************************************************
���㳵����ǰλ����ֲ�·������ƫ��
���룺
cur_pt ������ǰλ������
pt ��ǰ·������
pt_next ��ǰ·����һ��·������
�����
lat_dis ���غ������
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
�����������·��������������붫��нǣ���ʱ��Ϊ������Χ��[0,360��]
���룺
apoint:��һ��ȫ������
bpoint:�ڶ���ȫ������
�����
apoint��bpoint��������������ļн�
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
��ȡ��������֮��ļнǣ���ʱ��Ϊ������dir1ת��dir2
���룺
dir1:��׼����
dir2:����нǷ���
�����
����нǷ�����Ի�׼����ļнǣ�������֮��(-180��~ 180��]
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
�ж��Ƿ���Ҫ�ع滮
���룺
obpoints��ob_num:�ϰ������ϰ������
nearid:�ֲ�·���Ͼ��복ͷ������id
�����
afresh_cause���ع滮����
�Ƿ���Ҫ�ع滮
********************************************************************************************/
bool CPlanning::UpdatePlanJudge(const DecisionOut decision_result,const LocationOut vhcl_location,const int last_behavior, int &afreshcause)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();	

	afreshcause = 0;

	
	if (last_behavior != decision_result.behavior) //״̬�л�����ع滮
	{
		afreshcause = 1;
		return true;
	}

	if (fabs(path_lat_dis) > 0.2)       //����ƫ���������ع滮
	{
		afreshcause = 2;
		return true;
	}
	if (fabs(path_dir_err) > 45)         //����ƫ���������ع滮
	{
		afreshcause = 3;
		return true;
	}

	if ((vhcl_location.pos == 0)  && remain_dis < ROAD_REMAIN_DISTANCE )    //·��ʣ����벻���ع滮
	{
		afreshcause = 4;
		return true;
	}
	else if (vhcl_location.pos != 0 && remain_dis < INTER_REMAIN_DISTANCE)  //Ԥ·�ں�·��ʣ����벻���ع滮
	{
		afreshcause = 4;
		return true;
	}
	return false;
}

/************************************************************************************************************************************
�ֲ�·���滮
���룺
afresh_planning �ع滮��־λ
afresh_cause �ع滮ԭ��
vhcl_location ����λ��
aimpoint_far ԶԤ���
aimpoint_near ��Ԥ���
�����
road_points[] �滮�ľֲ�·��
*************************************************************************************************************************************/
void CPlanning::PathPlanning(const DecisionOut DecisionResult ,const int afresh_cause, LocationOut vhcl_location, const AimPoint aimpoint_far, const AimPoint aimpoint_near, GlobalPoint2D road_points[])
{
	GlobalPoint2D Bezier_points[200] = {};                  // ����洢·��
	GlobalPoint3D vhcl_point = vhcl_location.globalpoint;   // ������ǰ����
	BYTE vhcl_pos = vhcl_location.pos;                      // ������ǰ·�Σ�0��ʾ·�ϣ�1��ʾԤ·�ڣ�2��ʾ·��
	//vector<GlobalPoint2D> ref_points;
	GlobalPoint2D Ref_points[200] = {};                     // ����ο��洢·��
	

	// ��ʼ���洢·��
	memset(road_points, 0, sizeof(road_points));
	memset(Bezier_points, 0, sizeof(Bezier_points));

	// ��ʼ�滮
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
		// ��·����Ȼ�
		MeanPoints(Ref_points, aimpoint_far.Aim_id, Bezier_points, 200);
	}

	// ������ʼ���滮��·��
	memcpy(road_points, Bezier_points, sizeof(Bezier_points));
}

/****************************************************************************************************************************************
���ٹ滮
���룺
ob_flag �ֲ�·���ϰ����־λ
decision_result ���߹滮���
vhcl_location ������λ
mindist_lon �ֲ�·���ϰ����������
mindist_lat �ֲ�·���ϰ���������
*****************************************************************************************************************************************/
void CPlanning::SpeedPlanning(const bool ob_flag, const DecisionOut decision_result, const LocationOut vhcl_location,
	const double mindist_lon, const double mindist_lat,const FLOAT faraim_dis,double &brake_speed, bool &acc_flag, double &des_acc)
{
	switch (vhcl_location.pos)
	{
	case 0:     // ·�ϳ������ٹ滮
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
	case 1:     // Ԥ·�ڳ������ٹ滮
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
		break;  // ·�ڳ������ٹ滮
	default:
		break;  
	}
	
}
/******************************************************************************************************************
����ֲ��滮·������
���룺
last_Bpoints ��֡�滮·��
path_front_near_id ��ͷ�ڹ滮·����id
path_near_id CPTλ���ڹ滮·����id
�����
radius ��·����
*******************************************************************************************************************/
double CPlanning::CalculateRadius()//����滮·������
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

