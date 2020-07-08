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
	z_behavior = 1;               // ��ʼ��Ϊ��������
	z_velocity_expect = 10;

	z_period_max = 0;
	z_period_last = 0;
	z_behavior_to_dlg = 0;

	leftlight_time = 0;          // ��ת���ʱ��
	rightlight_time = 0;         // ��ת���ʱ��

	his_behavior = 1;            // ��ʼ����һ֡��Ϊ���߽��
	his_light_status = 0;        // ��ʼ����һ֡ת���״̬
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
�����̣߳�
0.��ȡ��̬���ݣ���·����������·���˹�ϵ����ͼ����ֱ�ӵ���app->decision_MapData[][][]��
while(1)
{
1.��ȡ��֪���룻
2.��ȡ��λ���룻
3.��ȡ������Ϣ������ת����������
4.��ȡ�滮·����
5.Switch(Pos)
{
case ·�ϣ�
case Ԥ·�ڣ�
case ·�ڣ�
}
6.������Ӧ��Ϣ���滮��
}
************************************************/
DWORD CDecision::CDecisionThread(void)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();


	z_RoadInfo = app->GetRoadInfo();             // ��ȡ��·���˹�ϵ

	z_RoadNavi = app->GetRoadNavi();             // ��ȡ��·������·��

	//app->x_criticalLocation.Lock();
	//z_LocationOut = app->GetLocationOut();     // ��ȡ��λ���
	//app->x_criticalLocation.Unlock();


	//while (z_LocationOut.period_last == 0)     // ˵����λ��δ�н�������
	//{
	//	z_LocationOut = app->GetLocationOut();   // ��ȡ��λ���
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


	// �ϰ�����ֵ�ʱ�估����
	//UINT Obsavoid_time = 0;                               // �ϰ�����ֵĴ���
	//UINT No_obsavoid_time = 0;                            // ���Ϲ�����û�г����ϰ������
	//static UINT obsavoid_time = 0;                        // �ϰ�����ֵĴ���
	//static UINT no_obsaviod_time = 0;                     // ���Ϲ������ϰ�����ʧ����

	static double leftlight_time = 0;                       // ��ת���ʱ��
	static double rightlight_time = 0;                      // ��ת���ʱ��   

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

		/*�����߳�����*/
		new_time.QuadPart = 0;
		QueryPerformanceCounter(&new_time);
		double curTime = (double)new_time.QuadPart;
		z_period_last = (curTime - hisTime) / SYS_Frequency * 1000;

		if (start_flag == 0)   //��һ��period_max����period_last
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
		z_LocationOut = app->GetLocationOut();        //��ȡ��λ���
		app->x_criticalLocation.Unlock();

		z_VehStatus = app->GetVehStatus();            //��ȡ����״̬  
		z_Obs = app->GetObj();                        //��ȡ�ϰ�����Ϣ

	    //z_DynaObs_front = app->GetDynaObj_front();   // ��ȡǰ����̬�ϰ�����Ϣ
	    //z_DynaObs_rear = app->GetDynaObj_rear();     // ��ȡ�󷽶�̬�ϰ�����Ϣ
		//z_Person = app->GetPerson();				   // ��ȡ������Ϣ
		//z_LinePoints = app->GetLinepoints();         // ��ȡ��������Ϣ
		app->x_criticalV2X_Data.Lock();
		z_V2XData = app->GetV2XData();
		z_RSIWarningPointList = app->GetRSIWarningPointList();
		app->x_criticalV2X_Data.Unlock();

		// ��ͬ��������
		switch (z_LocationOut.pos)
		{
		case 0:
			SegmentDecision();           // ·��·���滮
			break;
		case 1:
			PreStubDecision();           // Ԥ·��·���滮
			break;
		case 2:
			StubDecision();              // ·��·���滮
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

		// �洢��ʷ֡����		
		his_behavior = z_behavior;
		his_light_status = z_light_status;		
		his_target_lanenum = z_target_lanenum;

		app->SetDecisionOut(z_decisionout);

		SetEvent(app->x_DecisionEvent);
	}
	return 0;
}

/*************************************************************************************
�������ƣ�SegmentDecision() 
����������·����Ϊ����
���룺
�����
**************************************************************************************/
void CDecision::SegmentDecision()
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	vector<GlobalPoint2D> PathPoint_LongFront;           // ������ǰ����·��100m
	vector<GlobalPoint2D> PathPoint_Front_Long;
	vector<GlobalPoint2D> PathPoint_LF_Long;
	vector<GlobalPoint2D> PathPoint_RF_Long;

	vector<GlobalPoint2D> PathPoint_Front;                // ������ǰ����·��60m 
	vector<GlobalPoint2D> PathPoint_Rear;                 // �������󷽵�·��20m,
	vector<GlobalPoint2D> PathPoint_LF;                   // ������ǰ����·��60m,�������峵����������СΪ0	
	vector<GlobalPoint2D> PathPoint_LR;                   // ������󷽵�·��20m,�������޳�����������СΪ0
	vector<GlobalPoint2D> PathPoint_RF;	                  // ������ǰ����·��60m����ǰ����
	vector<GlobalPoint2D> PathPoint_RR;                   // �����Һ󷽵�·��20m��������
	
	double Width_CurLane = 0; 	                          //�������ڳ������

	UINT Navi_LaneChg=4;                                  // ����Ҫ��Ļ�����0��ʾ��Ҫ�󻻵���1��ʾ���󻻵���2��ʾ���һ���
	UINT Navi_LaneChg_times=0;                            // ����Ҫ��Ļ�������

	Path_Obs Path_Obs_F;                                  // ��ǰ������ǰ���ϰ�����Ϣ
	Path_Obs Path_Obs_R;                                  // ��ǰ���������ϰ�����Ϣ
	Path_Obs Path_Obs_LF;                                 // ��ǰ������ǰ���ϰ�����Ϣ
	Path_Obs Path_Obs_LR;                                 // ��ǰ��������ϰ�����Ϣ
	Path_Obs Path_Obs_RF;                                 // ��ǰ������ǰ���ϰ�����Ϣ
	Path_Obs Path_Obs_RR;                                 // ��ǰ�����Һ��ϰ�����Ϣ	

 //   static WORD obsavoid_time = 0;                        // �ϰ�����ִ���
 //   static WORD no_obsaviod_time = 0;                     // �����У�ԭ��������û�г����ϰ������
 //   static WORD frontobs_time = 0;                        // �ϰ�����ֵĴ���

	Behavior_Dec Cur_Behavior;                            // ��ǰ��Ϊ���߽��
	Behavior_Dec His_Behavior;                            // ��ʷ֡��Ϊ���߽��
	
	// V2X��־λ��ʼ��
	bool pedestrian_flag = false;
	bool construction_flag = false;
	WORD V2XLight_flag = 0;

	// ������ʼ��
	memset(&Path_Obs_F, 0, sizeof(Path_Obs_F));
	memset(&Path_Obs_R, 0, sizeof(Path_Obs_R));
	memset(&Path_Obs_LF, 0, sizeof(Path_Obs_LF));
	memset(&Path_Obs_LR, 0, sizeof(Path_Obs_LR));
	memset(&Path_Obs_RF, 0, sizeof(Path_Obs_RF));
	memset(&Path_Obs_RR, 0, sizeof(Path_Obs_RR));

	memset(&Cur_Behavior, 0, sizeof(Cur_Behavior));
	memset(&His_Behavior, 0, sizeof(His_Behavior));

	// �ж������Ƿ���Ҫ����
	Nav_LaneChange(z_LocationOut, z_RoadNavi, Navi_LaneChg, Navi_LaneChg_times);

	// ���س�����Χ���·��	
	LoadRefPath(z_LocationOut, PathPoint_Front, PathPoint_Rear, PathPoint_LF, PathPoint_LR, PathPoint_RF, PathPoint_RR, Width_CurLane);
	
	// ����������Χ�ϰ���
	AroundObstacle(PathPoint_Front, PathPoint_Rear, PathPoint_LF, PathPoint_LR, PathPoint_RF, PathPoint_RR, Path_Obs_F, Path_Obs_R, Path_Obs_LF, Path_Obs_LR, Path_Obs_RF, Path_Obs_RR, Width_CurLane);
	
	// V2X�����жϺ����������Ӧ�ı�־λ����Ϊ�ж�����
	//V2XPedestrianJudge(z_LocationOut, z_V2XData, PathPoint_LongFront, pedestrian_flag); //�������˼��Ԥ��

	//V2XSignalLight(z_V2XData, V2XLight_flag); //�źŵ��·�

	//V2XConstructionEvent(z_LocationOut, z_V2XData, z_RSIWarningPointList, PathPoint_LongFront, construction_flag); //ʩ����·Ԥ��

	V2XEventDecision(z_LocationOut, z_V2XData, z_RSIWarningPointList, PathPoint_Front_Long, PathPoint_LF_Long, PathPoint_RF_Long, pedestrian_flag, V2XLight_flag, construction_flag);

	// ��Ϊ���߽����ֵ
	Cur_Behavior.behavior = z_behavior;
	Cur_Behavior.light_status = z_light_status;
	Cur_Behavior.target_lanenum = z_target_lanenum;
	Cur_Behavior.lanechg_status = z_segment_lanechg_status;        //  ����״̬
	Cur_Behavior.obsavoid_status = z_segment_obsavoid_status;      //  ����״̬
	Cur_Behavior.behavior_to_dlg = z_behavior_to_dlg;   

	His_Behavior.behavior = his_behavior;
	His_Behavior.light_status = his_light_status;
	His_Behavior.target_lanenum = his_target_lanenum;	

	// ��Ϊ�ж�	
	BehaviorDecision(z_LocationOut, z_RoadNavi, Navi_LaneChg, Navi_LaneChg_times, Width_CurLane, PathPoint_Front, Path_Obs_F, Path_Obs_R, Path_Obs_LF, Path_Obs_LR, Path_Obs_RF, Path_Obs_RR,His_Behavior,Cur_Behavior);

	// ���پ���		
	SpeedDecision(Cur_Behavior, z_refpath, z_velocity_expect);

	// �ο�·��
	RefPath(Cur_Behavior, PathPoint_Front, PathPoint_LF, PathPoint_RF, z_refpath);	

	// ��ǰ��Ϊ���߽��
	z_behavior = Cur_Behavior.behavior;
	z_light_status = Cur_Behavior.light_status;
	z_target_lanenum = Cur_Behavior.target_lanenum;
	z_segment_lanechg_status = Cur_Behavior.lanechg_status;
	z_segment_obsavoid_status = Cur_Behavior.obsavoid_status;
	z_behavior_to_dlg = Cur_Behavior.behavior_to_dlg;	
	z_target_roadnum = z_LocationOut.road_num;

}

/*************************************************************************************
�������ƣ�PreStubDecision()
����������Ԥ·����Ϊ����
���룺
�����
**************************************************************************************/
void CDecision::PreStubDecision()
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	// ������ʼ��
	WORD light_status;                                 // ת���
	vector<GlobalPoint2D> PathPoint_Front;             // ����ǰ��·��
	// ��ǰ����״̬
	BYTE RoadNum_Cur = z_LocationOut.road_num;         //�������ڵ�·���		
	BYTE LaneNum_Cur = z_LocationOut.lane_num;         //�������ڳ������
	BYTE LastRoadNum_Cur = z_LocationOut.last_roadnum; // ·����һ�ε�·���
	BYTE NextRoadNum_Cur = z_LocationOut.next_roadnum; // ·����һ�ε�·���
	BYTE LastLaneNum_Cur = z_LocationOut.last_lanenum; // ·����һ�ε�·�ĳ������
	BYTE NextLaneNum_Cur = z_LocationOut.next_lanenum; // ·����һ�ε�·�ĳ������	
	int Id_CurLane = z_LocationOut.id[LaneNum_Cur - 1];// �����ڵ�ǰ����·��ID
	
	// ·����������
	BOOL Obs_flag = false;                             // Ԥ·�ڼ�·���ϰ����־λ
	Obs_To_Veh Obs_Front;                              // Ԥ·�ڼ�·���ϰ����೵��λ����Ϣ
	WORD Obs_Pathid_F;                                 // �ϰ�����·����ID
	ObPoint ObPoint_F;                                 // �ϰ������꼰����
	
	// ·��·������
	WORD IdSum_CurLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1].size();   
	// ·��·������
	WORD IdSum_Inter = app->decision_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1].size();


	// ��Ԥ·��·��ѹ��������
	for (WORD i = Id_CurLane; i < IdSum_CurLane; i++)   
	{
		GlobalPoint2D pt;
		pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
		pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
		PathPoint_Front.push_back(pt);
	}

	// ��·��·��ѹ��������
	for (WORD j = 0;j < IdSum_Inter;j++)
	{
		GlobalPoint2D pt;
		pt.x = app->decision_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1][j].global_point.x;
		pt.y = app->decision_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1][j].global_point.y;
		PathPoint_Front.push_back(pt);
	}

	// ����Ԥ·�ڼ�·���ϰ���
	Obs_flag = SearchObstacle(PathPoint_Front, z_Obs, -0.5*Vehicle_Width, 0.5*Vehicle_Width, Obs_Front.dis_lat, Obs_Front.dis_lng, ObPoint_F, Obs_Pathid_F);

	// Ԥ·�ڳ��ٹ滮
	if (Obs_Front.dis_lng < 13) 
	{		
		z_velocity_expect = max(Obs_Front.dis_lng - 3, 0);    // ���Ϻ�Ԥ·�ڳ��پ���
		z_behavior_to_dlg = 13;                               // ���ɱ����·�������ϰ����������
	}
	else 
	{
		z_velocity_expect = 10;                   // Ԥ·������10
		z_behavior_to_dlg = 1;                    // ���ɱ����·�������ϰ���
	}

	// ת���
	if (z_RoadNavi[z_LocationOut.path_num].stub_attribute == 3) 
	{
		light_status = 1;                         // ��ת��
	}
	else 
	{ 
		light_status = z_RoadNavi[z_LocationOut.path_num].stub_attribute;
	}
	
	z_behavior = 1;                                 // ������·��Ϊ,��������
	z_target_roadnum = RoadNum_Cur;                 // ��ǰĿ���·
	z_target_lanenum = LaneNum_Cur;                 // ��ǰĿ�공��
	z_velocity_expect = z_velocity_expect;          // ��������
	z_refpath = PathPoint_Front;                    // ����ǰ���ο�·��
	z_light_status = light_status;                  // ����ǰ����ͨ��״̬

	return;
}
/*************************************************************************************
�������ƣ�StubDecision()
����������·����Ϊ����
���룺
�����
**************************************************************************************/
void CDecision::StubDecision()
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	// ������ʼ��
	WORD light_status;                                 // ת���
	vector<GlobalPoint2D> PathPoint_Front;             // ����ǰ��·��
	// ��ǰ����״̬
	BYTE RoadNum_Next = z_LocationOut.road_num;        // ������һ�ε�·���		
	BYTE LaneNum_Next = z_LocationOut.lane_num;        // ������һ�γ������
	BYTE LastRoadNum_Cur = z_LocationOut.last_roadnum; // ·����һ�ε�·���
	BYTE NextRoadNum_Cur = z_LocationOut.next_roadnum; // ·����һ�ε�·���
	BYTE LastLaneNum_Cur = z_LocationOut.last_lanenum; // ·����һ�ε�·�ĳ������
	BYTE NextLaneNum_Cur = z_LocationOut.next_lanenum; // ·����һ�ε�·�ĳ������
	int Id_Inter = z_LocationOut.id[LastLaneNum_Cur - 1];   // ·�ڳ�����ǰλ��ID

	// ·����������
	BOOL Obs_flag = false;                             // Ԥ·�ڼ�·���ϰ����־λ
	Obs_To_Veh Obs_Front;                              // Ԥ·�ڼ�·���ϰ����೵��λ����Ϣ
	WORD Obs_Pathid_F;                                 // �ϰ�����·����ID
	ObPoint ObPoint_F;                                 // �ϰ������꼰����

	// ·��·������
	WORD IdSum_Inter = app->decision_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1].size();
	// ��һ��·��·������
	WORD IdSum_NextLane = app->decision_MapData[RoadNum_Next - 1][LaneNum_Next - 1].size();

	//// ��·��·��ѹ��������
	//PathPoint_Front.clear();
	for (WORD j = Id_Inter;j < IdSum_Inter;j++)
	{
		GlobalPoint2D pt;
		pt.x = app->decision_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1][j].global_point.x;
		pt.y = app->decision_InterMapData[LastRoadNum_Cur - 1][NextRoadNum_Cur - 1][LastLaneNum_Cur - 1][NextLaneNum_Cur - 1][j].global_point.y;
		PathPoint_Front.push_back(pt);
	}
	// ����·��·��ѹ��������
	for (WORD i = 0;i < min(60, IdSum_NextLane);i++)
	{
			GlobalPoint2D pt;
			pt.x = app->decision_MapData[RoadNum_Next - 1][LaneNum_Next - 1][i].global_point.x;
			pt.y = app->decision_MapData[RoadNum_Next - 1][LaneNum_Next - 1][i].global_point.y;
			PathPoint_Front.push_back(pt);
	}

	// ����Ԥ·�ڼ�·���ϰ���
	Obs_flag = SearchObstacle(PathPoint_Front, z_Obs, -0.5*Vehicle_Width, 0.5*Vehicle_Width, Obs_Front.dis_lat, Obs_Front.dis_lng, ObPoint_F, Obs_Pathid_F);

	// Ԥ·�ڳ��ٹ滮
	if (Obs_Front.dis_lng < 13)
	{
		z_velocity_expect = max(Obs_Front.dis_lng - 3, 0);    // ���Ϻ�Ԥ·�ڳ��پ���
		z_behavior_to_dlg = 13;                               // ���ɱ����·�������ϰ����������
	}
	else
	{
		z_velocity_expect = 10;                   // Ԥ·������10
		z_behavior_to_dlg = 1;                    // ���ɱ����·�������ϰ���
	}

	// ת���
	if (z_RoadNavi[z_LocationOut.path_num].stub_attribute == 3)
	{
		light_status = 1;                         // ��ת��
	}
	else
	{
		light_status = z_RoadNavi[z_LocationOut.path_num].stub_attribute;
	}

	z_behavior = 1;                                 // ������·��Ϊ
	z_target_roadnum = RoadNum_Next;                // ��ǰĿ���·
	z_target_lanenum = LaneNum_Next;                // ��ǰĿ�공��
	z_velocity_expect = z_velocity_expect;          // ��������
	z_refpath = PathPoint_Front;                    // ����ǰ���ο�·��
	z_light_status = light_status;                  // ����ǰ����ͨ��״̬
	return;
}

/*************************************************
���ƣ�CalcNaviLaneChgTimes
���������㵼��Ҫ��ı������
���룺
roadnum_cur ��ǰ�������
outlanenum[LANESUM]��ǰ��·����·���г���
lanechgdir �������ͣ����һ���
�����
times ��������
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
				AfxMessageBox("CalcNaviLaneChgTimes����1����");
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
				AfxMessageBox("CalcNaviLaneChgTimes����2����");
			}
		}
	}
	else
	{
		AfxMessageBox("CalcNaviLaneChgTimes����3����");
	}
	return times;
}

/******************************************************************************
���ƣ�LoadRefPath
���������س�����Χ���·��
����:
     z_LocationOut ������λ��Ϣ
�����
     PathPoint_Front ��ǰ��·��
	 PathPoint_Rear  ����·��
	 PathPoint_LF    ��ǰ��·��
	 PathPoint_LR    ���·��
	 PathPoint_RF    ��ǰ��·��
	 PathPoint_RR    �Һ�·��
*******************************************************************************/
void CDecision::LoadRefPath(const LocationOut z_LocationOut, vector<GlobalPoint2D> &PathPoint_Front, vector<GlobalPoint2D> &PathPoint_Rear, vector<GlobalPoint2D> &PathPoint_LF, vector<GlobalPoint2D> &PathPoint_LR,
	vector<GlobalPoint2D> &PathPoint_RF, vector<GlobalPoint2D> &PathPoint_RR,double &Width_CurLane)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	WORD RoadNum_Cur = z_LocationOut.road_num;            // ������ǰ��·���		
	WORD LaneNum_Cur = z_LocationOut.lane_num;            // ������ǰ�������

	WORD Id_CurLane = z_LocationOut.id[LaneNum_Cur - 1];  // �����ڵ�ǰ�����ϵ�·��id���
	WORD IdSum_CurLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1].size();   	// �������ڳ�����·��id����																							
	WORD LaneSum = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][0].lane_sum;     // �������ڵ�·��������

	// ��ͼ�ɱ�����ԣ�0���ɱ����1��������2���ұ����3�����ұ��
	WORD LaneChg = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][Id_CurLane].lanechg_attribute;

	
	// double Width_CurLane = 0; 	     //�������ڳ������
	WORD Id_LeftLane = 0;            //�����ڳ�����೵���ϵ�id��ţ��������޳�������idΪ0 	
	WORD IdSum_LeftLane = 0;         //�����ڳ�����೵����·��id�������������޳�������idsumΪ0 
	double Width_LeftLane = 0;		 //�������ڳ�����೵����ȣ��������޳�����������СΪ0 
	WORD Id_RightLane = 0;			 //�������Ҳ೵���ϵ�id��ţ�����Ҳ��޳�������idΪ0 		
	WORD IdSum_RightLane = 0;        //�������Ҳ೵����·��id����������Ҳ��޳�������idsumΪ0 
	double Width_RightLane = 0; 	 //�������ڳ����Ҳ೵����ȣ�����Ҳ��޳�����������СΪ0 	

	// �������
	Width_CurLane=(app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][Id_CurLane].lane_width) / 100.0;     

	// ������ǰ��60m��Χ·������
	for (WORD i = min(IdSum_CurLane, Id_CurLane + ID_MORE); i < min(IdSum_CurLane, Id_CurLane + 120 + ID_MORE); i++)   //120���㣬���60m
	{
		GlobalPoint2D pt;
		pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
		pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
		PathPoint_Front.push_back(pt);                    // ����ǰ60m��Χ·��洢����ǰ·������
	}

	// ��������20m��Χ·������
	for (WORD i = min(IdSum_CurLane, Id_CurLane + ID_MORE);i > max(0, Id_CurLane + ID_MORE - 40);i--)
	{
		GlobalPoint2D pt;
		pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
		pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
		PathPoint_Rear.push_back(pt);                     // ������20m��Χ·��洢������·������
	}

	// ������ǰ�������·������
	if (LaneChg == 1||LaneChg==3)
	{
		// ������ͬ���г���ʱ
		if (LaneNum_Cur > 1)   // Ҫ����೵�����򳵵������Ҫ����1
		{
			Id_LeftLane = z_LocationOut.id[LaneNum_Cur - 2];                                   // �����ڵ�ǰ������೵��ID
			IdSum_LeftLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2].size();   // ��೵��·������
			if ((Id_LeftLane > 0) && (Id_LeftLane < IdSum_LeftLane))                           // ������೵�������������Ͷ�ͷ����
			{
				Width_LeftLane = (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][Id_LeftLane].lane_width) / 100.0;

				GlobalPoint2D pt;
				for (WORD i = min(IdSum_LeftLane, Id_LeftLane + ID_MORE); i < min(IdSum_LeftLane, Id_LeftLane + 120 + ID_MORE); i++)   // 120���㣬���60m
				{
					pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][i].global_point.x;
					pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][i].global_point.y;
					PathPoint_LF.push_back(pt);
				}
				for (int i = min(IdSum_LeftLane, Id_LeftLane + ID_MORE); i > max(0, Id_LeftLane + ID_MORE - 40); i--)                  // ��40���㣬���60m
				{
					pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][i].global_point.x;
					pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][i].global_point.y;
					PathPoint_LR.push_back(pt);
				}
			}
		}
		// ������ͬ��û�г��������ǿ������������ɲο�·��
		else
		{
			// ������ǰ�ο�·��
			PathPoint_LF = CreateNewPath(PathPoint_Front, -1 * Width_CurLane);
			// �������ο�·��
			PathPoint_LR = CreateNewPath(PathPoint_Rear, -1 * Width_CurLane);			
		}
	}

	// ������ǰ�����Һ�·������
	if (LaneChg == 2)
	{
		//�Ҳ�ͬ���г���
		if (LaneNum_Cur < LaneSum)       //Ҫ����೵�����򳵵������ҪС�ڳ�������
		{
			Id_RightLane = z_LocationOut.id[LaneNum_Cur];                                    // �������Ҳ೵����Ӧ��·��ID
			IdSum_RightLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur].size();    // �Ҳ೵��·������

			if ((Id_RightLane > 0) && (Id_RightLane < IdSum_RightLane))                      // �����Ҳ����������Ͷ�ͷ����
			{
				// �Ҳ೵�����
				Width_RightLane = (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur][Id_RightLane].lane_width) / 100.0;
				GlobalPoint2D pt;
				for (WORD i = min(IdSum_RightLane, Id_RightLane + ID_MORE); i < min(IdSum_RightLane, Id_RightLane + 120 + ID_MORE); i++)   //120���㣬���60m
				{
					pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur][i].global_point.x;
					pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur][i].global_point.y;
					PathPoint_RF.push_back(pt);
				}
				for (int i = min(IdSum_RightLane, Id_RightLane + ID_MORE); i > max(0, Id_RightLane + ID_MORE - 40); i--)   //120���㣬���60m
				{
					pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur][i].global_point.x;
					pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur][i].global_point.y;
					PathPoint_RR.push_back(pt);
				}
			}
		}
		// �Ҳ�ͬ��û�г���
		else
		{
			// ������ǰ�ο�·��
			PathPoint_RF = CreateNewPath(PathPoint_Front, Width_CurLane);
			// �����Һ�ο�·��
			PathPoint_RR = CreateNewPath(PathPoint_Rear,  Width_CurLane);
		}

	}
}

/*******************************************************************************
���ƣ�Nav_LaneChange
���������ݵ����ж������Ƿ���Ҫ����
���룺
z_LocationOut ��λ������
z_RoadNavi    ����������
�����
Navi_LaneChg  ����������־λ
Navi_LaneChg_Times ������������
*******************************************************************************/
void CDecision::Nav_LaneChange(const LocationOut z_LocationOut, const vector<PathInfo> z_RoadNavi, UINT &Navi_LaneChg, UINT &Navi_LaneChg_times)
{
	UINT RoadNum_Cur = z_LocationOut.road_num;            // ������ǰ��·���		
	UINT LaneNum_Cur = z_LocationOut.lane_num;            // ������ǰ�������
	
	UINT PathNum = z_LocationOut.path_num;                // �����ڵ���·���ϵı�ţ���0��ʼ	
	//UINT Navi_LaneChg = 4;                                // �����Ƿ�Ҫ������0����Ҫ��1��Ҫ���󻻵���2��Ҫ���һ���	
	//UINT Navi_LaneChg_times = 0;                          // ����Ҫ�����Ĵ���
	 

	// �ж���ǰ������ƥ���·�ڳ���
	for (UINT i = 0; ((i < LANESUM) && (z_RoadNavi[PathNum].out_lane_no[i] != 0)); i++)
	{
		if (LaneNum_Cur == z_RoadNavi[PathNum].out_lane_no[i])
		{
			Navi_LaneChg = 0;
			break;
		}
	}

	// edit 20191114
	WORD out_lane_min = 1;              // ��ǰ��ͨ�е���С�������
	WORD out_lane_max = 1;              // ��ǰ��ͨ�е���󳵵����
	out_lane_min = z_RoadNavi[PathNum].out_lane_no[0];
	for (int i = 0;i < LANESUM;i++)
	{
		if (z_RoadNavi[PathNum].out_lane_no[i] > out_lane_max)
		{
			out_lane_max = z_RoadNavi[PathNum].out_lane_no[i];         // �������Ŀ�ͨ�г���
		}
	}


	// �ж�����Ҫ���������������ұ��
	if (Navi_LaneChg == 4)   //����Ҫ�󻻵�����
	{
		// if (LaneNum_Cur < z_RoadNavi[PathNum].out_lane_no[0])        // ǰ���ǵ�����·�ڵĳ������Ӧ��������
		if (LaneNum_Cur < out_lane_min)
		{
			Navi_LaneChg = 2;      //����Ҫ���һ���    
			Navi_LaneChg_times = CalcNaviLaneChgTimes(LaneNum_Cur, z_RoadNavi[PathNum].out_lane_no, 2);
		}
		//else if (LaneNum_Cur > z_RoadNavi[PathNum].out_lane_no[0])   // ǰ���ǵ�����·�ڵĳ������Ӧ��������
		else if (LaneNum_Cur > out_lane_max)
		{
			Navi_LaneChg = 1;     //����Ҫ���󻻵�
			Navi_LaneChg_times = CalcNaviLaneChgTimes(LaneNum_Cur, z_RoadNavi[PathNum].out_lane_no, 1);
		}
		else
		{
			Navi_LaneChg = 0;      //������������˴�˵��������
		}
	}
}

/********************************************************************
���ƣ�AroundObstacle
��������ȡ������Χ�ο�·���ϰ�����Ϣ
���룺
PathPoint_F           // ��ǰ����ǰ���ϰ���
PathPoint_R           // ��ǰ�������ϰ���
PathPoint_LF          // ��ǰ������ǰ���ϰ���
PathPoint_LR          // ��ǰ��������ϰ���
PathPoint_RF          // ��ǰ������ǰ���ϰ���
PathPoint_RR          // ��ǰ�����Һ��ϰ���
Width_CurLane         // �������
�����
Path_Obs_F            // ��ǰ����ǰ���ϰ�����Ϣ
Path_Obs_R            // ��ǰ�������ϰ�����Ϣ
Path_Obs_LF           // ��ǰ������ǰ���ϰ�����Ϣ
Path_Obs_LR           // ��ǰ��������ϰ�����Ϣ
Path_Obs_RF           // ��ǰ������ǰ���ϰ�����Ϣ
Path_Obs_RR           // ��ǰ�����Һ��ϰ�����Ϣ
********************************************************************/
void CDecision::AroundObstacle(const vector<GlobalPoint2D>PathPoint_F, const vector<GlobalPoint2D>PathPoint_R, const vector<GlobalPoint2D>PathPoint_LF, const vector<GlobalPoint2D>PathPoint_LR,
	const vector<GlobalPoint2D>PathPoint_RF, const vector<GlobalPoint2D>PathPoint_RR, Path_Obs &Path_Obs_F, Path_Obs &Path_Obs_R, Path_Obs &Path_Obs_LF, Path_Obs &Path_Obs_LR,
	Path_Obs &Path_Obs_RF, Path_Obs &Path_Obs_RR, const double Width_CurLane)
{
	
	bool Obs_flag_F = false;         // ǰ���ϰ����־λ
	Obs_To_Veh Obs_Pose_F;           // ������ǰ���ϰ���λ��
	WORD Obs_Pathid_F=0;               // ǰ���ϰ���ID
	ObPoint ObAttr_F;                // ǰ���ϰ�������

	bool Obs_flag_R = false;         // ���ϰ����־λ
	Obs_To_Veh Obs_Pose_R;           // ���������ϰ���λ��
	WORD Obs_Pathid_R=0;               // ���ϰ���ID
	ObPoint ObAttr_R;                // ���ϰ�������

	bool Obs_flag_LF = false;         // ��ǰ���ϰ����־λ
	Obs_To_Veh Obs_Pose_LF;           // ������ǰ���ϰ���λ��
	WORD Obs_Pathid_LF=0;               // ��ǰ���ϰ���ID
	ObPoint ObAttr_LF;                // ��ǰ���ϰ�������

	bool Obs_flag_LR = false;         // ����ϰ����־λ
	Obs_To_Veh Obs_Pose_LR;           // ��������ϰ���λ��
	WORD Obs_Pathid_LR=0;               // ����ϰ���ID
	ObPoint ObAttr_LR;                // ����ϰ�������

	bool Obs_flag_RF = false;         // ��ǰ���ϰ����־λ
	Obs_To_Veh Obs_Pose_RF;           // ������ǰ���ϰ���λ��
	WORD Obs_Pathid_RF=0;               // ��ǰ���ϰ���ID
	ObPoint ObAttr_RF;                // ��ǰ���ϰ�������

	bool Obs_flag_RR = false;         // �Һ��ϰ����־λ
	Obs_To_Veh Obs_Pose_RR;           // �����Һ��ϰ���λ��
	WORD Obs_Pathid_RR=0;               // �Һ��ϰ���ID
	ObPoint ObAttr_RR;                // �Һ��ϰ�������
	
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

	// ������ǰ���ϰ�����Ϣ
	if (PathPoint_F.size() != 0)
	{
		Obs_flag_F = SearchObstacle(PathPoint_F, z_Obs, -0.5*Vehicle_Width, 0.5*Vehicle_Width, Obs_Pose_F.dis_lat, Obs_Pose_F.dis_lng, ObAttr_F, Obs_Pathid_F);
	}

	// ���������ϰ�����Ϣ
	if (PathPoint_R.size() != 0)
	{
		Obs_flag_R= SearchObstacle(PathPoint_R, z_Obs, -0.5*Vehicle_Width, 0.5*Vehicle_Width, Obs_Pose_R.dis_lat, Obs_Pose_R.dis_lng, ObAttr_R, Obs_Pathid_R);
	}

	// ������ǰ���ϰ�����Ϣ
	if (PathPoint_LF.size() != 0)
	{
		Obs_flag_LF= SearchObstacle(PathPoint_LF, z_Obs, -0.5*Vehicle_Width, 0.5*Width_CurLane, Obs_Pose_LF.dis_lat, Obs_Pose_LF.dis_lng, ObAttr_LF, Obs_Pathid_LF);
	}


	// ��������ϰ�����Ϣ
	if (PathPoint_LR.size() != 0)
	{
		Obs_flag_LR = SearchObstacle(PathPoint_LR, z_Obs, -0.5*Vehicle_Width, 0.5*Width_CurLane, Obs_Pose_LR.dis_lat, Obs_Pose_LR.dis_lng, ObAttr_LR, Obs_Pathid_LR);
	}

	// �����Һ��ϰ�����Ϣ
	if (PathPoint_RF.size() != 0)
	{
		Obs_flag_RF = SearchObstacle(PathPoint_RF, z_Obs, -0.5*Width_CurLane, 0.5*Vehicle_Width, Obs_Pose_RF.dis_lat, Obs_Pose_RF.dis_lng, ObAttr_RF, Obs_Pathid_RF);
	}

	// �����Һ��ϰ�����Ϣ
	if (PathPoint_RR.size() != 0)
	{
		Obs_flag_RR = SearchObstacle(PathPoint_RR, z_Obs, -0.5*Width_CurLane, 0.5*Vehicle_Width, Obs_Pose_RR.dis_lat, Obs_Pose_RR.dis_lng, ObAttr_RR, Obs_Pathid_RR);
	}


	// ������ǰ����ǰ���ϰ�����Ϣ
	Path_Obs_F.Ob_Pose = Obs_Pose_F;
	Path_Obs_F.Obs_flag = Obs_flag_F;
	Path_Obs_F.Ob_Pathid = Obs_Pathid_F;
	Path_Obs_F.Ob_Attr = ObAttr_F;

	// ������ǰ�������ϰ�����Ϣ
	Path_Obs_R.Ob_Pose = Obs_Pose_R;
	Path_Obs_R.Obs_flag = Obs_flag_R;
	Path_Obs_R.Ob_Pathid = Obs_Pathid_R;
	Path_Obs_R.Ob_Attr = ObAttr_R;

	// ������೵��ǰ���ϰ�����Ϣ
	Path_Obs_LF.Ob_Pose = Obs_Pose_LF;
	Path_Obs_LF.Obs_flag = Obs_flag_LF;
	Path_Obs_LF.Ob_Pathid = Obs_Pathid_LF;
	Path_Obs_LF.Ob_Attr = ObAttr_LF;

	// ������೵�����ϰ�����Ϣ
	Path_Obs_LR.Ob_Pose = Obs_Pose_LR;
	Path_Obs_LR.Obs_flag = Obs_flag_LR;
	Path_Obs_LR.Ob_Pathid = Obs_Pathid_LR;
	Path_Obs_LR.Ob_Attr = ObAttr_LR;

	// �����Ҳ೵��ǰ���ϰ�����Ϣ
	Path_Obs_RF.Ob_Pose = Obs_Pose_RF;
	Path_Obs_RF.Obs_flag = Obs_flag_RF;
	Path_Obs_RF.Ob_Pathid = Obs_Pathid_RF;
	Path_Obs_RF.Ob_Attr = ObAttr_RF;

	// �����Ҳ೵�����ϰ�����Ϣ
	Path_Obs_RR.Ob_Pose = Obs_Pose_RR;
	Path_Obs_RR.Obs_flag = Obs_flag_RR;
	Path_Obs_RR.Ob_Pathid = Obs_Pathid_RR;
	Path_Obs_RR.Ob_Attr = ObAttr_RR;
}

/*********************************************************************************************
���ƣ�BehaviorDecision
��������Ϊ�������ж�
���룺
z_LocationOut   ��λ���
z_RoadNavi      �������
Path_Obs_F      ��ǰ������ǰ���ϰ�����Ϣ
Path_Obs_R      ��ǰ���������ϰ�����Ϣ
Path_Obs_LF     ��ǰ������ǰ���ϰ�����Ϣ
Path_Obs_LR     ��ǰ��������ϰ�����Ϣ
Path_Obs_RF     ��ǰ������ǰ���ϰ�����Ϣ
Path_Obs_RR     ��ǰ�����Һ��ϰ�����Ϣ
�����
behavior
**********************************************************************************************/
void CDecision::BehaviorDecision(const LocationOut z_LocationOut, const vector<PathInfo> z_RoadNavi, const UINT Navi_LaneChg, const UINT Navi_LaneChg_times,const double Width_CurLane, const vector<GlobalPoint2D>PathPoint_Front, const Path_Obs &Path_Obs_F, const Path_Obs &Path_Obs_R, const Path_Obs &Path_Obs_LF,
	const Path_Obs &Path_Obs_LR, const Path_Obs &Path_Obs_RF, const Path_Obs &Path_Obs_RR, const Behavior_Dec His_Behavior, Behavior_Dec &Cur_Behavior)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();


	WORD RoadNum_Cur = z_LocationOut.road_num;            // ������ǰ��·���		
	WORD LaneNum_Cur = z_LocationOut.lane_num;            // ������ǰ�������														  
	WORD PathNum = z_LocationOut.path_num;                // �����ڵ���·���ϵı�ţ���0��ʼ

	WORD Id_CurLane = z_LocationOut.id[LaneNum_Cur - 1];  // �����ڵ�ǰ�����ϵ�·��id���
	WORD IdSum_CurLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1].size();   	// �������ڳ�����·��id����																							
	WORD LaneSum = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][0].lane_sum;     // �������ڵ�·��������

	// ��ͼ�ɱ�����ԣ�0���ɱ����1��������2���ұ����3�����ұ��
	WORD LaneChg_Map = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][Id_CurLane].lanechg_attribute;
	
	static UINT obsavoid_time = 0;                          // �ϰ�����ֵĴ���
	static UINT no_obsaviod_time = 0;                       // ���Ϲ������ϰ�����ʧ����
	static UINT frontobs_time = 0;                          // �ϰ�����ֵĴ���
	
	// ��ͼ���Բ��ɱ��
	if (LaneChg_Map == 0)
	{		
		if (Path_Obs_F.Ob_Pose.dis_lng < 15)
		{
			no_obsaviod_time = 0;
			obsavoid_time++;
		
			Obs_To_Veh Path_Obs_F_L;        // ͬ����ǰ�����		 
			Obs_To_Veh Path_Obs_F_R;        // ͬ����ǰ���Ҳ�
			WORD Obs_Pathid_F;              // ͬ����ǰ�������ϰ���ID
			ObPoint ObPoint_F;              // ͬ����ǰ�������ϰ�������

			//static bool left_flag = false;     // �����ϱ�־λ
			//static bool right_flag = false;    // �����ϱ�־λ

			// ǰ�����������ϰ���
			if (obsavoid_time > 2)
			{
				vector<GlobalPoint2D> newpath;
				bool left_flag = false;     // �����ϱ�־λ
				for (BYTE i = 0; i < (Width_CurLane - Vehicle_Width) / 0.6; i++)
				{
					newpath = CreateNewPath(PathPoint_Front, -0.3*i);
					SearchObstacle(newpath, z_Obs, -0.5*Vehicle_Width, 0.5*Vehicle_Width, Path_Obs_F_L.dis_lat, Path_Obs_F_L.dis_lng, ObPoint_F, Obs_Pathid_F);
					if (Path_Obs_F_L.dis_lng > 25)
					{
						Cur_Behavior.behavior = 4;                                  // ������
						Cur_Behavior.target_lanenum= LaneNum_Cur;                   // Ŀ�공�����
						Cur_Behavior.light_status = 1;                              // ��ת��
						Cur_Behavior.obsavoid_status = 1;                           // ���Ϲ�����
						Cur_Behavior.behavior_to_dlg = 11;                          // ���ɱ����·�������ϰ���������ϣ�������
						left_flag = true;                                           // �����ϱ�־λ��true
						break;
					}
				}
				
				bool right_flag = false;    // �����ϱ�־λ
				if (left_flag == false)
				{
					for (BYTE i = 0; i < (Width_CurLane - Vehicle_Width) / 0.6; i++)
					{
						newpath = CreateNewPath(PathPoint_Front, 0.3*i);
						SearchObstacle(newpath, z_Obs, -0.5*Vehicle_Width, 0.5*Vehicle_Width, Path_Obs_F_R.dis_lat, Path_Obs_F_R.dis_lng, ObPoint_F, Obs_Pathid_F);
						if (Path_Obs_F_R.dis_lng > 25)
						{							
							Cur_Behavior.behavior = 5;                                  // ������
							Cur_Behavior.target_lanenum = LaneNum_Cur;                  // Ŀ�공�����
							Cur_Behavior.light_status = 2;                              // ��ת��
							Cur_Behavior.obsavoid_status = 1;                           // ���Ϲ�����
							Cur_Behavior.behavior_to_dlg = 12;                          // ���ɱ����·�������ϰ���������ϣ�������
							right_flag = true;                                          // �����ϱ�־λ��true
							break;
						}
					}
				}	
			}		
			// ǰ���������ϰ���ʱ
			else
			{
				Cur_Behavior.behavior = 1;                                  // ��������
				Cur_Behavior.target_lanenum = LaneNum_Cur;                  // Ŀ�공�����
				Cur_Behavior.light_status = 0;                              // �ر�ת���				
				Cur_Behavior.behavior_to_dlg = 1;                           // ���ɱ����·�������ϰ���������ϣ�������				
			}
		}
		else
		{
			if (z_segment_obsavoid_status == 0)
			{

				Cur_Behavior.behavior = 1;                                  // ��������
				Cur_Behavior.target_lanenum = LaneNum_Cur;                  // Ŀ�공�����
				Cur_Behavior.light_status = 0;                              // �ر�ת���				
				Cur_Behavior.behavior_to_dlg = 1;                           // ���ɱ����·�������ϰ���������ϣ�������	
								
			}
			else
			{
				no_obsaviod_time++;
				if (no_obsaviod_time > 3)
				{
					Cur_Behavior.behavior = 1;
					Cur_Behavior.target_lanenum = LaneNum_Cur;                  // Ŀ�공�����
					Cur_Behavior.light_status = 0;                              // �ر�ת���				
					Cur_Behavior.behavior_to_dlg = 1;                           // ���ɱ����·�������ϰ���������ϣ�������
					Cur_Behavior.obsavoid_status = 0;
				}
			}
			Cur_Behavior.behavior_to_dlg = 1;                                   //���ɱ����·�������ϰ���
		}
	}
	// ��ͼ���Կɱ��
	else
	{
		no_obsaviod_time = 0;
		obsavoid_time = 0;

		// �Ǳ��������
		if (Cur_Behavior.lanechg_status==0)
		{
			// �����л�������
			if (Navi_LaneChg != 0)
			{
				// ����Ҫ������໻��
				if (Navi_LaneChg == 1)
				{
					// ��ͼ��������໻��
					if (LaneChg_Map == 1 || LaneChg_Map == 3)
					{
						Cur_Behavior.behavior_to_dlg = 2;
						if (Cur_Behavior.light_status != 1)
						{
							Cur_Behavior.light_status = 1;
							leftlight_time = 0;
						}
						leftlight_time += z_period_last;

						// ����Ҫ����໻�����ж���໻�������Ƿ�����
						if ((Path_Obs_LF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10) || (Path_Obs_LF.Ob_Pose.dis_lng > 40))
						{
							// �ж�������������Ծ���
							if (Path_Obs_LR.Ob_Pose.dis_lng > 15)
							{
								// ת���ʱ������ʱ
								if (leftlight_time > 2000)
								{
									Cur_Behavior.behavior = 2;                       // ���󻻵�
									Cur_Behavior.target_lanenum = LaneNum_Cur - 1;   // Ŀ�공��
									Cur_Behavior.lanechg_status = 1;                 // ����������
								}
								// ת���ʱ��������ʱ
								else
								{
									Cur_Behavior.behavior = 1;                     // ���ֵ�ǰ����
									Cur_Behavior.target_lanenum = LaneNum_Cur;     // Ŀ�공��
									Cur_Behavior.lanechg_status = 0;               // �ǻ���������
								}

							}
							// ��೵���󷽵��۲�����
							else
							{
								Cur_Behavior.behavior = 1;                     // ���ֵ�ǰ����
								Cur_Behavior.target_lanenum = LaneNum_Cur;     // Ŀ�공��
								Cur_Behavior.lanechg_status = 0;               // �ǻ���������
							}
						}
						// ��೵����������
						else
						{
							Cur_Behavior.behavior = 1;                     // ���ֵ�ǰ����
							Cur_Behavior.target_lanenum = LaneNum_Cur;     // Ŀ�공��
							Cur_Behavior.lanechg_status = 0;               // �ǻ���������
						}

					}
					// ��ͼ���������󻻵�
					else
					{
						Cur_Behavior.behavior = 1;                     // ���ֵ�ǰ����
						Cur_Behavior.target_lanenum = LaneNum_Cur;     // Ŀ�공��
						Cur_Behavior.lanechg_status = 0;               // �ǻ���������
						Cur_Behavior.behavior_to_dlg = 4;              // ����Ҫ��������������Բ�ƥ��
					}
				}
				// ����Ҫ�����Ҳ໻��
				else if (Navi_LaneChg == 2)
				{
					if (LaneChg_Map == 2 || LaneChg_Map == 3)
					{
						Cur_Behavior.behavior_to_dlg = 3;    // ����Ҫ���ұ�������ұ��	
						// ��ת�Ƽ�ʱ
						if (Cur_Behavior.light_status != 2)
						{
							Cur_Behavior.lanechg_status = 2;
							rightlight_time = 0;
						}
						rightlight_time += z_period_last;

						// ���Ҳ໻��
						if ((Path_Obs_RF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10) || (Path_Obs_RF.Ob_Pose.dis_lng > 40))
						{
							// �жϺ���������Ծ���
							if (Path_Obs_RR.Ob_Pose.dis_lng > 15)    //����������,��ǰ�����ϰ�����ϰ���������15
							{
								// ת���ʱ������
								if (rightlight_time >= 2000)
								{
									Cur_Behavior.behavior = 3;                     // �ұ��
									Cur_Behavior.target_lanenum = LaneNum_Cur + 1;   // Ŀ�공��						
									Cur_Behavior.lanechg_status = 1;               // ���״̬��־λ							
								}
								// ת���ʱ��������
								else
								{
									Cur_Behavior.behavior = 1;                       // ��������
									Cur_Behavior.target_lanenum = LaneNum_Cur;       // Ŀ�공��
									Cur_Behavior.lanechg_status = 0;                 // �ǻ���������

								}
							}
							else 
							{
								Cur_Behavior.behavior = 1;                        // ��������
								Cur_Behavior.target_lanenum = LaneNum_Cur;        // Ŀ�공��
								Cur_Behavior.lanechg_status = 0;                  // �ǻ���������
							}
						}
						else
						{
							Cur_Behavior.behavior = 1;                       // ��������
							Cur_Behavior.target_lanenum = LaneNum_Cur;       // Ŀ�공��
							Cur_Behavior.lanechg_status = 0;                 // �ǻ���������
						}
					}
					else
					{
						Cur_Behavior.behavior = 1;                       // ��������
						Cur_Behavior.target_lanenum = LaneNum_Cur;       // Ŀ�공��
						Cur_Behavior.lanechg_status = 0;                 // �ǻ���������
						Cur_Behavior.behavior_to_dlg = 4;             // ����Ҫ��������������Բ�ƥ��
					}
				}
												
			}
			// ����û�л�������
			else
			{
				// ǰ����ʱ������ϰ���
				if (Path_Obs_F.Ob_Pose.dis_lng < (2 * 10 + 5))
				{
					frontobs_time++;
					// ǰ����ʱ������ϰ���
					if (frontobs_time > 2)
					{
						frontobs_time = 3;
						// ��ͼ��������
						if (LaneChg_Map == 1)
						{
							// �����ͬ�򳵵�
							if(LaneNum_Cur>1)
							{
								Cur_Behavior.behavior_to_dlg = 5;  // ������Ҫ������ǰ�����ϰ��������						

								// �ж��Ƿ���Ҫ����
								bool no_back_flag = true;    // �������Ƿ���Ҫ���أ�true����Ҫ�������false������Ҫ�����
								for (BYTE i = 0; i < LANESUM && z_RoadNavi[PathNum].out_lane_no[i] != 0; i++)
								{
									if (z_RoadNavi[PathNum].out_lane_no[i] == LaneNum_Cur - 1)
									{
										no_back_flag = false;
									}
								}

								// �ж���ͼ�Ƿ�������
								bool chg_condition_flag = false;          // false ��ʾ�����֮���ܻ�����true ��ʾ���㹻�ռ䷵��							
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

									// �����ؿռ����������²Żỻ��
									if (dis_chg1 > 60)     //������dis_chg2�������쵽�Ƿ�����һ�οɱ�����򣬶����ǵ�Ԥ·����
									{
										chg_condition_flag = true;    // �����������ܷ���
										if (z_light_status != 1)
										{
											z_light_status = 1;
											leftlight_time = 0;
										}
										leftlight_time += z_period_last;
										if (leftlight_time > 2000)         //����ת��2s
										{
											leftlight_time = 2000;
										}
									}
									else
									{
										z_light_status = 0;          // �ر�ת���      
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
										if (leftlight_time > 2000)                          //����ת��2s
										{
											leftlight_time = 2100;
										}
									}
									else
									{
										Cur_Behavior.light_status = 0; 
									}
								}

								// �ж�Ŀ�공���Ƿ�������
								if (chg_condition_flag == true)
								{
									// ��ǰ���ռ��Ƿ�����������
									if (Path_Obs_LF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10)
									{
										if (Path_Obs_LR.Ob_Pose.dis_lng > 10)
										{
											// ״̬��ʱ�����㻻��
											if (leftlight_time > 1500)
											{
												frontobs_time = 0;                              // �ϰ�����ִ�������
												Cur_Behavior.behavior = 2;                      // ����
												Cur_Behavior.target_lanenum = LaneNum_Cur - 1;  // Ŀ�공��
												Cur_Behavior.lanechg_status = 1;                // ȷ�������־λ
											}
											// ״̬��ʱ�������㻻��
											else
											{
												Cur_Behavior.behavior = 1;                      // ��������
												Cur_Behavior.target_lanenum = LaneNum_Cur;      // Ŀ�공��
												Cur_Behavior.lanechg_status = 0;                // �������������־λ
											}
										}
										else
										{
											Cur_Behavior.behavior = 1;                          // ��������
											Cur_Behavior.target_lanenum = LaneNum_Cur;          // Ŀ�공��
											Cur_Behavior.lanechg_status = 0;                    // �������������־λ
										}
									}
									else
									{
										Cur_Behavior.behavior = 1;                          // ��������
										Cur_Behavior.target_lanenum = LaneNum_Cur;          // Ŀ�공��
										Cur_Behavior.lanechg_status = 0;                    // �������������־λ
									}
								}
								else
								{
									Cur_Behavior.behavior = 1;                      // ��������
									Cur_Behavior.target_lanenum = LaneNum_Cur;      // Ŀ�공��
									Cur_Behavior.lanechg_status = 0;                // �������������־λ
								}
							}
							// ���û��ͬ�򳵵�
							else
							{
								Cur_Behavior.behavior = 1;                      // ��������
								Cur_Behavior.target_lanenum = LaneNum_Cur;      // Ŀ�공��
								Cur_Behavior.lanechg_status = 0;                // �������������־λ								
							}												

						}
						// ��ͼ�����ұ�����Ҳ����ͬ�򳵵�
						else if (LaneChg_Map == 2)
						{
							// �Ҳ���ͬ�򳵵�
							if (LaneNum_Cur < LaneSum)
							{
								Cur_Behavior.behavior_to_dlg = 6;    //������Ҫ������ǰ�����ϰ�����ұ��

								// �ж��Ƿ���Ҫ������
								bool no_back_flag = true;   //�������Ƿ���Ҫ���أ�true����Ҫ�������false������Ҫ�����
								for (BYTE i = 0; i < LANESUM && z_RoadNavi[PathNum].out_lane_no[i] != 0; i++)
								{
									if (z_RoadNavi[PathNum].out_lane_no[i] == LaneNum_Cur - 1)
										no_back_flag = false;
								}

								// �ж���ͼʣ�೤���Ƿ�������
								bool chg_condition_flag = false;  //�������������Ҫ��������򿴻���û���㹻�ľ����ñ������û�еĻ����ܱ��
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
									// ʣ�೤���Ƿ�������
									if (dis_chg1 > 50)
									{
										chg_condition_flag = true;
										if (Cur_Behavior.light_status != 2)
										{
											Cur_Behavior.light_status = 2;
											leftlight_time = 0;
										}
										leftlight_time += z_period_last;
										if (leftlight_time > 2000)                          //����ת��2s
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
										if (leftlight_time > 2000)                          //����ת��2s
										{
											leftlight_time = 2000;
										}
									}
								}

								// �ж�Ŀ�공���ռ��Ƿ�������
								if (chg_condition_flag == true)
								{
									// Ŀ�공��ǰ���ϰ������㻻��
									if (Path_Obs_RF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10)
									{
										// Ŀ�공���󷽾������㻻��
										if (Path_Obs_RR.Ob_Pose.dis_lng > 10)    //����������,��ǰ�����ϰ�����ϰ���������15
										{
											// ת���ʱ������
											if (leftlight_time > 1500)
											{
												frontobs_time = 0;
												Cur_Behavior.behavior = 3;                       // �ұ��
												Cur_Behavior.target_lanenum = LaneNum_Cur + 1;   // Ŀ�공��
												Cur_Behavior.lanechg_status = 1;                 // ������־λ										
											}
											// ת���ʱ��������
											else
											{
												Cur_Behavior.behavior = 1;                       // ��������
												Cur_Behavior.target_lanenum = LaneNum_Cur;      // Ŀ�공��
												Cur_Behavior.lanechg_status = 0;                 // ������־λ	
											}
										}
										// Ŀ�공���󷽾��벻������
										else
										{
											Cur_Behavior.behavior = 1;                       // ��������
											Cur_Behavior.target_lanenum = LaneNum_Cur;       // Ŀ�공��
											Cur_Behavior.lanechg_status = 0;                 // ������־λ	
										}
									}
									// Ŀ�공��ǰ���ϰ��ﲻ���㻻��
									else
									{
										Cur_Behavior.behavior = 1;                       // ��������
										Cur_Behavior.target_lanenum = LaneNum_Cur;       // Ŀ�공��
										Cur_Behavior.lanechg_status = 0;                 // ������־λ	
									}
								}
								else
								{
									Cur_Behavior.behavior = 1;                       // ��������
									Cur_Behavior.target_lanenum = LaneNum_Cur;       // Ŀ�공��
									Cur_Behavior.lanechg_status = 0;                 // ������־λ
								}
							}
							else
							{
								Cur_Behavior.behavior = 1;                       // ��������
								Cur_Behavior.target_lanenum = LaneNum_Cur;       // Ŀ�공��
								Cur_Behavior.lanechg_status = 0;                 // ������־λ
							}
							
						}
						// ��ͼ�������ұ��						
						else if (LaneChg_Map == 3)
						{
							z_behavior_to_dlg = 7;          //������Ҫ������ǰ�����ϰ�������ұ��

							// �ж�����໻���Ƿ���Ҫ����ԭ����
							bool no_back_leftchg_flag = true;   //�󻻵����Ƿ���Ҫ���أ�true����Ҫ�������false������Ҫ�����
							for (BYTE i = 0; i < LANESUM && z_RoadNavi[PathNum].out_lane_no[i] != 0; i++)
							{
								if (z_RoadNavi[PathNum].out_lane_no[i] == LaneNum_Cur - 1)
								{
									no_back_leftchg_flag = false;
								}

							}

							// �ж����Ҳ໻���Ƿ���Ҫ����ԭ����
							bool no_back_rightchg_flag = true;   //�һ������Ƿ���Ҫ���أ�true����Ҫ�������false������Ҫ�����
							for (BYTE i = 0; i < LANESUM && z_RoadNavi[PathNum].out_lane_no[i] != 0; i++)
							{
								if (z_RoadNavi[PathNum].out_lane_no[i] == LaneNum_Cur + 1)
								{
									no_back_rightchg_flag = false;
								}

							}

							// �ж�����໻��ʣ�೤���Ƿ����㻻������
							bool leftchg_condition_flag = false;  //�������������Ҫ��������򿴻���û���㹻�ľ����ñ������û�еĻ����ܱ��
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

									if (dis_chg1 > 50)     //������dis_chg2�������쵽�Ƿ�����һ�οɱ�����򣬶����ǵ�Ԥ·����
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
							
							

							// �ж����Ҳ໻��ʣ�೤���Ƿ����㻻������
							bool rightchg_condition_flag = false;  //������ұ������Ҫ��������򿴻���û���㹻�ľ����ñ������û�еĻ����ܱ��
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
									if (dis_chg1 > 50)     // ʣ�����
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
							

							// ����Ҫ����ԭ�����ҵ�ͼʣ��������㻻��
							if (leftchg_condition_flag == true && no_back_leftchg_flag == false)  //����������û����ҿ���������������
							{
								if (Cur_Behavior.lanechg_status != 1)
								{
									Cur_Behavior.light_status = 1;
									leftlight_time = 0;
								}
								leftlight_time += z_period_last;
								if (leftlight_time > 2000)                          //����ת��2s
								{
									leftlight_time = 2000;
								}

								if (Path_Obs_LF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10)
								{
									// Ŀ�공���󷽾������㻻��
									if (Path_Obs_LR.Ob_Pose.dis_lng > 10)    //����������,��ǰ�����ϰ�����ϰ���������10
									{
										// ת���ʱ�����㻻������
										if (leftlight_time > 2000)
										{
											frontobs_time = 0;
											Cur_Behavior.behavior = 2;                        // ����
											Cur_Behavior.target_lanenum = LaneNum_Cur - 1;    // Ŀ�공��
											Cur_Behavior.lanechg_status = 1;                  // ����״̬

										}
										// ת���ʱ�������㻻������
										else
										{
											Cur_Behavior.behavior = 1;                         // ��������
											Cur_Behavior.target_lanenum = LaneNum_Cur;         // Ŀ�공�� 
											Cur_Behavior.lanechg_status = 0;
										}
									}
									// Ŀ�공���󷽾��벻���㻻��
									else
									{
										Cur_Behavior.behavior = 1;                          // ��������
										Cur_Behavior.target_lanenum = LaneNum_Cur;          // Ŀ�공��	
										Cur_Behavior.lanechg_status = 0;
									}
								}
								else
								{
									Cur_Behavior.behavior = 1;                              // ��������
									Cur_Behavior.target_lanenum = LaneNum_Cur;              // Ŀ�공��		
									Cur_Behavior.lanechg_status = 0;
								}
							}
							// ����Ҫ����ԭ�����ҵ�ͼʣ��������㻻��
							else if (rightchg_condition_flag == true && no_back_rightchg_flag == false)  //����ұ�����û��������������㣬���ұ��
							{
								if (Cur_Behavior.light_status != 2)
								{
									Cur_Behavior.light_status = 2;
									leftlight_time = 0;
								}
								leftlight_time += z_period_last;
								if (leftlight_time > 2000)                          //����ת��2s
								{
									leftlight_time = 2000;
								}
								// Ŀ�공��ǰ�����㻻������
								if (Path_Obs_RF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10)
								{
									// Ŀ�공�������㻻������
									if (Path_Obs_RR.Ob_Pose.dis_lng > 10)    //����������,��ǰ�����ϰ�����ϰ���������15
									{
										// ת���ʱ�����㻻������
										if (leftlight_time > 2000)
										{
											frontobs_time = 0;
											Cur_Behavior.behavior = 3;                      // �ұ��
											Cur_Behavior.target_lanenum = LaneNum_Cur + 1;  // Ŀ�공��
											Cur_Behavior.lanechg_status = 1;                // ����������											
										}
										// ת���ʱ�������㻻������
										else
										{
											Cur_Behavior.behavior = 1;
											Cur_Behavior.target_lanenum = LaneNum_Cur;
										}
									}
									// Ŀ�공���󷽲����㻻������
									else
									{
										Cur_Behavior.behavior = 1;
										Cur_Behavior.target_lanenum = LaneNum_Cur;
									}
								}
							}
							// ��ͼʣ��������㻻������
							else if (leftchg_condition_flag == true)  //��������Ҫ��������������
							{
								if (Cur_Behavior.light_status != 1)
								{
									Cur_Behavior.lanechg_status = 1;
									leftlight_time = 0;
								}
								leftlight_time += z_period_last;
								if (leftlight_time > 2000)                          //����ת��2s
								{
									leftlight_time = 2000;
								}


								// Ŀ�공��ǰ���ռ����㻻������
								if (Path_Obs_LF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10)
								{
									// Ŀ�공���󷽾������㻻������
									if (Path_Obs_LR.Ob_Pose.dis_lng > 10)    //����������,��ǰ�����ϰ�����ϰ���������15
									{
										// ת���ʱ�����㻻������
										if (leftlight_time > 2000)
										{
											frontobs_time = 0;
											Cur_Behavior.behavior = 2;                      // ����
											Cur_Behavior.target_lanenum = LaneNum_Cur - 1;  // Ŀ�공��
											Cur_Behavior.lanechg_status = 1;                // ������											
										}
										// ת���ʱ�������㻻������
										else
										{
											Cur_Behavior.behavior = 1;                     // ����������
											Cur_Behavior.target_lanenum = LaneNum_Cur;     // Ŀ�공��
										}
									}
									// Ŀ�공���󷽲����㻻������
									else
									{
										Cur_Behavior.behavior = 1;
										Cur_Behavior.target_lanenum = LaneNum_Cur;
									}
								}
								// Ŀ�공��ǰ���ռ䲻���㻻������
								else
								{
									Cur_Behavior.behavior = 1;
									Cur_Behavior.target_lanenum = LaneNum_Cur;
								}
							}
							// ��ͼʣ��������㻻������
							else if (rightchg_condition_flag == true)  //�������У��Ϳ��Ƿ�����ұ��
							{
								if (Cur_Behavior.light_status != 2)
								{
									Cur_Behavior.light_status = 2;
									leftlight_time = 0;
								}
								leftlight_time += z_period_last;
								if (leftlight_time > 2000)                          //����ת��2s
								{
									leftlight_time = 2000;
								}

								// 
								if (Path_Obs_RF.Ob_Pose.dis_lng > Path_Obs_F.Ob_Pose.dis_lng + 10)
								{
									// Ŀ�공���󷽿ռ����㻻������
									if (Path_Obs_RR.Ob_Pose.dis_lng > 10)    //����������,��ǰ�����ϰ�����ϰ���������10
									{
										// ת���ʱ�����㻻������
										if (leftlight_time > 2000)
										{
											frontobs_time = 0;
											Cur_Behavior.behavior = 3;                      // �ұ��
											Cur_Behavior.target_lanenum = LaneNum_Cur = 1;  // Ŀ�공��
											Cur_Behavior.lanechg_status = 1;                // �����

										}
										// ת���ʱ�������㻻������
										else
										{
											Cur_Behavior.behavior = 1;
											Cur_Behavior.target_lanenum = LaneNum_Cur;
										}
									}
									// Ŀ�공���󷽿ռ䲻���㻻������
									else
									{
										Cur_Behavior.behavior = 1;
										Cur_Behavior.target_lanenum = LaneNum_Cur;
									}
								}
							}
							else              //��������
							{
								Cur_Behavior.behavior = 1;
								Cur_Behavior.target_lanenum = LaneNum_Cur;
								Cur_Behavior.lanechg_status = 0;
							}

						}
					}
					// ǰ���ϰ�����ʧ
					else
					{
						Cur_Behavior.behavior = 1;                   // ��������
						Cur_Behavior.target_lanenum = LaneNum_Cur;   // Ŀ�공��
						Cur_Behavior.lanechg_status = 0;						
					}
				}
				// ǰ�����ϰ���
				else
				{
					frontobs_time = 0;
					Cur_Behavior.behavior_to_dlg = 8;            // ������Ҫ�󻻵���ǰ�����ϰ���
					Cur_Behavior.behavior = 1;                   // ��������
					Cur_Behavior.target_lanenum = LaneNum_Cur;   // Ŀ�공��
					Cur_Behavior.lanechg_status = 0;             // �ǻ���״̬
				}
			}			
		}
		// ���������
		else if(Cur_Behavior.lanechg_status==1)
		{
			Cur_Behavior.behavior_to_dlg = 9;       // �����
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
���ƣ�SpeedDecision
���������پ���
���룺
�����
*********************************************************************/
void CDecision::SpeedDecision(const Behavior_Dec Cur_Behavior, vector<GlobalPoint2D>&z_refpath, double &z_velocity_expect)
{
	// ���Ϲ����м���
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
���ƣ�RefPath
����������ο�·��
���룺
�����
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
���ƣ�V2XPedestrianJudge
��������V2X·���豸��⵽��������ʱ���������ͱ�����Ϣ�����ѳ������м������С�
���룺����λ����Ϣ������λ����Ϣ�Ͳο�·����
��������˱�־λ��
*****************************************************************************/
void CDecision::V2XPedestrianJudge(const LocationOut z_LocationOut, const V2X_Data z_V2XData, vector<GlobalPoint2D> &PathPoint_LongFront, bool &pedestrian_flag)
{

	// ����ǰ��100m·��
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	WORD RoadNum_Cur = z_LocationOut.road_num;            // ������ǰ��·���		
	WORD LaneNum_Cur = z_LocationOut.lane_num;            // ������ǰ�������

	WORD Id_CurLane = z_LocationOut.id[LaneNum_Cur - 1];  // �����ڵ�ǰ�����ϵ�·��id���
	WORD IdSum_CurLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1].size();   	// �������ڳ�����·��id����																							

	// ������ǰ��100m��Χ·������
	for (WORD i = min(IdSum_CurLane, Id_CurLane); i < min(IdSum_CurLane, Id_CurLane + 200); i++)   //200���㣬���100m
	{
		GlobalPoint2D p;
		p.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
		p.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
		PathPoint_LongFront.push_back(p);                    // ����ǰ100m��Χ·��洢����ǰ·������
	}

	// �ֲ�������ʼ��
	double distance_pedestrian = 9999; // �����복����Ծ���
	double lat_pedestrian = 0; // ����γ��
	double lng_pedestrian = 0; // ���˾���
	int direction_pedestrian = 0; // ���˷���
	double WIDTH_LANE = 3.75; // ������

	double lng_distance = 9999; // ���˾��복���������
	double lat_distance = 9999; // ���˾��복�ĺ������
	double lat_distance_last = 0; // ��һ֡���˾���ο�·���ĺ������
	int lat_distance_time = 0; // ���˺�������С�Ĵ���
	double width = WIDTH_LANE + WIDTH_LANE / 2; //һ���복���Ŀ��

	// ��ȡV2X������Ϣ
	distance_pedestrian = z_V2XData.PedesDistance;
	lat_pedestrian = z_V2XData.PedesLatitude;
	lng_pedestrian = z_V2XData.PedesLongitude;
	direction_pedestrian = z_V2XData.PedesDirection;

	// �����˾�γ��λ��ת��Ϊȫ�������λ��
	GPSPoint2D pedestrian_gps;
	pedestrian_gps.lat = lat_pedestrian;
	pedestrian_gps.lng = lng_pedestrian;

	//������λ�ý��о�ƫ
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
		// ���˾��복��ǰ��·���ĺ������
		GlobalPoint2D pt;
		GlobalPoint2D pt_next;
		int index = 0;
		index = NearestId(pedestrian_global, PathPoint_LongFront);
		pt.x = PathPoint_LongFront[index].x;
		pt.y = PathPoint_LongFront[index].y;
		pt_next.x = PathPoint_LongFront[index + 1].x;
		pt_next.y = PathPoint_LongFront[index + 1].y;

		lat_distance = LatDis(pedestrian_global, pt, pt_next);

		// ���˾��복�����������
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

		// ���˾��복��·���������仯�������𽥱�С��˵�����˹켣�п�����ο�·�����棬���𽥱��˵�����˿���Զ�복��ǰ��
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

		// �ж��������ͺ�������Ƿ�С��һ�����룬������٣�����������ʻ
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
���ƣ�V2XSignalLight
�������źŵ��·���������������·��ǰ��ǰ���յ�·��RSU�豸���͵��źŵ�
״̬��Ϣ���������ʱ���ٵȴ����̵�ʱ��������ͨ����
��Ϊ��������Ҳ�����ƹ��ܣ��ʸú������ṩV2X�ĺ��̵���Ϣ��Planning�ο�ʹ��
���룺�źŵ���Ϣ
��������̵�״̬��Ϣ
*****************************************************************************/
void CDecision::V2XSignalLight(const V2X_Data z_V2XData, WORD &V2XLight_flag)
{
	// �ֲ�������ʼ��
	int Lane_occupied = 999; //��ǰ�����Ƿ�Ϊ�������ڳ���
	int SignalLight_state = 999; // V2X�źŵ�״̬���졢�̡��ƣ�

	//��ȡ�źŵ��·���Ϣ
	Lane_occupied = z_V2XData.SPATLaneOccupied;
	SignalLight_state = z_V2XData.SPATState;

	if (Lane_occupied == 1) //��ǰ�����Ǳ������ڳ���
	{
		if (SignalLight_state == 3 || SignalLight_state == 7) //3����ƣ�7���Ƶ�
		{
			V2XLight_flag = 1;
		}
		else if (SignalLight_state == 6)//6���̵�
		{
			V2XLight_flag = 2;
		}
		else
		{
			V2XLight_flag = 0;
		}
	}
	/*����ת����������
	else
	{
	}
	*/

}//���������V2X��Ϣ���󣬶��ڵ�ǰ�����Ƿ��Ǳ������ڳ������ж�����

/****************************************************************************
 ���ƣ�V2XConstructionEvent
 ������ʩ������Ԥ����·���豸�㲥ʩ����Ϣ��������ǰ���������¹滮·��
 ���룺ʩ���¼���Ϣ
 ������Ƿ�ʩ����־
 *****************************************************************************/
void CDecision::V2XConstructionEvent(const LocationOut z_LocationOut, const V2X_Data z_V2XData, const vector<WarningPoint> z_RSIWarningPointList, vector<GlobalPoint2D> &PathPoint_LongFront, bool &construction_flag)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();
	// �ֲ�������ʼ��
	double WIDTH_LANE = 3.75; // ������ 
	int constructionpoint_num = 0;
	int index = 0;//����ʩ��������ĳ���·��id
	double min_distance_construction = 9999; // ���������ʩ��·��ľ���
	float near_lat_constructionpoint = 0; // ���복�����ʩ����γ��
	float near_lng_constructionpoint = 0; // ���복�����ʩ���㾭��
	WORD RoadNum_Cur = z_LocationOut.road_num;            // ������ǰ��·���		
	WORD LaneNum_Cur = z_LocationOut.lane_num;            // ������ǰ�������
	WORD Id_CurLane = z_LocationOut.id[LaneNum_Cur - 1];  // �����ڵ�ǰ�����ϵ�·��id���
	WORD IdSum_CurLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1].size();   	// �������ڳ�����·��id����	
	
	// ���س�����ǰ��100m��Χ·������
	for (WORD i = min(IdSum_CurLane, Id_CurLane + ID_MORE); i < min(IdSum_CurLane, Id_CurLane + 200 + ID_MORE); i++)   //200���㣬���100m
	{
		GlobalPoint2D p;
		p.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
		p.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
		PathPoint_LongFront.push_back(p);                    // ����ǰ100m��Χ·��洢����ǰ·������
	}
	vector<WarningPoint> ConstructionPointList;
	//��ȡV2Xʩ������Ԥ����Ϣ
	ConstructionPointList = z_RSIWarningPointList;
	
	if (ConstructionPointList.size() == 0 && (z_V2XData.RSILatitude == 0 || z_V2XData.RSILongitude == 0))////û��ʩ������Ϣ���ж���ʩ���¼��������˳�����
	{
		construction_flag = false;
		return;
	}
	else if (ConstructionPointList.size() == 0 && z_V2XData.RSILatitude != 0 && z_V2XData.RSILongitude != 0) //��WarningListΪ�գ�ֻ���¼��㣬�¼�����Ϊʩ�������WarningPointList
	{
		WarningPoint ConstructionPoint;
		ConstructionPoint.latitude = z_V2XData.RSILatitude;
		ConstructionPoint.longitude = z_V2XData.RSILongitude;
		ConstructionPointList.push_back(ConstructionPoint);
	}

	//����WarningPointList���ҵ����복��������������ʩ���㣬�������侭�ȡ�γ�ȡ��복�����������
	for (WORD num = 0; num < ConstructionPointList.size(); num++)
	{
		double lng_distance_temp = 9999;
		GPSPoint2D constructionpoint_gps_temp;

		//��ʩ����ľ�γ�Ⱦ�ƫ
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
		//�ѵ�ǰʩ����ľ�γ������ת��Ϊ��·�����Ϊԭ���ȫ������
		GlobalPoint2D construction_global_temp;
		construction_global_temp = WGS84ToGlobal(constructionpoint_gps_temp);
		//����ȫ��������㳵���͵�ǰʩ���¼���ĺ����������
		index = NearestId(construction_global_temp, PathPoint_LongFront);//����ʩ��������ĳ���·��id
																		 //����������뵱ǰʩ������������lng_distance_temp
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
		//�ҵ����복��������������ʩ����ľ��ȡ�γ�ȡ��������
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
	GlobalPoint2D near_construction_globalpoint; //���복��������������ʩ����
	GPSPoint2D near_constructionpoint_gps;//���복��������������ʩ����
	near_constructionpoint_gps.lat = near_lat_constructionpoint;
	near_constructionpoint_gps.lng = near_lng_constructionpoint;
	near_construction_globalpoint = WGS84ToGlobal(near_constructionpoint_gps);
	double near_lat_distance = 0;//�������������ʩ����ĺ������
	pt.x = PathPoint_LongFront[index].x;
	pt.y = PathPoint_LongFront[index].y;
	pt_next.x = PathPoint_LongFront[index + 1].x;
	pt_next.y = PathPoint_LongFront[index + 1].y;
	near_lat_distance = LatDis(near_construction_globalpoint, pt, pt_next);//���ʩ�����복���ĺ������

	// �ж��������ͺ�������Ƿ�С��һ�����룬����ǰ����ǰ����ʩ���¼�������������ʻ
	if (min_distance_construction >= 0 && min_distance_construction <= 100)//��������0��С�ڵ���100m
	{
		if (near_lat_distance >= 0 && near_lat_distance < WIDTH_LANE / 2)//�Һ������С��·��һ��,ʩ���¼�����
		{
			construction_flag = true;
		}
		else
		{
			construction_flag = false;
		}

	}
	else //�������С��0m�����100mʩ���¼�δ����
	{
		construction_flag = false;
	}

}

/****************************************************************************
���ƣ�V2XEventDecision
�������ۺ��ж��źŵ��·���ʩ���������������˲����ߵĺ���
���룺��λ��Ϣ��V2X������Ϣ��ǰ��·����Ϣ
������źŵƱ�־λ��ʩ����־λ�����˱�־λ
*****************************************************************************/
void CDecision::V2XEventDecision(const LocationOut z_LocationOut, const V2X_Data z_V2XData, const vector<WarningPoint> z_RSIWarningPointList, vector<GlobalPoint2D> PathPoint_Front_Long, vector<GlobalPoint2D> PathPoint_LF_Long, vector<GlobalPoint2D> PathPoint_RF_Long, bool &pedestrian_flag, WORD &V2XLight_flag, bool &construction_flag)
{
	
	if (z_V2XData.V2XWarnStatus == 3)
	{
		V2XSignalLight(z_V2XData, V2XLight_flag); //�źŵ��·�
	}
	else if (z_V2XData.V2XWarnStatus == 4)
	{
		V2XConstructionEvent(z_LocationOut, z_V2XData, z_RSIWarningPointList, PathPoint_Front_Long, construction_flag); //ʩ������Ԥ��
		//V2XConstructionEventTemporal(z_LocationOut, z_V2XData, z_RSIWarningPointList, PathPoint_Front_Long, PathPoint_LF_Long, PathPoint_RF_Long, construction_flag);
		int a = 0;
	}
	else if (z_V2XData.V2XWarnStatus == 5)
	{
		V2XPedestrianJudge(z_LocationOut, z_V2XData, PathPoint_Front_Long, pedestrian_flag); //��������Ԥ��
	}
	else
	{
		return;
	}
}
/****************************************************************************
���ƣ�V2XConstructionEventTemporal
������ʩ������Ԥ����·���豸�㲥ʩ����Ϣ��������ǰ���������¹滮·��
���룺ʩ���¼���Ϣ
������Ƿ�ʩ����־
*****************************************************************************/
void CDecision::V2XConstructionEventTemporal(const LocationOut z_LocationOut, const V2X_Data z_V2XData, const vector<WarningPoint> z_RSIWarningPointList, vector<GlobalPoint2D> PathPoint_Front_Long, vector<GlobalPoint2D> PathPoint_LF_Long, vector<GlobalPoint2D> PathPoint_RF_Long, bool &construction_flag)
{
	CGAC_Auotpilot_DPApp *app = (CGAC_Auotpilot_DPApp*)AfxGetApp();

	WORD RoadNum_Cur = z_LocationOut.road_num;            // ������ǰ��·���		
	WORD LaneNum_Cur = z_LocationOut.lane_num;            // ������ǰ�������

	WORD Id_CurLane = z_LocationOut.id[LaneNum_Cur - 1];  // �����ڵ�ǰ�����ϵ�·��id���
	WORD IdSum_CurLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1].size();   	// �������ڳ�����·��id����																							
	WORD LaneSum = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][0].lane_sum;     // �������ڵ�·��������

	// ��ͼ�ɱ�����ԣ�0���ɱ����1��������2���ұ����3�����ұ��
	WORD LaneChg = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][Id_CurLane].lanechg_attribute;

	double Width_CurLane = 0; 	     //�������ڳ������
	WORD Id_LeftLane = 0;            //�����ڳ�����೵���ϵ�id��ţ��������޳�������idΪ0 	
	WORD IdSum_LeftLane = 0;         //�����ڳ�����೵����·��id�������������޳�������idsumΪ0 
	double Width_LeftLane = 0;		 //�������ڳ�����೵����ȣ��������޳�����������СΪ0 
	WORD Id_RightLane = 0;			 //�������Ҳ೵���ϵ�id��ţ�����Ҳ��޳�������idΪ0 		
	WORD IdSum_RightLane = 0;        //�������Ҳ೵����·��id����������Ҳ��޳�������idsumΪ0 
	double Width_RightLane = 0; 	 //�������ڳ����Ҳ೵����ȣ�����Ҳ��޳�����������СΪ0 	

	Width_CurLane = (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][Id_CurLane].lane_width) / 100.0;

	// ������ǰ��100m��Χ·������
	for (WORD i = min(IdSum_CurLane, Id_CurLane + ID_MORE); i < min(IdSum_CurLane, Id_CurLane + 200 + ID_MORE); i++)
	{
		GlobalPoint2D pt;
		pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.x;
		pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 1][i].global_point.y;
		PathPoint_Front_Long.push_back(pt);                     // ������60m��Χ·��洢������·������
	}

	// ������ǰ��·������
	if (LaneChg == 1)
	{
		if (LaneNum_Cur > 1)   // Ҫ����೵�����򳵵������Ҫ����1
		{
			Id_LeftLane = z_LocationOut.id[LaneNum_Cur - 2];                                   // �����ڵ�ǰ������೵��ID
			IdSum_LeftLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2].size();   // ��೵��·������
			if ((Id_LeftLane > 0) && (Id_LeftLane < IdSum_LeftLane))                           // ������೵�������������Ͷ�ͷ����
			{
				Width_LeftLane = (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][Id_LeftLane].lane_width) / 100.0;

				GlobalPoint2D pt;
				for (int i = min(IdSum_LeftLane, Id_LeftLane + ID_MORE); i < min(IdSum_LeftLane, Id_LeftLane + 200 + ID_MORE); i++)                  // ��120���㣬���60m
				{
					pt.x = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][i].global_point.x;
					pt.y = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur - 2][i].global_point.y;
					PathPoint_LF_Long.push_back(pt);
				}
			}
		}
	}

	// ������ǰ��·������
	if (LaneChg == 2)
	{
		if (LaneNum_Cur < LaneSum)       //Ҫ���Ҳ೵�����򳵵������ҪС�ڳ�������
		{
			Id_RightLane = z_LocationOut.id[LaneNum_Cur];                                    // �������Ҳ೵����Ӧ��·��ID
			IdSum_RightLane = app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur].size();    // �Ҳ೵��·������

			if ((Id_RightLane > 0) && (Id_RightLane < IdSum_RightLane))                      // �����Ҳ����������Ͷ�ͷ����
			{
				// �Ҳ೵�����
				Width_RightLane = (app->decision_MapData[RoadNum_Cur - 1][LaneNum_Cur][Id_RightLane].lane_width) / 100.0;
				GlobalPoint2D pt;
				for (int i = min(IdSum_RightLane, Id_RightLane + ID_MORE); i < min(IdSum_RightLane, Id_RightLane + 200 + ID_MORE); i++)   //120���㣬���60m
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

	if (rsi_distance >= 0 && rsi_distance <= 100) //�Գ���v2x��������С��100m���Ž����ж�
	{
		for (int i = 0; i < l_size_Point_F; i++)  //����v2x�������Գ�����ÿ��·��ľ��룬�洢��l_distance_v2x_pathpoint_rear������
		{
			l_distance = CalcDistance(rsipoint_Global2D, PathPoint_Front_Long[i]);
			l_distance_v2x_pathpoint_front.push_back(l_distance);
		}
		for (int i = 0; i < l_size_Point_F; i++) //l_distance_v2x_pathpoint_rear�����У��ҳ�v2x�������Գ�����·�����С����
		{
			if (l_distance_v2x_pathpoint_front[i] < l_min_distance_front)
			{
				l_min_distance_front = l_distance_v2x_pathpoint_front[i];
			}
		}

		for (int i = 0; i < l_size_Point_LF; i++)  //����v2x�������Գ����ÿ��·��ľ��룬�洢��l_distance_v2x_pathpoint_lr������
		{
			l_distance = CalcDistance(rsipoint_Global2D, PathPoint_LF_Long[i]);
			l_distance_v2x_pathpoint_lf.push_back(l_distance);
		}
		for (int i = 0; i < l_size_Point_LF; i++)  //l_distance_v2x_pathpoint_lr�����У��ҳ�v2x�������Գ����·�����С����
		{
			if (l_distance_v2x_pathpoint_lf[i] < l_min_distance_lf)
			{
				l_min_distance_lf = l_distance_v2x_pathpoint_lf[i];
			}
		}

		for (int i = 0; i < l_size_Point_RF; i++) //����v2x�������Գ��Һ�ÿ��·��ľ��룬�洢��l_distance_v2x_pathpoint_rr������
		{
			l_distance = CalcDistance(rsipoint_Global2D, PathPoint_RF_Long[i]);
			l_distance_v2x_pathpoint_rf.push_back(l_distance);
		}
		for (int i = 0; i < l_size_Point_RF; i++) //l_distance_v2x_pathpoint_rr�����У��ҳ�v2x�������Գ��Һ�·�����С����
		{
			if (l_distance_v2x_pathpoint_rf[i] < l_min_distance_rf)
			{
				l_min_distance_rf = l_distance_v2x_pathpoint_rf[i];
			}
		}

		if ((l_min_distance_lf < l_min_distance_rf) && (l_min_distance_lf < l_min_distance_front) && (l_min_distance_lf < 2)) //�ж�v2x�����Ƿ����Գ���󷽳���
		{
			construction_flag = false;
		}
		else if ((l_min_distance_rf < l_min_distance_lf) && (l_min_distance_rf < l_min_distance_front) && (l_min_distance_rf < 2)) //�ж�v2x�����Ƿ����Գ��Һ󷽳���
		{
			construction_flag = false;
		}
		else if ((l_min_distance_front < l_min_distance_lf) && (l_min_distance_front < l_min_distance_rf) && (l_min_distance_front < 2)) //�ж�v2x�����Ƿ����Գ����󷽳���
		{

			// �ֲ�������ʼ��
			double WIDTH_LANE = 3.75; // ������ 
			int constructionpoint_num = 0;
			int index = 0;//����ʩ��������ĳ���·��id
			double min_distance_construction = 9999; // ���������ʩ��·��ľ���
			float near_lat_constructionpoint = 0; // ���복�����ʩ����γ��
			float near_lng_constructionpoint = 0; // ���복�����ʩ���㾭��
			
			vector<WarningPoint> ConstructionPointList;
			//��ȡV2Xʩ������Ԥ����Ϣ
			ConstructionPointList = z_RSIWarningPointList;

			if (ConstructionPointList.size() == 0 && (z_V2XData.RSILatitude == 0 || z_V2XData.RSILongitude == 0))////û��ʩ������Ϣ���ж���ʩ���¼��������˳�����
			{
				construction_flag = false;
				return;
			}
			else if (ConstructionPointList.size() == 0 && z_V2XData.RSILatitude != 0 && z_V2XData.RSILongitude != 0) //��WarningListΪ�գ�ֻ���¼��㣬�¼�����Ϊʩ�������WarningPointList
			{
				WarningPoint ConstructionPoint;
				ConstructionPoint.latitude = z_V2XData.RSILatitude;
				ConstructionPoint.longitude = z_V2XData.RSILongitude;
				ConstructionPointList.push_back(ConstructionPoint);
			}

			//����WarningPointList���ҵ����복��������������ʩ���㣬�������侭�ȡ�γ�ȡ��복�����������
			for (WORD num = 0; num < ConstructionPointList.size(); num++)
			{
				double lng_distance_temp = 9999;
				GPSPoint2D constructionpoint_gps_temp;

				//��ʩ����ľ�γ�Ⱦ�ƫ
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
				//�ѵ�ǰʩ����ľ�γ������ת��Ϊ��·�����Ϊԭ���ȫ������
				GlobalPoint2D construction_global_temp;
				construction_global_temp = WGS84ToGlobal(constructionpoint_gps_temp);
				//����ȫ��������㳵���͵�ǰʩ���¼���ĺ����������
				index = NearestId(construction_global_temp, PathPoint_Front_Long);//����ʩ��������ĳ���·��id
																				 //����������뵱ǰʩ������������lng_distance_temp
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
				
				//�ҵ����복��������������ʩ����ľ��ȡ�γ�ȡ��������
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

			// �ж��������ͺ�������Ƿ�С��һ�����룬����ǰ����ǰ����ʩ���¼�������������ʻ
			if (min_distance_construction >= 0 && min_distance_construction <= 100)//��������0��С�ڵ���100m
			{
				construction_flag = true;
			}
			else //�������С��0m�����100mʩ���¼�δ����
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








