#pragma once
#include "Share.h"

using namespace std;

class CDecision :public CShare
{
private:         //��Ϊ���˽�б��������ڵĺ������Թ���
	vector<RoadInfo> z_RoadInfo;
	vector<PathInfo> z_RoadNavi;
	LocationOut z_LocationOut;
	VehStatus z_VehStatus;
	vector<ObPoint> z_Obs;
	vector<ObPoint> z_DynaObs_front;
	vector<ObPoint> z_DynaObs_rear;
	vector<GlobalPoint2D> z_Person;
	vector<GlobalPoint2D> z_LinePoints;


	vector<V2X_DataOut> z_V2XDataOutList;
	V2X_Data z_V2XData;
	vector<V2VWarn> z_V2VRemoteList;
	vector<WarningPoint> z_RSIWarningPointList;
	bool V2VICWWarnFlag;  //ʮ��·����ײԤ����־λ�����Գ�OBU����ʮ��·����ײԤ�����Ҷ�ӦԶ���뱾����һ����Χ��ʱ�����˱�־λ��Ϊtrue
	bool V2VICWclose2routeFlag;  //ʮ��·����ײ����Զ����������·����־λ����V2VICWWarnFlagΪtrue����Զ�����Գ�����·���Ͻ�ʱ�����˱�־λ��Ϊtrue

	WORD z_behavior_to_dlg;            
	bool z_segment_lanechg_status;    //·�ϱ��״̬��0���������֣�1�������
	bool z_segment_obsavoid_status;   //·�ϱ��״̬��0���������֣�1��������

	double leftlight_time;            // ��ת��ʱ��
	double rightlight_time;           // ��ת��ʱ��

	double z_period_last;
	double z_period_max;
	WORD z_behavior;                 // ������Ϊ. 1���������֣�2���󻻵���3���һ�����4�������ϣ�5�������ϣ�6��A*
	WORD z_target_roadnum;           // Ŀ���·���
	WORD z_target_lanenum;           // Ŀ�공�����
	WORD z_light_status;             // ת���״̬.0���޵� 1����ת��2����ת��3��˫��

	WORD his_behavior;               // ��һ֡��Ϊ���߽����1���������֣�2���󻻵���3���һ�����4�������ϣ�5�������ϣ�6��A*
	//WORD his_target_roadnum;         // ��һ֡Ŀ���·
	WORD his_target_lanenum;         // ��һ֡Ŀ�공��
	WORD his_light_status;           // ��һ֡ת���״̬��0���޵� 1����ת��2����ת��3��˫��


	double z_velocity_expect;        // Ŀ���ٶ�
	vector<GlobalPoint2D> z_refpath; // �ο�·��

	Path_Obs Path_Obs_F;                                  // ��ǰ������ǰ���ϰ�����Ϣ
	Path_Obs Path_Obs_R;                                  // ��ǰ���������ϰ�����Ϣ
	Path_Obs Path_Obs_LF;                                 // ��ǰ������ǰ���ϰ�����Ϣ
	Path_Obs Path_Obs_LR;                                 // ��ǰ��������ϰ�����Ϣ
	Path_Obs Path_Obs_RF;                                 // ��ǰ������ǰ���ϰ�����Ϣ
	Path_Obs Path_Obs_RR;                                 // ��ǰ�����Һ��ϰ�����Ϣ



private:
	CDecision();
	~CDecision();

	static DWORD threadEntry(LPVOID lpParam);
	DWORD CDecisionThread(void);

	void V2XPedestrianJudge(const LocationOut z_LocationOut, const V2X_Data z_V2XData, vector<GlobalPoint2D> &PathPoint_LongFront, bool &pedestrian_flag);
	void V2XSignalLight(const V2X_Data z_V2XData, WORD &V2XLight_flag);//�źŵ��·�
	void V2XConstructionEvent(const LocationOut z_LocationOut, const V2X_Data z_V2XData, const vector<WarningPoint> z_RSIWarningPointList, vector<GlobalPoint2D> &PathPoint_LongFront, bool &construction_flag);
	void V2XConstructionEventTemporal(const LocationOut z_LocationOut, const V2X_Data z_V2XData, const vector<WarningPoint> z_RSIWarningPointList, vector<GlobalPoint2D> PathPoint_Front_Long, vector<GlobalPoint2D> PathPoint_LF_Long, vector<GlobalPoint2D> PathPoint_RF_Long, bool &construction_flag);

	//void V2XEventDecision(const LocationOut z_LocationOut, const V2X_Data z_V2XData, const vector<WarningPoint> z_RSIWarningPointList, vector<GlobalPoint2D> &PathPoint_LongFront, bool &pedestrian_flag, WORD &V2XLight_flag, bool &construction_flag);
	void V2XEventDecision(const LocationOut z_LocationOut, const V2X_Data z_V2XData, const vector<WarningPoint> z_RSIWarningPointList, vector<GlobalPoint2D> PathPoint_Front_Long, vector<GlobalPoint2D> PathPoint_LF_Long, vector<GlobalPoint2D> PathPoint_RF_Long, bool &pedestrian_flag, WORD &V2XLight_flag, bool &construction_flag);

	void SegmentDecision();           // ·��
	
	void PreStubDecision();           // Ԥ·��
	
	void StubDecision();              // ·��

	// ���س�����Χ��Բο�·��
	void LoadRefPath(const LocationOut z_LocationOut, vector <GlobalPoint2D> &PathPoint_Front, vector<GlobalPoint2D> &PathPoint_Rear, vector<GlobalPoint2D> &PathPoint_LF, vector<GlobalPoint2D> &PathPoint_LR,
		vector<GlobalPoint2D> &PathPoint_RF, vector<GlobalPoint2D> &PathPoint_RR, double &Width_CurLane);
	
	// �ж������Ƿ���Ҫ������������������
	void Nav_LaneChange(const LocationOut z_LocationOut, const vector<PathInfo> z_RoadNavi, UINT &Navi_LaneChg, UINT &Navi_LaneChg_times);
	
	// ���㵼��Ҫ��Ļ�������
	BYTE CalcNaviLaneChgTimes(const BYTE roadnum_cur, const WORD outlanenum[LANESUM], BYTE lanechgdir);

	// ����������Χ�ϰ���
	void AroundObstacle(const vector<GlobalPoint2D>PathPoint_Front,const vector<GlobalPoint2D>PathPoint_Rear,const vector<GlobalPoint2D>PathPoint_LF,const vector<GlobalPoint2D>PathPoint_LR,
		const vector<GlobalPoint2D>PathPoint_RF,const vector<GlobalPoint2D>PathPoint_RR, Path_Obs &Path_Obs_F,Path_Obs &Path_Obs_R,Path_Obs &Path_Obs_LF,Path_Obs &Path_Obs_LR,
		Path_Obs &Path_Obs_RF,Path_Obs &Path_Obs_RR,const double Width_CurLane);

	// ��Ϊ�������ж�
	void BehaviorDecision(const LocationOut z_LocationOut, const vector<PathInfo> z_RoadNavi, const UINT Navi_LaneChg,const UINT Navi_LaneChg_times,const double Width_CurLane,const vector<GlobalPoint2D>PathPoint_Front,
		const Path_Obs &Path_Obs_F, const Path_Obs &Path_Obs_R, const Path_Obs &Path_Obs_LF, const Path_Obs &Path_Obs_LR,const Path_Obs &Path_Obs_RF, const Path_Obs &Path_Obs_RR,const Behavior_Dec His_Behavior, Behavior_Dec &Cur_Behavior);

	// ���پ���
	void SpeedDecision(const Behavior_Dec Cur_Behavior,vector<GlobalPoint2D>&z_refpath,double &z_velocity_expect);

	// �ο�·��
	void RefPath(const Behavior_Dec Cur_Behavior, const vector<GlobalPoint2D>PathPoint_Front, const vector<GlobalPoint2D>PathPoint_LF, const vector<GlobalPoint2D>PathPoint_RF,vector<GlobalPoint2D>&z_refpath);
	
public:
	static CDecision& CDecision::Instance();
	BYTE startCDecisionThread();

};