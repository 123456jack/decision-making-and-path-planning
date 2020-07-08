#pragma once
#include "Share.h"

using namespace std;

class CDecision :public CShare
{
private:         //作为类的私有变量，类内的函数可以共用
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
	bool V2VICWWarnFlag;  //十字路口碰撞预警标志位，当自车OBU发出十字路口碰撞预警、且对应远车离本车在一定范围内时，将此标志位置为true
	bool V2VICWclose2routeFlag;  //十字路口碰撞场景远车靠近导航路径标志位，当V2VICWWarnFlag为true，且远车离自车导航路径较近时，将此标志位置为true

	WORD z_behavior_to_dlg;            
	bool z_segment_lanechg_status;    //路上变道状态：0：车道保持，1：变道中
	bool z_segment_obsavoid_status;   //路上变道状态：0：车道保持，1：绕障中

	double leftlight_time;            // 左转灯时长
	double rightlight_time;           // 右转灯时长

	double z_period_last;
	double z_period_max;
	WORD z_behavior;                 // 决策行为. 1：车道保持，2：左换道，3：右换道，4：左绕障，5：右绕障，6：A*
	WORD z_target_roadnum;           // 目标道路编号
	WORD z_target_lanenum;           // 目标车道编号
	WORD z_light_status;             // 转向灯状态.0：无灯 1：左转，2：右转，3：双闪

	WORD his_behavior;               // 上一帧行为决策结果，1：车道保持，2：左换道，3：右换道，4：左绕障，5：右绕障，6：A*
	//WORD his_target_roadnum;         // 上一帧目标道路
	WORD his_target_lanenum;         // 上一帧目标车道
	WORD his_light_status;           // 上一帧转向灯状态，0：无灯 1：左转，2：右转，3：双闪


	double z_velocity_expect;        // 目标速度
	vector<GlobalPoint2D> z_refpath; // 参考路径

	Path_Obs Path_Obs_F;                                  // 当前车道正前方障碍物信息
	Path_Obs Path_Obs_R;                                  // 当前车道正后方障碍物信息
	Path_Obs Path_Obs_LF;                                 // 当前车道左前方障碍物信息
	Path_Obs Path_Obs_LR;                                 // 当前车道左后方障碍物信息
	Path_Obs Path_Obs_RF;                                 // 当前车道右前方障碍物信息
	Path_Obs Path_Obs_RR;                                 // 当前车道右后方障碍物信息



private:
	CDecision();
	~CDecision();

	static DWORD threadEntry(LPVOID lpParam);
	DWORD CDecisionThread(void);

	void V2XPedestrianJudge(const LocationOut z_LocationOut, const V2X_Data z_V2XData, vector<GlobalPoint2D> &PathPoint_LongFront, bool &pedestrian_flag);
	void V2XSignalLight(const V2X_Data z_V2XData, WORD &V2XLight_flag);//信号灯下发
	void V2XConstructionEvent(const LocationOut z_LocationOut, const V2X_Data z_V2XData, const vector<WarningPoint> z_RSIWarningPointList, vector<GlobalPoint2D> &PathPoint_LongFront, bool &construction_flag);
	void V2XConstructionEventTemporal(const LocationOut z_LocationOut, const V2X_Data z_V2XData, const vector<WarningPoint> z_RSIWarningPointList, vector<GlobalPoint2D> PathPoint_Front_Long, vector<GlobalPoint2D> PathPoint_LF_Long, vector<GlobalPoint2D> PathPoint_RF_Long, bool &construction_flag);

	//void V2XEventDecision(const LocationOut z_LocationOut, const V2X_Data z_V2XData, const vector<WarningPoint> z_RSIWarningPointList, vector<GlobalPoint2D> &PathPoint_LongFront, bool &pedestrian_flag, WORD &V2XLight_flag, bool &construction_flag);
	void V2XEventDecision(const LocationOut z_LocationOut, const V2X_Data z_V2XData, const vector<WarningPoint> z_RSIWarningPointList, vector<GlobalPoint2D> PathPoint_Front_Long, vector<GlobalPoint2D> PathPoint_LF_Long, vector<GlobalPoint2D> PathPoint_RF_Long, bool &pedestrian_flag, WORD &V2XLight_flag, bool &construction_flag);

	void SegmentDecision();           // 路上
	
	void PreStubDecision();           // 预路口
	
	void StubDecision();              // 路口

	// 加载车辆周围相对参考路径
	void LoadRefPath(const LocationOut z_LocationOut, vector <GlobalPoint2D> &PathPoint_Front, vector<GlobalPoint2D> &PathPoint_Rear, vector<GlobalPoint2D> &PathPoint_LF, vector<GlobalPoint2D> &PathPoint_LR,
		vector<GlobalPoint2D> &PathPoint_RF, vector<GlobalPoint2D> &PathPoint_RR, double &Width_CurLane);
	
	// 判定导航是否需要换道换道次数和类型
	void Nav_LaneChange(const LocationOut z_LocationOut, const vector<PathInfo> z_RoadNavi, UINT &Navi_LaneChg, UINT &Navi_LaneChg_times);
	
	// 计算导航要求的换道次数
	BYTE CalcNaviLaneChgTimes(const BYTE roadnum_cur, const WORD outlanenum[LANESUM], BYTE lanechgdir);

	// 搜索车辆周围障碍物
	void AroundObstacle(const vector<GlobalPoint2D>PathPoint_Front,const vector<GlobalPoint2D>PathPoint_Rear,const vector<GlobalPoint2D>PathPoint_LF,const vector<GlobalPoint2D>PathPoint_LR,
		const vector<GlobalPoint2D>PathPoint_RF,const vector<GlobalPoint2D>PathPoint_RR, Path_Obs &Path_Obs_F,Path_Obs &Path_Obs_R,Path_Obs &Path_Obs_LF,Path_Obs &Path_Obs_LR,
		Path_Obs &Path_Obs_RF,Path_Obs &Path_Obs_RR,const double Width_CurLane);

	// 行为决策与判定
	void BehaviorDecision(const LocationOut z_LocationOut, const vector<PathInfo> z_RoadNavi, const UINT Navi_LaneChg,const UINT Navi_LaneChg_times,const double Width_CurLane,const vector<GlobalPoint2D>PathPoint_Front,
		const Path_Obs &Path_Obs_F, const Path_Obs &Path_Obs_R, const Path_Obs &Path_Obs_LF, const Path_Obs &Path_Obs_LR,const Path_Obs &Path_Obs_RF, const Path_Obs &Path_Obs_RR,const Behavior_Dec His_Behavior, Behavior_Dec &Cur_Behavior);

	// 车速决策
	void SpeedDecision(const Behavior_Dec Cur_Behavior,vector<GlobalPoint2D>&z_refpath,double &z_velocity_expect);

	// 参考路径
	void RefPath(const Behavior_Dec Cur_Behavior, const vector<GlobalPoint2D>PathPoint_Front, const vector<GlobalPoint2D>PathPoint_LF, const vector<GlobalPoint2D>PathPoint_RF,vector<GlobalPoint2D>&z_refpath);
	
public:
	static CDecision& CDecision::Instance();
	BYTE startCDecisionThread();

};