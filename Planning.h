#pragma once
#include "Share.h"

class CPlanning :public CShare
{
private:
	DecisionOut z_DecisionOut;          // 行为决策结果信息
	LocationOut z_LocationOut;          // 车辆定位信息
	VehStatus z_VehStatus;              // 车辆位姿信息
	vector<ObPoint> z_Obs;              // 障碍物信息

	double z_period_last;               // 上一个周期时间    
	double z_period_max;                // 最大周期
	BYTE z_lanenum_cur;                 // 当前车道编号
	BYTE z_roadnum_cur;                 // 当前道路编号


	GlobalPoint2D z_refpath[REF_PATHPOINT];      // 参考路径点

	FLOAT faraim_dis;                            // 远预瞄距离
	FLOAT nearaim_dis;                           // 近预瞄距离
	AimPoint aimpoint_near;                      // 近预瞄点
	AimPoint aimpoint_far;                       // 远预瞄点


private:
	CPlanning();
	~CPlanning();

	static DWORD threadEntry(LPVOID lpParam);    // 规划线程回调函数 	
	DWORD CPlanningThread(void);                 // 规划线程执行函数

	static GlobalPoint2D last_Bpoints[200];      // 存储上一周期的路点
	GlobalPoint2D search_points[200];	         // 车辆沿原路径到新的预瞄点之间规划的一条曲线
	


public:
	static CPlanning& CPlanning::Instance();    // 规划线程单例初始化函数
	BYTE startCPlanningThread();                // 初始化规划线程函数

	double path_lat_dis;			         	// 车辆相对于局部规划路径横向偏差，点在路径的左边为正，右边为负
	bool afresh_planning;                        // 重规划标志位，若发生重规划为true，否则为false
	int afresh_cause;	                        // 重规划原因
	double remain_dis;				          	// 局部规划路径剩余距离
	double path_dir_err;				        // 车辆相对于局部规划路径航向偏差
	int path_near_id;				           	// 车辆相对于局部路径最近ID点
	int path_front_near_id;			         	// 已规划路径与车头最近点ID
	int his_behavior;                           // 上一帧行为
	double brakespeed;                           // 规划的期望车速
	bool acc_flag;                               // 制动减速度标志位，false表示舒适减速，true表示AEB减速度制动
	double des_acc;                              // 制动减速度

	inline int Sgn(double a) { return a > 0 ? 1 : -1; };//使用该函数的前提条件是fabs(a)>EPSILON

	// 计算预瞄距离
	void Calculate_aim_dis(DecisionOut decision_result,LocationOut vhcl_location,VehStatus vhel_status,FLOAT &faraim_dis,FLOAT &nearaim_dis);
	
	// 搜索预瞄点
	void SearchAimPoint(DecisionOut decision_result,LocationOut vhcl_location,VehStatus vhcl_status, AimPoint &aimpoint_far, AimPoint &aimpoint_near);

	// 初始规划
	void InitialPlanning(DecisionOut decision_result, LocationOut vhcl_location, VehStatus vhcl_status, const AimPoint aimpoint_far, const AimPoint aimpoint_near,GlobalPoint2D Bezier_points[]);

	// 相对距离路径判定
	void GetVhclLocalState(LocationOut vhcl_location, const GlobalPoint2D last_Bpoints[], double& mindist_lat, double& path_dir_err, int& mindist_id, int& front_mindist_id, double& remain_dis);

	// 重规划判定
	bool UpdatePlanJudge(const DecisionOut decision_result, const LocationOut vhcl_location,const int his_behavior,int &afreshcause);

	// 路径规划
	void PathPlanning(const DecisionOut z_DecisionOut, int afresh_cause, LocationOut vhcl_location,const AimPoint aimpoint_far,const AimPoint aimpoint_near,GlobalPoint2D road_points[]);

	// 车速规划
	void SpeedPlanning(const bool ob_flag, const DecisionOut decision_result, const LocationOut vhcl_location, const double mindist_lon,
		const double mindist_lat, const FLOAT faraim_dis,double &brakespeed,bool &acc_flag,double &des_acc);

	// 计算点到直线路径距离
	double GetLatDis(GlobalPoint2D cur_pt, GlobalPoint2D pt, GlobalPoint2D pt_next);
	// 计算两点航向
	double GetRoadAngle(GlobalPoint2D apoint, GlobalPoint2D bpoint);
	// 计算两个航向误差
	double CPlanning::GetAngleErr(double dir1, double dir2);	
	//计算规划路径曲率
	double CalculateRadius();
	//// 计算两点之间的距离
	//double CalculatePointDis(GlobalPoint2D F_point,GlobalPoint2D S_point);

};
