#pragma once
#include "Share.h"

class CPlanning :public CShare
{
private:
	DecisionOut z_DecisionOut;          // ��Ϊ���߽����Ϣ
	LocationOut z_LocationOut;          // ������λ��Ϣ
	VehStatus z_VehStatus;              // ����λ����Ϣ
	vector<ObPoint> z_Obs;              // �ϰ�����Ϣ

	double z_period_last;               // ��һ������ʱ��    
	double z_period_max;                // �������
	BYTE z_lanenum_cur;                 // ��ǰ�������
	BYTE z_roadnum_cur;                 // ��ǰ��·���


	GlobalPoint2D z_refpath[REF_PATHPOINT];      // �ο�·����

	FLOAT faraim_dis;                            // ԶԤ�����
	FLOAT nearaim_dis;                           // ��Ԥ�����
	AimPoint aimpoint_near;                      // ��Ԥ���
	AimPoint aimpoint_far;                       // ԶԤ���


private:
	CPlanning();
	~CPlanning();

	static DWORD threadEntry(LPVOID lpParam);    // �滮�̻߳ص����� 	
	DWORD CPlanningThread(void);                 // �滮�߳�ִ�к���

	static GlobalPoint2D last_Bpoints[200];      // �洢��һ���ڵ�·��
	GlobalPoint2D search_points[200];	         // ������ԭ·�����µ�Ԥ���֮��滮��һ������
	


public:
	static CPlanning& CPlanning::Instance();    // �滮�̵߳�����ʼ������
	BYTE startCPlanningThread();                // ��ʼ���滮�̺߳���

	double path_lat_dis;			         	// ��������ھֲ��滮·������ƫ�����·�������Ϊ�����ұ�Ϊ��
	bool afresh_planning;                        // �ع滮��־λ���������ع滮Ϊtrue������Ϊfalse
	int afresh_cause;	                        // �ع滮ԭ��
	double remain_dis;				          	// �ֲ��滮·��ʣ�����
	double path_dir_err;				        // ��������ھֲ��滮·������ƫ��
	int path_near_id;				           	// ��������ھֲ�·�����ID��
	int path_front_near_id;			         	// �ѹ滮·���복ͷ�����ID
	int his_behavior;                           // ��һ֡��Ϊ
	double brakespeed;                           // �滮����������
	bool acc_flag;                               // �ƶ����ٶȱ�־λ��false��ʾ���ʼ��٣�true��ʾAEB���ٶ��ƶ�
	double des_acc;                              // �ƶ����ٶ�

	inline int Sgn(double a) { return a > 0 ? 1 : -1; };//ʹ�øú�����ǰ��������fabs(a)>EPSILON

	// ����Ԥ�����
	void Calculate_aim_dis(DecisionOut decision_result,LocationOut vhcl_location,VehStatus vhel_status,FLOAT &faraim_dis,FLOAT &nearaim_dis);
	
	// ����Ԥ���
	void SearchAimPoint(DecisionOut decision_result,LocationOut vhcl_location,VehStatus vhcl_status, AimPoint &aimpoint_far, AimPoint &aimpoint_near);

	// ��ʼ�滮
	void InitialPlanning(DecisionOut decision_result, LocationOut vhcl_location, VehStatus vhcl_status, const AimPoint aimpoint_far, const AimPoint aimpoint_near,GlobalPoint2D Bezier_points[]);

	// ��Ծ���·���ж�
	void GetVhclLocalState(LocationOut vhcl_location, const GlobalPoint2D last_Bpoints[], double& mindist_lat, double& path_dir_err, int& mindist_id, int& front_mindist_id, double& remain_dis);

	// �ع滮�ж�
	bool UpdatePlanJudge(const DecisionOut decision_result, const LocationOut vhcl_location,const int his_behavior,int &afreshcause);

	// ·���滮
	void PathPlanning(const DecisionOut z_DecisionOut, int afresh_cause, LocationOut vhcl_location,const AimPoint aimpoint_far,const AimPoint aimpoint_near,GlobalPoint2D road_points[]);

	// ���ٹ滮
	void SpeedPlanning(const bool ob_flag, const DecisionOut decision_result, const LocationOut vhcl_location, const double mindist_lon,
		const double mindist_lat, const FLOAT faraim_dis,double &brakespeed,bool &acc_flag,double &des_acc);

	// ����㵽ֱ��·������
	double GetLatDis(GlobalPoint2D cur_pt, GlobalPoint2D pt, GlobalPoint2D pt_next);
	// �������㺽��
	double GetRoadAngle(GlobalPoint2D apoint, GlobalPoint2D bpoint);
	// ���������������
	double CPlanning::GetAngleErr(double dir1, double dir2);	
	//����滮·������
	double CalculateRadius();
	//// ��������֮��ľ���
	//double CalculatePointDis(GlobalPoint2D F_point,GlobalPoint2D S_point);

};
