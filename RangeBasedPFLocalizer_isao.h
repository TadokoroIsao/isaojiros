/*____________________________________________________________________
* Copyright (c) ASAMA-YAMASHITA Lab., Dept. of Precision Eng., The Univ. of Tokyo. 
* All rights reserved 2013                                                                 
* Description : Range-based particle filter localizer class [ADT]
* Author : Yonghoon JI (ji@robot.t.u-tokyo.ac.jp, ji.nineppl@gmail.com) 
* Date : January, 2014              
* Precondition  :
______________________________________________________________________*/

#ifndef RANGEBASEDPFLOCALIZER_H2
#define RANGEBASEDPFLOCALIZER_H2

#include <math.h>
#include <random>
#include "../State/RobotState.h"
#include "../State/MapState.h"
#include "../State/Calculus.h"
#include "./Particle.h"


class __declspec(dllexport)CRangeBasedPFLocalizer_isao
{
	typedef struct _tagParameter
	{
		int nMinParticleNum;
		int nMaxParticleNum;
		int nParticleDensity; // x��EEǥ�������� 1m�̰�E y��EEǥ�������� 1m�� �� ������ ������ ����E 1m*1m ������ ������ �Ѹ� ������ ����E

		double dDeviationforTrans; // ������ѿ�� ��E?��Ȯ�Ǽ�
		double dDeviationforRot; // ȸ��E�ѿ�?��E?��Ȯ�Ǽ�.
		double dDeviationforTransToRot; 	// ������ѿ�� ȸ��E�ѿ�?������ �ִ� ��Ȯ�Ǽ�.
		int nMinParticleSD;  // ������ġ���� ����E�� ������ ��ġ�� �� ������ ������E��Ǹ𵨸� ����EϿ?������ �� ������ ��.

		double dRangeMaxDist;
		int nRangeInterval;
	}Parameter;

	typedef struct _tagPolarRnageData
	{
		double r;
		double th;
	}PolarRangeData;

private:
	//Output domain
	CRobotState m_RobotState; 
	CRobotState m_PFRobotState;


	//Input domain
	CTWODMapState* m_pMap;
	CRobotState::Position m_DeltaPosition; 
	vector<CRobotState::Position> m_HitPointPositions;
	vector<int> m_i_feature_vec;
	CRobotState::Orientation m_DeltaOrientation; 
	vector<PolarRangeData>  m_PolarRangeData;

	Parameter m_Param;
	static float ** m_fRangeMCLPro;
	static float ** m_fRangeFPro;
	static float ** m_fRangeALLPro;
	PolarRangeData m_AccumulatedMovement;
	Particle * m_pParticles;
	int m_nParticleNum;
	double m_dParticleSD;

	void prediction();
	void update();
	void resampling();
	void estimation();
	void CalculateNoOfSamples();
	void Normalizing();

	void generateSensorModel();
	CRangeBasedPFLocalizer_isao::PolarRangeData transRangeDataCartesianToPolar(CRobotState::Position RangePosition);
	void predictRangeData(Particle Sample, vector<PolarRangeData> * pPredictedPolarRangeData  );
	void doRayTracing(int x_1, int x_2, int y_1, int y_2, int *hit_x, int *hit_y);
	bool isTimeToUpdate();
public:

	//Input operation
	inline void setParameter(int nMinParticleNum, int nMaxParticleNum, int nParticleDensity,
		double dDeviationforTrans, double dDeviationforRot, double dDeviationforTransToRot, int nMinParticleSD,
		double dRangeMaxDist, int nRangeInterval)
	{
		this->m_Param.nMinParticleNum = nMinParticleNum;
		this->m_Param.nMaxParticleNum = nMaxParticleNum;
		this->m_Param.nParticleDensity = nParticleDensity;
		this->m_Param.dDeviationforTrans = dDeviationforTrans;
		this->m_Param.dDeviationforRot = dDeviationforRot;
		this->m_Param.dDeviationforTransToRot = dDeviationforTransToRot;
		this->m_Param.nMinParticleSD = nMinParticleSD;
		this->m_Param.dRangeMaxDist = dRangeMaxDist;
		this->m_Param.nRangeInterval = nRangeInterval;
		generateSensorModel();
	}

	inline void setMap(CTWODMapState * pMap) {m_pMap = pMap; }; 
	inline void setRobotPose(CRobotState RobotState) {this->m_RobotState = RobotState;};

	//double dSize = 1.0, double dAngle = 5.0, int nMinSampleNum = 100, int nMaxSampleNum = 1000, int nSampleDensity = 100);
	void initParticles(double x, double y, double th, double dRadiusScope, double dAngleScope);

	inline void setControlInput(CRobotState::Position  DeltaPosition, CRobotState::Orientation  DeltaOrientation) 
	{
		this->m_DeltaPosition = DeltaPosition;
		this->m_DeltaOrientation = DeltaOrientation;
		m_AccumulatedMovement.r += sqrt(m_DeltaPosition.x *m_DeltaPosition.x +m_DeltaPosition.y *m_DeltaPosition.y);
		m_AccumulatedMovement.th += fabs(m_DeltaOrientation.yaw);
	};

	inline void setControlInput(double dDeltaX, double dDeltaY, double dDeltaYaw) 
	{ 
		m_DeltaPosition.x = dDeltaX;
		m_DeltaPosition.y = dDeltaY;
		m_DeltaPosition.z = 0.0;

		m_DeltaOrientation.roll = 0.0;
		m_DeltaOrientation.pitch = 0.0;
		m_DeltaOrientation.yaw = dDeltaYaw;

		m_AccumulatedMovement.r += sqrt(m_DeltaPosition.x *m_DeltaPosition.x +m_DeltaPosition.y *m_DeltaPosition.y);
		m_AccumulatedMovement.th += fabs(m_DeltaOrientation.yaw);
	};
	//inline void setRangeData(vector<CRobotState::Position> * pRangeData) {m_pRangeData = pRangeData;};
	void setRangeData(vector<CRobotState::Position> RangeData) ;
	void setFeatureData(vector<int> i_feature_vec);


	//Output operation
	inline CRobotState getState() {return this->m_RobotState; };
	//inline CRobotState getState() { return this->m_PFRobotState; };
	inline Particle * getParticles() {return this->m_pParticles; };
	inline int getParticlesNum() {return this->m_nParticleNum;};
	inline double getParticlesSD() {return this->m_dParticleSD;};
	//Algorithm
	void estimateState();
	float getFeatureValue(float x, float y);

public:
	CRangeBasedPFLocalizer_isao(void);
	~CRangeBasedPFLocalizer_isao(void);
};

#endif


