#include "StdAfx.h"
#include "RangeBasedPFLocalizer_isao.h"

float ** CRangeBasedPFLocalizer_isao::m_fRangeMCLPro = NULL;
float ** CRangeBasedPFLocalizer_isao::m_fRangeFPro = NULL;
float ** CRangeBasedPFLocalizer_isao::m_fRangeALLPro = NULL;

CRangeBasedPFLocalizer_isao::CRangeBasedPFLocalizer_isao(void)
{
		m_RobotState.initialize();
		m_RobotState.setCovarianceDimension(3);
		m_AccumulatedMovement.r = 0.0;
		m_AccumulatedMovement.th = 0.0;
		m_dParticleSD = 0.0;
}


CRangeBasedPFLocalizer_isao::~CRangeBasedPFLocalizer_isao(void)
{

	delete  m_pParticles;
		double dMaxDistanceMM = (double)m_Param.dRangeMaxDist*1000.0;
		for( int i=0;i<(int)(dMaxDistanceMM/10.0)+1;i++)
		{
			free(CRangeBasedPFLocalizer_isao::m_fRangeMCLPro[i]); 
		}
		free (CRangeBasedPFLocalizer_isao::m_fRangeMCLPro);	

}

void CRangeBasedPFLocalizer_isao::generateSensorModel()
{
	int i;
	float s;
	float fTemp;
	float fMaxValue;
	float fValueAtMaxRange;

	//GetPrivateProfileString("MCL","Sigma","1.0",buf,sizeof(buf),inifile);
	s = 1.0;//atof(buf);
	printf("Parameter loading from Parameters.ini....\n(Sigma = %.2f)\n", s);

	double dMaxDistanceMM = (double)m_Param.dRangeMaxDist*1000.0;


	CRangeBasedPFLocalizer_isao::m_fRangeMCLPro = (float**)calloc((int)(dMaxDistanceMM/10.0)+1,sizeof(float*));

	for(i=0;i<(int)(dMaxDistanceMM/10.0)+1;i++) {
		CRangeBasedPFLocalizer_isao::m_fRangeMCLPro[i] = (float*)calloc((int)(dMaxDistanceMM/10.0)+1,sizeof(float));
	}	
	int nnn = (int)((dMaxDistanceMM/10.0)+1);
// 	fMaxValue = (float)0.05;
// 	
// 	for(i=0;i<(int)(dMaxDistanceMM/10.0) + 1;i++)  // expected distance	
// 	{
// 		for(int j=0;j<(int)(dMaxDistanceMM / 10.0)+1;j++)  // measured distance
// 		{	
// 			CRangeBasedPFLocalizer_isao::m_fRangeMCLPro[i][j] = 0.001 + fMaxValue*exp(-((float)i*0.01-(float)j*0.01)*((float)i*0.01-(float)j*0.01)/2/s/s);
// 		}
// 	}

	CRangeBasedPFLocalizer_isao::m_fRangeFPro = (float**)calloc(256, sizeof(float*));

	for (i = 0; i < 256; i++) {
		CRangeBasedPFLocalizer_isao::m_fRangeFPro[i] = (float*)calloc(256, sizeof(float));
	}
	int mmm = 256;



	float sigma;
	for (int i = 0; i < (int)(dMaxDistanceMM / 10.0) + 1; i++)
	{	// expected distance
		sigma = 0.006f*(float)i*0.01f + 0.150f;

		for (int j = 0; j < (int)(dMaxDistanceMM / 10.0) + 1; j++)
		{	// measured distance
			//m_fRangeMCLPro[i][j] = 0.05*exp(-((float)i*0.01-(float)j*0.01)*((float)i*0.01-(float)j*0.01)/2/sigma/sigma)
			//+ 0.005*((float)m_dMaxDistance-(float)j*10.0)/(float)m_dMaxDistance;

			//CRangeBasedPFLocalizer_isao::m_fRangeMCLPro[i][j] = 1.0f / (sqrt(2.0f*(float)M_PI*sigma))*exp(-((float)i*0.01f - (float)j*0.01f)*((float)i*0.01f - (float)j*0.01f) / 2.0f / sigma / sigma)+ 0.005*((float)dMaxDistanceMM - (float)j*10.0f) / (float)dMaxDistanceMM;
			CRangeBasedPFLocalizer_isao::m_fRangeMCLPro[i][j] = (1.0f / (sigma*(sqrt(2.0f*(float)M_PI))))*exp(-((float)i*0.01f - (float)j*0.01f)*((float)i*0.01f - (float)j*0.01f) / 2.0f / sigma / sigma) + 0.005*((float)dMaxDistanceMM - (float)j*10.0f) / (float)dMaxDistanceMM;

			// 			if(((float)j*10.0)>((float)dMaxDistanceMM-300.0))
			// 				CRangeBasedPFLocalizer_isao::m_fRangeMCLPro[i][j] = (float)
						//CRangeBasedPFLocalizer_isao::m_fRangeMCLPro[i][j] *= (10.0 * 1.5);
		}
	}


	float sigma_h;
	for (int h = 0; h < 256; h++)
	{
		sigma_h = 1.2;
		//sigma_h = 0.006f*(float)h*0.01f + 0.150f;

		for (int k = 0; k < 256; k++)
		{
			CRangeBasedPFLocalizer_isao::m_fRangeFPro[h][k] = (1.0f / (sigma_h*(sqrt(2.0f*(float)M_PI))))*exp(-((float)h*0.01f - (float)k*0.01f)*((float)h*0.01f - (float)k*0.01f) / 2.0f / sigma_h / sigma_h);

		}

	}


	
	//ñﬁìx100Ç©0Ç©
	//for (int h = 0; h < 256; h++)
	//{
	//	for (int k = 0; k < 256; k++)
	//	{
	//		CRangeBasedPFLocalizer_isao::m_fRangeFPro[h][k] = 0;
	//		if(h==k)
	//		CRangeBasedPFLocalizer_isao::m_fRangeFPro[h][k] =100;
	//	}
	//}




	//float sigma_h;
	//for (int i = 1; i < 101; i++)
	//{
	//	sigma_h = i;

	//	for (int h = 0; h < 256; h++)
	//	{
	//		for (int k = 0; k < 256; k++)
	//		{
	//			CRangeBasedPFLocalizer_isao::m_fRangeFPro[h][k] = (1.0f / (sigma_h*(sqrt(2.0f*(float)M_PI))))*exp(-((float)h*0.01f - (float)k*0.01f)*((float)h*0.01f - (float)k*0.01f) / 2.0f / sigma_h / sigma_h);

	//		}
	//}

	//}

	
	printf("Sensor model of MCL was completely generated.\n");
}



//É}ÉbÉvÇ…ëÆê´ílÇíuÇ¢ÇƒÇ¢ÇÈÉ]Å[Éì
float CRangeBasedPFLocalizer_isao::getFeatureValue(float x, float y)
{   //07floorcut
	//float feature = 0.;
	//if (10 <= x && x <= 15 && 1 <= y && y <= 2)feature = 200.;
	//else if (21 <= x && x <= 23 && 7 <= y && y <= 8)feature = 200.;

	
	//07floorcut2 åÎç∑Ç»Çµ
	//float feature = 0.;
	//if (27 <= x && x <= 31 && 5 <= y && y <= 6)feature = 200.;
	//else if (34 <= x && x <= 36 && 3 <= y && y <= 4)feature = 200.;
	//else if (37 <= x && x <= 39 && 3 <= y && y <= 4)feature = 200.;
	//else if (40 <= x && x <= 42 && 3 <= y && y <= 4)feature = 200.;
	//else if (40 <= x && x <= 42 && 5 <= y && y <= 6)feature = 200.;
	//else if (44 <= x && x <= 49 && 5 <= y && y <= 6)feature = 200.;
	//else if (53 <= x && x <= 55 && 5 <= y && y <= 6)feature = 200.;
	//Å™ÉhÉA
	//else if (59 <= x && x <= 60 && 2 <= y && y <= 4)feature = 200.;
	//else if (61 <= x && x <= 64 && 10 <= y && y <= 11)feature = 200.;
	//Å™start(2,8,-90)
	//else if (0 <= x && x <= 1 && 3 <= y && y <= 6)feature = 200.;
	//else if (10 <= x && x <= 13 && 5 <= y && y <= 6)feature = 200.;
	//else if (20 <= x && x <= 23 && 3 <= y && y <= 4)feature = 200.;
	//Å™ÅyãtëñÅzstart(68,9.5)
	//if (0 <= x && x <= 5 && 3 <= y && y <= 6)feature = 200.;
	//else if (15 <= x && x <= 20 && 4 <= y && y <= 7)feature = 200.;
	//else if (39 <= x && x <= 47 && 3 <= y && y <= 5)feature = 200.;
	//else if (56 <= x && x <= 63 && 7 <= y && y <= 13)feature = 200.;

	////07floorcut2 åÎç∑Ç†ÇË
	//float feature = CCalculus::getInstance()->getRandomValue(10) + 10;//ëÆê´íl(0Å`20)
	//if (27 <= x && x <= 31 && 5 <= y && y <= 6)feature = CCalculus::getInstance()->getRandomValue(10) + 200;//ëÆê´íl(190Å`200);
	//else if (34 <= x && x <= 36 && 3 <= y && y <= 4)feature = CCalculus::getInstance()->getRandomValue(10) + 200;//ëÆê´íl(190Å`200);
	//else if (37 <= x && x <= 39 && 3 <= y && y <= 4)feature = CCalculus::getInstance()->getRandomValue(10) + 200;//ëÆê´íl(190Å`200);
	//else if (40 <= x && x <= 42 && 3 <= y && y <= 4)feature = CCalculus::getInstance()->getRandomValue(10) + 200;//ëÆê´íl(190Å`200);
	//else if (40 <= x && x <= 42 && 5 <= y && y <= 6)feature = CCalculus::getInstance()->getRandomValue(10) + 200;//ëÆê´íl(190Å`200);
	//else if (44 <= x && x <= 49 && 5 <= y && y <= 6)feature = CCalculus::getInstance()->getRandomValue(10) + 200;//ëÆê´íl(190Å`200);
	//else if (53 <= x && x <= 55 && 5 <= y && y <= 6)feature = CCalculus::getInstance()->getRandomValue(10) + 200;//ëÆê´íl(190Å`200);
	////Å™ÉhÉA
	//else if (0 <= x && x <= 1 && 3 <= y && y <= 6)feature = CCalculus::getInstance()->getRandomValue(10) + 200;//ëÆê´íl(190Å`200);
	//else if (10 <= x && x <= 13 && 5 <= y && y <= 6)feature = CCalculus::getInstance()->getRandomValue(10) + 200;//ëÆê´íl(190Å`200);
	//else if (20 <= x && x <= 23 && 3 <= y && y <= 4)feature = CCalculus::getInstance()->getRandomValue(10) + 200;//ëÆê´íl(190Å`200);
	////Å™ÅyãtëñÅzstart(68,9.5)

	//07floorcut2 åÎç∑Ç»Çµ Simulation03
	float feature = 0.;
	if (0 <= x && x <= 1 && 3 <= y && y <= 4)feature = 200.;//1
	//else if (5 <= x && x <= 9 && 3 <= y && y <= 4)feature = 200.;//2
	else if (11 <= x && x <= 12 && 3 <= y && y <= 4)feature = 200.;//3
	else if (22 <= x && x <= 24 && 3 <= y && y <= 4)feature = 200.;//4
	else if (33 <= x && x <= 36 && 3 <= y && y <= 4)feature = 200.;//5
	else if (37 <= x && x <= 40 && 3 <= y && y <= 4)feature = 200.;//6
	else if (44 <= x && x <= 46 && 3 <= y && y <= 4)feature = 200.;//7
	else if (52 <= x && x <= 55 && 3 <= y && y <= 4)feature = 200.;//8
	//Å™â∫ÇÃòLâ∫
	else if (57 <= x && x <= 59 && 1 <= y && y <= 2)feature = 200.;//9
	//àÍî‘â∫
	else if (59 <= x && x <= 60 && 3 <= y && y <= 6)feature = 200.;//10
	//
	else if (62 <= x && x <= 65 && 10 <= y && y <= 11)feature = 200.;//11
	else if (68 <= x && x <= 70 && 10 <= y && y <= 11)feature = 200.;//12
	//
	else if (68 <= x && x <= 70 && 8 <= y && y <= 9)feature = 200.;//13
	//
	//else if (59 <= x && x <= 60 && 12 <= y && y <= 16)feature = 200.;//14
	else if (59 <= x && x <= 60 && 19 <= y && y <= 20)feature = 200.;//15
	//
	else if (57 <= x && x <= 58 && 10 <= y && y <= 12)feature = 200.;//16
	//
	else if (35 <= x && x <= 37 && 5 <= y && y <= 6)feature = 200.;//17

	else if (27 <= x && x <= 30 && 5 <= y && y <= 6)feature = 200.;//18
	else if (13 <= x && x <= 16 && 5 <= y && y <= 6)feature = 200.;//19
	//
	else if (1 <= x && x <= 3 && 9 <= y && y <= 10)feature = 200.;//20

	

	return feature;
}


void CRangeBasedPFLocalizer_isao::estimateState()
{
	// 1. prediction
	prediction();
	
 	if(isTimeToUpdate())
	{
		// 2. update
		update();
	
		Normalizing();

		// 3. resampling
		resampling();
	}
		
 	// 4. estimation
 	estimation();
}


// void CRangeBasedPFLocalizer_isao::prediction()
// {
// 	double dEncoderDelta2[3];
// 	double dDeltaDistance, dDeltaTheta;
// 	int nCnt = 0;
// 	double s1,c1;
// 	double dPro;
// 	double dProTrans, dProTransRotate, dProRotate;
// 	double d1, d2, d3;
// 	double dNoiseTrans, dNoisexTrans, dNoiseyTrans,dNoiseTransRotate, dNoiseRotate;
// 	double dTransdistance;
// 	
// 	double	dMaxProForUpdate = 0.8;
// 	double dMinProForUpdate = 0.2;
// 	d1 = m_Param.dDeviationforTrans;
// 	d2 = m_Param.dDeviationforTransToRot;
// 	d3 = m_Param.dDeviationforRot;
// 
// 
// 	// Case that there are no encoder data, deactivating this function.
// 	if (m_DeltaPosition.x==0.0 && m_DeltaPosition.y==0.0 && m_DeltaOrientation.yaw==0.0) return;
// 
// 	double dDX = m_DeltaPosition.x*1000.0;
// 	double dDY = m_DeltaPosition.y*1000.0;
// 	double dDTh = m_DeltaOrientation.yaw;
// 	
// 	// Add noise. Motion model driven by Konolige
// 	int nX = 0;
// 	int nY = 0;
// 	for (int i=0; i<m_nParticleNum; i++) 
// 	{
// 		dTransdistance = sqrt(pow(dDX,2)+pow(dDY,2));			// ¡˜º±¿Ãµø∞≈∏Æ
// 		dNoiseTrans = CCalculus::getInstance()->getRandomValue(dTransdistance*d1);	// x πÅEEø¿¬˜¥¬ x πÅEE¿Ãµø∑Æø°∏∏ ∫Ò∑ .
// 		dNoiseTransRotate = CCalculus::getInstance()->getRandomValue(dTransdistance*d2);	// ¡˜º±ø˚—øø° µ˚∏• ∞¢µµø¿¬ÅE ¿Ãµø∞≈∏Æø° ∫Ò∑ «œø© ¡ı∞°«—¥Ÿ.
// 		dNoiseRotate = CCalculus::getInstance()->getRandomValue(dDTh*d3);
// 		dDeltaDistance = dTransdistance + dNoiseTrans;
// 		dDeltaTheta = dDTh			// ∞¢µµ ø¿¬˜¥¬,
// 		+ dNoiseTransRotate	// ¡˜º±¿Ãµøø° ¿««— ∞¢µµø¿¬˜øÕ
// 			+ dNoiseRotate;		// »∏¿ÅE?¿««— ∞¢µµø¿¬˜∏¶ ≈ÅE?±∏«ÿ¡¯¥Ÿ.
// 
// 		s1 = sin(   m_pParticles[i].th + dDTh + dNoiseRotate ); // yaw
// 		c1 = cos(   m_pParticles[i].th + dDTh + dNoiseRotate  ); // yaw
// 
// 		// ø¿¬˜∞° ∆˜«‘µ» ø£ƒ⁄¥ÅE¡§∫∏∏¶ ¿ÃøÅEœø?ª˘«√¿« ¿˝¥ÅE¬«•∞ÅEªÛø°º≠¿« ¿Ãµø¿ª ∞ËªÅE
// 		// Calculation of the movement with encoder data and errors in the the absoulte coordinate
// 
// 		m_pParticles[i].x += (c1*dDeltaDistance)*0.001;
// 		m_pParticles[i].y += (s1*dDeltaDistance)*0.001;
// 		m_pParticles[i].th += dDeltaTheta;
// 		if (m_pParticles[i].th>M_PI) m_pParticles[i].th -=2*M_PI;
// 		else if (m_pParticles[i].th<-M_PI) m_pParticles[i].th +=2*M_PI;
// 
// 		// ¿Ãµø∑Æø° µ˚∏• »Æ∑ÅE∞ªΩ≈.
// 		// ø£ƒ⁄¥ı∑Œ øπ√¯«— ∞™¿Ã ¡§»Æ«œ¥Ÿ∏ÅEø£ƒ⁄¥ÅE∫Ø»≠∑Æ∞ÅE≥ÅEÃ¡˚Ã?√ﬂ∞°µ» ∫Ø»≠∑Æ¿Ã ¿œƒ°«ÿæﬂ «œπ«∑Œ,
// 		// ø£ƒ⁄¥ÅE∫Ø»≠∑Æ∞ÅE≥ÅEÃ¡˚Ã?√ﬂ∞°µ» ∫Ø»≠∑Æ ªÁ¿Ã¿« ¬˜¿Ã∞° ¿€¿ªºˆ∑œ ≥Ù¿∫ »Æ∑ÅE?∫Œø©.
// 		dProTrans = CCalculus::getInstance()->getGaussianValue(dTransdistance*d1, dNoiseTrans);
// 		if (dProTrans<dMinProForUpdate)
// 			dProTrans = dMinProForUpdate;
// 		if (dProTrans>dMaxProForUpdate)
// 			dProTrans = dMaxProForUpdate;
// 		dProTransRotate = CCalculus::getInstance()->getGaussianValue(dTransdistance*d2, dNoiseTransRotate);
// 		if (dProTransRotate<dMinProForUpdate)
// 			dProTransRotate = dMinProForUpdate;
// 		if (dProTransRotate>dMaxProForUpdate)
// 			dProTransRotate = dMaxProForUpdate;
// 		dProRotate = CCalculus::getInstance()->getGaussianValue(dDTh*d3, dNoiseRotate);
// 		if (dProRotate<dMinProForUpdate)
// 			dProRotate = dMinProForUpdate;
// 		if (dProRotate>dMaxProForUpdate)
// 			dProRotate = dMaxProForUpdate;
// 
// 		m_pParticles[i].dPro *= dProTrans*dProTransRotate*dProRotate;
// 
// 
// 		nX = (int)(m_pParticles[i].x/m_pMap->getCellSize());
// 		nY = (int)(m_pParticles[i].y/m_pMap->getCellSize());
// 
// // 		if(			nX <= 0 || nX >= m_pMap->getMapSizeX() || nY <=0 || nY >= m_pMap->getMapSizeY() 			)
// // 		{
// // 			m_pParticles[i].dPro = 0.0;
// // 			continue;
// // 		}
// 			
// 
// 		if(m_pMap->get2DGridMap()[nX][nY] == CTWODMapState::UNKNOWN ||
// 			nX <= 0 || nX >= m_pMap->getMapSizeX() || nY <=0 || nY >= m_pMap->getMapSizeY() 			)
// 			m_pParticles[i].dPro = 0.0;
// 
// 	}
// 
// 
// }


void CRangeBasedPFLocalizer_isao::prediction()
{
	double dEncoderDelta2[3];
	int nCnt = 0;
	double s1,c1;
	double dPro;
	double d1, d2, d3;
	double dNoise;
	d1 = m_Param.dDeviationforTrans;
	d2 = m_Param.dDeviationforTransToRot;
	d3 = m_Param.dDeviationforRot;


	// Case that there are no encoder data, deactivating this function.
	if (m_DeltaPosition.x==0.0 && m_DeltaPosition.y==0.0 && m_DeltaOrientation.yaw==0.0) return;

	double dDX = m_DeltaPosition.x*1000.0;
	double dDY = m_DeltaPosition.y*1000.0;
	double dDTh = m_DeltaOrientation.yaw;

	// Add noise. Motion model driven by Konolige
	int nX = 0;
	int nY = 0;
	for (int i=0; i<m_nParticleNum; i++) 
	{

		//cout <<"D: " <<  m_pParticles[i].th*R2D << endl;
		dNoise = CCalculus::getInstance()->getRandomValue(dDX*d1);	
		//		if (dNoise>0) dNoise /= 2.0;
		dEncoderDelta2[0] = dDX+ dNoise;
		dEncoderDelta2[1] = dDY;
		dEncoderDelta2[2] = dDTh							// Angular error
			+ CCalculus::getInstance()->getRandomValue(dDX*d2)	// x directio movement
			+ CCalculus::getInstance()->getRandomValue(dDTh*d3);

		s1 = sin(   m_pParticles[i].th + dEncoderDelta2[2]/2.0   ); // yaw
		c1 = cos(   m_pParticles[i].th + dEncoderDelta2[2]/2.0   ); // yaw


		m_pParticles[i].x += (c1*dEncoderDelta2[0] + (-s1)*dEncoderDelta2[1])*0.001;
		m_pParticles[i].y += (s1*dEncoderDelta2[0] + (c1)*dEncoderDelta2[1])*0.001;
		m_pParticles[i].th += dEncoderDelta2[2];
		if (m_pParticles[i].th>M_PI) m_pParticles[i].th -=2*M_PI;
		else if (m_pParticles[i].th<-M_PI) m_pParticles[i].th +=2*M_PI;

		nX = (int)(m_pParticles[i].x/m_pMap->getCellSize());
		nY = (int)(m_pParticles[i].y/m_pMap->getCellSize());

		// 		if(			nX <= 0 || nX >= m_pMap->getMapSizeX() || nY <=0 || nY >= m_pMap->getMapSizeY() 			)
		// 		{
		// 			m_pParticles[i].dPro = 0.0;
		// 			continue;
		// 		}


		if(m_pMap->get2DGridMap()[nX][nY] == CTWODMapState::UNKNOWN ||
			nX <= 0 || nX >= m_pMap->getMapSizeX() || nY <=0 || nY >= m_pMap->getMapSizeY() 			)
			m_pParticles[i].dPro = 0.0;

	}


}


void CRangeBasedPFLocalizer_isao::update()
{

	vector<PolarRangeData>  * pPredictedPolarRangeData = new vector<PolarRangeData>  ;
	int nCnt = 0;	// the number of samples

	int nPre = 0; int nReal = 0;
	for(unsigned int i = 0 ; i < m_nParticleNum ; i++)
	{
		//ä÷êîpredictRangeDataÇÕm_pParticles[i]ÇéÛÇØéÊÇ¡ÇƒpPredictedPolarRangeDataÇéZèoÇµÇƒÇ¢ÇÈÅD
		pPredictedPolarRangeData->clear();
		predictRangeData(  m_pParticles[i],  pPredictedPolarRangeData);
		nCnt = 0;
		m_pParticles[i].dUpdatePro = 0.0;	

		for (int j=0;j<pPredictedPolarRangeData->size() ; j++ ) 
		{

// 			if (pPredictedPolarRangeData->at(j).r  == NULL || pPredictedPolarRangeData->at(j).r  == 0)
// 				continue;

			nPre = (int)(0.5 +pPredictedPolarRangeData->at(j).r *100.0);
			nReal = (int)(0.5 + m_PolarRangeData.at(j).r *100.0);

			if(nPre == 0) continue;
			if (nReal < 22 ) continue;
			if (nReal > m_Param.dRangeMaxDist * 100) continue;

			//è]óàÇÃñﬁìx
			//m_pParticles[i].dUpdatePro += (double)m_fRangeMCLPro[(int)(0.5 + pPredictedPolarRangeData->at(j).r *100.0)][(int)(0.5 + m_PolarRangeData.at(j).r *100.0)];
			
			//êVÇµÇ¢ëÆê´ílÇçló∂ÇµÇΩñﬁìx
			m_pParticles[i].dUpdatePro += (double)m_fRangeMCLPro[(int)(0.5 +pPredictedPolarRangeData->at(j).r *100.0)/*âºëzìIÇ»ãóó£*/]
				[(int)(0.5 + m_PolarRangeData.at(j).r *100.0)/*äœë™ÇµÇΩãóó£*/]                                            
				* m_fRangeFPro[(int)getFeatureValue(m_HitPointPositions[j].x, m_HitPointPositions[j].y)/*âºëzìIÇ»ëÆê´íl*/]
					[m_i_feature_vec[j]/*äœë™ÇµÇΩëÆê´íl*/];                                                                         
			nCnt++;
			//		m_stSample[i].dRangeUpdatePro *=pow((double)m_fRangeMCLPro[(int)(0.5 + m_dEstimatedRangeData[j]/10.0)][(int)(0.5 + m_dRangeData[j]/10.0)],0.02); // 0.2
		}

		if (nCnt>0)
		{
			m_pParticles[i].dUpdatePro /= (double)nCnt;
			//m_stSample[i].dMatchingError /= (double)nCnt;	//added CYK
			//m_dMinMatchingError += m_stSample[i].dMatchingError;
		}


		m_pParticles[i].dPro = m_pParticles[i].dPro * m_pParticles[i].dUpdatePro;
	}
	//cout << "pRangeDataSize : " << pPredictedPolarRangeData->size() << endl;
	delete pPredictedPolarRangeData;

}

void CRangeBasedPFLocalizer_isao::resampling()
{
	struct Accumulation{	
		double dSum;
		double x;
		double y;
		double th;
	};
	Accumulation *pAccumulation;

	double dSum = 0.0;
	int nNoOfAccumulated;
	int nCnt;
	double dRandPro;
	int i;

	pAccumulation = NULL;
	pAccumulation = (Accumulation*)calloc(m_nParticleNum,sizeof(Accumulation));

	//TRACE("SD: %.3f\n", m_dSampleSD);
// 	if (m_dParticleSD<m_Param.nMinParticleSD)
// 	{
// 		CalculateNoOfSamples(false);
// 		cout<<"[CLaserBasedParticleFilter] : No sensor update\n"<<endl;
// 	}
// 	else
	{
		nNoOfAccumulated=0;
		for (i =0; i<m_nParticleNum; i++) 
		{
			dSum += m_pParticles[i].dPro;
			if (m_pParticles[i].dPro==0.0) continue;
			pAccumulation[nNoOfAccumulated].dSum = dSum;
			pAccumulation[nNoOfAccumulated].x = m_pParticles[i].x;
			pAccumulation[nNoOfAccumulated].y = m_pParticles[i].y;
			pAccumulation[nNoOfAccumulated].th = m_pParticles[i].th;
			nNoOfAccumulated++;
		}

		if(dSum == 0.0) 		
		{
			for (int i =0; i<m_nParticleNum; i++) 	
				m_pParticles[i].dPro =  1.0/(double)m_nParticleNum;
			return ;
		}
		CalculateNoOfSamples();

		int nTemp=0;

		while(nTemp<m_nParticleNum) {
			nCnt = 0;
			dRandPro = (double)rand()/(double)RAND_MAX;
			while (pAccumulation[nCnt].dSum<dRandPro && nCnt<nNoOfAccumulated-1){
				nCnt++;
			}
			m_pParticles[nTemp].x = pAccumulation[nCnt].x;
			m_pParticles[nTemp].y = pAccumulation[nCnt].y;
			m_pParticles[nTemp].th = pAccumulation[nCnt].th;
			m_pParticles[nTemp].dPro =  1.0/(double)m_nParticleNum;
			nTemp++;
		}
	}
	free (pAccumulation);
	pAccumulation = NULL;
}

void CRangeBasedPFLocalizer_isao::Normalizing()
{
	double dSum = 0.0;
    int nCnt = 0;
    double dMax=0.0;

	for(int i=0; i<m_nParticleNum; i++)
		dSum += m_pParticles[i].dPro;
/*
	if (dSum==0.0) {
		for(int i=0; i<m_nSampleNum; i++)
			m_stSample[i].dPro = 1.0/(double)m_nSampleNum;
	} else {
		for(int i=0; i<m_nSampleNum; i++)
			m_stSample[i].dPro = m_stSample[i].dPro / dSum;
	}
*/
	if(dSum !=0.0){
		for(int i=0; i<m_nParticleNum; i++)
			m_pParticles[i].dPro = m_pParticles[i].dPro / dSum;
	}
}

CRangeBasedPFLocalizer_isao::PolarRangeData CRangeBasedPFLocalizer_isao::transRangeDataCartesianToPolar(CRobotState::Position RangePosition)
{
	PolarRangeData RangeData;
	double r = sqrt(  RangePosition.x*RangePosition.x + RangePosition.y*RangePosition.y  );
	if((RangePosition.x == 0.0 && RangePosition.y == 0.0) ||  r > m_Param.dRangeMaxDist)
	{
		RangeData.r = NULL;
		RangeData.th = NULL;
	}
	else
	{
		RangeData.r = r;
		RangeData.th = atan2(RangePosition.y , RangePosition.x );
		
	}
	return RangeData;
}



void CRangeBasedPFLocalizer_isao::predictRangeData(Particle Sample, vector<PolarRangeData> * pPredictedPolarRangeData  )
{
	double x = Sample.x; double y = Sample.y; double t = Sample.th;

	int nPosX1=0,nPosY1=0;
	int nPosX2=0,nPosY2=0;
	int nHitX=0, nHitY=0;	// ÉåÉCÉgÉåÅ[ÉVÉìÉO

	double dAngle=0.;
	double dPosTh=0.;

	double dCellSize = m_pMap->getCellSize();

	nPosX1 = (int)(x  /dCellSize);		
	nPosY1 = (int)(y  / dCellSize);			


	
	PolarRangeData PredictedData; PredictedData.r = 0.0; PredictedData.th = 0.0;
	//m_HitPointPositions
	m_HitPointPositions.clear();
	for(unsigned int i = 0 ; i < m_PolarRangeData.size() ; i++)
	{

		nPosX2 = nPosX1 + (int)(m_Param.dRangeMaxDist*cos(Sample.th+m_PolarRangeData[i].th)/dCellSize);		
		nPosY2 = nPosY1 + (int)(m_Param.dRangeMaxDist*sin(Sample.th+m_PolarRangeData[i].th)/dCellSize);	



		doRayTracing(nPosX1, nPosX2, nPosY1, nPosY2, &nHitX, &nHitY); // Ray-tracing ºÅEE
		CRobotState::Position HitPointPosition;
		HitPointPosition.x = nHitX * dCellSize;
		HitPointPosition.y = nHitY * dCellSize;
		m_HitPointPositions.push_back(HitPointPosition);


		if (nHitX<=10 || nHitX >= (m_pMap->getMapSizeX()-10) || nHitY<=10 || nHitY >=(m_pMap->getMapSizeY()-10)) 
		{
			PredictedData.r = NULL;
			PredictedData.th = NULL;
			
		}

		//20210131
		if (nHitX == 0 && nHitY == 0)
		{
			PredictedData.r = NULL;
			PredictedData.th = NULL;
		}

// 		else if (m_PolarRangeData[i].th == NULL && m_PolarRangeData[i].r == NULL)
// 		{
// 			PredictedData.r = NULL;
// 			PredictedData.th = NULL;
// 		}
		else
		{
			PredictedData.r = sqrt((double)(nHitX*dCellSize-nPosX1*dCellSize)*(nHitX*dCellSize-nPosX1*dCellSize)+
				(double)(nHitY*dCellSize-nPosY1*dCellSize)*(nHitY*dCellSize-nPosY1*dCellSize )	);
			PredictedData.th = m_PolarRangeData[i].th;

			//nHiXx,nHitY*dCellSize
		}
		pPredictedPolarRangeData->push_back(PredictedData);
	}
}


/**
 @brief : Function executing ray-tracing with vector of samples' position
*/
void CRangeBasedPFLocalizer_isao::doRayTracing(int x_1, int x_2, int y_1, int y_2, int *hit_x, int *hit_y)
{
	*hit_x == 0;
	*hit_y == 0;

	int eps = 0;
	int x_thres=x_2, y_thres=y_2;
/////////////////////////////////////////////////////////////////////
	if(x_1 < 0) x_1 = 0;
	else if(x_1 >= m_pMap->getMapSizeX()) x_1 = m_pMap->getMapSizeX()-1;
	if(y_1 < 0) y_1 = 0;
	else if(y_1 >= m_pMap->getMapSizeY()) y_1 = m_pMap->getMapSizeY()-1;

	
	//if(x_thres < 0) x_thres = 0;
	//else if(x_thres >= m_pMap->getMapSizeX()) x_thres = m_pMap->getMapSizeX()-1;
	//if(y_thres < 0) y_thres = 0;
	//else if(y_thres >= m_pMap->getMapSizeY()) y_thres = m_pMap->getMapSizeY()-1;
/////////////////////////////////////////////////////////////////////
		int x = x_1,y = y_1;

		int delx = x_thres - x_1;
		int dely = y_thres - y_1;


	if(delx > 0) {
		if(dely >=0) {
			if(delx > dely) {
				for(int x=x_1; x<=x_thres; x++) {
  					if(m_pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
    					
						*hit_x = x;
					    	*hit_y = y;
    						return;
  					}

  					eps += dely;
  					if((eps<<1) >= delx) {
  						y++;
    					eps -= delx;
  					}
				}

			}
			else { 

				for(y=y_1; y<=y_thres; y++) {
					if(m_pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN  || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
    					
						*hit_x = x;
						*hit_y = y;
						return;
  					}

  					eps += delx;
  					if((eps<<1) >= dely) {
    						x++;
    						eps -= dely;
  					}
				}

			}

		}
		else { 
			if(delx > -dely) {
				for(x=x_1; x<=x_thres; x++) {
  					if(m_pMap->get2DGridMap()[x][y] ==CTWODMapState::UNKNOWN  || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
    				
					       *hit_x = x;
					       *hit_y = y;
    					       return;
  					}

					eps += dely;
					if((eps<<1) <= -delx) {
						y--;
						eps += delx;
  					}
				}

			}
			else { 

				for(y=y_1; y>=y_thres; y--) {
  					if(m_pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN  || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
    				
						*hit_x = x;
						*hit_y = y;
						return;
					}

  					eps += delx;
  					if((eps<<1) >= -dely) {
    						x++;
    						eps += dely;
  					}
				}

			}
		}
	}

	else { 
		if(dely >= 0) {
			if(-delx > dely) {

				for(x=x_1; x>=x_thres; x--) {
					if(m_pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
				
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += dely;
					if((eps<<1) >= -delx) {
						y++;
						eps += delx;
					}
				}

			}
			else { 

				for(y=y_1; y<=y_thres; y++) {
					if(m_pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN  || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
					
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += delx;
					if((eps<<1) <= -dely) {
						x--;
						eps += dely;
					}
				}

			}
		}
		else { 
			if(-delx > -dely) {
				for(x=x_1; x>=x_thres; x--) {
					if(m_pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN  || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
						
							*hit_x = x;
							*hit_y = y;
							return;
					}

					eps -= dely;
					if((eps<<1) > -delx) {
						y--;
						eps += delx;
					}
				}

			}
			else { 
				for(y=y_1; y>=y_thres; y--) {
					if(m_pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN  || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
					
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps -= delx;
					if((eps<<1) >= -dely) {
						x--;
						eps += dely;
					}
				}

			}
		}
	}

}


 //m_pMap->get2DGridMap()[nStartPosition[0]][nStartPosition[1]] == CTWODMapState::OCCUPIED


void CRangeBasedPFLocalizer_isao::CalculateNoOfSamples()
{
	double dAvgPose[3], dSD[3];		


		dAvgPose[0] = 0.0;
		dAvgPose[1] = 0.0;
		dAvgPose[2] = 0.0;
		dSD[0] = 0.0;
		dSD[1] = 0.0;
		dSD[2] = 0.0;

		for(int i = 0; i<m_nParticleNum; i++) {
			dAvgPose[0] += m_pParticles[i].x*m_pParticles[i].dPro;
			dAvgPose[1] += m_pParticles[i].y*m_pParticles[i].dPro;
			dAvgPose[2] += m_pParticles[i].th*m_pParticles[i].dPro;
		}
		for(int i = 0; i<m_nParticleNum; i++) {
			dSD[0] += (m_pParticles[i].x-dAvgPose[0])*(m_pParticles[i].x-dAvgPose[0])*m_pParticles[i].dPro;
			dSD[1] += (m_pParticles[i].y-dAvgPose[1])*(m_pParticles[i].y-dAvgPose[1])*m_pParticles[i].dPro;
			dSD[2] += (m_pParticles[i].th-dAvgPose[2])*(m_pParticles[i].th-dAvgPose[2])*m_pParticles[i].dPro;
		}
 		m_dParticleSD = sqrt(dSD[0] + dSD[1]);
// 		m_dSampleSDForRotation = sqrt(dSD[2]);

		dSD[0] = sqrt(dSD[0]);
		dSD[1] = sqrt(dSD[1]);

		m_nParticleNum = (int)( 1.0*(dSD[0]*dSD[1])*m_Param.nParticleDensity );

		if (m_nParticleNum > m_Param.nMaxParticleNum) m_nParticleNum = m_Param.nMaxParticleNum;
		if (m_nParticleNum < m_Param.nMinParticleNum) m_nParticleNum = m_Param.nMinParticleNum;
	
}

void CRangeBasedPFLocalizer_isao::estimation()
{
	double dSum = 0.0;
	// 	if (m_dSamplePos[2]>120*D2R || m_dSamplePos[2]<-120*D2R) bAngleCalculationMode = true;
	// 	else bAngleCalculationMode = false;
	double dWSumX = 0.0;
	double dWSumY = 0.0;
	double dWSumZ = 0.0;
	// calculating robot pose by averaging poses of samples
	double dx, dy;
	dx = 0; dy = 0;
	for(int i=0; i<m_nParticleNum; i++)  
	{
		dSum += m_pParticles[i].dPro;
		dWSumX += m_pParticles[i].x*m_pParticles[i].dPro;
		dWSumY += m_pParticles[i].y*m_pParticles[i].dPro;
		
		//if(bAngleCalculationMode && m_stSample[i].t<0) m_dSamplePos[2] += (m_stSample[i].t+2.0*M_PI)*m_stSample[i].dPro;
		//else m_dSamplePos[2] += m_stSample[i].t*m_stSample[i].dPro;
		dx += cos(m_pParticles[i].th)*m_pParticles[i].dPro;
		dy += sin(m_pParticles[i].th)*m_pParticles[i].dPro;
	}
	double dCov[3][3] = {0};
	double xe = 0.0;
	double ye = 0.0;
	double te = 0.0;

	for(int i=0; i<m_nParticleNum; i++)  
	{
		xe = (m_pParticles[i].x - dWSumX/dSum);
		ye =  (    m_pParticles[i].y - dWSumY/dSum);
		te =  (     atan2(sin(m_pParticles[i].th), cos(m_pParticles[i].th)) - atan2(dy/dSum, dx/dSum) );
	
		 dCov[0][0] += xe*xe;   dCov[0][1] += xe*ye;   dCov[0][2] += xe*te;
		 dCov[1][0] += xe*ye;   dCov[1][1] += ye*ye;   dCov[1][2] += ye*te;
		 dCov[2][0] += te*xe;   dCov[2][1] += te*ye;   dCov[2][2] += te*te;	
	}

	if (dSum!=0.0) 
	{
		
		m_RobotState.setX(dWSumX/dSum);
		m_RobotState.setY(dWSumY/dSum);
		m_RobotState.setYaw(atan2(dy/dSum, dx/dSum)   );

		m_RobotState.setCovarianceElement(0,0,   dCov[0][0]  / (m_nParticleNum-1));
		m_RobotState.setCovarianceElement(0,1,   dCov[0][1]  / (m_nParticleNum-1));
		m_RobotState.setCovarianceElement(0,2,   dCov[0][2]  / (m_nParticleNum-1));

		m_RobotState.setCovarianceElement(1,0,   dCov[1][0]  / (m_nParticleNum-1));
		m_RobotState.setCovarianceElement(1,1,   dCov[1][1]  / (m_nParticleNum-1));
		m_RobotState.setCovarianceElement(1,2,   dCov[1][2]  / (m_nParticleNum-1));

		m_RobotState.setCovarianceElement(2,0,   dCov[2][0]  / (m_nParticleNum-1));
		m_RobotState.setCovarianceElement(2,1,   dCov[2][1]  / (m_nParticleNum-1));
		m_RobotState.setCovarianceElement(2,2,   dCov[2][2]  / (m_nParticleNum-1));
	}
}



void CRangeBasedPFLocalizer_isao::initParticles(double x, double y, double th, double dRadiusScope, double dAngleScope)
{
	m_nParticleNum = m_Param.nMaxParticleNum;
	m_pParticles = new Particle[m_nParticleNum];

	int nCnt = 0;
	double dx, dy, dr;

	m_dParticleSD = m_Param.nMinParticleSD;

	
	m_nParticleNum = (int)(dRadiusScope*dRadiusScope*4  *m_Param.nParticleDensity);
	//cout << "Particle Num : " << m_nParticleNum << endl;
	if (m_nParticleNum > m_Param.nMaxParticleNum) m_nParticleNum = m_Param.nMaxParticleNum;
	if (m_nParticleNum < m_Param.nMinParticleNum) m_nParticleNum = m_Param.nMinParticleNum;

	random_device rd;
	mt19937 rEngine(rd());
	uniform_int_distribution<> dist(0, dRadiusScope*1000.0*2);

	random_device rdth;
	mt19937 rEngineth(rdth());
	uniform_int_distribution<> distth(0, dAngleScope*1000.0*2);

	int nGX, nGY; int nmRad;
	int nX = 0; int nY = 0;
	while(nCnt<m_nParticleNum)
	{
		nGX = dist(rEngine);
		nGY = dist(rEngine); //mm
		nmRad = distth(rEngineth);

		m_pParticles[nCnt].x = (x - dRadiusScope)  +  (double)nGX*0.001;
		m_pParticles[nCnt].y = (y - dRadiusScope)  +  (double)nGY*0.001;
		m_pParticles[nCnt].z = 0.0; 
		m_pParticles[nCnt].th = (th - dAngleScope) + (double)nmRad*0.001;
		if (m_pParticles[nCnt].th>M_PI) m_pParticles[nCnt].th -=2*M_PI;
		else if (m_pParticles[nCnt].th<-M_PI) m_pParticles[nCnt].th +=2*M_PI;
		m_pParticles[nCnt].dPro = 1/(double)m_nParticleNum;

		nX = (int)(m_pParticles[nCnt].x/m_pMap->getCellSize());
		nY = (int)(m_pParticles[nCnt].y/m_pMap->getCellSize());

		if( nX <= 0 || nX >= m_pMap->getMapSizeX() || nY <=0 || nY >= m_pMap->getMapSizeY()	)
			continue;

		if(m_pMap->get2DGridMap()[nX][nY] == CTWODMapState::UNKNOWN
			|| m_pMap->get2DGridMap()[nX][nY] == CTWODMapState::OCCUPIED ) continue;

		//cout << m_pParticles[nCnt].x  << " " << m_pParticles[nCnt].y  << " " <<  m_pParticles[nCnt].th << endl;
		nCnt++;
	}
}

void CRangeBasedPFLocalizer_isao::setRangeData(vector<CRobotState::Position> RangeData) 
{
	m_PolarRangeData.clear();
	
	for(int i = 0 ; i < RangeData.size() ; i = i + m_Param.nRangeInterval )
	{
		double r = sqrt(  RangeData.at(i).x*RangeData.at(i).x + RangeData.at(i).y*RangeData.at(i).y  );
		//if (r > 0.22 && r < m_Param.dRangeMaxDist - 0.001)
			m_PolarRangeData.push_back(transRangeDataCartesianToPolar(RangeData.at(i)));
	}

}

void CRangeBasedPFLocalizer_isao::setFeatureData(vector<int> i_feature_vec)
{
	m_i_feature_vec = i_feature_vec;
}

bool CRangeBasedPFLocalizer_isao::isTimeToUpdate()
{
	if(m_PolarRangeData.size() == 0)
		return false;

// 	double dSum = 0.0;
// 	for (int i =0; i<m_nParticleNum; i++) 		dSum += m_pParticles[i].dPro;
// 	if(dSum == 0.0) 		return false;

	//if (m_AccumulatedMovement.r > 0.3 || m_AccumulatedMovement.th > 10.0*D2R) 
	//if (m_AccumulatedMovement.r > 0.5 || m_AccumulatedMovement.th > 20.0*D2R) 
	if (m_AccumulatedMovement.r > 0.15 || m_AccumulatedMovement.th > 5.0*D2R)  //For Mapping?
	{
			m_AccumulatedMovement.r = 0.0;
			m_AccumulatedMovement.th = 0.0;
			return true;
	}
		
	return false;
}