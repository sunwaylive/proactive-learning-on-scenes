#include "GraphCutBasicStruct.h"


CAreaInterest::CAreaInterest(vector<MyPointCloud_RGB_NORMAL> &pointCloud, vector<Normalt> &patchNomal)
{
	validFlag = true;
	mergeFlag = false;
	vecPatchPoint = pointCloud;
	patchNum = vecPatchPoint.size();
	vecPatchNormal = patchNomal;

	thresholdClose0 = 0.005;
	paraConvexK = 0.1;
	paraConvexT = 0.9;
	paraConcave = 1.0;
	paraGeometry = 0.4;
	paraAppearence = 0.3;

	PointCloudPreprocess();
	ComputeSmoothValue();

}

CAreaInterest::~CAreaInterest(void)
{
}

void CAreaInterest::PointCloudPreprocess()
{
	//Bounding box, Color average, Center position
	xMin = yMin = zMin = LARGE_NUM;
	xMax = yMax = zMax = SMALL_NUM;
	for(int i = 0;i <vecPatchPoint.size();i++)
	{
		ColorType color;
		MyPoint point;
		vector<int> vecColorDetial;
		vecColorDetial.resize(21);
		for(int j = 0;j < vecColorDetial.size();j++)
		{
			vecColorDetial[j] = 0;
		}

		color.mRed = color.mGreen = color.mBlue = 0;
		point.x = point.y = point.z =0;

		for(int j = 0;j < vecPatchPoint[i].mypoints.size();j++)
		{
			color.mRed += vecPatchPoint[i].mypoints[j].r;
			color.mGreen += vecPatchPoint[i].mypoints[j].g;
			color.mBlue += vecPatchPoint[i].mypoints[j].b;

			vecColorDetial[(int)(vecPatchPoint[i].mypoints[j].r / 40) ] += 1;
			vecColorDetial[(int)(vecPatchPoint[i].mypoints[j].g / 40)  + 7] += 1;
			vecColorDetial[(int)(vecPatchPoint[i].mypoints[j].b / 40)  + 14] += 1;

			point.x += vecPatchPoint[i].mypoints[j].x;
			point.y += vecPatchPoint[i].mypoints[j].y;
			point.z += vecPatchPoint[i].mypoints[j].z;
			if(xMax < vecPatchPoint[i].mypoints[j].x) xMax = vecPatchPoint[i].mypoints[j].x;
			if(yMax < vecPatchPoint[i].mypoints[j].y) yMax = vecPatchPoint[i].mypoints[j].y;
			if(zMax < vecPatchPoint[i].mypoints[j].z) zMax = vecPatchPoint[i].mypoints[j].z;
			if(xMin > vecPatchPoint[i].mypoints[j].x) xMin = vecPatchPoint[i].mypoints[j].x;
			if(yMin > vecPatchPoint[i].mypoints[j].y) yMin = vecPatchPoint[i].mypoints[j].y;
			if(zMin > vecPatchPoint[i].mypoints[j].z) zMin = vecPatchPoint[i].mypoints[j].z;
		}
		if(vecPatchPoint[i].mypoints.size() > 0)
		{
			color.mRed /= vecPatchPoint[i].mypoints.size();
			color.mGreen /= vecPatchPoint[i].mypoints.size();
			color.mBlue /= vecPatchPoint[i].mypoints.size();
			point.x/= vecPatchPoint[i].mypoints.size();
			point.y /= vecPatchPoint[i].mypoints.size();
			point.z /= vecPatchPoint[i].mypoints.size();
		}

		vecPatchColor.push_back(color);
		vecvecPatchColorDetial.push_back(vecColorDetial);
		vecPatchCenPoint.push_back(point);
	}
	boundingBoxSize = sqrt((xMax-xMin) * (xMax-xMin) + (yMax-yMin) * (yMax-yMin) + (zMax-zMin) * (zMax-zMin));



	//Distance between two patches
	vecvecPatchMinDis.resize(vecPatchPoint.size());
	vecvecPatchCenDis.resize(vecPatchPoint.size());
	vecvecPatchConnectFlag.resize(vecPatchPoint.size());
	for(int i = 0;i <vecPatchPoint.size();i++)
	{
		vecvecPatchMinDis[i].resize(vecPatchPoint.size(),LARGE_NUM);
		vecvecPatchCenDis[i].resize(vecPatchPoint.size(),LARGE_NUM);
		vecvecPatchConnectFlag[i].resize(vecPatchPoint.size(),false);
	}

	pair<int,int> pairPatchConnection;
	for(int i = 0;i < patchNum;i++)
	{
		for(int j = 0;j < patchNum;j++)
		{
			if(i < j)
			{
				bool  stable;   // if connect stable
				vecvecPatchMinDis[i][j] = vecvecPatchMinDis[j][i] = GetMinDisBetPatch(i,j,stable);
				vecvecPatchCenDis[i][j] = vecvecPatchCenDis[j][i] = GetCenDisBetPatch(i,j);

				if(vecvecPatchMinDis[i][j] < thresholdClose0 && stable)
				{
					pairPatchConnection.first = i;
					pairPatchConnection.second = j;
					vecpairPatchConnection.push_back(pairPatchConnection);

					pairPatchConnection.first = j;
					pairPatchConnection.second = i;
					vecpairPatchConnection.push_back(pairPatchConnection);

					vecvecPatchConnectFlag[i][j] = vecvecPatchConnectFlag[j][i] = true;
				} 
			}
		}
	}

}

double CAreaInterest::GetMinDisBetPatch(int m,int n,bool &stable)
{
	//////////////////////////////////kdtree
// 	double minDis = LARGE_NUM;
// 	vector<bool> vecPointClose0(vecPatchPoint[m].mypoints.size(),false);
// 	vector<bool> vecPointClose1(vecPatchPoint[n].mypoints.size(),false);
// 
// 	
// 	for(int i = 0;i < vecPatchPoint[n].mypoints.size();i++)
// 	{
// 		double pt[3];
// 		pt[0] = vecPatchPoint[n].mypoints[i].x;
// 		pt[1] = vecPatchPoint[n].mypoints[i].y;
// 		pt[2] = vecPatchPoint[n].mypoints[i].z;
// 
// 		struct kdres *presults0;
// 		presults0 = kd_nearest3( vecKDTree[m], pt[0], pt[1], pt[2]);
// 
// 		double dis;
// 		dis = sqrt((presults0->riter->item->pos[0] - pt[0]) * (presults0->riter->item->pos[0] - pt[0]) +
// 			  (presults0->riter->item->pos[1] - pt[1]) * (presults0->riter->item->pos[1] - pt[1]) +
// 		      (presults0->riter->item->pos[2] - pt[2]) * (presults0->riter->item->pos[2] - pt[2]));
// 
// 		if(dis < minDis)
// 			minDis = dis;
// 
// 		if(dis < thresholdClose0 * 3)
// 			vecPointClose1[i] = true;
// 
// 		kd_res_free( presults0 );
// 	}
// 
// 	for(int i = 0;i < vecPatchPoint[m].mypoints.size();i++)
// 	{
// 		double pt[3];
// 		pt[0] = vecPatchPoint[m].mypoints[i].x;
// 		pt[1] = vecPatchPoint[m].mypoints[i].y;
// 		pt[2] = vecPatchPoint[m].mypoints[i].z;
// 
// 		struct kdres *presults1;
// 		presults1 = kd_nearest3( vecKDTree[n], pt[0], pt[1], pt[2]);
// 
// 		double dis;
// 		dis = sqrt((presults1->riter->item->pos[0] - pt[0]) * (presults1->riter->item->pos[0] - pt[0]) +
// 			  (presults1->riter->item->pos[1] - pt[1]) * (presults1->riter->item->pos[1] - pt[1]) +
// 			  (presults1->riter->item->pos[2] - pt[2]) * (presults1->riter->item->pos[2] - pt[2]));
// 
// 		if(dis < minDis)
// 			minDis = dis;
// 		
// 		if(dis < thresholdClose0 * 3)
// 			vecPointClose0[i] = true;
// 
// 		kd_res_free( presults1 );
// 	}
// 
// 
// 	int count0=0;
// 	int count1=0;
// 	for(int i = 0; i < vecPointClose0.size();i++)
// 	{
// 		if(vecPointClose0[i] == true)
// 			count0++;
// 	}
// 	for(int i = 0; i < vecPointClose1.size();i++)
// 	{
// 		if(vecPointClose1[i] == true)
// 			count1++;
// 	}
// 	if(count0 > 5 && count1 >5)
// 		stable = true;
// 	else
// 		stable = false;
// 
// 	return minDis;

		
	//////////////////////////////////ÂùÁ¦·¨
	double minDis = LARGE_NUM;
	vector<bool> vecPointClose0(vecPatchPoint[m].mypoints.size(),false);
	vector<bool> vecPointClose1(vecPatchPoint[n].mypoints.size(),false);

	for(int i = 0;i < vecPatchPoint[m].mypoints.size();i++)
	{
		for(int j = 0;j < vecPatchPoint[n].mypoints.size();j++)
		{
			double dis = sqrt((vecPatchPoint[m].mypoints[i].x-vecPatchPoint[n].mypoints[j].x) * (vecPatchPoint[m].mypoints[i].x-vecPatchPoint[n].mypoints[j].x)
				+ (vecPatchPoint[m].mypoints[i].y-vecPatchPoint[n].mypoints[j].y) * (vecPatchPoint[m].mypoints[i].y-vecPatchPoint[n].mypoints[j].y)
				+ (vecPatchPoint[m].mypoints[i].z-vecPatchPoint[n].mypoints[j].z) * (vecPatchPoint[m].mypoints[i].z-vecPatchPoint[n].mypoints[j].z));
			if(minDis > dis)
			{
				minDis = dis;
			}

			if(dis < thresholdClose0 * 3)
			{
				vecPointClose0[i] = true;
				vecPointClose1[j] = true;
			}
		}
	}

	int count0=0;
	int count1=0;
	for(int i = 0; i < vecPointClose0.size();i++)
	{
		if(vecPointClose0[i] == true)
			count0++;
	}
	for(int i = 0; i < vecPointClose1.size();i++)
	{
		if(vecPointClose1[i] == true)
			count1++;
	}
	
	if(count0 > 5 && count1 >5)
		stable = true;
	else
		stable = false;

	return minDis;
}

double CAreaInterest::GetCenDisBetPatch(int m,int n)
{
	double dis =  sqrt((vecPatchCenPoint[m].x-vecPatchCenPoint[n].x) * (vecPatchCenPoint[m].x-vecPatchCenPoint[n].x)
		+ (vecPatchCenPoint[m].y-vecPatchCenPoint[n].y) * (vecPatchCenPoint[m].y-vecPatchCenPoint[n].y)
		+ (vecPatchCenPoint[m].z-vecPatchCenPoint[n].z) * (vecPatchCenPoint[m].z-vecPatchCenPoint[n].z));
	return dis;
}

void CAreaInterest::ComputeSmoothValue()
{
	for(int i = 0;i <vecpairPatchConnection.size();i++)
	{
		vecSmoothValue.push_back(GetBinarySmoothValue(vecpairPatchConnection[i].first,vecpairPatchConnection[i].second));
	}
}

double CAreaInterest::GetBinarySmoothValue(int m,int n)
{
	double smoothValue,geometryValue,appearenceValue;

	//normal
	MyPoint cenM,cenN;
	Normalt norM,norN,norMN,norNM;
	double nomalizeValue;

	cenM = vecPatchCenPoint[m];
	cenN = vecPatchCenPoint[n];
	norM = vecPatchNormal[m];
	norN = vecPatchNormal[n];

	//normalMN
	norMN.normal_x = cenN.x - cenM.x;
	norMN.normal_y = cenN.y - cenM.y;
	norMN.normal_z = cenN.z - cenM.z;
	nomalizeValue = sqrt(norMN.normal_x * norMN.normal_x + norMN.normal_y * norMN.normal_y + norMN.normal_z * norMN.normal_z);
	norMN.normal_x /= nomalizeValue;
	norMN.normal_y /= nomalizeValue;
	norMN.normal_z /= nomalizeValue;

	////normalNM
	norNM.normal_x = cenM.x - cenN.x;
	norNM.normal_y = cenM.y - cenN.y;
	norNM.normal_z = cenM.z - cenN.z;
	nomalizeValue = sqrt(norNM.normal_x * norNM.normal_x + norNM.normal_y * norNM.normal_y + norNM.normal_z * norNM.normal_z);
	norNM.normal_x /= nomalizeValue;
	norNM.normal_y /= nomalizeValue;
	norNM.normal_z /= nomalizeValue;

	//method1
	double xTemp0,yTemp0,zTemp0;
	double xTemp1,yTemp1,zTemp1;

	double convexValue0,convexValue1;   //convex if > 0
	CrossProduct(norN.normal_x,norN.normal_y,norN.normal_z, norM.normal_x,norM.normal_y,norM.normal_z, xTemp0,yTemp0,zTemp0);	
	CrossProduct(xTemp0,yTemp0,zTemp0, norMN.normal_x,norMN.normal_y,norMN.normal_z, xTemp1,yTemp1,zTemp1);
	convexValue0 = norM.normal_x * xTemp1 +  norM.normal_y * yTemp1 + norM.normal_z * zTemp1;
	CrossProduct(norM.normal_x,norM.normal_y,norM.normal_z, norN.normal_x,norN.normal_y,norN.normal_z, xTemp0,yTemp0,zTemp0);
	CrossProduct(xTemp0,yTemp0,zTemp0, norNM.normal_x,norNM.normal_y,norNM.normal_z, xTemp1,yTemp1,zTemp1);
	convexValue1 = norN.normal_x * xTemp1 +  norN.normal_y * yTemp1 + norN.normal_z * zTemp1;

	bool convexFlag;
	bool errorFlag = false;

	if(convexValue0 * convexValue1 > 0)
	{
		if(convexValue0 >= 0)	
			convexFlag = true;
		else	
			convexFlag = false;

		double cosValue;
		cosValue = (norM.normal_x * norN.normal_x + norM.normal_y * norN.normal_y + norM.normal_z * norN.normal_z);
		if(convexFlag)	
			geometryValue = paraConvexK * cosValue + paraConvexT;
		else	
			geometryValue = paraConcave * cosValue;

		if(geometryValue < 0)
			geometryValue = 0;
	}

	else if(convexValue0 * convexValue1 < 0)
	{
		//method2
		convexValue0 = norMN.normal_x * norN.normal_x +  norMN.normal_y * norN.normal_y + norMN.normal_z * norN.normal_z;
		convexValue1 = norNM.normal_x * norM.normal_x +  norNM.normal_y * norM.normal_y + norNM.normal_z * norM.normal_z;

		if(convexValue0 >= 0)	
			convexFlag = true;
		else	
			convexFlag = false;

		double cosValue;
		cosValue = (norM.normal_x * norN.normal_x + norM.normal_y * norN.normal_y + norM.normal_z * norN.normal_z);
		if(convexFlag)	
			geometryValue = paraConvexK * cosValue + paraConvexT;
		else	
			geometryValue = paraConcave * cosValue;

		if(geometryValue < 0)
			geometryValue = 0;

		if(convexValue0 * convexValue1 < 0)
			errorFlag = true;
	}

	if(errorFlag)
		geometryValue = 0.9;


	//color
	appearenceValue = 0;
	for(int i = 0;i < 21;i++)
	{
		double Mi,Ni;
		Mi = vecvecPatchColorDetial[m][i];
		Ni = vecvecPatchColorDetial[n][i];

		if(Mi != 0 || Ni != 0)
			appearenceValue += (Mi - Ni) * (Mi - Ni) / (Mi + Ni);
	}

	smoothValue = paraGeometry * geometryValue + paraAppearence * appearenceValue;

	vecGeometryValue.push_back(geometryValue);
	vecAppearenceValue.push_back(appearenceValue);
	vecGeometryConvex.push_back(convexFlag);

	return smoothValue;
}

void CAreaInterest::CrossProduct(double ax,double ay,double az,double bx,double by,double bz,double &rx,double &ry,double &rz)
{
	rx = ay*bz - az*by;
	ry = az*bx - ax*bz;
	rz = ax*by - ay*bx;
}

void CAreaInterest::Merge()
{
	mergeFlag = true;
}

void CAreaInterest::Invalidt()
{
	validFlag = false;
}
