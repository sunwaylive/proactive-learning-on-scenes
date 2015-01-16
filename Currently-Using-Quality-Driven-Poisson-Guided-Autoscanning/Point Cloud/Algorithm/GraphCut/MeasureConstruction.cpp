#include "MeasureConstruction.h"


CMeasureConstruction::CMeasureConstruction(void)
{
}

CMeasureConstruction::~CMeasureConstruction(void)
{
}

void CMeasureConstruction::DataIn()
{
	PointCloudPtr_RGB_NORMAL cloud(new PointCloud_RGB_NORMAL);
	loadPointCloud_normal_ply("data/S.ply", cloud, meshFaceS, meshVertexS);
	loadPointCloud_normal_ply("data/T.ply", cloud, meshFaceT, meshVertexT);

}

void CMeasureConstruction::SavePointsToOriginal(CMesh *original,int st)
{
	if(st == 0)//s
	{
		for(int i = 0;i < meshVertexS.vecVertex.size();i++)
		{
			Point_RGB_NORMAL point =  meshVertexS.vecVertex[i];
			CVertex new_point;
			new_point.P()[0] = point.x;
			new_point.P()[1] = point.y;
			new_point.P()[2] = point.z;
			new_point.N()[0] = point.normal_x;
			new_point.N()[1] = point.normal_y;
			new_point.N()[2] = point.normal_z;

			new_point.m_index = i;
			new_point.is_original = true;
			original->vert.push_back(new_point);
			original->bbox.Add(new_point.P());
		}
	}
	else//t
	{
		for(int i = 0;i < meshVertexT.vecVertex.size();i++)
		{
			Point_RGB_NORMAL point =  meshVertexT.vecVertex[i];
			CVertex new_point;
			new_point.P()[0] = point.x;
			new_point.P()[1] = point.y;
			new_point.P()[2] = point.z;
			new_point.N()[0] = point.normal_x;
			new_point.N()[1] = point.normal_y;
			new_point.N()[2] = point.normal_z;

			new_point.m_index = i;
			new_point.is_original = true;
			original->vert.push_back(new_point);
			original->bbox.Add(new_point.P());
		}
	}

	original->vn = original->vert.size();
}

void CMeasureConstruction::ComputeScore()
{
	double supportT,supportS;
	mutualDataSupport = supportT =supportS = 0;
	
	for(int i = 0;i < isoPointT.vecPoints.size();i++)
	{
		supportT += GetPiS(i);
	}
	supportT /= isoPointT.vecPoints.size();

	for(int i = 0;i < isoPointS.vecPoints.size();i++)
	{
		supportS += GetPiT(i);
	}
	supportS /= isoPointS.vecPoints.size();

	mutualDataSupport = supportS + supportT;
}

double CMeasureConstruction::GetPiS(int m)
{
	double sumValue = 0;
 	for(int i = 0;i < isoPointS.vecPoints.size();i++)
	{
		IsoPoint point0,point1;
		point0 = isoPointS.vecPoints[i];
		point1 = isoPointT.vecPoints[m];

		double doubleTemp0;
		doubleTemp0 = (point0.x-point1.x) * (point0.x-point1.x) + (point0.y-point1.y) * (point0.y-point1.y) + (point0.z-point1.z) * (point0.z-point1.z);
		doubleTemp0 = -sqrt(doubleTemp0);
		doubleTemp0 = doubleTemp0 /1.6 /1.6;

		double doubleTemp1;
		doubleTemp1 = (1 - point0.normal_x * point1.normal_x) * (1 - point0.normal_y * point1.normal_x) * (1 - point0.normal_z * point1.normal_z);
		doubleTemp1 = -sqrt(doubleTemp1);
		doubleTemp1 = doubleTemp1 /0.1 /0.1;

		sumValue += pow(E,doubleTemp0) * pow(E,doubleTemp1);
	}

	return sumValue;
}

double CMeasureConstruction::GetPiT(int m)
{
	double sumValue = 0;
	for(int i = 0;i < isoPointT.vecPoints.size();i++)
	{
		IsoPoint point0,point1;
		point0 = isoPointT.vecPoints[i];
		point1 = isoPointS.vecPoints[m];

		double doubleTemp0;
		doubleTemp0 = (point0.x-point1.x) * (point0.x-point1.x) + (point0.y-point1.y) * (point0.y-point1.y) + (point0.z-point1.z) * (point0.z-point1.z);
		doubleTemp0 = -sqrt(doubleTemp0);
		doubleTemp0 = doubleTemp0 /1.6 /1.6;

		double doubleTemp1;
		doubleTemp1 = (1 - point0.normal_x * point1.normal_x) * (1 - point0.normal_y * point1.normal_x) * (1 - point0.normal_z * point1.normal_z);
		doubleTemp1 = -sqrt(doubleTemp1);
		doubleTemp1 = doubleTemp1 /0.1 /0.1;

		sumValue += pow(E,doubleTemp0) * pow(E,doubleTemp1);
	}

	return sumValue;
}