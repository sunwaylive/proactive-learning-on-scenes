#pragma once

#include "visualizer.h"
#include "color_op.h"
#include "file_io.h"
#include "scene_seg.h"
#include "graph.h"

#define LARGE_NUM 9999999
#define SMALL_NUM -9999999

struct NEARBYPOINT
{
	int patchFirst;
	int patchSecond;

	int indexFirst;
	int indexSecond;
};

struct NEARBYPOINTSUM
{
	vector<NEARBYPOINT> nearbyPoint;
};

struct NEARBYNORMAL
{
	Normal normal0,normal1;
};

class CBinarySeg
{
public:
	CBinarySeg();
	~CBinarySeg(void);

public:
	vector<MyPointCloud_RGB_NORMAL> vecPatchPoint;
	vector<int> clusterPatchNum;
	//	PointCloudPtr_RGB_NORMAL tablePoint;
	vector<Normal> vecPatcNormal;
	vector<vector<NEARBYNORMAL>> vecvecPatctNearbyNormal;
	vector<MyPoint> vecPatchCenPoint;
	vector<ColorType> vecPatchColor;
	vector<vector<int>> vecvecPatchColorDetial;
	vector<pair<int,int>> vecpairPatchConnection;
	vector<vector<NEARBYPOINTSUM>> vecvecNearbyPoint; 

	double boundingBoxSize;
	double thresholdClose0; 
	double thresholdClose1; 
	double xMin,xMax,yMin,yMax,zMin,zMax;

	vector<vector<double>> vecvecPatchMinDis;
	vector<vector<double>> vecvecPatchCenDis;

	vector<double> vecDataValue;
	vector<double> vecSmoothValue;
	vector<double> vecGeometryValue;
	vector<double> vecAppearenceValue;

	int seedPatch; 

	vector<int> vecFore,vecBack;
	vector<double> vecFlow;
	double m_flow;

	//parameter
	double paraSmallS,paraSmallK;
	double paraLargeS,paraLargeK;
	double paraConvexK,paraConvexT,paraConcave;
	double paraGeometry,paraAppearence;
	double paraMinPatchInObject,paraMaxCutEnergy;
	double paraAlpha;

	int backSeedIndex;

	vector<bool> vecIfConnectTable;
	MyPoint tableCen;

	vector<struct kdtree*> vecKDTree;

	vector<vector<int>> vecvecObjectPool;

public:
	//	void AddTable(PointCloudPtr_RGB_NORMAL &table);
	void AddClusterPoints(vector<MyPointCloud_RGB_NORMAL> &points);
	void AddPatchNormal(vector<Normal> &normal);
	void MainStep();
	void GetAdjacency(int patchBegin,int patchEnd);
	void PointCloudPreprocess();
	void ComputeDataValue();
	void GraphCutSolve(vector<int> &vecObjectHypo, double &cutEnergy);
	double GetMinDisBetPatch(int m,int n,bool &stable);
	double GetCenDisBetPatch(int m,int n);
	void ComputeSmoothValue();
	double GetBinaryDataValue(double d);
	double GetBinarySmoothValue(int m,int n);
	bool IfConnectTable(vector<MyPt_RGB_NORMAL> points);
	void NomalizeData();
	void NomalizeAppearence();
	void NomalizeSmooth();
};

