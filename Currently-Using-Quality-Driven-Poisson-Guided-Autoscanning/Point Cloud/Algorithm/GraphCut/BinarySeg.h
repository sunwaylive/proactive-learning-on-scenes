#pragma once

#include "scene_seg.h"
#include "color_op.h"
#include "graph.h"
#include "kdtree.h"

#define LARGE_NUM 9999999
#define SMALL_NUM -9999999

struct GRAPHSHOW
{
	vector<MyPt_RGB_NORMAL> vecNodes;
	vector<pair<int,int>> vecEdges;
};

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
	Normalt normal0,normal1;
};

class CBinarySeg
{
public:
	CBinarySeg();
	~CBinarySeg(void);

public:
	vector<MyPointCloud_RGB_NORMAL> vecPatchPoint;//shiyifei
	vector<int> clusterPatchNum;
	MyPointCloud_RGB_NORMAL tablePoint;
	vector<Normalt> vecPatcNormal;
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
	vector<vector<bool>> vecvecPatchConnectFlag;

	vector<double> vecDataValue;
	vector<double> vecSmoothValue;
	vector<double> vecGeometryValue;
	vector<double> vecAppearenceValue;
	vector<bool> vecGeometryConvex;

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
	double paraSmoothAdjust;

	int backSeedIndex;

	vector<bool> vecIfConnectTable;
	MyPoint tableCen;

	vector<struct kdtree*> vecKDTree;

	GRAPHSHOW graphInit;

	double maxSV,minSV;
public:
//	void AddTable(PointCloudPtr_RGB_NORMAL &table);
	void AddClusterPoints(vector<MyPointCloud_RGB_NORMAL> &points);
	void AddPatchNormal(vector<Normalt> &normal);
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
	void NomalizeAppearence();
	void NomalizeSmooth();
	void ConstructGraph();
};

