#pragma once

//#include "scene_seg.h"
#include "Algorithm/Common/common_type.h"
#include "Algorithm/Common/color_op.h"
#include "graph.h"
#include "kdtree.h"

class CBinarySeg
{
public:
	vector<vector<NEARBYNORMAL>> vecvecPatctNearbyNormal;
	vector<vector<int>> vecvecPatchColorDetial;
	vector<vector<NEARBYPOINTSUM>> vecvecNearbyPoint; 
	double thresholdClose0; 
	double thresholdClose1; 
	vector<vector<double>> vecvecPatchMinDis;
	vector<vector<double>> vecvecPatchCenDis;
	vector<double> vecDataValue;
	vector<double> vecGeometryValue;
	vector<double> vecAppearenceValue;
	vector<bool> vecGeometryConvex;
	int seedPatch; 
	vector<int> vecFore,vecBack;
	vector<double> vecFlow;
	double m_flow;
	int backSeedIndex;
	vector<struct kdtree*> vecKDTree;
	double maxSV,minSV;

	//parameter
	double paraSmallS,paraSmallK;
	double paraLargeS,paraLargeK;
	double paraConvexK,paraConvexT,paraConcave;
	double paraGeometry,paraAppearence;
	double paraMinPatchInObject,paraMaxCutEnergy;
	double paraAlpha;
	double paraSmoothAdjust;

public:
	CBinarySeg();
	~CBinarySeg(void);
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

