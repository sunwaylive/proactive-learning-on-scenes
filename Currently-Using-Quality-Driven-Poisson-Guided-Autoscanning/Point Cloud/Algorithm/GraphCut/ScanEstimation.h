#pragma once

//#include "scene_seg.h"
#include "GraphCutGlobalHeader.h"
#include "Algorithm/Common/common_type.h"
#include "Algorithm/Common/color_op.h"
#include "PoissonParam.h"
#include "CMesh.h"
#include "Algorithm/GraphCut/GraphCutBasicStruct.h"


class CScanEstimation
{
public:
	vector<MyPointCloud_RGB_NORMAL> vecPatchPoint;
	vector<MyPoint> vecPatchCenPoint;
	vector<vector<int>> vecvecMultiResult;
	vector<vector<IsoPoint>> vecvecIsoPoint;
	vector<int> clusterPatchNum;
	vector<int> clusterPatchInitIndex;
	vector<int> clusterPatchNumLocal;
	vector<int> clusterPatchInitIndexLocal;
	
	vector<pair<int,int>> vecpairPatchConnection;
	vector<vector<bool>> vecvecPatchConnectFlag;

	vector<double> vecObjectness;
	vector<double> vecSeparateness;

	vector<pair<int,int>> vecpairSeperatenessEdge;
	vector<vector<pair<int,int>>> vecvecpairSeperatenessSmallEdge;

	vector<double> vecGeometryValue;
	vector<double> vecAppearenceValue;
	vector<double> vecSmoothValue;
	vector<bool> vecGeometryConvex;
	double maxSV,minSV;
	double paraConfidence;
	double paraSmoothAdjust;

	vector<ObjectIsoPoint> vecObjectIsoPoint;
	vector<double> vecPatchConfidenceScore;

	vector<ObjectHypo> vecObjectHypo;
	vector<EdgeHypo> vecEdgeHypo;
	vector<int> vecObjectSorting;
	vector<int> vecEdgeSorting;

	GRAPHSHOW graphContract;

public:
	CScanEstimation(void);
	~CScanEstimation(void);
	void Clear();
	void saveMultiResultToOriginal(CMesh *original, int m);
	void ScoreUpdate();
	void ComputeScore();
	void GetColour(double v,double vmin,double vmax,double &r,double &g,double &b);
	void UpdateGraph();
	void ComputeObjectness(int m);
	void ComputeSeparateness(int m,int n);
	double GaussianFunction(double x);
	double ComputePatchConfidenceScore(int objectIndex,int patchIndex);
	double ComputePointConfidenceScore(int objectIndex,MyPoint_RGB_NORMAL point);
	int GetAreaIndex(int patchIndex);
	void Sorting(vector<ObjectHypo> obj,vector<EdgeHypo> edge,vector<int> &objSorting,vector<int> &edgeSorting);
};
