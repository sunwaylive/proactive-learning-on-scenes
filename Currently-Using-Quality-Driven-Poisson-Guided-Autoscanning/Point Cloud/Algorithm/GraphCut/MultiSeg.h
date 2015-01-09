#pragma once

#include "Algorithm/GraphCut/graph.h"
#include "Algorithm/GraphCut/GCoptimization.h"
#include "Algorithm/GraphCut/BinarySeg.h"
#include "Algorithm/Common/common_type.h"
#include "Algorithm/GraphCut/GraphCutBasicStruct.h"


class CMultiSeg
{
public:
	CMultiSeg(void);
	~CMultiSeg(void);

public:
	vector<MyPointCloud_RGB_NORMAL> vecPatchPoint;
	vector<pair<int,int>> vecpairPatchConnection;
	vector<double> vecSmoothValue;
	vector<double> vecDataValue;

	vector<MyPoint> vecPatchCenPoint;
	vector<COLORMODEL> vecObjectColorModel;
	vector<ColorType> vecPatchColor;
	double boundingBoxSize;

	vector<double> vecCenterDistance;
	vector<double> vecColorSimilarity;
	vector<double> vecObjectCount;

	vector<int> clusterPatchNum;
	vector<int> clusterPatchInitIndex;
	vector<int> clusterPatchInterval;
	vector<int> vecObjectClusteringIndex;  

	vector<double> vecObjectness;
	vector<double> vecSeparateness;
	vector<pair<int,int>> vecpairSeperatenessEdge;
	vector<vector<pair<int,int>>> vecvecpairSeperatenessSmallEdge;

	vector<vector<bool>> vecvecPatchConnectFlag;

	GRAPHSHOW graphContract;
	vector<vector<int>> vecvecMultiResult;
	
	vector<vector<int>> vecvecObjectPoolClustering;
	vector<int> vecObjectPoolClusteringCount;
public:
	void Clear();
	void MainStep();
	void GetColorModel();
	void AddObjectPool();
	double GetMultiDataValue(int SiteID,int LableID);
	void GraphCutSolve();
	void ComputeScore();
	void ComputeObjectness(int m);
	void ComputeSeparateness(int m,int n);
	void ConstructGraph();
	int GetAreaIndex(int patchIndex);
};

