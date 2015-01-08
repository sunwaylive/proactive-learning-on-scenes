#pragma once
#include "Algorithm/GraphCut/graph.h"
#include "Algorithm/Common/common_type.h"
#include "Algorithm/GraphCut/GraphCutBasicStruct.h"



class CClustering
{
public:
	vector<MyPointCloud_RGB_NORMAL> vecPatchPoint;
	vector<vector<int>> vecvecObjectPool;
	vector<vector<int>> vecvecObjectPoolClustering;
	vector<int> vecObjectPoolClusteringCount;

	int initObject;
	double paraH, paraSi;
	
public:
	CClustering(void);
	~CClustering(void);
	void Clear();
	void AddObjectPool();
	void MainStep();
	void CleanObjectPool();
	void MeanShift();
	void GetArea(vector<int> vecObjectPool,vector<int> &vecInArea);
	double GetJaccardIndex(vector<int> vecObjectPool, int n);
	void GetMeanShiftSVector(vector<int> vecInArea, vector<int> &objectPool);
};

