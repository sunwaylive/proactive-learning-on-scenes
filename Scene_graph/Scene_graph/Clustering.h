#pragma once

#include "scene_seg.h"
#include "color_op.h"
#include "graph.h"

#define LARGE_NUM 9999999
#define SMALL_NUM -9999999


class CClustering
{
public:
	CClustering(void);
	~CClustering(void);

public:
	vector<MyPointCloud_RGB_NORMAL> vecPatchPoint;
	vector<vector<int>> vecvecObjectPool;
	vector<int> clusterPatchNum;
	int initObject;
	
	double paraH, paraSi;
	
	vector<vector<int>> vecvecObjectPoolClustering;
	vector<int> vecObjectPoolClusteringCount;

public:
	void AddObjectPool();
	void MainStep();
	void CleanObjectPool();
	void MeanShift();
	void GetArea(vector<int> vecObjectPool,vector<int> &vecInArea);
	double GetJaccardIndex(vector<int> vecObjectPool, int n);
	void GetMeanShiftSVector(vector<int> vecInArea, vector<int> &objectPool);
};

