#pragma once

#include "scene_seg.h"
#include "color_op.h"
#include "graph.h"

#define LARGE_NUM 9999999
#define SMALL_NUM -9999999

extern vector<vector<int>> vecvecObjectPoolClustering;
extern vector<int> vecObjectPoolClusteringCount;
extern vector<vector<int>> vecvecObjectPool;


class CClustering
{
public:
	CClustering(void);
	~CClustering(void);

public:
	vector<MyPointCloud_RGB_NORMAL> vecPatchPoint;
	vector<int> clusterPatchNum;
	int initObject;
	
	double paraH, paraSi;
	
public:
	void AddObjectPool();
	void MainStep();
	void CleanObjectPool();
	void MeanShift();
	void GetArea(vector<int> vecObjectPool,vector<int> &vecInArea);
	double GetJaccardIndex(vector<int> vecObjectPool, int n);
	void GetMeanShiftSVector(vector<int> vecInArea, vector<int> &objectPool);
};

