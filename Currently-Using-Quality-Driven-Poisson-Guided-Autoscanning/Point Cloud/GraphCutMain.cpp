#pragma once

#include "Algorithm/GraphCut/PointCloudAnalysis.h"

extern vector<vector<int>> vecvecObjectPool;
vector<vector<int>> vecvecObjectPoolClustering;
vector<int> vecObjectPoolClusteringCount;

double xMin,yMin,zMin,xMax,yMax,zMax;

CPointCloudAnalysis cPointCloudAnalysis;


void GraphCutMain()
{
	cPointCloudAnalysis.MainStep();
}