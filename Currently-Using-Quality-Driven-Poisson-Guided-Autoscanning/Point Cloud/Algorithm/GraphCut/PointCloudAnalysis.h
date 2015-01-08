#pragma once

#include "Algorithm/Common/color_op.h"
#include "Algorithm/GraphCut/graph.h"
#include "Algorithm/Common/common_type.h"
#include "Algorithm/GraphCut/BinarySeg.h"
#include "Algorithm/GraphCut/Clustering.h"
#include "Algorithm/GraphCut/MultiSeg.h"
#include "Algorithm/GraphCut/ScanEstimation.h"


class CPointCloudAnalysis
{
public:
	CBinarySeg cBinarySeg;
	CClustering cClustering;
	CMultiSeg cMultiSeg;
	CScanEstimation cScanEstimation; 

public:
	CPointCloudAnalysis(void);
	~CPointCloudAnalysis(void);
	void MainStep(bool initFlag,int newAreaNum = 0);
	void DataIn();
	int DataUpdate();
	void BinarySegmentation(bool initFlag,int newAreaNum = 0);
	void Clustering();
	void MultiSegmentation();
	void ScanEstimation();
	void Merge(int pushArea);
	void ReAnalysis(int pushArea);
};

