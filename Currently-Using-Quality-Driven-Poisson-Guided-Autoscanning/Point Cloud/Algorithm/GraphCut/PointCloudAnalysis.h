#pragma once

#include "Algorithm/GraphCut/color_op.h"
#include "Algorithm/GraphCut/scene_seg.h"
#include "Algorithm/GraphCut/graph.h"
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
	void MainStep();
	void DataIn();
	void BinarySegmentation();
	void Clustering();
	void MultiSegmentation();
	void ScanEstimation();
};

