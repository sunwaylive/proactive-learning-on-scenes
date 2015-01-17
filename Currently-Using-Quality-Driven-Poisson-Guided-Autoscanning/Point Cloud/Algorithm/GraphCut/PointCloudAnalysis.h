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

	vector<MyPointCloud_RGB_NORMAL> vecPatchPoint;
	vector<Normalt> vecPatcNormal;
	MyPointCloud_RGB_NORMAL tablePoint;
	vector<int> clusterPatchNum;
	vector<int> clusterPatchInitIndex;
	vector<CAreaInterest> vecAreaInterest;

	double xMin,xMax,yMin,yMax,zMin,zMax;
	double boundingBoxSize;

	double maxAV;
	double paraGeometry,paraAppearence;
	double paraSmoothAdjust;

	GRAPHSHOW patchGraph;
	GRAPHSHOW contractionGraph;

	ColorSet colorSet;

public:
	CPointCloudAnalysis(void);
	~CPointCloudAnalysis(void);
	void MainStep(bool initFlag,int newAreaNum = 0);
	void InitAreaInterest();
	void ComputeObjectHypo();
	void NormalizeAppearanceTerm();
	void NormalizeSmoothTerm();
	void DataIn();
	int DataUpdate();
	void BinarySegmentation(bool initFlag,int newAreaNum = 0);
	void Clustering();
	void MultiSegmentation();
	void ScanEstimation();
	void Merge(int pushArea);
	void ReAnalysis(int pushArea,int newAreaNum);
	void GraphUpdate();
	void NormalizeNewAppearanceTerm(int areaIndex);
	void NormalizeNewSmoothTerm(int areaIndex);
	void ScanEstimation(int areaIndex);
};

