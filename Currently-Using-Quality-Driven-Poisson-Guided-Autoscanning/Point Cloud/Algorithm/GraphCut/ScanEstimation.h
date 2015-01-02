#pragma once

#include "scene_seg.h"
#include "color_op.h"
#include "PoissonParam.h"
#include "CMesh.h"

#define LARGE_NUM 9999999
#define SMALL_NUM -9999999

struct ISOPOINT
{
	int objectIndex;
	double x,y,z;
	double fg,fs;
};


class CScanEstimation
{
public:
	CScanEstimation(void);
	~CScanEstimation(void);

public:
	vector<MyPointCloud_RGB_NORMAL> vecPatchPoint;
	vector<vector<int>> vecvecMultiResult;
	vector<vector<ISOPOINT>> vecvecIsoPoint;

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

public:
	void runComputeIsoGradientConfidence();
  void saveMultiResultToOriginal(CMesh *original, int m);
	void MainStep(CMesh *original);
};
