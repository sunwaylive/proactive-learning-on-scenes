#pragma once

#include "scene_seg.h"
#include "color_op.h"
#include "PoissonParam.h"
#include "CMesh.h"


struct ISOPOINT
{
	int objectIndex;
	double x,y,z;
	double fg,fs;
	double f;
};

struct OBJECTISOPOINT
{
	vector<ISOPOINT> objectIsoPoint;
};

class CScanEstimation
{
public:
	vector<OBJECTISOPOINT> vecObjectIsoPoint;
	vector<pair<int,int>> vecpairPatchConnection;
	vector<double> vecPatchConfidenceScore;
	vector<double> vecGeometryValue;
	vector<double> vecAppearenceValue;
	vector<bool> vecGeometryConvex;
	double maxSV,minSV;
	double paraConfidence;
	double paraSmoothAdjust;

public:
	CScanEstimation(void);
	~CScanEstimation(void);
	void runComputeIsoGradientConfidence();
  void saveMultiResultToOriginal(CMesh *original, int m);
	void MainStep(CMesh *original);
	void ComputeScore();
	void ComputeObjectness(int m);
	void ComputeSeparateness(int m,int n);
	double GaussianFunction(double x);
};
