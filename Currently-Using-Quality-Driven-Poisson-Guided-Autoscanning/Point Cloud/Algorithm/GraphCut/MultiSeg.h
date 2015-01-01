#pragma once

#include "scene_seg.h"
#include "color_op.h"
#include "graph.h"
#include "GCoptimization.h"
#include "BinarySeg.h"

#define LARGE_NUM 9999999
#define SMALL_NUM -9999999

extern vector<vector<int>> vecvecObjectPoolClustering;
extern vector<int> vecObjectPoolClusteringCount;


#include "matrix.h"  

#ifndef _NO_NAMESPACE
using namespace std;

#define STD std
#else
#define STD
#endif

#ifndef _NO_TEMPLATE
typedef matrix<double> Matrix;
#else
typedef matrix Matrix;
#endif

#ifndef _NO_EXCEPTION
#  define TRYBEGIN()	try {
#  define CATCHERROR()	} catch (const STD::exception& e) { \
	cerr << "Error: " << e.what() << endl; }
#else
#  define TRYBEGIN()
#  define CATCHERROR()
#endif

struct COLORMODEL
{
	double muRed,muGreen,muBlue;
	double sitaRed,sitaGreen,sitaBlue;
	double sitaRG,sitaRB,sitaGB;
};

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
	vector<int> clusterPatchInterval;
	vector<int> vecObjectClusteringIndex;  

	vector<double> vecObjectness;
	vector<double> vecSeparateness;
	vector<pair<int,int>> vecpairSeperatenessEdge;
	vector<vector<pair<int,int>>> vecvecpairSeperatenessSmallEdge;

	vector<vector<bool>> vecvecPatchConnectFlag;

	GRAPHSHOW graphContract;
	vector<vector<int>> vecvecMultiResult;

public:
	void MainStep();
	void GetColorModel();
	void AddObjectPool();
	double GetMultiDataValue(int SiteID,int LableID);
	void GraphCutSolve();
	void ComputeScore();
	void ComputeObjectness(int m);
	void ComputeSeparateness(int m,int n);
	void ConstructGraph();
};

