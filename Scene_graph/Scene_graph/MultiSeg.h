#pragma once

#include "scene_seg.h"
#include "color_op.h"
#include "graph.h"
#include "GCoptimization.h"

#define LARGE_NUM 9999999
#define SMALL_NUM -9999999



#include "matrix.h"  

#ifndef _NO_NAMESPACE
using namespace std;
using namespace math;
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

	vector<vector<int>> vecvecObjectPoolClustering;
	vector<int> vecObjectPoolClusteringCount;
	vector<vector<int>> vecvecMultiResult;

public:
	void MainStep();
	void GetColorModel();
	void AddObjectPool();
	double GetMultiDataValue(int SiteID,int LableID);
};

