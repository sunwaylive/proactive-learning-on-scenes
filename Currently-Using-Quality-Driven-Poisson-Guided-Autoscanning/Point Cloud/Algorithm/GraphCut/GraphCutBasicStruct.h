#pragma once
#include "Algorithm/Common/common_type.h"
#include "Algorithm/Common/color_op.h"
#include "Algorithm/GraphCut/matrix.h"  
#include "Algorithm/GraphCut/graph.h"
#include "Algorithm/GraphCut/GCoptimization.h"
#include "CMesh.h"
#include "Algorithm/Common/file_io.h"

#define LARGE_NUM 9999999
#define SMALL_NUM -9999999
#define E 2.71828182

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



struct IsoPoint
{
	int objectIndex;
	double x,y,z;
	double normal_x,normal_y,normal_z;
	double fg,fs;
	double f;
};

struct ObjectIsoPoint
{
	vector<IsoPoint> vecPoints;
};

struct COLORMODEL
{
	double muRed,muGreen,muBlue;
	double sitaRed,sitaGreen,sitaBlue;
	double sitaRG,sitaRB,sitaGB;
};

struct ObjectHypo
{
	vector<int> patchIndex;
	int areaIndex;
	double objectness;
	bool mergeFlag;
	MyPoint cenPoint;
	ObjectIsoPoint isoPoints;
};

struct EdgeHypo
{
	int begin,end;
	int areaIndex;
	vector<pair<int,int>> pairPatch;
	double separateness;
};

class CAreaInterest
{
public:
	bool validFlag;
	bool mergeFlag;

	vector<MyPointCloud_RGB_NORMAL> vecPatchPoint;
	vector<Normalt> vecPatchNormal;
	vector<MyPoint> vecPatchCenPoint;
	vector<ColorType> vecPatchColor;
	int patchNum;
	double xMin,xMax,yMin,yMax,zMin,zMax;
	double boundingBoxSize;
	
	vector<vector<double>> vecvecPatchMinDis;
	vector<vector<double>> vecvecPatchCenDis;
	vector<vector<bool>> vecvecPatchConnectFlag;

	vector<pair<int,int>> vecpairPatchConnection;
	vector<vector<int>> vecvecPatchColorDetial;
	
	//parameter
	double thresholdClose0; 
	double paraConvexK,paraConvexT,paraConcave;
	double paraGeometry,paraAppearence;
	double paraSmallS,paraSmallK;
	double paraLargeS,paraLargeK;
	double paraMinPatchInObject,paraMaxCutEnergy;
	double paraAlpha;
	double paraSmoothAdjust;
	double paraH;

	vector<double> vecSmoothValue;
	vector<double> vecGeometryValue;
	vector<double> vecAppearenceValue;
	vector<bool> vecGeometryConvex;
	vector<double> vecDataValue;

	int seedPatch; 
	GRAPHSHOW graphInit;
	vector<vector<int>> vecvecObjectPool;

	vector<vector<int>> vecvecObjectPoolClustering;
	vector<int> vecObjectPoolClusteringCount;

	vector<int> clusterPatchInterval;
	vector<double> vecObjectness;
	vector<double> vecSeparateness;
	vector<pair<int,int>> vecpairSeperatenessEdge;
	vector<vector<pair<int,int>>> vecvecpairSeperatenessSmallEdge;
	vector<ObjectHypo> vecObjectHypo;
	vector<EdgeHypo> vecEdgeHypo;
	GRAPHSHOW graphContract;
	vector<vector<int>> vecvecMultiResult;

	double paraConfidence;
	vector<double> vecPatchConfidenceScore;

public:
	CAreaInterest(vector<MyPointCloud_RGB_NORMAL> &pointCloud, vector<Normalt> &patchNomal);
	~CAreaInterest(void);
	void MainStep();
	void Preprocess();
	double GetMinDisBetPatch(int m,int n,bool &stable);
	double GetCenDisBetPatch(int m,int n);
	void ComputeSmoothValue();
	double GetBinarySmoothValue(int m,int n);
	void CrossProduct(double ax,double ay,double az,double bx,double by,double bz,double &rx,double &ry,double &rz);
	void Merge();
	void Invalidt();
	void BinarySeg();
	void ConstructPatchGraph();
	double GetBinaryDataValue(double d);
	void ComputeDataValue();
	void SolveBinaryGraphCut(vector<int> &vecObjectHypo, double &cutEnergy);
	void Clustering();
	void CleanObjectPool();
	void MeanShift();
	void GetArea(vector<int> vecObjectPool,vector<int> &vecInArea);
	double GetJaccardIndex(vector<int> vecObjectPool, int n);
	void GetMeanShiftSVector(vector<int> vecInArea, vector<int> &objectPool);
	void MultiSeg();
	void AddObjectPool();
	double GetMultiDataValue(int SiteID,int LableID);
	void SolveMultiGraphCut();
	void ComputeHypo();
	void ComputeObjectness(int m);
	void ComputeSeparateness(int m,int n);
	void ConstructContractionGraph();
	void SaveObjectHypoToOriginal(CMesh *original, int m);
	void ScoreUpdate();
	double ComputePatchConfidenceScore(int objectIndex,int patchIndex);
	double ComputePointConfidenceScore(int objectIndex, MyPoint_RGB_NORMAL point);
	void ComputeScore();
	void UpdateObjectness(int m);
	void UpdateSeparateness(int m,int n);
	double GaussianFunction(double x);
	void UpdateGraph();
	void GetColour(double v,double vmin,double vmax,double &r,double &g,double &b);
	void GetNewOneObjectPly();
	void ComputeOverallHypo();
	void ComputeNewContractGraph();
};