#pragma once
#include "Algorithm/Common/common_type.h"
#include "Algorithm/Common/color_op.h"
#include "Algorithm/GraphCut/matrix.h"  

#define LARGE_NUM 9999999
#define SMALL_NUM -9999999

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


struct GRAPHSHOW
{
	vector<MyPt_RGB_NORMAL> vecNodes;
	vector<pair<int,int>> vecEdges;
};

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
};

struct EdgeHypo
{
	int begin,end;
	int areaIndex;
	double separateness;
};

class CAreaInterest
{
public:
	bool validFlag;
	bool mergeFlag;
	vector<MyPointCloud_RGB_NORMAL> vecPatchPoint;
	int patchNum;
	vector<Normalt> vecPatchNormal;
	vector<MyPoint> vecPatchCenPoint;
	vector<ColorType> vecPatchColor;
	vector<vector<int>> vecvecPatchColorDetial;
	double xMin,xMax,yMin,yMax,zMin,zMax;
	double boundingBoxSize;

	vector<pair<int,int>> vecpairPatchConnection;
	vector<vector<bool>> vecvecPatchConnectFlag;
	vector<vector<double>> vecvecPatchMinDis;
	vector<vector<double>> vecvecPatchCenDis;
	double thresholdClose0; 

	double paraConvexK,paraConvexT,paraConcave;
	double paraGeometry,paraAppearence;
	vector<double> vecSmoothValue;
	vector<double> vecGeometryValue;
	vector<double> vecAppearenceValue;
	vector<bool> vecGeometryConvex;

public:
	CAreaInterest(vector<MyPointCloud_RGB_NORMAL> &pointCloud, vector<Normalt> &patchNomal);
	~CAreaInterest(void);
	void PointCloudPreprocess();
	double GetMinDisBetPatch(int m,int n,bool &stable);
	double GetCenDisBetPatch(int m,int n);
	void ComputeSmoothValue();
	double GetBinarySmoothValue(int m,int n);
	void CrossProduct(double ax,double ay,double az,double bx,double by,double bz,double &rx,double &ry,double &rz);
	void Merge();
};