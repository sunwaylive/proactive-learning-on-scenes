#pragma once

//#include "scene_seg.h"
#include "Algorithm/Common/common_type.h"
#include "Algorithm/Common/color_op.h"
#include "Algorithm/GraphCut/graph.h"
#include "kdtree.h"
#include "Algorithm/GraphCut/GraphCutBasicStruct.h"

class CBinarySeg
{
public:
	CBinarySeg();
	~CBinarySeg(void);

public:
	vector<MyPointCloud_RGB_NORMAL> vecPatchPoint;
	vector<int> clusterPatchNum;
	vector<int> clusterPatchInitIndex;
	MyPointCloud_RGB_NORMAL tablePoint;
	vector<Normalt> vecPatcNormal;
	vector<MyPoint> vecPatchCenPoint;
	vector<ColorType> vecPatchColor;
	vector<vector<int>> vecvecPatchColorDetial;
	vector<pair<int,int>> vecpairPatchConnection;

	double boundingBoxSize;
	double thresholdClose0; 
	double xMin,xMax,yMin,yMax,zMin,zMax;

	vector<vector<double>> vecvecPatchMinDis;
	vector<vector<double>> vecvecPatchCenDis;
	vector<vector<bool>> vecvecPatchConnectFlag;

	vector<double> vecDataValue;
	vector<double> vecSmoothValue;
	vector<double> vecGeometryValue;
	vector<double> vecAppearenceValue;
	vector<bool> vecGeometryConvex;

	vector<CAreaInterest> vecAreaInterest;


	int seedPatch; 

	vector<int> vecFore,vecBack;
	vector<double> vecFlow;
	double m_flow;

	//parameter
	double paraSmallS,paraSmallK;
	double paraLargeS,paraLargeK;
	double paraConvexK,paraConvexT,paraConcave;
	double paraGeometry,paraAppearence;
	double paraMinPatchInObject,paraMaxCutEnergy;
	double paraAlpha;
	double paraSmoothAdjust;

	int backSeedIndex;

	vector<bool> vecIfConnectTable;
	MyPoint tableCen;

	vector<struct kdtree*> vecKDTree;

	GRAPHSHOW graphInit;
	vector<vector<int>> vecvecObjectPool;

	double maxSV,minSV;
public:
	void AddTable(MyPointCloud_RGB_NORMAL &table);
	void AddClusterPoints(vector<MyPointCloud_RGB_NORMAL> &points);
	void AddPatchNormal(vector<Normalt> &normal);
	void Clear();
	void MainStep(bool initFlag,int newAreaNum = 0);
	void InitAreaInterest();
	void UpdateAreaInterest(int newAreaNum);
	void CollectAreaInterest();
	void GetAdjacency(int patchBegin,int patchEnd);
	void AddMatrixIn(vector<vector<double>>  &matrixSmall,vector<vector<double>> &matrixBig,int beginIndex,int endIndex);
	void AddMatrixInBool(vector<vector<bool>>  &matrixSmall,vector<vector<bool>> &matrixBig,int beginIndex,int endIndex);
	void AddConnectionIn(vector<pair<int,int>> &connectionSmall,vector<pair<int,int>> &connectionBig,int beginIndex,int endIndex);
	pair<int,int> GetAreaIndex(int index);
	void ComputeDataValue();
	void GraphCutSolve(vector<int> &vecObjectHypo, double &cutEnergy);
	double GetMinDisBetPatch(int m,int n,bool &stable);
	double GetCenDisBetPatch(int m,int n);
	void ComputeSmoothValue();
	double GetBinaryDataValue(double d);
	double GetBinarySmoothValue(int m,int n);
	bool IfConnectTable(vector<MyPt_RGB_NORMAL> points);
	void NomalizeAppearence();
	void NomalizeSmooth();
	void ConstructGraph();
};
