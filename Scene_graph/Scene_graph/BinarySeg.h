#pragma once

#include "visualizer.h"
#include "color_op.h"
#include "file_io.h"
#include "scene_seg.h"
#include "graph.h"

#define LARGE_NUM 9999999
#define SMALL_NUM -9999999

class CBinarySeg
{
public:
	vector<MyPointCloud_RGB> vecPatchPoint;
	vector<Normal> vecPatcNormal;
	vector<MyPoint> vecPatchCenPoint;
	vector<ColorType> vecPatchColor;
	vector<pair<int,int>> vecpairPatchConnection;

	double boundingBoxSize;
	double closeThreshold;
	double xMin,xMax,yMin,yMax,zMin,zMax;

	vector<vector<double>> vecvecPatchMinDis;
	vector<vector<double>> vecvecPatchCenDis;

	vector<double> vecDataValue;
	vector<double> vecSmoothValue;
	vector<pair<int,int>> verpairSmoothVertex;

	int m; 

	vector<int> vecFore,vecBack;

	//parameter
	double paraClose;
	double paraS,paraK;
	double paraConvexK,paraConvexT,paraConcave;
	double paraGeometry,paraAppearence;

public:
	CBinarySeg(vector<MyPointCloud_RGB> points,vector<Normal> normals);
	~CBinarySeg(void);

	void MainStep();
	void PointCloudPreprocess();
	void GraphConstruct();
	void GraphCutSolve();
	double GetMinDisBetPatch(int m,int n);
	double GetCenDisBetPatch(int m,int n);
	double GetBinaryDataValue(double d);
	double GetBinarySmoothValue(int m,int n);

};

