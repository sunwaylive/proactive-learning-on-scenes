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
	void runPoissonFieldAndExtractIsoPoints_ByEXE(int m);
	void MainStep();
	void samplePointsFromMesh(CMesh& mesh, CMesh* points);
};

class BaseSampler
{
public:
  BaseSampler(CMesh* _m){m=_m; uvSpaceFlag = false; qualitySampling=false; };
  CMesh *m;
  int texSamplingWidth;
  int texSamplingHeight;
  bool uvSpaceFlag;
  bool qualitySampling;

  void AddVert(const CMesh::VertexType &p) 
  {
    tri::Allocator<CMesh>::AddVertices(*m,1);
    m->vert.back().ImportData(p);
  }

  void AddFace(const CMesh::FaceType &f, CMesh::CoordType p) 
  {
     //cout << "######  3.2.2.1 #########" << endl;

    tri::Allocator<CMesh>::AddVertices(*m,1);

     //cout << "######  3.2.2.2 #########" << endl;
    m->vert.back().P() = f.P(0)*p[0] + f.P(1)*p[1] +f.P(2)*p[2];
    m->vert.back().N() = f.V(0)->N()*p[0] + f.V(1)->N()*p[1] + f.V(2)->N()*p[2];

 /*   if (qualitySampling)	
      m->vert.back().Q() = f.V(0)->Q()*p[0] + f.V(1)->Q()*p[1] + f.V(2)->Q()*p[2];*/
  }
 
}; // end class BaseSampler