#pragma once
#include "Algorithm/Common/common_type.h"
#include "Algorithm/Common/color_op.h"
#include "Algorithm/Common/file_io.h"
#include "Algorithm/GraphCut/GraphCutBasicStruct.h"

class CMeasureConstruction
{
public:
	MeshFace meshFaceS,meshFaceT;
	MeshVertex meshVertexS,meshVertexT;
	ObjectIsoPoint isoPointS,isoPointT;
	double mutualDataSupport;

public:
	CMeasureConstruction(void);
	~CMeasureConstruction(void);
	void DataIn();
	void ComputeScore();
	void SavePointsToOriginal(CMesh *original,int st);
	double GetPiS(int i);
	double GetPiT(int j);
};

