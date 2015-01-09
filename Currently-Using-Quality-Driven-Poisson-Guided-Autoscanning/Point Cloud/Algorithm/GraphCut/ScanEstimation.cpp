#include "ScanEstimation.h"



CScanEstimation::CScanEstimation(void)
{
	paraConfidence = 2;
	paraSmoothAdjust = 0.75;
}


CScanEstimation::~CScanEstimation(void)
{
}

void CScanEstimation::Clear()
{
	vecPatchConfidenceScore.clear();
	vecvecIsoPoint.clear();
	vecObjectIsoPoint.clear();
	vecObjectHypo.clear();
	vecEdgeHypo.clear();
	vecObjectSorting.clear();
	vecEdgeSorting.clear();
}

void CScanEstimation::saveMultiResultToOriginal( CMesh *original, int m )
{
	for(int i = 0;i < vecvecMultiResult[m].size();i++)
	{
		for(int j = 0;j < vecPatchPoint[vecvecMultiResult[m][i]].mypoints.size();j++)
		{
			MyPoint_RGB_NORMAL point = vecPatchPoint[vecvecMultiResult[m][i]].mypoints[j];
			CVertex new_point;
			new_point.P()[0] = point.x;
			new_point.P()[1] = point.y;
			new_point.P()[2] = point.z;
			new_point.N()[0] = point.normal_x;
			new_point.N()[1] = point.normal_y;
			new_point.N()[2] = point.normal_z;

			new_point.m_index = i;
			new_point.is_original = true;
			original->vert.push_back(new_point);
			original->bbox.Add(new_point.P());
		}
	}
	original->vn = original->vert.size();
	return;
}


void CScanEstimation::ScoreUpdate()
{
	//get confidence score for each patch
	vecPatchConfidenceScore.resize(vecPatchPoint.size(),0);
	for(int i = 0;i < vecvecMultiResult.size();i++)
	{
		for(int j = 0;j < vecvecMultiResult[i].size();j++)
		{
			int patchIndex = vecvecMultiResult[i][j];
			vecPatchConfidenceScore[patchIndex] = ComputePatchConfidenceScore(i,patchIndex);
		}
	}

	//update the smooth term
	for(int i = 0;i < vecpairPatchConnection.size();i++)
	{
		double confidenceScore;
		confidenceScore = vecPatchConfidenceScore[vecpairPatchConnection[i].first] + vecPatchConfidenceScore[vecpairPatchConnection[i].second];

		if(vecGeometryConvex[i])
			vecGeometryValue[i] *= confidenceScore;
		else
			vecGeometryValue[i] /= confidenceScore;

		double valueBefore = vecSmoothValue[i];
		vecSmoothValue[i] = vecGeometryValue[i] + vecAppearenceValue[i];

		double para = 0.3;
		vecSmoothValue[i] =  (vecSmoothValue[i] - minSV)/(maxSV - minSV);
		vecSmoothValue[i] = paraSmoothAdjust * pow(2.7183,- (1 - vecSmoothValue[i]) * (1 - vecSmoothValue[i]) / para /para);
	}

	ComputeScore();
	UpdateGraph();
}


double CScanEstimation::ComputePatchConfidenceScore(int objectIndex,int patchIndex)
{
	double confidenceScore = 0;
	for(int m = 0;m < vecPatchPoint[patchIndex].mypoints.size();m++)
	{
		MyPoint_RGB_NORMAL point;
		point = vecPatchPoint[patchIndex].mypoints[m];
		confidenceScore += ComputePointConfidenceScore(objectIndex,point);
	}
	confidenceScore /= vecPatchPoint[patchIndex].mypoints.size();

	return confidenceScore;
}

double CScanEstimation::ComputePointConfidenceScore(int objectIndex, MyPoint_RGB_NORMAL point)
{
	double confidenceScore = 0;
	for(int k = 0;k < vecObjectIsoPoint[objectIndex].objectIsoPoint.size();k++)
	{
		double distance;
		distance = sqrt((vecObjectIsoPoint[objectIndex].objectIsoPoint[k].x - point.x) * (vecObjectIsoPoint[objectIndex].objectIsoPoint[k].x - point.x)
					  + (vecObjectIsoPoint[objectIndex].objectIsoPoint[k].y - point.y) * (vecObjectIsoPoint[objectIndex].objectIsoPoint[k].y - point.y)
					  + (vecObjectIsoPoint[objectIndex].objectIsoPoint[k].z - point.z) * (vecObjectIsoPoint[objectIndex].objectIsoPoint[k].z - point.z));
		double weight;
		weight = GaussianFunction(distance);
		confidenceScore += weight* vecObjectIsoPoint[objectIndex].objectIsoPoint[k].f;
	}

	return confidenceScore;
}


void CScanEstimation::ComputeScore()
{
	for(int i = 0;i < vecvecMultiResult.size();i++)
	{
		ComputeObjectness(i); 
	}

	for(int i = 0;i < vecvecMultiResult.size();i++)
	{
		for(int j = 0;j < vecvecMultiResult.size();j++)
		{
			if(i == j)	continue;
			ComputeSeparateness(i,j);
		}
	}

	ofstream outFileh("Output\\ObjectHypo.txt");
	for(int i = 0;i < vecObjectHypo.size();i++)
	{
		ObjectHypo objectHypo;
		objectHypo = vecObjectHypo[i];
		outFileh << "objectHypo.patchIndex " << objectHypo.patchIndex.size() <<  endl;
		outFileh << "objectHypo.objectness " << objectHypo.objectness <<  endl;
		outFileh << "objectHypo.areaIndex " << objectHypo.areaIndex <<  endl;
	}
	outFileh.close();

	ofstream outFilee("Output\\EdgeHypo.txt");
	for(int i = 0;i < vecEdgeHypo.size();i++)
	{
		EdgeHypo edgeHypo;
		edgeHypo = vecEdgeHypo[i];
		outFilee << "edgeHypo.begin " << edgeHypo.begin <<  endl;
		outFilee << "edgeHypo.end " << edgeHypo.end <<  endl;
		outFilee << "edgeHypo.areaIndex " << edgeHypo.areaIndex <<  endl;
		outFilee << "edgeHypo.separateness " << edgeHypo.separateness <<  endl;

	}
	outFilee.close();

	Sorting(vecObjectHypo,vecEdgeHypo,vecObjectSorting,vecEdgeSorting);

	ofstream outFiles("Output\\Sorting.txt");
	for(int i = 0;i < vecObjectSorting.size();i++)
	{
		outFiles << "vecObjectSorting: " << vecObjectSorting[i] <<  endl;
	}
	for(int i = 0;i < vecEdgeSorting.size();i++)
	{
		outFiles << "vecEdgeSorting: " << vecEdgeSorting[i] <<  endl;
	}
	outFiles.close();

}

void CScanEstimation::ComputeObjectness(int m)
{
	double objectness = 0;
	double fSum = 0;
	double concaveSum = 0;
	for(int i = 0;i < vecvecMultiResult[m].size();i++)
	{
		int patchIndex = vecvecMultiResult[m][i];
		fSum += vecPatchConfidenceScore[patchIndex];
	}

	double edgeNum = 0;
	for(int i = 0;i < vecpairPatchConnection.size();i++)
	{
		bool firstFlag = false;
		bool secondFlag = false;
		for(int j = 0;j < vecvecMultiResult[m].size();j++)
		{
			if(vecpairPatchConnection[i].first == vecvecMultiResult[m][j])
				firstFlag = true;
			if(vecpairPatchConnection[i].second == vecvecMultiResult[m][j])
				secondFlag = true;
		}
		if(firstFlag && secondFlag)
		{
			edgeNum++;
			if(!vecGeometryConvex[i])
				concaveSum += vecSmoothValue[i];
		}
	}
	concaveSum /= edgeNum;
	concaveSum *= 1000;

	if(concaveSum < 0.000001)
		concaveSum = 0;

	objectness = fSum * concaveSum; 
	vecObjectness.push_back(objectness);

	ObjectHypo objectHypo;
	objectHypo.patchIndex = vecvecMultiResult[m];
	objectHypo.objectness = objectness;
	objectHypo.areaIndex = GetAreaIndex(vecvecMultiResult[m][0]);
	objectHypo.mergeFlag = false;
	vecObjectHypo.push_back(objectHypo);

}

void CScanEstimation::ComputeSeparateness(int m,int n)
{
	double separateness = 0;
	vector<pair<int,int>> vecpairSeperatenessSmallEdge;
	for(int i = 0;i < vecvecMultiResult[m].size();i++)
	{
		int patchIndex = vecvecMultiResult[m][i];

		// connect before
		vector<int> vecConnectPatchBefore;
		for(int j = 0;j < vecvecPatchConnectFlag[patchIndex].size();j++)
		{
			if(vecvecPatchConnectFlag[patchIndex][j])
			{
				vecConnectPatchBefore.push_back(j);
			}
		}

		// disconnnect after
		vector<pair<int,int>> vecpairDisconnect;
		pair<int,int> pairsDisconnect;
		for(int j = 0;j < vecConnectPatchBefore.size();j++)
		{
			pairsDisconnect.first = patchIndex;
			pairsDisconnect.second = vecConnectPatchBefore[j];

			for(int k = 0;k < vecvecMultiResult[n].size();k++)
			{
				if(vecvecMultiResult[n][k] == vecConnectPatchBefore[j])
					vecpairDisconnect.push_back(pairsDisconnect);
			}
		}

		for(int j = 0;j < vecpairDisconnect.size();j++)
		{
			for(int k = 0;k < vecpairPatchConnection.size();k++)
			{
				if(vecpairPatchConnection[k].first == vecpairDisconnect[j].first && vecpairPatchConnection[k].second == vecpairDisconnect[j].second)
				{
					separateness += vecSmoothValue[k];
					vecpairSeperatenessSmallEdge.push_back(vecpairDisconnect[j]);
				}	
			}
		}
	}

	if(separateness > 0)
	{
		vecSeparateness.push_back(separateness);

		EdgeHypo edgeHypo;
		edgeHypo.begin = m;
		edgeHypo.end = n;
		edgeHypo.areaIndex =  GetAreaIndex(vecvecMultiResult[m][0]);
		edgeHypo.separateness = separateness;
		vecEdgeHypo.push_back(edgeHypo);
	}
}

double CScanEstimation::GaussianFunction(double x)
{
	double para = 0.05;
	double value = pow(2.7183,- x * x / para / para);

	return value;
}

int CScanEstimation::GetAreaIndex(int patchIndex)
{
	for(int i = 0;i < clusterPatchInitIndex.size();i++)
	{
		if(patchIndex >= clusterPatchInitIndex[i] && patchIndex < clusterPatchInitIndex[i] + clusterPatchNum[i])
			return i;
	}
}

void CScanEstimation::Sorting(vector<ObjectHypo> obj,vector<EdgeHypo> edge,vector<int> &objSorting,vector<int> &edgeSorting)
{
	objSorting.clear();
	for(int i = 0;i < obj.size();i++)
	{
		objSorting.push_back(i);
	}
	
	for(int i =0 ; i< obj.size()-1; ++i) 
	{  
		for(int j = 0; j < obj.size()-i-1; ++j) 
		{  
			if(obj[j].objectness < obj[j+1].objectness)  
			{  
				double tmp = obj[j].objectness; obj[j].objectness = obj[j+1].objectness;  obj[j+1].objectness = tmp;
				int tmpIndex = objSorting[j]; objSorting[j] = objSorting[j+1]; objSorting[j+1] = tmpIndex;
			}  
		}  
	}  

	edgeSorting.clear();
	for(int i = 0;i < edge.size();i++)
	{
		edgeSorting.push_back(i);
	}

	for(int i =0 ; i< edge.size()-1; ++i) 
	{  
		for(int j = 0; j < edge.size()-i-1; ++j) 
		{  
			if(edge[j].separateness < edge[j+1].separateness)  
			{  
				double tmp = edge[j].separateness; edge[j].separateness = edge[j+1].separateness;  edge[j+1].separateness = tmp;
				int tmpIndex = edgeSorting[j]; edgeSorting[j] = edgeSorting[j+1]; edgeSorting[j+1] = tmpIndex;
			}  
		}  
	}  
}

void CScanEstimation::GetColour(double v,double vmin,double vmax,double &r,double &g,double &b)
{
	double dv;

	if (v < vmin)
		v = vmin;
	if (v > vmax)
		v = vmax;
	dv = vmax - vmin;

	if (v < (vmin + 0.25 * dv)) {
		r = 0;
		g = 4 * (v - vmin) / dv;
	} else if (v < (vmin + 0.5 * dv)) {
		r = 0;
		b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
	} else if (v < (vmin + 0.75 * dv)) {
		r = 4 * (v - vmin - 0.5 * dv) / dv;
		b = 0;
	} else {
		g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
		b = 0;
	}

}

void CScanEstimation::UpdateGraph()
{
	ofstream outFileg("Output\\11111.txt");
	outFileg << "let's begin :) " << endl;

	for(int i = 0;i < vecObjectness.size();i++)
	{
		double r,g,b;
		GetColour(vecObjectness[i],0,3,r,g,b);
		graphContract.vecNodes[i].r = r;
		graphContract.vecNodes[i].g = g;
		graphContract.vecNodes[i].b = b;
	}

	for(int i = 0;i < vecSeparateness.size();i++)
	{
		double r,g,b;
		GetColour(vecSeparateness[i],0,3,r,g,b);
		graphContract.vecEdgeColor.push_back(r);
		graphContract.vecEdgeColor.push_back(g);
		graphContract.vecEdgeColor.push_back(b);
	}
	outFileg << "111111111  " << vecObjectness.size() << "  " << vecSeparateness.size()<<endl;
	outFileg.close();
}