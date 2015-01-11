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
	ofstream outFilegs("Output\\SmoothValue.txt",ios::app);
	for(int i = 0;i < vecSmoothValue.size();i++)
	{
		outFilegs << "vecSmoothValue: " << vecSmoothValue[i] << endl;
	}
	outFilegs.close();


	ofstream outFileg("Output\\RunSU.txt");
	outFileg << "let's begin :) " << endl;

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

	outFileg << "let's begin 1:) " << endl;
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

	outFileg << "let's begin 2:) " << endl;
	ComputeScore();
	outFileg << "let's begin 3:) " << endl;
	UpdateGraph();
	outFileg << "let's begin 4:) " << endl;

	outFileg << "Show results finished :)   " << endl;

	outFileg.close();
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

	Sorting(vecObjectHypo,vecEdgeHypo,vecObjectSorting,vecEdgeSorting);

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
	ofstream outFiles("Output\\Objectness.txt",ios::app);

	double objectness = 0;
	double fSum = 0;
	double concaveSum = 0;
	for(int i = 0;i < vecvecMultiResult[m].size();i++)
	{
		int patchIndex = vecvecMultiResult[m][i];
		fSum += vecPatchConfidenceScore[patchIndex];
	}
	outFiles << "1 " <<  endl;

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
	outFiles << "2 " <<  endl;

	if(edgeNum > 0)
		concaveSum /= edgeNum;
	else
		concaveSum = 0;

	concaveSum *= 1000;

	if(concaveSum < 0.000001)
		concaveSum = 0;

	objectness = fSum * concaveSum; 

	outFiles << "3 " <<  endl;

	vecObjectHypo[m].objectness = objectness;

	outFiles << "4 " <<  endl;
	outFiles.close();
}

void CScanEstimation::ComputeSeparateness(int m,int n)
{
	double separateness = 0;
	int hypoIndex;

	

	for(int i = 0;i < vecEdgeHypo.size();i++)
	{
		if(vecEdgeHypo[i].begin == m && vecEdgeHypo[i].end == n)
		{
			hypoIndex = i;
		}
	}

	for(int i = 0;i < vecEdgeHypo[hypoIndex].pairPatch.size();i++)
	{
		for(int j = 0;j < vecpairPatchConnection.size();j++)
		{
			if(vecEdgeHypo[hypoIndex].pairPatch[i].first == vecpairPatchConnection[j].first && vecEdgeHypo[hypoIndex].pairPatch[i].second == vecpairPatchConnection[j].second)
			{
				separateness += vecSmoothValue[j];
				break;
			}
		}
	}
	
	vecEdgeHypo[hypoIndex].separateness = separateness;
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
	r = g = b =1.0;
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
	double objMax,objMin,sepaMax,sepaMin;
	objMax = sepaMax = SMALL_NUM;
	sepaMax = sepaMin = LARGE_NUM;

	for(int i = 0;i < vecObjectHypo.size();i++)
	{
		if(objMax < vecObjectHypo[i].objectness)
			objMax =vecObjectHypo[i].objectness;
		if(objMin > vecObjectHypo[i].objectness)
			objMin = vecObjectHypo[i].objectness;
	}
	objMax = 2000;
	objMin = 0;

	for(int i = 0;i < vecEdgeHypo.size();i++)
	{
		if(sepaMax < vecEdgeHypo[i].separateness)
			sepaMax =vecEdgeHypo[i].separateness;
		if(sepaMin > vecEdgeHypo[i].separateness)
			sepaMin = vecEdgeHypo[i].separateness;

	}
	sepaMax = 0.5;
	sepaMin = 0;


	for(int i = 0;i < vecObjectHypo.size();i++)
	{
		if(vecObjectHypo[i].objectness < 0)
			vecObjectHypo[i].objectness = 0;
		double r,g,b;
		GetColour(vecObjectHypo[i].objectness,objMin,objMax,r,g,b);
		graphContract.vecNodes[i].r = r;
		graphContract.vecNodes[i].g = g;
		graphContract.vecNodes[i].b = b;
	}

	for(int i = 0;i < vecEdgeHypo.size();i++)
	{
		double r,g,b;
		GetColour(vecEdgeHypo[i].separateness,sepaMin,sepaMax,r,g,b);
		graphContract.vecEdgeColor.push_back(r);
		graphContract.vecEdgeColor.push_back(g);
		graphContract.vecEdgeColor.push_back(b);
	}

}