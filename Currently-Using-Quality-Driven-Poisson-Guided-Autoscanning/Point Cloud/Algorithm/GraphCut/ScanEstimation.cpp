#include "ScanEstimation.h"



CScanEstimation::CScanEstimation(void)
{
	paraConfidence = 2;
	paraSmoothAdjust = 0.75;
}


CScanEstimation::~CScanEstimation(void)
{
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

}

void CScanEstimation::ComputeObjectness(int m)
{
	vecObjectness.clear();

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
	vecSeparateness.clear();

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
