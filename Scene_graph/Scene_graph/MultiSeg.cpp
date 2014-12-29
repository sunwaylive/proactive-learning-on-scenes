#include "MultiSeg.h"


CMultiSeg::CMultiSeg(void)
{

}

CMultiSeg::~CMultiSeg(void)
{

}

void CMultiSeg::AddObjectPool()
{
	//¶Ápatch point
	ifstream inFile("Output\\PatchPoint-big.txt",std::ios::in);
	char buf[256000];
	bool flagStop = false;
	int count=0;
	while (inFile.getline(buf, sizeof buf))
	{
		MyPointCloud_RGB_NORMAL patchTemp;
		istringstream line(buf);

		flagStop = false;
		do
		{
			//¶Áµã×ø±ê
			MyPt_RGB_NORMAL pointTemp;
			pointTemp.x = 100;
			line >> pointTemp.x;
			line >> pointTemp.y;
			line >> pointTemp.z;
			line >> pointTemp.r;
			line >> pointTemp.g;
			line >> pointTemp.b;

			if(pointTemp.x<100 && pointTemp.x>-100)
				patchTemp.mypoints.push_back(pointTemp);
			else
				flagStop = true;
		}
		while(flagStop == false);

		vecPatchPoint.push_back(patchTemp);
		count++;
	}
	inFile.close();
}


void CMultiSeg::MainStep()
{
	GetColorModel();

	clusterPatchInterval.push_back(0);
	for(int i = 0;i < clusterPatchNum.size();i++)
	{
		int interval = 0;
		for(int j = 0;j <= i;j++)
		{
			interval += clusterPatchNum[j];
		}
		clusterPatchInterval.push_back(interval);
	}

	for(int i = 0;i < vecvecObjectPoolClustering.size();i++)
	{
		for(int j = 0;j < clusterPatchInterval.size();j++)
		{
			if(vecvecObjectPoolClustering[i][0] < clusterPatchInterval[j])
			{
				vecObjectClusteringIndex.push_back(j - 1);
				break;
			}
		}
	}
	
	int num_sites, num_labels;
	num_sites = vecPatchPoint.size();
	num_labels = vecvecObjectPoolClustering.size();

	GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(num_sites, num_labels);
//	GCoptimizationGeneralGraph gc(num_sites, num_labels);
	
	for(int i = 0;i < num_labels;i++)
	{
		for(int j = 0;j < num_labels;j++)
		{
			gc->setSmoothCost(i,j,1);
		}
	}

	//smooth
	for(int i = 0;i < vecpairPatchConnection.size();i++)
	{
// 		if(i%2 == 0)
// 		{
			gc->setNeighbors(vecpairPatchConnection[i].first,vecpairPatchConnection[i].second,vecSmoothValue[i]);
//		}
	}

	//output
	ofstream outFile2("Output\\MultiSmooth.txt",ios::app);
	for(int i = 0;i < vecpairPatchConnection.size();i++)
	{
		outFile2 <<  "vecSmoothValue[i]:" << vecSmoothValue[i] << endl;
	}
	outFile2.close();

	
	//data
	for(int i = 0;i < num_sites;i++)
	{
		for(int j = 0;j < num_labels;j++)
		{
			int siteCluster;
			for(int k = 0;k < clusterPatchInterval.size();k++)
			{	
				if(i < clusterPatchInterval[k])
				{
					siteCluster = k - 1;
					break;
				}
			}
			if(siteCluster == vecObjectClusteringIndex[j])
			{
				double dataValue = GetMultiDataValue(i,j);
				if(dataValue < 0)
					dataValue = 0;
				gc->setDataCost(i,j,dataValue);
			}
			else
			{
				gc->setDataCost(i,j,LARGE_NUM);
			}
		}
	}
	

	gc->swap(2);

	vector<int> vecResult(num_sites,-1);
	for ( int  i = 0; i < num_sites; i++ )
		vecResult[i] = gc->whatLabel(i);

	vecvecMultiResult.resize(num_labels);
	for(int i = 0;i < vecResult.size();i++)
	{
		vecvecMultiResult[vecResult[i]].push_back(i);
	}

}

void CMultiSeg::GetColorModel()
{
	for(int i = 0;i < vecvecObjectPoolClustering.size();i++)
	{
		double muRed,muGreen,muBlue;
		double sitaRed,sitaGreen,sitaBlue;
		double sitaRG,sitaRB,sitaGB;
		muRed = muGreen = muBlue = sitaRed = sitaGreen = sitaBlue = sitaRG = sitaRB = sitaGB = 0;
		int pointNum;
		pointNum = 0;

		for(int j = 0;j <vecvecObjectPoolClustering[i].size();j ++)
		{
			double patchIndex = vecvecObjectPoolClustering[i][j];
			for(int k = 0;k < vecPatchPoint[patchIndex].mypoints.size();k++)
			{
				muRed += vecPatchPoint[patchIndex].mypoints[k].r;
				muGreen += vecPatchPoint[patchIndex].mypoints[k].g;
				muBlue += vecPatchPoint[patchIndex].mypoints[k].b;
				pointNum++;
			}
		}
		muRed /= pointNum;
		muGreen /= pointNum;
		muBlue /= pointNum;

		for(int j = 0;j <vecvecObjectPoolClustering[i].size();j ++)
		{
			double patchIndex = vecvecObjectPoolClustering[i][j];
			for(int k = 0;k < vecPatchPoint[patchIndex].mypoints.size();k++)
			{
				sitaRed += (vecPatchPoint[patchIndex].mypoints[k].r - muRed) * (vecPatchPoint[patchIndex].mypoints[k].r - muRed);
				sitaGreen += (vecPatchPoint[patchIndex].mypoints[k].g - muGreen) * (vecPatchPoint[patchIndex].mypoints[k].g - muGreen);
				sitaBlue += (vecPatchPoint[patchIndex].mypoints[k].b - muBlue) * (vecPatchPoint[patchIndex].mypoints[k].b - muBlue);

				sitaRG += (vecPatchPoint[patchIndex].mypoints[k].r - muRed) * (vecPatchPoint[patchIndex].mypoints[k].g - muGreen);
				sitaRB += (vecPatchPoint[patchIndex].mypoints[k].r - muRed) * (vecPatchPoint[patchIndex].mypoints[k].b - muBlue);
				sitaGB += (vecPatchPoint[patchIndex].mypoints[k].g - muGreen) * (vecPatchPoint[patchIndex].mypoints[k].b - muBlue);
			}
		}

		sitaRed /= pointNum - 1;
		sitaGreen /= pointNum - 1;
		sitaBlue /= pointNum - 1;
		sitaRG /= pointNum - 1;
		sitaRB /= pointNum - 1;
		sitaGB /= pointNum - 1;

		COLORMODEL colorModel;
		colorModel.muBlue = muBlue;
		colorModel.muGreen = muGreen;
		colorModel.muRed = muRed;
		colorModel.sitaBlue = sitaBlue;
		colorModel.sitaGreen = sitaGreen;
		colorModel.sitaRed = sitaRed;
		colorModel.sitaRG = sitaRG;
		colorModel.sitaRB = sitaRB;
		colorModel.sitaGB = sitaGB;
		vecObjectColorModel.push_back(colorModel);
	}
}

double CMultiSeg::GetMultiDataValue(int SiteID,int LableID)
{
	//center distance
	MyPoint objectCen;
	objectCen.x = objectCen.y = objectCen.z = 0;
	for(int i = 0;i < vecvecObjectPoolClustering[LableID].size();i++)
	{
		int patchIndex = vecvecObjectPoolClustering[LableID][i];
		objectCen.x += vecPatchCenPoint[patchIndex].x;
		objectCen.y += vecPatchCenPoint[patchIndex].y;
		objectCen.z += vecPatchCenPoint[patchIndex].z;
	}
	objectCen.x /= vecvecObjectPoolClustering[LableID].size();
	objectCen.y /= vecvecObjectPoolClustering[LableID].size();
	objectCen.z /= vecvecObjectPoolClustering[LableID].size();

	double centerDistance;
	centerDistance = sqrt((vecPatchCenPoint[SiteID].x - objectCen.x) * (vecPatchCenPoint[SiteID].x - objectCen.x) +
			(vecPatchCenPoint[SiteID].y - objectCen.y) * (vecPatchCenPoint[SiteID].y - objectCen.y) +
			(vecPatchCenPoint[SiteID].z - objectCen.z) * (vecPatchCenPoint[SiteID].z - objectCen.z));
	centerDistance /= boundingBoxSize;
	centerDistance = 1 - centerDistance;

	//object count
	double objectCount;
	if(find(vecvecObjectPoolClustering[LableID].begin(),vecvecObjectPoolClustering[LableID].end(),SiteID)
		== vecvecObjectPoolClustering[LableID].end())
		objectCount = 0;
	else
		objectCount = vecObjectPoolClusteringCount[LableID];
	
	int objectCountMax = -1;
	for(int i = 0;i < vecObjectPoolClusteringCount.size();i++)
	{
		if(objectCountMax < vecObjectPoolClusteringCount[i])
			objectCountMax = vecObjectPoolClusteringCount[i];
	}
	objectCount /= objectCountMax;
	objectCount = (7 - 10 * objectCount)/7;
	if(objectCount < 0)
		objectCount =0;


	//color similarity
	double colorSimilarity;
	Matrix covariance(3,3);
	covariance(0,0) = vecObjectColorModel[LableID].sitaRed;
	covariance(1,1) = vecObjectColorModel[LableID].sitaGreen;
	covariance(2,2) = vecObjectColorModel[LableID].sitaBlue;
	covariance(0,1) = covariance(1,0) = vecObjectColorModel[LableID].sitaRG;
	covariance(0,2) = covariance(2,0) = vecObjectColorModel[LableID].sitaRB;
	covariance(1,2) = covariance(1,2) = vecObjectColorModel[LableID].sitaGB;

	Matrix dif(1,3);
	Matrix difT(3,1);
	dif(0,0) = difT(0,0) = vecPatchColor[SiteID].mRed - vecObjectColorModel[LableID].muRed;
	dif(0,1) = difT(1,0) = vecPatchColor[SiteID].mGreen - vecObjectColorModel[LableID].muGreen;
	dif(0,2) = difT(2,0) = vecPatchColor[SiteID].mBlue - vecObjectColorModel[LableID].muBlue;
	
	Matrix covarianceInv,difTCovarianceInv,result;
	covarianceInv = covariance.Inv();
	difTCovarianceInv = dif * covarianceInv;
	result = difTCovarianceInv* difT;
	colorSimilarity = 1 / covariance.Det() * pow(2.7183, -0.5 * result(0,0));
	colorSimilarity = 1- colorSimilarity;

	double dataValue;
	double para0,para1,para2;
	para0 = 0;
	para1 = 10;
	para2 = 0;

	dataValue = (para0 * centerDistance + para1 * objectCount + para2 * colorSimilarity) / (para0 + para1 + para2) * 3;

	//output
	ofstream outFile2("Output\\MultiData.txt",ios::app);
	outFile2 <<  "centerDistance:" << centerDistance <<  "  objectCount:" << objectCount <<  "  colorSimilarity:" << colorSimilarity <<  "  dataValue:" << dataValue << endl;
	outFile2 << "  " << endl;
	outFile2.close();

	return dataValue; 
}
