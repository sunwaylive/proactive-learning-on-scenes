#include "PointCloudAnalysis.h"

double maxSV,minSV;

CPointCloudAnalysis::CPointCloudAnalysis(void)
{
	paraGeometry = 0.4;
	paraAppearence = 0.3;
	paraSmoothAdjust = 0.7;
}


CPointCloudAnalysis::~CPointCloudAnalysis(void)
{
}

void CPointCloudAnalysis::MainStep(bool initFlag,int newAreNum)
{
	//output
	ofstream outFile("Output\\CCA.txt");

	InitAreaInterest();
	outFile <<  "1" <<endl;
	
	ComputeObjectHypo();
	outFile <<  "2" <<endl;

	outFile.close();
}

void CPointCloudAnalysis::ScanEstimation()
{
	ofstream outFileg("Output\\ScanEstimation.txt",ios::app);
	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		vecAreaInterest[i].ScoreUpdate();
	}
	
	double objectnessMax = SMALL_NUM;
	double objectnessMin = LARGE_NUM;

	outFileg.close();
}

void CPointCloudAnalysis::GraphUpdate()
{
	patchGraph.vecNodes.clear();
	patchGraph.vecEdges.clear();
	patchGraph.vecEdgeColor.clear();
	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		if(!vecAreaInterest[i].validFlag)	continue;
		patchGraph.vecNodes.insert(patchGraph.vecNodes.end(),vecAreaInterest[i].graphInit.vecNodes.begin(),vecAreaInterest[i].graphInit.vecNodes.end());
		patchGraph.vecEdges.insert(patchGraph.vecEdges.end(),vecAreaInterest[i].graphInit.vecEdges.begin(),vecAreaInterest[i].graphInit.vecEdges.end());
	}

	contractionGraph.vecNodes.clear();
	contractionGraph.vecEdges.clear();
	contractionGraph.vecEdgeColor.clear();
	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		if(!vecAreaInterest[i].validFlag)	continue;
		contractionGraph.vecNodes.insert(contractionGraph.vecNodes.end(),vecAreaInterest[i].graphContract.vecNodes.begin(),vecAreaInterest[i].graphContract.vecNodes.end());
		contractionGraph.vecEdges.insert(contractionGraph.vecEdges.end(),vecAreaInterest[i].graphContract.vecEdges.begin(),vecAreaInterest[i].graphContract.vecEdges.end());
		contractionGraph.vecEdgeColor.insert(contractionGraph.vecEdgeColor.end(),vecAreaInterest[i].graphContract.vecEdgeColor.begin(),vecAreaInterest[i].graphContract.vecEdgeColor.end());
	}
}

void CPointCloudAnalysis::InitAreaInterest()
{
	ofstream outFile("Output\\InitAreaInterest.txt");

	int initIndex = 0;
	for(int i = 0;i < clusterPatchNum.size(); i++)
	{
		clusterPatchInitIndex.push_back(initIndex);
		initIndex += clusterPatchNum[i];
	}
	outFile <<  "1" <<endl;

	//init areainterest
	for(int i = 0;i < clusterPatchInitIndex.size();i++)
	{
		outFile <<  i <<endl;
		int patchBegin,patchEnd;
		patchBegin = clusterPatchInitIndex[i];
		patchEnd = clusterPatchInitIndex[i] + clusterPatchNum[i];
		outFile <<  i <<endl;
		vector<MyPointCloud_RGB_NORMAL> patchPoint;
		vector<Normalt> patcNormal;
		for(int j = patchBegin;j < patchEnd;j++)
		{
			patchPoint.push_back(vecPatchPoint[j]);
			patcNormal.push_back(vecPatcNormal[j]);
		}
		outFile <<  i <<endl;

		CAreaInterest cAreaInterest(patchPoint,patcNormal);
		vecAreaInterest.push_back(cAreaInterest);
		outFile <<  i <<endl;
	}

	//bounding box
	xMax = yMax = zMax = SMALL_NUM;
	xMin = yMin = zMin = LARGE_NUM;
	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		if(xMax < vecAreaInterest[i].xMax)
			xMax = vecAreaInterest[i].xMax;
		if(yMax < vecAreaInterest[i].yMax)
			yMax = vecAreaInterest[i].yMax;
		if(zMax < vecAreaInterest[i].zMax)
			zMax = vecAreaInterest[i].zMax;
		if(xMin > vecAreaInterest[i].xMin)
			xMin = vecAreaInterest[i].xMin;
		if(yMin > vecAreaInterest[i].yMin)
			yMin = vecAreaInterest[i].yMin;
		if(zMin > vecAreaInterest[i].zMin)
			zMin = vecAreaInterest[i].zMin;
	}

	//color set
	ColorShow colorTemp;

	double colorChoice[16][3] = {
		{130, 130, 240}, {255, 120, 120}, {46, 254, 100}, {250, 88, 172},
		{250, 172, 88}, {129, 247, 216}, {150, 150, 50}, {226, 169, 143}, {8, 138, 41}, 
		{1, 223, 215}, {11, 76, 95}, {190, 182, 90},
		{245, 169, 242}, {75, 138, 8}, {247, 254, 46}, {88, 172, 250}
	};

	for(int i=0;i<16;i++)
	{
		colorTemp.r = colorChoice[i][0];	
		colorTemp.g = colorChoice[i][1]; 
		colorTemp.b = colorChoice[i][2]; 
		colorSet.color.push_back(colorTemp);
	}

	outFile.close();
}

void CPointCloudAnalysis::ComputeObjectHypo()
{
	ofstream outFile("Output\\COH.txt");

	outFile <<  "1" <<endl;
	NormalizeAppearanceTerm();
	outFile <<  "2" <<endl;
	NormalizeSmoothTerm();
	outFile <<  "3" <<endl;

	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		outFile << i <<endl;
		vecAreaInterest[i].MainStep();
	}

	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		contractionGraph.vecNodes.insert(contractionGraph.vecNodes.end(),vecAreaInterest[i].graphContract.vecNodes.begin(),vecAreaInterest[i].graphContract.vecNodes.end());
		contractionGraph.vecEdges.insert(contractionGraph.vecEdges.end(),vecAreaInterest[i].graphContract.vecEdges.begin(),vecAreaInterest[i].graphContract.vecEdges.end());
	}

	outFile.close();
}

void CPointCloudAnalysis::NormalizeAppearanceTerm()
{
	maxAV = SMALL_NUM;
	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		for(int j = 0;j < vecAreaInterest[i].vecAppearenceValue.size();j++)
		{
			if(maxAV < vecAreaInterest[i].vecAppearenceValue[j])
				maxAV = vecAreaInterest[i].vecAppearenceValue[j];
		}
	}
	maxAV = 1162;

	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		for(int j = 0;j < vecAreaInterest[i].vecAppearenceValue.size();j++)
		{
			vecAreaInterest[i].vecAppearenceValue[j] = vecAreaInterest[i].vecAppearenceValue[j] / maxAV;
			vecAreaInterest[i].vecAppearenceValue[j] = 1 - vecAreaInterest[i].vecAppearenceValue[j];
		}
	}

	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		for(int j = 0;j < vecAreaInterest[i].vecAppearenceValue.size();j++)
		{
			vecAreaInterest[i].vecAppearenceValue[j] = vecAreaInterest[i].vecAppearenceValue[j] / maxAV;
			vecAreaInterest[i].vecAppearenceValue[j] = 1 - vecAreaInterest[i].vecAppearenceValue[j];

			vecAreaInterest[i].vecGeometryValue[j] = paraGeometry * vecAreaInterest[i].vecGeometryValue[j];
			vecAreaInterest[i].vecAppearenceValue[j] = paraAppearence * vecAreaInterest[i].vecAppearenceValue[j];
			vecAreaInterest[i].vecSmoothValue[j] = vecAreaInterest[i].vecGeometryValue[j] + vecAreaInterest[i].vecAppearenceValue[j];
		}
	}


	ofstream outFile("Output\\smooth0.txt");
	outFile << "maxAV: " << maxAV << endl;
	outFile << "maxSV: " << maxSV << endl;
	outFile << "minSV: " << minSV << endl;
	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		outFile << "/////////////////////////// " << endl;
		for(int j = 0;j < vecAreaInterest[i].vecSmoothValue.size();j++)
		{
			outFile <<  "vecGeometryValue:  " << vecAreaInterest[i].vecGeometryValue[j] << endl;
			outFile <<  "vecAppearenceValue:  " << vecAreaInterest[i].vecAppearenceValue[j] << endl;
			outFile <<  "vecSmoothValue:  " << vecAreaInterest[i].vecSmoothValue[j] << endl;
			outFile << endl;
		}

	}
	outFile.close();
}

void CPointCloudAnalysis::NormalizeSmoothTerm()
{
	maxSV = SMALL_NUM;
	minSV = LARGE_NUM;

	double para = 0.3;
	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		for(int j = 0;j < vecAreaInterest[i].vecSmoothValue.size();j++)
		{
		if(maxSV < vecAreaInterest[i].vecSmoothValue[j])
			maxSV = vecAreaInterest[i].vecSmoothValue[j];
		if(minSV > vecAreaInterest[i].vecSmoothValue[j])
			minSV = vecAreaInterest[i].vecSmoothValue[j];
		}
	}
	maxSV = 0.7;
	minSV = 0.1;


	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		for(int j = 0;j < vecAreaInterest[i].vecSmoothValue.size();j++)
		{
			vecAreaInterest[i].vecSmoothValue[j] =  (vecAreaInterest[i].vecSmoothValue[j] - minSV)/(maxSV - minSV);
			vecAreaInterest[i].vecSmoothValue[j] = paraSmoothAdjust * pow(2.7183,- (1 - vecAreaInterest[i].vecSmoothValue[j]) * (1 - vecAreaInterest[i].vecSmoothValue[j]) / para /para);
		}
	}	

	ofstream outFile("Output\\smooth1.txt");
	outFile << "maxAV: " << maxAV << endl;
	outFile << "maxSV: " << maxSV << endl;
	outFile << "minSV: " << minSV << endl;
	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		outFile << "/////////////////////////// " << endl;
		for(int j = 0;j < vecAreaInterest[i].vecSmoothValue.size();j++)
		{
			outFile <<  "vecGeometryValue:  " << vecAreaInterest[i].vecGeometryValue[j] << endl;
			outFile <<  "vecAppearenceValue:  " << vecAreaInterest[i].vecAppearenceValue[j] << endl;
			outFile <<  "vecSmoothValue:  " << vecAreaInterest[i].vecSmoothValue[j] << endl;
			outFile << endl;
		}
		
	}
	outFile.close();

}

void CPointCloudAnalysis::DataIn()
{
// 	ifstream inFile("Input\\PatchPoint-table0.txt",std::ios::in);
// 	ifstream inFile1("Input\\ClusterSize-table0.txt",std::ios::in);
// 	ifstream inFile2("Input\\PatchNormal-table0.txt",std::ios::in);
// 	ifstream inFile3("Input\\TableCloud-table0.txt",std::ios::in);

// 	ifstream inFile("Input\\PatchPoint-table1.txt",std::ios::in);
// 	ifstream inFile1("Input\\ClusterSize-table1.txt",std::ios::in);
// 	ifstream inFile2("Input\\PatchNormal-table1.txt",std::ios::in);
// 	ifstream inFile3("Input\\TableCloud-table1.txt",std::ios::in);
//  
// 	ifstream inFile("Input\\PatchPoint-table2.txt",std::ios::in);
// 	ifstream inFile1("Input\\ClusterSize-table2.txt",std::ios::in);
// 	ifstream inFile2("Input\\PatchNormal-table2.txt",std::ios::in);
// 	ifstream inFile3("Input\\TableCloud-table2.txt",std::ios::in);
// 
// 	ifstream inFile("Input\\PatchPoint-table3.txt",std::ios::in);
// 	ifstream inFile1("Input\\ClusterSize-table3.txt",std::ios::in);
// 	ifstream inFile2("Input\\PatchNormal-table3.txt",std::ios::in);
// 	ifstream inFile3("Input\\TableCloud-table3.txt",std::ios::in);


	//¶Ápatch point
// 	char buf[256000];
// 	bool flagStop = false;
// 	int count=0;
// 
// 
// 	while (inFile.getline(buf, sizeof buf))
// 	{
// 		MyPointCloud_RGB_NORMAL patchTemp;
// 		istringstream line(buf);
// 
// 		flagStop = false;
// 		do
// 		{
// 			//¶Áµã×ø±ê
// 			MyPt_RGB_NORMAL pointTemp;
// 			pointTemp.x = 100;
// 			line >> pointTemp.x;
// 			line >> pointTemp.y;
// 			line >> pointTemp.z;
// 			line >> pointTemp.normal_x;
// 			line >> pointTemp.normal_y;
// 			line >> pointTemp.normal_z;
// 			line >> pointTemp.r;
// 			line >> pointTemp.g;
// 			line >> pointTemp.b;
// 
// 
// 			if(pointTemp.x<100 && pointTemp.x>-100)
// 				patchTemp.mypoints.push_back(pointTemp);
// 			else
// 				flagStop = true;
// 		}
// 		while(flagStop == false);
// 
// 		cBinarySeg.vecPatchPoint.push_back(patchTemp);
// 		count++;
// 	}
// 	inFile.close();
// 
// 	//¶Ácluster size
// 	vector<int> clusterPatchNum;
// 	flagStop = false;
// 	while (inFile1.getline(buf, sizeof buf))
// 	{
// 		istringstream line(buf);
// 		int num,num_old;
// 		do
// 		{
// 			num = -1;
// 			line >> num;
// 			if(num>0 && num<9999)
// 			{
// 				cBinarySeg.clusterPatchNum.push_back(num);
// 				num_old = num;
// 			}
// 			else
// 				flagStop = true;
// 		}
// 		while( flagStop == false);
// 	}
// 	inFile1.close();
// 
// 	//¶Ánormal
// 	vector<Normalt> vecPatchNormal;
// 	while (inFile2.getline(buf, sizeof buf))
// 	{
// 		istringstream line(buf);
// 		Normalt nor;
// 		line >> nor.normal_x;
// 		line >> nor.normal_y;
// 		line >> nor.normal_z;
// 		cBinarySeg.vecPatcNormal.push_back(nor);
// 	}
// 	inFile2.close();
// 
// 	//¶Átable point
// 	flagStop = false;
// 	count=0;
// 	while (inFile3.getline(buf, sizeof buf))
// 	{
// 		istringstream line(buf);
// 		//¶Áµã×ø±ê
// 		MyPt_RGB_NORMAL pointTemp;
// 
// 		line >> pointTemp.x;
// 		line >> pointTemp.y;
// 		line >> pointTemp.z;
// 
// 		cBinarySeg.tablePoint.mypoints.push_back(pointTemp);
// 		count++;
// 	}
// 	inFile3.close();


	ofstream outFile1("Output\\duibudui.txt");
	outFile1 << "vecPatchPoint  " << cBinarySeg.vecPatchPoint.size() << endl;
	outFile1 << "vecPatchNormal  " << cBinarySeg.vecPatcNormal.size() << endl;
	outFile1 << "clusterPatchNum  " << cBinarySeg.clusterPatchNum.size() << endl;
	outFile1.close();
}

int CPointCloudAnalysis::DataUpdate()
{
	ifstream inFile("Input\\PatchPoint-table0.txt",std::ios::in);
	ifstream inFile1("Input\\ClusterSize-table0.txt",std::ios::in);
	ifstream inFile2("Input\\PatchNormal-table0.txt",std::ios::in);
	ifstream inFile3("Input\\TableCloud-table0.txt",std::ios::in);


	//¶Ápatch point
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
			line >> pointTemp.normal_x;
			line >> pointTemp.normal_y;
			line >> pointTemp.normal_z;
			line >> pointTemp.r;
			line >> pointTemp.g;
			line >> pointTemp.b;


			if(pointTemp.x<100 && pointTemp.x>-100)
				patchTemp.mypoints.push_back(pointTemp);
			else
				flagStop = true;
		}
		while(flagStop == false);

		cBinarySeg.vecPatchPoint.push_back(patchTemp);
		count++;
	}
	inFile.close();

	//¶Ácluster size
	int newAreaNum = 0;
	vector<int> clusterPatchNum;
	flagStop = false;
	while (inFile1.getline(buf, sizeof buf))
	{
		istringstream line(buf);
		int num,num_old;
		do
		{
			num = -1;
			line >> num;
			if(num>0 && num<9999)
			{
				cBinarySeg.clusterPatchNum.push_back(num);
				num_old = num;
				newAreaNum++;
			}
			else
				flagStop = true;
		}
		while( flagStop == false);
	}
	inFile1.close();

	//¶Ánormal
	vector<Normalt> vecPatchNormal;
	while (inFile2.getline(buf, sizeof buf))
	{
		istringstream line(buf);
		Normalt nor;
		line >> nor.normal_x;
		line >> nor.normal_y;
		line >> nor.normal_z;
		cBinarySeg.vecPatcNormal.push_back(nor);
	}
	inFile2.close();

	return newAreaNum;
}

void CPointCloudAnalysis::BinarySegmentation(bool initFlag, int newAreaNum)
{
	cBinarySeg.MainStep(initFlag,newAreaNum);
}

void CPointCloudAnalysis::Clustering()
{
	cClustering.Clear();
	cClustering.vecPatchPoint = cBinarySeg.vecPatchPoint;
	cClustering.vecvecObjectPool = cBinarySeg.vecvecObjectPool;
	cClustering.MainStep();

}

void CPointCloudAnalysis::MultiSegmentation()
{
	cMultiSeg.Clear();

// 	cMultiSeg.vecPatchPoint = cBinarySeg.vecPatchPoint;
// 	cMultiSeg.vecpairPatchConnection = cBinarySeg.vecpairPatchConnection;
// 	cMultiSeg.vecSmoothValue = cBinarySeg.vecSmoothValue;
// 	cMultiSeg.vecPatchCenPoint = cBinarySeg.vecPatchCenPoint;
// 	cMultiSeg.vecPatchColor = cBinarySeg.vecPatchColor;
// 	cMultiSeg.boundingBoxSize = cBinarySeg.boundingBoxSize;
// 	cMultiSeg.clusterPatchNum = cBinarySeg.clusterPatchNum;
// 	cMultiSeg.clusterPatchInitIndex = cBinarySeg.clusterPatchInitIndex;
// 	cMultiSeg.clusterPatchNumLocal = cBinarySeg.clusterPatchNumLocal;
// 	cMultiSeg.clusterPatchInitIndexLocal = cBinarySeg.clusterPatchInitIndexLocal;
// 	cMultiSeg.vecvecPatchConnectFlag = cBinarySeg.vecvecPatchConnectFlag;
	cMultiSeg.xMax = cBinarySeg.xMax;
	cMultiSeg.xMin = cBinarySeg.xMin;
	cMultiSeg.yMax = cBinarySeg.yMax;
	cMultiSeg.yMin = cBinarySeg.yMin;
	cMultiSeg.zMax = cBinarySeg.zMax;
	cMultiSeg.zMin = cBinarySeg.zMin;

	cMultiSeg.vecvecObjectPoolClustering = cClustering.vecvecObjectPoolClustering;
	cMultiSeg.vecObjectPoolClusteringCount = cClustering.vecObjectPoolClusteringCount;

	cMultiSeg.MainStep();

}

void CPointCloudAnalysis::Merge(int pushArea)
{
	ofstream outFileg("Output\\Merge.txt");
	outFileg << "let's begin :) " << endl;

	//merge area
	vecAreaInterest[pushArea].mergeFlag = true;
	vecAreaInterest[pushArea].GetNewOneObjectPly();
	vecAreaInterest[pushArea].ComputeOverallHypo();
//	vecAreaInterest[pushArea].ComputeNewContractGraph();

	outFileg.close();
}

void CPointCloudAnalysis::ReAnalysis(int pushArea,int newAreaNum)
{
	ofstream outFile("Output\\ReAnalysis.txt");

	//invalid the area
	vecAreaInterest[pushArea].Invalidt();
	outFile << "1" << endl;

	int initIndex = 0;
	clusterPatchInitIndex.clear();
	for(int i = 0;i < clusterPatchNum.size(); i++)
	{
		clusterPatchInitIndex.push_back(initIndex);
		initIndex += clusterPatchNum[i];
	}
	outFile << "2" << endl;

	for(int i = clusterPatchInitIndex.size() - newAreaNum;i < clusterPatchInitIndex.size();i++)
	{
		outFile << "i:" << i << endl;
		int patchBegin,patchEnd;
		patchBegin = clusterPatchInitIndex[i];
		patchEnd = clusterPatchInitIndex[i] + clusterPatchNum[i];

		outFile << "i:" << i << endl;
		vector<MyPointCloud_RGB_NORMAL> patchPoint;
		vector<Normalt> patcNormal;

		outFile << "i:" << i << endl;
		for(int j = patchBegin;j < patchEnd;j++)
		{
			patchPoint.push_back(vecPatchPoint[j]);
			patcNormal.push_back(vecPatcNormal[j]);
		}

		outFile << "i:" << i << endl;
		CAreaInterest cAreaInterest(patchPoint,patcNormal);
		vecAreaInterest.push_back(cAreaInterest);

		outFile << "i:" << i << endl;
	}
	outFile << "3" << endl;

	for(int i = clusterPatchInitIndex.size() - newAreaNum;i < clusterPatchInitIndex.size();i++)
	{
		NormalizeNewAppearanceTerm(i);
		NormalizeNewSmoothTerm(i);
	}
	outFile << "4" << endl;

	for(int i = clusterPatchInitIndex.size() - newAreaNum;i < clusterPatchInitIndex.size();i++)
	{
		vecAreaInterest[i].MainStep();
	}
	outFile << "5" << endl;

	outFile.close();
}

void CPointCloudAnalysis::NormalizeNewAppearanceTerm(int areaIndex)
{
	for(int i = 0;i < vecAreaInterest[areaIndex].vecAppearenceValue.size();i++)
	{
		vecAreaInterest[areaIndex].vecAppearenceValue[i] = vecAreaInterest[areaIndex].vecAppearenceValue[i] / maxAV;
		vecAreaInterest[areaIndex].vecAppearenceValue[i] = 1 - vecAreaInterest[areaIndex].vecAppearenceValue[i];
	}


	for(int i = 0;i < vecAreaInterest[areaIndex].vecAppearenceValue.size();i++)
	{
		vecAreaInterest[areaIndex].vecAppearenceValue[i] = vecAreaInterest[areaIndex].vecAppearenceValue[i] / maxAV;
		vecAreaInterest[areaIndex].vecAppearenceValue[i] = 1 - vecAreaInterest[areaIndex].vecAppearenceValue[i];

		vecAreaInterest[areaIndex].vecGeometryValue[i] = paraGeometry * vecAreaInterest[areaIndex].vecGeometryValue[i];
		vecAreaInterest[areaIndex].vecAppearenceValue[i] = paraAppearence * vecAreaInterest[areaIndex].vecAppearenceValue[i];
		vecAreaInterest[areaIndex].vecSmoothValue[i] = vecAreaInterest[areaIndex].vecGeometryValue[i] + vecAreaInterest[areaIndex].vecAppearenceValue[i];
	}

	ofstream outFile("Output\\smooth2.txt");
	outFile << "maxAV: " << maxAV << endl;
	outFile << "maxSV: " << maxSV << endl;
	outFile << "minSV: " << minSV << endl;
	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		outFile << "/////////////////////////// " << endl;
		for(int j = 0;j < vecAreaInterest[i].vecSmoothValue.size();j++)
		{
			outFile <<  "vecGeometryValue:  " << vecAreaInterest[i].vecGeometryValue[j] << endl;
			outFile <<  "vecAppearenceValue:  " << vecAreaInterest[i].vecAppearenceValue[j] << endl;
			outFile <<  "vecSmoothValue:  " << vecAreaInterest[i].vecSmoothValue[j] << endl;
			outFile << endl;
		}

	}
	outFile.close();
}

void CPointCloudAnalysis::NormalizeNewSmoothTerm(int areaIndex)
{
	double para = 0.3;
	for(int i = 0;i < vecAreaInterest[areaIndex].vecSmoothValue.size();i++)
	{
		vecAreaInterest[areaIndex].vecSmoothValue[i] =  (vecAreaInterest[areaIndex].vecSmoothValue[i] - minSV)/(maxSV - minSV);
		vecAreaInterest[areaIndex].vecSmoothValue[i] = paraSmoothAdjust * pow(2.7183,- (1 - vecAreaInterest[areaIndex].vecSmoothValue[i]) * (1 - vecAreaInterest[areaIndex].vecSmoothValue[i]) / para /para);
	}

	ofstream outFile("Output\\smooth3.txt");
	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		outFile << "/////////////////////////// " << endl;
		for(int j = 0;j < vecAreaInterest[i].vecSmoothValue.size();j++)
		{
			outFile <<  "vecGeometryValue:  " << vecAreaInterest[i].vecGeometryValue[j] << endl;
			outFile <<  "vecAppearenceValue:  " << vecAreaInterest[i].vecAppearenceValue[j] << endl;
			outFile <<  "vecSmoothValue:  " << vecAreaInterest[i].vecSmoothValue[j] << endl;
			outFile << endl;
		}

	}
	outFile.close();
}

void CPointCloudAnalysis::ScanEstimation(int areaIndex)
{
	ofstream outFileg("Output\\ObjectnessSeparateness.txt",ios::app);
	vecAreaInterest[areaIndex].ScoreUpdate();
	outFileg << "area: " <<  areaIndex << endl;
	outFileg.close();
}