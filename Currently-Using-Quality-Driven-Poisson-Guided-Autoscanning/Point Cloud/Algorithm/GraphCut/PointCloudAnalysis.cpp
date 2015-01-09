#include "PointCloudAnalysis.h"


CPointCloudAnalysis::CPointCloudAnalysis(void)
{
}


CPointCloudAnalysis::~CPointCloudAnalysis(void)
{
}


void CPointCloudAnalysis::MainStep(bool initFlag,int newAreNum)
{
	//output
	ofstream outFile1("Output\\MainStep.txt",ios::app);
	outFile1 <<  "  0" <<endl;

	if(initFlag)
		DataIn();
	outFile1 <<  "  1" <<endl;
	BinarySegmentation(initFlag,newAreNum);
	outFile1 <<  "  2" <<endl;
	Clustering();
	outFile1 <<  "  3" <<endl;
	MultiSegmentation();
	outFile1 <<  "  4" <<endl;
	ScanEstimation();
	outFile1 <<  "  5" <<endl;
	outFile1.close();
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

	cMultiSeg.vecPatchPoint = cBinarySeg.vecPatchPoint;
	cMultiSeg.vecpairPatchConnection = cBinarySeg.vecpairPatchConnection;
	cMultiSeg.vecSmoothValue = cBinarySeg.vecSmoothValue;
	cMultiSeg.vecPatchCenPoint = cBinarySeg.vecPatchCenPoint;
	cMultiSeg.vecPatchColor = cBinarySeg.vecPatchColor;
	cMultiSeg.boundingBoxSize = cBinarySeg.boundingBoxSize;
	cMultiSeg.clusterPatchNum = cBinarySeg.clusterPatchNum;
	cMultiSeg.clusterPatchInitIndex = cBinarySeg.clusterPatchInitIndex;
	cMultiSeg.vecvecPatchConnectFlag = cBinarySeg.vecvecPatchConnectFlag;

	cMultiSeg.vecvecObjectPoolClustering = cClustering.vecvecObjectPoolClustering;
	cMultiSeg.vecObjectPoolClusteringCount = cClustering.vecObjectPoolClusteringCount;

	cMultiSeg.MainStep();
}

void CPointCloudAnalysis::ScanEstimation()
{
	cScanEstimation.Clear();

	cScanEstimation.vecSmoothValue = cBinarySeg.vecSmoothValue;
	cScanEstimation.vecGeometryConvex = cBinarySeg.vecGeometryConvex;
	cScanEstimation.vecGeometryValue = cBinarySeg.vecGeometryValue;
	cScanEstimation.vecAppearenceValue = cBinarySeg.vecAppearenceValue;
	cScanEstimation.maxSV = cBinarySeg.maxSV;
	cScanEstimation.minSV = cBinarySeg.minSV;

	cScanEstimation.vecPatchPoint = cMultiSeg.vecPatchPoint;
	cScanEstimation.vecvecMultiResult = cMultiSeg.vecvecMultiResult;
	cScanEstimation.clusterPatchNum = cMultiSeg.clusterPatchNum;
	cScanEstimation.clusterPatchInitIndex = cMultiSeg.clusterPatchInitIndex;
	cScanEstimation.vecvecPatchConnectFlag = cMultiSeg.vecvecPatchConnectFlag;
	cScanEstimation.vecpairSeperatenessEdge = cMultiSeg.vecpairSeperatenessEdge;
	cScanEstimation.vecpairPatchConnection = cMultiSeg.vecpairPatchConnection;
	cScanEstimation.vecvecpairSeperatenessSmallEdge = cMultiSeg.vecvecpairSeperatenessSmallEdge;
	cScanEstimation.vecObjectness = cMultiSeg.vecObjectness;
	cScanEstimation.vecSeparateness = cMultiSeg.vecSeparateness;
	cScanEstimation.graphContract = cMultiSeg.graphContract;
}

void CPointCloudAnalysis::Merge(int pushArea)
{
	//merge area
	cBinarySeg.vecAreaInterest[pushArea].Merge();

	//add a objecthypo
	ObjectHypo objectHypo;
	objectHypo.areaIndex = pushArea;
	objectHypo.objectness = 0;
	objectHypo.patchIndex.clear();
	objectHypo.mergeFlag = true;
	cScanEstimation.vecObjectHypo.push_back(objectHypo);

	//delete some objecthypo
	for(int i = 0;i < cScanEstimation.vecObjectHypo.size();)
	{
		if(cScanEstimation.vecObjectHypo[i].areaIndex == pushArea)
			cScanEstimation.vecObjectHypo.erase(cScanEstimation.vecObjectHypo.begin() + i);
		else
			i++;
	}

}

void CPointCloudAnalysis::ReAnalysis(int pushArea)
{
	//invalid the area
	cBinarySeg.vecAreaInterest[pushArea].Invalidt();

	//delete some objecthypo
	for(int i = 0;i < cScanEstimation.vecObjectHypo.size();)
	{
		if(cScanEstimation.vecObjectHypo[i].areaIndex == pushArea)
			cScanEstimation.vecObjectHypo.erase(cScanEstimation.vecObjectHypo.begin() + i);
		else
			i++;
	}
}
