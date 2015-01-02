#include "PointCloudAnalysis.h"


CPointCloudAnalysis::CPointCloudAnalysis(void)
{
}


CPointCloudAnalysis::~CPointCloudAnalysis(void)
{
}


void CPointCloudAnalysis::MainStep()
{
	DataIn();
	BinarySegmentation();
	Clustering();
	MultiSegmentation();
	//ScanEstimation();
}


void CPointCloudAnalysis::DataIn()
{
	ifstream inFile("Input\\PatchPoint-table0.txt",std::ios::in);
	ifstream inFile1("Input\\ClusterSize-table0.txt",std::ios::in);
	ifstream inFile2("Input\\PatchNormal-table0.txt",std::ios::in);
	ifstream inFile3("Input\\TableCloud-table0.txt",std::ios::in);

	// 	ifstream inFile("Input\\PatchPoint-table1.txt",std::ios::in);
	// 	ifstream inFile1("Input\\ClusterSize-table1.txt",std::ios::in);
	// 	ifstream inFile2("Input\\PatchNormal-table1.txt",std::ios::in);
	// 	ifstream inFile3("Input\\TableCloud-table1.txt",std::ios::in);

	// 	ifstream inFile("Input\\PatchPoint-table2.txt",std::ios::in);
	// 	ifstream inFile1("Input\\ClusterSize-table2.txt",std::ios::in);
	// 	ifstream inFile2("Input\\PatchNormal-table2.txt",std::ios::in);
	// 	ifstream inFile3("Input\\TableCloud-table2.txt",std::ios::in);

	// 	ifstream inFile("Input\\PatchPoint-table3.txt",std::ios::in);
	// 	ifstream inFile1("Input\\ClusterSize-table3.txt",std::ios::in);
	// 	ifstream inFile2("Input\\PatchNormal-table3.txt",std::ios::in);
	// 	ifstream inFile3("Input\\TableCloud-table3.txt",std::ios::in);


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
				clusterPatchNum.push_back(num);
				num_old = num;
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
		vecPatchNormal.push_back(nor);
	}
	inFile2.close();

	//¶Ápatch point
	flagStop = false;
	count=0;
	while (inFile3.getline(buf, sizeof buf))
	{
		istringstream line(buf);
		//¶Áµã×ø±ê
		MyPt_RGB_NORMAL pointTemp;

		line >> pointTemp.x;
		line >> pointTemp.y;
		line >> pointTemp.z;

		cBinarySeg.tablePoint.mypoints.push_back(pointTemp);
		count++;
	}
	inFile3.close();


	int begin = 0;
	int end = 0;
	for(int i = 0;i < clusterPatchNum.size();i++)
	{
		begin = end ;
		end = begin + clusterPatchNum[i];
		vector<MyPointCloud_RGB_NORMAL> vecPatchPointTemp;
		for(int j = begin;j < end;j++)
		{
			vecPatchPointTemp.push_back(cBinarySeg.vecPatchPoint[j]);
		}
		cBinarySeg.AddClusterPoints(vecPatchPointTemp);
	}
	cBinarySeg.AddPatchNormal(vecPatchNormal);
}

void CPointCloudAnalysis::BinarySegmentation()
{
	cBinarySeg.MainStep();
}

void CPointCloudAnalysis::Clustering()
{
	cClustering.vecPatchPoint = cBinarySeg.vecPatchPoint;

	cClustering.MainStep();
}

void CPointCloudAnalysis::MultiSegmentation()
{
	cMultiSeg.vecPatchPoint = cBinarySeg.vecPatchPoint;
	cMultiSeg.vecpairPatchConnection = cBinarySeg.vecpairPatchConnection;
	cMultiSeg.vecSmoothValue = cBinarySeg.vecSmoothValue;
	cMultiSeg.vecPatchCenPoint = cBinarySeg.vecPatchCenPoint;
	cMultiSeg.vecPatchColor = cBinarySeg.vecPatchColor;
	cMultiSeg.boundingBoxSize = cBinarySeg.boundingBoxSize;
	cMultiSeg.clusterPatchNum = cBinarySeg.clusterPatchNum;
	cMultiSeg.vecvecPatchConnectFlag = cBinarySeg.vecvecPatchConnectFlag;

	cMultiSeg.MainStep();
}

void CPointCloudAnalysis::ScanEstimation(CMesh *original)
{
	cScanEstimation.vecSmoothValue = cBinarySeg.vecSmoothValue;
	cScanEstimation.vecGeometryConvex = cBinarySeg.vecGeometryConvex;
	cScanEstimation.maxSV = cBinarySeg.maxSV;
	cScanEstimation.minSV = cBinarySeg.minSV;

	cScanEstimation.vecPatchPoint = cMultiSeg.vecPatchPoint;
	cScanEstimation.vecvecMultiResult = cMultiSeg.vecvecMultiResult;
	cScanEstimation.vecvecPatchConnectFlag = cMultiSeg.vecvecPatchConnectFlag;
	cScanEstimation.vecpairSeperatenessEdge = cMultiSeg.vecpairSeperatenessEdge;
	cScanEstimation.vecpairPatchConnection = cMultiSeg.vecpairPatchConnection;
	cScanEstimation.vecvecpairSeperatenessSmallEdge = cMultiSeg.vecvecpairSeperatenessSmallEdge;
	cScanEstimation.vecObjectness = cMultiSeg.vecObjectness;
	cScanEstimation.vecSeparateness = cMultiSeg.vecSeparateness;

	cScanEstimation.MainStep(original);
}