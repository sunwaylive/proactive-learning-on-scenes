#include "GraphCutGlobalHeader.h"
#include "BinarySeg.h"



CBinarySeg::CBinarySeg()
{
	thresholdClose0 = 0.005; 

	paraSmallK = 1;
	paraSmallS = 0.02;
	paraLargeK = 20;
	paraLargeS = 0.1;

	paraConvexK = 0.1;
	paraConvexT = 0.9;
	paraConcave = 1.0;

	paraGeometry = 0.4;
	paraAppearence = 0.3;

	paraMinPatchInObject = 3;
	paraMaxCutEnergy = 18.328;

	paraSmoothAdjust = 0.7;

	maxSV = SMALL_NUM;
	minSV = LARGE_NUM;
}

CBinarySeg::~CBinarySeg(void)
{

}

void CBinarySeg::AddClusterPoints(vector<MyPointCloud_RGB_NORMAL> &points)
{
	for(int i=0;i<points.size();i++)
	{
		vecPatchPoint.push_back(points[i]);
	}
	clusterPatchNum.push_back(points.size());
}

void CBinarySeg::AddPatchNormal(vector<Normalt> &normal)
{
	vecPatcNormal = normal;
}

void CBinarySeg::MainStep(bool initFlag,int newAreaNum)
{
	clock_t time[10];
	ofstream outFilet("Output\\Time.txt");
	time[0] = clock();

	if(initFlag)
		InitAreaInterest();
	else
		UpdateAreaInterest(newAreaNum);

	time[1] = clock();

	CollectAreaInterest();
	time[2] = clock();

	ConstructGraph();

	vecvecObjectPool.clear();
	for(int i =0; i <vecPatchPoint.size(); i ++)
	{
		seedPatch = i;
		int flagStop = true;
		paraLargeS = 0.01;
		while(paraLargeS < 0.4 && flagStop)
		{
			vector<int> vecObjectHypo;
			double cutEnergy;
			vecObjectHypo.clear();
			ComputeDataValue();
			GraphCutSolve(vecObjectHypo,cutEnergy);
			if(vecObjectHypo.size() > paraMinPatchInObject && cutEnergy < paraMaxCutEnergy)
			{
				vecvecObjectPool.push_back(vecObjectHypo);
				flagStop = false;
			}	
			paraLargeS += (double)0.01;
		}

		if(flagStop)
		{
			vector<int> vecObjectHypo;
			vecvecObjectPool.push_back(vecObjectHypo);
		}
	}
	time[3] = clock();

	double duration0 = (double)(time[1] - time[0]) / CLOCKS_PER_SEC;
	double duration1 = (double)(time[2] - time[1]) / CLOCKS_PER_SEC;
	double duration2 = (double)(time[3] - time[2]) / CLOCKS_PER_SEC;
	outFilet <<  "step1:" << duration0 <<  endl;
	outFilet <<  "step2:" << duration1 <<  endl;
	outFilet <<  "step3:" << duration2 <<  endl;
	outFilet <<  "  " <<  endl;
	outFilet.close();

	//output
	ofstream outFile1("Output\\ObjectPool.txt");
	for(int i=0;i<vecvecObjectPool.size();i++)
	{
		for(int j=0;j<vecvecObjectPool[i].size();j++)
		{
			outFile1 << vecvecObjectPool[i][j] <<  "  ";
		}
		outFile1 << "  " << endl;
	}
	outFile1.close();
}

void CBinarySeg::InitAreaInterest()
{
	int initIndex = 0;
	for(int i = 0;i < clusterPatchNum.size(); i++)
	{
		clusterPatchInitIndex.push_back(initIndex);
		initIndex += clusterPatchNum[i];
	}

	for(int i = 0;i <clusterPatchInitIndex.size();i++)
	{
		int patchBegin,patchEnd;
		patchBegin = clusterPatchInitIndex[i];
		patchEnd = clusterPatchInitIndex[i] + clusterPatchNum[i];

		vector<MyPointCloud_RGB_NORMAL> patchPoint;
		vector<Normalt> patcNormal;
		for(int j = patchBegin;j < patchEnd;j++)
		{
			patchPoint.push_back(vecPatchPoint[j]);
			patcNormal.push_back(vecPatcNormal[j]);
		}
		CAreaInterest cAreaInterest(patchPoint,patcNormal);
		vecAreaInterest.push_back(cAreaInterest);
	}
}

void CBinarySeg::UpdateAreaInterest(int newAreaNum) //Òª¸Ä
{
	int initIndex = 0;
	clusterPatchInitIndex.clear();
	for(int i = 0;i < clusterPatchNum.size(); i++)
	{
		clusterPatchInitIndex.push_back(initIndex);
		initIndex += clusterPatchNum[i];
	}

	for(int i = clusterPatchInitIndex.size() - newAreaNum;i < clusterPatchInitIndex.size();i++)
	{
		int patchBegin,patchEnd;
		patchBegin = clusterPatchInitIndex[i];
		patchEnd = clusterPatchInitIndex[i] + clusterPatchNum[i];

		vector<MyPointCloud_RGB_NORMAL> patchPoint;
		vector<Normalt> patcNormal;
		for(int j = patchBegin;j < patchEnd;j++)
		{
			patchPoint.push_back(vecPatchPoint[j]);
			patcNormal.push_back(vecPatcNormal[j]);
		}
		CAreaInterest cAreaInterest(patchPoint,patcNormal);
		vecAreaInterest.push_back(cAreaInterest);
	}
}


void CBinarySeg::CollectAreaInterest()
{
	vecPatchPoint.clear();
	vecPatcNormal.clear();
	vecvecPatchMinDis.clear();
	vecvecPatchCenDis.clear();
	vecvecPatchConnectFlag.clear();
	vecSmoothValue.clear();
	vecAppearenceValue.clear();
	vecGeometryValue.clear();
	vecGeometryConvex.clear();
	vecPatchCenPoint.clear();
	vecPatchColor.clear();
	vecpairPatchConnection.clear();
	vecvecPatchColorDetial.clear();
	vecIfConnectTable.clear();
	graphInit.vecEdges.clear();
	graphInit.vecNodes.clear();
	vecvecObjectPool.clear();
	
	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		vecPatchPoint.insert(vecPatchPoint.end(),vecAreaInterest[i].vecPatchPoint.begin(),vecAreaInterest[i].vecPatchPoint.end());
	}

	vecvecPatchMinDis.resize(vecPatchPoint.size());
	vecvecPatchCenDis.resize(vecPatchPoint.size());
	vecvecPatchConnectFlag.resize(vecPatchPoint.size());
	for(int i = 0;i <vecPatchPoint.size();i++)
	{
		vecvecPatchMinDis[i].resize(vecPatchPoint.size(),LARGE_NUM);
		vecvecPatchCenDis[i].resize(vecPatchPoint.size(),LARGE_NUM);
		vecvecPatchConnectFlag[i].resize(vecPatchPoint.size(),false);
	}

	xMin = yMin = zMin = LARGE_NUM;
	xMax = yMax = zMax = SMALL_NUM;
	for(int i = 0;i < vecAreaInterest.size();i++)
	{
		vecPatcNormal.insert(vecPatcNormal.end(),vecAreaInterest[i].vecPatchNormal.begin(),vecAreaInterest[i].vecPatchNormal.end());
		vecSmoothValue.insert(vecSmoothValue.end(),vecAreaInterest[i].vecSmoothValue.begin(),vecAreaInterest[i].vecSmoothValue.end());
		vecAppearenceValue.insert(vecAppearenceValue.end(),vecAreaInterest[i].vecAppearenceValue.begin(),vecAreaInterest[i].vecAppearenceValue.end());
		vecGeometryValue.insert(vecGeometryValue.end(),vecAreaInterest[i].vecGeometryValue.begin(),vecAreaInterest[i].vecGeometryValue.end());
		vecGeometryConvex.insert(vecGeometryConvex.end(),vecAreaInterest[i].vecGeometryConvex.begin(),vecAreaInterest[i].vecGeometryConvex.end());
		vecPatchCenPoint.insert(vecPatchCenPoint.end(),vecAreaInterest[i].vecPatchCenPoint.begin(),vecAreaInterest[i].vecPatchCenPoint.end());
		vecPatchColor.insert(vecPatchColor.end(),vecAreaInterest[i].vecPatchColor.begin(),vecAreaInterest[i].vecPatchColor.end());

		int patchBegin,patchEnd;
		patchBegin = clusterPatchInitIndex[i];
		patchEnd = clusterPatchInitIndex[i] + clusterPatchNum[i];
		AddMatrixIn(vecAreaInterest[i].vecvecPatchMinDis,vecvecPatchMinDis,patchBegin,patchEnd);
		AddMatrixIn(vecAreaInterest[i].vecvecPatchCenDis,vecvecPatchCenDis,patchBegin,patchEnd);
		AddMatrixInBool(vecAreaInterest[i].vecvecPatchConnectFlag,vecvecPatchConnectFlag,patchBegin,patchEnd);
		AddConnectionIn(vecAreaInterest[i].vecpairPatchConnection,vecpairPatchConnection,patchBegin,patchEnd);

		if(xMax < vecAreaInterest[i].xMax) xMax = vecAreaInterest[i].xMax;
		if(yMax < vecAreaInterest[i].yMax) yMax = vecAreaInterest[i].yMax;
		if(zMax < vecAreaInterest[i].zMax) zMax = vecAreaInterest[i].zMax;
		if(xMin > vecAreaInterest[i].xMin) xMin = vecAreaInterest[i].xMin;
		if(yMin > vecAreaInterest[i].yMin) yMin = vecAreaInterest[i].yMin;
		if(zMin > vecAreaInterest[i].zMin) zMin = vecAreaInterest[i].zMin;

	}
	boundingBoxSize = sqrt((xMax-xMin) * (xMax-xMin) + (yMax-yMin) * (yMax-yMin) + (zMax-zMin) * (zMax-zMin));

	NomalizeAppearence();
	NomalizeSmooth();

	//output
//	ofstream outFile0("Output\\basicinfo.txt");
// 	for(int i = 0;i <vecSmoothValue.size();i++)
// 	{
// 		outFile0 << "vecSmoothValue: " << vecSmoothValue[i] << " " << endl;
// 	}
// 	outFile0 << "   " << endl;
// 	for(int i = 0;i <vecAppearenceValue.size();i++)
// 	{
// 		outFile0 << "vecAppearenceValue: " << vecAppearenceValue[i] << " "<< endl ;
// 	}
// 	outFile0 << "   " << endl;
// 
// 	for(int i = 0;i <vecGeometryValue.size();i++)
// 	{
// 		outFile0 << "vecGeometryValue: " << vecGeometryValue[i] << " "<< endl ;
// 	}
// 	outFile0 << "   " << endl;
// 
// 	for(int i = 0;i <vecpairPatchConnection.size();i++)
// 	{
// 		outFile0 << "first: " << vecpairPatchConnection[i].first << "second: " << vecpairPatchConnection[i].second << " " << endl;
// 	}
// 	outFile0 << "   " << endl;
// 	for(int i = 0;i <vecPatchPoint.size();i++)
// 	{
// 		for(int j = 0;j <vecPatchPoint.size();j++)
// 		{
// 			outFile0  << vecvecPatchMinDis[0][j] << " " ;
// 		}
// 		outFile0 << "   " << endl;
// 	}
// 
// 	outFile0.close();
}

void CBinarySeg::AddMatrixIn(vector<vector<double>>  &matrixSmall,vector<vector<double>> &matrixBig,int beginIndex,int endIndex)
{
	for(int i = 0;i < (endIndex - beginIndex);i++)
	{
		for(int j = 0;j < (endIndex - beginIndex);j++)
		{
			matrixBig[i + beginIndex][j + beginIndex] = matrixSmall[i][j];
		}
	}
}

void CBinarySeg::AddMatrixInBool(vector<vector<bool>>  &matrixSmall,vector<vector<bool>> &matrixBig,int beginIndex,int endIndex)
{
	for(int i = 0;i < (endIndex - beginIndex);i++)
	{
		for(int j = 0;j < (endIndex - beginIndex);j++)
		{
			matrixBig[i + beginIndex][j + beginIndex] = matrixSmall[i][j];
		}
	}

}

void CBinarySeg::AddConnectionIn(vector<pair<int,int>> &connectionSmall,vector<pair<int,int>> &connectionBig,int beginIndex,int endIndex)
{
	for(int i = 0;i < connectionSmall.size();i++)
	{
		pair<int,int> connection;
		connection.first = connectionSmall[i].first + beginIndex;
		connection.second = connectionSmall[i].second + beginIndex;
		connectionBig.push_back(connection);
	}
}

void CBinarySeg::ComputeDataValue()
{
	vecDataValue.clear();
	for(int i = 0;i <vecPatchPoint.size();i++)
	{
		vecDataValue.push_back(GetBinaryDataValue(vecvecPatchCenDis[seedPatch][i]));
	}
}

void CBinarySeg::GraphCutSolve(vector<int>& vecObjectHypo, double &cutEnergy)
{
	typedef Graph<double,double,double> GraphType;
	GraphType *g = new GraphType(/*estimated # of nodes*/ vecPatchPoint.size() + 1, 
		/*estimated # of edges*/ vecpairPatchConnection.size() + vecIfConnectTable.size()); 

	g -> add_node(vecPatchPoint.size() + 1); 

	for(int i = 0;i <vecDataValue.size();i++)
	{
		if(i == seedPatch)
			g -> add_tweights(i, LARGE_NUM, 0);
		else if(i != seedPatch)
			g -> add_tweights(i, 0, vecDataValue[i]);
	}
	g -> add_tweights(vecDataValue.size(), 0, LARGE_NUM);			//table data

	for(int i = 0;i <vecSmoothValue.size();i++)
	{
		g -> add_edge(vecpairPatchConnection[i].first, vecpairPatchConnection[i].second, vecSmoothValue[i], vecSmoothValue[i]);
	}
	for(int i = 0;i <vecIfConnectTable.size();i++)
	{
		if(vecIfConnectTable[i])
			g -> add_edge(i, vecDataValue.size(), 0, 0);			//table smooth
	}

	m_flow = g -> maxflow();
	cutEnergy = m_flow;

	int countSOURCE,countSINK;
	countSINK = countSOURCE =0;

	vecFore.clear();
	vecBack.clear();

	for(int i=0;i<vecPatchPoint.size();i++)
		if (g->what_segment(i) == GraphType::SOURCE)
		{
			countSOURCE++;
			vecFore.push_back(i);
			vecObjectHypo.push_back(i);
		}
		else
		{
			countSINK++;
			vecBack.push_back(i);
		}

		delete g; 

//		printf("foreground: %d ,background: %d\n",countSOURCE,countSINK);
}

double CBinarySeg::GetBinaryDataValue(double d)
{
	if(d == LARGE_NUM || d == 0)  
	{
		return LARGE_NUM;
	}
	else
	{
		double penaltyValue, panaltySmallValue, penaltyLargeValue;
		penaltyValue = 0;

		panaltySmallValue =  paraSmallK * (d - 0.813342* paraSmallS);
		if(panaltySmallValue < 0)
			panaltySmallValue =  0;

		penaltyLargeValue =  paraLargeK * (d - 0.813342* paraLargeS);
		if(penaltyLargeValue < 0)
			penaltyLargeValue =  0;

		penaltyValue = panaltySmallValue + penaltyLargeValue;
		return penaltyValue;
	}

}

void CBinarySeg::AddTable(MyPointCloud_RGB_NORMAL &table)
{
	tablePoint = table;
 	tableCen.x = tableCen.y = tableCen.z = 0;
	
	//useless
// 	for(int i=0;i<tablePoint->size();i++)
// 	{
// 		tableCen.x += tablePoint->at(i).x;
// 		tableCen.y += tablePoint->at(i).y;
// 		tableCen.z += tablePoint->at(i).z;
// 	}
// 	tableCen.x /= tablePoint->size();
// 	tableCen.y /= tablePoint->size();
// 	tableCen.z /= tablePoint->size();
}


bool CBinarySeg::IfConnectTable(vector<MyPt_RGB_NORMAL> points)
{
	//jerrysyf
	// 	for(int i=0;i<points.size();i++)
	// 	{
	// 		for(int j=0;j<tablePoint->size();j++)
	// 		{
	// 			double dis = sqrt((points[i].x - tablePoint->at(j).x) * (points[i].x - tablePoint->at(j).x) +
	// 							  (points[i].y - tablePoint->at(j).y) * (points[i].y - tablePoint->at(j).y) +
	// 							  (points[i].z - tablePoint->at(j).z) * (points[i].z - tablePoint->at(j).z));
	// 			if(dis < thresholdClose0)
	// 			{
	// 				return true;
	// 			}
	// 		}
	// 	}
	// 	return false;

	return true;
}

void CBinarySeg::NomalizeAppearence()
{
	double maxAV = SMALL_NUM;
	for(int i = 0;i <vecpairPatchConnection.size();i++)
	{
		if(maxAV < vecAppearenceValue[i])
			maxAV = vecAppearenceValue[i];
	}
	for(int i = 0;i <vecpairPatchConnection.size();i++)
	{
		vecAppearenceValue[i] = vecAppearenceValue[i] / maxAV;
		vecAppearenceValue[i] = 1 - vecAppearenceValue[i];
	}

	for(int i = 0;i <vecpairPatchConnection.size();i++)
	{
		vecGeometryValue[i] = paraGeometry * vecGeometryValue[i];
		vecAppearenceValue[i] = paraAppearence * vecAppearenceValue[i];
		vecSmoothValue[i] = vecGeometryValue[i] + vecAppearenceValue[i];
	}
}

void CBinarySeg::NomalizeSmooth()
{
	double para = 0.3;
	for(int i = 0;i <vecSmoothValue.size();i++)
	{
		if(maxSV < vecSmoothValue[i])
			maxSV = vecSmoothValue[i];
		if(minSV > vecSmoothValue[i])
			minSV = vecSmoothValue[i];
	}
	for(int i = 0;i <vecSmoothValue.size();i++)
	{
		vecSmoothValue[i] =  (vecSmoothValue[i] - minSV)/(maxSV - minSV);
		vecSmoothValue[i] = paraSmoothAdjust * pow(2.7183,- (1 - vecSmoothValue[i]) * (1 - vecSmoothValue[i]) / para /para);
	}
}

void CBinarySeg::ConstructGraph()
{
	graphInit.vecNodes.clear();
	graphInit.vecEdges.clear();

	MyPt_RGB_NORMAL point;
	for(int i = 0;i < vecPatchCenPoint.size();i++)
	{
		point.x = vecPatchCenPoint[i].x;
		point.y = vecPatchCenPoint[i].y;
		point.z = vecPatchCenPoint[i].z;
		graphInit.vecNodes.push_back(point);
	}

	graphInit.vecEdges = vecpairPatchConnection;
}