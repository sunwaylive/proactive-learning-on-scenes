#include "GraphCutBasicStruct.h"

extern double maxSV,minSV;
int superInt = 0;
MeshFace meshFaceExtra;
MeshVertex meshVertexExtra;

CAreaInterest::CAreaInterest(vector<MyPointCloud_RGB_NORMAL> &pointCloud, vector<Normalt> &patchNomal)
{
	validFlag = true;
	mergeFlag = false;
	vecPatchPoint = pointCloud;
	patchNum = vecPatchPoint.size();
	vecPatchNormal = patchNomal;

	//preprocess
	thresholdClose0 = 0.005;
	paraConvexK = 0.1;
	paraConvexT = 0.9;
	paraConcave = 1.0;
	paraGeometry = 0.4;
	paraAppearence = 0.3;
	thresholdClose0 = 0.005; 

	//binaryseg
	paraSmallK = 1;
	paraSmallS = 0.02;
	paraLargeK = 20;
	paraLargeS = 0.1;
	paraMinPatchInObject = 3;
	paraMaxCutEnergy = 18.328;
	paraSmoothAdjust = 0.7;

	//multiseg
	paraH = 0.9;

	//scanestimation
	paraConfidence = 2;

	maxSV = SMALL_NUM;
	minSV = LARGE_NUM;

	Preprocess();
	ComputeSmoothValue();
	ConstructPatchGraph();

}

CAreaInterest::~CAreaInterest(void)
{
}

void CAreaInterest::MainStep()
{
	ofstream outFile("Output\\MainStep.txt",ios::app);

	outFile <<  "superInt: " << superInt <<endl;
	BinarySeg();
	outFile <<  "1" <<endl;
 	Clustering();
 	outFile <<  "2" <<endl;
 	MultiSeg();
 	outFile <<  "3" <<endl;

	outFile.close();
}

void CAreaInterest::Preprocess()
{
	//Bounding box, Color average, Center position
	xMin = yMin = zMin = LARGE_NUM;
	xMax = yMax = zMax = SMALL_NUM;
	for(int i = 0;i <vecPatchPoint.size();i++)
	{
		ColorType color;
		MyPoint point;
		vector<int> vecColorDetial;
		vecColorDetial.resize(21);
		for(int j = 0;j < vecColorDetial.size();j++)
		{
			vecColorDetial[j] = 0;
		}

		color.mRed = color.mGreen = color.mBlue = 0;
		point.x = point.y = point.z =0;

		for(int j = 0;j < vecPatchPoint[i].mypoints.size();j++)
		{
			color.mRed += vecPatchPoint[i].mypoints[j].r;
			color.mGreen += vecPatchPoint[i].mypoints[j].g;
			color.mBlue += vecPatchPoint[i].mypoints[j].b;

			vecColorDetial[(int)(vecPatchPoint[i].mypoints[j].r / 40) ] += 1;
			vecColorDetial[(int)(vecPatchPoint[i].mypoints[j].g / 40)  + 7] += 1;
			vecColorDetial[(int)(vecPatchPoint[i].mypoints[j].b / 40)  + 14] += 1;

			point.x += vecPatchPoint[i].mypoints[j].x;
			point.y += vecPatchPoint[i].mypoints[j].y;
			point.z += vecPatchPoint[i].mypoints[j].z;
			if(xMax < vecPatchPoint[i].mypoints[j].x) xMax = vecPatchPoint[i].mypoints[j].x;
			if(yMax < vecPatchPoint[i].mypoints[j].y) yMax = vecPatchPoint[i].mypoints[j].y;
			if(zMax < vecPatchPoint[i].mypoints[j].z) zMax = vecPatchPoint[i].mypoints[j].z;
			if(xMin > vecPatchPoint[i].mypoints[j].x) xMin = vecPatchPoint[i].mypoints[j].x;
			if(yMin > vecPatchPoint[i].mypoints[j].y) yMin = vecPatchPoint[i].mypoints[j].y;
			if(zMin > vecPatchPoint[i].mypoints[j].z) zMin = vecPatchPoint[i].mypoints[j].z;
		}
		if(vecPatchPoint[i].mypoints.size() > 0)
		{
			color.mRed /= vecPatchPoint[i].mypoints.size();
			color.mGreen /= vecPatchPoint[i].mypoints.size();
			color.mBlue /= vecPatchPoint[i].mypoints.size();
			point.x/= vecPatchPoint[i].mypoints.size();
			point.y /= vecPatchPoint[i].mypoints.size();
			point.z /= vecPatchPoint[i].mypoints.size();
		}

		vecPatchColor.push_back(color);
		vecvecPatchColorDetial.push_back(vecColorDetial);
		vecPatchCenPoint.push_back(point);
	}
	boundingBoxSize = sqrt((xMax-xMin) * (xMax-xMin) + (yMax-yMin) * (yMax-yMin) + (zMax-zMin) * (zMax-zMin));



	//Distance between two patches
	vecvecPatchMinDis.resize(vecPatchPoint.size());
	vecvecPatchCenDis.resize(vecPatchPoint.size());
	vecvecPatchConnectFlag.resize(vecPatchPoint.size());
	for(int i = 0;i <vecPatchPoint.size();i++)
	{
		vecvecPatchMinDis[i].resize(vecPatchPoint.size(),LARGE_NUM);
		vecvecPatchCenDis[i].resize(vecPatchPoint.size(),LARGE_NUM);
		vecvecPatchConnectFlag[i].resize(vecPatchPoint.size(),false);
	}

	pair<int,int> pairPatchConnection;
	for(int i = 0;i < patchNum;i++)
	{
		for(int j = 0;j < patchNum;j++)
		{
			if(i < j)
			{
				bool  stable;   // if connect stable
				vecvecPatchMinDis[i][j] = vecvecPatchMinDis[j][i] = GetMinDisBetPatch(i,j,stable);
				vecvecPatchCenDis[i][j] = vecvecPatchCenDis[j][i] = GetCenDisBetPatch(i,j);

				if(vecvecPatchMinDis[i][j] < thresholdClose0 && stable)
				{
					pairPatchConnection.first = i;
					pairPatchConnection.second = j;
					vecpairPatchConnection.push_back(pairPatchConnection);

					pairPatchConnection.first = j;
					pairPatchConnection.second = i;
					vecpairPatchConnection.push_back(pairPatchConnection);

					vecvecPatchConnectFlag[i][j] = vecvecPatchConnectFlag[j][i] = true;
				} 
			}
		}
	}

}

void CAreaInterest::ComputeSmoothValue()
{
	for(int i = 0;i <vecpairPatchConnection.size();i++)
	{
		vecSmoothValue.push_back(GetBinarySmoothValue(vecpairPatchConnection[i].first,vecpairPatchConnection[i].second));
	}
}

double CAreaInterest::GetMinDisBetPatch(int m,int n,bool &stable)
{	
	double minDis = LARGE_NUM;
	vector<bool> vecPointClose0(vecPatchPoint[m].mypoints.size(),false);
	vector<bool> vecPointClose1(vecPatchPoint[n].mypoints.size(),false);

	for(int i = 0;i < vecPatchPoint[m].mypoints.size();i++)
	{
		for(int j = 0;j < vecPatchPoint[n].mypoints.size();j++)
		{
			double dis = sqrt((vecPatchPoint[m].mypoints[i].x-vecPatchPoint[n].mypoints[j].x) * (vecPatchPoint[m].mypoints[i].x-vecPatchPoint[n].mypoints[j].x)
				+ (vecPatchPoint[m].mypoints[i].y-vecPatchPoint[n].mypoints[j].y) * (vecPatchPoint[m].mypoints[i].y-vecPatchPoint[n].mypoints[j].y)
				+ (vecPatchPoint[m].mypoints[i].z-vecPatchPoint[n].mypoints[j].z) * (vecPatchPoint[m].mypoints[i].z-vecPatchPoint[n].mypoints[j].z));
			if(minDis > dis)
			{
				minDis = dis;
			}

			if(dis < thresholdClose0 * 3)
			{
				vecPointClose0[i] = true;
				vecPointClose1[j] = true;
			}
		}
	}

	int count0=0;
	int count1=0;
	for(int i = 0; i < vecPointClose0.size();i++)
	{
		if(vecPointClose0[i] == true)
			count0++;
	}
	for(int i = 0; i < vecPointClose1.size();i++)
	{
		if(vecPointClose1[i] == true)
			count1++;
	}
	
	if(count0 > 5 && count1 >5)
		stable = true;
	else
		stable = false;

	return minDis;
}

double CAreaInterest::GetCenDisBetPatch(int m,int n)
{
	double dis =  sqrt((vecPatchCenPoint[m].x-vecPatchCenPoint[n].x) * (vecPatchCenPoint[m].x-vecPatchCenPoint[n].x)
		+ (vecPatchCenPoint[m].y-vecPatchCenPoint[n].y) * (vecPatchCenPoint[m].y-vecPatchCenPoint[n].y)
		+ (vecPatchCenPoint[m].z-vecPatchCenPoint[n].z) * (vecPatchCenPoint[m].z-vecPatchCenPoint[n].z));
	return dis;
}

double CAreaInterest::GetBinarySmoothValue(int m,int n)
{
	double smoothValue,geometryValue,appearenceValue;

	//normal
	MyPoint cenM,cenN;
	Normalt norM,norN,norMN,norNM;
	double nomalizeValue;

	cenM = vecPatchCenPoint[m];
	cenN = vecPatchCenPoint[n];
	norM = vecPatchNormal[m];
	norN = vecPatchNormal[n];

	//normalMN
	norMN.normal_x = cenN.x - cenM.x;
	norMN.normal_y = cenN.y - cenM.y;
	norMN.normal_z = cenN.z - cenM.z;
	nomalizeValue = sqrt(norMN.normal_x * norMN.normal_x + norMN.normal_y * norMN.normal_y + norMN.normal_z * norMN.normal_z);
	norMN.normal_x /= nomalizeValue;
	norMN.normal_y /= nomalizeValue;
	norMN.normal_z /= nomalizeValue;

	////normalNM
	norNM.normal_x = cenM.x - cenN.x;
	norNM.normal_y = cenM.y - cenN.y;
	norNM.normal_z = cenM.z - cenN.z;
	nomalizeValue = sqrt(norNM.normal_x * norNM.normal_x + norNM.normal_y * norNM.normal_y + norNM.normal_z * norNM.normal_z);
	norNM.normal_x /= nomalizeValue;
	norNM.normal_y /= nomalizeValue;
	norNM.normal_z /= nomalizeValue;

	//method1
	double xTemp0,yTemp0,zTemp0;
	double xTemp1,yTemp1,zTemp1;

	double convexValue0,convexValue1;   //convex if > 0
	CrossProduct(norN.normal_x,norN.normal_y,norN.normal_z, norM.normal_x,norM.normal_y,norM.normal_z, xTemp0,yTemp0,zTemp0);	
	CrossProduct(xTemp0,yTemp0,zTemp0, norMN.normal_x,norMN.normal_y,norMN.normal_z, xTemp1,yTemp1,zTemp1);
	convexValue0 = norM.normal_x * xTemp1 +  norM.normal_y * yTemp1 + norM.normal_z * zTemp1;
	CrossProduct(norM.normal_x,norM.normal_y,norM.normal_z, norN.normal_x,norN.normal_y,norN.normal_z, xTemp0,yTemp0,zTemp0);
	CrossProduct(xTemp0,yTemp0,zTemp0, norNM.normal_x,norNM.normal_y,norNM.normal_z, xTemp1,yTemp1,zTemp1);
	convexValue1 = norN.normal_x * xTemp1 +  norN.normal_y * yTemp1 + norN.normal_z * zTemp1;

	bool convexFlag;
	bool errorFlag = false;

	if(convexValue0 * convexValue1 > 0)
	{
		if(convexValue0 >= 0)	
			convexFlag = true;
		else	
			convexFlag = false;

		double cosValue;
		cosValue = (norM.normal_x * norN.normal_x + norM.normal_y * norN.normal_y + norM.normal_z * norN.normal_z);
		if(convexFlag)	
			geometryValue = paraConvexK * cosValue + paraConvexT;
		else	
			geometryValue = paraConcave * cosValue;

		if(geometryValue < 0)
			geometryValue = 0;
	}

	else if(convexValue0 * convexValue1 < 0)
	{
		//method2
		convexValue0 = norMN.normal_x * norN.normal_x +  norMN.normal_y * norN.normal_y + norMN.normal_z * norN.normal_z;
		convexValue1 = norNM.normal_x * norM.normal_x +  norNM.normal_y * norM.normal_y + norNM.normal_z * norM.normal_z;

		if(convexValue0 >= 0)	
			convexFlag = true;
		else	
			convexFlag = false;

		double cosValue;
		cosValue = (norM.normal_x * norN.normal_x + norM.normal_y * norN.normal_y + norM.normal_z * norN.normal_z);
		if(convexFlag)	
			geometryValue = paraConvexK * cosValue + paraConvexT;
		else	
			geometryValue = paraConcave * cosValue;

		if(geometryValue < 0)
			geometryValue = 0;

		if(convexValue0 * convexValue1 < 0)
			errorFlag = true;
	}

	if(errorFlag)
		geometryValue = 0.9;


	//color
	appearenceValue = 0;
	for(int i = 0;i < 21;i++)
	{
		double Mi,Ni;
		Mi = vecvecPatchColorDetial[m][i];
		Ni = vecvecPatchColorDetial[n][i];

		if(Mi != 0 || Ni != 0)
			appearenceValue += (Mi - Ni) * (Mi - Ni) / (Mi + Ni);
	}

	smoothValue = paraGeometry * geometryValue + paraAppearence * appearenceValue;

	vecGeometryValue.push_back(geometryValue);
	vecAppearenceValue.push_back(appearenceValue);
	vecGeometryConvex.push_back(convexFlag);

	return smoothValue;
}

void CAreaInterest::CrossProduct(double ax,double ay,double az,double bx,double by,double bz,double &rx,double &ry,double &rz)
{
	rx = ay*bz - az*by;
	ry = az*bx - ax*bz;
	rz = ax*by - ay*bx;
}

void CAreaInterest::Merge()
{
	mergeFlag = true;
}

void CAreaInterest::Invalidt()
{
	validFlag = false;
}

void CAreaInterest::BinarySeg()
{
	ofstream outFile("Output\\BinarySeg.txt",ios::app);
	outFile << " superInt: "  << superInt <<endl;
	outFile.close();

	vecvecObjectPool.clear();
	for(int i =0; i <vecPatchPoint.size(); i ++)
	{
//		outFile <<  i <<endl;
		seedPatch = i;
		int flagStop = true;
		paraLargeS = 0.01;
		while(paraLargeS < 1 && flagStop)
		{
//			outFile <<  "0.4" <<endl;
			vector<int> vecObjectHypo;
			double cutEnergy;
			vecObjectHypo.clear();
			ComputeDataValue();
//			outFile <<  "0.5" <<endl;

			SolveBinaryGraphCut(vecObjectHypo,cutEnergy);
//			outFile <<  "0.6" <<endl;

			if(vecObjectHypo.size() > paraMinPatchInObject /*&& cutEnergy < paraMaxCutEnergy*/)
			{
				vecvecObjectPool.push_back(vecObjectHypo);
				flagStop = false;
			}	
//			outFile <<  "0.7" <<endl;
			paraLargeS += (double)0.01;
		}

		if(flagStop)
		{
			vector<int> vecObjectHypo;
			vecvecObjectPool.push_back(vecObjectHypo);
		}
	}

//	outFile.close();

	

	//output
	ofstream outFile1("Output\\ObjectPool.txt",ios::app);
	outFile1 << " superInt: "  << superInt <<endl;
	for(int i=0;i<vecvecObjectPool.size();i++)
	{
		for(int j=0;j<vecvecObjectPool[i].size();j++)
		{
			outFile1 << vecvecObjectPool[i][j] <<  "  ";
		}
		outFile1 << "  " << endl;
	}
	outFile1.close();

	superInt++;
}

void CAreaInterest::ConstructPatchGraph()
{
	ofstream outFile("Output\\ConstructPatchGraph.txt");
	outFile <<  "1" <<endl;

	MyPt_RGB_NORMAL point;
	for(int i = 0;i < vecPatchCenPoint.size();i++)
	{
		point.x = vecPatchCenPoint[i].x;
		point.y = vecPatchCenPoint[i].y;
		point.z = vecPatchCenPoint[i].z;
		point.r = 0.9;
		point.g = 0.0;
		point.b = 0.0;
		graphInit.vecNodes.push_back(point);
		outFile <<  i <<endl;
	}

	outFile <<  "///////////////" <<endl;
	pair<MyPt_RGB_NORMAL,MyPt_RGB_NORMAL> edge;
	MyPt_RGB_NORMAL point0,point1;
	for(int i = 0;i < vecpairPatchConnection.size();i++)
	{
		int index0,index1;
		index0 =vecpairPatchConnection[i].first;
		index1 =vecpairPatchConnection[i].second;
		edge.first = graphInit.vecNodes[index0];
		edge.second = graphInit.vecNodes[index1];
		graphInit.vecEdges.push_back(edge);
		outFile <<  i <<endl;
	} 
}

void CAreaInterest::ComputeDataValue()
{
	vecDataValue.clear();
	for(int i = 0;i <vecPatchPoint.size();i++)
	{
		vecDataValue.push_back(GetBinaryDataValue(vecvecPatchCenDis[seedPatch][i]));
	}
}

double CAreaInterest::GetBinaryDataValue(double d)
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

void CAreaInterest::SolveBinaryGraphCut(vector<int>& vecObjectHypo, double &cutEnergy)
{
	typedef Graph<double,double,double> GraphType;
	GraphType *g = new GraphType(vecPatchPoint.size(), vecpairPatchConnection.size()); 

	g -> add_node(vecPatchPoint.size() ); 

	for(int i = 0;i <vecDataValue.size();i++)
	{
		if(i == seedPatch)
			g -> add_tweights(i, LARGE_NUM, 0);
		else if(i != seedPatch)
			g -> add_tweights(i, 0, vecDataValue[i]);
	}

	for(int i = 0;i <vecSmoothValue.size();i++)
	{
		g -> add_edge(vecpairPatchConnection[i].first, vecpairPatchConnection[i].second, vecSmoothValue[i], vecSmoothValue[i]);
	}

	cutEnergy = g -> maxflow();

	int countSOURCE,countSINK;
	countSINK = countSOURCE =0;

	for(int i=0;i<vecPatchPoint.size();i++)
		if (g->what_segment(i) == GraphType::SOURCE)
		{
			countSOURCE++;
			vecObjectHypo.push_back(i);
		}
		else
		{
			countSINK++;
		}
	
	delete g; 
}

void CAreaInterest::Clustering()
{
	CleanObjectPool();
	MeanShift();
}

void CAreaInterest::CleanObjectPool()
{
	vector<vector<int>> vecvecObjectPoolClean;
	for(int i =0;i < vecvecObjectPool.size();i++)
	{
		if(vecvecObjectPool[i].size() != 0)
		{
			vecvecObjectPoolClean.push_back(vecvecObjectPool[i]);
		}
	}
	vecvecObjectPool = vecvecObjectPoolClean;
}

void CAreaInterest::MeanShift()
{
	for(int i = 0;i < vecvecObjectPool.size();i++)
	{
		int initObject = i;

		vector<int> vecInArea;
		vector<int> vecObjectPool,vecObjectPoolOld;
		vecObjectPool = vecvecObjectPool[initObject];
		do
		{
			vecObjectPoolOld = vecObjectPool;
			vecInArea.clear();
			vecObjectPool.clear();
			GetArea(vecObjectPoolOld,vecInArea);
			GetMeanShiftSVector(vecInArea,vecObjectPool);

		}
		while(vecObjectPool != vecObjectPoolOld);
		vecvecObjectPoolClustering.push_back(vecObjectPool);
	}

	vector<vector<int>> vecvecObjectPoolClusteringUnique;
	for(int i =0;i < vecvecObjectPoolClustering.size();i++)
	{
		bool flagExist = false;
		for(int j = 0;j < vecvecObjectPoolClusteringUnique.size();j++)
		{
			if(vecvecObjectPoolClustering[i] == vecvecObjectPoolClusteringUnique[j])
			{
				vecObjectPoolClusteringCount[j]++;
				flagExist = true;
				break;
			}
		}
		if(!flagExist)
		{
			vecvecObjectPoolClusteringUnique.push_back(vecvecObjectPoolClustering[i]);
			vecObjectPoolClusteringCount.push_back(1);
		}
	}
	vecvecObjectPoolClustering = vecvecObjectPoolClusteringUnique;

	ofstream outFile1("Output\\ObjectPoolClustering.txt",ios::app);
	outFile1 << " superInt: "  << superInt <<endl;
	for(int i=0;i<vecvecObjectPoolClustering.size();i++)
	{
		for(int j=0;j<vecvecObjectPoolClustering[i].size();j++)
		{
			outFile1 << vecvecObjectPoolClustering[i][j] <<  "  ";
		}
		outFile1 << "  " << endl;
	}
	outFile1.close();
}

void CAreaInterest::GetArea(vector<int> vecObjectPool,vector<int> &vecInArea)
{
	vector<double> vecJaccardIndex;
	for(int i = 0;i < vecvecObjectPool.size();i++)
	{
		double jaccardIndex = GetJaccardIndex(vecObjectPool, i);
		if(jaccardIndex > paraH)
			vecInArea.push_back(i);
	}
}

double CAreaInterest::GetJaccardIndex(vector<int> vecObjectPool, int n)
{
	vector<int> SamePatch, DiffPatch;
	for(int i = 0; i < vecObjectPool.size();i++)
	{
		bool flagSame = false;
		for(int j = 0;j < vecvecObjectPool[n].size();j++)
		{
			if(vecObjectPool[i] == vecvecObjectPool[n][j])
			{
				SamePatch.push_back(vecObjectPool[i]);
				flagSame = true;
				break;
			}	
		}
		if(!flagSame)
			DiffPatch.push_back(vecObjectPool[i]);
	}

	for(int j = 0;j < vecvecObjectPool[n].size();j++)
	{
		bool flagSame = false;
		for(int i = 0; i < vecObjectPool.size();i++)
		{
			if(vecObjectPool[i] == vecvecObjectPool[n][j])
			{
				flagSame = true;
				break;
			}	
		}
		if(!flagSame)
			DiffPatch.push_back(vecvecObjectPool[n][j]);
	}


	int samePointNum, diffPointNum;
	double jaccardIndex;
	samePointNum = diffPointNum =0;
	for(int i = 0; i < SamePatch.size();i++)
	{
		samePointNum += vecPatchPoint[SamePatch[i]].mypoints.size();
	}
	for(int i = 0; i < DiffPatch.size();i++)
	{
		diffPointNum += vecPatchPoint[DiffPatch[i]].mypoints.size();
	}
	jaccardIndex = double(samePointNum) / (samePointNum + diffPointNum);

	return jaccardIndex;
}

void CAreaInterest::GetMeanShiftSVector(vector<int> vecInArea, vector<int> &objectPool)
{
	vector<int> vecPatch;
	vector<double> vecCount;
	for(int i = 0;i < vecInArea.size();i++)
	{
		for(int j = 0;j<vecvecObjectPool[vecInArea[i]].size();j++)
		{
			vector<int>::iterator vecPatchIt;
			vecPatchIt = find(vecPatch.begin(),vecPatch.end(),vecvecObjectPool[vecInArea[i]][j]);
			if(vecPatchIt == vecPatch.end())
			{
				vecPatch.push_back(vecvecObjectPool[vecInArea[i]][j]);
				vecCount.push_back(1);
			}
			else
			{
				for(int k = 0;k <vecPatch.size();k++)
				{
					if(vecPatch[k] == *vecPatchIt)
						vecCount[k]++;
				}
			}	
		}
	}

	for(int i = 0;i < vecCount.size();i++)
	{
		vecCount[i] /= vecInArea.size();
		if(vecCount[i] > 0.5)
			objectPool.push_back(vecPatch[i]);
	}
}

void CAreaInterest::MultiSeg()
{
	SolveMultiGraphCut();
	ComputeHypo();
	ConstructContractionGraph();
}

double CAreaInterest::GetMultiDataValue(int SiteID,int LableID)
{
	ofstream outFile1("Output\\GetMultiDataValue.txt",ios::app);
	outFile1 << " superInt: "  << superInt <<endl;
	outFile1.close();

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


	double dataValue;
	dataValue = objectCount * 3;

	return dataValue; 
}

void CAreaInterest::SolveMultiGraphCut()
{	
	ofstream outFile1("Output\\SolveMultiGraphCut1.txt",ios::app);
	outFile1 << " superInt: "  << superInt <<endl;
	
	int num_sites, num_labels;
	num_sites = vecPatchPoint.size();
	num_labels = vecvecObjectPoolClustering.size();

	if(num_labels == 1 )
	{
		vecvecMultiResult.resize(1);
		for(int i = 0;i < num_sites;i++)
		{
			vecvecMultiResult[0].push_back(i);
		}
		return;
	}
	else if(num_labels == 0)
	{
		vecvecMultiResult.resize(1);
		for(int i = 0;i < num_sites;i++)
		{
			vecvecMultiResult[0].push_back(i);
		}
		return;
	}

	outFile1 << " num_sites: "  << num_sites <<endl;
	outFile1 << " num_labels: "  << num_labels <<endl;

	GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(num_sites, num_labels);

	for(int i = 0;i < num_labels;i++)
	{
		for(int j = 0;j < num_labels;j++)
		{
			gc->setSmoothCost(i,j,1);
		}
	}

	outFile1.close();

	//smooth
	for(int i = 0;i < vecpairPatchConnection.size();i++)
	{
		gc->setNeighbors(vecpairPatchConnection[i].first,vecpairPatchConnection[i].second,vecSmoothValue[i]);
	}


	//data
	for(int i = 0;i < num_sites;i++)
	{
		for(int j = 0;j < num_labels;j++)
		{
			double dataValue = GetMultiDataValue(i,j);
			if(dataValue < 0)
				dataValue = 0;
			gc->setDataCost(i,j,dataValue);
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

	vector<vector<int>> vecvecMultiResultClean;
	for(int i =0;i < vecvecMultiResult.size();i++)
	{
		if(vecvecMultiResult[i].size() > 0)
		{
			vecvecMultiResultClean.push_back(vecvecMultiResult[i]);
		}
	}
	vecvecMultiResult = vecvecMultiResultClean;


	ofstream outFile("Output\\MultiResult.txt",ios::app);
	outFile << " superInt: "  << superInt <<endl;
	for(int i=0;i<vecvecMultiResult.size();i++)
	{
		for(int j=0;j<vecvecMultiResult[i].size();j++)
		{
			outFile << vecvecMultiResult[i][j] <<  "  ";
		}
		outFile << "  " << endl;
	}
	outFile.close();
}

void CAreaInterest::ComputeHypo()
{
	ofstream outFileg("Output\\ComputeHypo.txt",ios::app);
	outFileg << "let's begin :) " << endl;

	for(int i = 0;i < vecvecMultiResult.size();i++)
	{
		ComputeObjectness(i);
		outFileg << "objectness:" << i << endl;
	}

	for(int i = 0;i < vecvecMultiResult.size();i++)
	{
		for(int j = 0;j < vecvecMultiResult.size();j++)
		{
			if(i >= j)	continue;
			ComputeSeparateness(i,j);
			outFileg << "sepateness:" << i  << "   " << j << endl;
		}
	}
	outFileg.close();
}

void CAreaInterest::ComputeObjectness(int m)
{
	ObjectHypo objectHypo;
	objectHypo.patchIndex = vecvecMultiResult[m];
	objectHypo.objectness = 0;
	objectHypo.areaIndex = 0;
	objectHypo.mergeFlag = false;
	vecObjectHypo.push_back(objectHypo);
}

void CAreaInterest::ComputeSeparateness(int m,int n)
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
		EdgeHypo edgeHypo;
		edgeHypo.begin = m;
		edgeHypo.end = n;
		edgeHypo.areaIndex = 0;
		edgeHypo.pairPatch = vecpairSeperatenessSmallEdge;
		edgeHypo.separateness = 0;
		vecEdgeHypo.push_back(edgeHypo);
	}
}

void CAreaInterest::ConstructContractionGraph()
{
	for(int i = 0;i < vecObjectHypo.size();i++)
	{
		MyPt_RGB_NORMAL point;
		point.x = point.y = point.z = 0;
		for(int j = 0;j < vecObjectHypo[i].patchIndex.size();j++)
		{
			point.x += vecPatchCenPoint[vecObjectHypo[i].patchIndex[j]].x;
			point.y += vecPatchCenPoint[vecObjectHypo[i].patchIndex[j]].y;
			point.z += vecPatchCenPoint[vecObjectHypo[i].patchIndex[j]].z;
		}
		point.x /= vecObjectHypo[i].patchIndex.size();
		point.y /= vecObjectHypo[i].patchIndex.size();
		point.z /= vecObjectHypo[i].patchIndex.size();
		point.r = 0.2;
		point.g = 0.2;
		point.b = 1.0;

		vecObjectHypo[i].cenPoint.x = point.x;
		vecObjectHypo[i].cenPoint.y = point.y;
		vecObjectHypo[i].cenPoint.z = point.z;

		graphContract.vecNodes.push_back(point);
	}

	pair<MyPt_RGB_NORMAL,MyPt_RGB_NORMAL> edge;
	MyPt_RGB_NORMAL point0,point1;
	for(int i = 0;i < vecEdgeHypo.size();i++)
	{
		int index0,index1;
		index0 = vecEdgeHypo[i].begin;
		index1 = vecEdgeHypo[i].end;
		edge.first = graphContract.vecNodes[index0];
		edge.second = graphContract.vecNodes[index1];
		graphContract.vecEdges.push_back(edge);
	} 
}

void CAreaInterest::SaveObjectHypoToOriginal(CMesh *original,int m)
{
	for(int i = 0;i < vecObjectHypo[m].patchIndex.size();i++)
	{
		int patchIndex = vecObjectHypo[m].patchIndex[i];
		for(int j = 0;j < vecPatchPoint[patchIndex].mypoints.size();j++)
		{
			MyPoint_RGB_NORMAL point =  vecPatchPoint[patchIndex].mypoints[j];
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
}

void CAreaInterest::ScoreUpdate()
{
	ofstream outFileg("Output\\ScoreUpdate.txt");
	outFileg << "let's begin :) " << endl;

// 	//get confidence score for each patch
// 	vecPatchConfidenceScore.resize(vecPatchPoint.size(),0);
// 	for(int i = 0;i < vecvecMultiResult.size();i++)
// 	{
// 		for(int j = 0;j < vecvecMultiResult[i].size();j++)
// 		{
// 			int patchIndex = vecvecMultiResult[i][j];
// 			vecPatchConfidenceScore[patchIndex] = ComputePatchConfidenceScore(i,patchIndex);
// 		}
// 	}
// 
// 	//update the smooth term
// 	for(int i = 0;i < vecpairPatchConnection.size();i++)
// 	{
// 		double confidenceScore;
// 		confidenceScore = vecPatchConfidenceScore[vecpairPatchConnection[i].first] + vecPatchConfidenceScore[vecpairPatchConnection[i].second];
// 
// 		if(vecGeometryConvex[i])
// 			vecGeometryValue[i] *= confidenceScore;
// 		else
// 			vecGeometryValue[i] /= confidenceScore;
// 
// 		double valueBefore = vecSmoothValue[i];
// 		vecSmoothValue[i] = vecGeometryValue[i] + vecAppearenceValue[i];
// 
// 		double para = 0.3;
// 		vecSmoothValue[i] =  (vecSmoothValue[i] - minSV)/(maxSV - minSV);
// 		vecSmoothValue[i] = paraSmoothAdjust * pow(2.7183,- (1 - vecSmoothValue[i]) * (1 - vecSmoothValue[i]) / para /para);
// 	}

	outFileg << "let's begin 2:) " << endl;
	ComputeScore();
	outFileg << "let's begin 3:) " << endl;
	UpdateGraph();
	outFileg << "let's begin 4:) " << endl;

	outFileg << "Show results finished :)   " << endl;

	outFileg.close();
}

double CAreaInterest::ComputePatchConfidenceScore(int objectIndex,int patchIndex)
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

double CAreaInterest::ComputePointConfidenceScore(int objectIndex, MyPoint_RGB_NORMAL point)
{
	double confidenceScore = 0;
	for(int k = 0;k < vecObjectHypo[objectIndex].isoPoints.vecPoints.size();k++)
	{
		double distance;
		distance = sqrt((vecObjectHypo[objectIndex].isoPoints.vecPoints[k].x - point.x) * (vecObjectHypo[objectIndex].isoPoints.vecPoints[k].x - point.x)
					  + (vecObjectHypo[objectIndex].isoPoints.vecPoints[k].y - point.y) * (vecObjectHypo[objectIndex].isoPoints.vecPoints[k].y - point.y)
					  + (vecObjectHypo[objectIndex].isoPoints.vecPoints[k].z - point.z) * (vecObjectHypo[objectIndex].isoPoints.vecPoints[k].z - point.z));
		double weight;
		weight = GaussianFunction(distance);
		confidenceScore += weight* vecObjectHypo[objectIndex].isoPoints.vecPoints[k].f;
	}
	return confidenceScore;
}

void CAreaInterest::ComputeScore()
{
	//check here
	ofstream outFileh("Output\\ObjectHypo.txt",ios::app);
	for(int i = 0;i < vecvecMultiResult.size();i++)
	{
		UpdateObjectness(i); 
	}

	for(int i = 0;i < vecObjectHypo.size();i++)
	{
		ObjectHypo objectHypo;
		objectHypo = vecObjectHypo[i];
		outFileh << "objectHypo.patchIndex " << objectHypo.patchIndex.size() <<  endl;
		outFileh << "objectHypo.objectness " << objectHypo.objectness <<  endl;
	}
	outFileh.close();


	ofstream outFilee("Output\\EdgeHypo.txt",ios::app);
	for(int i = 0;i < vecvecMultiResult.size();i++)
	{
		for(int j = 0;j < vecvecMultiResult.size();j++)
		{
			if(i == j)	continue;
			UpdateSeparateness(i,j);
		}
	}

	for(int i = 0;i < vecEdgeHypo.size();i++)
	{
		EdgeHypo edgeHypo;
		edgeHypo = vecEdgeHypo[i];
		outFilee << "edgeHypo.begin " << edgeHypo.begin <<  endl;
		outFilee << "edgeHypo.end " << edgeHypo.end <<  endl;
		outFilee << "edgeHypo.separateness " << edgeHypo.separateness <<  endl;
	}
	outFilee.close();

// 	Sorting(vecObjectHypo,vecEdgeHypo,vecObjectSorting,vecEdgeSorting);
// 
// 	ofstream outFiles("Output\\Sorting.txt");
// 	for(int i = 0;i < vecObjectSorting.size();i++)
// 	{
// 		outFiles << "vecObjectSorting: " << vecObjectSorting[i] <<  endl;
// 	}
// 	for(int i = 0;i < vecEdgeSorting.size();i++)
// 	{
// 		outFiles << "vecEdgeSorting: " << vecEdgeSorting[i] <<  endl;
// 	}
// 	outFiles.close();

}

void CAreaInterest::UpdateObjectness(int m)
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
//			if(!vecGeometryConvex[i])
				concaveSum += 1 - vecSmoothValue[i];
		}
	}

	if(edgeNum > 0)
		concaveSum /= edgeNum;
	else
		concaveSum = 0;

	concaveSum *= 1000;

	if(concaveSum < 0.000001)
		concaveSum = 0;

//	objectness = fSum * concaveSum; 
	objectness = concaveSum; 

	vecObjectHypo[m].objectness = objectness;

}

void CAreaInterest::UpdateSeparateness(int m,int n)
{
	ofstream outFilee("Output\\ComputeSeparateness.txt");

	double separateness = 0;
	int hypoIndex = -1;

	for(int i = 0;i < vecEdgeHypo.size();i++)
	{
		outFilee << "i: " << i << endl;
		if(vecEdgeHypo[i].begin == m && vecEdgeHypo[i].end == n)
		{
			hypoIndex = i;
			outFilee << "yes" << endl;
		}
	}

	if(hypoIndex == -1)		return;

	outFilee << "1" <<endl;
	for(int i = 0;i < vecEdgeHypo[hypoIndex].pairPatch.size();i++)
	{
		for(int j = 0;j < vecpairPatchConnection.size();j++)
		{
			outFilee << "i: " << i << "j: " << j << endl;
			if(vecEdgeHypo[hypoIndex].pairPatch[i].first == vecpairPatchConnection[j].first && vecEdgeHypo[hypoIndex].pairPatch[i].second == vecpairPatchConnection[j].second)
			{
				separateness += vecSmoothValue[j];
				outFilee << "break " << endl;
				break;
			}
			if(vecEdgeHypo[hypoIndex].pairPatch[i].first == vecpairPatchConnection[j].second && vecEdgeHypo[hypoIndex].pairPatch[i].second == vecpairPatchConnection[j].first)
			{
				separateness += vecSmoothValue[j];
				outFilee << "break " << endl;
				break;
			}
		}
	}

	outFilee << "2" <<endl;
	vecEdgeHypo[hypoIndex].separateness = separateness;
}

double CAreaInterest::GaussianFunction(double x)
{
	double para = 1;
	double value = pow(2.7183,- x * x / para / para);

	return value;
}

void CAreaInterest::UpdateGraph()
{
	//check here
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
	objMax = 3000;
	objMin = 0;

	for(int i = 0;i < vecEdgeHypo.size();i++)
	{
		if(sepaMax < vecEdgeHypo[i].separateness)
			sepaMax =vecEdgeHypo[i].separateness;
		if(sepaMin > vecEdgeHypo[i].separateness)
			sepaMin = vecEdgeHypo[i].separateness;

	}
// 	sepaMax = 0.02;
// 	sepaMin = 0;

	ofstream outFileg("Output\\ObjectnessSeparateness.txt",ios::app);
	outFileg << "let's begin :) " << endl;

	for(int i = 0;i < vecObjectHypo.size();i++)
	{
		if(vecObjectHypo[i].objectness < 0)
			vecObjectHypo[i].objectness = 0;
		double r,g,b;
		GetColour(vecObjectHypo[i].objectness,objMin,objMax,r,g,b);
		graphContract.vecNodes[i].r = r;
		graphContract.vecNodes[i].g = g;
		graphContract.vecNodes[i].b = b;
		outFileg << "i: " <<  i<< "  objectness: " << vecObjectHypo[i].objectness <<   endl;
	}

	outFileg << "//////////////////////" << endl;
	for(int i = 0;i < vecEdgeHypo.size();i++)
	{
		double r,g,b;
		GetColour(vecEdgeHypo[i].separateness,sepaMin,sepaMax,r,g,b);
		graphContract.vecEdgeColor.push_back(r);
		graphContract.vecEdgeColor.push_back(g);
		graphContract.vecEdgeColor.push_back(b);
		outFileg << "i: " << i << "  separateness: " << vecEdgeHypo[i].separateness <<   endl;
	}
	outFileg.close();

}

void CAreaInterest::GetColour(double v,double vmin,double vmax,double &r,double &g,double &b)
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

void CAreaInterest::GetNewOneObjectPly()
{
	ofstream outFileg("Output\\GetNewOneObjectPly.txt",ios::app);
	outFileg << "let's begin :) " << endl;

	PointCloudPtr_RGB_NORMAL cloud(new PointCloud_RGB_NORMAL);
	MeshFace meshFaceAdd;
	MeshVertex meshVertexAdd;
	loadPointCloud_normal_ply("test/1.ply", cloud, meshFaceAdd, meshVertexAdd);

	double r,g,b;
	r = double(rand()%255);
	g = double(rand()%255);
	b = double(rand()%255);

	for(int i = 0; i < meshFaceAdd.vecFace.size();i++)
	{
		meshFaceAdd.vecFace[i].p0 += meshVertexExtra.vecVertex.size();
		meshFaceAdd.vecFace[i].p1 += meshVertexExtra.vecVertex.size();
		meshFaceAdd.vecFace[i].p2 += meshVertexExtra.vecVertex.size();

		meshFaceAdd.vecFace[i].r = r;
		meshFaceAdd.vecFace[i].g = g;
		meshFaceAdd.vecFace[i].b = b;
	}

	meshVertexExtra.vecVertex.insert(meshVertexExtra.vecVertex.end(),meshVertexAdd.vecVertex.begin(),meshVertexAdd.vecVertex.end());
	meshFaceExtra.vecFace.insert(meshFaceExtra.vecFace.end(),meshFaceAdd.vecFace.begin(),meshFaceAdd.vecFace.end());


	MyPt_RGB_NORMAL point;
	point.x = point.y = point.z = 0;
	for(int j = 0;j < meshVertexAdd.vecVertex.size();j++)
	{
		point.x += meshVertexAdd.vecVertex[j].x;
		point.y += meshVertexAdd.vecVertex[j].y;
		point.z += meshVertexAdd.vecVertex[j].z;
	}
	point.x /= meshVertexAdd.vecVertex.size();
	point.y /= meshVertexAdd.vecVertex.size();
	point.z /= meshVertexAdd.vecVertex.size();
	point.r = 0;
	point.g = 0;
	point.b = 1.0;


	graphContract.vecNodes.clear();
	graphContract.vecEdges.clear();
	graphContract.vecEdgeColor.clear();

	graphContract.vecNodes.push_back(point);

	outFileg.close();
}

void CAreaInterest::ComputeOverallHypo()
{
	ofstream outFileg("Output\\ComputeOverallHypo.txt",ios::app);
	outFileg << "let's begin :) " << endl;

	vecObjectHypo.clear();
	ObjectHypo objectHypo;
	for(int i = 0;i < vecPatchPoint.size();i++)
	{
		objectHypo.patchIndex.push_back(i);
	}
	objectHypo.objectness = 0;
	objectHypo.mergeFlag = true;
	vecObjectHypo.push_back(objectHypo);

	outFileg.close();
}

void CAreaInterest::ComputeNewContractGraph()
{
	graphContract.vecNodes.clear();
	graphContract.vecEdges.clear();
	graphContract.vecEdgeColor.clear();

	for(int i = 0;i < vecObjectHypo.size();i++)
	{
		MyPt_RGB_NORMAL point;
		point.x = point.y = point.z = 0;
		for(int j = 0;j < vecObjectHypo[i].patchIndex.size();j++)
		{
			point.x += vecPatchCenPoint[vecObjectHypo[i].patchIndex[j]].x;
			point.y += vecPatchCenPoint[vecObjectHypo[i].patchIndex[j]].y;
			point.z += vecPatchCenPoint[vecObjectHypo[i].patchIndex[j]].z;
		}
		point.x /= vecObjectHypo[i].patchIndex.size();
		point.y /= vecObjectHypo[i].patchIndex.size();
		point.z /= vecObjectHypo[i].patchIndex.size();
		point.r = 0;
		point.g = 0;
		point.b = 1.0;

		vecObjectHypo[i].cenPoint.x = point.x;
		vecObjectHypo[i].cenPoint.y = point.y;
		vecObjectHypo[i].cenPoint.z = point.z;

		graphContract.vecNodes.push_back(point);
	}
}