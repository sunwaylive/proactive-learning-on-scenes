#include "BinarySeg.h"

vector<vector<int>> vecvecObjectPool;

CBinarySeg::CBinarySeg()
{
	thresholdClose0 = 0.005;  //判断patch是否相连
	thresholdClose1 = 0.05;	  //计算局部法向量
	paraK = 3;
	paraS = 0.02;
	paraConvexK = 0.1;
	paraConvexT = 0.9;
	paraConcave = 1.0;
	paraGeometry = 0.9;
	paraAppearence = 0.6;
	seedPatch = 0;
	paraAlpha = 0.0;
}

CBinarySeg::~CBinarySeg(void)
{

}

void CBinarySeg::AddTable(PointCloudPtr_RGB_NORMAL &table)
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

void CBinarySeg::AddClusterPoints(vector<MyPointCloud_RGB_NORMAL> &points)
{
	for(int i=0;i<points.size();i++)
	{
		vecPatchPoint.push_back(points[i]);
	}
	clusterPatchNum.push_back(points.size());
}

void CBinarySeg::AddPatchNormal(vector<Normal> &normal)
{
	vecPatcNormal = normal;
}

void CBinarySeg::MainStep()
{
	PointCloudPreprocess();

	paraGeometry = 0.4;
	paraAppearence = 0.1;
	paraAlpha = 0.8;

// 	while(paraGeometry < 0.3)
// 	{
// 		while(paraAppearence < 0.3)
// 		{
// 			while(paraAlpha < 3)
// 			{
				vecvecObjectPool.clear();
//				for(int i =172; i <240; i ++)
				for(int i =0; i <vecPatchPoint.size(); i ++)
				{
					seedPatch = i;
					vector<int> vecObjectHypo;
					vecObjectHypo.clear();
					GraphConstruct();
					GraphCutSolve(vecObjectHypo);
					vecvecObjectPool.push_back(vecObjectHypo);
				}

				//output
				ofstream outFile0("Output\\ObjectPool.txt",ios::out|ios::app);
				outFile0 <<paraK<<  "  "<<paraGeometry<<  "  "<<paraAppearence<<  "  "<<paraAlpha<<  "  "<< endl;
				for(int i=0;i<vecvecObjectPool.size();i++)
				{
					for(int j=0;j<vecvecObjectPool[i].size();j++)
					{
						outFile0 << vecvecObjectPool[i][j] <<  "  ";
					}
					outFile0 << endl;
				}
				outFile0 <<  "  " << endl;
				outFile0 <<  "  " << endl;
				outFile0.close();

//				paraAlpha += (double)0.2;
//			}
// 			paraAlpha = 0;
// 			paraAppearence += (double)0.02;
// 		}
// 		paraAppearence = 0;
// 		paraGeometry += (double)0.02;
// 	}	


		
		//output
// 		ofstream outFile1("Output\\PatchPoint.txt");
// 		for(int i=0;i<vecPatchPoint.size();i++)
// 		{
// 			for(int j=0;j<vecPatchPoint[i].mypoints.size();j++)
// 			{
// 				outFile1<< vecPatchPoint[i].mypoints[j].x << "  " <<
// 					vecPatchPoint[i].mypoints[j].y << "  " <<
// 					vecPatchPoint[i].mypoints[j].z << "  " <<
// 					vecPatchPoint[i].mypoints[j].r << "  " <<
// 					vecPatchPoint[i].mypoints[j].g << "  " <<
// 					vecPatchPoint[i].mypoints[j].b << "  " ;
// 			}
// 			outFile1 <<  endl;
// 		}
// 		outFile1.close();

 	//output
// 	ofstream outFile1("Output\\ClusterSize.txt");
// 	for(int i=0;i<clusterPatchNum.size();i++)
// 	{
// 		outFile1 << clusterPatchNum[i] <<  "  ";
// 		
// 	}
// 	outFile1 <<  "  " << endl;
// 	outFile1.close();
// 
// 	//output
// 	ofstream outFile2("Output\\PatchNormal.txt");
// 	for(int i=0;i<vecPatcNormal.size();i++)
// 	{
// 		outFile2 << vecPatcNormal[i].normal_x <<  "  " << vecPatcNormal[i].normal_y <<  "  " << vecPatcNormal[i].normal_z << endl;
// 	}
// 	outFile2.close();
}

void CBinarySeg::PointCloudPreprocess()
{
	//bounding box, color average, center position
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
// 			color.mRed += vecPatchPoint[i].mypoints[j].r;
// 			color.mGreen += vecPatchPoint[i].mypoints[j].g;
// 			color.mBlue += vecPatchPoint[i].mypoints[j].b;

			vecColorDetial[(int)(vecPatchPoint[i].mypoints[j].r / 40) ] += 1;
			vecColorDetial[(int)(vecPatchPoint[i].mypoints[j].g / 40)  + 7] += 1;
			vecColorDetial[(int)(vecPatchPoint[i].mypoints[j].b / 40)  + 14] += 1;

			point.x += vecPatchPoint[i].mypoints[j].x;
			point.y += vecPatchPoint[i].mypoints[j].y;
			point.z += vecPatchPoint[i].mypoints[j].z;
			if(xMax < point.x) xMax = vecPatchPoint[i].mypoints[j].x;
			if(yMax < point.y) yMax = vecPatchPoint[i].mypoints[j].y;
			if(zMax < point.z) zMax = vecPatchPoint[i].mypoints[j].z;
			if(xMin > point.x) xMin = vecPatchPoint[i].mypoints[j].x;
			if(yMin > point.y) yMin = vecPatchPoint[i].mypoints[j].y;
			if(zMin > point.z) zMin = vecPatchPoint[i].mypoints[j].z;
		}
		if(vecPatchPoint[i].mypoints.size() > 0)
		{
// 			color.mRed /= vecPatchPoint[i].mypoints.size();
// 			color.mGreen /= vecPatchPoint[i].mypoints.size();
// 			color.mBlue /= vecPatchPoint[i].mypoints.size();
			point.x/= vecPatchPoint[i].mypoints.size();
			point.y /= vecPatchPoint[i].mypoints.size();
			point.z /= vecPatchPoint[i].mypoints.size();
		}

		vecPatchColor.push_back(color);
		vecvecPatchColorDetial.push_back(vecColorDetial);
		vecPatchCenPoint.push_back(point);
	}
	boundingBoxSize = sqrt(pow(xMax-xMin,2) + pow(yMax-yMin,2) +pow(zMax-zMin,2));

	//distance between patches
	vecvecPatchMinDis.resize(vecPatchPoint.size());
	vecvecPatchCenDis.resize(vecPatchPoint.size());
//	vecvecNearbyPoint.resize(vecPatchPoint.size());
//	vecvecPatctNearbyNormal.resize(vecPatchPoint.size());
	for(int i = 0;i <vecPatchPoint.size();i++)
	{
		vecvecPatchMinDis[i].resize(vecPatchPoint.size());
		vecvecPatchCenDis[i].resize(vecPatchPoint.size());
//		vecvecNearbyPoint[i].resize(vecPatchPoint.size());
//		vecvecPatctNearbyNormal[i].resize(vecPatchPoint.size());
	}

	int countPatch = 0;
	int patchBegin,patchEnd;
	for(int i = 0;i <clusterPatchNum.size();i++)
	{
		patchBegin = countPatch;
		patchEnd = countPatch + clusterPatchNum[i];
		GetAdjacency(patchBegin,patchEnd);
		countPatch += clusterPatchNum[i];
	}
		
	//handle table
	vecIfConnectTable.clear();
	for(int i = 0;i <vecPatchPoint.size();i++)
	{
		vecIfConnectTable.push_back(IfConnectTable(vecPatchPoint[i].mypoints));
	}

	//output
// 	ofstream outFile1("Output\\color.txt");
// 	for(int i = 0;i <vecvecPatchColorDetial.size();i++)
// 	{
// 		outFile1 << "patch "<< i << endl;
// 		for(int j=0;j<vecvecPatchColorDetial[i].size();j++)
// 		{
// 			outFile1 << "data " << vecvecPatchColorDetial[i][j] << endl;
// 		}
// 		outFile1 << " " << endl;
// 		outFile1 << " " << endl;
// 	}
// 	outFile1.close();

	//output
	ofstream outFile0("Output\\basicinfo.txt");
	for(int i = 0;i <clusterPatchNum.size();i++)
	{
		outFile0 << "clusterPatchNum: " << clusterPatchNum[i] << " " ;
	}
	outFile0 << "   " << endl;
	
	outFile0 << "vecPatchPoint: " << vecPatchPoint.size() << " " ;
	outFile0 << "   " << endl;

	outFile0 << "bounding box   xMax: " << xMax << " xMin: " << xMin 
						  << " yMax: " << yMax << " yMin: " << yMin
						  << " zMax: " << zMax << " zMin: " << zMin << endl;
// 	for(int i = 0;i <vecPatchColor.size();i++)
// 	{
// 		outFile0 << "color: " << vecPatchColor[i].mBlue << " " << vecPatchColor[i].mRed << " " << vecPatchColor[i].mGreen << endl;
// 	}
// 	outFile0 << "   " << endl;

	for(int i = 0;i <vecPatchPoint.size();i++)
	{
		outFile0 << "patch size: " << vecPatchPoint[i].mypoints.size() << endl;
	}
	outFile0 << "   " << endl;

	for(int i = 0;i <vecPatchCenPoint.size();i++)
	{
		outFile0 << "center point: " << vecPatchCenPoint[i].x << " " << vecPatchCenPoint[i].y << " " << vecPatchCenPoint[i].z << endl;
	}
	outFile0 << "   " << endl;

	for(int i = 0;i <vecPatcNormal.size();i++)
	{
		outFile0 << "average normal: " << vecPatcNormal[i].normal_x << " "<< vecPatcNormal[i].normal_y << " " <<vecPatcNormal[i].normal_z << endl;
	}
	outFile0 << "   " << endl;
	outFile0.close();

	//output
	ofstream outFile2("Output\\nearbyinfo.txt");
// 	for(int i = 0;i <vecIfConnectTable.size();i++)
// 	{
// 		outFile2 << "connecttable " << vecIfConnectTable[i] <<  " "  ;
// 	}
// 	outFile2 << "   " << endl;

	for(int i = 0;i <vecpairPatchConnection.size();i++)
	{
		outFile2 << "patch " << vecpairPatchConnection[i].first << " "<< vecpairPatchConnection[i].second << " "  << endl;
	}
	outFile2 << "   " << endl;

// 	for(int i = 0;i <vecvecNearbyPoint.size();i++)
// 	{
// 		for(int j = 0;j <vecvecNearbyPoint[i].size();j++)
// 		{
// 			if(vecvecNearbyPoint[i][j].nearbyPoint.size()>0)
// 				for(int k = 0;k < vecvecNearbyPoint[i][j].nearbyPoint.size();k++)
// 					outFile2 << "nearby " << vecvecNearbyPoint[i][j].nearbyPoint[k].indexFirst << " "<< vecvecNearbyPoint[i][j].nearbyPoint[k].indexSecond << " patch " <<
// 					vecvecNearbyPoint[i][j].nearbyPoint[k].patchFirst << " "<< vecvecNearbyPoint[i][j].nearbyPoint[k].patchSecond << " " << endl;
// 		}
// 	}
	outFile2 << "   " << endl;
	outFile2.close();

// 	ofstream outFile3("Output\\nearbynormal.txt");
// 	for(int i = 0;i <vecvecPatctNearbyNormal.size();i++)
// 	{
// 		for(int j = 0;j <vecvecPatctNearbyNormal[i].size();j++)
// 		{
// 			if(vecvecNearbyPoint[i][j].nearbyPoint.size())
// 				outFile3 << "nearbynormal  i " << i << "  j  " << j << "   num  " << 
// 										   vecvecNearbyPoint[i][j].nearbyPoint.size() <<"  value  "<<
// 										   vecvecPatctNearbyNormal[i][j].normal0.normal_x << " "<< 
// 				                           vecvecPatctNearbyNormal[i][j].normal0.normal_y << " "<< 
// 										   vecvecPatctNearbyNormal[i][j].normal0.normal_z << " "<< 
// 										   vecvecPatctNearbyNormal[i][j].normal1.normal_x << " "<< 
// 										   vecvecPatctNearbyNormal[i][j].normal1.normal_y << " "<< 
// 										   vecvecPatctNearbyNormal[i][j].normal1.normal_z << endl;
// 		}
// 	}
// 	outFile3 << "   " << endl;
// 	outFile3.close();

}

void CBinarySeg::GraphConstruct()
{
	//graph value
	pair<int,int> pairSmoothVertex;

	vecDataValue.clear();
	vecSmoothValue.clear();
	vecGeometryValue.clear();
	vecAppearenceValue.clear();

	for(int i = 0;i <vecPatchPoint.size();i++)
	{
		vecDataValue.push_back(GetBinaryDataValue(vecvecPatchCenDis[seedPatch][i]));
	}

	for(int i = 0;i <vecpairPatchConnection.size();i++)
	{
		vecSmoothValue.push_back(GetBinarySmoothValue(vecpairPatchConnection[i].first,vecpairPatchConnection[i].second));
	}
	
	//color nomalization
	double maxSV = SMALL_NUM;
	double minSV = LARGE_NUM;

	for(int i = 0;i <vecpairPatchConnection.size();i++)
	{
		if(maxSV < vecAppearenceValue[i])
			maxSV = vecAppearenceValue[i];
		if(minSV > vecAppearenceValue[i])
			minSV = vecAppearenceValue[i];
	}
	for(int i = 0;i <vecpairPatchConnection.size();i++)
	{
		vecAppearenceValue[i] = (vecAppearenceValue[i] - minSV)/(maxSV - minSV);
		vecAppearenceValue[i] = 1 - vecAppearenceValue[i];
	}

	for(int i = 0;i <vecpairPatchConnection.size();i++)
	{
		vecGeometryValue[i] = paraGeometry * vecGeometryValue[i];
		vecAppearenceValue[i] = paraAppearence * vecAppearenceValue[i];
		vecSmoothValue[i] = vecGeometryValue[i] + vecAppearenceValue[i];
	}

// 	NomalizeData();
 	NomalizeSmooth();

	//output
	ofstream outFile1("Output\\datasmooth.txt");
	for(int i = 0;i <vecPatchColor.size();i++)
	{
		outFile1 << "data  " << vecDataValue[i] << endl;
	}
	for(int i = 0;i <vecpairPatchConnection.size();i++)
	{
		outFile1 << "GeometryValue  " << vecGeometryValue[i] << endl;
		outFile1 << "AppearenceValue  " << vecAppearenceValue[i] << endl;
		outFile1 << "smooth  " << vecSmoothValue[i] << endl;
	}
	outFile1.close();
}

void CBinarySeg::GraphCutSolve(vector<int>& vecObjectHypo)
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
			g -> add_tweights(i, paraAlpha, vecDataValue[i]);
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

	printf("foreground: %d ,background: %d\n",countSOURCE,countSINK);
}

void CBinarySeg::GetAdjacency(int patchBegin,int patchEnd)
{
	pair<int,int> pairPatchConnection;
	for(int i = patchBegin;i <patchEnd;i++)
		for(int j = patchBegin;j <patchEnd;j++)
			if(i != j)
			{
				bool  stable;   // if connect stable
				vecvecPatchMinDis[i][j] = vecvecPatchMinDis[j][i] = GetMinDisBetPatch(i,j,stable);
				vecvecPatchCenDis[i][j] = vecvecPatchMinDis[j][i] = GetCenDisBetPatch(i,j);
				pairPatchConnection.first = i;
				pairPatchConnection.second = j;
				if(vecvecPatchMinDis[i][j] < thresholdClose0 && stable)
				{
					vecpairPatchConnection.push_back(pairPatchConnection);
				} 
				pairPatchConnection.first = j;
				pairPatchConnection.second = i;
				if(vecvecPatchMinDis[i][j] < thresholdClose0 && stable)
				{
					vecpairPatchConnection.push_back(pairPatchConnection);
				} 
			}
			else
			{
				vecvecPatchMinDis[i][j] = LARGE_NUM;
				vecvecPatchCenDis[i][j] = LARGE_NUM;
			}

}

double CBinarySeg::GetMinDisBetPatch(int m,int n,bool &stable)
{
	double minDis=LARGE_NUM;
// 	Normal normal0,normal1;
// 	normal0.normal_x = normal0.normal_y = normal0.normal_z = 0;
// 	normal1.normal_x = normal1.normal_y = normal1.normal_z = 0;

	vector<double> vecDisQueue;
	vector<bool> vecPointClose0(vecPatchPoint[m].mypoints.size(),false);
	vector<bool> vecPointClose1(vecPatchPoint[n].mypoints.size(),false);
	
	for(int i = 0;i < vecPatchPoint[m].mypoints.size();i++)
	{
		for(int j = 0;j < vecPatchPoint[n].mypoints.size();j++)
		{
			double dis = sqrt(pow(vecPatchPoint[m].mypoints[i].x-vecPatchPoint[n].mypoints[j].x,2)
				+ pow(vecPatchPoint[m].mypoints[i].y-vecPatchPoint[n].mypoints[j].y,2)
				+ pow(vecPatchPoint[m].mypoints[i].z-vecPatchPoint[n].mypoints[j].z,2));
			if(minDis > dis)
			{
				minDis = dis;
			}

			if(dis < thresholdClose0 * 3)
			{
				vecPointClose0[i] = true;
				vecPointClose1[j] = true;
			}
// 			if(dis < thresholdClose1 * boundingBoxSize)
// 			{
// 				NEARBYPOINT nearbyPoint;
// 				nearbyPoint.patchFirst = m;
// 				nearbyPoint.patchFirst = n;
// 				nearbyPoint.indexFirst = i;
// 				nearbyPoint.indexSecond = j;
// 				vecvecNearbyPoint[m][n].nearbyPoint.push_back(nearbyPoint);
// 
// 				//计算相对normal
// 				normal0.normal_x += vecPatchPoint[m].mypoints[i].normal_x;
// 				normal0.normal_y += vecPatchPoint[m].mypoints[i].normal_y;
// 				normal0.normal_z += vecPatchPoint[m].mypoints[i].normal_z;
// 
// 				normal1.normal_x += vecPatchPoint[n].mypoints[j].normal_x;
// 				normal1.normal_y += vecPatchPoint[n].mypoints[j].normal_y;
// 				normal1.normal_z += vecPatchPoint[n].mypoints[j].normal_z;
// 			}
		}
	}

// 	if(vecvecNearbyPoint[m][n].nearbyPoint.size())
// 	{
// 		normal0.normal_x /=  vecvecNearbyPoint[m][n].nearbyPoint.size();
// 		normal0.normal_y /=  vecvecNearbyPoint[m][n].nearbyPoint.size();
// 		normal0.normal_z /=  vecvecNearbyPoint[m][n].nearbyPoint.size();
// 		normal1.normal_x /=  vecvecNearbyPoint[m][n].nearbyPoint.size();
// 		normal1.normal_y /=  vecvecNearbyPoint[m][n].nearbyPoint.size();
// 		normal1.normal_z /=  vecvecNearbyPoint[m][n].nearbyPoint.size();
// 
// 		double nomalizeValue;
// 		nomalizeValue = sqrt(normal0.normal_x * normal0.normal_x +
// 						 	 normal0.normal_y * normal0.normal_y +
// 							 normal0.normal_z * normal0.normal_z);
// 		vecvecPatctNearbyNormal[m][n].normal0.normal_x = normal0.normal_x / nomalizeValue;
// 		vecvecPatctNearbyNormal[m][n].normal0.normal_y = normal0.normal_y / nomalizeValue;
// 		vecvecPatctNearbyNormal[m][n].normal0.normal_z = normal0.normal_z / nomalizeValue;
// 
// 		nomalizeValue = sqrt(normal1.normal_x * normal1.normal_x +
// 			normal1.normal_y * normal1.normal_y +
// 			normal1.normal_z * normal1.normal_z);
// 		vecvecPatctNearbyNormal[m][n].normal1.normal_x = normal1.normal_x / nomalizeValue;
// 		vecvecPatctNearbyNormal[m][n].normal1.normal_y = normal1.normal_y / nomalizeValue;
// 		vecvecPatctNearbyNormal[m][n].normal1.normal_z = normal1.normal_z / nomalizeValue;
// 
// 		vecvecPatctNearbyNormal[n][m] = vecvecPatctNearbyNormal[m][n];
// 	}

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

double CBinarySeg::GetCenDisBetPatch(int m,int n)
{
	double dis =  sqrt(pow(vecPatchCenPoint[m].x-vecPatchCenPoint[n].x,2)
		+ pow(vecPatchCenPoint[m].y-vecPatchCenPoint[n].y,2)
		+ pow(vecPatchCenPoint[m].z-vecPatchCenPoint[n].z,2));
	return dis;
}

double CBinarySeg::GetBinaryDataValue(double d)
{
	if(d == LARGE_NUM || d == 0)  
	{
		return LARGE_NUM;
	}
	else
	{
		double penaltyValue = 0;
		if(d > boundingBoxSize * paraS)
			penaltyValue =  paraK * (d - boundingBoxSize* paraS);
		return penaltyValue;
	}
	
}

double CBinarySeg::GetBinarySmoothValue(int m,int n)
{
	double smoothValue,geometryValue,appearenceValue;

	//normal
	MyPoint cenM,cenN;
	Normal norM,norN,norMN,norNM;
	double nomalizeValue;

	cenM = vecPatchCenPoint[m];
	cenN = vecPatchCenPoint[n];

// 	if(m < n)
// 	{
// 		norM = vecvecPatctNearbyNormal[m][n].normal0;
// 		norN = vecvecPatctNearbyNormal[m][n].normal1;
// 	}
// 	else
// 	{
// 		norM = vecvecPatctNearbyNormal[m][n].normal1;
// 		norN = vecvecPatctNearbyNormal[m][n].normal0;
// 	}

	norM = vecPatcNormal[m];
	norN = vecPatcNormal[n];

	norMN.normal_x = cenN.x - cenM.x;
	norMN.normal_y = cenN.y - cenM.y;
	norMN.normal_z = cenN.z - cenM.z;
	nomalizeValue = sqrt(norMN.normal_x * norMN.normal_x + norMN.normal_y * norMN.normal_y + norMN.normal_z * norMN.normal_z);
	norMN.normal_x /= nomalizeValue;
	norMN.normal_y /= nomalizeValue;
	norMN.normal_z /= nomalizeValue;

	bool convexFlag;
	double convexValue;   //convex if > 0
	convexValue = norMN.normal_x * norN.normal_x +  norMN.normal_y * norN.normal_y + norMN.normal_z * norN.normal_z;
	
	if(convexValue >= 0)	
		convexFlag = true;
	else	
		convexFlag = false;

	double cosValue;
	cosValue = (norM.normal_x * norN.normal_x + norM.normal_y * norN.normal_y + norM.normal_z * norN.normal_z);
	if(convexFlag)	
		geometryValue = paraConvexK * cosValue + paraConvexT;
	else	
		geometryValue = paraConcave * cosValue;
	if(geometryValue < 0)	geometryValue = 0;

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

	return smoothValue;
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

void CBinarySeg::NomalizeData()
{
	double maxSV = SMALL_NUM;
	double minSV = LARGE_NUM;

	for(int i = 0;i <vecDataValue.size();i++)
	{
		if(maxSV < vecDataValue[i] && vecDataValue[i] != LARGE_NUM)
			maxSV = vecDataValue[i];
		if(minSV > vecDataValue[i] && vecDataValue[i] != LARGE_NUM)
			minSV = vecDataValue[i];
	}
	for(int i = 0;i <vecDataValue.size();i++)
	{
		if( vecDataValue[i] != LARGE_NUM)
			vecDataValue[i] =  (vecDataValue[i] - minSV)/(maxSV - minSV)/2 + 1;
	}
}

void CBinarySeg::NomalizeSmooth()
{
	double maxSV = SMALL_NUM;
	double minSV = LARGE_NUM;

	for(int i = 0;i <vecSmoothValue.size();i++)
	{
		if(maxSV < vecSmoothValue[i])
			maxSV = vecSmoothValue[i];
		if(minSV > vecSmoothValue[i])
			minSV = vecSmoothValue[i];
	}
	for(int i = 0;i <vecSmoothValue.size();i++)
	{
		vecSmoothValue[i] =  0.5 * (vecSmoothValue[i] - minSV)/(maxSV - minSV);
	}
}