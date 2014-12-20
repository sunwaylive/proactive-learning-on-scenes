#include "BinarySeg.h"



CBinarySeg::CBinarySeg(vector<MyPointCloud_RGB> points,vector<Normal> normals)
{
	vecPatchPoint = points;
	vecPatcNormal = normals;

	paraClose = 150.0;
	paraK = 0.5;
	paraS = 0.0001;
	paraConvexK = 0.1;
	paraConvexT = 0.9;
	paraConcave = 1.0;
	paraGeometry = 1.25;
	paraAppearence = 1.25;
}


CBinarySeg::~CBinarySeg(void)
{
}

void CBinarySeg::MainStep()
{
	PointCloudPreprocess();
	GraphConstruct();
	GraphCutSolve();		
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
		color.mRed = color.mGreen = color.mBlue = 0;
		point.x = point.y = point.z =0;

		for(int j = 0;j < vecPatchPoint[i].mypoints.size();j++)
		{
			color.mRed += vecPatchPoint[i].mypoints[j].r;
			color.mGreen += vecPatchPoint[i].mypoints[j].g;
			color.mBlue += vecPatchPoint[i].mypoints[j].b;
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
			color.mRed /= vecPatchPoint[i].mypoints.size();
			color.mGreen /= vecPatchPoint[i].mypoints.size();
			color.mBlue /= vecPatchPoint[i].mypoints.size();
			point.x/= vecPatchPoint[i].mypoints.size();
			point.y /= vecPatchPoint[i].mypoints.size();
			point.z /= vecPatchPoint[i].mypoints.size();
		}

		vecPatchColor.push_back(color);
		vecPatchCenPoint.push_back(point);
	}

	boundingBoxSize = sqrt(pow(xMax-xMin,2) + pow(yMax-yMin,2) +pow(zMax-zMin,2));
	closeThreshold = boundingBoxSize / paraClose;

	//distance between patches
	pair<int,int> pairPatchConnection;
	vecvecPatchMinDis.resize(vecPatchPoint.size());
	vecvecPatchCenDis.resize(vecPatchPoint.size());
	for(int i = 0;i <vecPatchPoint.size();i++)
	{
		vecvecPatchMinDis[i].resize(vecPatchPoint.size());
		vecvecPatchCenDis[i].resize(vecPatchPoint.size());
	}

	int countLine=0;
	for(int i = 0;i <vecPatchPoint.size();i++)
		for(int j = 0;j <vecPatchPoint.size();j++)
			if(i != j)
			{
				vecvecPatchMinDis[i][j] = GetMinDisBetPatch(i,j);
				vecvecPatchCenDis[i][j] = GetCenDisBetPatch(i,j);
				pairPatchConnection.first = i;
				pairPatchConnection.second = j;
				if(vecvecPatchMinDis[i][j] < closeThreshold)
				{
					vecpairPatchConnection.push_back(pairPatchConnection);
					PointXYZ point0,point1;
					point0.x = vecPatchCenPoint[i].x;
					point0.y = vecPatchCenPoint[i].y;
					point0.z = vecPatchCenPoint[i].z;
					point1.x = vecPatchCenPoint[j].x;
					point1.y = vecPatchCenPoint[j].y;
					point1.z = vecPatchCenPoint[j].z;
					// 				  std::stringstream st0;
					// 				  st0<<"a"<<countLine<<"0";
					// 				  std::string id_line0=st0.str();
					// 				  vs.viewer->addLine(point0,point1,255,0,0,id_line0);
					// 				  countLine++;
				} 
			}
			else
			{
				vecvecPatchMinDis[i][j] = 0;
				vecvecPatchCenDis[i][j] = 0;
			}

	//output
// 	ofstream outFile0("kankan.txt");
// 	for(int i = 0;i <vecPatchColor.size();i++)
// 	{
// 		outFile0 << "color " << vecPatchColor[i].mBlue << " " << vecPatchColor[i].mRed << " " << vecPatchColor[i].mGreen << endl;
// 	}
// 	for(int i = 0;i <vecPatchPoint.size();i++)
// 	{
// 		outFile0 << "patch size " << vecPatchPoint[i].mypoints.size() << endl;
// 	}
// 	for(int i = 0;i <vecPatchCenPoint.size();i++)
// 	{
// 		outFile0 << "center point " << vecPatchCenPoint[i].x << vecPatchCenPoint[i].y << vecPatchCenPoint[i].z << endl;
// 	}
// 	for(int i = 0;i <vecPatcNormal.size();i++)
// 	{
// 		outFile0 << "normal " << vecPatcNormal[i].normal_x << vecPatcNormal[i].normal_y <<vecPatcNormal[i].normal_z << endl;
// 	}
// 	outFile0.close();

}

void CBinarySeg::GraphConstruct()
{
	/******************Graph Preprocess One************************/
	//graph value
	m = 300;
	pair<int,int> pairSmoothVertex;
	for(int i = 0;i <vecPatchPoint.size();i++)
	{
		vecDataValue.push_back(GetBinaryDataValue(vecvecPatchCenDis[m][i]));
	}

	for(int i = 0;i <vecpairPatchConnection.size();i++)
	{
		vecSmoothValue.push_back(GetBinarySmoothValue(vecpairPatchConnection[i].first,vecpairPatchConnection[i].second));
	}

	//output
// 	ofstream outFile1("kankan1.txt");
// 	for(int i = 0;i <vecPatchColor.size();i++)
// 	{
// 		outFile1 << "data  " << vecDataValue[i] << endl;
// 	}
// 	for(int i = 0;i <vecpairPatchConnection.size();i++)
// 	{
// 		outFile1 << "smooth  " << vecSmoothValue[i] << endl;
// 	}
// 	outFile1.close();
}

void CBinarySeg::GraphCutSolve()
{
	typedef Graph<double,double,double> GraphType;
	GraphType *g = new GraphType(/*estimated # of nodes*/ vecPatchPoint.size(), /*estimated # of edges*/ vecpairPatchConnection.size()); 

	g -> add_node(vecPatchPoint.size()); 


	for(int i = 0;i <vecDataValue.size();i++)
	{
		if(i != m)
			g -> add_tweights(i, 0, vecDataValue[i]);
		else
			g -> add_tweights(i, LARGE_NUM, 0);
	}

	for(int i = 0;i <vecSmoothValue.size();i++)
	{
		g -> add_edge(vecpairPatchConnection[i].first, vecpairPatchConnection[i].second, vecSmoothValue[i], vecSmoothValue[i]);
	}

	double flow = g -> maxflow();

	printf("Flow = %f\n", flow);
	printf("Minimum cut:\n");

	
	int countSOURCE,countSINK;
	countSINK = countSOURCE =0;
	for(int i=0;i<vecPatchPoint.size();i++)
		if (g->what_segment(i) == GraphType::SOURCE)
		{
			countSOURCE++;
			vecFore.push_back(i);
		}
		else
		{
			countSINK++;
			vecBack.push_back(i);
		}

	delete g; 
	printf("foreground: %d ,background: %d\n",countSOURCE,countSINK);

}

double CBinarySeg::GetMinDisBetPatch(int m,int n)
{
	double minDis=LARGE_NUM;
	for(int i = 0;i < vecPatchPoint[m].mypoints.size();i++)
	{
		for(int j = 0;j < vecPatchPoint[n].mypoints.size();j++)
		{
			double dis = sqrt(pow(vecPatchPoint[m].mypoints[i].x-vecPatchPoint[n].mypoints[j].x,2)
				+ pow(vecPatchPoint[m].mypoints[i].y-vecPatchPoint[n].mypoints[j].y,2)
				+ pow(vecPatchPoint[m].mypoints[i].z-vecPatchPoint[n].mypoints[j].z,2));
			if(minDis > dis)	
				minDis = dis;
		}
	}
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
	double penaltyValue = 0;
	if(d > boundingBoxSize * paraS)
		penaltyValue =  paraK * (d - boundingBoxSize* paraS) / (1-paraS);
	return penaltyValue;
}

double CBinarySeg::GetBinarySmoothValue(int m,int n)
{
	double smoothValue,geometryValue,appearenceValue;

	//normal
	MyPoint cenM,cenN;
	Normal norM,norN,norMN;
	cenM = vecPatchCenPoint[m];
	cenN = vecPatchCenPoint[n];
	norM = vecPatcNormal[m];
	norN = vecPatcNormal[n];
	norMN.normal_x = cenN.x - cenM.x;
	norMN.normal_y = cenN.y - cenM.y;
	norMN.normal_z = cenN.z - cenM.z;

	bool convexFlag;
	double convexValue;   //convex if > 0
	convexValue = norMN.normal_x * norN.normal_x +  norMN.normal_y * norN.normal_y + norMN.normal_z * norN.normal_z;
	if(convexValue >= 0)	convexFlag = true;
	else	convexFlag = false;

	double cosValue;
	cosValue = (norM.normal_x * norN.normal_x + norM.normal_y * norN.normal_y + norM.normal_z * norN.normal_z);
	if(convexFlag)	geometryValue = paraConvexK * cosValue + paraConvexT;
	else	geometryValue = paraConcave * cosValue;
	if(geometryValue < 0)	geometryValue = 0;

	//color
	appearenceValue = (vecPatchColor[m].mRed - vecPatchColor[n].mRed) 
		+(vecPatchColor[m].mGreen- vecPatchColor[n].mGreen)
		+(vecPatchColor[m].mBlue - vecPatchColor[n].mBlue);
	appearenceValue /= 256 * 3;
	if(appearenceValue < 0)	appearenceValue = 0;

	smoothValue = paraGeometry * geometryValue + paraAppearence * appearenceValue;
	return smoothValue;
}  
