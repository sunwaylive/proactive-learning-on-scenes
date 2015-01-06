#include "GraphCutGlobalHeader.h"
#include "Clustering.h"


CClustering::CClustering(void)
{
	paraH = 0.9;
}


CClustering::~CClustering(void)
{
}


void CClustering::AddObjectPool()
{
	//¶Ápatch point
// 	vecPatchPoint.clear();
// 	ifstream inFile("Output\\PatchPoint-table1.txt",std::ios::in);
// 	char buf[256000];
// 	bool flagStop = false;
// 	int count=0;
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
// 			line >> pointTemp.r;
// 			line >> pointTemp.g;
// 			line >> pointTemp.b;
// 
// 			if(pointTemp.x<100 && pointTemp.x>-100)
// 				patchTemp.mypoints.push_back(pointTemp);
// 			else
// 				flagStop = true;
// 		}
// 		while(flagStop == false);
// 
// 		vecPatchPoint.push_back(patchTemp);
// 		count++;
// 	}
// 	inFile.close();

	//¶ÁObject Pool
// 	vecvecObjectPool.clear();
// 	ifstream inFile1("Output\\ObjectPool.txt",std::ios::in);
// 	char buf[256000];
// 	bool flagStop = false;
// 	int count=0;
// 	while (inFile1.getline(buf, sizeof buf))
// 	{
// 		vector<int> vecObject;
// 		istringstream line(buf);
// 		if(count != 0)
// 		{
// 			flagStop = false;
// 			int index,indexOld;
// 			index = indexOld = -1;
// 			do
// 			{
// 				line >> index;
// 				if(index != indexOld && index <LARGE_NUM && index >0)
// 					vecObject.push_back(index);
// 				else
// 					flagStop = true;
// 				indexOld = index;
// 			}
// 			while(!flagStop);
// 			vecvecObjectPool.push_back(vecObject);
// 		}
// 		count++;
// 	}
// 	inFile1.close();
}

void CClustering::MainStep()
{
	CleanObjectPool();
	MeanShift();
}

void CClustering::CleanObjectPool()
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

void CClustering::MeanShift()
{
	for(int i = 0;i < vecvecObjectPool.size();i++)
	{
		initObject = i;

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

	ofstream outFile1("Output\\ObjectPoolClustering.txt");
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

void CClustering::GetArea(vector<int> vecObjectPool,vector<int> &vecInArea)
{
	vector<double> vecJaccardIndex;
	for(int i = 0;i < vecvecObjectPool.size();i++)
	{
		double jaccardIndex = GetJaccardIndex(vecObjectPool, i);
		if(jaccardIndex > paraH)
			vecInArea.push_back(i);
	}
}

double CClustering::GetJaccardIndex(vector<int> vecObjectPool, int n)
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

void CClustering::GetMeanShiftSVector(vector<int> vecInArea, vector<int> &objectPool)
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