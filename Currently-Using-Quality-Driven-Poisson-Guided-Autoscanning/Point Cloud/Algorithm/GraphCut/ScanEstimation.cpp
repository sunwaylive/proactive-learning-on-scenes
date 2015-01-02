#include "GraphCutGlobalHeader.h"
#include "ScanEstimation.h"
#include <wrap/io_trimesh/io_mask.h>
#include "vcg/complex/trimesh/point_sampling.h"
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>
#include "Poisson/Geometry.h"
#include "Poisson/PoissonParam.h"


extern vector<MyPointCloud_RGB_NORMAL> vecPatchPoint;
extern vector<MyPoint> vecPatchCenPoint;
extern vector<vector<bool>> vecvecPatchConnectFlag;
extern vector<vector<int>> vecvecMultiResult;
extern vector<double> vecSmoothValue;
extern vector<double> vecObjectness;
extern vector<double> vecSeparateness;
extern vector<pair<int,int>> vecpairSeperatenessEdge;
extern vector<vector<pair<int,int>>> vecvecpairSeperatenessSmallEdge;


CScanEstimation::CScanEstimation(void)
{
	paraConfidence = 2;
	paraSmoothAdjust = 0.75;
}


CScanEstimation::~CScanEstimation(void)
{
}


void CScanEstimation::MainStep()
{
	//get vecvecIsoPoint
	for(int i = 0;i < vecvecMultiResult.size();i++)
	{
//		runPoissonFieldAndExtractIsoPoints_ByEXE(i);
	}
	
// 	//get confidence score for each patch
// 	for(int i = 0;i < vecPatchPoint.size();i++)
// 	{
// 		double confidenceScore = 0;
// 		for(int j = 0;j < vecObjectIsoPoint[i].objectIsoPoint.size();j++)
// 		{
// 			double distance;
// 			distance = sqrt((vecObjectIsoPoint[i].objectIsoPoint[j].x - vecPatchCenPoint[i].x) * (vecObjectIsoPoint[i].objectIsoPoint[j].x - vecPatchCenPoint[i].x)
// 				+ (vecObjectIsoPoint[i].objectIsoPoint[j].y - vecPatchCenPoint[i].y) * (vecObjectIsoPoint[i].objectIsoPoint[j].y - vecPatchCenPoint[i].y)
// 				+ (vecObjectIsoPoint[i].objectIsoPoint[j].z - vecPatchCenPoint[i].z) * (vecObjectIsoPoint[i].objectIsoPoint[j].z - vecPatchCenPoint[i].z));
// 			double weight;
// 			weight = GaussianFunction(distance);
// 			confidenceScore += weight* vecObjectIsoPoint[i].objectIsoPoint[j].f;
// 		}
// 		vecPatchConfidenceScore.push_back(confidenceScore);
// 	}
// 	
// 	//update the smooth term
// 	for(int i = 0;i < vecpairPatchConnection.size();i++)
// 	{
// 		double confidenceScore;
// 		confidenceScore = vecPatchConfidenceScore[vecpairPatchConnection[i].first]*vecPatchConfidenceScore[vecpairPatchConnection[i].second];
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
// 
// 	ComputeScore();


	//for each seperatenessEdge
// 	for(int i = 0;i < vecvecpairSeperatenessSmallEdge.size();i++)
// 	{
// 		double cutCostDiff = 0;
// 		for(int j = 0;j < vecvecpairSeperatenessSmallEdge[i].size();j++)
// 		{
// 			double confidenceScore = ComputeConfidenceScore(vecvecpairSeperatenessSmallEdge[i][j].first,vecvecpairSeperatenessSmallEdge[i][j].second);  //0--largenum
// 			for(int k = 0;k < vecpairPatchConnection.size();k++)
// 			{
// 				if(vecpairPatchConnection[k].first == vecvecpairSeperatenessSmallEdge[i][j].first && vecpairPatchConnection[k].second == vecvecpairSeperatenessSmallEdge[i][j].second)
// 				{
// 					if(vecGeometryConvex[k])
// 						vecGeometryValue[k] *= confidenceScore;
// 					else
// 						vecGeometryValue[k] /= confidenceScore;
// 
// 					double valueBefore = vecSmoothValue[k];
// 					vecSmoothValue[k] = vecGeometryValue[k] + vecAppearenceValue[k];
// 
// 					double para = 0.3;
// 					vecSmoothValue[k] =  (vecSmoothValue[k] - minSV)/(maxSV - minSV);
// 					vecSmoothValue[k] = paraSmoothAdjust * pow(2.7183,- (1 - vecSmoothValue[k]) * (1 - vecSmoothValue[k]) / para /para);
// 
// 					cutCostDiff += vecSmoothValue[k] - valueBefore;
// 					break;
// 				}	
// 			}
// 		}
// 		vecSeparateness[i] += cutCostDiff;
// 		vecObjectness[vecpairSeperatenessEdge[i].first] += cutCostDiff;
// 		vecObjectness[vecpairSeperatenessEdge[i].second] += cutCostDiff;
// 	}

}

void CScanEstimation::runPoissonFieldAndExtractIsoPoints_ByEXE(int m)
{
	PoissonParam Par;
	Par.Depth = 7;
//	Par.Depth = para->getDouble("Max Depth");

// 	CMesh* target = NULL;
// 	if (para->getBool("Run Poisson On Original"))
// 	{
// 		target = original;
// 	}
// 	else if (para->getBool("Run Poisson On Samples"))
// 	{
// 		target = samples;
// 	}
// 	else
// 	{
// 		cout << "Run on original or sample?" << endl;
// 		return;
// 	}
// 
// 	Timer timer;
// 	timer.start("write ply file");
// 	std::cout<<"sample point num: " << samples->vert.size() <<std::endl;
// 	int mask= tri::io::Mask::IOM_VERTNORMAL;// add vertcord will cause crash
// 	tri::io::ExporterPLY<CMesh>::Save(*target, "poisson_in.ply", mask, false);
// 	timer.end();
// 
// 	timer.start("run Poisson");

// 	int pointNum = 0;
// 	for(int i = 0;i < vecvecMultiResult[m].size();i++)
// 	{
// 		pointNum += vecPatchPoint[vecvecMultiResult[m][i]].mypoints.size();
// 	}
// 
// 	std::ofstream outFile;
// 	outFile.open("poisson_in.ply");
// 	//save m_grid
// 	outFile << "ply\n";
// 	outFile << "format ascii 1.0\n";
// 	outFile << "comment VCGLIB generated\n";
// 	outFile << "element vertex " << pointNum << "\n";
// 	outFile << "property float x\n";
// 	outFile << "property float y\n";
// 	outFile << "property float z\n";
// 
// 	outFile << "property float nx\n";
// 	outFile << "property float ny\n";
// 	outFile << "property float nz\n";
// 
// 	outFile << "element face " << 0 << "\n";
// 	outFile << "property list uchar int vertex_indices\n";
// 	outFile << "end_header\n";
// 
// 	for(int i = 0;i < vecvecMultiResult[m].size();i++)
// 	{
// 		for(int j = 0;j < vecPatchPoint[vecvecMultiResult[m][i]].mypoints.size();j++)
// 		{
// 			MyPoint_RGB_NORMAL point = vecPatchPoint[vecvecMultiResult[m][i]].mypoints[j];
// 			outFile << point.x << " " << point.y << " " << point.z << " " << 
// 				point.normal_x << " " << point.normal_y << " " << point.normal_z<<" " <<  endl;
// 		}
// 	}
// 	outFile.close();
// 
// 	char mycmd[100];
// 	sprintf(mycmd, "PoissonRecon.exe --in poisson_in.ply --out poisson_out.ply --voxel poisson_field.raw --depth %d --pointWeight 0", Par.Depth);
// 	system(mycmd); 
// 
// 	CMesh tentative_mesh;
// 	CMesh* iso_points;
// 
//  	int mask= vcg::tri::io::Mask::IOM_VERTNORMAL ;
// 	int err = vcg::tri::io::Importer<CMesh>::Open(tentative_mesh, "poisson_out.ply", mask);  
// 	if(err) 
// 	{
// 		cout << "Failed reading mesh: " << err << "\n";
// 		return;
// 	}  
// 
// 	if (tentative_mesh.vert.empty())
// 	{
// 		cout << "tentative mesh empty" << endl;
// 		return;
// 	}
// 
// 	iso_points->vert.clear();    //??????????????????
// 
// 	samplePointsFromMesh(tentative_mesh, iso_points);
// 
// 	for (int i = 0; i < iso_points->vert.size(); i++)
// 	{
// 		CVertex& v = iso_points->vert[i];
// 		v.is_iso = true;
// 		v.m_index = i;
// 		v.eigen_confidence = 0;
// 		v.N().Normalize();
// 		v.recompute_m_render();
// 	}
// 	iso_points->vn = iso_points->vert.size();

}

void CScanEstimation::samplePointsFromMesh(CMesh& mesh, CMesh* points)
{
// 	mesh.bbox.SetNull();
// 	for (int i = 0; i < mesh.vert.size(); i++)
// 	{
// 		mesh.bbox.Add(mesh.vert[i]);
// 	}
// 	mesh.vn = mesh.vert.size();
// 	mesh.fn = mesh.face.size();
// 	vcg::tri::UpdateNormals<CMesh>::PerVertex(mesh);
// 
// 	float radius = 0;
// // 	int sampleNum = para->getDouble("Poisson Disk Sample Number");	//??????????????????
// // 	if (sampleNum <= 100)
// // 	{
// 	int	sampleNum = 100;
// //	}
// 	radius = vcg::tri::SurfaceSampling<CMesh,BaseSampler>::ComputePoissonDiskRadius(mesh, sampleNum);
// 	// first of all generate montecarlo samples for fast lookup
// 	CMesh *presampledMesh=&(mesh);
// 	CMesh MontecarloMesh; // this mesh is used only if we need real poisson sampling (and therefore we need to choose points different from the starting mesh vertices)
// 
// 	BaseSampler sampler(&MontecarloMesh);
// 	sampler.qualitySampling =true;
// 	vcg::tri::SurfaceSampling<CMesh,BaseSampler>::Montecarlo(mesh, sampler, sampleNum*20);
// 	MontecarloMesh.bbox = mesh.bbox; // we want the same bounding box
// 	presampledMesh=&MontecarloMesh;
// 
// 	BaseSampler mps(points);
// 	vcg::tri::SurfaceSampling<CMesh,BaseSampler>::PoissonDiskParam pp;
// 	vcg::tri::SurfaceSampling<CMesh,BaseSampler>::PoissonDisk(mesh, mps, *presampledMesh, radius,pp);

	
	
}



void CScanEstimation::runComputeIsoGradientConfidence()
{
// 	if (para->getBool("Use Confidence 1"))
// 	{
// 		for (int i = 0; i < iso_points->vn; i++)
// 		{
// 			iso_points->vert[i].eigen_confidence = 0.5;
// 		}
// 		return;
// 	}
// 
// 	if (para->getBool("Use Confidence 4"))
// 	{
// 		GlobalFun::normalizeConfidence(iso_points->vert, 0);
// 	}
// 
// 	vector<float> confidences_temp;
// 	iso_points->vn = iso_points->vert.size();
// 	for (int i = 0; i < iso_points->vn; i++)
// 	{
// 		confidences_temp.push_back(iso_points->vert[i].eigen_confidence);
// 	}
// 
// 	if (field_points->vert.empty())
// 	{
// 		cout << "need field points" << endl;
// 		return;
// 	}
// 	else
// 	{
// 		cout << "field points: " << field_points->vert.size() << endl;
// 	}
// 
// 	double radius = para->getDouble("CGrid Radius");
// 	double radius2 = radius * radius;
// 	double iradius16 = -4.0 / radius2;
// 
// 	double sigma = global_paraMgr.norSmooth.getDouble("Sharpe Feature Bandwidth Sigma");
// 	//double sigma = 35;
// 	double sigma_threshold = pow(max(1e-8,1-cos(sigma/180.0*3.1415926)), 2);
// 
// 	GlobalFun::computeBallNeighbors(iso_points, field_points, 
// 		radius, field_points->bbox);
// 
// 	for (int i = 0; i < iso_points->vn; i++)
// 	{
// 		CVertex& v = iso_points->vert[i];
// 
// 		float positive_sum = 0.0;
// 		float negative_sum = 0.0;
// 		float positive_w_sum = 0.0;
// 		float negative_w_sum = 0.0;
// 
// 		for (int j = 0; j < v.original_neighbors.size(); j++)
// 		{
// 			int index = v.original_neighbors[j];
// 			CVertex& t = field_points->vert[index];
// 
// 			//cout << t.eigen_confidence << endl;
// 
// 			Point3f diff = t.P() - v.P();
// 			Point3f vn = v.N();
// 			float proj = diff * v.N();
// 
// 			float dist2  = diff.SquaredNorm();
// 			float w1 = exp(dist2 * iradius16);
// 			float w2 = 1.0;
// 
// 			if (proj > 0)
// 			{
// 				w2 = exp(-pow(1-vn*diff.Normalize(), 2)/sigma_threshold); 
// 			}
// 			else
// 			{
// 				vn *= -1;
// 				w2 = exp(-pow(1-vn*diff.Normalize(), 2)/sigma_threshold); 
// 			} 
// 
// 			float w = w1 * w2;
// 
// 			if (proj > 0)
// 			{
// 				positive_sum += w * t.eigen_confidence;
// 				positive_w_sum += w;
// 			}
// 			else
// 			{
// 				negative_sum += w * t.eigen_confidence;
// 				negative_w_sum += w;
// 			}
// 		}
// 
// 		if (positive_w_sum > 0 && negative_w_sum > 0)
// 		{
// 			v.eigen_confidence = abs(positive_sum / positive_w_sum - negative_sum / negative_w_sum);
// 			//cout << v.eigen_confidence << endl;
// 		}
// 		else
// 		{
// 			v.eigen_confidence = 1e-6;
// 		}
// 	}
// 	GlobalFun::normalizeConfidence(iso_points->vert, 0);
// 
// 	if (para->getBool("Use Confidence 4"))
// 	{
// 		for (int i = 0; i < iso_points->vn; i++)
// 		{
// 			CVertex& v = iso_points->vert[i];
// 			//file2 << v.eigen_confidence << endl;
// 			float temp_confidence = confidences_temp[i];
// 			v.eigen_confidence *= temp_confidence;
// 		}
// 
// 		GlobalFun::normalizeConfidence(iso_points->vert, 0);
// 	}
// 	//if (para->getBool("Use Confidence 4"))
// 	//{
// 	//  if (para->getBool("Use Confidence 3"))
// 	//  {
// 
// 	//    for (int i = 0; i < iso_points->vn; i++)
// 	//    {
// 	//      CVertex& v = iso_points->vert[i];
// 	//      //file2 << v.eigen_confidence << endl;
// 	//      float temp_confidence = confidences_temp[i];
// 	//      float combine_confidence = std::sqrt(temp_confidence * temp_confidence 
// 	//                                 + v.eigen_confidence * v.eigen_confidence);
// 	//      v.eigen_confidence = combine_confidence;
// 	//    }
// 
// 	//    normalizeConfidence(iso_points->vert, 0);
// 
// 	//  }
// 	//  else
// 	//  {
// 	//    for (int i = 0; i < iso_points->vn; i++)
// 	//    {
// 	//      CVertex& v = iso_points->vert[i];
// 	//      //file2 << v.eigen_confidence << endl;
// 	//      float temp_confidence = confidences_temp[i];
// 	//      v.eigen_confidence *= temp_confidence;
// 	//    }
// 
// 	//    normalizeConfidence(iso_points->vert, 0);
// 	//  }
// 
// 	//  for (int i = 0; i < iso_points->vn; i++)
// 	//  {
// 	//    CVertex& v = iso_points->vert[i];
// 	//    //file3 << v.eigen_confidence << endl;
// 	//  }
// 	//}
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
}

void CScanEstimation::ComputeObjectness(int m)
{
	vecObjectness.clear();

	double objectness = 0;
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
		vector<int> vecDisconnectPatchAfter;
		for(int j = 0;j < vecConnectPatchBefore.size();j++)
		{
			int connectPatchIndex = vecConnectPatchBefore[j];
			bool exsitFlag = false;
			for(int k = 0;k < vecvecMultiResult[m].size();k++)
			{
				if(i == k)
					continue;
				if(connectPatchIndex == vecvecMultiResult[m][k])
					exsitFlag = true;
			}
			if(!exsitFlag)
				vecDisconnectPatchAfter.push_back(connectPatchIndex);
		}

		for(int j = 0;j < vecDisconnectPatchAfter.size();j++)
		{
			for(int k = 0;k < vecpairPatchConnection.size();k++)
			{
				if(vecpairPatchConnection[k].first == patchIndex && vecpairPatchConnection[k].second == vecDisconnectPatchAfter[j])
				{
					objectness += vecSmoothValue[k];
				}	
			}
		}
	}
	vecObjectness.push_back(objectness);

	ofstream outFile("Output\\Objectness1.txt",ios::app);
	outFile << objectness <<  endl;
	outFile.close();
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

		ofstream outFile("Output\\Separateness1.txt",ios::app);
		outFile << separateness<< endl;
		outFile.close();
	}

}

double CScanEstimation::GaussianFunction(double x)
{
	double para = 0.05;
	double value = pow(2.7183,- x * x / para / para);

	return value;
}
