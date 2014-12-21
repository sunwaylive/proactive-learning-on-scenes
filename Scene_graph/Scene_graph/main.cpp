#include "visualizer.h"
#include "color_op.h"
#include "file_io.h"
#include "scene_seg.h"


vector<MyPointCloud_RGB> patch_clouds;
vector<MyPoint> vecPatchCenPoint;
vector<ColorType> vecPatchColor;
vector<Normal> vecPatcNormal;
vector<pair<int,int>> vecpairPatchConnection;

double GetMinDisBetPatch(int m,int n)
{
  double minDis=99999999;
  for(int i = 0;i < patch_clouds[m].mypoints.size();i++)
  {
    for(int j = 0;j < patch_clouds[n].mypoints.size();j++)
    {
      double dis = sqrt(pow(patch_clouds[m].mypoints[i].x-patch_clouds[n].mypoints[j].x,2)
        + pow(patch_clouds[m].mypoints[i].y-patch_clouds[n].mypoints[j].y,2)
        + pow(patch_clouds[m].mypoints[i].z-patch_clouds[n].mypoints[j].z,2));
      if(minDis > dis)	
        minDis = dis;
    }
  }
  return minDis;
}

double GetCenDisBetPatch(int m,int n)
{
  double dis =  sqrt(pow(vecPatchCenPoint[m].x-vecPatchCenPoint[n].x,2)
    + pow(vecPatchCenPoint[m].y-vecPatchCenPoint[n].y,2)
    + pow(vecPatchCenPoint[m].z-vecPatchCenPoint[n].z,2));
  return dis;
}

double GetBinaryDataValue(double d)
{
  // 	double w,o;
  // 	o = 0.1;
  // 	w = exp(-pow(d/1,2));
  // 	return w;
  double penaltyValue;
  penaltyValue = 0.5 + 0.2 * d;
  return penaltyValue;
}

double GetBinarySmoothValue(int m,int n)
{
  double smoothValue,geometryValue,appearenceValue;

  //考虑法向量
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
  if(convexFlag)
  {
    geometryValue = 0.1 * cosValue + 0.9;
  }
  else
  {
    geometryValue = 2 * cosValue;
  }

  //考虑颜色
  appearenceValue = (vecPatchColor[m].mRed - vecPatchColor[n].mRed) 
    +(vecPatchColor[m].mGreen- vecPatchColor[n].mGreen)
    +(vecPatchColor[m].mBlue - vecPatchColor[n].mBlue);
  appearenceValue /= 256 * 3;

  smoothValue = geometryValue + appearenceValue;
  return smoothValue;
}  

int main (int argc, char *argv[])
{
  Visualizer vs;
  vs.viewer->removeAllPointClouds();
  vs.viewer->removeCoordinateSystem();
  vs.viewer->setBackgroundColor(0,0,0);

  PointCloudPtr_RGB_NORMAL cloud(new PointCloud_RGB_NORMAL);
  //loadPointCloud_normal_ply("data/big_table_normal.ply", cloud, normals);
  loadPointCloud_normal_ply("data/small_normal.ply", cloud);

  PointCloudPtr_RGB_NORMAL cloud_mark(new PointCloud_RGB_NORMAL);
  pcl::copyPointCloud(*cloud,*cloud_mark);

  /******************detect table************************/
  PointCloudPtr_RGB_NORMAL tabletopCloud(new PointCloud_RGB_NORMAL());
  PointCloudPtr_RGB_NORMAL planeCloud(new PointCloud_RGB_NORMAL());
  //detect_table_plane_r(cloud, planeCloud, tabletopCloud);
  detect_table_plane(cloud, planeCloud, tabletopCloud);
  //showPointClound(planeCloud,"planeCloud");

  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  detect_table(cloud, coefficients_plane, inliers_plane);

  PointCloudPtr_RGB_NORMAL table_cloud(new PointCloud_RGB_NORMAL());

  pcl::ExtractIndices<Point_RGB_NORMAL> extract0;// Create the filtering object
  // Extract the inliers
  extract0.setInputCloud (cloud);
  extract0.setIndices (inliers_plane);
  extract0.setNegative (false);
  extract0.filter (*table_cloud);

  PointCloudPtr_RGB pc(new PointCloud_RGB);
  
  for(int i=0;i<table_cloud->size();i++){
    Point_RGB pr;
    pr.x=table_cloud->at(i).x;
    pr.y=table_cloud->at(i).y;
    pr.z=table_cloud->at(i).z;
    pr.r=table_cloud->at(i).r;
    pr.g=table_cloud->at(i).g;
    pr.b=table_cloud->at(i).b;
    pc->push_back(pr);
  }

  //showPointCloud (pc,"table_cloud");

  cout<<"pc->size()================"<<pc->size()<<endl;

  vs.viewer->addPointCloud (pc, "table_cloud");

  //cv::Point2f p0;
  //cv::Point2f p1;
  //cv::Point2f p2;
  //cv::Point2f p3;

  //find_min_rect(table_cloud, p0, p1, p2, p3);

  ///******************Euclidean Cluster Extraction************************/
  //std::vector<PointCloudPtr_RGB> cluster_points;

  //object_seg_ECE(tabletopCloud, cluster_points);

  //PointCloudPtr_RGB remaining_cloud(new PointCloud_RGB());

  //for(int i=0;i<cluster_points.size();i++){
  //  vector<MyPointCloud_RGB> plane_clouds;
  //  vector<MyPointCloud> rect_clouds;
  //  vector<MyPointCloud_RGB> cylinder_clouds;
  //  vector<MyPointCloud_RGB> sphere_clouds;

  //  object_fitting(cluster_points.at(i), plane_clouds, rect_clouds, cylinder_clouds, sphere_clouds, remaining_cloud);

  //  //color cloud
  //  std::vector<ColorType> colors;
  //  std::vector<ColorType> excludeColors;
  //  excludeColors.push_back(ColorType(0,0,0));
  //  excludeColors.push_back(ColorType(255,255,255));
  //  int sum_num=cylinder_clouds.size()+sphere_clouds.size()+plane_clouds.size();
  //  if(sum_num>0){
  //    //getUniqueColors(sum_num, colors, excludeColors);
  //    getDiffColors(sum_num, colors);
  //  }

  //  cout<<"sum_num:"<<sum_num<<endl;
  //  cout<<"colors.size:"<<colors.size()<<endl;

  //  int index=0;

  //  for(int m=0;m<cylinder_clouds.size();m++){
  //     
  //    PointCloudPtr_RGB cloud_in_cylinder(new PointCloud_RGB());
  //    MyPointCloud_RGB2PointCloud(cylinder_clouds.at(m), cloud_in_cylinder);

  //    for(int n=0;n<cloud_in_cylinder->size();n++){
  //      cloud_in_cylinder->at(n).r=colors.at(index).mRed;
  //      cloud_in_cylinder->at(n).g=colors.at(index).mGreen;
  //      cloud_in_cylinder->at(n).b=colors.at(index).mBlue;

  //      /*cloud_in_cylinder->at(n).r=255;
  //      cloud_in_cylinder->at(n).g=255;
  //      cloud_in_cylinder->at(n).b=255;*/
  //    }

  //    index++;

  //    std::stringstream str;
  //    str<<"cloud_in_cylinder"<<i<<m;
  //    std::string id_pc=str.str();

  //    vs.viewer->addPointCloud (cloud_in_cylinder, id_pc);
  //  }

  //  for(int m=0;m<sphere_clouds.size();m++){
  //    PointCloudPtr_RGB cloud_in_sphere(new PointCloud_RGB());
  //    MyPointCloud_RGB2PointCloud(sphere_clouds.at(m), cloud_in_sphere);

  //    for(int n=0;n<cloud_in_sphere->size();n++){
  //      cloud_in_sphere->at(n).r=colors.at(index).mRed;
  //      cloud_in_sphere->at(n).g=colors.at(index).mGreen;
  //      cloud_in_sphere->at(n).b=colors.at(index).mBlue;

  //      /*cloud_in_sphere->at(n).r=0;
  //      cloud_in_sphere->at(n).g=255;
  //      cloud_in_sphere->at(n).b=0;*/
  //    }

  //    index++;

  //    std::stringstream str;
  //    str<<"cloud_in_sphere"<<i<<m;
  //    std::string id_pc=str.str();

  //    vs.viewer->addPointCloud (cloud_in_sphere, id_pc);
  //  }

  //  for(int m=0;m<plane_clouds.size();m++){
  //    PointCloudPtr_RGB cloud_in_plane(new PointCloud_RGB());
  //    MyPointCloud_RGB2PointCloud(plane_clouds.at(m), cloud_in_plane);

  //    for(int n=0;n<cloud_in_plane->size();n++){
  //      cloud_in_plane->at(n).r=colors.at(index).mRed;
  //      cloud_in_plane->at(n).g=colors.at(index).mGreen;
  //      cloud_in_plane->at(n).b=colors.at(index).mBlue;

  //      /*cloud_in_plane->at(n).r=0;
  //      cloud_in_plane->at(n).g=0;
  //      cloud_in_plane->at(n).b=255;*/
  //    }

  //    //cout<<"colors.at(index).mRed:"<<colors.at(index).mRed<<" "<<"colors.at(index).mGreen:"<<colors.at(index).mGreen<<" "<<"colors.at(index).mBlue:"<<colors.at(index).mBlue<<endl;

  //    index++;

  //    std::stringstream str;
  //    str<<"cloud_in_plane"<<i<<m;
  //    std::string id_pc=str.str();

  //    vs.viewer->addPointCloud (cloud_in_plane, id_pc);
  //  }

  //  //for remaining clouds, do over-segmentation
  //  float voxel_resolution = 0.008f;
  //  float seed_resolution = 0.08f;
  //  float color_importance = 0.2f;
  //  float spatial_importance = 0.4f;
  //  float normal_importance = 1.0f;

  //  PointCloudT::Ptr colored_cloud(new PointCloudT);
  //  vector<MyPointCloud_RGB> patch_clouds;
  //  VCCS_over_segmentation(remaining_cloud,normals,voxel_resolution,seed_resolution,color_importance,spatial_importance,normal_importance,patch_clouds,colored_cloud);

  //  std::stringstream str;
  //  str<<"colored_voxel_cloud"<<i;
  //  std::string id_pc=str.str();

  //  vs.viewer->addPointCloud (colored_cloud, id_pc);

  //  cout<<"rect_clouds.size()==========================================:"<<rect_clouds.size()<<endl;

  //  /*for(int k=0;k<rect_clouds.size();k++){
  //    PointCloudPtr pc(new PointCloud);
  //    MyPointCloud2PointCloud(rect_clouds.at(k), pc);

  //    std::stringstream st0;
  //    std::stringstream st1;
  //    std::stringstream st2;
  //    std::stringstream st3;

  //    st0<<"a"<<i<<k<<"0";
  //    st1<<"a"<<i<<k<<"1";
  //    st2<<"a"<<i<<k<<"2";
  //    st3<<"a"<<i<<k<<"3";

  //    std::string id_line0=st0.str();
  //    std::string id_line1=st1.str();
  //    std::string id_line2=st2.str();
  //    std::string id_line3=st3.str();

  //    vs.viewer->addLine(pc->at(0),pc->at(1),255,0,0,id_line0);
  //    vs.viewer->addLine(pc->at(1),pc->at(2),255,0,0,id_line1);
  //    vs.viewer->addLine(pc->at(2),pc->at(3),255,0,0,id_line2);
  //    vs.viewer->addLine(pc->at(3),pc->at(0),255,0,0,id_line3);
  //  }*/
  //}

  float voxel_resolution = 0.004f;
  float seed_resolution = 0.06f;
  float color_importance = 0.2f;
  float spatial_importance = 0.4f;
  float normal_importance = 1.0f;

  /******************Euclidean Cluster Extraction************************/
  std::vector<PointCloudPtr_RGB_NORMAL> cluster_points;

  object_seg_ECE(tabletopCloud, cluster_points);

  for(int i=0;i<cluster_points.size();i++){

    PointCloudT::Ptr colored_cloud(new PointCloudT);
    vector<MyPointCloud_RGB> patch_clouds;
    PointNCloudT::Ptr normal_cloud(new PointNCloudT);
    VCCS_over_segmentation(cluster_points.at(i),voxel_resolution,seed_resolution,color_importance,spatial_importance,normal_importance,patch_clouds,colored_cloud,normal_cloud);

    std::stringstream str;
    str<<"colored_voxel_cloud"<<i;
    std::string id_pc=str.str();

    vs.viewer->addPointCloud (colored_cloud, id_pc);

    str<<"supervoxel_normals"<<i;
    id_pc=str.str();
    vs.viewer->addPointCloudNormals<pcl::PointNormal> (normal_cloud,1,0.05f, id_pc);

    /*cout<<"patch_clouds.size():"<<patch_clouds.size()<<endl;
    cout<<"normal_cloud->size():"<<normal_cloud->size()<<endl;
    PointNCloudT::Ptr normal_cloud_tem(new PointNCloudT);
    for(int j=0;j<patch_clouds.size();j++){
    normal_cloud_tem->push_back(normal_cloud->at(j));

    PointCloudPtr_RGB pc(new PointCloud_RGB);
    MyPointCloud_RGB2PointCloud(patch_clouds.at(j), pc);

    std::stringstream str;
    str<<"patch_clouds"<<i<<j;
    std::string id_pc=str.str();

    vs.viewer->addPointCloud (pc, id_pc);
    }

    std::stringstream str;
    str<<"supervoxel_normals"<<i;
    std::string id_pc=str.str();
    vs.viewer->addPointCloudNormals<pcl::PointNormal> (normal_cloud_tem,1,0.05f, id_pc);*/
  }

  /******************Graph Pre All************************/
  //   for(int i = 0;i <patch_clouds.size();i++)
  //   {
  // 	  ColorType color;
  // 	  MyPoint point;
  // 	  color.mRed = color.mGreen = color.mBlue = 0;
  // 	  point.x = point.y = point.z =0;
  // 
  // 	  for(int j = 0;j < patch_clouds[i].mypoints.size();j++)
  // 	  {
  // 		  color.mRed += patch_clouds[i].mypoints[j].r;
  // 		  color.mGreen += patch_clouds[i].mypoints[j].g;
  // 		  color.mBlue += patch_clouds[i].mypoints[j].b;
  // 		  point.x += patch_clouds[i].mypoints[j].x;
  // 		  point.y += patch_clouds[i].mypoints[j].y;
  // 		  point.z += patch_clouds[i].mypoints[j].z;
  // 	  }//
  // 	  if(patch_clouds[i].mypoints.size() > 0)
  // 	  {
  // 		  color.mRed /= patch_clouds[i].mypoints.size();
  // 		  color.mGreen /= patch_clouds[i].mypoints.size();
  // 		  color.mBlue /= patch_clouds[i].mypoints.size();
  // 		  point.x/= patch_clouds[i].mypoints.size();
  // 		  point.y /= patch_clouds[i].mypoints.size();
  // 		  point.z /= patch_clouds[i].mypoints.size();
  // 	  }
  // 	  
  // 	  vecPatchColor.push_back(color);
  // 	  vecPatchCenPoint.push_back(point);
  // 
  // 	  Normal nor;
  // 	  nor.normal_x = nor.normal_y = nor.normal_z = 0.577;
  // 	  vecPatcNormal.push_back(nor);
  //   }
  // 
  //   vector<vector<double>> vecvecPatchMinDis;
  //   vector<vector<double>> vecvecPatchCenDis;
  //   pair<int,int> pairPatchConnection;
  // 
  //   vecvecPatchMinDis.resize(patch_clouds.size());
  //   vecvecPatchCenDis.resize(patch_clouds.size());
  //   for(int i = 0;i <patch_clouds.size();i++)
  //   {
  // 	  vecvecPatchMinDis[i].resize(patch_clouds.size());
  // 	  vecvecPatchCenDis[i].resize(patch_clouds.size());
  //   }
  // 
  //   for(int i = 0;i <patch_clouds.size();i++)
  // 	  for(int j = 0;j <patch_clouds.size();j++)
  // 		  if(i != j)
  // 		  {
  // 			  vecvecPatchMinDis[i][j] = GetMinDisBetPatch(i,j);
  // 			  vecvecPatchCenDis[i][j] = GetCenDisBetPatch(i,j);
  // 			  pairPatchConnection.first = i;
  // 			  pairPatchConnection.second = j;
  // 			  if(vecvecPatchMinDis[i][j]<0.1)
  // 				 vecpairPatchConnection.push_back(pairPatchConnection);
  // 		  }
  // 		  else
  // 		  {
  // 			  vecvecPatchMinDis[i][j] = 0;
  // 			  vecvecPatchCenDis[i][j] = 0;
  // 		  }
  // 
  // 	
  //    /******************Graph Pre One************************/
  // 	int m=0; 
  // 	vector<double> vecDataValue;
  // 	vector<double> vecSmoothValue;
  // 	vector<pair<int,int>> verpairSmoothVertex;
  // 	pair<int,int> pairSmoothVertex;
  // 
  // 	for(int i = 0;i <patch_clouds.size();i++)
  // 	{
  // 		vecDataValue.push_back(GetBinaryDataValue(vecvecPatchMinDis[m][i]));
  // 	}
  // 
  // 	for(int i = 0;i <vecpairPatchConnection.size();i++)
  // 	{
  // 		vecSmoothValue.push_back(GetBinarySmoothValue(vecpairPatchConnection[i].first,vecpairPatchConnection[i].second));
  // 	}
  // 
  // 
  //   /******************Graph Cut************************/
  //   typedef Graph<int,int,int> GraphType;
  //   GraphType *g = new GraphType(/*estimated # of nodes*/ patch_clouds.size(), /*estimated # of edges*/ vecpairPatchConnection.size()); 
  // 
  //   g -> add_node(patch_clouds.size()); 
  // 
  //   for(int i = 0;i <vecDataValue.size();i++)
  //   {
  // 	  g -> add_tweights( i,   /* capacities */  0, vecDataValue[i]);
  //   }
  // 
  //   for(int i = 0;i <vecSmoothValue.size();i++)
  //   {
  // 	  g -> add_edge( vecpairPatchConnection[i].first,vecpairPatchConnection[i].second,    /* capacities */  vecSmoothValue[i], vecSmoothValue[i]);
  //   }
  // 
  //   int flow = g -> maxflow();
  // 
  //   printf("Flow = %d\n", flow);
  //   printf("Minimum cut:\n");
  // 
  //   int countSOURCE,countSINK;
  //   countSINK = countSOURCE =0;
  //   for(int i=0;i<patch_clouds.size();i++)
  //   if (g->what_segment(i) == GraphType::SOURCE)
  // 	  countSOURCE++;
  //   else
  // 	  countSINK++;
  // 
  //   printf("node0 is in the SOURCE set %d,%d\n",countSOURCE,countSINK);
  // 
  //   delete g;

  vs.show();

  return 0;
}
