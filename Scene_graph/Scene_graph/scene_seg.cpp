#include <iostream>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include "pcl/filters/extract_indices.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/common/transforms.h>
#include <math.h>

#include "scene_seg.h"
#include "Wm5IntrTriangle3Triangle3.h"
#include "Wm5IntrTriangle3Sphere3.h"
#include "Wm5IntrTriangle3Cylinder3.h"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "supervoxel_clustering.h"

#include <pcl/octree/octree.h>

#define PI 3.1415926535

using namespace std;
using namespace Wm5;

//for least square
bool getPlaneByLeastSquare(PointCloudPtr_RGB cloud_all_in_plane, pcl::ModelCoefficients::Ptr coefficients)
{
  double coeffA = 0.0, coeffB = 0.0, coeffC = 0.0, coeffD = 0.0;
  int matrixSize = 3;
  Eigen::Matrix3Xd mpara( matrixSize, matrixSize );
  for (int i =0; i<matrixSize;i++){
    for (int j=0; j<matrixSize;j++){
      mpara(i,j) = 0;
    }
  }

  double sumx, sumy, sumz;
  double averx, avery, averz;

  sumx=sumy=sumz=0;

  for(int i=0;i<cloud_all_in_plane->points.size();i++){
    sumx+=cloud_all_in_plane->points.at(i).x;
    sumy+=cloud_all_in_plane->points.at(i).y;
    sumz+=cloud_all_in_plane->points.at(i).z;
  }

  averx=sumx/cloud_all_in_plane->points.size();
  avery=sumy/cloud_all_in_plane->points.size();
  averz=sumz/cloud_all_in_plane->points.size();

  for(int i=0;i<cloud_all_in_plane->points.size();i++){
    mpara( 0, 0 ) += pow( cloud_all_in_plane->points.at(i).x - averx, 2 );
    mpara( 0, 1 ) += ( cloud_all_in_plane->points.at(i).x - averx ) * ( cloud_all_in_plane->points.at(i).y - avery );
    mpara( 0, 2 ) += ( cloud_all_in_plane->points.at(i).x - averx ) * ( cloud_all_in_plane->points.at(i).z - averz );

    mpara( 1, 0 ) += ( cloud_all_in_plane->points.at(i).x - averx ) * ( cloud_all_in_plane->points.at(i).y - avery );
    mpara( 1, 1 ) += pow( cloud_all_in_plane->points.at(i).y - avery, 2 );
    mpara( 1, 2 ) += ( cloud_all_in_plane->points.at(i).y - avery ) * ( cloud_all_in_plane->points.at(i).z - averz );

    mpara( 2, 0 ) += ( cloud_all_in_plane->points.at(i).x - averx ) * ( cloud_all_in_plane->points.at(i).z - averz );
    mpara( 2, 1 ) += ( cloud_all_in_plane->points.at(i).y - avery ) * ( cloud_all_in_plane->points.at(i).z - averz );
    mpara( 2, 2 ) += pow( cloud_all_in_plane->points.at(i).z - averz, 2 );
  }

  Eigen::EigenSolver<Eigen::Matrix3Xd> msolver( mpara );
  complex<double> lambda1 = msolver.eigenvalues()[0];
  complex<double> lambda2 = msolver.eigenvalues()[1];
  complex<double> lambda3 = msolver.eigenvalues()[2];
  int minEigenValue = (( lambda1.real() < lambda2.real() ) ? 0 :1 );
  minEigenValue = (( msolver.eigenvalues()[minEigenValue].real() < lambda3.real() )? minEigenValue : 2);
  coeffA = msolver.eigenvectors().col(minEigenValue)[0].real();
  coeffB = msolver.eigenvectors().col(minEigenValue)[1].real();
  coeffC = msolver.eigenvectors().col(minEigenValue)[2].real();
  coeffD = -( coeffA * averx + coeffB * avery + coeffC * averz );

  cout<<endl;
  cout<<coeffA<<"==========="<<coeffB<<"=============="<<coeffC<<"============"<<coeffD<<endl;

  coefficients->values.push_back(coeffA);
  coefficients->values.push_back(coeffB);
  coefficients->values.push_back(coeffC);
  coefficients->values.push_back(coeffD);

  return true;
}



//show rgb cloud
void showPointCloud (PointCloudPtr_RGB cloud,std::string name)
{
  pcl::visualization::CloudViewer viewer (name);

  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {

  }
}

//show cloud
void showPointCloud2 (PointCloudPtr cloud,std::string name)
{
  pcl::visualization::CloudViewer viewer (name);

  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {

  }
}

//compute bounding box
void com_bounding_box(PointCloudPtr_RGB cloud,float *min_x,float *min_y,float *min_z, float *max_x, float *max_y, float *max_z){
  *min_x=cloud->points[0].x;
  *min_y=cloud->points[0].y;
  *min_z=cloud->points[0].z;
  *max_x=cloud->points[0].x;
  *max_y=cloud->points[0].y;
  *max_z=cloud->points[0].z;

  for (int i=0; i<cloud->size(); ++i) {
    float x, y, z;
    x=cloud->points[i].x;
    y=cloud->points[i].y;
    z=cloud->points[i].z;

    if(x<(*min_x)){
      (*min_x)=x;
    }
    else if(x>(*max_x)){
      (*max_x)=x;
    }

    if(y<(*min_y)){
      (*min_y)=y;
    }
    else if(y>(*max_y)){
      (*max_y)=y;
    }

    if(z<(*min_z)){
      (*min_z)=z;
    }
    else if(z>(*max_z)){
      (*max_z)=z;
    }
  }
}

//compute max value, min value, and average value along z axis of the point cloud
void com_max_and_min_and_avg_z(PointCloudPtr_RGB cloud,float *min_z,float *max_z,float *avg_z){
  *min_z=cloud->points[0].z;
  *max_z=cloud->points[0].z;

  float sum_z=0;

  for (int i=0; i<cloud->size(); ++i) {
    float z;

    z=cloud->points[i].z;

    sum_z+=z;

    if(z<(*min_z)){
      (*min_z)=z;
    }
    else if(z>(*max_z)){
      (*max_z)=z;
    }
  }

  *avg_z=sum_z/cloud->size();
}

//append a cloud to another cloud
void appendCloud_RGB(PointCloudPtr_RGB sourceCloud,PointCloudPtr_RGB targetCloud){
  for(int i=0;i<sourceCloud->size();i++){
    targetCloud->push_back(sourceCloud->at(i));
  }
}

//append a cloud to another cloud
void appendCloud(PointCloudPtr sourceCloud,PointCloudPtr targetCloud){
  for(int i=0;i<sourceCloud->size();i++){
    targetCloud->push_back(sourceCloud->at(i));
  }
}

//get rotation matrix
void getRotationMatrix(Eigen::Vector3d &axis, double angleArc, Eigen::Matrix4d &matrix)
{
  axis.normalize();

  matrix(0,0) = cos(angleArc)+(1-cos(angleArc))*axis(0)*axis(0) ;
  matrix(1,0) = (1-cos(angleArc))*axis(0)*axis(1) + sin(angleArc)*axis(2);
  matrix(2,0) = (1-cos(angleArc))*axis(0)*axis(2)-sin(angleArc)*axis(1);
  matrix(3,0) = 0;

  matrix(0,1) = (1-cos(angleArc))*axis(0)*axis(1) -sin(angleArc)*axis(2);
  matrix(1,1) = cos(angleArc)+(1-cos(angleArc))*axis(1)*axis(1);
  matrix(2,1) = (1-cos(angleArc))*axis(2)*axis(1) + sin(angleArc)*axis(0);
  matrix(3,1) = 0;

  matrix(0,2) = (1-cos(angleArc))*axis(2)*axis(0) + sin(angleArc)*axis(1);
  matrix(1,2) = (1-cos(angleArc))*axis(2)*axis(1) - sin(angleArc)*axis(0);
  matrix(2,2) = cos(angleArc) + (1-cos(angleArc))*axis(2)*axis(2);
  matrix(3,2) = 0;

  matrix(0,3) = 0;
  matrix(1,3) = 0;
  matrix(2,3) = 0;
  matrix(3,3) = 1;
}

//find a minimum bounding rect
void find_min_rect(PointCloudPtr_RGB cloud, cv::Point2f &p0,cv::Point2f &p1,cv::Point2f &p2,cv::Point2f &p3){
  std::vector<cv::Point2f> points_clu_2d;

  for(int j=0;j<cloud->points.size();j++){
    points_clu_2d.push_back(cv::Point2f(cloud->points[j].x, cloud->points[j].y));
  }

  cv::RotatedRect rect = cv::minAreaRect(cv::Mat(points_clu_2d));

  float width= rect.size.width;
  float height= rect.size.height;

  p0.x=rect.center.x-width/2.0;
  p0.y=rect.center.y-height/2.0;

  p1.x=rect.center.x-width/2.0;
  p1.y=rect.center.y+height/2.0;

  p2.x=rect.center.x+width/2.0;
  p2.y=rect.center.y+height/2.0;

  p3.x=rect.center.x+width/2.0;
  p3.y=rect.center.y-height/2.0;

  float ang=(rect.angle/180.0)*PI;

  float x0=rect.center.x+(p0.x-rect.center.x)*cos(ang)-(p0.y-rect.center.y)*sin(ang);
  float y0=rect.center.y+(p0.x-rect.center.x)*sin(ang)+(p0.y-rect.center.y)*cos(ang);

  float x1=rect.center.x+(p1.x-rect.center.x)*cos(ang)-(p1.y-rect.center.y)*sin(ang);
  float y1=rect.center.y+(p1.x-rect.center.x)*sin(ang)+(p1.y-rect.center.y)*cos(ang);

  float x2=rect.center.x+(p2.x-rect.center.x)*cos(ang)-(p2.y-rect.center.y)*sin(ang);
  float y2=rect.center.y+(p2.x-rect.center.x)*sin(ang)+(p2.y-rect.center.y)*cos(ang);

  float x3=rect.center.x+(p3.x-rect.center.x)*cos(ang)-(p3.y-rect.center.y)*sin(ang);
  float y3=rect.center.y+(p3.x-rect.center.x)*sin(ang)+(p3.y-rect.center.y)*cos(ang);

  p0.x=x0;
  p0.y=y0;

  p1.x=x1;
  p1.y=y1;

  p2.x=x2;
  p2.y=y2;

  p3.x=x3;
  p3.y=y3;
}

//pcl pointCloud pop up
void pointCloudPopUp(PointCloudPtr_RGB cloud){
  PointCloudPtr_RGB pc(new PointCloud_RGB);

  for(int i=0;i<cloud->size()-1;i++){
    pc->push_back(cloud->at(i));
  }

  cloud->clear();

  pcl::copyPointCloud(*pc,*cloud);
}

//get Rect For PlaneCloud
void getRectForPlaneCloud(PointCloudPtr_RGB plane_cloud, pcl::ModelCoefficients::Ptr plane_coefficients, PointCloudPtr rect_cloud){
  PointCloudPtr_RGB cloud_in_plane(new PointCloud_RGB);

  PointCloudPtr_RGB plane_cloud_tem(new PointCloud_RGB);
  pcl::copyPointCloud(*plane_cloud,*plane_cloud_tem);

  Point_RGB pr;
  pr.x=0;
  pr.y=0;
  pr.z=(-plane_coefficients->values[3])/plane_coefficients->values[2];

  plane_cloud_tem->push_back(pr);

  Eigen::Vector3d plane_normal;
  plane_normal << plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2];
  plane_normal.normalize();

  double angle=acos(plane_normal.dot(Eigen::Vector3d(0,0,1)));
  Eigen::Vector3d axis=plane_normal.cross(Eigen::Vector3d(0,0,1));
  axis.normalize();

  Eigen::Matrix4d matrix;
  getRotationMatrix(axis, angle, matrix);

  Eigen::Matrix4f matrix_transform = matrix.cast<float>();
  pcl::transformPointCloud (*plane_cloud_tem, *cloud_in_plane, matrix_transform);

  Point_RGB new_pr=cloud_in_plane->at(cloud_in_plane->size()-1);

  pointCloudPopUp(cloud_in_plane);

  cv::Point2f p0;
  cv::Point2f p1;
  cv::Point2f p2;
  cv::Point2f p3;

  std::cout<<"cloud_in_plane->size:"<< cloud_in_plane->size() <<std::endl;
  find_min_rect(cloud_in_plane, p0,p1,p2,p3);

  /*float min_z,max_z,avg_z;
  com_max_and_min_and_avg_z(cloud_in_plane,&min_z,&max_z,&avg_z);

  float cloud_z=min_z;

  if(max_z-avg_z<avg_z-min_z){
  cloud_z=max_z;
  }*/

  PointCloudPtr points(new PointCloud());
  points->push_back(Point(p0.x,p0.y,new_pr.z));
  points->push_back(Point(p1.x,p1.y,new_pr.z));
  points->push_back(Point(p2.x,p2.y,new_pr.z));
  points->push_back(Point(p3.x,p3.y,new_pr.z));

  Eigen::Matrix4d matrix_reverse;
  getRotationMatrix(axis, -angle, matrix_reverse);

  Eigen::Matrix4f matrix_transform_reverse = matrix_reverse.cast<float>();
  pcl::transformPointCloud (*points, *rect_cloud, matrix_transform_reverse);
}

//rotate a 2d point by a 2d point
void rotatePoint2ByPoint2(float p_x,float p_y,float cen_x,float cen_y,float ang,float *new_x,float *new_y){
  *new_x=cen_x+(p_x-cen_x)*cos(ang)-(p_y-cen_y)*sin(ang);
  *new_y=cen_y+(p_x-cen_x)*sin(ang)+(p_y-cen_y)*cos(ang);
}

//sample rect
void samplePlane(MyPointCloud& rect_mpc, float grid_length, MyPointCloud& sample_mpt){
  sample_mpt.mypoints.clear();

  float w=sqrt(pow(rect_mpc.mypoints.at(0).x-rect_mpc.mypoints.at(1).x, 2)+pow(rect_mpc.mypoints.at(0).y-rect_mpc.mypoints.at(1).y, 2)+pow(rect_mpc.mypoints.at(0).z-rect_mpc.mypoints.at(1).z, 2));
  float h=sqrt(pow(rect_mpc.mypoints.at(0).x-rect_mpc.mypoints.at(3).x, 2)+pow(rect_mpc.mypoints.at(0).y-rect_mpc.mypoints.at(3).y, 2)+pow(rect_mpc.mypoints.at(0).z-rect_mpc.mypoints.at(3).z, 2));

  int num_w=(int)(w/grid_length);
  int num_h=(int)(h/grid_length);

  Eigen::Vector3f normal_w;
  normal_w << rect_mpc.mypoints.at(0).x-rect_mpc.mypoints.at(1).x, rect_mpc.mypoints.at(0).y-rect_mpc.mypoints.at(1).y, rect_mpc.mypoints.at(0).z-rect_mpc.mypoints.at(1).z;
  normal_w.normalize();

  Eigen::Vector3f normal_h;
  normal_h << rect_mpc.mypoints.at(0).x-rect_mpc.mypoints.at(3).x, rect_mpc.mypoints.at(0).y-rect_mpc.mypoints.at(3).y, rect_mpc.mypoints.at(0).z-rect_mpc.mypoints.at(3).z;
  normal_h.normalize();

  Eigen::Vector3f first_pt;
  first_pt<<rect_mpc.mypoints.at(0).x,rect_mpc.mypoints.at(0).y,rect_mpc.mypoints.at(0).z;

  for(int i=0;i<num_w;i++){
    for(int j=0;j<num_h;j++){
      Eigen::Vector3f new_pt=first_pt-normal_w*(i+1)*grid_length-normal_h*(j+1)*grid_length;

      MyPt mp={new_pt[0],new_pt[1],new_pt[2]};
      sample_mpt.mypoints.push_back(mp);
    }
  }
}

//sample cylinder
void sampleCylinder(Point cenPoint0, Point cenPoint1, Vector3<float>& direction, float r, float grid_length, MyPointCloud& mpt){
  float height=sqrt(pow(cenPoint0.x-cenPoint1.x, 2)+pow(cenPoint0.y-cenPoint1.y, 2)+pow(cenPoint0.z-cenPoint1.z, 2));

  int num_cir=(int)((2*PI*r)/grid_length);
  int num_h=(int)(height/grid_length);

  Eigen::Vector3d direction_normal;
  direction_normal << direction[0], direction[1], direction[2];
  direction_normal.normalize();

  double angle0=acos(direction_normal.dot(Eigen::Vector3d(0,0,1)));
  Eigen::Vector3d axis0=direction_normal.cross(Eigen::Vector3d(0,0,1));
  axis0.normalize();

  Eigen::Matrix4d matrix0;
  getRotationMatrix(axis0, angle0, matrix0);

  PointCloud cloud;
  cloud.push_back(cenPoint0);
  cloud.push_back(cenPoint1);

  PointCloudPtr cloud_up(new PointCloud);
  PointCloudPtr cloud_temp(new PointCloud);

  Eigen::Matrix4f matrix_transform0 = matrix0.cast<float>();
  pcl::copyPointCloud(cloud,*cloud_temp);
  pcl::transformPointCloud (*cloud_temp, cloud, matrix_transform0);

  Point firstPoint0(cloud.at(0).x,cloud.at(0).y+r,cloud.at(0).z);
  Point firstPoint1(cloud.at(1).x,cloud.at(1).y+r,cloud.at(1).z);

  float ang=grid_length/r;
  int pos_neg_flag=1;

  if(firstPoint0.z>firstPoint1.z){
    pos_neg_flag=-1;
  }

  for(int i=0;i<num_cir;i++){
    float new_x=0;
    float new_y=0;
    rotatePoint2ByPoint2(firstPoint0.x,firstPoint0.y,cloud.at(0).x,cloud.at(0).y,ang*i,&new_x,&new_y);
    for(int j=0;j<num_h;j++){
      cloud_up->push_back(Point(new_x,new_y,cloud.at(0).z+pos_neg_flag*(j+1)*grid_length));
    }
  }

  Eigen::Matrix4d matrix1;
  getRotationMatrix(axis0, -angle0, matrix1);

  Eigen::Matrix4f matrix_transform1 = matrix1.cast<float>();
  pcl::copyPointCloud(*cloud_up,*cloud_temp);
  pcl::transformPointCloud (*cloud_temp, *cloud_up, matrix_transform1);


  PointCloud2MyPointCloud(cloud_up, mpt);
}

//sample sphere
void sampleSphere(Point cenPoint, float r, float grid_length, MyPointCloud& mpt){

  int woof_num=(int)((PI*r)/grid_length);

  PointCloudPtr sample_clound(new PointCloud);
  PointCloudPtr cloud_woof_points(new PointCloud);
  PointCloudPtr cloud_cen_points(new PointCloud);

  Point startPoint(cenPoint.x,cenPoint.y,cenPoint.z+r);

  sample_clound->push_back(startPoint);

  /*Eigen::Vector3d normal0;
  normal0 << 0, 0, r;
  normal0.normalize();

  Eigen::Vector3d normal1;
  normal1 << 0, 1, 0;
  normal1.normalize();*/

  float ang=grid_length/r;

  for(int i=0;i<woof_num;i++){
    float new_z=0;
    float new_x=0;
    rotatePoint2ByPoint2(startPoint.z,startPoint.x,cenPoint.z,cenPoint.x,ang*(i+1),&new_z,&new_x);

    cloud_woof_points->push_back(Point(new_x,cenPoint.y,new_z));
    cloud_cen_points->push_back(Point(cenPoint.x,cenPoint.y,new_z));
  }

  for(int i=0;i<woof_num;i++){
    Point cen_cir(cloud_cen_points->at(i).x,cloud_cen_points->at(i).y,cloud_cen_points->at(i).z);
    Point first_point(cloud_woof_points->at(i).x,cloud_woof_points->at(i).y,cloud_woof_points->at(i).z);
    float cir_r=sqrt(pow(first_point.x-cen_cir.x, 2)+pow(first_point.y-cen_cir.y, 2)+pow(first_point.z-cen_cir.z, 2));
    int num=(int)((2*PI*cir_r)/grid_length);
    float ang_tem=grid_length/cir_r;

    for(int j=0;j<num;j++){
      float new_x=0;
      float new_y=0;
      rotatePoint2ByPoint2(first_point.x,first_point.y,cen_cir.x,cen_cir.y,ang_tem*j,&new_x,&new_y);

      sample_clound->push_back(Point(new_x,new_y,cen_cir.z));
    }
  }

  PointCloud2MyPointCloud(sample_clound, mpt);
}

//if a point is in a cloud
bool isPointInCloud(Point pt, PointCloudPtr cloud){
  for(int i=0;i<cloud->size();i++){
    if(pt.x==cloud->at(i).x&&pt.y==cloud->at(i).y&&pt.z==cloud->at(i).z){
      return true;
    }
  }

  return false;
}

//find nearest neighbor
bool findNearestNeighbor(PointCloudPtr cloud, PointCloudPtr except_cloud, Point search_pt, Point& finded_pt){
  int min=10;
  bool flag=false;

  for(int i=0;i<cloud->size();i++){
    int dis=sqrt(pow(search_pt.x-cloud->at(i).x, 2)+pow(search_pt.y-cloud->at(i).y, 2)+pow(search_pt.z-cloud->at(i).z, 2));
    if(dis<min){
      min=dis;
      finded_pt.x=cloud->at(i).x;
      finded_pt.y=cloud->at(i).y;
      finded_pt.z=cloud->at(i).z;

      if(!isPointInCloud(finded_pt, except_cloud)){
        flag=true;
      }
    }
  }

  return flag;
}

//get intersection points
void get_intr_points(MyPointCloud& source_mpc, MyPointCloud& sample_mpc, float search_r, int* intr_points_num){
  *intr_points_num=0;

  PointCloudPtr source_cloud(new PointCloud);
  MyPointCloud2PointCloud(source_mpc, source_cloud);

  PointCloudPtr sample_cloud(new PointCloud);
  MyPointCloud2PointCloud(sample_mpc, sample_cloud);

  PointCloudPtr intr_cloud(new PointCloud);

  float resolution = 0.005f;

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

  octree.setInputCloud (source_cloud);
  octree.addPointsFromInputCloud ();

  for(int i=0;i<sample_mpc.mypoints.size();i++){
    // Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = search_r;

    if (octree.radiusSearch(sample_cloud->at(i), radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      if(pointIdxRadiusSearch.size()>0){
        PointCloudPtr cloud_tem(new PointCloud);

        for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j){
          cloud_tem->push_back(source_cloud->points[ pointIdxRadiusSearch[j]]);
        }

        Point finded_pt;

        if(findNearestNeighbor(cloud_tem, intr_cloud, sample_cloud->at(i), finded_pt)){
          intr_cloud->push_back(finded_pt);
          (*intr_points_num)+=1;
        }
      }
    }
  }
}

//compute Jaccard Index
void computeJaccardIndex(int a_num, int intr_num, float *result){
  *result=intr_num*1.0/(a_num+intr_num);
}

//compute Plane Jaccard Index
void computePlaneJaccardIndex(MyPointCloud& source_mpc, MyPointCloud& rect_mpc, float grid_length, float *result){
  MyPointCloud sample_mpc;
  samplePlane(rect_mpc, grid_length, sample_mpc);

  /*PointCloudPtr pc(new PointCloud);
  MyPointCloud2PointCloud(sample_mpc,pc);

  PointCloudPtr pc_source(new PointCloud);
  MyPointCloud2PointCloud(source_mpc,pc_source);

  showPointCloud2(pc,"plane_sample");

  showPointCloud2(pc_source,"plane_source");*/

  int intr_points_num;
  get_intr_points(source_mpc, sample_mpc, 0.0025, &intr_points_num);

  cout<<"source_mpc_plane.mypoints.size():"<<source_mpc.mypoints.size()<<endl;
  cout<<"sample_mpc_plane.mypoints.size():"<<sample_mpc.mypoints.size()<<endl;
  cout<<"intr_points_num_plane:"<<intr_points_num<<endl;

  float rate=intr_points_num*1.0/sample_mpc.mypoints.size();
  cout<<"rate_plane>>>>>>>>>>>>>>>>>>>>>>>>>>>:"<<rate<<endl;
  if(rate>0.1){
    computeJaccardIndex(source_mpc.mypoints.size(), intr_points_num, result);
  }
  else{
    *result=0;
  }
}

//compute Cylinder Jaccard Index
void computeCylinderJaccardIndex(MyPointCloud& source_mpc, Point cenPoint0, Point cenPoint1, Vector3<float>& direction, float r, float grid_length, float *result){
  MyPointCloud sample_mpc;
  sampleCylinder(cenPoint0, cenPoint1, direction,r,grid_length,sample_mpc);

  /*PointCloudPtr pc(new PointCloud);
  MyPointCloud2PointCloud(sample_mpc,pc);
  showPointCloud2(pc,"cylinder");*/

  int intr_points_num;
  get_intr_points(source_mpc, sample_mpc, 0.0025, &intr_points_num);

  cout<<"source_mpc_cylinder.mypoints.size():"<<source_mpc.mypoints.size()<<endl;
  cout<<"sample_mpc_cylinder.mypoints.size():"<<sample_mpc.mypoints.size()<<endl;
  cout<<"intr_points_num_cylinder:"<<intr_points_num<<endl;

  float rate=intr_points_num*1.0/sample_mpc.mypoints.size();
  cout<<"rate_cylinder>>>>>>>>>>>>>>>>>>>>>>>>>>>:"<<rate<<endl;
  if(rate>0.05){
   computeJaccardIndex(source_mpc.mypoints.size(), intr_points_num, result);
  }
  else{
    *result=0;
  }
}

//compute Sphere Jaccard Index
void computeSphereJaccardIndex(MyPointCloud& source_mpc, Point cenPoint, float r, float grid_length, float *result){
  MyPointCloud sample_mpc;
  sampleSphere(cenPoint, r, grid_length, sample_mpc);

  /*PointCloudPtr pc(new PointCloud);
  MyPointCloud2PointCloud(sample_mpc,pc);
  showPointCloud2(pc,"sphere");*/

  int intr_points_num;
  get_intr_points(source_mpc, sample_mpc, 0.0025, &intr_points_num);

  cout<<"source_mpc_sphere.mypoints.size():"<<source_mpc.mypoints.size()<<endl;
  cout<<"sample_mpc_sphere.mypoints.size():"<<sample_mpc.mypoints.size()<<endl;
  cout<<"intr_points_num_sphere:"<<intr_points_num<<endl;

  float rate=intr_points_num*1.0/sample_mpc.mypoints.size();
  cout<<"rate_sphere>>>>>>>>>>>>>>>>>>>>>>>>>>>:"<<rate<<endl;
  computeJaccardIndex(source_mpc.mypoints.size(), intr_points_num, result);

}

//detect table
void detect_table(PointCloudPtr_RGB sourceCloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers){
  // Create the segmentation object
  pcl::SACSegmentation<Point_RGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.015);

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (sourceCloud);
  seg.segment (*inliers, *coefficients);

  std::cout<< "palne_seg.getProbability():" <<seg.getProbability()<<std::endl;

  if (inliers->indices.size () == 0)
  {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    return;
  }
  else{
    std::cout<< "coefficients:" <<  *coefficients <<std::endl;
  }
}

//detect table plane
void detect_table_plane(PointCloudPtr_RGB sourceCloud, PointCloudPtr_RGB planeCloud, PointCloudPtr_RGB remainCloud){

  pcl::ExtractIndices<Point_RGB> extract;// Create the filtering object

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<Point_RGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.015);


  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (sourceCloud);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    return;
  }

  Eigen::Vector3d table_normal;
  table_normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
  table_normal.normalize();

  double angle=acos(table_normal.dot(Eigen::Vector3d(0,0,1)));
  Eigen::Vector3d axis=table_normal.cross(Eigen::Vector3d(0,0,1));
  axis.normalize();

  Eigen::Matrix4d matrix;
  getRotationMatrix(axis, angle, matrix);
  Eigen::Matrix4f matrix_transform = matrix.cast<float>();
  PointCloud_RGB sourceCloud_temp;
  pcl::copyPointCloud(*sourceCloud,sourceCloud_temp);
  pcl::transformPointCloud (sourceCloud_temp, *sourceCloud, matrix_transform);

  // Extract the inliers
  extract.setInputCloud (sourceCloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*planeCloud);
  std::cerr << "PointCloud representing the planar component: " << planeCloud->width * planeCloud->height << " data points." << std::endl;

  float min_x,min_y,min_z, max_x, max_y, max_z;
  com_bounding_box(planeCloud,&min_x,&min_y,&min_z, &max_x, &max_y, &max_z);

  for(int i=0;i<sourceCloud->size();i++){
    sourceCloud->at(i).z-=max_z;
  }

  PointCloudPtr_RGB cloud_tem(new PointCloud_RGB());
  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*cloud_tem);

  for(int i=0;i<cloud_tem->size();i++){
    if(cloud_tem->points[i].z>0){
      remainCloud->push_back(cloud_tem->at(i));
    }
  }
}

//detect table plane
void detect_table_plane_r(PointCloudPtr_RGB sourceCloud, PointCloudPtr_RGB planeCloud, PointCloudPtr_RGB remainCloud){

  pcl::ExtractIndices<Point_RGB> extract;// Create the filtering object

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<Point_RGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.015);


  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (sourceCloud);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    return;
  }

  Eigen::Vector3d table_normal;
  table_normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
  table_normal.normalize();

  double angle=acos(table_normal.dot(Eigen::Vector3d(0,0,1)));
  Eigen::Vector3d axis=table_normal.cross(Eigen::Vector3d(0,0,1));
  axis.normalize();

  Eigen::Matrix4d matrix;
  getRotationMatrix(axis, -(PI-angle), matrix);
  Eigen::Matrix4f matrix_transform = matrix.cast<float>();
  PointCloud_RGB sourceCloud_temp;
  pcl::copyPointCloud(*sourceCloud,sourceCloud_temp);
  pcl::transformPointCloud (sourceCloud_temp, *sourceCloud, matrix_transform);

  //pcl::io::savePCDFileASCII ("test_pcd.pcd", *sourceCloud);

  // Extract the inliers
  extract.setInputCloud (sourceCloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*planeCloud);
  std::cerr << "PointCloud representing the planar component: " << planeCloud->width * planeCloud->height << " data points." << std::endl;

  float min_x,min_y,min_z, max_x, max_y, max_z;
  com_bounding_box(planeCloud,&min_x,&min_y,&min_z, &max_x, &max_y, &max_z);

  for(int i=0;i<sourceCloud->size();i++){
    sourceCloud->at(i).z-=max_z;
  }

  PointCloudPtr_RGB cloud_tem(new PointCloud_RGB());
  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*cloud_tem);

  for(int i=0;i<cloud_tem->size();i++){
    if(cloud_tem->points[i].z>0){
      remainCloud->push_back(cloud_tem->at(i));
    }
  }
}

//Euclidean Cluster Extraction
void object_seg_ECE(PointCloudPtr_RGB cloud, std::vector<PointCloudPtr_RGB> &cluster_points){
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<Point_RGB>::Ptr tree (new pcl::search::KdTree<Point_RGB>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point_RGB> ec;
  ec.setClusterTolerance (0.015); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (50000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    PointCloudPtr_RGB cloud_cluster (new PointCloud_RGB);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    }
    cluster_points.push_back(cloud_cluster);

    j++;
  }
}

//Find Points In Cylinder
void findPointsIn_Cylinder(PointCloudPtr_RGB cloud, pcl::ModelCoefficients::Ptr &coefficients, pcl::PointIndices::Ptr &inliers){

  Eigen::Vector3d cylinder_normal;
  cylinder_normal << coefficients->values[3], coefficients->values[4], coefficients->values[5];
  cylinder_normal.normalize();

  double min=0;
  double max=0;
  bool flag=true;

  for(int i=0;i<cloud->size();i++){
    double x=cloud->points[i].x;
    double y=cloud->points[i].y;
    double z=cloud->points[i].z;

    double distance=sqrt(pow(coefficients->values[0]-x, 2)+pow(coefficients->values[1]-y, 2)+pow(coefficients->values[2]-z, 2));

    Eigen::Vector3d tem_normal;
    tem_normal << x-coefficients->values[0], y-coefficients->values[1], z-coefficients->values[2];
    tem_normal.normalize();

    double angle=acos(cylinder_normal.dot(tem_normal));
    //double angle1=std::abs(acos(cylinder_normal.dot(Eigen::Vector3d(0,0,-1)))-PI/2);
    //double dis_z=coefficients->values[6]*cos(angle1);

    if(std::abs(std::abs(distance*sin(angle))-coefficients->values[6])<=0.005){

      double tem_normal_dis=distance*(cylinder_normal.dot(tem_normal));
      double tem_cen_z=coefficients->values[2]+tem_normal_dis*(cylinder_normal.dot(Eigen::Vector3d(0,0,1)));

      if(flag){
        min=tem_normal_dis;
        max=tem_normal_dis;
        flag=false;
      }

      if(tem_normal_dis<min){
        if(tem_cen_z>0){
          min=tem_normal_dis;
        }
      }

      if(tem_normal_dis>max){
        if(tem_cen_z>0){
          max=tem_normal_dis;
        }
      }
    }
  }

  if(min!=max){

    for(int i=0;i<cloud->size();i++){
      double x=cloud->points[i].x;
      double y=cloud->points[i].y;
      double z=cloud->points[i].z;

      double distance=sqrt(pow(coefficients->values[0]-x, 2)+pow(coefficients->values[1]-y, 2)+pow(coefficients->values[2]-z, 2));

      Eigen::Vector3d tem_normal;
      tem_normal << x-coefficients->values[0], y-coefficients->values[1], z-coefficients->values[2];
      tem_normal.normalize();

      double angle=acos(cylinder_normal.dot(tem_normal));

      if(std::abs(distance*sin(angle))<=(coefficients->values[6]+0.01)){
        if(distance*(cylinder_normal.dot(tem_normal))<=max&&distance*(cylinder_normal.dot(tem_normal))>=min){
          inliers->indices.push_back(i);
        }
      }
    }

    coefficients->values.push_back(coefficients->values[0]+max*(cylinder_normal.dot(Eigen::Vector3d(1,0,0))));
    coefficients->values.push_back(coefficients->values[1]+max*(cylinder_normal.dot(Eigen::Vector3d(0,1,0))));
    coefficients->values.push_back(coefficients->values[2]+max*(cylinder_normal.dot(Eigen::Vector3d(0,0,1))));

    coefficients->values[0]+=min*(cylinder_normal.dot(Eigen::Vector3d(1,0,0)));
    coefficients->values[1]+=min*(cylinder_normal.dot(Eigen::Vector3d(0,1,0)));
    coefficients->values[2]+=min*(cylinder_normal.dot(Eigen::Vector3d(0,0,1)));

    std::cerr << "Cylinder coefficients==: " << *coefficients << std::endl;
  }
}

//cylinder fitting
void cylinder_fitting(PointCloudPtr_RGB cloud, MyPointCloud_RGB &cylinder_cloud, pcl::ModelCoefficients::Ptr cylinder_coefficients,PointCloudPtr_RGB remained_cloud){
  // All the objects needed
  pcl::PassThrough<Point_RGB> pass;

  // Datasets
  pcl::PointCloud<Point_RGB>::Ptr cloud_filtered (new pcl::PointCloud<Point_RGB>);
  pcl::PointCloud<Point_RGB>::Ptr cloud_f (new pcl::PointCloud<Point_RGB>);

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.0);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  pcl::SACSegmentationFromNormals<Point_RGB, pcl::Normal> seg;

  pcl::ExtractIndices<Point_RGB> extract;
  pcl::NormalEstimation<Point_RGB, pcl::Normal> ne;
  pcl::search::KdTree<Point_RGB>::Ptr tree (new pcl::search::KdTree<Point_RGB> ());

  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  //seg.setDistanceThreshold (0.04);
  seg.setDistanceThreshold (0.025);
  seg.setRadiusLimits (0.01, 0.1);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *cylinder_coefficients);

  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  PointCloudPtr_RGB cloud_cylinder (new PointCloud_RGB());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.size()<500){
    std::cerr << "Can't find the cylindrical component." << std::endl;
  }
  else
  {
    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;

    pcl::PointIndices::Ptr tem_inliers(new pcl::PointIndices);
    findPointsIn_Cylinder(cloud_filtered, cylinder_coefficients, tem_inliers);

    std::cout<<"inliers->indices.size:"<< tem_inliers->indices.size() <<std::endl;

    double h=sqrt(pow(cylinder_coefficients->values[0]-cylinder_coefficients->values[7], 2)+
      pow(cylinder_coefficients->values[1]-cylinder_coefficients->values[8], 2)+
      pow(cylinder_coefficients->values[2]-cylinder_coefficients->values[9], 2));


    if(tem_inliers->indices.size()==0||tem_inliers->indices.size()>10000||h>0.5){
    }
    else{
      extract.setIndices (tem_inliers);
      extract.setNegative (false);
      extract.filter (*cloud_f);

      PointCloud2MyPointCloud_RGB(cloud_f, cylinder_cloud);


      extract.setIndices (tem_inliers);
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_filtered.swap (cloud_f);
    }
  }

  pcl::copyPointCloud(*cloud_filtered,*remained_cloud);
}

//segment cylinder from the data
void object_seg_Cylinder(PointCloudPtr_RGB cloud, std::vector<MyPointCloud_RGB> &cluster_points, std::vector<pcl::ModelCoefficients> &coefficients,PointCloudPtr_RGB remained_cloud){
  // All the objects needed
  pcl::PassThrough<Point_RGB> pass;

  // Datasets
  pcl::PointCloud<Point_RGB>::Ptr cloud_filtered (new pcl::PointCloud<Point_RGB>);
  pcl::PointCloud<Point_RGB>::Ptr cloud_filtered_tem (new pcl::PointCloud<Point_RGB>);
  pcl::PointCloud<Point_RGB>::Ptr cloud_f (new pcl::PointCloud<Point_RGB>);

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.0);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  //pcl::copyPointCloud(*cloud_filtered,*cloud_filtered_tem);

  int d=0;//debug

  do{
    pcl::SACSegmentationFromNormals<Point_RGB, pcl::Normal> seg;

    pcl::ExtractIndices<Point_RGB> extract;
    pcl::NormalEstimation<Point_RGB, pcl::Normal> ne;
    pcl::search::KdTree<Point_RGB>::Ptr tree (new pcl::search::KdTree<Point_RGB> ());

    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    //seg.setDistanceThreshold (0.04);
    seg.setDistanceThreshold (0.025);
    seg.setRadiusLimits (0.01, 0.1);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    //std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
    std::cout<< "seg.getProbability():" << seg.getProbability() <<std::endl;

    /*if(d==1){
    showPointClound (cloud_filtered,"0");
    }*/

    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<Point_RGB>::Ptr cloud_cylinder (new pcl::PointCloud<Point_RGB> ());
    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.size()<500){
      appendCloud_RGB(cloud_filtered,cloud_filtered_tem);
      std::cerr << "Can't find the cylindrical component." << std::endl;
      break;
    }
    else
    {
      std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;

      pcl::PointIndices::Ptr tem_inliers(new pcl::PointIndices);
      findPointsIn_Cylinder(cloud_filtered, coefficients_cylinder, tem_inliers);

      std::cout<<"inliers->indices.size:"<< tem_inliers->indices.size() <<std::endl;

      double h=sqrt(pow(coefficients_cylinder->values[0]-coefficients_cylinder->values[7], 2)+
        pow(coefficients_cylinder->values[1]-coefficients_cylinder->values[8], 2)+
        pow(coefficients_cylinder->values[2]-coefficients_cylinder->values[9], 2));


      if(tem_inliers->indices.size()==0||tem_inliers->indices.size()>10000||h>0.5){
        extract.setIndices (inliers_cylinder);
        extract.setNegative (false);
        extract.filter (*cloud_f);

        appendCloud_RGB(cloud_f,cloud_filtered_tem);

        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
      }
      else{
        double l=2*PI*coefficients_cylinder->values[6];
        double rate=inliers_cylinder->indices.size()/(h*l);
        cout<<"rate_cylinder========================:"<< rate <<endl;

        if(rate>10000){
          extract.setIndices (tem_inliers);
          extract.setNegative (false);
          extract.filter (*cloud_f);

          MyPointCloud_RGB mpc;
          PointCloud2MyPointCloud_RGB(cloud_f, mpc);
          cluster_points.push_back(mpc);

          extract.setIndices (tem_inliers);
          extract.setNegative (true);
          extract.filter (*cloud_f);
          cloud_filtered.swap (cloud_f);

          //showPointClound (cloud_filtered,"1");

          //cluster_inliers.push_back(*tem_inliers);

          /*coefficients.push_back(*coefficients_cylinder);
          extract.setIndices (tem_inliers);
          extract.setInputCloud (cloud_filtered_tem);
          extract.setNegative (true);
          extract.filter (*cloud_f);
          cloud_filtered_tem.swap (cloud_f);*/

          d++;
        }
        else{
          extract.setIndices (inliers_cylinder);
          extract.setNegative (false);
          extract.filter (*cloud_f);

          appendCloud_RGB(cloud_f,cloud_filtered_tem);

          extract.setIndices (inliers_cylinder);
          extract.setNegative (true);
          extract.filter (*cloud_f);
          cloud_filtered.swap (cloud_f);
        }
      }

      //extract.setInputCloud (cloud_filtered);
    }

  }while(1);

  cout<<"coefficients.size()++++++++++++++++++++:"<<coefficients.size()<<endl;
  pcl::copyPointCloud(*cloud_filtered_tem,*remained_cloud);
}


//Find Points In Sphere
void findPointsIn_Sphere(PointCloudPtr_RGB cloud, pcl::ModelCoefficients::Ptr &coefficients, pcl::PointIndices::Ptr &inliers){
  for(int i=0;i<cloud->size();i++){
    float x=cloud->points[i].x;
    float y=cloud->points[i].y;
    float z=cloud->points[i].z;

    float dist=sqrt(pow((x-coefficients->values[0]),2)+pow((y-coefficients->values[1]),2)+pow((z-coefficients->values[2]),2));

    if(dist<=coefficients->values[3]+0.01){
      inliers->indices.push_back(i);
    }
  }
}

//sphere fitting
void sphere_fitting(PointCloudPtr_RGB cloud, MyPointCloud_RGB &sphere_cloud, pcl::ModelCoefficients::Ptr sphere_coefficients, PointCloudPtr_RGB remained_cloud){
  // Datasets
  PointCloudPtr_RGB cloud_filtered (new PointCloud_RGB);
  PointCloudPtr_RGB cloud_f (new PointCloud_RGB);

  pcl::copyPointCloud(*cloud,*cloud_filtered);

  // Create the segmentation object
  pcl::SACSegmentation<Point_RGB> seg;
  pcl::PointIndices::Ptr inliers_sphere(new pcl::PointIndices);
  pcl::ExtractIndices<Point_RGB> extract;

  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_SPHERE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.02);

  // Segment the sphere component from the remaining cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers_sphere, *sphere_coefficients);


  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_sphere);
  extract.setNegative (false);
  pcl::PointCloud<Point_RGB>::Ptr cloud_sphere (new pcl::PointCloud<Point_RGB> ());
  extract.filter (*cloud_sphere);

  double rate=0;

  if (inliers_sphere->indices.size () < 500)
  {
    std::cerr << "Could not estimate a sphere model for the given dataset." << std::endl;
    pcl::copyPointCloud(*cloud_filtered,*remained_cloud);
    return;
  }

  if(sphere_coefficients->values[3]>0.5||sphere_coefficients->values[2]-sphere_coefficients->values[3]<0){
    std::cerr << "Could not estimate a sphere model for the given dataset===." << std::endl;
    pcl::copyPointCloud(*cloud_filtered,*remained_cloud);
    return;
  }
  else{
    std::cout<< "coefficients_sphere:" <<  *sphere_coefficients <<std::endl;
    pcl::PointIndices::Ptr tem_inliers(new pcl::PointIndices);
    findPointsIn_Sphere(cloud_filtered, sphere_coefficients, tem_inliers);


    extract.setIndices (tem_inliers);
    extract.setNegative (false);
    extract.filter (*cloud_f);

    PointCloud2MyPointCloud_RGB(cloud_f, sphere_cloud);

    extract.setIndices (tem_inliers);
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
  }

  pcl::copyPointCloud(*cloud_filtered,*remained_cloud);
}

//segment sphere from the data
void object_seg_Sphere(PointCloudPtr_RGB cloud, std::vector<MyPointCloud_RGB> &cluster_points, std::vector<pcl::ModelCoefficients> &coefficients,PointCloudPtr_RGB remained_cloud){

  // Datasets
  PointCloudPtr_RGB cloud_filtered (new PointCloud_RGB);
  PointCloudPtr_RGB cloud_filtered_tem (new PointCloud_RGB);
  PointCloudPtr_RGB cloud_f (new PointCloud_RGB);

  //pcl::copyPointCloud(*cloud,*cloud_filtered);
  pcl::copyPointCloud(*cloud,*cloud_filtered_tem);

  while(1){
    // Create the segmentation object
    pcl::SACSegmentation<Point_RGB> seg;
    pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_sphere(new pcl::PointIndices);
    pcl::ExtractIndices<Point_RGB> extract;

    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_SPHERE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.1);

    // Segment the sphere component from the remaining cloud
    seg.setInputCloud (cloud_filtered_tem);
    seg.segment (*inliers_sphere, *coefficients_sphere);

    std::cout<< "sphere_seg.getProbability():" <<seg.getProbability()<<std::endl;

    extract.setInputCloud (cloud_filtered_tem);
    extract.setIndices (inliers_sphere);
    extract.setNegative (false);
    pcl::PointCloud<Point_RGB>::Ptr cloud_sphere (new pcl::PointCloud<Point_RGB> ());
    extract.filter (*cloud_sphere);

    double rate=0;

    if (inliers_sphere->indices.size () < 500)
    {
      appendCloud_RGB(cloud_filtered_tem,cloud_filtered);
      std::cerr << "Could not estimate a sphere model for the given dataset." << std::endl;
      break;
    }
    else{
      double area=4*PI*coefficients_sphere->values[3]*coefficients_sphere->values[3];
      rate=inliers_sphere->indices.size()/area;
      cout<<"rate_sphere========================:"<< rate <<endl;
    }

    if(coefficients_sphere->values[3]>0.5||coefficients_sphere->values[2]-coefficients_sphere->values[3]<0||rate<5000){
      std::cerr << "Could not estimate a sphere model for the given dataset===." << std::endl;
      extract.setIndices (inliers_sphere);
      extract.setNegative (false);
      extract.filter (*cloud_f);

      appendCloud_RGB(cloud_f,cloud_filtered);

      extract.setIndices (inliers_sphere);
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_filtered_tem.swap (cloud_f);
    }
    else{
      coefficients.push_back(*coefficients_sphere);
      std::cout<< "coefficients_sphere:" <<  *coefficients_sphere <<std::endl;
      pcl::PointIndices::Ptr tem_inliers(new pcl::PointIndices);
      findPointsIn_Sphere(cloud_filtered_tem, coefficients_sphere, tem_inliers);

      //cluster_inliers.push_back(*tem_inliers);

      extract.setIndices (tem_inliers);
      extract.setNegative (false);
      extract.filter (*cloud_f);

      MyPointCloud_RGB mpc;
      PointCloud2MyPointCloud_RGB(cloud_f, mpc);
      cluster_points.push_back(mpc);

      extract.setIndices (tem_inliers);
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_filtered_tem.swap (cloud_f);

      /*extract.setInputCloud(cloud_filtered);
      extract.setIndices (tem_inliers);
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_filtered.swap (cloud_f);*/
    }
  }
  pcl::copyPointCloud(*cloud_filtered,*remained_cloud);

  //showPointClound (cloud_filtered,"sssss");
}

//plane for boxs fitting
void plane_for_boxs_fitting(PointCloudPtr_RGB sourceCloud, MyPointCloud_RGB &plane_cloud, MyPointCloud &rect_cloud, pcl::ModelCoefficients::Ptr plane_coefficients, PointCloudPtr_RGB remained_cloud){
  PointCloudPtr_RGB cloud_tem(new PointCloud_RGB);
  pcl::copyPointCloud(*sourceCloud,*cloud_tem);

  PointCloudPtr_RGB cloud_p (new PointCloud_RGB);
  PointCloudPtr_RGB cloud_f (new PointCloud_RGB);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::ExtractIndices<Point_RGB> extract;

  // Create the segmentation object
  pcl::SACSegmentation<Point_RGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.005);

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_tem);
  seg.segment (*inliers, *plane_coefficients);
  if (inliers->indices.size () == 0)
  {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    pcl::copyPointCloud(*cloud_tem,*remained_cloud);
    return;
  }

  // Extract the inliers
  extract.setInputCloud (cloud_tem);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);
  std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

  double rate=0;

  if(cloud_p->size()<200){
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    pcl::copyPointCloud(*cloud_tem,*remained_cloud);
    return;
  }
 /* else{
    PointCloudPtr_RGB cloud_in_plane(new PointCloud_RGB());

    Eigen::Vector3d plane_normal;
    plane_normal << plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2];
    plane_normal.normalize();

    double angle=acos(plane_normal.dot(Eigen::Vector3d(0,0,1)));
    Eigen::Vector3d axis=plane_normal.cross(Eigen::Vector3d(0,0,1));
    axis.normalize();

    Eigen::Matrix4d matrix;
    getRotationMatrix(axis, angle, matrix);

    Eigen::Matrix4f matrix_transform = matrix.cast<float>();
    pcl::transformPointCloud (*cloud_p, *cloud_in_plane, matrix_transform);

    cv::Point2f p0;
    cv::Point2f p1;
    cv::Point2f p2;
    cv::Point2f p3;

    find_min_rect(cloud_in_plane, p0, p1, p2, p3);

    float a=sqrt(pow((p0.x-p1.x),2)+pow((p0.y-p1.y),2));
    float b=sqrt(pow((p2.x-p1.x),2)+pow((p2.y-p1.y),2));
    double area=a*b;

    rate=cloud_p->size()/area;

    cout<<"rate_plane===============::"<<rate<<endl;
  }

  if(rate>30000){*/
    PointCloudPtr rect_cl(new PointCloud);
    getRectForPlaneCloud(cloud_p, plane_coefficients, rect_cl);
    PointCloud2MyPointCloud_RGB(cloud_p, plane_cloud);
    PointCloud2MyPointCloud(rect_cl, rect_cloud);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_tem.swap (cloud_f);
 // }

  pcl::copyPointCloud(*cloud_tem,*remained_cloud);
}

void detect_plane_for_boxs(PointCloudPtr_RGB sourceCloud, std::vector<pcl::ModelCoefficients> &coefficients_vector, std::vector<MyPointCloud_RGB> &plane_points,PointCloudPtr_RGB remained_cloud){

  PointCloudPtr_RGB clound_tem(new PointCloud_RGB);
  PointCloudPtr_RGB cloud_remaining(new PointCloud_RGB);
  pcl::copyPointCloud(*sourceCloud,*clound_tem);

  PointCloudPtr_RGB cloud_p (new PointCloud_RGB);
  PointCloudPtr_RGB cloud_f (new PointCloud_RGB);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::ExtractIndices<Point_RGB> extract;

  // Create the segmentation object
  pcl::SACSegmentation<Point_RGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.005);


  while (1)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (clound_tem);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      appendCloud_RGB(clound_tem,cloud_remaining);

      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (clound_tem);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    double rate=0;

    if(cloud_p->size()<200){
      appendCloud_RGB(clound_tem,cloud_remaining);
      break;
    }
    else{
      PointCloudPtr_RGB cloud_in_plane(new PointCloud_RGB());
      //pcl::copyPointCloud(*,*cloud_in_plane);

      Eigen::Vector3d plane_normal;
      plane_normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
      plane_normal.normalize();

      double angle=acos(plane_normal.dot(Eigen::Vector3d(0,0,1)));
      Eigen::Vector3d axis=plane_normal.cross(Eigen::Vector3d(0,0,1));
      axis.normalize();

      Eigen::Matrix4d matrix;
      getRotationMatrix(axis, angle, matrix);

      Eigen::Matrix4f matrix_transform = matrix.cast<float>();
      pcl::transformPointCloud (*cloud_p, *cloud_in_plane, matrix_transform);

      cv::Point2f p0;
      cv::Point2f p1;
      cv::Point2f p2;
      cv::Point2f p3;

      find_min_rect(cloud_in_plane, p0, p1, p2, p3);

      float a=sqrt(pow((p0.x-p1.x),2)+pow((p0.y-p1.y),2));
      float b=sqrt(pow((p2.x-p1.x),2)+pow((p2.y-p1.y),2));
      double area=a*b;

      rate=cloud_p->size()/area;

      cout<<"rate_plane===============::"<<rate<<endl;
    }

    if(rate>30000){
      coefficients_vector.push_back(*coefficients);

      MyPointCloud_RGB mpc;
      PointCloud2MyPointCloud_RGB(cloud_p, mpc);
      plane_points.push_back(mpc);
    }
    else{
      extract.setNegative (false);
      extract.filter (*cloud_f);

      appendCloud_RGB(cloud_f,cloud_remaining);
    }

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    clound_tem.swap (cloud_f);
  }

  pcl::copyPointCloud(*cloud_remaining,*remained_cloud);
}

//Wm5IntrTriangle3Triangle3
bool testIntrTriangle3Triangle3(MyPt p00,MyPt p01,MyPt p02, MyPt p10,MyPt p11,MyPt p12){

  Triangle3<float> triangle1(Vector3<float>(p00.x,p00.y,p00.z),Vector3<float>(p01.x,p01.y,p01.z),Vector3<float>(p02.x,p02.y,p02.z));
  Triangle3<float> triangle2(Vector3<float>(p10.x,p10.y,p10.z),Vector3<float>(p11.x,p11.y,p11.z),Vector3<float>(p12.x,p12.y,p12.z));

  IntrTriangle3Triangle3<float> intTri3Tri3(triangle1, triangle2);
  bool bo=intTri3Tri3.Test();

  return bo;
}

//If a rectangle intersects with the other
bool testIntrRectangle3Rectangle3(MyPt p0_0, MyPt p0_1,MyPt p0_2,MyPt p0_3, MyPt p1_0,MyPt p1_1,MyPt p1_2,MyPt p1_3){
  if(testIntrTriangle3Triangle3(p0_0,p0_1,p0_2, p1_0,p1_1,p1_2)){
    return true;
  }
  if(testIntrTriangle3Triangle3(p0_0,p0_3,p0_2, p1_0,p1_1,p1_2)){
    return true;
  }
  if(testIntrTriangle3Triangle3(p0_0,p0_1,p0_2, p1_0,p1_3,p1_2)){
    return true;
  }
  if(testIntrTriangle3Triangle3(p0_0,p0_3,p0_2, p1_0,p1_3,p1_2)){
    return true;
  }

  return false;
}

//Find Points In Box
void findPointsIn_Box(PointCloudPtr_RGB cloud, PointCloudPtr box_points, pcl::PointIndices::Ptr &inliers){

  Eigen::Vector3d box_plane_normal0;
  Eigen::Vector3d box_plane_normal1;
  Eigen::Vector3d box_plane_normal2;
  box_plane_normal0 << box_points->points[0].x-box_points->points[1].x,box_points->points[0].y-box_points->points[1].y,box_points->points[0].z-box_points->points[1].z;
  box_plane_normal1 << box_points->points[1].x-box_points->points[2].x,box_points->points[1].y-box_points->points[2].y,box_points->points[1].z-box_points->points[2].z;
  box_plane_normal2 << box_points->points[0].x-box_points->points[4].x,box_points->points[0].y-box_points->points[4].y,box_points->points[0].z-box_points->points[4].z;

  for(int i=0;i<cloud->size();i++){
    float x=cloud->points[i].x;
    float y=cloud->points[i].y;
    float z=cloud->points[i].z;

    int cout0=0;
    int cout1=0;
    int cout2=0;

    for(int j=0;j<box_points->size();j++){
      Eigen::Vector3d normal_tem;
      normal_tem << x-box_points->points[j].x,y-box_points->points[j].y,z-box_points->points[j].z;

      if(box_plane_normal0.dot(normal_tem)<0){
        cout0++;
      }

      if(box_plane_normal1.dot(normal_tem)<0){
        cout1++;
      }

      if(box_plane_normal2.dot(normal_tem)<0){
        cout2++;
      }
    }

    if(cout0==4&&cout1==4&&cout2==4){
      inliers->indices.push_back(i);
    }
  }
}

//seg plane in objects
void object_seg_Plane(PointCloudPtr_RGB sourceCloud, std::vector<pcl::ModelCoefficients> &coefficients_vector, std::vector<MyPointCloud_RGB>& new_plane_clouds, std::vector<MyPointCloud> &debug_clouds,PointCloudPtr_RGB remained_cloud){

  std::vector<MyPointCloud_RGB> plane_clouds;

  PointCloudPtr_RGB clound_tem(new PointCloud_RGB);
  PointCloudPtr_RGB cloud_f (new PointCloud_RGB);
  pcl::copyPointCloud(*sourceCloud,*clound_tem);

  detect_plane_for_boxs(sourceCloud,coefficients_vector,plane_clouds,remained_cloud);

  if(coefficients_vector.size()<2){
    if(coefficients_vector.size()==1){
      PointCloudPtr_RGB cloud_temp(new PointCloud_RGB);
      PointCloud points_temp;

      PointCloudPtr_RGB cloud_in_plane(new PointCloud_RGB());
      //pcl::copyPointCloud(plane_clouds.at(0),*cloud_in_plane);
      MyPointCloud_RGB2PointCloud(plane_clouds.at(0), cloud_in_plane);
      std::cout<<"plane_clouds.at(0).mypoints.size:"<< plane_clouds.at(0).mypoints.size() <<std::endl;
      std::cout<<"cloud_in_plane->size:"<< cloud_in_plane->size() <<std::endl;

      new_plane_clouds.push_back(plane_clouds.at(0));

      Eigen::Vector3d plane_normal;
      plane_normal << coefficients_vector.at(0).values[0], coefficients_vector.at(0).values[1], coefficients_vector.at(0).values[2];
      plane_normal.normalize();

      double angle=acos(plane_normal.dot(Eigen::Vector3d(0,0,1)));
      Eigen::Vector3d axis=plane_normal.cross(Eigen::Vector3d(0,0,1));
      axis.normalize();

      Eigen::Matrix4d matrix;
      getRotationMatrix(axis, angle, matrix);

      Eigen::Matrix4f matrix_transform = matrix.cast<float>();
      //pcl::copyPointCloud(plane_clouds.at(0),cloud_temp);
      MyPointCloud_RGB2PointCloud(plane_clouds.at(0), cloud_temp);
      pcl::transformPointCloud (*cloud_temp, *cloud_in_plane, matrix_transform);

      cv::Point2f p0;
      cv::Point2f p1;
      cv::Point2f p2;
      cv::Point2f p3;

      printf("0000000000000000003\n");
      std::cout<<"cloud_in_plane->size:"<< cloud_in_plane->size() <<std::endl;
      find_min_rect(cloud_in_plane, p0,p1,p2,p3);

      float min_z,max_z,avg_z;
      com_max_and_min_and_avg_z(cloud_in_plane,&min_z,&max_z,&avg_z);

      float cloud_z=min_z;

      if(max_z-avg_z<avg_z-min_z){
        cloud_z=max_z;
      }

      PointCloudPtr points(new PointCloud());
      points->push_back(Point(p0.x,p0.y,cloud_z));
      points->push_back(Point(p1.x,p1.y,cloud_z));
      points->push_back(Point(p2.x,p2.y,cloud_z));
      points->push_back(Point(p3.x,p3.y,cloud_z));

      Eigen::Matrix4d matrix_reverse;
      getRotationMatrix(axis, -angle, matrix_reverse);

      Eigen::Matrix4f matrix_transform_reverse = matrix_reverse.cast<float>();
      pcl::copyPointCloud(*points,points_temp);
      pcl::transformPointCloud (points_temp, *points, matrix_transform_reverse);

      MyPointCloud mpc;
      PointCloud2MyPointCloud(points, mpc);

      debug_clouds.push_back(mpc);
    }

    return;
  }

  std::vector<Eigen::Vector3d> normals;
  std::vector<pcl::ModelCoefficients::Ptr> coefficientsPtr_vector;
  std::vector<Eigen::Vector3d> new_normals;
  std::vector<pcl::ModelCoefficients::Ptr> new_coefficientsPtr_vector;
  std::vector<MyPointCloud> rect_clouds;

  for(int i=0;i<coefficients_vector.size();i++){
    Eigen::Vector3d plane_normal;
    plane_normal << coefficients_vector.at(i).values[0], coefficients_vector.at(i).values[1], coefficients_vector.at(i).values[2];
    plane_normal.normalize();
    normals.push_back(plane_normal);
  }

  for(int i=0;i<coefficients_vector.size();i++){
    pcl::ModelCoefficients::Ptr mcf(new pcl::ModelCoefficients ());
    for(int j=0;j<coefficients_vector.at(i).values.size();j++){
      mcf->values.push_back(coefficients_vector.at(i).values[j]);
    }
    coefficientsPtr_vector.push_back(mcf);
  }

  //===========For redundant and approximately parallel rectagles that intersects with each other, merge them by least squares.
  int cout = coefficientsPtr_vector.size();

  std::cout<<"&&&&&&&&&&&&&&&&&******cout"<<cout<<std::endl;

  while(cout >0){
    PointCloud_RGB cloud_temp;
    PointCloud points_temp;
    std::vector<MyPointCloud> horizontal_points;
    std::vector<int> horizontal_index;

    int index=0;
    for(int i=0;i<coefficientsPtr_vector.size();i++){
      if(coefficientsPtr_vector.at(i)->values.size()!=0){
        index=i;
        break;
      }
    }

    PointCloudPtr_RGB cloud_projected0(new PointCloud_RGB());
    //pcl::copyPointCloud(plane_clouds.at(index),*cloud_projected0);
    MyPointCloud_RGB2PointCloud(plane_clouds.at(index), cloud_projected0);
    std::cout<<"plane_clouds.at(index).mypoints.size:"<<plane_clouds.at(index).mypoints.size()<<std::endl;
    std::cout<<"cloud_projected0->size:"<<cloud_projected0->size()<<std::endl;
    //showPointCloud (cloud_projected0,"test");

    double angle0=acos(normals[index].dot(Eigen::Vector3d(0,0,1)));
    Eigen::Vector3d axis0=normals[index].cross(Eigen::Vector3d(0,0,1));
    axis0.normalize();

    Eigen::Matrix4d matrix0;
    getRotationMatrix(axis0, angle0, matrix0);

    Eigen::Matrix4f matrix_transform0 = matrix0.cast<float>();
    pcl::copyPointCloud(*cloud_projected0,cloud_temp);
    pcl::transformPointCloud (cloud_temp, *cloud_projected0, matrix_transform0);

    cv::Point2f p0_0;
    cv::Point2f p1_0;
    cv::Point2f p2_0;
    cv::Point2f p3_0;

    printf("00000000000000000004\n");
    find_min_rect(cloud_projected0, p0_0,p1_0,p2_0,p3_0);

    float min_z0,max_z0,avg_z0;
    com_max_and_min_and_avg_z(cloud_projected0,&min_z0,&max_z0,&avg_z0);

    float cloud_z0=min_z0;

    if(max_z0-avg_z0<avg_z0-min_z0){
      cloud_z0=max_z0;
    }

    std::cout<<"cloud_z0:"<<cloud_z0<<std::endl;
    std::cout<<"cloud_projected0->points[0].z:"<<cloud_projected0->points[0].z<<std::endl;

    PointCloudPtr points0(new PointCloud());
    points0->push_back(Point(p0_0.x,p0_0.y,cloud_z0));
    points0->push_back(Point(p1_0.x,p1_0.y,cloud_z0));
    points0->push_back(Point(p2_0.x,p2_0.y,cloud_z0));
    points0->push_back(Point(p3_0.x,p3_0.y,cloud_z0));

    Eigen::Matrix4d matrix0_reverse;
    getRotationMatrix(axis0, -angle0, matrix0_reverse);

    Eigen::Matrix4f matrix_transform0_reverse = matrix0_reverse.cast<float>();
    pcl::copyPointCloud(*points0,points_temp);
    pcl::transformPointCloud (points_temp, *points0, matrix_transform0_reverse);

    MyPointCloud mpc0;
    PointCloud2MyPointCloud(points0, mpc0);
    horizontal_points.push_back(mpc0);
    horizontal_index.push_back(index);

    MyPointCloud_RGB pit_tem;

    for(int i=index+1;i<coefficientsPtr_vector.size();i++){

      if(coefficientsPtr_vector.at(i)->values.size()!=0){

        //horizontal
        if(std::abs(normals[index].dot(normals[i])-1)<0.03){

          horizontal_index.push_back(i);

          PointCloudPtr_RGB cloud_projected_h(new PointCloud_RGB());
          //pcl::copyPointCloud(plane_clouds.at(i),*cloud_projected_h);
          MyPointCloud_RGB2PointCloud(plane_clouds.at(i), cloud_projected_h);

          double angle_h=acos(normals[i].dot(Eigen::Vector3d(0,0,1)));
          Eigen::Vector3d axis_h=normals[i].cross(Eigen::Vector3d(0,0,1));
          axis_h.normalize();

          Eigen::Matrix4d matrix_h;
          getRotationMatrix(axis_h, angle_h, matrix_h);

          Eigen::Matrix4f matrix_transform_h = matrix_h.cast<float>();
          pcl::copyPointCloud(*cloud_projected_h,cloud_temp);
          pcl::transformPointCloud (cloud_temp, *cloud_projected_h, matrix_transform_h);

          cv::Point2f p0_h;
          cv::Point2f p1_h;
          cv::Point2f p2_h;
          cv::Point2f p3_h;

          printf("000000000000000000000");
          find_min_rect(cloud_projected_h, p0_h,p1_h,p2_h,p3_h);

          float min_z_h,max_z_h,avg_z_h;
          com_max_and_min_and_avg_z(cloud_projected_h,&min_z_h,&max_z_h,&avg_z_h);

          float cloud_z_h=min_z_h;

          if(max_z_h-avg_z_h<avg_z_h-min_z_h){
            cloud_z_h=max_z_h;
          }

          PointCloudPtr points_h(new PointCloud());
          points_h->push_back(Point(p0_h.x,p0_h.y,cloud_z_h));
          points_h->push_back(Point(p1_h.x,p1_h.y,cloud_z_h));
          points_h->push_back(Point(p2_h.x,p2_h.y,cloud_z_h));
          points_h->push_back(Point(p3_h.x,p3_h.y,cloud_z_h));

          Eigen::Matrix4d matrix_h_reverse;
          getRotationMatrix(axis_h, -angle_h, matrix_h_reverse);

          Eigen::Matrix4f matrix_transform_h_reverse = matrix_h_reverse.cast<float>();
          pcl::copyPointCloud(*points_h,points_temp);
          pcl::transformPointCloud (points_temp, *points_h, matrix_transform_h_reverse);

          MyPt p0_0={points0->points[0].x,points0->points[0].y,points0->points[0].z};
          MyPt p0_1={points0->points[1].x,points0->points[1].y,points0->points[1].z};
          MyPt p0_2={points0->points[2].x,points0->points[2].y,points0->points[2].z};
          MyPt p0_3={points0->points[3].x,points0->points[3].y,points0->points[3].z};
          MyPt p1_0={points_h->points[0].x,points_h->points[0].y,points_h->points[0].z};
          MyPt p1_1={points_h->points[1].x,points_h->points[1].y,points_h->points[1].z};
          MyPt p1_2={points_h->points[2].x,points_h->points[2].y,points_h->points[2].z};
          MyPt p1_3={points_h->points[3].x,points_h->points[3].y,points_h->points[3].z};

          if(testIntrRectangle3Rectangle3(p0_0,p0_1,p0_2,p0_3, p1_0,p1_1,p1_2,p1_3)){
            MyPointCloud mpc_h;
            PointCloud2MyPointCloud(points_h, mpc_h);
            horizontal_points.push_back(mpc_h);

            for(int k=0;k<plane_clouds.at(i).mypoints.size();k++){
              pit_tem.mypoints.push_back(plane_clouds.at(i).mypoints.at(k));
            }
          }
          else{
            new_plane_clouds.push_back(plane_clouds.at(i));
          }
        }
      }
    }


    if(pit_tem.mypoints.size()>0){
      for(int k=0;k<plane_clouds.at(index).mypoints.size();k++){
        pit_tem.mypoints.push_back(plane_clouds.at(index).mypoints.at(k));
      }

      new_plane_clouds.push_back(pit_tem);
    }
    else{
      new_plane_clouds.push_back(plane_clouds.at(index));
    }


    if(horizontal_points.size()>1){
      PointCloudPtr_RGB cloud_all_in_plane(new PointCloud_RGB());

      //cout<<"horizontal_points2.size():"<<horizontal_points2.size()<<endl;
      for(int i=0;i<horizontal_points.size();i++){

        //cout<<"plane_clouds.at(horizontal_index.at(i)).points.size():"<<plane_clouds.at(horizontal_index.at(i)).points.size()<<endl;
        for(int j=0;j<plane_clouds.at(horizontal_index.at(i)).mypoints.size();j++){

          Point_RGB point_tem;
          point_tem.x=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).x;
          point_tem.y=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).y;
          point_tem.z=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).z;
          point_tem.r=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).r;
          point_tem.g=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).g;
          point_tem.b=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).b;

          cloud_all_in_plane->points.push_back(point_tem);
        }
      }

      //showPointClound (cloud_all_in_plane,"cloud_all_in_plane");

      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      getPlaneByLeastSquare(cloud_all_in_plane , coefficients);

      //cout<<"coefficients*******************:"<<*coefficients<<endl;

      Eigen::Vector3d new_plane_normal;
      new_plane_normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
      new_plane_normal.normalize();

      double new_angle=acos(new_plane_normal.dot(Eigen::Vector3d(0,0,1)));
      Eigen::Vector3d new_axis=new_plane_normal.cross(Eigen::Vector3d(0,0,1));
      new_axis.normalize();

      Eigen::Matrix4d new_matrix;
      getRotationMatrix(new_axis, new_angle, new_matrix);

      Eigen::Matrix4f new_matrix_transform = new_matrix.cast<float>();
      pcl::copyPointCloud(*cloud_all_in_plane,cloud_temp);
      pcl::transformPointCloud (cloud_temp, *cloud_all_in_plane, new_matrix_transform);

      cv::Point2f new_p0;
      cv::Point2f new_p1;
      cv::Point2f new_p2;
      cv::Point2f new_p3;

      printf("000000000000000000001");
      find_min_rect(cloud_all_in_plane, new_p0,new_p1,new_p2,new_p3);

      float min_z_new,max_z_new,avg_z_new;
      com_max_and_min_and_avg_z(cloud_all_in_plane,&min_z_new,&max_z_new,&avg_z_new);

      float cloud_z_new=min_z_new;

      if(max_z_new-avg_z_new<avg_z_new-min_z_new){
        cloud_z_new=max_z_new;
      }

      PointCloudPtr new_points(new PointCloud());
      new_points->push_back(Point(new_p0.x,new_p0.y,cloud_z_new));
      new_points->push_back(Point(new_p1.x,new_p1.y,cloud_z_new));
      new_points->push_back(Point(new_p2.x,new_p2.y,cloud_z_new));
      new_points->push_back(Point(new_p3.x,new_p3.y,cloud_z_new));

      Eigen::Matrix4d new_matrix_reverse;
      getRotationMatrix(new_axis, -new_angle, new_matrix_reverse);

      Eigen::Matrix4f new_matrix_transform_reverse = new_matrix_reverse.cast<float>();
      pcl::copyPointCloud(*new_points,points_temp);
      pcl::transformPointCloud (points_temp, *new_points, new_matrix_transform_reverse);

      MyPointCloud new_mpc;
      PointCloud2MyPointCloud(new_points, new_mpc);
      rect_clouds.push_back(new_mpc);

      pcl::ModelCoefficients::Ptr mc(new pcl::ModelCoefficients ());
      for(int i=0;i<coefficients->values.size();i++){
        mc->values.push_back(coefficients->values[i]);
      }
      new_coefficientsPtr_vector.push_back(mc);

      for(int i=0;i<horizontal_points.size();i++){
        //coefficientsPtr_vector.pop_back(horizontal_index.at(i));
        coefficientsPtr_vector.at(horizontal_index.at(i))->values.clear();
        cout--;
      }

    }
    else{
      rect_clouds.push_back(horizontal_points.at(0));

      printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");

      pcl::ModelCoefficients::Ptr mc(new pcl::ModelCoefficients ());
      for(int i=0;i<coefficientsPtr_vector.at(index)->values.size();i++){
        mc->values.push_back(coefficientsPtr_vector.at(index)->values[i]);
      }
      new_coefficientsPtr_vector.push_back(mc);

      coefficientsPtr_vector.at(index)->values.clear();
      cout--;
    }
  }

  coefficients_vector.clear();
  for(int k=0;k<new_coefficientsPtr_vector.size();k++){
    coefficients_vector.push_back(*(new_coefficientsPtr_vector.at(k)));
  }

  //for debug
  for(int i=0;i<rect_clouds.size();i++){
    debug_clouds.push_back(rect_clouds.at(i));
  }
}

//VCCS over-segmentation
void VCCS_over_segmentation(PointCloudPtr_RGB cloud, NormalCloudTPtr normals, float voxel_resolution,float seed_resolution,float color_importance,float spatial_importance,float normal_importance,vector<MyPointCloud_RGB>& patch_clouds, PointCloudT::Ptr colored_cloud){
  PointCloudT::Ptr ct(new PointCloudT);

  for(int l=0;l<cloud->size();l++){
    PointT ptt;
    ptt.x=cloud->at(l).x;
    ptt.y=cloud->at(l).y;
    ptt.z=cloud->at(l).z;
    ptt.r=cloud->at(l).r;
    ptt.g=cloud->at(l).g;
    ptt.b=cloud->at(l).b;
    ptt.a=0;
    ct->push_back(ptt);
  }

  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution, false);
  super.setInputCloud (ct);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);

  super.setNormalCloud (normals);

  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

  printf("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);
  printf("Found %d supervoxels\n", supervoxel_clusters.size ());

  super.getPatchCloud(patch_clouds);

  PointCloudT::Ptr cvc = super.getColoredCloud();
  pcl::copyPointCloud(*cvc,*colored_cloud);
  //viewer->addPointCloud (colored_voxel_cloud, "colored voxels");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "colored voxels");
}

//merge plane in objects
void merge_Plane(std::vector<MyPointCloud_RGB>& plane_clouds, std::vector<pcl::ModelCoefficients> &coefficients_vector, std::vector<MyPointCloud_RGB>& new_plane_clouds, std::vector<MyPointCloud>& new_rect_clouds){

  PointCloudPtr_RGB cloud_f (new PointCloud_RGB);

  if(coefficients_vector.size()<2){
    if(coefficients_vector.size()==1){
      PointCloudPtr_RGB cloud_temp(new PointCloud_RGB);
      PointCloud points_temp;

      PointCloudPtr_RGB cloud_in_plane(new PointCloud_RGB());

      MyPointCloud_RGB2PointCloud(plane_clouds.at(0), cloud_in_plane);
      std::cout<<"plane_clouds.at(0).mypoints.size:"<< plane_clouds.at(0).mypoints.size() <<std::endl;
      std::cout<<"cloud_in_plane->size:"<< cloud_in_plane->size() <<std::endl;

      new_plane_clouds.push_back(plane_clouds.at(0));

      Eigen::Vector3d plane_normal;
      plane_normal << coefficients_vector.at(0).values[0], coefficients_vector.at(0).values[1], coefficients_vector.at(0).values[2];
      plane_normal.normalize();

      double angle=acos(plane_normal.dot(Eigen::Vector3d(0,0,1)));
      Eigen::Vector3d axis=plane_normal.cross(Eigen::Vector3d(0,0,1));
      axis.normalize();

      Eigen::Matrix4d matrix;
      getRotationMatrix(axis, angle, matrix);

      Eigen::Matrix4f matrix_transform = matrix.cast<float>();
      MyPointCloud_RGB2PointCloud(plane_clouds.at(0), cloud_temp);
      pcl::transformPointCloud (*cloud_temp, *cloud_in_plane, matrix_transform);

      cv::Point2f p0;
      cv::Point2f p1;
      cv::Point2f p2;
      cv::Point2f p3;

      std::cout<<"cloud_in_plane->size:"<< cloud_in_plane->size() <<std::endl;
      find_min_rect(cloud_in_plane, p0,p1,p2,p3);

      float min_z,max_z,avg_z;
      com_max_and_min_and_avg_z(cloud_in_plane,&min_z,&max_z,&avg_z);

      float cloud_z=min_z;

      if(max_z-avg_z<avg_z-min_z){
        cloud_z=max_z;
      }

      PointCloudPtr points(new PointCloud());
      points->push_back(Point(p0.x,p0.y,cloud_z));
      points->push_back(Point(p1.x,p1.y,cloud_z));
      points->push_back(Point(p2.x,p2.y,cloud_z));
      points->push_back(Point(p3.x,p3.y,cloud_z));

      Eigen::Matrix4d matrix_reverse;
      getRotationMatrix(axis, -angle, matrix_reverse);

      Eigen::Matrix4f matrix_transform_reverse = matrix_reverse.cast<float>();
      pcl::copyPointCloud(*points,points_temp);
      pcl::transformPointCloud (points_temp, *points, matrix_transform_reverse);

      MyPointCloud mpc;
      PointCloud2MyPointCloud(points, mpc);

      new_rect_clouds.push_back(mpc);
    }

    return;
  }

  std::vector<Eigen::Vector3d> normals;
  std::vector<pcl::ModelCoefficients::Ptr> coefficientsPtr_vector;
  std::vector<Eigen::Vector3d> new_normals;
  std::vector<pcl::ModelCoefficients::Ptr> new_coefficientsPtr_vector;
  std::vector<MyPointCloud> rect_clouds;

  for(int i=0;i<coefficients_vector.size();i++){
    Eigen::Vector3d plane_normal;
    plane_normal << coefficients_vector.at(i).values[0], coefficients_vector.at(i).values[1], coefficients_vector.at(i).values[2];
    plane_normal.normalize();
    normals.push_back(plane_normal);
  }

  for(int i=0;i<coefficients_vector.size();i++){
    pcl::ModelCoefficients::Ptr mcf(new pcl::ModelCoefficients ());
    for(int j=0;j<coefficients_vector.at(i).values.size();j++){
      mcf->values.push_back(coefficients_vector.at(i).values[j]);
    }
    coefficientsPtr_vector.push_back(mcf);
  }

  //===========For redundant and approximately parallel rectagles that intersects with each other, merge them by least squares.
  int cout = coefficientsPtr_vector.size();
  std::cout<<"&&&&&&&&&&&&&&&&&******cout"<<cout<<std::endl;

  while(cout >0){
    PointCloud_RGB cloud_temp;
    PointCloud points_temp;
    std::vector<MyPointCloud> horizontal_points;
    std::vector<int> horizontal_index;

    int index=0;
    for(int i=0;i<coefficientsPtr_vector.size();i++){
      if(coefficientsPtr_vector.at(i)->values.size()!=0){
        index=i;
        break;
      }
    }

    PointCloudPtr_RGB cloud_projected0(new PointCloud_RGB());
    //pcl::copyPointCloud(plane_clouds.at(index),*cloud_projected0);
    MyPointCloud_RGB2PointCloud(plane_clouds.at(index), cloud_projected0);
    std::cout<<"plane_clouds.at(index).mypoints.size:"<<plane_clouds.at(index).mypoints.size()<<std::endl;
    std::cout<<"cloud_projected0->size:"<<cloud_projected0->size()<<std::endl;
    //showPointCloud (cloud_projected0,"test");

    double angle0=acos(normals[index].dot(Eigen::Vector3d(0,0,1)));
    Eigen::Vector3d axis0=normals[index].cross(Eigen::Vector3d(0,0,1));
    axis0.normalize();

    Eigen::Matrix4d matrix0;
    getRotationMatrix(axis0, angle0, matrix0);

    Eigen::Matrix4f matrix_transform0 = matrix0.cast<float>();
    pcl::copyPointCloud(*cloud_projected0,cloud_temp);
    pcl::transformPointCloud (cloud_temp, *cloud_projected0, matrix_transform0);

    cv::Point2f p0_0;
    cv::Point2f p1_0;
    cv::Point2f p2_0;
    cv::Point2f p3_0;

    find_min_rect(cloud_projected0, p0_0,p1_0,p2_0,p3_0);

    float min_z0,max_z0,avg_z0;
    com_max_and_min_and_avg_z(cloud_projected0,&min_z0,&max_z0,&avg_z0);

    float cloud_z0=min_z0;

    if(max_z0-avg_z0<avg_z0-min_z0){
      cloud_z0=max_z0;
    }

    std::cout<<"cloud_z0:"<<cloud_z0<<std::endl;
    std::cout<<"cloud_projected0->points[0].z:"<<cloud_projected0->points[0].z<<std::endl;

    PointCloudPtr points0(new PointCloud());
    points0->push_back(Point(p0_0.x,p0_0.y,cloud_z0));
    points0->push_back(Point(p1_0.x,p1_0.y,cloud_z0));
    points0->push_back(Point(p2_0.x,p2_0.y,cloud_z0));
    points0->push_back(Point(p3_0.x,p3_0.y,cloud_z0));

    Eigen::Matrix4d matrix0_reverse;
    getRotationMatrix(axis0, -angle0, matrix0_reverse);

    Eigen::Matrix4f matrix_transform0_reverse = matrix0_reverse.cast<float>();
    pcl::copyPointCloud(*points0,points_temp);
    pcl::transformPointCloud (points_temp, *points0, matrix_transform0_reverse);

    MyPointCloud mpc0;
    PointCloud2MyPointCloud(points0, mpc0);
    horizontal_points.push_back(mpc0);
    horizontal_index.push_back(index);

    MyPointCloud_RGB pit_tem;

    for(int i=index+1;i<coefficientsPtr_vector.size();i++){

      if(coefficientsPtr_vector.at(i)->values.size()!=0){

        //horizontal
        if(std::abs(normals[index].dot(normals[i])-1)<0.03){

          horizontal_index.push_back(i);

          PointCloudPtr_RGB cloud_projected_h(new PointCloud_RGB());
          //pcl::copyPointCloud(plane_clouds.at(i),*cloud_projected_h);
          MyPointCloud_RGB2PointCloud(plane_clouds.at(i), cloud_projected_h);

          double angle_h=acos(normals[i].dot(Eigen::Vector3d(0,0,1)));
          Eigen::Vector3d axis_h=normals[i].cross(Eigen::Vector3d(0,0,1));
          axis_h.normalize();

          Eigen::Matrix4d matrix_h;
          getRotationMatrix(axis_h, angle_h, matrix_h);

          Eigen::Matrix4f matrix_transform_h = matrix_h.cast<float>();
          pcl::copyPointCloud(*cloud_projected_h,cloud_temp);
          pcl::transformPointCloud (cloud_temp, *cloud_projected_h, matrix_transform_h);

          cv::Point2f p0_h;
          cv::Point2f p1_h;
          cv::Point2f p2_h;
          cv::Point2f p3_h;

          find_min_rect(cloud_projected_h, p0_h,p1_h,p2_h,p3_h);

          float min_z_h,max_z_h,avg_z_h;
          com_max_and_min_and_avg_z(cloud_projected_h,&min_z_h,&max_z_h,&avg_z_h);

          float cloud_z_h=min_z_h;

          if(max_z_h-avg_z_h<avg_z_h-min_z_h){
            cloud_z_h=max_z_h;
          }

          PointCloudPtr points_h(new PointCloud());
          points_h->push_back(Point(p0_h.x,p0_h.y,cloud_z_h));
          points_h->push_back(Point(p1_h.x,p1_h.y,cloud_z_h));
          points_h->push_back(Point(p2_h.x,p2_h.y,cloud_z_h));
          points_h->push_back(Point(p3_h.x,p3_h.y,cloud_z_h));

          Eigen::Matrix4d matrix_h_reverse;
          getRotationMatrix(axis_h, -angle_h, matrix_h_reverse);

          Eigen::Matrix4f matrix_transform_h_reverse = matrix_h_reverse.cast<float>();
          pcl::copyPointCloud(*points_h,points_temp);
          pcl::transformPointCloud (points_temp, *points_h, matrix_transform_h_reverse);

          MyPt p0_0={points0->points[0].x,points0->points[0].y,points0->points[0].z};
          MyPt p0_1={points0->points[1].x,points0->points[1].y,points0->points[1].z};
          MyPt p0_2={points0->points[2].x,points0->points[2].y,points0->points[2].z};
          MyPt p0_3={points0->points[3].x,points0->points[3].y,points0->points[3].z};
          MyPt p1_0={points_h->points[0].x,points_h->points[0].y,points_h->points[0].z};
          MyPt p1_1={points_h->points[1].x,points_h->points[1].y,points_h->points[1].z};
          MyPt p1_2={points_h->points[2].x,points_h->points[2].y,points_h->points[2].z};
          MyPt p1_3={points_h->points[3].x,points_h->points[3].y,points_h->points[3].z};

          if(testIntrRectangle3Rectangle3(p0_0,p0_1,p0_2,p0_3, p1_0,p1_1,p1_2,p1_3)){
            MyPointCloud mpc_h;
            PointCloud2MyPointCloud(points_h, mpc_h);
            horizontal_points.push_back(mpc_h);

            for(int k=0;k<plane_clouds.at(i).mypoints.size();k++){
              pit_tem.mypoints.push_back(plane_clouds.at(i).mypoints.at(k));
            }
          }
          else{
            new_plane_clouds.push_back(plane_clouds.at(i));
          }
        }
      }
    }

    if(pit_tem.mypoints.size()>0){
      for(int k=0;k<plane_clouds.at(index).mypoints.size();k++){
        pit_tem.mypoints.push_back(plane_clouds.at(index).mypoints.at(k));
      }

      new_plane_clouds.push_back(pit_tem);
    }
    else{
      new_plane_clouds.push_back(plane_clouds.at(index));
    }


    if(horizontal_points.size()>1){
      PointCloudPtr_RGB cloud_all_in_plane(new PointCloud_RGB());

      //cout<<"horizontal_points2.size():"<<horizontal_points2.size()<<endl;
      for(int i=0;i<horizontal_points.size();i++){

        //cout<<"plane_clouds.at(horizontal_index.at(i)).points.size():"<<plane_clouds.at(horizontal_index.at(i)).points.size()<<endl;
        for(int j=0;j<plane_clouds.at(horizontal_index.at(i)).mypoints.size();j++){

          Point_RGB point_tem;
          point_tem.x=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).x;
          point_tem.y=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).y;
          point_tem.z=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).z;
          point_tem.r=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).r;
          point_tem.g=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).g;
          point_tem.b=plane_clouds.at(horizontal_index.at(i)).mypoints.at(j).b;

          cloud_all_in_plane->points.push_back(point_tem);
        }
      }

      //showPointClound (cloud_all_in_plane,"cloud_all_in_plane");

      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      getPlaneByLeastSquare(cloud_all_in_plane , coefficients);

      //cout<<"coefficients*******************:"<<*coefficients<<endl;

      Eigen::Vector3d new_plane_normal;
      new_plane_normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
      new_plane_normal.normalize();

      double new_angle=acos(new_plane_normal.dot(Eigen::Vector3d(0,0,1)));
      Eigen::Vector3d new_axis=new_plane_normal.cross(Eigen::Vector3d(0,0,1));
      new_axis.normalize();

      Eigen::Matrix4d new_matrix;
      getRotationMatrix(new_axis, new_angle, new_matrix);

      Eigen::Matrix4f new_matrix_transform = new_matrix.cast<float>();
      pcl::copyPointCloud(*cloud_all_in_plane,cloud_temp);
      pcl::transformPointCloud (cloud_temp, *cloud_all_in_plane, new_matrix_transform);

      cv::Point2f new_p0;
      cv::Point2f new_p1;
      cv::Point2f new_p2;
      cv::Point2f new_p3;

      find_min_rect(cloud_all_in_plane, new_p0,new_p1,new_p2,new_p3);

      float min_z_new,max_z_new,avg_z_new;
      com_max_and_min_and_avg_z(cloud_all_in_plane,&min_z_new,&max_z_new,&avg_z_new);

      float cloud_z_new=min_z_new;

      if(max_z_new-avg_z_new<avg_z_new-min_z_new){
        cloud_z_new=max_z_new;
      }

      PointCloudPtr new_points(new PointCloud());
      new_points->push_back(Point(new_p0.x,new_p0.y,cloud_z_new));
      new_points->push_back(Point(new_p1.x,new_p1.y,cloud_z_new));
      new_points->push_back(Point(new_p2.x,new_p2.y,cloud_z_new));
      new_points->push_back(Point(new_p3.x,new_p3.y,cloud_z_new));

      Eigen::Matrix4d new_matrix_reverse;
      getRotationMatrix(new_axis, -new_angle, new_matrix_reverse);

      Eigen::Matrix4f new_matrix_transform_reverse = new_matrix_reverse.cast<float>();
      pcl::copyPointCloud(*new_points,points_temp);
      pcl::transformPointCloud (points_temp, *new_points, new_matrix_transform_reverse);

      MyPointCloud new_mpc;
      PointCloud2MyPointCloud(new_points, new_mpc);
      rect_clouds.push_back(new_mpc);

      pcl::ModelCoefficients::Ptr mc(new pcl::ModelCoefficients ());
      for(int i=0;i<coefficients->values.size();i++){
        mc->values.push_back(coefficients->values[i]);
      }
      new_coefficientsPtr_vector.push_back(mc);

      for(int i=0;i<horizontal_points.size();i++){
        //coefficientsPtr_vector.pop_back(horizontal_index.at(i));
        coefficientsPtr_vector.at(horizontal_index.at(i))->values.clear();
        cout--;
      }

    }
    else{
      rect_clouds.push_back(horizontal_points.at(0));

      pcl::ModelCoefficients::Ptr mc(new pcl::ModelCoefficients ());
      for(int i=0;i<coefficientsPtr_vector.at(index)->values.size();i++){
        mc->values.push_back(coefficientsPtr_vector.at(index)->values[i]);
      }
      new_coefficientsPtr_vector.push_back(mc);

      coefficientsPtr_vector.at(index)->values.clear();
      cout--;
    }
  }

  coefficients_vector.clear();
  for(int k=0;k<new_coefficientsPtr_vector.size();k++){
    coefficients_vector.push_back(*(new_coefficientsPtr_vector.at(k)));
  }

  for(int i=0;i<rect_clouds.size();i++){
    new_rect_clouds.push_back(rect_clouds.at(i));
  }
}

//object fitting
void object_fitting(PointCloudPtr_RGB cloud, vector<MyPointCloud_RGB> &plane_clouds, std::vector<MyPointCloud> &rect_clouds, vector<MyPointCloud_RGB> &cylinder_clouds, vector<MyPointCloud_RGB> &sphere_clouds, PointCloudPtr_RGB remained_cloud){
  PointCloudPtr_RGB cloud_tem(new PointCloud_RGB);
  pcl::copyPointCloud(*cloud, *cloud_tem);

  std::vector<pcl::ModelCoefficients> plane_coefficients_vector;
  vector<MyPointCloud_RGB> plane_clouds_tem;

  while(1){
    PointCloudPtr_RGB remained_tem0(new PointCloud_RGB);
    PointCloudPtr_RGB remained_tem1(new PointCloud_RGB);
    PointCloudPtr_RGB remained_tem2(new PointCloud_RGB);

    MyPointCloud_RGB cylinder_cloud;
    MyPointCloud_RGB sphere_cloud;
    MyPointCloud_RGB plane_cloud;

    MyPointCloud cylinder_cloud_n;
    MyPointCloud sphere_cloud_n;
    MyPointCloud plane_cloud_n;
    MyPointCloud rect_cloud;

    pcl::ModelCoefficients::Ptr cylinder_coefficients(new pcl::ModelCoefficients());
    pcl::ModelCoefficients::Ptr sphere_coefficients(new pcl::ModelCoefficients());
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());

    float result0=0;
    float result1=0;
    float result2=0;

    cylinder_fitting(cloud_tem, cylinder_cloud, cylinder_coefficients,remained_tem0);
    sphere_fitting(cloud_tem, sphere_cloud, sphere_coefficients, remained_tem1);
    plane_for_boxs_fitting(cloud_tem, plane_cloud, rect_cloud, plane_coefficients, remained_tem2);

    MyPointCloud_RGB2MyPointCloud(cylinder_cloud, cylinder_cloud_n);
    MyPointCloud_RGB2MyPointCloud(sphere_cloud, sphere_cloud_n);
    MyPointCloud_RGB2MyPointCloud(plane_cloud, plane_cloud_n);

    if(cylinder_cloud.mypoints.size()==0&&sphere_cloud.mypoints.size()==0&&plane_cloud.mypoints.size()==0){
      printf("Can't fit any more\n");
      break;
    }

    if(cylinder_cloud.mypoints.size()>0){
      Point cenPoint0(cylinder_coefficients->values[0],cylinder_coefficients->values[1],cylinder_coefficients->values[2]);
      Point cenPoint1(cylinder_coefficients->values[7],cylinder_coefficients->values[8],cylinder_coefficients->values[9]);
      Vector3<float> direction(cylinder_coefficients->values[3],cylinder_coefficients->values[4],cylinder_coefficients->values[5]);

      computeCylinderJaccardIndex(cylinder_cloud_n, cenPoint0, cenPoint1, direction, cylinder_coefficients->values[6], 0.002, &result0);
    }

    if(sphere_cloud.mypoints.size()>0){
      Point cenPoint(sphere_coefficients->values[0],sphere_coefficients->values[1],sphere_coefficients->values[2]);
      computeSphereJaccardIndex(sphere_cloud_n, cenPoint, sphere_coefficients->values[3], 0.002, &result1);
    }

    if(plane_cloud.mypoints.size()>0){
      computePlaneJaccardIndex(plane_cloud_n, rect_cloud, 0.002, &result2);
    }

    cout<<"result0=====================:"<<result0<<endl;
    cout<<"result1=====================:"<<result1<<endl;
    cout<<"result2=====================:"<<result2<<endl;

    float thresdhold=0.17;

    if(result0<thresdhold&&result1<thresdhold&&result2<thresdhold){
      printf("Can't fit any more\n");
      break;
    }

    //use cylinder fitting
    if(result0>result1&&result0>result2&&result0>thresdhold){
      pcl::copyPointCloud(*remained_tem0, *cloud_tem);
      cylinder_clouds.push_back(cylinder_cloud);

      /*PointCloudPtr_RGB cloud_in_cylinder(new PointCloud_RGB());
      MyPointCloud_RGB2PointCloud(cylinder_cloud, cloud_in_cylinder);
      showPointCloud(cloud_in_cylinder,"cloud_in_cylinder");*/
    }

    //use sphere fitting
    if(result1>result0&&result1>result2&&result1>thresdhold){
      pcl::copyPointCloud(*remained_tem1, *cloud_tem);
      sphere_clouds.push_back(sphere_cloud);
    }

    //use plane fitting
    if(result2>result0&&result2>result1&&result2>thresdhold){
      pcl::copyPointCloud(*remained_tem2, *cloud_tem);
      plane_clouds_tem.push_back(plane_cloud);
      plane_coefficients_vector.push_back(*plane_coefficients);
    }
  }

  merge_Plane(plane_clouds_tem, plane_coefficients_vector, plane_clouds, rect_clouds);

  pcl::copyPointCloud(*cloud_tem, *remained_cloud);
}