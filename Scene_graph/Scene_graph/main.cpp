#include "visualizer.h"
#include "color_op.h"
#include "file_io.h"
#include "scene_seg.h"

int main (int argc, char *argv[])
{
  Visualizer vs;
  vs.viewer->removeAllPointClouds();
  vs.viewer->removeCoordinateSystem();

  PointCloudPtr_RGB cloud(new PointCloud_RGB);
  loadPointCloud_ply("E:/HaoLi/MyWorkspace/Scene_graph/Scene_graph/data/testData.ply", cloud);
  //loadPointCloud_ply("/home/hao/Desktop/scene_data/scene03.ply", cloud);
  PointCloudPtr_RGB cloud_mark(new PointCloud_RGB);
  pcl::copyPointCloud(*cloud,*cloud_mark);

  /******************detect table************************/
  PointCloudPtr_RGB tabletopCloud(new PointCloud_RGB());
  PointCloudPtr_RGB planeCloud(new PointCloud_RGB());
  detect_table_plane_r(cloud, planeCloud, tabletopCloud);
  //detect_table_plane(cloud, planeCloud, tabletopCloud);
  //showPointClound(planeCloud,"planeCloud");

  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  detect_table(cloud, coefficients_plane, inliers_plane);

  PointCloudPtr_RGB cloud_tem(new PointCloud_RGB());

  pcl::ExtractIndices<Point_RGB> extract0;// Create the filtering object
  // Extract the inliers
  extract0.setInputCloud (cloud);
  extract0.setIndices (inliers_plane);
  extract0.setNegative (false);
  extract0.filter (*cloud_tem);

  cv::Point2f p0;
  cv::Point2f p1;
  cv::Point2f p2;
  cv::Point2f p3;

  find_min_rect(cloud_tem, p0, p1, p2, p3);

  /******************Euclidean Cluster Extraction************************/
  std::vector<PointCloudPtr_RGB> cluster_points;

  object_seg_ECE(tabletopCloud, cluster_points);

  PointCloudPtr_RGB remaining_cloud(new PointCloud_RGB());

  for(int i=0;i<cluster_points.size();i++){
    vector<MyPointCloud_RGB> plane_clouds;
    vector<MyPointCloud> rect_clouds;
    vector<MyPointCloud_RGB> cylinder_clouds;
    vector<MyPointCloud_RGB> sphere_clouds;

    object_fitting(cluster_points.at(i), plane_clouds, rect_clouds, cylinder_clouds, sphere_clouds, remaining_cloud);

    //color cloud
    std::vector<ColorType> colors;
    std::vector<ColorType> excludeColors;
    excludeColors.push_back(ColorType(0,0,0));
    excludeColors.push_back(ColorType(255,255,255));
    int sum_num=cylinder_clouds.size()+sphere_clouds.size()+plane_clouds.size();
    if(sum_num>0){
      //getUniqueColors(sum_num, colors, excludeColors);
      getDiffColors(sum_num, colors);
    }

    cout<<"sum_num:"<<sum_num<<endl;
    cout<<"colors.size:"<<colors.size()<<endl;

    int index=0;

    for(int m=0;m<cylinder_clouds.size();m++){
      PointCloudPtr_RGB cloud_in_cylinder(new PointCloud_RGB());
      MyPointCloud_RGB2PointCloud(cylinder_clouds.at(m), cloud_in_cylinder);

      for(int n=0;n<cloud_in_cylinder->size();n++){
        //cloud_in_cylinder->at(n).r=colors.at(index).mRed;
        //cloud_in_cylinder->at(n).g=colors.at(index).mGreen;
        //cloud_in_cylinder->at(n).b=colors.at(index).mBlue;

        cloud_in_cylinder->at(n).r=255;
        cloud_in_cylinder->at(n).g=0;
        cloud_in_cylinder->at(n).b=0;
      }

      index++;

      std::stringstream str;
      str<<"cloud_in_cylinder"<<i<<m;
      std::string id_pc=str.str();

      vs.viewer->addPointCloud (cloud_in_cylinder, id_pc);
    }

    for(int m=0;m<sphere_clouds.size();m++){
      PointCloudPtr_RGB cloud_in_sphere(new PointCloud_RGB());
      MyPointCloud_RGB2PointCloud(sphere_clouds.at(m), cloud_in_sphere);

      for(int n=0;n<cloud_in_sphere->size();n++){
        //cloud_in_sphere->at(n).r=colors.at(index).mRed;
        //cloud_in_sphere->at(n).g=colors.at(index).mGreen;
        //cloud_in_sphere->at(n).b=colors.at(index).mBlue;

        cloud_in_sphere->at(n).r=0;
        cloud_in_sphere->at(n).g=255;
        cloud_in_sphere->at(n).b=0;
      }

      index++;

      std::stringstream str;
      str<<"cloud_in_sphere"<<i<<m;
      std::string id_pc=str.str();

      vs.viewer->addPointCloud (cloud_in_sphere, id_pc);
    }

    for(int m=0;m<plane_clouds.size();m++){
      PointCloudPtr_RGB cloud_in_plane(new PointCloud_RGB());
      MyPointCloud_RGB2PointCloud(plane_clouds.at(m), cloud_in_plane);

      for(int n=0;n<cloud_in_plane->size();n++){
        //cloud_in_plane->at(n).r=colors.at(index).mRed;
        //cloud_in_plane->at(n).g=colors.at(index).mGreen;
        //cloud_in_plane->at(n).b=colors.at(index).mBlue;

        cloud_in_plane->at(n).r=0;
        cloud_in_plane->at(n).g=0;
        cloud_in_plane->at(n).b=255;
      }

      cout<<"colors.at(index).mRed:"<<colors.at(index).mRed<<" "<<"colors.at(index).mGreen:"<<colors.at(index).mGreen<<" "<<"colors.at(index).mBlue:"<<colors.at(index).mBlue<<endl;

      index++;

      std::stringstream str;
      str<<"cloud_in_plane"<<i<<m;
      std::string id_pc=str.str();

      vs.viewer->addPointCloud (cloud_in_plane, id_pc);
    }

    //for remaining clouds, do over-segmentation
    float voxel_resolution = 0.008f;
    float seed_resolution = 0.08f;
    float color_importance = 0.2f;
    float spatial_importance = 0.4f;
    float normal_importance = 1.0f;

    PointCloudT::Ptr colored_voxel_cloud(new PointCloudT);
    VCCS_over_segmentation(remaining_cloud, voxel_resolution,seed_resolution,color_importance,spatial_importance,normal_importance, colored_voxel_cloud);

    /*std::stringstream str;
    str<<"colored_voxel_cloud"<<i;
    std::string id_pc=str.str();

    vs.viewer->addPointCloud (colored_voxel_cloud, id_pc);*/
  }

  vs.show();

  return 0;
}
