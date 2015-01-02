//#include "visualizer.h"
//#include "color_op.h"
//#include "file_io.h"
//#include "scene_seg.h"
//
//int main (int argc, char *argv[])
//{
//  Visualizer vs;
//  vs.viewer->removeAllPointClouds();
//  vs.viewer->removeCoordinateSystem();
//  vs.viewer->setBackgroundColor(0,0,0);
//
//  PointCloudPtr_RGB_NORMAL cloud(new PointCloud_RGB_NORMAL);
//  loadPointCloud_normal_ply("data/big_table_normal.ply", cloud);
//  //loadPointCloud_normal_ply("data/table3.ply", cloud);
//  //loadPointCloud_normal_ply("data/big_table_normal.ply", cloud);
//  //loadPointCloud_normal_ply("data/big_table_normal.ply", cloud);
//
//  PointCloudPtr_RGB cc(new PointCloud_RGB);
//
//  for(int i=0;i<cloud->size();i++){
//    Point_RGB pr;
//    pr.x=cloud->at(i).x;
//    pr.y=cloud->at(i).y;
//    pr.z=cloud->at(i).z;
//    pr.r=cloud->at(i).r;
//    pr.g=cloud->at(i).g;
//    pr.b=cloud->at(i).b;
//    cc->push_back(pr);
//  }
//
//  showPointCloud(cc,"cloud");
//
//  PointCloudPtr_RGB_NORMAL cloud_mark(new PointCloud_RGB_NORMAL);
//  pcl::copyPointCloud(*cloud,*cloud_mark);
//
//  /******************detect table************************/
//  PointCloudPtr_RGB_NORMAL tabletopCloud(new PointCloud_RGB_NORMAL());
//  PointCloudPtr_RGB_NORMAL planeCloud(new PointCloud_RGB_NORMAL());
//  //detect_table_plane_r(cloud, planeCloud, tabletopCloud);
//  detect_table_plane(cloud, planeCloud, tabletopCloud);
//  //showPointClound(planeCloud,"planeCloud");
//
//  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
//  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
//  detect_table(cloud, coefficients_plane, inliers_plane);
//
//  PointCloudPtr_RGB_NORMAL table_cloud(new PointCloud_RGB_NORMAL());
//  pcl::ExtractIndices<Point_RGB_NORMAL> extract0;// Create the filtering object
//  // Extract the inliers
//  extract0.setInputCloud (cloud);
//  extract0.setIndices (inliers_plane);
//  extract0.setNegative (false);
//  extract0.filter (*table_cloud);
//
//  PointCloudPtr_RGB pc(new PointCloud_RGB);
//
//  for(int i=0;i<table_cloud->size();i++){
//    Point_RGB pr;
//    pr.x=table_cloud->at(i).x;
//    pr.y=table_cloud->at(i).y;
//    pr.z=table_cloud->at(i).z;
//    pr.r=table_cloud->at(i).r;
//    pr.g=table_cloud->at(i).g;
//    pr.b=table_cloud->at(i).b;
//    pc->push_back(pr);
//  }
//
//  showPointCloud (pc,"table_cloud");
//
//  vs.viewer->addPointCloud (pc, "table_cloud");
//
//  //  cBinarySeg.AddTable(table_cloud);
//
//  //cv::Point2f p0;
//  //cv::Point2f p1;
//  //cv::Point2f p2;
//  //cv::Point2f p3;
//
//  //find_min_rect(table_cloud, p0, p1, p2, p3);
//
//  /******************Euclidean Cluster Extraction************************/
//  std::vector<PointCloudPtr_RGB_NORMAL> cluster_points;
//
//  object_seg_ECE(tabletopCloud, cluster_points);
//
//  PointCloudPtr_RGB_NORMAL remaining_cloud(new PointCloud_RGB_NORMAL());
//
//  for(int i=0;i<cluster_points.size();i++){
//    vector<MyPointCloud_RGB_NORMAL> plane_clouds;
//    vector<MyPointCloud> rect_clouds;
//    vector<MyPointCloud_RGB_NORMAL> cylinder_clouds;
//    vector<MyPointCloud_RGB_NORMAL> sphere_clouds;
//
//    object_fitting(cluster_points.at(i), plane_clouds, rect_clouds, cylinder_clouds, sphere_clouds, remaining_cloud);
//
//    //color cloud
//    std::vector<ColorType> colors;
//    std::vector<ColorType> excludeColors;
//    excludeColors.push_back(ColorType(0,0,0));
//    excludeColors.push_back(ColorType(255,255,255));
//    int sum_num=cylinder_clouds.size()+sphere_clouds.size()+plane_clouds.size();
//    if(sum_num>0){
//      //getUniqueColors(sum_num, colors, excludeColors);
//      getDiffColors(sum_num, colors);
//    }
//
//    cout<<"sum_num:"<<sum_num<<endl;
//    cout<<"colors.size:"<<colors.size()<<endl;
//
//    int index=0;
//
//    for(int m=0;m<cylinder_clouds.size();m++){
//
//      PointCloudPtr_RGB_NORMAL cloud_in_cylinder(new PointCloud_RGB_NORMAL());
//      MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(cylinder_clouds.at(m), cloud_in_cylinder);
//
//      for(int n=0;n<cloud_in_cylinder->size();n++){
//        cloud_in_cylinder->at(n).r=colors.at(index).mRed;
//        cloud_in_cylinder->at(n).g=colors.at(index).mGreen;
//        cloud_in_cylinder->at(n).b=colors.at(index).mBlue;
//
//        /*cloud_in_cylinder->at(n).r=255;
//        cloud_in_cylinder->at(n).g=255;
//        cloud_in_cylinder->at(n).b=255;*/
//      }
//
//      index++;
//
//      std::stringstream str;
//      str<<"cloud_in_cylinder"<<i<<m;
//      std::string id_pc=str.str();
//
//      PointCloudPtr_RGB pc_cylinder(new PointCloud_RGB);
//      for(int i=0;i<cloud_in_cylinder->size();i++){
//        Point_RGB pr;
//        pr.x=cloud_in_cylinder->at(i).x;
//        pr.y=cloud_in_cylinder->at(i).y;
//        pr.z=cloud_in_cylinder->at(i).z;
//        pr.r=cloud_in_cylinder->at(i).r;
//        pr.g=cloud_in_cylinder->at(i).g;
//        pr.b=cloud_in_cylinder->at(i).b;
//        pc_cylinder->push_back(pr);
//      }
//      vs.viewer->addPointCloud (pc_cylinder, id_pc);
//    }
//
//    for(int m=0;m<sphere_clouds.size();m++){
//      PointCloudPtr_RGB_NORMAL cloud_in_sphere(new PointCloud_RGB_NORMAL());
//      MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(sphere_clouds.at(m), cloud_in_sphere);
//
//      for(int n=0;n<cloud_in_sphere->size();n++){
//        cloud_in_sphere->at(n).r=colors.at(index).mRed;
//        cloud_in_sphere->at(n).g=colors.at(index).mGreen;
//        cloud_in_sphere->at(n).b=colors.at(index).mBlue;
//
//        /*cloud_in_sphere->at(n).r=0;
//        cloud_in_sphere->at(n).g=255;
//        cloud_in_sphere->at(n).b=0;*/
//      }
//
//      index++;
//
//      std::stringstream str;
//      str<<"cloud_in_sphere"<<i<<m;
//      std::string id_pc=str.str();
//
//      PointCloudPtr_RGB pc_sphere(new PointCloud_RGB);
//      for(int i=0;i<cloud_in_sphere->size();i++){
//        Point_RGB pr;
//        pr.x=cloud_in_sphere->at(i).x;
//        pr.y=cloud_in_sphere->at(i).y;
//        pr.z=cloud_in_sphere->at(i).z;
//        pr.r=cloud_in_sphere->at(i).r;
//        pr.g=cloud_in_sphere->at(i).g;
//        pr.b=cloud_in_sphere->at(i).b;
//        pc_sphere->push_back(pr);
//      }
//
//      vs.viewer->addPointCloud (pc_sphere, id_pc);
//    }
//
//    for(int m=0;m<plane_clouds.size();m++){
//      PointCloudPtr_RGB_NORMAL cloud_in_plane(new PointCloud_RGB_NORMAL());
//      MyPointCloud_RGB_NORMAL2PointCloud_RGB_NORMAL(plane_clouds.at(m), cloud_in_plane);
//
//      for(int n=0;n<cloud_in_plane->size();n++){
//        cloud_in_plane->at(n).r=colors.at(index).mRed;
//        cloud_in_plane->at(n).g=colors.at(index).mGreen;
//        cloud_in_plane->at(n).b=colors.at(index).mBlue;
//
//        /*cloud_in_plane->at(n).r=0;
//        cloud_in_plane->at(n).g=0;
//        cloud_in_plane->at(n).b=255;*/
//      }
//
//      index++;
//
//      std::stringstream str;
//      str<<"cloud_in_plane"<<i<<m;
//      std::string id_pc=str.str();
//
//      PointCloudPtr_RGB pc_plane(new PointCloud_RGB);
//      for(int i=0;i<cloud_in_plane->size();i++){
//        Point_RGB pr;
//        pr.x=cloud_in_plane->at(i).x;
//        pr.y=cloud_in_plane->at(i).y;
//        pr.z=cloud_in_plane->at(i).z;
//        pr.r=cloud_in_plane->at(i).r;
//        pr.g=cloud_in_plane->at(i).g;
//        pr.b=cloud_in_plane->at(i).b;
//        pc_plane->push_back(pr);
//      }
//
//      vs.viewer->addPointCloud (pc_plane, id_pc);
//    }
//
//    //for remaining clouds, do over-segmentation
//    float voxel_resolution = 0.006f;
//    float seed_resolution = 0.06f;
//    float color_importance = 0.2f;
//    float spatial_importance = 0.4f;
//    float normal_importance = 1.0f;
//
//    PointCloudT::Ptr colored_cloud(new PointCloudT);
//    vector<MyPointCloud_RGB_NORMAL> patch_clouds;
//    PointNCloudT::Ptr normal_cloud(new PointNCloudT);
//    VCCS_over_segmentation(remaining_cloud,voxel_resolution,seed_resolution,color_importance,spatial_importance,normal_importance,patch_clouds,colored_cloud,normal_cloud);
//
//    std::stringstream str;
//    str<<"colored_voxel_cloud"<<i;
//    std::string id_pc=str.str();
//
//    vs.viewer->addPointCloud (colored_cloud, id_pc);
//
//    cout<<"rect_clouds.size()==========================================:"<<rect_clouds.size()<<endl;
//
//    /*for(int k=0;k<rect_clouds.size();k++){
//    PointCloudPtr pc(new PointCloud);
//    MyPointCloud2PointCloud(rect_clouds.at(k), pc);
//
//    std::stringstream st0;
//    std::stringstream st1;
//    std::stringstream st2;
//    std::stringstream st3;
//
//    st0<<"a"<<i<<k<<"0";
//    st1<<"a"<<i<<k<<"1";
//    st2<<"a"<<i<<k<<"2";
//    st3<<"a"<<i<<k<<"3";
//
//    std::string id_line0=st0.str();
//    std::string id_line1=st1.str();
//    std::string id_line2=st2.str();
//    std::string id_line3=st3.str();
//
//    vs.viewer->addLine(pc->at(0),pc->at(1),255,0,0,id_line0);
//    vs.viewer->addLine(pc->at(1),pc->at(2),255,0,0,id_line1);
//    vs.viewer->addLine(pc->at(2),pc->at(3),255,0,0,id_line2);
//    vs.viewer->addLine(pc->at(3),pc->at(0),255,0,0,id_line3);
//    }*/
//  }
//
//  vs.show();
//
//  return 0;
//}
