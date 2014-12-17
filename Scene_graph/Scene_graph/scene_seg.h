#ifndef SCENE_SEG_H
#define SCENE_SEG_H

#include "common_type.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

typedef struct TriFace{
Point p0;
Point p1;
Point p2;
}Face;

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

#define PI 3.1415926535

using namespace std;
using namespace Wm5;

//show rgb cloud
void showPointCloud (PointCloudPtr_RGB cloud,std::string name);
//show cloud
void showPointCloud2 (PointCloudPtr cloud,std::string name);
//detect table
void detect_table(PointCloudPtr_RGB sourceCloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers);
//detect table plane
void detect_table_plane(PointCloudPtr_RGB sourceCloud, PointCloudPtr_RGB planeCloud, PointCloudPtr_RGB remainCloud);
//detect table plane
void detect_table_plane_r(PointCloudPtr_RGB sourceCloud, PointCloudPtr_RGB planeCloud, PointCloudPtr_RGB remainCloud);
//Euclidean Cluster Extraction
void object_seg_ECE(PointCloudPtr_RGB clound, std::vector<PointCloudPtr_RGB> &cluster_points);
//segment cylinder from the data
void object_seg_Cylinder(PointCloudPtr_RGB cloud, std::vector<MyPointCloud_RGB> &cluster_points, std::vector<pcl::ModelCoefficients> &coefficients,PointCloudPtr_RGB remained_cloud);
//segment sphere from the data
void object_seg_Sphere(PointCloudPtr_RGB cloud, std::vector<MyPointCloud_RGB> &cluster_points, std::vector<pcl::ModelCoefficients> &coefficients,PointCloudPtr_RGB remained_cloud);
//seg box in objects
void object_seg_Box(PointCloudPtr_RGB sourceCloud, std::vector<pcl::ModelCoefficients> &coefficients_vector, std::vector<MyPointCloud_RGB>& plane_clouds, std::vector<MyPointCloud> &clouds, std::vector<MyPointCloud> &debug_clouds,PointCloudPtr_RGB remained_cloud);
//find a minimum bounding rect
void find_min_rect(PointCloudPtr_RGB cloud, cv::Point2f &p0,cv::Point2f &p1,cv::Point2f &p2,cv::Point2f &p3);
//seg plane in objects
void object_seg_Plane(PointCloudPtr_RGB sourceCloud, std::vector<pcl::ModelCoefficients> &coefficients_vector, std::vector<MyPointCloud_RGB>& new_plane_clouds, std::vector<MyPointCloud> &debug_clouds,PointCloudPtr_RGB remained_cloud);
//VCCS over-segmentation
void VCCS_over_segmentation(PointCloudPtr_RGB cloud, float voxel_resolution,float seed_resolution,float color_importance,float spatial_importance,float normal_importance,PointCloudT::Ptr colored_voxel_cloud);
//compute Plane Jaccard Index
void computePlaneJaccardIndex(MyPointCloud& source_mpc, MyPointCloud& rect_mpc, float grid_length, float *result);
//compute Cylinder Jaccard Index
void computeCylinderJaccardIndex(MyPointCloud& source_mpc, Point cenPoint0, Point cenPoint1, Vector3<float>& direction, float r, float grid_length, float *result);
//compute Sphere Jaccard Index
void computeSphereJaccardIndex(MyPointCloud& source_mpc, Point cenPoint, float r, float grid_length, float *result);
//object fitting
void object_fitting(PointCloudPtr_RGB cloud, vector<MyPointCloud_RGB> &plane_clouds, std::vector<MyPointCloud> &rect_clouds, vector<MyPointCloud_RGB> &cylinder_clouds, vector<MyPointCloud_RGB> &sphere_clouds, PointCloudPtr_RGB remained_cloud);

#endif // SCENE_SEG_H
