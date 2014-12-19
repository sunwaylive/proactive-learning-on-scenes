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
//find a minimum bounding rect
void find_min_rect(PointCloudPtr_RGB cloud, cv::Point2f &p0,cv::Point2f &p1,cv::Point2f &p2,cv::Point2f &p3);
//VCCS over-segmentation
void VCCS_over_segmentation(PointCloudPtr_RGB cloud, NormalCloudTPtr normals, float voxel_resolution,float seed_resolution,float color_importance,float spatial_importance,float normal_importance,vector<MyPointCloud_RGB>& patch_clouds, PointCloudT::Ptr colored_cloud);
//object fitting
void object_fitting(PointCloudPtr_RGB cloud, vector<MyPointCloud_RGB> &plane_clouds, std::vector<MyPointCloud> &rect_clouds, vector<MyPointCloud_RGB> &cylinder_clouds, vector<MyPointCloud_RGB> &sphere_clouds, PointCloudPtr_RGB remained_cloud);

#endif // SCENE_SEG_H
