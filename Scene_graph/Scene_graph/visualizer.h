#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "common_type.h"
#include <pcl/visualization/pcl_visualizer.h>

class Visualizer
{
public:
    Visualizer();
    ~Visualizer();

public:
	//void addPointCloud_RGB_NORMAL(PointCloudPtr_RGB_NORMAL cloud, char* id);
	void show();

public:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};

#endif // VISUALIZER_H