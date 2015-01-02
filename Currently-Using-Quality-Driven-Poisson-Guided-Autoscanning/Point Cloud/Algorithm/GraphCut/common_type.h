#ifndef COMMON_TYPE_H
#define COMMON_TYPE_H

#include <vector>

using namespace std;


struct Normalt 
{
	float normal_x, normal_y, normal_z;
};


typedef struct MyPoint{
  float x;
  float y;
  float z;
}MyPt;

typedef struct MyPoint_RGB{
  float x;
  float y;
  float z;
  float r;
  float g;
  float b;
}MyPt_RGB;

typedef struct MyPoint_RGB_NORMAL{
  float x;
  float y;
  float z;
  float r;
  float g;
  float b;
  float normal_x;
  float normal_y;
  float normal_z;
}MyPt_RGB_NORMAL;

class MyPointCloud{
public:
  MyPointCloud();
  ~MyPointCloud();

public:
  vector<MyPt> mypoints;
};

class MyPointCloud_RGB{
public:
  MyPointCloud_RGB(){}
  ~MyPointCloud_RGB(){}

public:
  vector<MyPt_RGB> mypoints;
};

class MyPointCloud_RGB_NORMAL{
public:
  MyPointCloud_RGB_NORMAL(){}
  ~MyPointCloud_RGB_NORMAL(){}

public:
  vector<MyPt_RGB_NORMAL> mypoints;
};

#endif 