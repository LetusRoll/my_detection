/*
 * @Description: types
 * @Version: 2.0
 * @Author: CXY
 * @Date: 2021-10-10 18:33:55
 * @LastEditors: CXY
 * @LastEditTime: 2021-10-16 20:21:10
 */
#ifndef POINT_TYPES_H
#define POINT_TYPES_H



#include<pcl/point_types.h>
#include<pcl_ros/point_cloud.h>
//定义点云的格式
typedef pcl::PointXYZI Pt;
typedef pcl::PointCloud<Pt> PtC;


#endif