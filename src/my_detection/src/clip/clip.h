/*
 * @Description: 删除多余的点，划分合适的区域
 * @Version: 2.0
 * @Author: CXY
 * @Date: 2021-10-10 18:51:35
 * @LastEditors: CXY
 * @LastEditTime: 2021-10-23 12:19:57
 */
#ifndef CLIP_H
#define CLIP_H

#include "my_detection/point_types.h"
#include<pcl/filters/voxel_grid.h>
#include<ros/ros.h>

class Clip
{
    private:
    //设置三个方向上最大最小检测范围
    float min_x=-30.0;
    float max_x=50.0;
    float min_y=-15.0;
    float max_y=15.0;
    float min_z=-2.2;
    float max_z=0.1;

    //设置车辆自身坐标，便于删除自身的点云
    float min_base_x=-2.0;
    float max_base_x=2.0;
    float min_base_y=-1.0;
    float max_base_y=1.0;
    float min_base_z=-2.2;
    float max_base_z=0.05;
    
    float leaf_size=0.1;

    public:
    Clip(ros::NodeHandle &nh,ros::NodeHandle &private_nh);
    void Process(const PtC::Ptr &in_cloud,PtC::Ptr &out_cloud);
    //判断该点是否在范围内
    bool IsIn (const float &a,const float &min,const float &max);
    ~Clip();


};



#endif
