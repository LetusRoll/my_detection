/*
 * @Description: 
 * @Version: 2.0
 * @Author: CXY
 * @Date: 2021-10-10 21:24:43
 * @LastEditors: CXY
 * @LastEditTime: 2021-10-22 20:07:57
 */
#ifndef REMOVE_GROUND_H
#define REMOVE_GORUND_H

#include"my_detection/point_types.h"
//#include<my_detection/include/my_detection/point_types.h>
#include <vector>
using std::vector;

#define PI 3.14159


inline float CalculateR(const Pt &point)
{
    return sqrt(point.x * point.x +point.y * point.y);
}

class RemoveGround
{

    private:
    struct AR
    {
        Pt point;
        int angle_index;
        int radius_index;
        float angle;
        float radius;
    };
    const float delta_radius_=0.1;//单位半径大小
    const float delta_angle_=0.18;//单位角度大小
    const float SENSOR_HEIGHT=2.2;//传感器高度
    const float min_delta_height_=0.05;//前后两点最小高度差
    const float local_max_slope=8.0;//前后两点之间的坡度阈值
    const float global_max_slope=5.0;//从传感器地面投影点到点云的坡度阈值
    const float max_delta_radius_=0.2;//最大半径差
    const int radial_num=ceil(360/delta_angle_);
    public:
    RemoveGround(const PtC::Ptr &in_cloud,PtC::Ptr &out_cloud);
    ~RemoveGround();
    void XYZI2AR(const PtC::Ptr &in_cloud,vector<vector<AR> > &out);//将点云的XYZI格式转化成角度半径的形式
    void Segment(vector<vector<AR> >&out,PtC::Ptr &out_cloud);
};




#endif