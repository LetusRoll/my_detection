/*
 * @Description: 删除多余的点，划分合适的区域
 * @Version: 2.0
 * @Author: CXY
 * @Date: 2021-10-10 18:51:40
 * @LastEditors: CXY
 * @LastEditTime: 2021-10-22 19:02:30
 */
#include"clip.h"

Clip::Clip(const PtC::Ptr &in_cloud,PtC::Ptr &out_cloud)
{
    PtC::Ptr filtered_cloud(new PtC);
    
    //删除掉多余的点云
    for(size_t i=0;i<in_cloud->size();++i)
    {
        float x=in_cloud->points[i].x;
        float y=in_cloud->points[i].y;
        float z=in_cloud->points[i].z;
        if(IsIn(x,min_x,max_x)&&(IsIn(y,min_y,max_y)&&(IsIn(z,min_z,max_z))))
        {
            filtered_cloud->points.push_back(in_cloud->points[i]);
        }
    }

    //降采样
    pcl::VoxelGrid<Pt> vg_filter;
    vg_filter.setInputCloud(filtered_cloud);
    vg_filter.setLeafSize(leaf_size,leaf_size,leaf_size);
    vg_filter.filter(*out_cloud);
    

}

//判断点云是否在规定范围内
bool Clip::IsIn(const float &a,const float &min,const float &max)
{
    return (a>min)&&(a<max);
}

Clip::~Clip()
{
    
}