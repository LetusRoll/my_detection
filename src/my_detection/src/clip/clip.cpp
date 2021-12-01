/*
 * @Description: 删除多余的点，划分合适的区域
 * @Version: 2.0
 * @Author: CXY
 * @Date: 2021-10-10 18:51:40
 * @LastEditors: CXY
 * @LastEditTime: 2021-10-22 19:02:30
 */
#include"clip.h"

Clip::Clip(ros::NodeHandle &nh,ros::NodeHandle &private_nh)
{
    private_nh.param<float>("min_x", min_x, -30.0);
    private_nh.param<float>("max_x", max_x, 50.0);
    private_nh.param<float>("min_y", min_y, -15.0);
    private_nh.param<float>("max_y", max_y, 15.0);
    private_nh.param<float>("min_z", min_z, -2.2);
    private_nh.param<float>("max_z", max_z, 1.0);
    private_nh.param<float>("min_base_x", min_base_x, -2.0);
    private_nh.param<float>("max_base_x", max_base_x, 2.0);
    private_nh.param<float>("min_base_y", min_base_y, -1.0);
    private_nh.param<float>("max_base_y", max_base_y, 1.0);
    private_nh.param<float>("min_base_z", min_base_z, -2.2);
    private_nh.param<float>("max_base_z", max_base_z, 0.05);

}



void Clip::Process(const PtC::Ptr &in_cloud,PtC::Ptr &out_cloud)
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
	    if(!(IsIn(x,min_base_x,max_base_x)&&IsIn(y,min_base_y,max_base_y)&&IsIn(z,min_base_z,max_base_z)))            
	    {		
		filtered_cloud->points.push_back(in_cloud->points[i]);
	    }
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
