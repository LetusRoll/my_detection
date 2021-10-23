/*
 * @Description: 
 * @Version: 2.0
 * @Author: CXY
 * @Date: 2021-10-15 21:46:34
 * @LastEditors: CXY
 * @LastEditTime: 2021-10-23 18:48:16
 */
/*
 * 
 *    ┏┓　　　┏┓
 *  ┏┛┻━━━┛┻┓
 *  ┃　　　　　　　┃
 *  ┃　　　━　　　┃
 *  ┃　＞　　　＜　┃
 *  ┃　　　　　　　┃
 *  ┃...　⌒　...　┃
 *  ┃　　　　　　　┃
 *  ┗━┓　　　┏━┛
 *      ┃　　　┃　
 *      ┃　　　┃
 *      ┃　　　┃
 *      ┃　　　┃  神兽保佑
 *      ┃　　　┃  代码无bug　　
 *      ┃　　　┃
 *      ┃　　　┗━━━┓
 *      ┃　　　　　　　┣┓
 *      ┃　　　　　　　┏┛
 *      ┗┓┓┏━┳┓┏┛
 *        ┃┫┫　┃┫┫
 *        ┗┻┛　┗┻┛
 */


/*
 * @Description: 
 * @Version: 2.0
 * @Author: CXY
 * @Date: 2021-10-15 21:46:34
 * @LastEditors: CXY
 * @LastEditTime: 2021-10-15 22:56:18
 */
#include"../include/my_detection/point_types.h"
#include"clip/clip.h"
#include"remove_ground/remove_ground.h"
#include"cluster/CVC.h"

#include"object_builders/base_object_builder.hpp"
#include"object_builders/object_builder_manager.hpp"
#include<ros/ros.h>
#include<time.h>
#include<boost/shared_ptr.hpp>
#include<pcl_ros/transforms.h>
#include<pcl/conversions.h>
#include<visualization_msgs/MarkerArray.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<geometry_msgs/Quaternion.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include<memory>
using namespace std;
ros::Subscriber sub;
ros::Publisher clipped_pub,non_ground_cloud_pub,cluster_pub,box_pub;

void Callback(const sensor_msgs::PointCloud2::Ptr &msg)
{
    clock_t start=clock();
    PtC::Ptr in_cloud(new PtC);
    PtC::Ptr clipped_cloud(new PtC);
    PtC::Ptr non_ground_cloud(new PtC);
    PtC::Ptr clustered_cloud(new PtC);
    
    pcl::fromROSMsg(*msg,*in_cloud);//ros->pcl
    ROS_INFO("The sum of points :  %d",in_cloud->size());
    Clip clip(in_cloud,clipped_cloud);//删除多余的点
    ROS_INFO("Clipped!");
    sensor_msgs::PointCloud2::Ptr clipped_cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*clipped_cloud,*clipped_cloud_msg);
    clipped_cloud_msg->header=msg->header;
    clipped_pub.publish(clipped_cloud_msg);

    clock_t t0=clock();
    RemoveGround remove(clipped_cloud,non_ground_cloud);//滤除地面点
    ROS_INFO("Ground is removed!");
    clock_t t1=clock();
    ROS_INFO("Time of removing ground : %f  s",(float)((t1-t0)/CLOCKS_PER_SEC));
    //发布非地面点云
    sensor_msgs::PointCloud2::Ptr non_ground_cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*non_ground_cloud,*non_ground_cloud_msg);
    non_ground_cloud_msg->header=msg->header;
    non_ground_cloud_pub.publish(non_ground_cloud_msg);

    vector<int> cluster_indices;
    //cluster_indices.resize(non_ground_cloud->size());
    
    clock_t t2=clock();

    self_cluster::CVC CVC_cluster(non_ground_cloud,cluster_indices);//CVC聚类
    vector<int> cluster_index;//数量超过10的点云簇序号
    CVC_cluster.SelectMajorCluster(cluster_indices,cluster_index);


    ROS_INFO("Clustered!");
    ROS_INFO("The amount of clusters :  %d",cluster_index.size());
    clock_t t3=clock();
    ROS_INFO("Time of clustering : %f  s",(float)((t3-t2)/CLOCKS_PER_SEC));
    
    std::vector<PtC::Ptr> cloud_clusters;//点云簇列表
    //遍历索引表，寻找点数超过10的点云簇，并传入cloud_clusters
    for(int i=0;i<cluster_index.size();++i)
    {
        PtC::Ptr cluster(new PtC);
        for(int j=0;j<cluster_indices.size();++j)
        {
            if(cluster_indices[j]==cluster_index[i])
            {
                cluster->points.push_back(non_ground_cloud->points[j]);
            }
	    

        }
        cloud_clusters.push_back(cluster);
    }
    //发布聚类点云
    
    sensor_msgs::PointCloud2::Ptr clustered_cloud_msg(new sensor_msgs::PointCloud2());
    PtC::Ptr clusters(new PtC);
    for(int i=0;i<cloud_clusters.size();++i)
    {
        *clusters+=*cloud_clusters[i];
    }
    
    pcl::toROSMsg(*clusters,*clustered_cloud_msg);
    clustered_cloud_msg->header=msg->header;
    cluster_pub.publish(clustered_cloud_msg);
    clock_t end=clock();
    ROS_INFO("Total time: %lf  s",(double)(end-start)/CLOCKS_PER_SEC);


    //Apollo bounding box
    //define object builder
    boost::shared_ptr<autosense::object_builder::BaseObjectBuilder> object_builder_;
    
    // create object builder by manager
    object_builder_ = autosense::object_builder::createObjectBuilder();
    
    // build 3D orientation bounding box for clustering point cloud
    
    std::vector<autosense::ObjectPtr> objects;
    object_builder_->build(cloud_clusters, &objects);

    
    //发布包围盒
    visualization_msgs::MarkerArray box_array;
    for(int i=0;i<objects.size();++i)
    {
        if((objects[i]->height>4.0)||(objects[i]->length>10.0)||(objects[i]->width>4.0)
            ||(objects[i]->height<0.5))
        {
            continue;
        }        
        
        
        visualization_msgs::Marker marker;
        marker.header = msg->header;
        marker.ns = "bounding box";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose.position.x=objects[i]->ground_center[0];
        marker.pose.position.y=objects[i]->ground_center[1];
        marker.pose.position.z=objects[i]->ground_center[2]+objects[i]->height/2;
        
        marker.pose.orientation=tf::createQuaternionMsgFromYaw(objects[i]->yaw_rad);    
        marker.scale.x=objects[i]->length;
        marker.scale.y=objects[i]->width;
        marker.scale.z=objects[i]->height;
        marker.color.a = 0.3;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(1.0);
        box_array.markers.push_back(marker);
    }

    box_pub.publish(box_array);
  

}



int main(int argc,char **argv)
{
    ros::init(argc,argv,"my_detection");
    ros::NodeHandle nh;
    
    sub=nh.subscribe("/rslidar_points_throttle",1,Callback);
    //发布各个topic
    clipped_pub=nh.advertise<sensor_msgs::PointCloud2>("/clipped_cloud",100);
    non_ground_cloud_pub=nh.advertise<sensor_msgs::PointCloud2>("/non_ground_cloud",100);
    cluster_pub=nh.advertise<sensor_msgs::PointCloud2>("/clusters",100);
    box_pub=nh.advertise<visualization_msgs::MarkerArray>("/box",100);
    ros::spin();
    return 0;
}


