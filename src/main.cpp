/*
 * @Description: 
 * @Version: 2.0
 * @Author: CXY
 * @Date: 2021-10-15 21:46:34
 * @LastEditors: CXY
 * @LastEditTime: 2021-10-25 10:47:33
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
#include<jsk_recognition_msgs/PolygonArray.h>
#include<jsk_rviz_plugins/PictogramArray.h>
#include<geometry_msgs/Quaternion.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<memory>
using namespace std;
ros::Subscriber sub;
ros::Publisher clipped_pub,ground_cloud_pub,non_ground_cloud_pub,cluster_pub,box_pub,vehicle_pub;

bool IsVehicle(const autosense::ObjectPtr &object)
{
    if ((object->length<6.0)&&(object->length>3.0))
    {
        if((object->width<4.0)&&(object->width>1.5))
        {
            if((object->height<3.0)&&(object->height>1.0))
                return true;
        }
    }
    else
    {
        return false;
    }
}

void Callback(const sensor_msgs::PointCloud2::Ptr &msg)
{
    clock_t start=clock();
    PtC::Ptr in_cloud(new PtC);
    PtC::Ptr clipped_cloud(new PtC);
    PtC::Ptr non_ground_cloud(new PtC);
    PtC::Ptr ground_cloud(new PtC);
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
    RemoveGround remove(clipped_cloud,non_ground_cloud,ground_cloud);//滤除地面点
    
    ROS_INFO("Ground is removed!");
    clock_t t1=clock();
    ROS_INFO("Time of removing ground : %f  s",(float)((t1-t0)/CLOCKS_PER_SEC));
    //发布非地面点云
    sensor_msgs::PointCloud2::Ptr non_ground_cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*non_ground_cloud,*non_ground_cloud_msg);
    non_ground_cloud_msg->header=msg->header;
    non_ground_cloud_pub.publish(non_ground_cloud_msg);
    //发布地面点云
    sensor_msgs::PointCloud2::Ptr ground_cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*ground_cloud,*ground_cloud_msg);
    ground_cloud_msg->header=msg->header;
    ground_cloud_pub.publish(ground_cloud_msg);


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

    vector<int> is_vehicle=vector<int>(cloud_clusters.size(),0);

    //Apollo bounding box
    //define object builder
    boost::shared_ptr<autosense::object_builder::BaseObjectBuilder> object_builder_;
    
    // create object builder by manager
    object_builder_ = autosense::object_builder::createObjectBuilder();
    
    // build 3D orientation bounding box for clustering point cloud
    
    std::vector<autosense::ObjectPtr> objects;
    object_builder_->build(cloud_clusters, &objects);

    
    //发布包围盒
    visualization_msgs::MarkerArray vehicle_array;
    for(int i=0;i<objects.size();++i)
    {
        if(IsVehicle(objects[i]))
        {
            Eigen::Vector4f min, max;
            pcl::getMinMax3D(*objects[i]->cloud, min, max);

            visualization_msgs::Marker marker;
            marker.header = msg->header;
            marker.ns = "vehicle";
            marker.id = i;
            marker.type = visualization_msgs::Marker::LINE_LIST;

            geometry_msgs::Point p[24];
            p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
            p[1].x = min[0];  p[1].y = max[1];  p[1].z = max[2];
            p[2].x = max[0];  p[2].y = max[1];  p[2].z = max[2];
            p[3].x = max[0];  p[3].y = min[1];  p[3].z = max[2];
            p[4].x = max[0];  p[4].y = max[1];  p[4].z = max[2];
            p[5].x = max[0];  p[5].y = max[1];  p[5].z = min[2];
            p[6].x = min[0];  p[6].y = min[1];  p[6].z = min[2];
            p[7].x = max[0];  p[7].y = min[1];  p[7].z = min[2];
            p[8].x = min[0];  p[8].y = min[1];  p[8].z = min[2];
            p[9].x = min[0];  p[9].y = max[1];  p[9].z = min[2];
            p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
            p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
            p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
            p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
            p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
            p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
            p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
            p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
            p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
            p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
            p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
            p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
            p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
            p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
            
            
            for(int i = 0; i < 24; i++) 
            {
                marker.points.push_back(p[i]);
            }
        
            marker.scale.x = 0.02;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.5;
            marker.lifetime = ros::Duration(0.1);
            // marker.pose.position.x=objects[i]->ground_center[0];
            // marker.pose.position.y=objects[i]->ground_center[1];
            // marker.pose.position.z=objects[i]->ground_center[2]+objects[i]->height/2;
            
            // marker.pose.orientation=tf::createQuaternionMsgFromYaw(objects[i]->yaw_rad);    
            // marker.scale.x=objects[i]->length;
            // marker.scale.y=objects[i]->width;
            // marker.scale.z=objects[i]->height;
            // marker.color.a = 0.3;
            // marker.color.r = 0.0;
            // marker.color.g = 1.0;
            // marker.color.b = 0.0;
            // marker.lifetime = ros::Duration(1.0);
            vehicle_array.markers.push_back(marker);
            is_vehicle[i]=1;
        }
    }
    vehicle_pub.publish(vehicle_array);

    visualization_msgs::MarkerArray box_array;
    //凸包包围盒
    for (int i=0;i<cloud_clusters.size();++i)
    {
        // //去除离群点
        // PtC::Ptr filtered_cluster (new PtC);
        // pcl::StatisticalOutlierRemoval<Pt> sor;
        // sor.setInputCloud(cloud_clusters[i]);
        // sor.setMeanK(20);//50个临近点
        // sor.setStddevMulThresh(1.0);//距离大于1倍标准方差
        // sor.filter(*filtered_cluster);
        if(is_vehicle[i]==0)
        {
            Eigen::Vector4f min,max;
            pcl::getMinMax3D(*cloud_clusters[i],min,max);
            std::vector<cv::Point2f> points;
            for(int j=0;j<cloud_clusters[i]->size();++j)
            {
                cv::Point2f pt;
                pt.x=cloud_clusters[i]->points[j].x;
                pt.y=cloud_clusters[i]->points[j].y;
                points.push_back(pt);
            }

            std::vector<cv::Point2f> hull;
            cv::convexHull(points,hull);
            geometry_msgs::PolygonStamped polygon;
            for(size_t j=0;j<hull.size()+1;j++)
            {
                geometry_msgs::Point32 point;
                point.x=hull[j%hull.size()].x;
                point.y=hull[j%hull.size()].y;
                point.z=min[2];

                polygon.polygon.points.push_back(point);
            }

            for(size_t j=0;j<hull.size()+1;j++)
            {
                geometry_msgs::Point32 point;
                point.x=hull[j%hull.size()].x;
                point.y=hull[j%hull.size()].y;
                point.z=max[2];

                polygon.polygon.points.push_back(point);
            }
            visualization_msgs::Marker box;
            box.lifetime=ros::Duration(1.0);
            box.header=msg->header;
            box.type=visualization_msgs::Marker::LINE_STRIP;
            box.action=visualization_msgs::Marker::ADD;
            box.id=i;
            box.ns="box";
            box.scale.x=0.2;
            box.pose.orientation.x=0.0;
            box.pose.orientation.y=0.0;
            box.pose.orientation.z=0.0;
            box.pose.orientation.w=1.0;
            box.color.a=0.5;
            box.color.g=1.0;
            box.color.b=0.0;
            box.color.r=0.0;
            for(auto const &point: polygon.polygon.points)
            {
                geometry_msgs::Point tmp_point;
                tmp_point.x=point.x;
                tmp_point.y=point.y;
                tmp_point.z=point.z;
                box.points.push_back(tmp_point);
            }
            box_array.markers.push_back(box);
        }
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
    ground_cloud_pub=nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud",100);
    cluster_pub=nh.advertise<sensor_msgs::PointCloud2>("/clusters",100);
    box_pub=nh.advertise<visualization_msgs::MarkerArray>("/box",100);
    vehicle_pub=nh.advertise<visualization_msgs::MarkerArray>("/vehicle",100);
    ros::spin();
    return 0;
}


