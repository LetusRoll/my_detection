#include"perception.h"

Perception::Perception(ros::NodeHandle &nh,ros::NodeHandle &private_nh):clip(nh,private_nh),
rm_ground(nh,private_nh),cvc_cluster(nh,private_nh)
{
    private_nh.param<float>("delta_box_length", delta_box_length, 5.0);
    sub=nh.subscribe("/rslidar_points_throttle",1,&Perception::Callback,this);
    //发布各个topic
    clipped_pub=nh.advertise<sensor_msgs::PointCloud2>("/clipped_cloud",100);
    non_ground_cloud_pub=nh.advertise<sensor_msgs::PointCloud2>("/non_ground_cloud",100);
    ground_cloud_pub=nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud",100);
    cluster_pub=nh.advertise<sensor_msgs::PointCloud2>("/clusters",100);
    box_pub=nh.advertise<visualization_msgs::MarkerArray>("/box",100);
    //vehicle_pub=nh.advertise<visualization_msgs::MarkerArray>("/vehicle",100);
    ros::spin();
}
void Perception::BigBox2SmallBox(autosense::ObjectPtr &object,vector<PtC::Ptr> &small_clusters)
{
    Eigen::Vector4f min_object, max_object;
    pcl::getMinMax3D(*object->cloud, min_object, max_object);
    int box_num=int((max_object[0]-min_object[0])/delta_box_length)+1;
    //small_clusters.resize(box_num);
    for(int i=0;i<box_num;++i)
    {
        PtC::Ptr small_cluster(new PtC);
        for(int j=0;j<object->cloud->points.size();++j)
        {
            int num=int((object->cloud->points[j].x-min_object[0])/delta_box_length);
            if(num==i)
            {
                small_cluster->push_back(object->cloud->points[j]);
                
            }
        }
        assert(small_cluster->size()!=0);
        small_clusters.push_back(small_cluster);
    }
}

void Perception::Callback(const sensor_msgs::PointCloud2::Ptr &msg)
{
    clock_t start=clock();
    PtC::Ptr in_cloud(new PtC);
    PtC::Ptr clipped_cloud(new PtC);
    PtC::Ptr non_ground_cloud(new PtC);
    PtC::Ptr ground_cloud(new PtC);
    PtC::Ptr clustered_cloud(new PtC);
    
    pcl::fromROSMsg(*msg,*in_cloud);//ros->pcl
    ROS_INFO("The sum of points :  %d",in_cloud->size());
    clip.Process(in_cloud,clipped_cloud);//删除多余的点
    //ROS_INFO("Clipped!");
    sensor_msgs::PointCloud2::Ptr clipped_cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*clipped_cloud,*clipped_cloud_msg);
    clipped_cloud_msg->header=msg->header;
    clipped_pub.publish(clipped_cloud_msg);

    clock_t t0=clock();
    rm_ground.Process(clipped_cloud,non_ground_cloud,ground_cloud);//滤除地面点
    
    //ROS_INFO("Ground is removed!");
    clock_t t1=clock();
    //ROS_INFO("Time of removing ground : %f  s",(float)((t1-t0)/CLOCKS_PER_SEC));
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
    clock_t t2=clock();

    cvc_cluster.Process(non_ground_cloud,cluster_indices);//CVC聚类
    vector<int> cluster_index;//数量超过10的点云簇序号
    cvc_cluster.SelectMajorCluster(cluster_indices,cluster_index);


    //ROS_INFO("Clustered!");
    ROS_INFO("The amount of clusters :  %d",cluster_index.size());
    clock_t t3=clock();
    //ROS_INFO("Time of clustering : %f  s",(float)((t3-t2)/CLOCKS_PER_SEC));
    
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
    

    //Apollo bounding box
    //define object builder
    boost::shared_ptr<autosense::object_builder::BaseObjectBuilder> object_builder_;
    
    // create object builder by manager
    object_builder_ = autosense::object_builder::createObjectBuilder();
    
    // build 3D orientation bounding box for clustering point cloud
    
    std::vector<autosense::ObjectPtr> objects;
    object_builder_->build(cloud_clusters, &objects);

    //将大型包围盒划分成几个小的包围盒
    visualization_msgs::MarkerArray boxes;
    int count=0;
    for(int i=0;i<objects.size();++i)
    {
        
        if(objects[i]->length>10)
        {
           
            vector<PtC::Ptr> small_clusters;
           
            BigBox2SmallBox(objects[i],small_clusters);
            
            
            for(int j=0;j<small_clusters.size();++j)
            {
                Eigen::Vector4f min,max;
                pcl::getMinMax3D(*small_clusters[j],min,max);
                std::vector<cv::Point2f> points;
                for(int k=0;k<small_clusters[j]->size();++k)
                {
                    cv::Point2f pt;
                    pt.x=small_clusters[j]->points[k].x;
                    pt.y=small_clusters[j]->points[k].y;
                    points.push_back(pt);
                }

                std::vector<cv::Point2f> hull;
                cv::convexHull(points,hull);
                geometry_msgs::PolygonStamped polygon;
                for(size_t k=0;k<hull.size()+1;k++)
                {
                    geometry_msgs::Point32 point;
                    point.x=hull[k%hull.size()].x;
                    point.y=hull[k%hull.size()].y;
                    point.z=min[2];

                    polygon.polygon.points.push_back(point);
                }

                for(size_t k=0;k<hull.size()+1;k++)
                {
                    geometry_msgs::Point32 point;
                    point.x=hull[k%hull.size()].x;
                    point.y=hull[k%hull.size()].y;
                    point.z=max[2];

                    polygon.polygon.points.push_back(point);
                }
                visualization_msgs::Marker box;
                box.lifetime=ros::Duration(0.1);
                box.header=msg->header;
                box.type=visualization_msgs::Marker::LINE_STRIP;
                box.action=visualization_msgs::Marker::ADD;
                box.id=count;
                box.ns="small_box";
                box.scale.x=0.1;
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
                boxes.markers.push_back(box);
                count++;
            }
        }
        else
        {
        
            //计算凸包
            Eigen::Vector4f min,max;
            pcl::getMinMax3D(*cloud_clusters[i],min,max);
            geometry_msgs::PolygonStamped polygon;
            for(size_t j=0;j<objects[i]->polygon.points.size()+1;j++)
            {
                geometry_msgs::Point32 point;
                point.x=objects[i]->polygon.points[j%objects[i]->polygon.points.size()].x;
                point.y=objects[i]->polygon.points[j%objects[i]->polygon.points.size()].y;
                point.z=min[2];

                polygon.polygon.points.push_back(point);
            }

            for(size_t j=0;j<objects[i]->polygon.points.size()+1;j++)
            {
                geometry_msgs::Point32 point;
                point.x=objects[i]->polygon.points[j%(objects[i]->polygon.points.size())].x;
                point.y=objects[i]->polygon.points[j%(objects[i]->polygon.points.size())].y;
                point.z=max[2];

                polygon.polygon.points.push_back(point);
            }
            visualization_msgs::Marker box;
            box.lifetime=ros::Duration(0.1);
            box.header=msg->header;
            box.type=visualization_msgs::Marker::LINE_STRIP;
            box.action=visualization_msgs::Marker::ADD;
            box.id=count;
            box.ns="polygongal_box";
            box.scale.x=0.1;
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
            boxes.markers.push_back(box);
            count++;
        }
    }
    box_pub.publish(boxes);
    clock_t end=clock();
    ROS_INFO("Total time: %lf  s",(double)(end-start)/CLOCKS_PER_SEC);
    ROS_INFO("FPS: %lf  Hz",(double)CLOCKS_PER_SEC/(end-start));

    // //矩形包围盒
    // visualization_msgs::MarkerArray boxes;
    // for(int i=0;i<objects.size();++i)
    // {
    //     if(objects[i]->length>7)
    //     {
    //         continue;
    //     }
    //     else
    //     {
    //         
    //         visualization_msgs::Marker box;
    //         box.header=msg->header;
    //         box.ns="obb_box";
    //         box.id=i;
    //         box.type = visualization_msgs::Marker::CUBE;

    //         
    //         box.pose.position.x=objects[i]->ground_center[0];
    //         box.pose.position.y=objects[i]->ground_center[1];
    //         box.pose.position.z=objects[i]->ground_center[2]+objects[i]->height/2;
    //         box.pose.orientation=tf::createQuaternionMsgFromYaw(objects[i]->yaw_rad);
    //         box.scale.x=objects[i]->length;
    //         box.scale.y=objects[i]->width;
    //         box.scale.z=objects[i]->height;
            
    //         box.color.a=0.1;
    //         box.color.r=0.0;
    //         box.color.g=1.0;
    //         box.color.b=0.0;
    //         box.lifetime=ros::Duration(0.2);
    //         boxes.markers.push_back(box);
    //     }
    // }



    //vehicle_flag记录点云簇是否符合车辆特征，0不符合，1符合
    // vector<int> vehicle_flag=vector<int>(objects.size(),0);
    // //发布车辆包围盒
    // visualization_msgs::MarkerArray vehicle_array;
    // visualization_msgs::MarkerArray box_array;
    // for(int i=0;i<objects.size();++i)
    // {
    //     if(IsVehicle(objects[i]))
    //     {
    //         vehicle_flag[i]=1; 
    //         Eigen::Vector4f min, max;
    //         pcl::getMinMax3D(*objects[i]->cloud, min, max);

    //         visualization_msgs::Marker marker;
    //         marker.header = msg->header;
    //         marker.ns = "vehicle";
    //         marker.id = i;
    //         marker.type = visualization_msgs::Marker::LINE_LIST;

    //         geometry_msgs::Point p[24];
    //         p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
    //         p[1].x = min[0];  p[1].y = max[1];  p[1].z = max[2];
    //         p[2].x = max[0];  p[2].y = max[1];  p[2].z = max[2];
    //         p[3].x = max[0];  p[3].y = min[1];  p[3].z = max[2];
    //         p[4].x = max[0];  p[4].y = max[1];  p[4].z = max[2];
    //         p[5].x = max[0];  p[5].y = max[1];  p[5].z = min[2];
    //         p[6].x = min[0];  p[6].y = min[1];  p[6].z = min[2];
    //         p[7].x = max[0];  p[7].y = min[1];  p[7].z = min[2];
    //         p[8].x = min[0];  p[8].y = min[1];  p[8].z = min[2];
    //         p[9].x = min[0];  p[9].y = max[1];  p[9].z = min[2];
    //         p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
    //         p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
    //         p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
    //         p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
    //         p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
    //         p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
    //         p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
    //         p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
    //         p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
    //         p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
    //         p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
    //         p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
    //         p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
    //         p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
            
            
    //         for(int i = 0; i < 24; i++) 
    //         {
    //             marker.points.push_back(p[i]);
    //         }
        
    //         marker.scale.x = 0.02;
    //         marker.color.a = 1.0;
    //         marker.color.r = 0.0;
    //         marker.color.g = 1.0;
    //         marker.color.b = 0.5;
    //         marker.lifetime = ros::Duration(0.1);
    //         vehicle_array.markers.push_back(marker);
    //     }
    // }
    // vehicle_pub.publish(vehicle_array);


   


    // visualization_msgs::MarkerArray box_array;
    // //凸包包围盒
    // for (int i=0;i<cloud_clusters.size();++i)
    // {
        
    //     //计算凸包
    //     Eigen::Vector4f min,max;
    //     pcl::getMinMax3D(*cloud_clusters[i],min,max);
    //     std::vector<cv::Point2f> points;
    //     for(int j=0;j<cloud_clusters[i]->size();++j)
    //     {
    //         cv::Point2f pt;
    //         pt.x=cloud_clusters[i]->points[j].x;
    //         pt.y=cloud_clusters[i]->points[j].y;
    //         points.push_back(pt);
    //     }

    //     std::vector<cv::Point2f> hull;
    //     cv::convexHull(points,hull);
    //     geometry_msgs::PolygonStamped polygon;
    //     for(size_t j=0;j<hull.size()+1;j++)
    //     {
    //         geometry_msgs::Point32 point;
    //         point.x=hull[j%hull.size()].x;
    //         point.y=hull[j%hull.size()].y;
    //         point.z=min[2];

    //         polygon.polygon.points.push_back(point);
    //     }

    //     for(size_t j=0;j<hull.size()+1;j++)
    //     {
    //         geometry_msgs::Point32 point;
    //         point.x=hull[j%hull.size()].x;
    //         point.y=hull[j%hull.size()].y;
    //         point.z=max[2];

    //         polygon.polygon.points.push_back(point);
    //     }
    //     visualization_msgs::Marker box;
    //     box.lifetime=ros::Duration(0.1);
    //     box.header=msg->header;
    //     box.type=visualization_msgs::Marker::LINE_STRIP;
    //     box.action=visualization_msgs::Marker::ADD;
    //     box.id=i;
    //     box.ns="box";
    //     box.scale.x=0.2;
    //     box.pose.orientation.x=0.0;
    //     box.pose.orientation.y=0.0;
    //     box.pose.orientation.z=0.0;
    //     box.pose.orientation.w=1.0;
    //     box.color.a=0.5;
    //     box.color.g=1.0;
    //     box.color.b=0.0;
    //     box.color.r=0.0;
    //     for(auto const &point: polygon.polygon.points)
    //     {
    //         geometry_msgs::Point tmp_point;
    //         tmp_point.x=point.x;
    //         tmp_point.y=point.y;
    //         tmp_point.z=point.z;
    //         box.points.push_back(tmp_point);
    //     }
    //     box_array.markers.push_back(box);
        
    // }
    // box_pub.publish(box_array);
    // clock_t end=clock();
    // ROS_INFO("Total time: %lf  s",(double)(end-start)/CLOCKS_PER_SEC);
  
}

Perception::~Perception(){}