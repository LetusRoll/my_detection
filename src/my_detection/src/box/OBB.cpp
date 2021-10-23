#include"OBB.h"
namespace Box
{
    OBB::OBB(ros::NodeHandle nh,ros::NodeHandle private_nh)
    {

    }
    OBB::~OBB(){}
    void OBB::addbox (PtC::Ptr in_cloud,std::vector<int> clusters_point_indices,const sensor_msgs::PointCloud2::Ptr in,visualization_msgs::MarkerArray &markerarray)
    {
        // PC::Ptr in_cloud_z0(new PC);
        // for (int i=0;i<in_cloud->points.size();i++)
        // {
        //     P point;
        //     point.x=in_cloud->points[i].x;
        //     point.y=in_cloud->points[i].y;
        //     point.z=0;
        //     point.intensity=in_cloud->points[i].intensity;
        //     in_cloud_z0->push_back(point);
        // }

        for(int i=0;i<clusters_point_indices.size();i++)
        {    
            PC::Ptr cluster(new PC);
            pcl::copyPointCloud(*in_cloud,clusters_point_indices[i].indices,*cluster);
            pcl::MomentOfInertiaEstimation<P> features_extractor;

            // PC::Ptr cluster_plane(new PC);
            // for(int j=0;j<cluster->size();j++)
            // {
            //     P point;
            //     point.x=cluster->points[j].x;
            //     point.y=cluster->points[j].y;
            //     point.z=0;
            //     point.intensity=cluster->points[j].intensity;
            //     cluster_plane->push_back(point);
            // }

            features_extractor.setInputCloud(cluster);
            features_extractor.compute();
            std::vector<float> moment_of_inertia;
            std::vector<float> eccentricity;
            P min_point_OBB;
            P max_point_OBB;
            P position_OBB;
            Eigen::Matrix3f rotation_matrix_OBB;
            float major_value,middle_value,minor_value;
            Eigen::Vector3f major_vector,middle_vector,minor_vector;
            Eigen::Vector3f mass_center;
            features_extractor.getMomentOfInertia(moment_of_inertia);
            features_extractor.getEccentricity(eccentricity);
            features_extractor.getOBB(min_point_OBB,max_point_OBB,position_OBB,rotation_matrix_OBB);
            features_extractor.getEigenValues(major_value,middle_value,minor_value);
            features_extractor.getEigenVectors(major_vector,middle_vector,minor_vector);
            features_extractor.getMassCenter(mass_center);
            Eigen::Vector3f position(position_OBB.x,position_OBB.y,position_OBB.z);
            Eigen::Quaternionf quat(rotation_matrix_OBB);

            // Eigen::Matrix3f rotation_matrix_OBB_z;
            // rotation_matrix_OBB_z.col(0)=rotation_matrix_OBB.col(0);
            // rotation_matrix_OBB_z.col(1)=rotation_matrix_OBB.col(1);
            // rotation_matrix_OBB_z.col(2)<<0,0,1;
            // Eigen::Quaternionf quat_z(rotation_matrix_OBB_z);

            Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
            Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
            Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
            Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
            Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
            Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
            Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
            Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
            
            p1 = rotation_matrix_OBB * p1 + position;
            p2 = rotation_matrix_OBB * p2 + position;
            p3 = rotation_matrix_OBB * p3 + position;
            p4 = rotation_matrix_OBB * p4 + position;
            p5 = rotation_matrix_OBB * p5 + position;
            p6 = rotation_matrix_OBB * p6 + position;
            p7 = rotation_matrix_OBB * p7 + position;
            p8 = rotation_matrix_OBB * p8 + position;
            
            geometry_msgs::Point p[24];
            p[0].x=p1[0];p[0].y=p1[1];p[0].z=p1[2];
            p[1].x=p2[0];p[1].y=p2[1];p[1].z=p2[2];
            p[2].x=p1[0];p[2].y=p1[1];p[2].z=p1[2];
            p[3].x=p4[0];p[3].y=p4[1];p[3].z=p4[2];
            p[4].x=p1[0];p[4].y=p1[1];p[4].z=p1[2];
            p[5].x=p5[0];p[5].y=p5[1];p[5].z=p5[2];
            p[6].x=p5[0];p[6].y=p5[1];p[6].z=p5[2];
            p[7].x=p6[0];p[7].y=p6[1];p[7].z=p6[2];
            p[8].x=p5[0];p[8].y=p5[1];p[8].z=p5[2];
            p[9].x=p8[0];p[9].y=p8[1];p[9].z=p8[2];
            p[10].x=p2[0];p[10].y=p2[1];p[10].z=p2[2];
            p[11].x=p6[0];p[11].y=p6[1];p[11].z=p6[2];
            p[12].x=p6[0];p[12].y=p6[1];p[12].z=p6[2];
            p[13].x=p7[0];p[13].y=p7[1];p[13].z=p7[2];
            p[14].x=p7[0];p[14].y=p7[1];p[14].z=p7[2];
            p[15].x=p8[0];p[15].y=p8[1];p[15].z=p8[2];
            p[16].x=p2[0];p[16].y=p2[1];p[16].z=p2[2];
            p[17].x=p3[0];p[17].y=p3[1];p[17].z=p3[2];
            p[18].x=p4[0];p[18].y=p4[1];p[18].z=p4[2];
            p[19].x=p8[0];p[19].y=p8[1];p[19].z=p8[2];
            p[20].x=p3[0];p[20].y=p3[1];p[20].z=p3[2];
            p[21].x=p4[0];p[21].y=p4[1];p[21].z=p4[2];
            p[22].x=p3[0];p[22].y=p3[1];p[22].z=p3[2];
            p[23].x=p7[0];p[23].y=p7[1];p[23].z=p7[2];

            visualization_msgs::Marker marker;
            marker.ns = "detection_box";
            marker.id=i;
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.header=in->header;
            for(int j = 0; j < 24; j++) 
            {
                marker.points.push_back(p[j]);
            }

            marker.scale.x = 0.1;
            marker.color.a = 0.5;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.5;
            marker.lifetime = ros::Duration(0.1);
            markerarray.markers.push_back(marker);




            // Eigen::Vector4f min, max;
            // Eigen::Vector4f centroid;
            // pcl::compute3DCentroid(*cluster,centroid);
            // pcl::getMinMax3D(*cluster, min, max);
            // geometry_msgs::Point pose;
            
            // visualization_msgs::Marker marker;
            // marker.header=in->header;
            // marker.ns="detection_box";
            // marker.id=i;
            // marker.type=visualization_msgs::Marker::CUBE;
            // marker.action=visualization_msgs::Marker::ADD;
            // marker.pose.position.x=centroid[0];
            // marker.pose.position.y=centroid[1];
            // marker.pose.position.z=centroid[2];
            // marker.pose.orientation.x=quat.x();
            // marker.pose.orientation.y=quat.y();
            // marker.pose.orientation.z=quat.z();
            // marker.pose.orientation.w=quat.w();
            // marker.scale.x=(max[0]-min[0]);
            // marker.scale.y=(max[1]-min[1]);
            // marker.scale.z=(max[2]-min[2]);
            // if(marker.scale.x==0)
            // {
            //     marker.scale.x=0.1;
            // }
            // if(marker.scale.y==0)
            // {
            //     marker.scale.y=0.1;
            // }
            // if(marker.scale.z==0)
            // {
            //     marker.scale.z=0.1;
            // }
            // marker.color.a = 0.5;
            // marker.color.r = 0.0;
            // marker.color.g = 1.0;
            // marker.color.b = 0.0;
            // marker.lifetime=ros::Duration();
            // markerarray.markers.push_back(marker);
            
        }
       
    }



    // void OBB::addbox (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr > > &cloud_clustered,const sensor_msgs::PointCloud2::Ptr in,visualization_msgs::MarkerArray &marker_array)
    // {
    //     //ROS_INFO("cloud_clustered number:   %d",cloud_clustered.size());
    //     marker_array.markers.clear();
    //     for(int i=0;i<cloud_clustered.size();i++)
    //     {
    //         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
    //         for (int j=0;j<(cloud_clustered[i]->size());j++)
    //         {
    //             //ROS_INFO("cloud_clustered number:  %d",cloud_clustered[i]->size());
    //             pcl::PointXYZ P;    //必须要声明一个点，否则会报错，（核心转储）
    //             P.x=cloud_clustered[i]->points[j].x;
    //             P.y=cloud_clustered[i]->points[j].y;
    //             P.z=0;
    //             //P.rgb=cloud_clustered[i]->points[j].rgb;
    //             cloud_plane->push_back(P);
    //         }

    //         pcl::MomentOfInertiaEstimation<pcl::PointXYZ> features_extractor;
    //         features_extractor.setInputCloud(cloud_plane);
    //         features_extractor.compute();
    //         std::vector<float> moment_of_inertia;
    //         std::vector<float> eccentricity;
    //         pcl::PointXYZRGB min_point_OBB;
    //         pcl::PointXYZRGB max_point_OBB;
    //         pcl::PointXYZRGB position_OBB;
    //         Eigen::Matrix3f rotation_matrix_OBB;
    //         float major_value,middle_value,minor_value;
    //         Eigen::Vector3f major_vector,middle_vector,minor_vector;
    //         Eigen::Vector3f mass_center;
    //         features_extractor.getMomentOfInertia(moment_of_inertia);
    //         features_extractor.getEccentricity(eccentricity);
    //         features_extractor.getOBB(min_point_OBB,max_point_OBB,position_OBB,rotation_matrix_OBB);
    //         features_extractor.getEigenValues(major_value,middle_value,minor_value);
    //         features_extractor.getEigenVectors(major_vector,middle_vector,minor_vector);
    //         features_extractor.getMassCenter(mass_center);
            
    //         Eigen::Vector3f position(position_OBB.x,position_OBB.y,position_OBB.z);
            
    //         Eigen::Quaternionf quat(rotation_matrix_OBB);
    //         //ROS_INFO("x1:   %f",max_point_OBB.x-min_point_OBB.x);

    //         Eigen::Vector4f min, max;
    //         Eigen::Vector4f centroid;
    //         pcl::compute3DCentroid(*(cloud_clustered[i]),centroid);
    //         pcl::getMinMax3D(*(cloud_clustered[i]), min, max);
    //         geometry_msgs::Point pose;
            
    //         visualization_msgs::Marker marker;
    //         marker.header=in->header;
    //         marker.ns="detection_box";
    //         marker.id=i;
    //         marker.type=visualization_msgs::Marker::CUBE;
    //         marker.action=visualization_msgs::Marker::ADD;

    //         marker.pose.position.x=centroid[0];
    //         marker.pose.position.y=centroid[1];
    //         marker.pose.position.z=centroid[2];

    //         marker.pose.orientation.x=quat.x();
    //         marker.pose.orientation.y=quat.y();
    //         marker.pose.orientation.z=quat.z();
    //         marker.pose.orientation.w=quat.w();

    //         marker.scale.x=(max[0]-min[0]);
    //        // ROS_INFO("x2:  %f",marker.scale.x);
    //         marker.scale.y=(max[1]-min[1]);
    //         marker.scale.z=(max[2]-min[2]);

    //         if(marker.scale.x==0)
    //         {
    //             marker.scale.x=0.1;
    //         }
    //         if(marker.scale.y==0)
    //         {
    //             marker.scale.y=0.1;
    //         }
    //         if(marker.scale.z==0)
    //         {
    //             marker.scale.z=0.1;
    //         }
    //         marker.color.a = 0.5;
    //         marker.color.r = 0.0;
    //         marker.color.g = 1.0;
    //         marker.color.b = 0.0;

    //         marker.lifetime=ros::Duration();
    //         marker_array.markers.push_back(marker);

    //     }

    //}



}
