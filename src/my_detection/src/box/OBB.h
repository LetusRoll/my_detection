#ifndef OBB_H
#define OBB_H
#include "my_detection/point_types.h"
#include<pcl/features/moment_of_inertia_estimation.h>
#include<visualization_msgs/MarkerArray.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
namespace Box
{
    class OBB
    {
        public:
            OBB(ros::NodeHandle nh,ros::NodeHandle private_nh);
            ~OBB();
            void addbox(PtC::Ptr in_cloud,std::vector<int> clusters_point_indices,const sensor_msgs::PointCloud2::Ptr in,visualization_msgs::MarkerArray &markerarray);
        
            //void addbox (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr > > &cloud_clustered,const sensor_msgs::PointCloud2::Ptr in,visualization_msgs::MarkerArray &marker_array);

        private:

    };
}
#endif
