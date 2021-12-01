#include"../include/my_detection/point_types.h"
#include"clip/clip.h"
#include"remove_ground/remove_ground.h"
#include"cluster/CVC.h"
//#include"cluster/CVC_cluster.h"
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




inline bool IsVehicle(const autosense::ObjectPtr &object)
{
    if((object->length>3.0)&&(object->length<6.0))
    {
        if((object->width>1.5)&&(object->width<4.0))
        {
            if((object->height>1.0)&&(object->height<3.0))
                return true;
        }
    }
    else
    {
        return false;
    }
}

class Perception
{
    private:
    float delta_box_length;
    Clip clip;
    RemoveGround rm_ground;
    CVC cvc_cluster;
    ros::Subscriber sub;
    ros::Publisher clipped_pub,ground_cloud_pub,non_ground_cloud_pub,cluster_pub,box_pub,vehicle_pub;
    public:
    Perception(ros::NodeHandle &nh,ros::NodeHandle &private_nh);
    void BigBox2SmallBox(autosense::ObjectPtr &object,vector<PtC::Ptr> &small_clusters);
    void Callback(const sensor_msgs::PointCloud2::Ptr &msg);
    ~Perception();
};