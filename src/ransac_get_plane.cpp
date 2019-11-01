#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
#include <ros/timer.h>
// ros_msg
#include <sensor_msgs/PointCloud2.h>
//#include <nav_msgs/Odometry.h>
// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/visualization/cloud_viewer.h>
//eigen
#include <Eigen/Dense>
//tf
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

//cpp
#include <ctime>
#include <math.h>
//#include <mutex>
//#include <boost/circular_buffer.hpp>


namespace ground_exract {

    class Ransac_plane {
    public:
        Ransac_plane() {
        }
        virtual  ~Ransac_plane(){
        }

        void onInit(){
            p_nh = ros::NodeHandle("~");
            // init
            distance_threshold = p_nh.param<float>("distance_threshold",0.05);
            //Topic you want to subscribe
            ground_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_detect/ground_points",2);
            raw_pc_suber = nh.subscribe("/ground_detect/filter_local_cloud", 2, &Ransac_plane::callback, this);
        }

        void callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr ori_pc (new pcl::PointCloud<pcl::PointXYZ> ());
            pcl::fromROSMsg(*pc_msg, *ori_pc);

            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);

            seg.setDistanceThreshold (distance_threshold);

            seg.setInputCloud (ori_pc);
            seg.segment (*inliers, *coefficients);

            pcl::PointCloud<pcl::PointXYZ>::Ptr result_pc (new pcl::PointCloud<pcl::PointXYZ> ());


            for (int indice : inliers->indices){
                result_pc->push_back(ori_pc->points[indice]);
            }
            pcl_conversions::toPCL(pc_msg->header,result_pc->header);
            ground_pub.publish(result_pc);

        }

    private:
        ros::NodeHandle nh;
        ros::NodeHandle p_nh;
        ros::Publisher ground_pub;
        ros::Subscriber raw_pc_suber;

        float distance_threshold;

    };//End of class SubscribeAndPublish


}
int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "plane_get_node");
    ground_exract::Ransac_plane ransacPlane;
    ransacPlane.onInit();
    ros::spin();

    return 0;
}