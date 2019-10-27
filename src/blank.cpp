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

    class Lidar_analyse {
    public:
        Lidar_analyse() {
        }
        virtual  ~Lidar_analyse(){
        }

        void onInit(){
            p_nh = ros::NodeHandle("~");
            // init

            //Topic you want to publish
//            pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);

            //Topic you want to subscribe
            raw_pc_suber = nh.subscribe("/subscribed_topic", 1, &Lidar_analyse::callback, this);
        }

        void callback(const sensor &input) {
            PUBLISHED_MESSAGE_TYPE output;
            .... do something with the input and generate the output...
            pub_.publish(output);
        }

    private:
        ros::NodeHandle nh;
        ros::NodeHandle p_nh;
        ros::Publisher pub_;
        ros::Subscriber raw_pc_suber;

    };//End of class SubscribeAndPublish

    int main(int argc, char **argv) {
        //Initiate ROS
        ros::init(argc, argv, "lidar_analyse_node");
        Lidar_analyse lidarAnalyse;
        lidarAnalyse.onInit();
        ros::spin();

        return 0;
    }
}