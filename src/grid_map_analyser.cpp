#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
#include <ros/timer.h>

// ros_msg
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
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
#include <ground_detect/Grid_map.hpp>
//#include <mutex>
//#include <boost/circular_buffer.hpp>


namespace ground_exract {

    using namespace Eigen;
    using namespace std;



    class Grid_map {
    public:
        Grid_map() {
        }
        virtual  ~Grid_map(){
        }

        void onInit(){
//            occ map
            resolution = 0.2;
            x_meters = 20;
            y_meters = 24;
            frame_name = "velo_middle";
            cloud_sub = nh.subscribe("/ground_detect/filter_local_cloud",2,&Grid_map::point_callback,this);
            grid_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/ground_detect/env_map",2, true);

        }
        void point_callback(const sensor_msgs::PointCloud2ConstPtr& point_msg){
            GridMapUtility gridMapUtility(x_meters,y_meters,resolution);
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud  (new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*point_msg,*pointCloud);
            gridMapUtility.pc_map_analyse(pointCloud);
            map_publish(gridMapUtility.occupancyGrid);

        }
//        void inline

        void map_publish(nav_msgs::OccupancyGrid& occupancyGrid){
            occupancyGrid.header.frame_id=frame_name;
            grid_map_pub.publish(occupancyGrid);
        }

    private:
        ros::NodeHandle nh;
        ros::NodeHandle p_nh;
        //puber
        ros::Publisher grid_map_pub;
        //suber
        ros::Subscriber cloud_sub;
        //param
        float resolution;
        float x_meters;
        float y_meters;
        string frame_name;

    };//End of class SubscribeAndPublish

}

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "grid_map_provider");
    ground_exract::Grid_map gridMap;
    gridMap.onInit();
    ros::spin();

    return 0;
}

//occ map
//nav_msgs::OccupancyGrid occupancyGrid;
//occupancyGrid.header.frame_id="world";
//occupancyGrid.info.resolution=1;
//occupancyGrid.info.width=10;
//occupancyGrid.info.height=10;
//occupancyGrid.data.push_back(100);
//occupancyGrid.data.push_back(0);
//occupancyGrid.data.push_back(100);
//occupancyGrid.data.push_back(0);
//occupancyGrid.data.push_back(100);
//occupancyGrid.data.push_back(0);
//
////Topic you want to publish
//pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/OC", 1,true);
//pub_.publish(occupancyGrid);