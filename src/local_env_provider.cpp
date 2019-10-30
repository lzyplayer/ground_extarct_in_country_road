#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
#include <ros/timer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
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
//#include <mutex>
//#include <boost/circular_buffer.hpp>


namespace ground_exract {

    using namespace Eigen;
    using namespace std;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> pioneer_lidar_sync_policy;



    class Lidar_filter {
    public:
        Lidar_filter() {
           }
        virtual  ~Lidar_filter(){
        }

        void onInit(){
            p_nh = ros::NodeHandle("~");
            // init
            Motion_ml << 0.115046251181516,-0.993042339189164,0.0251251401347837,-0.321385000000000,0.735848984094894,0.102186185526057,0.669390959077090,0.678507000000000,-0.667301006065173,-0.0585226115731963,0.742485334022875,-0.354922000000000,0,0,0,1;
            Motion_mr <<-0.034197,0.999413,0.002200,-0.342517,-0.759464,-0.024556,-0.650086,-0.759176,-0.649650,-0.023902,0.759857,-0.323078,0.000000,0.000000,0.000000,1.000000;
            //            Motion_mr << -0.0270922226574828,0.999630437289618,0.00223613855685278,-0.342517000000000,-0.759610091255644,-0.0191328889883868,-0.650097255663759,-0.759176000000000,-0.649814220169173,-0.0193111730126922,0.759847720180040,-0.323078000000000,0,0,0,1;
            //Topic you want to subscribe
//            raw_pc_suber = nh.subscribe("/subscribed_topic", 1, &Lidar_analyse::callback, this);
            left_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/lidar/vlp16_left/PointCloud2",5);
            rigth_sub  = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh,"/lidar/vlp16_right/PointCloud2",5);
            middle_sub  = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh,"/lidar/vlp32_middle/PointCloud2",5);
            sync = new message_filters::Synchronizer<pioneer_lidar_sync_policy> (pioneer_lidar_sync_policy(10),*left_sub,*rigth_sub,*middle_sub);
            sync->registerCallback(boost::bind(&Lidar_filter::points_callback,this,_1,_2,_3));
            filterd_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_detect/filter_local_cloud",2);
        }

        void points_callback (const sensor_msgs::PointCloud2ConstPtr& left_pc ,const sensor_msgs::PointCloud2ConstPtr& right_pc ,const sensor_msgs::PointCloud2ConstPtr& middle_pc ) const{
            pcl::PointCloud<pcl::PointXYZI>::Ptr left_ori (new pcl::PointCloud<pcl::PointXYZI> ());
            pcl::PointCloud<pcl::PointXYZI>::Ptr right_ori (new pcl::PointCloud<pcl::PointXYZI> ());
//            pcl::PointCloud<pcl::PointXYZI>::Ptr middle_ori (new pcl::PointCloud<pcl::PointXYZI> ());
            pcl::fromROSMsg(*left_pc, *left_ori);
            pcl::fromROSMsg(*right_pc, *right_ori);
//            pcl::fromROSMsg(*middle_pc, *middle_ori);
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_left (new pcl::PointCloud<pcl::PointXYZI> ());
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_right (new pcl::PointCloud<pcl::PointXYZI> ());
            pcl::transformPointCloud (*left_ori, *transformed_cloud_left, Motion_ml);
            pcl::transformPointCloud (*right_ori, *transformed_cloud_right, Motion_mr);
            pcl::PointCloud<pcl::PointXYZI> merged_full_cloud  = *transformed_cloud_left + *transformed_cloud_right;
//            merged_full_cloud += *middle_ori;
//            pcl::PointCloud<pcl::PointXYZI> merged_full_cloud  = *transformed_cloud_left ;
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
            filtered_cloud->reserve(merged_full_cloud.size()/5);
            std::copy_if(merged_full_cloud.begin(),merged_full_cloud.end(),std::back_inserter(filtered_cloud->points),
                         [&](const pcl::PointXYZI& p){
                             return p.x>0 && p.x<20 && abs(p.y)<12 && p.z<-1.3;
                         }
            );
            filtered_cloud->width = filtered_cloud->points.size();
            filtered_cloud->height = 1;
            pcl_conversions::toPCL(middle_pc->header,filtered_cloud->header);
            //pub
            filterd_cloud_pub.publish(filtered_cloud);
        }

    private:
        ros::NodeHandle nh;
        ros::NodeHandle p_nh;
        //puber
        ros::Publisher filterd_cloud_pub;
        //suber
        message_filters::Subscriber<sensor_msgs::PointCloud2> *left_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> *rigth_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> *middle_sub;
        message_filters::Synchronizer<pioneer_lidar_sync_policy> *sync;
        //param
         Matrix4f Motion_ml;
         Matrix4f Motion_mr;


    };//End of class SubscribeAndPublish

}

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "lidar_filter_node");
    ground_exract::Lidar_filter lidarFilter;
    lidarFilter.onInit();
    ros::spin();

    return 0;
}

