#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
#include <ros/timer.h>
// ros_msg
#include <sensor_msgs/PointCloud2.h>
//#include <nav_msgs/Odometry.h>
#include <ground_detect/road_range.h>
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
#include <map>
//#include <mutex>
//#include <boost/circular_buffer.hpp>
#include <ground_detect/utility.hpp>

using namespace std;


class Lidar_analyse {
public:
    Lidar_analyse() {
    }
    virtual  ~Lidar_analyse(){
    }

    void onInit(){
        p_nh = ros::NodeHandle("~");
        // init
        low_lines = p_nh.param<int>("low_lines",6);
//        z_threshold = p_nh.param<float>("z_threshold",0.02);
//        distance_threshold = p_nh.param<float>("distance_threshold",0.2);
        range_init();
        //suber and puber
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_detect/forward_points",2);
        raw_pc_suber = nh.subscribe("/lidar/vlp32_middle/PointCloud2", 2, &Lidar_analyse::points_callback, this);
        range_suber = nh.subscribe("/ground_detect/road_range",2,&Lidar_analyse::range_callback,this);
    }
    void range_init(){
        for (size_t i=0;i<low_lines*2;i++){
            scan_range.road_range.push_back ((pow(-1,i+2))*M_PI/5);
        }
    }

    void range_callback(const ground_detect::road_rangeConstPtr& range_msg){
        scan_range.road_range  = range_msg->road_range;
        scan_range.header = range_msg->header;
    }

    void points_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)  {
        vector<pcl::PointCloud<pcl::PointXYZI> > laserCloudScans;
        laserCloudScans.resize(low_lines);
        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloud_msg, *curr_cloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr lines_cloud (new pcl::PointCloud<pcl::PointXYZI>());
        // per point
        for (size_t i =0 ; i < curr_cloud->size();i++) {
            pcl::PointXYZI p;
            p.x = curr_cloud->points[i].x;
            p.y = curr_cloud->points[i].y;
            p.z = curr_cloud->points[i].z;
            // skip NaN and INF valued points
            if (!pcl_isfinite(p.x) ||
                !pcl_isfinite(p.y) ||
                !pcl_isfinite(p.z)) {
                continue;
            }
            if (p.x < 0) {
                continue;
            }
            // calculate vertical point angle and scan ID
            float angle = atan(p.z / std::sqrt(p.x * p.x + p.y * p.y));
            int scanID = m.find(int((angle * 180 / M_PI) * 10+sgn(angle) *0.01f))->second;
            if (scanID >= low_lines)
                continue;
            // calculate horzion ori in range
            float ori = atan2(p.y, p.x);

            if (ori < scan_range.road_range[scanID * 2 ]+(M_PI/9/(scanID+1)) && ori > scan_range.road_range[scanID * 2 + 1]-(M_PI/9/(scanID+1))) {
                p.intensity = float(scanID)+ori/10+0.5f;
    //            cout<<"p.intensity: "<<p.intensity<<endl;
                laserCloudScans[scanID].push_back(p);
            }
        }

        //pub
        pcl::PointCloud<pcl::PointXYZI> selectPoints;
        pcl_conversions::toPCL(cloud_msg->header, selectPoints.header);
        for (int i=0; i<low_lines;i++){
            selectPoints = selectPoints+laserCloudScans[i];
        }

         cloud_pub.publish(selectPoints);


    }




private:
    ros::NodeHandle nh;
    ros::NodeHandle p_nh;
    ros::Publisher cloud_pub;
    ros::Subscriber raw_pc_suber;
    ros::Subscriber range_suber;




    //param
    const map<int,int> m = {{150,31}, {103,30}, {70,29}, {46,28}, {33,27}, {23,26}, {16,25}, {13,24}, {10,23}, {6,22}, {3,21}, {0,20}, {-3,19}, {-6,18}, {-10,17}, {-13,16}, {-16,15}, {-20,14}, {-23,13}, {-26,12}, {-30,11}, {-33,10}, {-36,9}, {-40,8}, {-46,7}, {-53,6}, {-61,5}, {-72,4}, {-88,3}, {-113,2}, {-156,1}, {-250,0}};
    int low_lines;
//    float z_threshold;
//    float distance_threshold;
    ground_detect::road_range scan_range;

};//End of class SubscribeAndPublish

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "lidar_analyse_node");
    Lidar_analyse lidarAnalyse;
    lidarAnalyse.onInit();
    ros::spin();

    return 0;
}

//        //per line
//        for (int i = 0;i<low_lines;i++){
//            long int N = laserCloudScans[i].size();
//            for(size_t j=0;j<10;j++){
//                laserCloudScans[i].push_back(laserCloudScans[i].points[j]);
//            }
//            //per line point
//            for (size_t j=5;j<N+5;j++){
//                int inline_num =0;
//                pcl::PointXYZI sor = laserCloudScans[i].points[j];
//                for (int tar_ind =1;tar_ind<=5;tar_ind++){
//                    pcl::PointXYZI tar = laserCloudScans[i].points[j-tar_ind];
//                    if(abs(sor.z-tar.z)<z_threshold){
//                        float diffx = sor.x-tar.x;
//                        float diffy = sor.y-tar.y;
//                        float diffz = sor.z-tar.z;
//                        if ((diffx*diffx + diffy*diffy +diffz*diffz)<distance_threshold*distance_threshold)
//                            inline_num++;
//                    }
//                }
//                if (inline_num>5)
//                    selectPoints.push_back(sor);
//            }
//        }