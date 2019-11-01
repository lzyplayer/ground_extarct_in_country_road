#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
#include <ros/timer.h>
// ros_msg
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
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
#include <boost/circular_buffer.hpp>
#include <ecl/geometry.hpp>


namespace ground_exract {
    using namespace Eigen;
    using namespace std;
    using namespace boost;
    class Path_provider {
    public:
        Path_provider() {
        }
        virtual  ~Path_provider(){
        }

        void onInit(){
            p_nh = ros::NodeHandle("~");
            low_lines = p_nh.param<int>("low_lines",5);

            // init
            path_pub = nh.advertise<nav_msgs::Path>("/ground_detect/Path_original",2);
            ground_pc_suber = nh.subscribe("/ground_detect/ground_points", 2, &Path_provider::callback, this);
            path_buffer = vector<boost::circular_buffer<Vector3f>>(low_lines,   boost::circular_buffer<Vector3f>(20));

        }

        void callback(const sensor_msgs::PointCloud2ConstPtr& ground_pc_msg) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pc (new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*ground_pc_msg, *ground_pc);
            //check empty
            if (ground_pc->empty())  return;
            nav_msgs::Path path;
            path.header = ground_pc_msg->header;
            // CubicSpline
            ecl::Array<double> x_set(low_lines);
            ecl::Array<double> y_set(low_lines);
            // calculate center point
            int point_p = 0;
            for (int i = 0; i < low_lines; ++i) {
                float sum_x=0, sum_y=0,sum_z=0;
                int line_point_num=0;
                while (int(ground_pc->points[point_p].intensity)==i && point_p<ground_pc->size()){
                    sum_x+=ground_pc->points[point_p].x;
                    sum_y+=ground_pc->points[point_p].y;
                    sum_z+=ground_pc->points[point_p].z;
                    line_point_num++;
                    point_p++;
                }
                if (line_point_num<5)continue;
                geometry_msgs::PoseStamped curr_pose;
                curr_pose.header = ground_pc_msg->header;
                Vector3f pos (sum_x/line_point_num, sum_y/line_point_num, sum_z/line_point_num);
                path_buffer[i].push_back(pos);
                Vector3f path_scan_av = std::accumulate(path_buffer[i].begin(),path_buffer[i].end(),Vector3f(0,0,0));
                path_scan_av/=path_buffer[i].size();
                curr_pose.pose.position.x = path_scan_av[0];
                curr_pose.pose.position.y = path_scan_av[1];
                curr_pose.pose.position.z = path_scan_av[2];
//                 CubicSpline
                cout<<double(path_scan_av[0])<<endl;
                x_set[i] = double(path_scan_av[0]);
                y_set[i] = double(path_scan_av[1]);
                path.poses.push_back(curr_pose);
            }
            ecl::CubicSpline cubic = ecl::CubicSpline::Natural(x_set, y_set);
            cout<<cubic<<endl;
            path_pub.publish(path);


        }

    private:
        ros::NodeHandle nh;
        ros::NodeHandle p_nh;

        ros::Subscriber ground_pc_suber;
        ros::Publisher path_pub;
        vector<boost::circular_buffer<Vector3f>> path_buffer;

        //param
        int low_lines;

    };//End of class SubscribeAndPublish


}    int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "path_provider_node");
    ground_exract::Path_provider pathProvider;
    pathProvider.onInit();
    ros::spin();

    return 0;
}