#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
//#include <ros/timer.h>
// ros_msg
//#include <sensor_msgs/PointCloud2.h>
//#include <nav_msgs/Odometry.h>
// pcl
//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/common/transforms.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/visualization/cloud_viewer.h>
//eigen
#include <Eigen/Dense>
////tf
//#include <tf/transform_datatypes.h>
//#include <tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>
//#include <tf_conversions/tf_eigen.h>

//cpp
//#include <ctime>
#include <math.h>
//#include <mutex>
//#include <boost/circular_buffer.hpp>
//
using namespace Eigen;
using namespace std;



template <typename Derived>
void transform_path_point(const MatrixBase<Derived>& in_points ) {
    //tranform point to fit
    int point_num = in_points.cols();
    double turn_oriten = -atan2(in_points(1,point_num-1),in_points(0,point_num-1));
    Matrix2d rotation_m;
    rotation_m << cos(turn_oriten) , -sin(turn_oriten),sin(turn_oriten),cos(turn_oriten);
    Matrix2d inv_rotation_m;
    inv_rotation_m << cos(-turn_oriten) , -sin(-turn_oriten),sin(-turn_oriten),cos(-turn_oriten);
    //transform to x axes
    Matrix2Xd rotated_path_point = rotation_m * in_points;
    // (optional) if you need Vector
    VectorXd trans_x = rotated_path_point.row(0);
    VectorXd trans_y = rotated_path_point.row(1);
    // do your cubic spine
    //
    // end
    // (optional) if you using Vector
    // put result in trans_x and trans_y
    Matrix2Xd refiltered_points (2,trans_x.size());
    refiltered_points.row(0) = trans_x;
    refiltered_points.row(1) = trans_y;
    //transform back
    Matrix2Xd reversed_path_point = inv_rotation_m * in_points;
}
    int main(int argc, char **argv) {
//        //Initiate ROS
        ros::init(argc, argv, "lidar_analyse_node");
//        Lidar_analyse lidarAnalyse;
//        lidarAnalyse.onInit();
//        ros::spin();

        Vector3d x(6,5,6);
        Vector3d y(1,2,3);
        Matrix2Xd tom (2,x.size());
        tom.row(0) = x;
        tom.row(1) = y;
        transform_path_point(tom);
        //Initiate ROS
//        try
//        Eigen::Vector2d tom(1,2);
//        Eigen::Vector2d maya(2,3);
//        tom = Vector2d::Constant(2,0);
//
//        Eigen::Vector2d caca = (tom+maya)/2;
//        tom.col(0) = Vector2d(1,2);
//        tom(0,1) =1;
//
//        cout<<tom<<endl;

        return 0;
    }

//
//namespace ground_exract {
//
//    class Lidar_analyse {
//    public:
//        Lidar_analyse() {
//        }
//        virtual  ~Lidar_analyse(){
//        }
//
//        void onInit(){
//            p_nh = ros::NodeHandle("~");
//            // init
//
//            //Topic you want to publish
////            pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);
//
//            //Topic you want to subscribe
//            raw_pc_suber = nh.subscribe("/subscribed_topic", 1, &Lidar_analyse::callback, this);
//        }
//
//        void callback(const sensor &input) {
//            PUBLISHED_MESSAGE_TYPE output;
//            .... do something with the input and generate the output...
//            pub_.publish(output);
//        }
//
//    private:
//        ros::NodeHandle nh;
//        ros::NodeHandle p_nh;
//        ros::Publisher pub_;
//        ros::Subscriber raw_pc_suber;
//
//    };//End of class SubscribeAndPublish
//
//    int main(int argc, char **argv) {
//        //Initiate ROS
//        ros::init(argc, argv, "lidar_analyse_node");
//        Lidar_analyse lidarAnalyse;
//        lidarAnalyse.onInit();
//        ros::spin();
//
//        return 0;
//    }
//}