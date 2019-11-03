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
            low_lines = p_nh.param<int>("low_lines",4);
            abandon_min_points_num = p_nh.param<int>("abandon_min_points_num",12);
            geometry_center_frame  ="geometry_center";
            //frame_select 1.geometry_center 2.velo_middle
            // init
            curved_path_pub = nh.advertise<nav_msgs::Path>("/ground_detect/Path_curved",2);

            path_pub = nh.advertise<nav_msgs::Path>("/ground_detect/Path_original",2);
            ground_pc_suber = nh.subscribe("/ground_detect/ground_points", 2, &Path_provider::callback, this);
            // path buffer intialise
            path_buffer = vector<boost::circular_buffer<Vector3d>>(low_lines,   boost::circular_buffer<Vector3d>(15));
            M_geo_vleom << 0.999971,-0.007000,-0.003000,-0.064900,0.007000,0.999976,-0.000021,0.000000,0.003000,-0.000000,0.999996,1.550000,0.000000,0.000000,0.000000,1.000000;
//            M_geo_vleom.setIdentity();
        }

        void callback(const sensor_msgs::PointCloud2ConstPtr& ground_pc_msg) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pc (new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*ground_pc_msg, *ground_pc);
            //check empty
            if (ground_pc->empty())  return;
            nav_msgs::Path path;
            path.header = ground_pc_msg->header;
            path.header.frame_id=geometry_center_frame;
            //  center path point init
            Matrix2Xd path_point(2,low_lines);
            //add start point
            int point_p = 0; int path_point_num=0;
            for (int i = 0; i < low_lines; ++i) {
                float sum_x=0, sum_y=0,sum_z=0;
                int line_point_num=0;
                while (point_p<ground_pc->size() && int(ground_pc->points[point_p].intensity)==i){
                    sum_x+=ground_pc->points[point_p].x;
                    sum_y+=ground_pc->points[point_p].y;
                    sum_z+=ground_pc->points[point_p].z;
                    line_point_num++;
                    point_p++;
                }
                //check point num
                if (line_point_num<4) continue;
                geometry_msgs::PoseStamped curr_pose;
                curr_pose.header = ground_pc_msg->header;
                curr_pose.header.frame_id = geometry_center_frame;
                Vector3d pos (sum_x/line_point_num, sum_y/line_point_num, sum_z/line_point_num);
                path_buffer[i].push_back(pos);
                Vector3d path_scan_av = std::accumulate(path_buffer[i].begin(),path_buffer[i].end(),Vector3d(0,0,0));
                path_scan_av/=path_buffer[i].size();
                Vector4d simi_path_scan_av = Vector4d::Constant(1);
                simi_path_scan_av.head(3) = path_scan_av;
//                cout<<simi_path_scan_av<<endl;

                Vector4d path_scan_av_geo = M_geo_vleom * simi_path_scan_av;
                curr_pose.pose.position.x = path_scan_av_geo[0];
                curr_pose.pose.position.y = path_scan_av_geo[1];
                curr_pose.pose.position.z = path_scan_av_geo[2];
                //  note point
                path_point.col(path_point_num) = path_scan_av.head(2);
                path_point_num++;
                path.poses.push_back(curr_pose);
            }
            //transform point for curve
            //point low check and add mid
            if(path_point_num<=2){
                if(path_point_num<2) return;
//                path_point.col(path_point_num-1) = (path_point.col(path_point_num) +path_point.col(path_point_num-2))/2;
                Matrix2d point_double_matrpath = path_point.block(0,0,2,2);
                path_point.block(0,1,2,2)=point_double_matrpath;
//                cout<<"path_point.block(0,1,2,2)"<<endl<<path_point.block(0,1,2,2)<<endl;
                path_point.col(0) = Vector2d::Constant(2,0);
                path_point_num++;
//                cout<<"path_point.block(0,0,2,path_point_num)"<<endl<<path_point.block(0,0,2,path_point_num)<<endl;
            }
            const auto& curved_path = transform_path_point(path_point.block(0,0,2,path_point_num),path_point_num,ground_pc_msg->header);
            curved_path_pub.publish(*curved_path);
            path_pub.publish(path);


        }


        template <typename Derived>
        nav_msgs::PathConstPtr transform_path_point(const MatrixBase<Derived>& in_points ,const int point_num ,const std_msgs::Header head) {
            //tranform point to fit
            double turn_oriten = -atan2(in_points(1,point_num-1),in_points(0,point_num-1));
            Matrix2d rotation_m;
            rotation_m << cos(turn_oriten) , -sin(turn_oriten),sin(turn_oriten),cos(turn_oriten);
            Matrix2d inv_rotation_m;
            inv_rotation_m << cos(-turn_oriten) , -sin(-turn_oriten),sin(-turn_oriten),cos(-turn_oriten);
            //transform to x axes
//            cout<<"in_points.block(0,0,2,point_num)"<<endl<<in_points.block(0,0,2,point_num)<<endl;

            Matrix2Xd rotated_path_point = rotation_m * in_points.block(0,0,2,point_num);
//            cout<<"rotated_path_point"<<endl<<rotated_path_point<<endl;
            // CubicSpline init
            ecl::Array<double> x_set(point_num);
            ecl::Array<double> y_set(point_num);
            double x_max_range = rotated_path_point(0,point_num-1);
            for(int i=0;i<point_num;++i){
                x_set[i] = rotated_path_point(0,i);
                y_set[i] = rotated_path_point(1,i);
            }
//            cout<<"x_set"<<x_set<<endl;
//            cout<<"y_set"<<y_set<<endl;
            //curve func 1.cubicSpline Natural 2.Smooth Linear Spline
            ecl::CubicSpline cubic = ecl::CubicSpline::Natural(x_set, y_set);
//            double max_curvature = 0.5;
//            SmoothLinearSpline spline(x_set, y_set, max_curvature);
            //ready path
            nav_msgs::PathPtr pathPtr(new nav_msgs::Path());
            pathPtr->header = head;
            pathPtr->header.frame_id=geometry_center_frame;
            for (int j = 0; j <x_max_range*10; ++j) {
                double x_var =double(j)/10.0f;
                double y_var =cubic(x_var);
                if(y_var!=y_var || abs(y_var)>50){
                    ROS_INFO("nan or large move detected! using last path, use caution!");
                }
                if( j<5)cout<<"y_var "<<y_var<<endl;
                Vector4d point_velm(cos(-turn_oriten)*x_var -sin(-turn_oriten)*y_var,sin(-turn_oriten)*x_var+cos(-turn_oriten)*y_var,0,1);
                Vector4d point_geo = M_geo_vleom *point_velm;
                geometry_msgs::PoseStamped curr_pose;
                curr_pose.header = head;
                curr_pose.header.frame_id=geometry_center_frame;
                curr_pose.pose.position.x = point_geo(0);
                curr_pose.pose.position.y = point_geo(1);
                curr_pose.pose.position.z = 0;
                pathPtr->poses.push_back(curr_pose);

            }
            last_path_ptr = pathPtr;
            return pathPtr;
        }

    private:
        ros::NodeHandle nh;
        ros::NodeHandle p_nh;

        ros::Subscriber ground_pc_suber;
        ros::Publisher path_pub;
        ros::Publisher curved_path_pub;
        vector<boost::circular_buffer<Vector3d>> path_buffer;
        Matrix4d M_geo_vleom;

        //param
        int low_lines;
        int abandon_min_points_num;
        string geometry_center_frame;

        //history
        nav_msgs::PathPtr last_path_ptr;

    };//End of class SubscribeAndPublish


}    int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "path_provider_node");
    ground_exract::Path_provider pathProvider;
    pathProvider.onInit();
    ros::spin();

    return 0;
}