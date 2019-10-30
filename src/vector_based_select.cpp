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
#include <map>
#include "boost/assign.hpp"
#include <ground_detect/LineVector.hpp>
//#include <mutex>
//#include <boost/circular_buffer.hpp>

using namespace std;
using namespace Eigen;


class Ground_line_extract {
public:
    Ground_line_extract() {
    }
    virtual  ~Ground_line_extract(){
    }

    void onInit(){
        p_nh = ros::NodeHandle("~");
        lines = 32 ;
        line_degree_threshold = p_nh.param<int>("line_degree_threshold",15);
        lowest_segment_point_num =  p_nh.param<int>("lowest_segment_point_num",5);
        line_merge_degree_threshold=  p_nh.param<int>("line_merge_degree_threshold",20);
        // init
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_detect/ground_points",2);
        region_suber = nh.subscribe("/ground_detect/filter_local_cloud", 2, &Ground_line_extract::points_callback, this);
//        range_suber = nh.subscribe("/ground_detect/road_range",2,&Lidar_analyse::range_callback,this);
    }


    void points_callback(const sensor_msgs::PointCloud2ConstPtr& point_msg){
        vector<pcl::PointCloud<pcl::PointXYZI>> multi_rings = line_divide(point_msg);
//        vector<vector<ground_exract::LineVector>> multi_ring_segments ;
//        for(auto& pc: multi_rings){
//            multi_ring_segments.push_back(segment_exract(pc));
//        }
        pcl::PointCloud<pcl::PointXYZI> selected_points;
        for(auto& pc: multi_rings){
            vector<int> curr_line_indi = segment_exract(pc);
            for(int i : curr_line_indi){
                selected_points.push_back(pc[i]);
            }
        }
        pcl_conversions::toPCL(point_msg->header,selected_points.header);
        cloud_pub.publish(selected_points);
    }

    vector<int> segment_exract(pcl::PointCloud<pcl::PointXYZI>& line_cloud) const {
        bool in_line= false;
        vector<int> sele_points;
        vector<ground_exract::LineVector> segments_in_line;
        //check empty
        if(line_cloud.empty()) return sele_points;
        ground_exract::LineVector curr_line(int(line_cloud[0].intensity),line_degree_threshold);
        for(size_t i=1;i<line_cloud.size();++i){

            if(!in_line){
                curr_line.indicator.push_back(i);
                curr_line.oritention = Vector3f(line_cloud[i].x-line_cloud[i-1].x,line_cloud[i].y-line_cloud[i-1].y,line_cloud[i].z-line_cloud[i-1].z);
                in_line= true;
            } else{
                if(!curr_line.push_point( Vector3f(line_cloud[i].x-line_cloud[i-1].x,line_cloud[i].y-line_cloud[i-1].y,line_cloud[i].z-line_cloud[i-1].z),i)){
                    in_line= false;
                    if (curr_line.indicator.size()>lowest_segment_point_num){ //line at least points
                        segments_in_line.push_back(curr_line);
                    }
                    curr_line.indicator.clear();
                }
            }
        }
        //check empty
        if(segments_in_line.empty())  return sele_points;
        int max_size=-1;
        int max_index=-1;
        for(int i=0; i<segments_in_line.size();i++){
            if(int(segments_in_line[i].indicator.size())>max_size){
                max_size = segments_in_line[i].indicator.size();
                max_index = i;
            }
        }
        //merge lines based on main orientenion
        sele_points = merge_segments_in_line(segments_in_line, segments_in_line[max_index].oritention);

        return sele_points;


    }
    vector<int> merge_segments_in_line (const vector<ground_exract::LineVector>& segments,const Vector3f& main_ori )const {
        vector<int> points_indi;
        vector<ground_exract::LineVector> merged_segments;
        for(auto& s : segments){
            if(cal_angle(s.oritention,main_ori)<line_merge_degree_threshold*M_PI/180){ //min  angle to merge lines
                points_indi.insert(points_indi.end(),s.indicator.begin(),s.indicator.end());
            }
        }
        return points_indi;
    }

    inline float cal_angle(const Vector3f& vc1 , const Vector3f& vc2 ) const{
        return acos(vc1.cwiseProduct(vc2).sum()/sqrt(vc1.array().square().sum() * vc2.array().square().sum()));
    }


    vector<pcl::PointCloud<pcl::PointXYZI>> line_divide (const sensor_msgs::PointCloud2ConstPtr& p_msg ) const {
        vector<pcl::PointCloud<pcl::PointXYZI>> scan_lines;
        scan_lines.resize(lines);
        pcl::PointCloud<pcl::PointXYZI>::Ptr c_pc(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*p_msg, *c_pc);
        for(size_t i=0 ;i<c_pc->size();i++){
            scan_lines[int(c_pc->points[i].intensity)].push_back(c_pc->points[i]);
        }
        return scan_lines;
    }



private:
    ros::NodeHandle nh;
    ros::NodeHandle p_nh;
    ros::Publisher cloud_pub;
    ros::Subscriber region_suber;

    //param
    int lines;
    int line_degree_threshold;
    int lowest_segment_point_num;
    int line_merge_degree_threshold;

};//End of class SubscribeAndPublish

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "ground_exract_node");
    Ground_line_extract groundLineExtract;
    groundLineExtract.onInit();
    ros::spin();

    return 0;
}

