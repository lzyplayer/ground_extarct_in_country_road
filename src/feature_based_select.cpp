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
#include "boost/assign.hpp"
//#include <mutex>
//#include <boost/circular_buffer.hpp>
#include <ground_detect/LinePart.hpp>

using namespace std;


class Ground_extract {
public:
    Ground_extract() {
    }
    virtual  ~Ground_extract(){
    }

    void onInit(){
        p_nh = ros::NodeHandle("~");
        // init
        low_lines = p_nh.param<int>("low_lines",6);
        distanceSq_threshold = p_nh.param<float>("distance_threshold",0.1f);
        distanceSq_threshold *= distanceSq_threshold;
        z_threshold =p_nh.param<float>("z_threshold",0.03f);
//        z_low_threshold =p_nh.param<float>("z_low_threshold",-2.1f);
        lowest_segment_point_num = p_nh.param<int>("lowest_segment_point_num",5);
        range_init();
//        z_threshold = p_nh.param<float>("z_threshold",0.02);
//        distance_threshold = p_nh.param<float>("distance_threshold",0.2);
        init_flag= false;
        //suber and puber
//        scan_range_pub = nh.advertise<ground_detect::road_range>("/ground_detect/road_range",2);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_detect/ground_points",2);
        region_suber = nh.subscribe("/ground_detect/forward_points", 2, &Ground_extract::points_callback, this);
//        range_suber = nh.subscribe("/ground_detect/road_range",2,&Lidar_analyse::range_callback,this);
    }
    void range_init(){
        for (size_t i=0;i<low_lines*2;i++){
            scan_range.road_range.push_back ((pow(-1,i+2))*M_PI/2);
        }
    }

    void points_callback(const sensor_msgs::PointCloud2ConstPtr& point_msg){
//        cout<<"---------start---------"<<endl;
        vector<pcl::PointCloud<pcl::PointXYZI>> lines = line_divide(point_msg);
        vector<vector<pcl::PointCloud<pcl::PointXYZI>>> line_parts_set;
//        cout<<"--------divded complete----------"<<endl;
        for (int i=0;i<low_lines;++i){

            vector<pcl::PointCloud<pcl::PointXYZI>> line_segments;
            pcl::PointCloud<pcl::PointXYZI> curr_line = lines[i];
            // may have no points
            if(curr_line.empty())   {
                line_parts_set.push_back(line_segments);
                continue;
            }
            curr_line.push_back(curr_line.points[0]);
            uint N = curr_line.size();
            pcl::PointCloud<pcl::PointXYZI> segment;
            for (int j=0;j<curr_line.size();++j){
                float diffx =curr_line.points[j].x-curr_line.points[j+1].x;
                float diffy =curr_line.points[j].y-curr_line.points[j+1].y;
                float diffz =curr_line.points[j].z-curr_line.points[j+1].z;

                if(diffx*diffx+diffy*diffy+diffz*diffz*10< distanceSq_threshold && j!=N-1)
                    segment.push_back(curr_line.points[j]);
                else{
                    if(segment.size()>=lowest_segment_point_num)
                        line_segments.push_back(segment);
                    segment.clear();
                }
            }
            line_parts_set.push_back(line_segments);
        }
//        cout<<"--------selected segments complete----------"<<endl;
        // analyse segments
        pcl::PointCloud<pcl::PointXYZI> ground_cloud;
        if(!init_flag){
            for (int i = 0; i < low_lines; ++i) {
                int max_len=0;
                int indi=-1;
                //line_parts_set maybe empty
                if(line_parts_set.empty()) continue;
                vector<pcl::PointCloud<pcl::PointXYZI>> segments = line_parts_set[i];
                //segments maybe empty
                if(segments.empty()) continue;
                //
                for (int j = 0; j < segments.size(); ++j) {
                    if (segments[j].size()>max_len){
                        max_len= segments[j].size();
                        indi=j;
                    }
                }
                //get original in reused intensity
                float left_r =-M_PI/2;
                float right_r = M_PI/2;
                for (auto& p : segments[indi].points) {
                    float ori = (p.intensity-int(p.intensity) -0.5f)*10;
                    if(ori>left_r)
                        left_r = ori;
                    else if(ori<right_r)
                        right_r = ori;
                }
                scan_range.road_range[ 2*i] = left_r;
                scan_range.road_range[ 2*i+1] =right_r;

                //set cloud
                ground_cloud+=segments[indi];
                init_flag = true;
            }
        }else{

            for (int i = 0; i <  low_lines; ++i) {
                //line_parts_set maybe empty
                if(line_parts_set.empty()) continue;
                vector<pcl::PointCloud<pcl::PointXYZI>> segments = line_parts_set[i];
                auto line_cloud =  segment_join(segments);
                //segments maybe empty
                if(line_cloud->empty()) continue;
                //get original in reused intensity
                float left_r =-M_PI/2;
                float right_r = M_PI/2;
                for (auto& p : line_cloud->points) {
                    float ori = get_ori_from_intensity(p.intensity);
                    if(ori>left_r)
                        left_r = ori;
                    else if(ori<right_r)
                        right_r = ori;
                }
                scan_range.road_range[ 2*i] = left_r;
                scan_range.road_range[ 2*i+1] =right_r;
                ground_cloud+=*line_cloud;

            }

        }
        scan_range.header.stamp = point_msg->header.stamp;
//        scan_range_pub.publish(scan_range);
        pcl_conversions::toPCL(point_msg->header, ground_cloud.header);
        cloud_pub.publish(ground_cloud);
//        cout<<"----cloud_published------"<<endl;
    }

    vector<pcl::PointCloud<pcl::PointXYZI>> line_divide (const sensor_msgs::PointCloud2ConstPtr& p_msg ) const {
        vector<pcl::PointCloud<pcl::PointXYZI>> scan_lines;
        scan_lines.resize(low_lines);
        pcl::PointCloud<pcl::PointXYZI>::Ptr c_pc(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*p_msg, *c_pc);
        for(size_t i=0 ;i<c_pc->size();i++){
//            if(c_pc->points[i].z>z_low_threshold)
                scan_lines[int(c_pc->points[i].intensity)].push_back(c_pc->points[i]);
        }
        return scan_lines;
    }
    float  get_ori_from_intensity(float intensity) const {
        return  (intensity-int(intensity) -0.5f)*10;
    }

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr segment_join (const vector<pcl::PointCloud<pcl::PointXYZI>>& segments_in_line) const {
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged_line  (new  pcl::PointCloud<pcl::PointXYZI>());
        //check empty
        if(segments_in_line.empty())
            return merged_line;
        //get average height
         pcl::PointCloud<pcl::PointXYZI> first_len;
        for (auto& s : segments_in_line){
            //check empty
            if (s.empty())continue;
            if(s.size()>first_len.size()) first_len=s;
        }
        //check empty
        if(first_len.empty())
            return merged_line;
        float av_z_height=0;
        for (auto& p : first_len){
            av_z_height+=p.z;
        }
        av_z_height/=first_len.size();
        //merge segments
        for (auto& s : segments_in_line){
            //check empty
            if (s.empty())continue;
            float  z_height=0;
            for (auto& p: s){
                z_height+=p.z;
            }
            z_height/=s.size();
            if (abs(z_height-av_z_height)< z_threshold){

                    *merged_line+=s;
            }
        }
        return merged_line;
//        //check first two lines
//        int segment_num = segments_in_line.size();
//        pcl::PointCloud<pcl::PointXYZI> first_len;
//        pcl::PointCloud<pcl::PointXYZI> second_len;
//        for (auto& s : segments_in_line){
//            //check empty
//            if (s.empty())continue;
//            if(s.size()>second_len.size()){
//                if(s.size()>first_len.size()){
//                    second_len=first_len;
//                    first_len = s;
//                } else second_len=s;
//            }
//        }
//        if(second_len.size()>8)
//            *merged_line = first_len+second_len;
//        else
//            *merged_line = first_len;

    }

        private:
    ros::NodeHandle nh;
    ros::NodeHandle p_nh;
    ros::Publisher cloud_pub;
//    ros::Publisher scan_range_pub;
    ros::Subscriber region_suber;
//    ros::Subscriber range_suber;




    //param
    int low_lines;
    int lowest_segment_point_num;
    float distanceSq_threshold;
    float z_threshold;
//    float z_low_threshold;

    bool init_flag;
    ground_detect::road_range scan_range;

};//End of class SubscribeAndPublish

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "ground_exract_node");
    Ground_extract groundExtract;
    groundExtract.onInit();
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
//for (auto& n : segments)
//curr_line_full_cloud += n;