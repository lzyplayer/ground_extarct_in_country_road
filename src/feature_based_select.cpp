#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
#include <ros/timer.h>
// ros_msg
#include <sensor_msgs/PointCloud2.h>
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
        lowest_segment_point_num = p_nh.param<int>("lowest_segment_point_num",5);
        range_init();
//        poseStamped.pose.position.x
        init_flag= false;
        //suber and puber
        scan_range_pub = nh.advertise<ground_detect::road_range>("/ground_detect/road_range",2);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_detect/ground_points",2);
        region_suber = nh.subscribe("/ground_detect/forward_points", 2, &Ground_extract::points_callback, this);
    }
    void range_init(){
        for (size_t i=0;i<low_lines*2;i++){
            scan_range.road_range.push_back ((pow(-1,i+2))*M_PI/4);
        }
    }


    void points_callback(const sensor_msgs::PointCloud2ConstPtr& point_msg){
//        cout<<"---------start---------"<<endl;
        vector<pcl::PointCloud<pcl::PointXYZI>> lines = line_divide(point_msg);
        vector<vector<vector<int>>> segments_scans_pi;
//        cout<<"--------divded complete----------"<<endl;
        for (int i=0;i<low_lines;++i){
              segments_scans_pi.push_back(generate_segments_in_scan(lines[i]));
        }
//        cout<<"--------selected segments complete----------"<<endl;
        // analyse segments
        pcl::PointCloud<pcl::PointXYZI> ground_cloud;
        if(!init_flag){
            for (int i = 0; i < low_lines; ++i) {
                ground_cloud += *segments_select_init(segments_scans_pi[i],lines[i]);
            }
            init_flag = true;
        }else{
            for (int i = 0; i <  low_lines; ++i) {
                ground_cloud+=*segments_select_run(segments_scans_pi[i],lines[i]);
            }

        }
        scan_range.header.stamp = point_msg->header.stamp;
        scan_range_pub.publish(scan_range);
        pcl_conversions::toPCL(point_msg->header, ground_cloud.header);
        cloud_pub.publish(ground_cloud);
//        cout<<"----cloud_published------"<<endl;
    }
    vector<vector<int>> generate_segments_in_scan(const pcl::PointCloud<pcl::PointXYZI>& scan )const {

        vector<vector<int>> segments;
        if (scan.empty()) return segments;
        int scanID = int(scan[0].intensity);
        vector<int> segment;
        for (int i=0;i<scan.size();++i){
            float diffx =scan.points[i].x-scan.points[i+1].x;
            float diffy =scan.points[i].y-scan.points[i+1].y;
            float diffz =scan.points[i].z-scan.points[i+1].z;

            if(diffx*diffx+diffy*diffy+diffz*diffz*10< distanceSq_threshold+(scanID-1)*0.02 && i!=scan.size()-1)
                segment.push_back(i);
            else{
                if(segment.size()>=lowest_segment_point_num)
                    segments.push_back(segment);
                segment.clear();
            }
        }
        return segments;
    }

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr segments_select_init(const vector<vector<int>>& segments ,const pcl::PointCloud<pcl::PointXYZI>& scan){
        int max_len=0;
        int indi=-1;
        pcl::PointCloud<pcl::PointXYZI>::Ptr final_line (new pcl::PointCloud<pcl::PointXYZI>());
        //empty check
        if (segments.empty()) return final_line;
        for (int i = 0; i < segments.size(); ++i) {
            if (int(segments[i].size())>max_len){
                max_len = int(segments[i].size());
                indi =i;
            }
        }
        //get original in reused intensity
        float left_r =-M_PI/2;
        float right_r = M_PI/2;
        for (int  i :  segments[indi]) {
            float ori = (scan[i].intensity-int(scan[i].intensity) -0.5f)*10;
            if(ori>left_r)
                left_r = ori;
            else if(ori<right_r)
                right_r = ori;
        }
        scan_range.road_range[ 2*int(scan[0].intensity)] = left_r;
        scan_range.road_range[ 2*int(scan[0].intensity)+1] =right_r;

        //generate_line_cloud
        for(int i : segments[indi]){
            final_line->push_back(scan[i]);
        }
        return final_line ;

    }

    pcl::PointCloud<pcl::PointXYZI>::ConstPtr segments_select_run(const vector<vector<int>>& segments ,const pcl::PointCloud<pcl::PointXYZI>& scan){
        int max_len=0;
        int indi=-1;
        pcl::PointCloud<pcl::PointXYZI>::Ptr final_line (new pcl::PointCloud<pcl::PointXYZI>());
        //empty check
        if (segments.empty()) return final_line;
        for (int i = 0; i < segments.size(); ++i) {
            if(segments[i].empty())continue;
            if (int(segments[i].size())>max_len){
                max_len = int(segments[i].size());
                indi =i;
            }
        }
        // calcu most popular height
        float av_z_height=0;
        for(int i : segments[indi]){
            av_z_height+=scan[i].z;
        }
        av_z_height/=segments[indi].size();
        // join based on av_z
        vector<int> sele_indi;
        for (auto& s : segments){
            //check empty
            if (s.empty())continue;
            float  z_height=0;
            for (auto& p: s){
                z_height+=scan[p].z;
            }
            z_height/=s.size();
            if (abs(z_height-av_z_height)< z_threshold){
                sele_indi.push_back(s[0]);
                sele_indi.push_back(s[s.size()-1]);
            }
        }
        for (int i=sele_indi[0];i<=sele_indi[sele_indi.size()-1];++i){
            final_line->push_back(scan[i]);
        }
        //mod angle
        //get original in reused intensity
        float left_r =-M_PI/2;
        float right_r = M_PI/2;
        for (auto& p : final_line->points) {
            float ori = get_ori_from_intensity(p.intensity);
            if(ori>left_r)
                left_r = ori;
            else if(ori<right_r)
                right_r = ori;
        }
        scan_range.road_range[ 2*int(scan[0].intensity)] = left_r;
        scan_range.road_range[ 2*int(scan[0].intensity)+1] =right_r;
        return final_line;
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

        private:
    ros::NodeHandle nh;
    ros::NodeHandle p_nh;
    ros::Publisher cloud_pub;
    ros::Publisher scan_range_pub;
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