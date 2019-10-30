//
// Created by vickylzy on 19-10-29.
//

#ifndef SRC_GRID_MAP_HPP
#define SRC_GRID_MAP_HPP

#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>

namespace ground_exract{

    using namespace std;
    using namespace boost::accumulators;

class GridMapUtility{


public:
    GridMapUtility(float xMeters = 20, float yMeters = 24, float resolution = 0.25) : _x_meters(xMeters), _y_meters(yMeters),
                                                                     _resolution(resolution) {
        _width=   int(_x_meters/_resolution);
        _height = int(_y_meters/_resolution);
        occupancyGrid.info.origin.orientation.x=0;
        occupancyGrid.info.origin.orientation.y=0;
        occupancyGrid.info.origin.orientation.z=0;
        occupancyGrid.info.origin.orientation.w=0;
        occupancyGrid.info.origin.position.x=0;
        occupancyGrid.info.origin.position.y=-1*(_y_meters/2);
        occupancyGrid.info.width=_width;
        occupancyGrid.info.height=_height;
        occupancyGrid.info.resolution=_resolution;
        _indicator = vector<vector<uint16_t >>(_height*_width);
        occupancyGrid.data = vector<int8_t >(_height*_width,100);
    }
    void pc_map_analyse(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& point_cloud){
        for(size_t i=0;i<point_cloud->size();++i){
            if(point_cloud->points[i].x==point_cloud->points[i].x)
                _indicator[get_cell_ID(point_cloud->points[i].x,point_cloud->points[i].y)].push_back(i);
        }
        for(size_t j=_width;j<=(_width*(_height-1));++j){
            //skep edge
            if (j%_width==0 || (j+1)%_width==0)
                continue;
            int point_size = _indicator[j].size()+_indicator[up(j)].size()+_indicator[down(j)].size()+_indicator[left(j)].size()+_indicator[right(j)].size()+_indicator[up(left(j))].size()+_indicator[down(left(j))].size()+_indicator[up(right(j))].size()+_indicator[down(right(j))].size();
            if(point_size>1){
                vector<uint16_t > curr_indicator = merge_v(j,point_size);
                vector<float> z_value;
                for( uint16_t& it : curr_indicator){
                    z_value.push_back(point_cloud->points[it].z*100);
                }
                accumulator_set<float , stats<tag::variance> > acc;
                for_each(z_value.begin(),z_value.end(),bind<void>(ref(acc),_1));

                float var_f = variance(acc);
                if(var_f<30)
                    occupancyGrid.data[j]=var_f*3.3;

            }

        }
    }

    vector<uint16_t > merge_v(int curr_ID,int point_size){
        vector<uint16_t > merged_vector;
        merged_vector.reserve(point_size);
        merged_vector.insert(merged_vector.end(),_indicator[curr_ID].begin(),_indicator[curr_ID].end());
        merged_vector.insert(merged_vector.end(),_indicator[up(curr_ID)].begin(),_indicator[up(curr_ID)].end());
        merged_vector.insert(merged_vector.end(),_indicator[down(curr_ID)].begin(),_indicator[down(curr_ID)].end());
        merged_vector.insert(merged_vector.end(),_indicator[left(curr_ID)].begin(),_indicator[left(curr_ID)].end());
        merged_vector.insert(merged_vector.end(),_indicator[right(curr_ID)].begin(),_indicator[right(curr_ID)].end());
        merged_vector.insert(merged_vector.end(),_indicator[up(left(curr_ID))].begin(),_indicator[up(left(curr_ID))].end());
        merged_vector.insert(merged_vector.end(),_indicator[down(left(curr_ID))].begin(),_indicator[down(left(curr_ID))].end());
        merged_vector.insert(merged_vector.end(),_indicator[up(right(curr_ID))].begin(),_indicator[up(right(curr_ID))].end());
        merged_vector.insert(merged_vector.end(),_indicator[down(right(curr_ID))].begin(),_indicator[down(right(curr_ID))].end());
        return merged_vector;

    }

    inline int up(int curr_ID){
        return curr_ID+1;
    }
    inline int down(int curr_ID){
        return curr_ID-1;
    }

    inline int right(int curr_ID){
        return curr_ID - _width;
    }
    inline int left(int curr_ID){
        return curr_ID + _width;
    }


    inline int get_cell_ID(float x, float y){
        int re = (int ((y+_y_meters/2)/_resolution))  *_width+int(x/_resolution);
        return  (int ((y+_y_meters/2)/_resolution))  *_width+int(x/_resolution);
    }
public:
    nav_msgs::OccupancyGrid occupancyGrid;
private:
    float _x_meters;
    float _y_meters;
    float _resolution;
    int _width;
    int _height;
    vector<vector<uint16_t>> _indicator;
    float max_c=0;
};

}

#endif //SRC_GRID_MAP_HPP
