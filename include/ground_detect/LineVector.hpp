//
// Created by vickylzy on 19-10-30.
//

#ifndef SRC_LINEVECTOR_HPP
#define SRC_LINEVECTOR_HPP


#include "std_msgs/String.h"
// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
//eigen
#include <Eigen/Dense>

//cpp
#include <ctime>
#include <math.h>
#include <map>
#include "boost/assign.hpp"
#include <ground_detect/LineVector.hpp>
//#include <mutex>
//#include <boost/circular_buffer.hpp>

namespace ground_exract{

    using namespace std;
    using namespace Eigen;

    class LineVector{
    public:
        LineVector(int scanId, int lineDegree) : scanID(scanId), line_degree(lineDegree) {}

        virtual ~LineVector() {}

        bool push_point(Vector3f direction ,int index){
//            float dot = oritention.cwiseProduct(direction).sum();
//            float lenSq1 = direction.array().square().sum();
//            float lenSq2 = oritention.array().square().sum();
//            float angle = acos(dot/sqrt(lenSq1 * lenSq2));
            float angle = acos( oritention.cwiseProduct(direction).sum()/sqrt(direction.array().square().sum() * oritention.array().square().sum()));
            if (angle< line_degree*M_PI/180){
                indicator.push_back(index);
                oritention+=direction;
                return true;
            } else
                return false;

        }

    public:
        int scanID;
        vector<int> indicator;
        Vector3f oritention;
        int line_degree;
    };

}

#endif //SRC_LINEVECTOR_HPP
