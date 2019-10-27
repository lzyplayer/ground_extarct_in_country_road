//
// Created by vickylzy on 19-10-26.
//

#ifndef SRC_LINEPART_HPP
#define SRC_LINEPART_HPP


#include <pcl/point_types.h>

using namespace std;

class LinePart{
public:

private:
    int scanID;
//    int start_pos;
//    int end_pos;
    vector<pcl::PointXYZI> points;


public:
    LinePart(int scanId) : scanID(scanId) {}

    virtual ~LinePart() {

    }

    void push_point(const pcl::PointXYZI& p){
        this->points.push_back(p);

    }

public:
    int getScanId() const {
        return scanID;
    }

    void setScanId(int scanId) {
        scanID = scanId;
    }

//    int getStartPos() const {
//        return start_pos;
//    }
//
//    void setStartPos(int startPos) {
//        start_pos = startPos;
//    }
//
//    int getEndPos() const {
//        return end_pos;
//    }
//
//    void setEndPos(int endPos) {
//        end_pos = endPos;
//    }

    int getLineLength() const {
        return this->points.size();
    }



//    const vector<pcl::PointXYZI> &getPoints() const {
//        return points;
//    }
//
//    void setPoints(const vector<pcl::PointXYZI> &points) {
//        LinePart::points = points;
//    }
};

#endif //SRC_LINEPART_HPP
