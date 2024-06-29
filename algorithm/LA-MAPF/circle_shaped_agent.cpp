//
// Created by yaozhuo on 2024/6/27.
//

#include "circle_shaped_agent.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // check whether two moving circle are collide with each other
    bool isCollide(const CircleAgent<2>& a1, const Pose<int, 2>& s1, const Pose<int, 2>& e1,
                   const CircleAgent<2>& a2, const Pose<int, 2>& s2, const Pose<int, 2>& e2) {

        namespace bg = boost::geometry;
        using bg_pt = bg::model::point<int, 2, bg::cs::cartesian>;
        using bg_seg = bg::model::segment<bg_pt>;
        // calculate the shortest distance between two segments
        bg_pt pt1(s1.pt_[0], s1.pt_[1]), pt2(e1.pt_[0], e1.pt_[1]),
                pt3(s2.pt_[0], s2.pt_[1]), pt4(e2.pt_[0], e2.pt_[1]);
        bg_seg seg1(pt1, pt2), seg2(pt3, pt4);
        //std::cout << " bg::distance(seg1, seg2) = " << bg::distance(seg1, seg2) << std::endl;
        return bg::distance(seg1, seg2) <= (a1.radius_ + a2.radius_);
    }

    // check whether one moving circle are collide with one waiting circle
    bool isCollide(const CircleAgent<2>& a1, const Pose<int, 2>& s1, const Pose<int, 2>& e1,
                   const CircleAgent<2>& a2, const Pose<int, 2>& s2) {

        namespace bg = boost::geometry;
        using bg_pt = bg::model::point<int, 2, bg::cs::cartesian>;
        using bg_seg = bg::model::segment<bg_pt>;
        // calculate the shortest distance between a point and a segment
        bg_pt pt1(s1.pt_[0], s1.pt_[1]), pt2(e1.pt_[0], e1.pt_[1]),
                pt3(s2.pt_[0], s2.pt_[1]);
        bg_seg seg1(pt1, pt2);
        //std::cout << " bg::distance(seg1, pt3) = " << bg::distance(seg1, pt3) << std::endl;
        return bg::distance(seg1, pt3) <= (a1.radius_ + a2.radius_);
    }

    // check whether one moving circle are collide with one waiting circle
    bool isCollide(const CircleAgent<2>& a1, const Pose<int, 2>& s1,
                   const CircleAgent<2>& a2, const Pose<int, 2>& s2, const Pose<int, 2>& e2) {

        return isCollide(a2, s2, e2, a1, s1);
    }

    bool isCollide(const CircleAgent<2>& a1, const Pose<int, 2>& s1,
                   const CircleAgent<2>& a2, const Pose<int, 2>& s2) {
        return (a1.radius_ + a2.radius_) >= (s1.pt_ - s2.pt_).Norm();
    }

    void DrawOnCanvas(const CircleAgent<2>& circle, const Pose<int, 2>& pose,
                      Canvas& canvas, const cv::Vec3b& color, bool fill) {
        canvas.drawCircleInt(pose.pt_[0], pose.pt_[1], floor(circle.radius_*canvas.zoom_ratio_),
                             true, fill ? -1 : 1, color);

    }

}
