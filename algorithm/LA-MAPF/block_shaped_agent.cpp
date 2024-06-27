//
// Created by yaozhuo on 2024/6/9.
//

#include "block_shaped_agent.h"

namespace freeNav::LayeredMAPF::LA_MAPF {


    // check whether two moving circle are collide with each other
    bool isCollide(const BlockAgent_2D& a1, const Pose<int, 2>& s1, const Pose<int, 2>& e1,
                   const BlockAgent_2D& a2, const Pose<int, 2>& s2, const Pose<int, 2>& e2) {
        // use inner circle and out circle for accelerate
        std::vector<double> dists = {(s1.pt_ - s2.pt_).Norm(), (s1.pt_ - e2.pt_).Norm(),
                                     (e1.pt_ - s2.pt_).Norm(), (e1.pt_ - e2.pt_).Norm()};

        std::sort(dists.begin(), dists.end(), [&](const double& v1, const double& v2) {
            return v1 < v2;
        });

        if(dists.front() < a1.incircle_radius_ + a2.incircle_radius_) {
            return true;
        }

        if(dists.back() > a1.excircle_radius_ + a2.excircle_radius_) {
            return false;
        }

        // rectangle overlap check
        auto rects1 = a1.getPosedRectangle(s1);
        auto recte1 = a1.getPosedRectangle(e1);
        auto rects2 = a2.getPosedRectangle(s2);
        auto recte2 = a2.getPosedRectangle(e2);

        if(isRectangleOverlap(rects1.first, rects1.second, rects2.first, rects2.second)) {
            return true;
        }
        if(isRectangleOverlap(rects1.first, rects1.second, recte2.first, recte2.second)) {
            return true;
        }
        if(isRectangleOverlap(recte1.first, recte1.second, rects2.first, rects2.second)) {
            return true;
        }
        if(isRectangleOverlap(recte1.first, recte1.second, recte2.first, recte2.second)) {
            return true;
        }
        // rotate overlap check
        // TODO: do collide check between geometry elements rather than grid by grid check
        //     e.g.,
        bool a1_rotate = (s1.orient_ != e1.orient_);
        bool a2_rotate = (s2.orient_ != e2.orient_);
        // both no rotate
        if(!a1_rotate && !a2_rotate) { return false; }
        // if a1 rotate, check a1 rotate with other two rectangle
        const auto& a1_coverage_pair = a1.getPosedRotateCoverage(s1.pt_, s1.orient_, e1.orient_);
        if(a1_rotate) {
            for(const auto& pt : a1_coverage_pair.first) {
                if(isPointInRectangle<float, int, 2>(rects2.first, rects2.second, pt)) { return true; }
            }
            for(const auto& pt : a1_coverage_pair.second) {
                if(isPointInRectangle<float, int, 2>(rects2.first, rects2.second, pt)) { return true; }
            }
            for(const auto& pt : a1_coverage_pair.first) {
                if(isPointInRectangle<float, int, 2>(recte2.first, recte2.second, pt)) { return true; }
            }
            for(const auto& pt : a1_coverage_pair.second) {
                if(isPointInRectangle<float, int, 2>(recte2.first, recte2.second, pt)) { return true; }
            }
        }
        // if a2 rotate, check a2 rotate with other two rectangle
        const auto& a2_coverage_pair = a2.getPosedRotateCoverage(s2.pt_, s2.orient_, e2.orient_);
        if(a2_rotate) {
            for(const auto& pt : a2_coverage_pair.first) {
                if(isPointInRectangle<float, int, 2>(rects1.first, rects1.second, pt)) { return true; }
            }
            for(const auto& pt : a2_coverage_pair.second) {
                if(isPointInRectangle<float, int, 2>(rects1.first, rects1.second, pt)) { return true; }
            }
            for(const auto& pt : a2_coverage_pair.first) {
                if(isPointInRectangle<float, int, 2>(recte1.first, recte1.second, pt)) { return true; }
            }
            for(const auto& pt : a2_coverage_pair.second) {
                if(isPointInRectangle<float, int, 2>(recte1.first, recte1.second, pt)) { return true; }
            }
        }
        // if both rotate, check whether two fan overlap
        // a1.dim_ should be equal to a2.dim_
        if(a1_rotate && a2_rotate) {
            if(isPointSetOverlap(a1_coverage_pair.first,  a2_coverage_pair.first,  a1.dim_)) { return true; }
            if(isPointSetOverlap(a1_coverage_pair.first,  a2_coverage_pair.second, a1.dim_)) { return true; }
            if(isPointSetOverlap(a1_coverage_pair.second, a2_coverage_pair.first,  a1.dim_)) { return true; }
            if(isPointSetOverlap(a1_coverage_pair.second, a2_coverage_pair.second, a1.dim_)) { return true; }
        }
        return false;
    }

    // check whether one moving circle are collide with one waiting circle
    bool isCollide(const BlockAgent_2D& a1, const Pose<int, 2>& s1, const Pose<int, 2>& e1,
                   const BlockAgent_2D& a2, const Pose<int, 2>& s2) {
        // use inner circle and out circle for accelerate
        std::vector<double> dists = {(s1.pt_ - s2.pt_).Norm(), (s1.pt_ - s2.pt_).Norm()};

        std::sort(dists.begin(), dists.end(), [&](const double& v1, const double& v2) {
            return v1 < v2;
        });

        if(dists.front() < a1.incircle_radius_ + a2.incircle_radius_) {
            return true;
        }

        if(dists.back() > a1.excircle_radius_ + a2.excircle_radius_) {
            return false;
        }

        // rectangle overlap check
        auto rects1 = a1.getPosedRectangle(s1);
        auto recte1 = a1.getPosedRectangle(e1);
        auto rects2 = a2.getPosedRectangle(s2);
        if(isRectangleOverlap(rects1.first, rects1.second, rects2.first, rects2.second)) {
            return true;
        }
        if(isRectangleOverlap(recte1.first, recte1.second, rects2.first, rects2.second)) {
            return true;
        }
        // rotate in rectangle check
        if(s1.pt_ == e1.pt_) {
            const auto& pt_pair = a1.getPosedRotateCoverage(s1.pt_, s1.orient_, e1.orient_);
            for(const auto& pt : pt_pair.first) {
                if(isPointInRectangle<float, int, 2>(rects2.first, rects2.second, pt)) { return true; }
            }
            for(const auto& pt : pt_pair.second) {
                if(isPointInRectangle<float, int, 2>(rects2.first, rects2.second, pt)) { return true; }
            }
        }
        return false;
    }

    // check whether one moving circle are collide with one waiting circle
    bool isCollide(const BlockAgent_2D& a1, const Pose<int, 2>& s1,
                   const BlockAgent_2D& a2, const Pose<int, 2>& s2, const Pose<int, 2>& e2) {

        return isCollide(a2, s2, e2, a1, s1);
    }

    bool isCollide(const BlockAgent_2D& a1, const Pose<int, 2>& s1,
                   const BlockAgent_2D& a2, const Pose<int, 2>& s2) {
        // rectangle overlap check
        auto rects1 = a1.getPosedRectangle(s1);
        auto rects2 = a2.getPosedRectangle(s2);
        if(isRectangleOverlap(rects1.first, rects1.second, rects2.first, rects2.second)) {
            return true;
        }
        return false;
    }


    void DrawOnCanvas(const BlockAgent_2D& block, const Pose<int, 2>& pose,
                      Canvas& canvas, const cv::Vec3b& color) {
        const auto& rect = block.getPosedRectangle(pose); // agents
        canvas.drawRectangleFloat(rect.first, rect.second, true, -1, color);
    }

}