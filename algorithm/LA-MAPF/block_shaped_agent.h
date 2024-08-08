//
// Created by yaozhuo on 2024/6/8.
//

#ifndef LAYEREDMAPF_BLOCK_SHAPED_AGENT_H
#define LAYEREDMAPF_BLOCK_SHAPED_AGENT_H
#include "common.h"
#include "../../freeNav-base/visualization/canvas/canvas.h"
#include <sstream>
#include <boost/algorithm/string/split.hpp>

namespace freeNav::LayeredMAPF::LA_MAPF {


    template<Dimension N>
    std::pair<Pointis<N>, Pointis<N> > getBlockCoverage(const Pointf<N>& min_pt, const Pointf<N>& max_pt) {
        Pointis<N> retv_full, retv_part;
        Pointi<N> min_pti, max_pti;
        DimensionLength dim[N];
        for(int i=0; i<N; i++) {
            min_pti[i] = round(min_pt[i]);
            max_pti[i] = round(max_pt[i]);
            dim[i]     = max_pti[i] - min_pti[i] + 1;
        }
        Id total_index = getTotalIndexOfSpace<N>(dim);
        Pointi<N> temp_pt;
        bool full_occ = true;
        for(int id=0; id<total_index; id++) {
            temp_pt = min_pti + IdToPointi<N>(id, dim);
            full_occ = true;
            for(int i=0; i<N; i++) {
                if(temp_pt[i] < min_pt[i] + 0.5 || temp_pt[i] > max_pt[i] - 0.5) {
                    full_occ = false;
                    break;
                }
            }
            if(full_occ) {
                retv_full.push_back(temp_pt);
            } else {
//                retv_part.push_back(temp_pt);
                retv_full.push_back(temp_pt); // current considering all grid as full occupied, for simplification
            }
        }
        return {retv_full, retv_part};
    }

    template<Dimension N>
    float getInnerRadiusOfBlock(const Pointf<N>& min, const Pointf<N>& max) {
        float min_dim = MAX<float>;
        for(int i=0; i<N; i++) {
            min_dim = std::min(min_dim, fabs(max[i]));
            min_dim = std::min(min_dim, fabs(min[i]));
        }
        return min_dim;
    }

    template<Dimension N>
    float getOuterRadiusOfBlock(const Pointf<N>& min, const Pointf<N>& max) {
        return std::max(min.Norm(), max.Norm());
    }

    struct BlockAgent_2D : public Agent<2> {

    public:

        BlockAgent_2D(const Pointf<2>& min_pt, const Pointf<2>& max_pt, int id, DimensionLength* dim)
            :Agent<2>(getOuterRadiusOfBlock<2>(min_pt, max_pt), getInnerRadiusOfBlock<2>(min_pt, max_pt), id, dim),
            min_pt_(min_pt), max_pt_(max_pt),
            grids_(getAllOrientCoverage_2D(getBlockCoverage<2>(min_pt, max_pt))) {
            // require the motion center of block are within the block
            for(int dim=0;dim<2; dim++) {
                assert(min_pt_[dim] < 0);
                assert(max_pt_[dim] > 0);
            }
            assert(min_pt_[1] + max_pt_[1] == 0);
            initRotateCoverage();
        }

        // pre-calculation coverage in all direction
        std::vector<std::pair<Pointis<2>, Pointis<2> > > static getAllOrientCoverage_2D(const std::pair<Pointis<2>, Pointis<2> >& pts_pair) {
            std::vector<std::pair<Pointis<2>, Pointis<2> > > retv = {pts_pair};
            Pointi<2> new_pt;
            for(int orient=1; orient<4; orient++) {
                Pointis<2> buff_full, buff_part;
                for(const auto& pt : pts_pair.first) {
                    new_pt = rotatePoint<int, 2>(pt,  ROTATE_MATRIXS[2][orient]);
                    buff_full.push_back(new_pt);
                }
                for(const auto& pt : pts_pair.second) {
                    new_pt = rotatePoint<int, 2>(pt,  ROTATE_MATRIXS[2][orient]);
                    buff_part.push_back(new_pt);
                }
                buff_full.shrink_to_fit();
                buff_part.shrink_to_fit();
                retv.push_back({buff_full, buff_part});
            }
            return retv;
        }

        bool isCollide(const Pose<int, 2>& pose,
                       DimensionLength* dim,
                       const IS_OCCUPIED_FUNC<2>& isoc,
                       const DistanceMapUpdater<2>& distance_table) const {
            if(isoc(pose.pt_)) { return true; }
            // if the block is in a grid
            if(this->excircle_radius_ < 0.5) { return false; }
            Id pt_id = PointiToId(pose.pt_, dim);
            // if inner circle contain obstacle, is collide
            if(this->incircle_radius_ >= .5 && distance_table.getClosestDistance(pt_id) <= this->incircle_radius_) {
                return true;
            }
            // if outer circle contain no obstacle, is not collide
            if(distance_table.getClosestDistance(pt_id) > this->excircle_radius_) {
                return false;
            }
            // do grid by grid state check
            const std::pair<Pointis<2>, Pointis<2> > grids_in_range = getPoseOccupiedGrid(pose);
            for(const auto& grid : grids_in_range.first) {
                if(isoc(grid)) {
//                    std::cout << " pose.pt_ = " << pose.pt_ << std::endl;
//                    std::cout << " grid = " << grid << std::endl;
                    return true;
                }
            }
            for(const auto& grid : grids_in_range.second) {
                if(isoc(grid)) {
//                    std::cout << " pose.pt_ = " << pose.pt_ << std::endl;
//                    std::cout << " grid = " << grid << std::endl;
                    return true;
                }
            }
            return false;
        }

        bool isCollide(const Pose<int, 2>& edge_from,
                       const Pose<int, 2>& edge_to,
                       DimensionLength* dim,
                       const IS_OCCUPIED_FUNC<2>& isoc,
                       const DistanceMapUpdater<2>& distance_table) const {
            const Pointi<2> pt1 = edge_from.pt_, pt2 = edge_to.pt_;
            if(isoc(pt1) || isoc(pt2)) { return true; }
            if(pt1 == pt2) {
                // the angle of orientation change must be equal to 90 degree
                if(edge_from.orient_ / 2 == edge_to.orient_ / 2) {
                    return true;
                } else {
                    // rotate collision check
                    if(edge_from.orient_ != edge_to.orient_ && isRotateCollideWithMap(pt1, edge_from.orient_, edge_to.orient_, isoc)) { return true; }
                    else {
                        return false;
                    }
                }
            }
            if(edge_to.orient_ != edge_from.orient_) { return true; }; // if not the same position, can not change in orientation
            assert((pt1 - pt2).Norm() <= 1);
            // can only move in current orient
            Pointi<2> mov_vector = pt2 - pt1;
            // must move in current orientation
            if(mov_vector[edge_to.orient_/2] == 0) { return true; }
            if(edge_to.orient_ % 2 == 0) {
                // move forward
                if(mov_vector[edge_to.orient_/2] != 1) { return true;}
            } else {
                // move backward
                if(mov_vector[edge_to.orient_/2] != -1) { return true;}
            }
            return false;
        }

        Pointis<2> getTransferOccupiedGrid(const Pose<int, 2>& edge_from,
                                           const Pose<int, 2>& edge_to) const override {
            std::pair<Pointis<2>, Pointis<2>> pts1 = getPoseOccupiedGrid(edge_from),
                                              pts2 = getPoseOccupiedGrid(edge_to);
            pts1.first.insert(pts1.first.end(), pts2.first.begin(), pts2.first.end());
            if(edge_from.orient_ == edge_to.orient_) {
                return pts1.first;
            } else {
                std::pair<Pointis<2>, Pointis<2>> pts3 = getPosedRotateCoverage(edge_from.pt_, edge_from.orient_, edge_to.orient_);
                pts1.first.insert(pts1.first.end(), pts3.first.begin(), pts3.first.end());
                return pts1.first;
            }
        }

        std::pair<Pointis<2>, Pointis<2>> getPoseOccupiedGrid(const Pose<int, 2>& pose) const override {
            // determine rotate matrix
            std::vector<std::vector<int> > rotate_matrix;
            // rotate grids
            Pointis<2> retv_full = {}, retv_part = {};
            for(const auto& grid : grids_[pose.orient_].first) {
                retv_full.push_back(grid + pose.pt_);
            }
            for(const auto& grid : grids_[pose.orient_].second) {
                retv_part.push_back(grid + pose.pt_);
            }
            return {retv_full, retv_part};
        }

        // there are four types of rotate coverage, rotate the default one to get other coverage
        std::pair<Pointis<2>, Pointis<2> > getRotateCoverage(const int& orient_start, const int& orient_end) const {
            if(orient_start == orient_end) { return {{}, {}}; }
            int orient_1 = std::min(orient_start, orient_end);
            int orient_2 = std::max(orient_start, orient_end);
            //std::cout << "orient_start, orient_end " << orient_start << ", " << orient_end << std::endl;
            assert(orient_1 == 0 || orient_1 == 1);
            assert(orient_2 == 2 || orient_2 == 3);
            bool x_reverse = false, y_reverse = false;
            if(orient_1 == 1) { x_reverse = true; }
            if(orient_2 == 3) { y_reverse = true; }
            int flag = ( x_reverse ? 1 : 0) + (y_reverse ? 2 : 0);
            return rotate_pts_[flag];
        }

        std::pair<Pointis<2>, Pointis<2> > getPosedRotateCoverage(const Pointi<2>& offset, const int& orient_start, const int& orient_end) const {
            auto rotate_pair = getRotateCoverage(orient_start, orient_end);
            for(auto& pt : rotate_pair.first) {
                pt = pt + offset;
            }
            for(auto& pt : rotate_pair.second) {
                pt = pt + offset;
            }
            return rotate_pair;
        }

        // check whether rotate is collide
        bool isRotateCollideWithMap(const Pointi<2>& center_pt,
                                    const int& orient_start,
                                    const int& orient_end,
                                    const IS_OCCUPIED_FUNC<2>& isoc) const {
            assert(orient_start / 2 != orient_end / 2);
            // get rotate coverage
            const auto& rotate_f_b = getRotateCoverage(orient_start, orient_end);
            // check them state one by one
            for(const auto& pt : rotate_f_b.first) {
                if(isoc(center_pt + pt)) { return true; }
            }
            for(const auto& pt : rotate_f_b.second) {
                if(isoc(center_pt + pt)) { return true; }
            }
            return false;
        }

        void drawOnCanvas(const Pose<int, 2>& pose,
                          Canvas& canvas, const cv::Vec3b& color, bool fill=true) const {
            const auto& rect = getPosedRectangle(pose); // agents
            canvas.drawRectangleFloat(rect.first, rect.second, true, fill ? -1 : 1, color);
        }

        // get half of the coverage (when x>= 0)
        std::vector<std::pair<Pointis<2>, Pointis<2> > > getInitRotateCoverage(Pointf<2> pt, bool reverse = false) const {
            if(reverse) { pt = Pointf<2>() - pt; }
            assert(pt[0] > 0 && pt[1] > 0);
            double radius = pt.Norm(), temp_dist, max_dist = 0, min_dist = MAX<double>;
            Pointi<2> pt_expand;
            pt_expand[0] = ceil(pt[0]);
            pt_expand[1] = ceil(pt[1]);

            int arc = ceil(pt_expand.Norm());
            int min_x = -round(pt[1]), max_x = arc;
            int min_y = -round(pt[1]), max_y = arc;
            double max_2 = pt_expand[0]*pt_expand[0] + pt_expand[1]*pt_expand[1];

            std::vector<std::pair<Pointis<2>, Pointis<2> > > retv(4);
            Pointi<2> local_pt;
            int dist_2;

            int max_width = std::max(pt[0], pt[1]);
            bool full_coverage = true;
            Pointfs<2> corners = getAllCornerOfGrid<2>();

            Pointi<2> max_pti, min_pti;
            max_pti[0] = floor(max_pt_[0]);
            max_pti[1] = floor(max_pt_[1]);

            min_pti[0] = ceil(min_pt_[0]);
            min_pti[1] = ceil(min_pt_[1]);

            for(int x=min_x; x<= max_x; x++) {
                for(int y=min_y; y<= max_y; y++) {
                    if(x < 0 && y < 0) { continue; }
                    // cut overlap with itself coverage check
                    //if (x >= 0 && x <= pt[0] && y >= -pt[1] && y <= pt[1]) { continue; }
                    //if (y >= 0 && y <= pt[0] && x >= -pt[1] && x <= pt[1]) { continue; }

                    if(pt[0] <= pt[1]) {
                        // ignore diagonal center
                        if(x > 0 && y > 0 && x <= max_width && y <= max_width) { continue; }
                    }
                    dist_2 = x*x + y*y;

                    if(dist_2 > max_2) { continue; }

                    local_pt[0] = x, local_pt[1] = y;
                    max_dist = 0, min_dist = MAX<float>;
                    full_coverage = true;
                    for(const auto& offset : corners) {
                        temp_dist = sqrt(pow((offset[0] + local_pt[0]), 2) + pow((offset[1] + local_pt[1]), 2));
                        if(temp_dist > max_dist) { max_dist = temp_dist; }
                        if(temp_dist < min_dist) { min_dist = temp_dist; }
                    }

                    if(reverse) {
                        local_pt = Pointi<2>() - local_pt;
                    }

                    // cut overlap with the whole block's coverage check
//                    if (local_pt[0] >= min_pt_[0] && local_pt[0] <= max_pt_[0] &&
//                        local_pt[1] >= -max_pt_[1] && local_pt[1] <= max_pt_[1]) {
//                        continue;
//                    }
//                    if (local_pt[1] >= min_pt_[0] && local_pt[1] <= max_pt_[0] &&
//                        local_pt[0] >= -max_pt_[1] && local_pt[0] <= max_pt_[1]) {
//                        continue;
//                    }

                    if (local_pt[0] > min_pti[0]  && local_pt[0] < max_pti[0] &&
                        local_pt[1] > -max_pti[1] && local_pt[1] < max_pti[1]) {
                        continue;
                    }
                    if (local_pt[1] > min_pti[0]  && local_pt[1] < max_pti[0] &&
                        local_pt[0] > -max_pti[1] && local_pt[0] < max_pti[1]) {
                        continue;
                    }

                    if(local_pt[0] == max_pti[1] && local_pt[1] < 0) {
                        continue;
                    }
                    if(local_pt[0] == -max_pti[1] && local_pt[1] > 0) {
                        continue;
                    }

                    if(local_pt[1] == max_pti[1] && local_pt[0] < 0) {
                        continue;
                    }
                    if(local_pt[1] == -max_pti[1] && local_pt[0] > 0) {
                        continue;
                    }

                    if(min_dist > radius) {
                        if(local_pt == Pointi<2>()) {
                            // current agent may small than a grid, considering as part occupied
                            retv[0].second.push_back({ local_pt[0],  local_pt[1]});
                            retv[1].second.push_back({-local_pt[0],  local_pt[1]});
                            retv[2].second.push_back({ local_pt[0], -local_pt[1]});
                            retv[3].second.push_back({-local_pt[0], -local_pt[1]});
                        }
                        continue;
                    }
                    if(max_dist < radius) {
                        full_coverage = true;
                    } else {
                        full_coverage = false;
                    }

                    if(full_coverage) {
                        retv[0].first.push_back({ local_pt[0],  local_pt[1]});
                        retv[1].first.push_back({-local_pt[0],  local_pt[1]});
                        retv[2].first.push_back({ local_pt[0], -local_pt[1]});
                        retv[3].first.push_back({-local_pt[0], -local_pt[1]});
                    } else {
                        // current considering all grid as full occupied, for simplification
                        retv[0].first. push_back({ local_pt[0],  local_pt[1]});
                        retv[1].first. push_back({-local_pt[0],  local_pt[1]});
                        retv[2].first. push_back({ local_pt[0], -local_pt[1]});
                        retv[3].first. push_back({-local_pt[0], -local_pt[1]});

//                        retv[0].second.push_back({ local_pt[0],  local_pt[1]});
//                        retv[1].second.push_back({-local_pt[0],  local_pt[1]});
//                        retv[2].second.push_back({ local_pt[0], -local_pt[1]});
//                        retv[3].second.push_back({-local_pt[0], -local_pt[1]});
                    }
                }
            }
            return retv;
        }

        void initRotateCoverage() {
            // front rotate
            auto front_pair = getInitRotateCoverage(max_pt_, false);
            // backward rotate
            auto back_pair  = getInitRotateCoverage(min_pt_, true);
            for(int i=0; i<4; i++) {
                front_pair[i].first. insert(front_pair[i].first.end(),  back_pair[i].first.begin(),  back_pair[i].first.end());
                front_pair[i].second.insert(front_pair[i].second.end(), back_pair[i].second.begin(), back_pair[i].second.end());
            }
            rotate_pts_ = front_pair;
        }

        std::pair<Pointf<2>, Pointf<2> > getPosedRectangle(const Pose<int, 2>& pose) const {
            Pointf<2> new_min = pointTransfer_2D<float>(min_pt_, pose);
            Pointf<2> new_max = pointTransfer_2D<float>(max_pt_, pose);
            //std::cout << "old_min, old_max = " << min_pt_ << " " << max_pt_ << std::endl;
            //std::cout << "new_min, new_max = " << new_min << " " << new_max << std::endl;
            return {{std::min(new_min[0], new_max[0]), std::min(new_min[1], new_max[1])},
                    {std::max(new_min[0], new_max[0]), std::max(new_min[1], new_max[1])}};
        }

        Pointfs<2> getRotatedFans(const int& orient_start, const int& orient_end) const {
            Pointf<2> p1 = min_pt_, p2 = min_pt_, p3 = min_pt_, p4 = max_pt_;
            p2[0] = max_pt_[0], p3[1] = max_pt_[1];

            Pointfs<2> all_fan = {p1, p2, p3, p4};
            Pointfs<2> retv;
            for(const auto& pt : all_fan) {
                retv.push_back(pointRotate_2D(pt, orient_start));
                retv.push_back(pointRotate_2D(pt, orient_end));
            }
            return retv;
        }

        std::string serialize(const Pose<int, 2>& start_pose, const Pose<int, 2>& target_pose) const {
            std::stringstream ss;
            ss << id_ << " "
               << min_pt_[0] << " " << min_pt_[1] << " "
               << max_pt_[0] << " " << max_pt_[1] << " "
               << start_pose.pt_[0]  << " " << start_pose.pt_[1]  << " " << start_pose.orient_ << " "
               << target_pose.pt_[0] << " " << target_pose.pt_[1] << " " << target_pose.orient_;
            return ss.str();
        }

        static std::pair<BlockAgent_2D, InstanceOrient<2> > deserialize(const std::string& string, DimensionLength* dim) {
            std::vector<std::string> strs;
            boost::split(strs, string, boost::is_any_of(" "), boost::token_compress_on);
            assert(strs.size() == 11);
            int id = atoi(strs[0].c_str());
            Pointf<2> min_pt, max_pt;
            min_pt[0] = atof(strs[1].c_str());
            min_pt[1] = atof(strs[2].c_str());

            max_pt[0] = atof(strs[3].c_str());
            max_pt[1] = atof(strs[4].c_str());

            BlockAgent_2D agent(min_pt, max_pt, id, dim);

            Pose<int, 2> start_pose ({atoi(strs[5].c_str()), atoi(strs[6].c_str())}, atoi(strs[7].c_str()));
            Pose<int, 2> target_pose({atoi(strs[8].c_str()), atoi(strs[9].c_str())}, atoi(strs[10].c_str()));

            return {agent, {start_pose, target_pose}};
        }



//        Pointfs<2> getPoseRotatedFans(const Pointi<2>& origin, const int& orient_start, const int& orient_end) const {
//            Pointfs<2> pts = getRotatedFans(orient_start, orient_end), retv;
//            Pointf<2> origin_f;
//            origin_f[0] = origin[0];
//            origin_f[0] = origin[1];
//            for(int i=0; i<pts.size(); i++) {
//                const auto& pt = pts[i];
//                if(i%2 == 0) {
//                    retv.push_back(origin_f);
//                }
//                retv.push_back(origin_f + pt);
//            }
//            return retv;
//        }

        Pointf<2> min_pt_;
        Pointf<2> max_pt_;

        // which grid current agent cover if it is in zero position, zero orient
        //Pointis<2> grids_;

        std::vector<std::pair<Pointis<2>, Pointis<2> > >  grids_;

        // grid covered when rotate, assume is 0 rotate to 1
//        std::vector<std::pair<Pointis<2>, Pointis<2> > > backward_rotate_pts_;
//        std::vector<std::pair<Pointis<2>, Pointis<2> > > front_rotate_pts_;

        std::vector<std::pair<Pointis<2>, Pointis<2> > > rotate_pts_;

    };

    // all fan is default to 90 degree
//    template <typename T>
//    bool isCollideBetweenTwoFan(const Point<T, 2>& c1, const Point<T, 2>& p1, const Point<T, 2>& p2,
//                                const Point<T, 2>& c2, const Point<T, 2>& p3, const Point<T, 2>& p4) {
//        double r1 = (p1 - c1).Norm(), r2 = (p3 - c2).Norm();
//        if(r1 + r2 > (c1 - c2).Norm()) { return true; }
//        // check line c1,c2 and its angle between line c1,p1 c1,p2, and between line c2,p3 c2,p4
//        if(isPointInFan(c1, p1, p2, c2)) { return true; }
//        if(isPointInFan(c2, p3, p4, c2)) { return true; }
//        return false;
//    }

    // the fan is default to 90 degree
//    template <typename T>
//    bool isPointInFan(const Point<T, 2>& c1, const Point<T, 2>& p1, const Point<T, 2>& p2,
//                      const Point<T, 2>& pt) {
//        Point<T, 2> c1_pt = pt - c1, c1_p1 = p1 - c1, c1_p2 = p2 - c1;
//        if(c1_pt*c1_p1 < 0 || c1_pt*c1_p2 < 0) { return true; }
//        return false;
//    }

    std::ostream& operator << (std::ostream& os, const BlockAgent_2D& block);

    // check whether two moving circle are collide with each other
    bool isCollide(const BlockAgent_2D& a1, const Pose<int, 2>& s1, const Pose<int, 2>& e1,
                   const BlockAgent_2D& a2, const Pose<int, 2>& s2, const Pose<int, 2>& e2);

    // check whether one moving circle are collide with one waiting circle
    bool isCollide(const BlockAgent_2D& a1, const Pose<int, 2>& s1, const Pose<int, 2>& e1,
                   const BlockAgent_2D& a2, const Pose<int, 2>& s2);

    // check whether one moving circle are collide with one waiting circle
    bool isCollide(const BlockAgent_2D& a1, const Pose<int, 2>& s1,
                   const BlockAgent_2D& a2, const Pose<int, 2>& s2, const Pose<int, 2>& e2);

    bool isCollide(const BlockAgent_2D& a1, const Pose<int, 2>& s1,
                   const BlockAgent_2D& a2, const Pose<int, 2>& s2);


    typedef std::vector<BlockAgent_2D> BlockAgents_2D;


    std::string serialize(const BlockAgent_2D& block, const Pose<int, 2>& pose);

    std::pair<BlockAgent_2D, Pose<int, 2> > deserializeBlockAgent_2D(const std::string& string, DimensionLength* dim);

}

#endif //LAYEREDMAPF_BLOCK_SHAPED_AGENT_H
