//
// Created by yaozhuo on 2024/6/8.
//

#ifndef LAYEREDMAPF_BLOCK_SHAPED_AGENT_H
#define LAYEREDMAPF_BLOCK_SHAPED_AGENT_H
#include "common.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    // TODO: ignore grid inside incircle radius, to save storage space
    template<Dimension N>
    Pointis<N> getBlockCoverage(const Pointf<N>& min_pt, const Pointf<N>& max_pt) {
        Pointis<N> retv;
        Pointi<N> min_pti, max_pti;
        DimensionLength dim[N];
        for(int i=0; i<N; i++) {
            min_pti[i] = floor(min_pt[i]);
            max_pti[i] = ceil(max_pt[i]);
            dim[i]     = max_pti[i] - min_pti[i];
        }
        Id total_index = getTotalIndexOfSpace<N>(dim);
        Pointi<N> temp_pt;
        for(int id=0; id<total_index; id++) {
            temp_pt = min_pti + IdToPointi<N>(id, dim);
            retv.push_back(temp_pt);
        }
        return retv;
    }

    template<Dimension N>
    float getInnerRadiusOfBlock(const Pointf<N>& min, const Pointf<N>& max) {
        Pointf<N> dim = max - min;
        float min_dim = MAX<float>;
        for(int i=0; i<N; i++) {
            min_dim = std::min(min_dim, dim[i]);
        }
        return min_dim/N;
    }

    template<Dimension N>
    float getOuterRadiusOfBlock(const Pointf<N>& min, const Pointf<N>& max) {
        Pointf<2> dim = max - min;
        float square = 0;
        for(int i=0; i<N; i++) {
            square += dim[i]*dim[i];
        }
        return sqrt(square);
    }

    struct BlockAgent_2D : public Agent<2> {

    public:

        BlockAgent_2D(const Pointf<2>& min_pt, const Pointf<2>& max_pt, int id)
            :Agent<2>(getOuterRadiusOfBlock<2>(min_pt, max_pt), getInnerRadiusOfBlock<2>(min_pt, max_pt), id)
            , min_pt_(min_pt), max_pt_(max_pt), dim_ (max_pt - min_pt), grids_(getBlockCoverage<2>(min_pt, max_pt)) {
            // require the motion center of block are within the block
            for(int dim=0;dim<2; dim++) {
                assert(min_pt_[dim] < 0);
                assert(max_pt_[dim] > 0);
            }
            assert(min_pt_[1] + max_pt_[1] == 0);
            initRotateCoverage();
        }

        Pointis<2> getGridWithinPosedBlock(const Pointf<2>& min_pt, const Pointf<2>& max_pt, const Pose<2>& pose) const {
            // determine rotate matrix
            std::vector<std::vector<int> > rotate_matrix;
            // rotate grids
            Pointis<2> retv;
            for(const auto& grid : grids_) {
                retv.push_back(rotatePoint<int, 2>(grid + pose.pt_,  ROTATE_MATRIXS[2][pose.orient_]));
            }
            return retv;
        }

        bool isCollide(const Pose<2>& pose,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<2>& isoc,
                               const DistanceMapUpdater<2>& distance_table) const {
            if(isoc(pose.pt_)) { return true; }
            // if the block is in a grid
            if(this->excircle_radius_ < 0.5) { return false; }
            Id pt_id = PointiToId(pose.pt_, dim);
            // if inner circle contain obstacle, is collide
            if(distance_table.getClosestDistance(pt_id) <= this->incircle_radius_) {
                return true;
            }
            // if outer circle contain no obstacle, is not collide
            if(distance_table.getClosestDistance(pt_id) > this->excircle_radius_) {
                return false;
            }
            // do grid by grid state check
            const Pointis<2> grids_in_range = getGridWithinPosedBlock(min_pt_, max_pt_, pose);
            for(const auto& grid : grids_in_range) {
                if(isoc(grid)) { return true; }
            }
            return false;
        }

        bool isCollide(const Pose<2>& edge_from,
                      const Pose<2>& edge_to,
                      DimensionLength* dim,
                      const IS_OCCUPIED_FUNC<2>& isoc,
                      const DistanceMapUpdater<2>& distance_table) const {
            // TODO：
            return false;
        }

        // assume pt that out of range are occupied
//        static bool rotateCheckStatic(const Pointf<2>& min_pt, const Pointf<2>& max_pt,
//                                      const Pose<2>& pose,
//                                      const float& incircle_radius,
//                                      const float& excircle_radius,
//                                      const int& start_orient, const int& end_orient,
//                                      DimensionLength* dim,
//                                      const IS_OCCUPIED_FUNC<2>& isoc,
//                                      const DistanceMapUpdater<2>& distance_table) {
//            assert(start_orient != end_orient); // must change in direction
//            assert(start_orient/2 != end_orient/2); // must not 180 degree change
//
//            int half_range = ceil(excircle_radius);
//            float incircle_radius_2 = pow(incircle_radius, 2);
//            float excircle_radius_2 = pow(excircle_radius, 2);
//            Pointi<2> local_pt, global_pt;
//            int dist_2;
//            for(int x=-half_range; x<=half_range; x++) {
//                for(int y=-half_range; y<=half_range; y++) {
//                    local_pt[0] = x, local_pt[1] = y;
//                    dist_2 = x*x + y*y;
//                    // no need to check inside incircle or out of excircle
//                    if(dist_2 < incircle_radius_2) { continue; }
//                    if(dist_2 > excircle_radius_2) { continue; }
//
//                    global_pt = pointTransfer_2D(local_pt, pose);
//                    if(isoc(global_pt)) {
//                        // check whether in rotate range
//                    }
//                }
//            }
//            return false;
//        }

        // get half of the coverage (when x>= 0)
        Pointis<2> getRotateCoverage(Pointf<2> pt, bool reverse = false) const {
            if(reverse) { pt = Pointf<2>() - pt; }
            assert(pt[0] > 0 && pt[1] > 0);
            int arc = ceil(pt.Norm());
            int min_x = -ceil(pt[1]), max_x = arc;
            int min_y = -ceil(pt[1]), max_y = arc;
            int max_2 = pt[0]*pt[0] + pt[1]*pt[1];

            Pointis<2> retv;
            Pointi<2> local_pt;
            int dist_2;

            int max_width = std::max(pt[0], pt[1]);

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
                    if(reverse) {
                        local_pt = Pointi<2>()-local_pt;
                    }

                    // cut overlap with the whole block's coverage check
                    if (local_pt[0] >= min_pt_[0] && local_pt[0] <= max_pt_[0] && local_pt[1] >= -max_pt_[1] && local_pt[1] <= max_pt_[1]) { continue; }
                    if (local_pt[1] >= min_pt_[0] && local_pt[1] <= max_pt_[0] && local_pt[0] >= -max_pt_[1] && local_pt[0] <= max_pt_[1]) { continue; }

                    retv.push_back(local_pt);
                }
            }
            return retv;
        }

        void initRotateCoverage() {
            float incircle_radius_2 = pow(incircle_radius_, 2);
            float excircle_radius_2 = pow(excircle_radius_, 2);

            std::cout << " incircle_radius   " << incircle_radius_ << std::endl;
            std::cout << " excircle_radius   " << excircle_radius_ << std::endl;

            std::cout << " incircle_radius_2 " << incircle_radius_2 << std::endl;
            std::cout << " excircle_radius_2 " << excircle_radius_2 << std::endl;

            // front rotate
            front_rotate_pts    = getRotateCoverage(max_pt_, false);
            // backward rotate
            backward_rotate_pts = getRotateCoverage(min_pt_, true);
        }

        const Pointf<2>& min_pt_, max_pt_, dim_;

        // which grid current agent cover if it is in zero position, zero orient
        Pointis<2> grids_;

        // grid covered when rotate, assume is 0 rotate to 1
        Pointis<2> backward_rotate_pts;
        Pointis<2> front_rotate_pts;

    };

}

#endif //LAYEREDMAPF_BLOCK_SHAPED_AGENT_H
