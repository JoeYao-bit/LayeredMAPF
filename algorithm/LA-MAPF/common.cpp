//
// Created by yaozhuo on 2024/6/1.
//

#include "common.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    MemoryRecorder memory_recorder(1);

    // yz: determine priority of conflicts
    bool operator<(const Conflict &conflict1, const Conflict &conflict2) // return true if conflict2 has higher priority
    {
//        if (conflict1.priority == conflict2.priority) {
//            if (conflict1.type == conflict2.type) {
//                if (conflict1.secondary_priority == conflict2.secondary_priority) {
//                    return rand() % 2;
//                }
//                return conflict1.secondary_priority > conflict2.secondary_priority;
//            }
//            return conflict1.type > conflict2.type;
//        }
//        return conflict1.priority > conflict2.priority;
        if(conflict1.t1 <= conflict2.t1) {
            if(conflict1.t1 == conflict2.t1) {
                return rand()%2 == 0;
            } else {
                return true;
            }
        }
        return false;
    }

    bool isSamePath(const std::vector<size_t>& path1, const std::vector<size_t>& path2) {
        if(path1.size() != path2.size()) { return false; }
        for(int t = 0; t < path1.size(); t++) {
            if(path1[t] != path2[t]) {
                return false;
            }
        }
        return true;
    }

    RotateMatrix initializeRotateMatrix(const int& dim, const int& orient) {
        RotateMatrix retv = RotateMatrix(dim, std::vector<int>(dim, 0));
        // 1, get unit matrix
        for(int i=0; i<dim; i++) {
            retv[dim][dim] = 1;
        }
        // 2，set value
        return retv;
    }

    // for calculate rotate matrix about 2D and 3D maps
    RotateMatrixs initializeRotateMatrixs(const int& dim) {
        assert(dim == 2);
        RotateMatrixs retv;
        std::cout << __FUNCTION__ << "..." << std::endl;
        for(int orient=0; orient<2*dim; orient++) {
            double theta = orientToPi_2D(orient);
//            std::cout << " theta = " << theta << std::endl;
            retv.push_back(initRotateMatrix_2D(theta));
        }
        return retv;
    }

    std::vector<RotateMatrixs> initializeAllRotateMatrixs() {
        std::vector<RotateMatrixs> retv;
        retv.push_back({}); // occupied 0 dimension
        retv.push_back({}); // occupied 1 dimension
//        for(int dim=2; dim<=3; dim++) {
//            retv.push_back(initializeRotateMatrixs(dim));
//        }
        retv.push_back(initializeRotateMatrixs(2));
        return retv;
    }

    std::vector<RotateMatrixs> ROTATE_MATRIXS = initializeAllRotateMatrixs();

    RotateMatrix initRotateMatrix_2D(const double& angle) {
        RotateMatrix mt = RotateMatrix(2, std::vector<int>(2, 0));
        mt[0][0] =  cos(angle);
        mt[0][1] =  sin(angle);
        mt[1][0] = -sin(angle);
        mt[1][1] =  cos(angle);
        return mt;
    }

    double orientToPi_2D(const int& orient) {
        double theta = 0;
        switch (orient) {
            case 0:
                theta = 0;
                break;
            case 1:
                theta = M_PI;
                break;
            case 2:
                theta = M_PI/2;
                break;
            case 3:
                theta = 3*M_PI/2;
                break;
            default:
                std::cerr << "wrong 2D orient = " << orient << std::endl;
                exit(0);
                break;
        }
        return theta;
    }

    float get_random_float(std::mt19937 *MT, float from, float to) {
        std::uniform_real_distribution<float> r(from, to);
        return r(*MT);
    }

    std::pair<int, std::set<int> > getMaxLevel(const std::vector<std::set<int> >& all_levels) {
        size_t max_level_size = 0;
        std::set<int> max_level;
        int index = 0;
        for(int i=0; i<all_levels.size(); i++) {
            if(max_level_size < all_levels[i].size()) {
                max_level = all_levels[i];
                max_level_size = all_levels[i].size();
                index = i;
            }
        }
        return {index, max_level};
    }

    size_t getMaxLevelSize(const std::vector<std::set<int> >& all_levels) {
        return getMaxLevel(all_levels).second.size();
    }


    // get the i th largest level, i_th start with 0
    std::pair<int, std::set<int> > getMaxLevel(const std::vector<std::set<int> >& all_levels, const int& i_th) {
        assert(!all_levels.empty());
        std::vector<int> sorting_vec(1, 0); // length, index
        for(int i=1; i<all_levels.size(); i++) {
            for(int j=0; j<sorting_vec.size(); j++) {
                if(all_levels[i].size() > all_levels[sorting_vec[j]].size()) {
                    sorting_vec.insert(sorting_vec.begin() + j, i);
                    break;
                } else if(j == sorting_vec.size() - 1) {
                    sorting_vec.push_back(i);
                    break;
                }
            }
        }
        if(i_th > sorting_vec.size()) {
            return {0, {}};
        } else {
            return {sorting_vec[i_th], {all_levels[sorting_vec[i_th]]}};
        }
    }



    // input a beam of laser scan, predict whether a robot will collide
    // angle = beam's angle in radius, dist = dist to center of agent
    bool PointInRectangle(const Pointf<2>& min_pt, const Pointf<2>& max_pt, float angle, float dist) {
        float x = dist*cos(angle), y = dist*sin(angle);
        if(x > max_pt[0] || x < min_pt[0]) {
            return false;
        }
        if(y > max_pt[1] || y < min_pt[1]) {
            return false;
        }
        return true;
    }

    // start angle is default to 0
    // end angle should be minor than 90 degree (i.e., 0.5*M_PI)
    bool PointInSector(float r, float end_angle, float angle, float dist) {
        if(r < dist) { return false; }
        if(end_angle < 0) {
            if(angle > 1e-4 || angle < end_angle - 1e-4) { return false; }
        } else {
            if(angle < -1e-4 || angle > end_angle + 1e-4) { return false; }
        }
        return true;
    }

    bool CircleAgentMoveCollisionCheck(float r, const Pointf<2>& move_vec, float angle, float dist) {
        assert(angle < 2*M_PI && angle >= 0);
        float x = dist*cos(angle), y = dist*sin(angle);
        // check whether collide with current circle
        if(dist <= r) { return true; }
        // check whether collide with future circle
        Pointf<2> beam_pt{x, y};
        Pointf<2> to_future = beam_pt - move_vec;
        if(to_future.Norm() < r) { return true; }
        // check whether collide with circle during move, via dist to line
        Pointf<2> origin_pt{0, 0};
        if(pointDistToLine(beam_pt, origin_pt, move_vec) <= r) { return true; }
        return false;
    }

    // assume agent only move in x direction, i.e., front or back
    bool BlockAgentMoveCollisionCheck(const Pointf<2>& min_pt, const Pointf<2>& max_pt,
                                      float move_dist, float angle, float dist) {
        //assert(min_pt[1] == -max_pt[1]);
        float new_max_x, new_min_x;
        if(move_dist < 0) {
            new_max_x = max_pt[0];
            new_min_x = min_pt[0] + move_dist;
        } else {
            new_max_x = max_pt[0] + move_dist;
            new_min_x = min_pt[0];
        }
        Pointf<2> new_min_pt{new_min_x, min_pt[1]}, new_max_pt{new_max_x, max_pt[1]};
        return PointInRectangle(new_min_pt, new_max_pt, angle, dist);
    }

    bool BlockAgentRotateCollisionCheck(const Pointf<2>& min_pt, const Pointf<2>& max_pt,
                                        float rotate_angle, float angle, float dist) {
        // check whether in current block
        if(PointInRectangle(min_pt, max_pt, angle, dist)) { return true; }
        // check whether in future block, rotate the beam to do collide check
        if(PointInRectangle(min_pt, max_pt, angle - rotate_angle, dist)) { return true; }
        // check whether intersect with the sector formed by block during rotate
        // front ector
        float r_front = max_pt.Norm();
        if(PointInSector(r_front, rotate_angle, angle, dist)) { return true; }
        // back sector
        float r_back = min_pt.Norm();
        if(PointInSector(r_back, rotate_angle, M_PI-angle, dist)) { return true; }
        return false;
    }

}