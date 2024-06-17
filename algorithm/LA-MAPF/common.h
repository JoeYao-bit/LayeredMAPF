//
// Created by yaozhuo on 2024/5/19.
//

#ifndef LAYEREDMAPF_COMMON_H
#define LAYEREDMAPF_COMMON_H

#include <limits>

#include <boost/geometry.hpp>
#include <boost/unordered_map.hpp>
#include <boost/heap/pairing_heap.hpp>

#include "../freeNav-base/basic_elements/distance_map_update.h"
#include "../../freeNav-base/basic_elements/point.h"
#include "../freeNav-base/basic_elements/point.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES MAX<size_t> / 2

    // <agent id, node from, node to, time range start, time range end>
    typedef std::tuple<int, size_t, size_t, int, int> Constraint;

    typedef std::list<std::shared_ptr<Constraint> > Constraints;

    // conflict resulted by two agents, and constraint store what move and when cause conflict
    //typedef std::pair<Constraint, Constraint> Conflict;

    enum conflict_type {
        STANDARD, TYPE_COUNT
    };

    enum conflict_priority {
        CARDINAL, PSEUDO_CARDINAL, SEMI, NON, UNKNOWN, PRIORITY_COUNT
    };

    struct Conflict {
        Conflict(int a1, int a2, const Constraints& cs1, const Constraints& cs2)
        : a1(a1), a2(a2), cs1(cs1), cs2(cs2) {
            int agent; size_t from, to;
            int start_t, end_t;
            // get start time index of conflict 1
            for(const auto& cs : cs1) {
                std::tie(agent, from, to, start_t, end_t) = *cs;
                t1 = start_t < t1 ? start_t : t1;
                t2 = end_t > t2   ? end_t   : t2;
            }
            for(const auto& cs : cs2) {
                std::tie(agent, from, to, start_t, end_t) = *cs;
                t1 = start_t < t1 ? start_t : t1;
                t2 = end_t > t2   ? end_t   : t2;
            }
        }

        int a1, a2; // agent id
        int t1 = MAX<int>, t2 = 0; // time range of constraints
        Constraints cs1;
        Constraints cs2;

        conflict_type type;
        conflict_priority priority = conflict_priority::UNKNOWN;
        double secondary_priority = 0; // used as the tie-breaking creteria for conflict selection
    };

    typedef std::vector<std::shared_ptr<Conflict> > Conflicts;

    bool operator<(const Conflict &conflict1, const Conflict &conflict2);


    typedef std::vector<size_t> LAMAPF_Path; // node id sequence

    bool isSamePath(const std::vector<size_t>& path1, const std::vector<size_t>& path2);

    template <Dimension N>
    struct Agent {

        explicit Agent(float excircle_radius, float incircle_radius, int id) : excircle_radius_(excircle_radius), incircle_radius_(incircle_radius),id_(id) {}

        virtual bool isCollide(const Pose<int, N>& pose,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N>& isoc,
                               const DistanceMapUpdater<N>& distance_table) const = 0;

        virtual bool isCollide(const Pose<int, N>& edge_from,
                               const Pose<int, N>& edge_to,
                               DimensionLength* dim,
                               const IS_OCCUPIED_FUNC<N>& isoc,
                               const DistanceMapUpdater<N>& distance_table) const = 0;

        float excircle_radius_, incircle_radius_;

        int id_;

    };

    template <Dimension N>
    using Agents = std::vector<Agent<N> >;

    class HighLvNode;

    /* [row][col] */
    typedef std::vector<std::vector<int> > RotateMatrix;

    typedef std::vector<RotateMatrix> RotateMatrixs;


    RotateMatrix initializeRotateMatrix(const int& dim, const int& orient);

    // for calculate rotate matrix about 2D and 3D maps
    RotateMatrixs initializeRotateMatrixs(const int& dim);

    std::vector<RotateMatrixs> initializeAllRotateMatrixs();

    template<typename T, Dimension N>
    Point<T, N> rotatePoint(const Point<T, N>& pt, const RotateMatrix& mt) {
        Point<T, N> retv;
        for(int row=0; row<N; row++) {
            retv[row] = 0;
            for(int col=0; col<N; col++) {
                retv[row] = retv[row] + pt[col]*mt[col][row];
            }
        }
        return retv;
    }

    template <Dimension N>
    bool isPointSetOverlap(const Pointis<N>& set1, const Pointis<N>& set2, DimensionLength* dim) {
        IdSet ids;
        for(const auto& pt : set1) {
            ids.insert(PointiToId(pt, dim));
        }
        Id id;
        for(const auto& pt : set2) {
            id = PointiToId(pt, dim);
            if(ids.find(id) != ids.end()) {
                return true;
            }
        }
        return false;
    }

    template<typename T, Dimension N>
    bool isRectangleOverlap(const Point<T, N>& r1_min, const Point<T, N>& r1_max,
                            const Point<T, N>& r2_min, const Point<T, N>& r2_max) {
        for(int dim=0; dim<N; dim++) {
            assert(r1_min[dim] < r1_max[dim]);
            assert(r2_min[dim] < r2_max[dim]);
            if(r1_max[dim] < r2_min[dim]) {
                return false;
            }
            if(r1_min[dim] > r2_max[dim]) {
                return false;
            }
        }
        return true;
    }

    template<typename T1, typename T2, Dimension N>
    bool isPointInRectangle(const Point<T1, N>& r1_min, const Point<T1, N>& r1_max,
                            const Point<T2, N>& pt) {
        std::vector<bool> flag(N, false);
        for(int dim=0; dim<N; dim++) {
            // if in all dimension that two rectangle are overlaped, they are overlaped
            if( (pt[dim] - r1_min[dim]) * (pt[dim] - r1_max[dim]) <= 0 ) { flag[dim] = true; }
        }
        return flag == std::vector<bool>(N, true);
    }

    // public static element
    extern std::vector<RotateMatrixs> ROTATE_MATRIXS;

    RotateMatrix initRotateMatrix_2D(const double& angle);

    double orientToPi_2D(const int& orient);

    template<typename T>
    Point<T, 2> pointRotate_2D(const Point<T, 2>& pt, const int& orient) {
        return rotatePoint<T, 2>(pt, ROTATE_MATRIXS[2][orient]);
    }

    template<typename T>
    Point<T, 2> pointTransfer_2D(const Point<T, 2>& pt, const Pose<int, 2>& pose) {
        Point<T, 2> rotated_pt = pointRotate_2D(pt, pose.orient_);
        rotated_pt[0] = rotated_pt[0] + pose.pt_[0];
        rotated_pt[1] = rotated_pt[1] + pose.pt_[1];
        return  rotated_pt;
    }

}

#endif //LAYEREDMAPF_COMMON_H
