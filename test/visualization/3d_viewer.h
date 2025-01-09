//
// Created by yaozhuo on 2022/2/26.
//

#ifndef FREENAV_3D_VIEWER_H
#define FREENAV_3D_VIEWER_H

#include <mutex>
#include <thread>
#include <deque>
#include <functional>
#include <condition_variable>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/LU"
#include "eigen3/Eigen/Core"
#include <pangolin/pangolin.h>

#include "../../freeNav-base/basic_elements/point.h"
#include "../../freeNav-base/visualization/3d_viewer/3d_viewer.h"
#include "../../freeNav-base/dependencies/3d_octomap/octomap_loader.h"
#include "../../freeNav-base/dependencies/thread_pool.h"
#include "../../freeNav-base/dependencies/3d_octomap/octomap_loader.h"
#include "../../freeNav-base/dependencies/3d_textmap/voxel_loader.h"

#include "../../algorithm/constraint_table_CBS/common.h"

namespace freeNav::LayeredMAPF {

    struct Viewer3D : public Viewer3DBase {

        Viewer3D() : Viewer3DBase() {}

        ~Viewer3D() {}

        // 2D MAPF scene visualization
        void DrawGridMapAtTimeIndex(DimensionLength *dimension_info, const IS_OCCUPIED_FUNC<2> &is_occupied,
                                    int time_index);

        void DrawMAPFPathAtTimeIndex(int time_index, bool draw_specific, int path_id, bool fix_height = true);

        void DrawOneMAPFPathAtTimeIndex(int time_index, int path_id, bool fix_height = true);

        void SetMapfPath(const Paths<2> &multiple_paths) {
            mapf_paths_ = multiple_paths;
            max_time_index_ = 0;
            for (const auto &path : mapf_paths_) {
                max_time_index_ = std::max((int) path.size(), max_time_index_);
            }
        }

        static Paths<2> mapf_paths_; // for MAPF visualization

    };
}
#endif //FREENAV_3D_VIEWER_H
