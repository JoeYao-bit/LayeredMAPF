//
// Created by yaozhuo on 2022/2/26.
//

#include "3d_viewer.h"
#include "../../freeNav-base/dependencies/color_table.h"
using namespace freeNav;
using namespace freeNav::LayeredMAPF;
bool window_running = true;

freeNav::Paths<2> Viewer3D::mapf_paths_;

void Viewer3D::DrawGridMapAtTimeIndex(DimensionLength *dimension_info,
                                          const IS_OCCUPIED_FUNC<2> &is_occupied,
                                          int time_index) {
    Pointi<2> pt;
    double r, g, b;
    for (int x = 0; x < dimension_info[0]; x++) {
        for (int y = 0; y < dimension_info[1]; y++) {
            pt[0] = x, pt[1] = y;
            r = g = b = (is_occupied(pt) ? 0.1 : 1.0);
            DrawVoxel(x, y, time_index, 0.5, r, g, b, true);
        }
    }
}


void Viewer3D::DrawMAPFPathAtTimeIndex(int time_index, bool draw_specific, int path_id, bool fix_height) {
    if (mapf_paths_.empty()) return;
    Pointi<2> pt;
    if (time_index >= 0) {
        if (path_id < 0) path_id = 0;
        path_id = path_id % mapf_paths_.size();
        for (int i = 0; i < mapf_paths_.size(); i++) {
            if (mapf_paths_[i].empty()) continue;
            auto color = COLOR_TABLE[i % 30];
            // draw path at time index
            if (time_index >= mapf_paths_[i].size()) {
                pt = mapf_paths_[i].back();
            } else {
                pt = mapf_paths_[i][time_index];
            }
            double r = .5, g = .5, b = .5;
            if (!draw_specific || path_id == i) {
                r = ((int) color[0]) / 255., g = ((int) color[1]) / 255., b = ((int) color[2]) / 255.;
            }
            if (fix_height) {
                DrawVoxel(pt[0], pt[1], .05, 0.5, r, g, b, true);
            } else {
                DrawVoxel(pt[0], pt[1], .05 + time_index, 0.5, r, g, b, true);
            }
            //break;
            // }
        }
    }
}

void Viewer3D::DrawOneMAPFPathAtTimeIndex(int time_index, int path_id, bool fix_height) {
    if (mapf_paths_.empty() || mapf_paths_.size() == 1) return;
    Pointi<2> pt;
    if (time_index >= 0) {
        if (path_id < 0) path_id = 0;
        path_id = path_id % mapf_paths_.size();
        auto color = COLOR_TABLE[path_id % 30];
        double r = .5, g = .5, b = .5;
        r = ((int) color[0]) / 255., g = ((int) color[1]) / 255., b = ((int) color[2]) / 255.;
        for (int i = 0; i < mapf_paths_[path_id].size(); i++) {
            pt = mapf_paths_[path_id][i];
            if (fix_height) {
                DrawVoxel(pt[0], pt[1], .05, 0.5, r, g, b, true);
            } else {
                DrawVoxel(pt[0], pt[1], .05 + time_index, 0.5, r, g, b, true);
            }
        }
    }
}
