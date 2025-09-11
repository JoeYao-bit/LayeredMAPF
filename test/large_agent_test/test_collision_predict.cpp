//
// Created by yaozhuo on 9/9/25.
//

#include "../../algorithm/LA-MAPF/common.h"
#include "../../algorithm/LA-MAPF/circle_shaped_agent.h"
#include "../../algorithm/LA-MAPF/block_shaped_agent.h"

#include <gtest/gtest.h>
#include "common_interfaces.h"

using namespace freeNav;
using namespace freeNav::LayeredMAPF;
using namespace freeNav::LayeredMAPF::LA_MAPF;

int main() {
    int canvas_size_x = 30, canvas_size_y = 30;
    float resolution = 10;
    Canvas canvas("Coverage visualize", canvas_size_x, canvas_size_y, resolution, 20);

    Pointfs<2> pts;
    std::vector<float> angles, dists;
    // simulated laser scan

    for(float angle=0; angle<=2*M_PI; angle+=0.05*M_PI) {
        for(float dist=0.00; dist<=1.0; dist+=0.05) {
            float x = dist*cos(angle), y = dist*sin(angle);
            pts.push_back(Pointf<2>{x, y});
            angles.push_back(angle);
            dists.push_back(dist);
            //std::cout << "angle = " << angle << ", dist = " << dist << ", pt = " << pts.back() << std::endl;
        }
    }
    float width_of_block = 0.2;
    float min_x = -.1, max_x = .4;
    Pointf<2> min_pt{min_x, -width_of_block}, max_pt{max_x, width_of_block};
    float move_dist = -0.3;
    float rotate_angle = 0.8*M_PI;
    float r = 0.4, end_angle = -0.8*M_PI;
    Pointf<2> mov_vec{-0.3, 0.5};
    int count = 0;
    while(true) {
        count ++;
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawAxis(canvas_size_x/2/resolution, canvas_size_y/2/resolution, .2);

        // draw block and check, ok
        if(0) {
            canvas.drawRectangle(min_pt[0], min_pt[1], max_pt[0], max_pt[1]);

            for (int i = 0; i < pts.size(); i++) {

                const auto &pt = pts[i];
                const auto &angle = angles[i];
                const auto &dist = dists[i];
                if (PointInRectangle(min_pt, max_pt, angle, dist)) {
                    canvas.drawCircle(pt[0], pt[1], .2, true, -1, COLOR_TABLE[1]);
                } else {
                    canvas.drawCircle(pt[0], pt[1], .2, true, -1, COLOR_TABLE[0]);
                }
            }
        }
        // draw eclipse and check, ok
        if(0) {
            canvas.drawEclipse(0., 0., r, 0., end_angle, true, 1);
            // check whether point in circle
            for (int i = 0; i < pts.size(); i++) {
                const auto &pt = pts[i];
                const auto &angle = angles[i];
                const auto &dist = dists[i];
                if (PointInSector(r, 0, end_angle, angle, dist)) {
                    canvas.drawCircle(pt[0], pt[1], .02, true, -1, COLOR_TABLE[1]);
                } else {
                    canvas.drawCircle(pt[0], pt[1], .02, true, -1, COLOR_TABLE[0]);
                }
            }
        }
        // draw circle move and vec, ok
        if(0) {
            canvas.drawCircle(0, 0, r, true, 1);
            canvas.drawCircle(mov_vec[0], mov_vec[1], r, true, 1);
            // check whether point in circle
            for (int i = 0; i < pts.size(); i++) {
                const auto &pt = pts[i];
                const auto &angle = angles[i];
                const auto &dist = dists[i];
                if (CircleAgentMoveCollisionCheck(r, mov_vec, angle, dist)) {
                    canvas.drawCircle(pt[0], pt[1], .02, true, -1, COLOR_TABLE[1]);
                } else {
                    canvas.drawCircle(pt[0], pt[1], .02, true, -1, COLOR_TABLE[0]);
                }
            }
        }
        // draw move block and check, ok
        if(0) {
            canvas.drawRectangle(min_pt[0], min_pt[1], max_pt[0], max_pt[1]);
            canvas.drawRectangle(min_pt[0] + move_dist, min_pt[1],
                                 max_pt[0] + move_dist, max_pt[1],
                                 true, 1, COLOR_TABLE[0]);
            // check whether point in circle
            for (int i = 0; i < pts.size(); i++) {
                const auto &pt = pts[i];
                const auto &angle = angles[i];
                const auto &dist = dists[i];
                if (BlockAgentMoveCollisionCheck(min_pt, max_pt, move_dist, angle, dist)) {
                    canvas.drawCircle(pt[0], pt[1], .02, true, -1, COLOR_TABLE[1]);
                } else {
                    canvas.drawCircle(pt[0], pt[1], .02, true, -1, COLOR_TABLE[0]);
                }
            }
        }
        // draw rotate block and check, ok
        // draw raw block, ok
        if(1) {
            canvas.drawRectangle(min_pt[0], min_pt[1], max_pt[0], max_pt[1]);
            Pointf<2> pt1, pt2, pt3, pt4;

            pt1 = Pointf<2>{min_pt[0], max_pt[1]};
            pt2 = Pointf<2>{max_pt[0], max_pt[1]};
            pt3 = Pointf<2>{max_pt[0], min_pt[1]};
            pt4 = Pointf<2>{min_pt[0], min_pt[1]};

            pt1 = rotatePtf(pt1, rotate_angle);
            pt2 = rotatePtf(pt2, rotate_angle);
            pt3 = rotatePtf(pt3, rotate_angle);
            pt4 = rotatePtf(pt4, rotate_angle);

            // draw rotated block,
            canvas.drawLine(pt1[0], pt1[1], pt2[0], pt2[1]);
            canvas.drawLine(pt2[0], pt2[1], pt3[0], pt3[1]);
            canvas.drawLine(pt3[0], pt3[1], pt4[0], pt4[1]);
            canvas.drawLine(pt4[0], pt4[1], pt1[0], pt1[1]);

            // check whether point in circle
            for (int i = 0; i < pts.size(); i++) {
                const auto &pt = pts[i];
                const auto &angle = angles[i];
                const auto &dist = dists[i];
                if (BlockAgentRotateCollisionCheck(min_pt, max_pt, rotate_angle, angle, dist, count== 1)) {
                    canvas.drawCircle(pt[0], pt[1], .02, true, -1, COLOR_TABLE[1]);
                } else {
                    canvas.drawCircle(pt[0], pt[1], .02, true, -1, COLOR_TABLE[0]);
                }
            }
        }
        char key = canvas.show(2000);
        switch (key) {
            default:
                break;
        }
//        return 0;
    }

}