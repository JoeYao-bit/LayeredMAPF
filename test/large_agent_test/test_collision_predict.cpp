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

    for(float angle=-M_PI; angle<=M_PI; angle+=0.05*M_PI) {
        for(float dist=0.1; dist<=1.0; dist+=0.05) {
            float x = dist*cos(angle), y = dist*sin(angle);
            pts.push_back(Pointf<2>{x, y});
            angles.push_back(angle);
            dists.push_back(dist);
            std::cout << "angle = " << angle << ", dist = " << dist << ", pt = " << pts.back() << std::endl;
        }
    }

    Pointf<2> min_pt{-0.3, -0.4}, max_pt{0.5, 0.4};
    float r = 0.8, end_angle = -0.8*M_PI;

    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawAxis(canvas_size_x/2/resolution, canvas_size_y/2/resolution, .2);

        // draw block and check, ok
        canvas.drawRectangle(min_pt[0], min_pt[1], max_pt[0], max_pt[1]);
//
//        for(int i=0; i<pts.size(); i++) {
//
//            const auto& pt = pts[i];
//            const auto& angle = angles[i];
//            const auto& dist = dists[i];
//            if(PointInRectangle(min_pt, max_pt, angle, dist)) {
//                canvas.drawCircle(pt[0], pt[1], .2, true, -1, COLOR_TABLE[1]);
//            } else {
//                canvas.drawCircle(pt[0], pt[1], .2, true, -1, COLOR_TABLE[0]);
//            }
//        }

        // draw eclipse and check, ok
        canvas.drawEclipse(0., 0., r, 0., end_angle, true, 1);
        // check whether point in circle
        for(int i=0; i<pts.size(); i++) {
            const auto& pt = pts[i];
            const auto& angle = angles[i];
            const auto& dist = dists[i];
            if(PointInSector(r, end_angle, angle, dist)) {
                canvas.drawCircle(pt[0], pt[1], .02, true, -1, COLOR_TABLE[1]);
            } else {
                canvas.drawCircle(pt[0], pt[1], .02, true, -1, COLOR_TABLE[0]);
            }
        }

        char key = canvas.show(100);
        switch (key) {
            default:
                break;
        }
    }

}