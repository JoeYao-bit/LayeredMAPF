//
// Created by yaozhuo on 2024/6/30.
//

#include "../../algorithm/LA-MAPF/common.h"
#include "../../algorithm/LA-MAPF/circle_shaped_agent.h"
#include "../../algorithm/LA-MAPF/block_shaped_agent.h"
#include <gtest/gtest.h>
#include "common_interfaces.h"

using namespace freeNav;
using namespace freeNav::LayeredMAPF;
using namespace freeNav::LayeredMAPF::LA_MAPF;

TEST(getAllCornerOfGrid, test) {
    const auto& corners_2D = getAllCornerOfGrid<2>();
    for(const auto& pt : corners_2D) {
        std::cout << "2D: " << pt << std::endl;
    }
    const auto& corners_3D = getAllCornerOfGrid<3>();
    for(const auto& pt : corners_3D) {
        std::cout << "3D: " << pt << std::endl;
    }
}

template<typename AgentType>
void visualizeAgentCoverage(const AgentType& agent, const std::pair<Pointis<2>, Pointis<2> >& coverage_pair) {
    Canvas canvas("Coverage visualize", dim[0], dim[1], .1, zoom_ratio);
    int half_x = dim[0]/2, half_y = dim[1]/2;
    auto mouse_call_back = [](int event, int x, int y, int flags, void *) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            std::cout << "** get pt (" << x << ", " << y << ") **" << std::endl;
            pt1[0] = x;
            pt1[1] = y;
        }
    };
    canvas.setMouseCallBack(mouse_call_back);
    bool draw_half = true;
    bool draw_full = true;
    bool draw_agent = true;

    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
//        canvas.drawGridMap(dim, is_occupied);
        if(draw_full) {
            for (const auto &pt : coverage_pair.first) {
                canvas.drawGrid(pt[0] + dim[0] / 2, pt[1] + dim[1] / 2, cv::Vec3b::all(0));
            }
        }
        if(draw_half) {
            for(const auto& pt : coverage_pair.second) {
                canvas.drawGrid(pt[0] + dim[0]/2, pt[1] + dim[1]/2, cv::Vec3b::all(200));
            }
        }
        if(draw_agent) {
            DrawOnCanvas(agent, Pose<int, 2>(Pointi<2>{half_x, half_y}, 0), canvas, COLOR_TABLE[0], false);
        }
        char key = canvas.show(100);
        switch (key) {
            case 'h':
                draw_half = !draw_half;
                break;
            case 'f':
                draw_full = !draw_full;
                break;
            case 'a':
                draw_agent = !draw_agent;
                break;
            default:
                break;
        }
    }
}

TEST(CircleCoverageGrid, test) {
    CircleAgent<2> agent(2.2, 0);
    std::pair<Pointis<2>, Pointis<2> > coverage_pair = getCircleCoverage<2>(agent.radius_);
    visualizeAgentCoverage<CircleAgent<2> >(agent, coverage_pair);
}

