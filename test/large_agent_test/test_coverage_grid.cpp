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
    CircleAgent<2> agent(2.2, 0, dim);
    std::pair<Pointis<2>, Pointis<2> > coverage_pair = getCircleCoverage<2>(agent.radius_);
    visualizeAgentCoverage<CircleAgent<2> >(agent, coverage_pair);
}

TEST(BlockCoverageGrid, test) {
//    BlockAgent_2D agent(Pointf<2>{-2.7, -1.2}, Pointf<2>{4.3, 1.2}, 0, nullptr);
    BlockAgent_2D agent(Pointf<2>{-.2, -.3}, Pointf<2>{.4, .3}, 0, nullptr);
    std::pair<Pointis<2>, Pointis<2> > coverage_pair = agent.grids_[0];
    visualizeAgentCoverage<BlockAgent_2D >(agent, coverage_pair);
}

TEST(BlockRotateCoverageGrid, test) {
//    BlockAgent_2D agent(Pointf<2>{-2.7, -1.2}, Pointf<2>{4.3, 1.2}, 0, nullptr);
    BlockAgent_2D agent(Pointf<2>{-10.2, -5.3}, Pointf<2>{8.4, 5.3}, 0, nullptr);
    std::pair<Pointis<2>, Pointis<2> > coverage_pair = agent.rotate_pts_[0];
    double min_radius = agent.min_pt_.Norm(), max_radius = agent.max_pt_.Norm();
//    visualizeAgentCoverage<BlockAgent_2D >(agent, coverage_pair);
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
    bool draw_part = true;
    bool draw_full = true;
    bool draw_agent = true;
    bool draw_rect_grid = false;
    bool draw_circle = true;
    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
//        canvas.drawGridMap(dim, is_occupied);
        if(draw_rect_grid) {
            for(const auto& pt : agent.grids_[2].second) {
                canvas.drawGrid(pt[0] + half_x, pt[1] + half_y, cv::Vec3b::all(200));
            }
            for(const auto& pt : agent.grids_[0].second) {
                canvas.drawGrid(pt[0] + half_x, pt[1] + half_y, cv::Vec3b::all(200));
            }
            for(const auto& pt : agent.grids_[0].first) {
                canvas.drawGrid(pt[0] + half_x, pt[1] + half_y, cv::Vec3b::all(0));
            }
            for(const auto& pt : agent.grids_[2].first) {
                canvas.drawGrid(pt[0] + half_x, pt[1] + half_y, cv::Vec3b::all(0));
            }
        }

        if(draw_full) {
            for (const auto &pt : coverage_pair.first) {
                canvas.drawGrid(pt[0] + dim[0] / 2, pt[1] + dim[1] / 2, cv::Vec3b::all(0));
            }
        }
        if(draw_part) {
            for(const auto& pt : coverage_pair.second) {
                canvas.drawGrid(pt[0] + dim[0]/2, pt[1] + dim[1]/2, cv::Vec3b::all(200));
            }
        }

        if(draw_circle) {
            canvas.drawCircleInt(half_x, half_y, min_radius*zoom_ratio, true, 1, COLOR_TABLE[0]);
            canvas.drawCircleInt(half_x, half_y, max_radius*zoom_ratio, true, 1, COLOR_TABLE[0]);
        }
        if(draw_agent) {
            DrawOnCanvas(agent, Pose<int, 2>(Pointi<2>{half_x, half_y}, 0), canvas, COLOR_TABLE[0], false);
            DrawOnCanvas(agent, Pose<int, 2>(Pointi<2>{half_x, half_y}, 2), canvas, COLOR_TABLE[0], false);
        }
        char key = canvas.show(100);
        switch (key) {
            case 'p':
                draw_part = !draw_part;
                break;
            case 'f':
                draw_full = !draw_full;
                break;
            case 'a':
                draw_agent = !draw_agent;
                break;
            case 'r':
                draw_rect_grid = !draw_rect_grid;
                break;
            case 'c':
                draw_circle = !draw_circle;
                break;
            default:
                break;
        }
    }
}

