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


TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto dim = loader.getDimensionInfo();

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

void visualizeAgentCoverage(const AgentPtr<2>& agent, const std::pair<Pointis<2>, Pointis<2> >& coverage_pair) {
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
            agent->drawOnCanvas(Pose<int, 2>(Pointi<2>{half_x, half_y}, 0), canvas, COLOR_TABLE[0], false);
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
    auto agent = std::make_shared<CircleAgent<2> >(2.2, 0, dim);
    std::pair<Pointis<2>, Pointis<2> > coverage_pair = getCircleCoverage<2>(agent->radius_);
    visualizeAgentCoverage(agent, coverage_pair);
}

TEST(BlockCoverageGrid, test) {
//    BlockAgent_2D agent(Pointf<2>{-2.7, -1.2}, Pointf<2>{4.3, 1.2}, 0, nullptr);
    auto agent = std::make_shared<BlockAgent_2D>(Pointf<2>{-.2, -.3}, Pointf<2>{.4, .3}, 0, nullptr);
    std::pair<Pointis<2>, Pointis<2> > coverage_pair = agent->grids_[0];
    visualizeAgentCoverage(agent, coverage_pair);
}

TEST(BlockRotateCoverageGrid, test) {



//    visualizeAgentCoverage<BlockAgent_2D >(agent, coverage_pair);
    Canvas canvas("Coverage visualize", dim[0], dim[1], .1, zoom_ratio);
    float half_x = dim[0]/2, half_y = dim[1]/2;
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

    float start_x = -10.2, end_x = 8.4;


    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
//        canvas.drawGridMap(dim, is_occupied);

    //    BlockAgent_2D agent(Pointf<2>{-2.7, -1.2}, Pointf<2>{4.3, 1.2}, 0, nullptr);
        BlockAgent_2D agent(Pointf<2>{start_x, -5}, Pointf<2>{end_x, 5}, 0, nullptr);
        std::pair<Pointis<2>, Pointis<2> > coverage_pair = agent.rotate_pts_[0];

        double min_radius = agent.min_pt_.Norm(), max_radius = agent.max_pt_.Norm();

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
            //agent.drawOnCanvas(Pose<int, 2>(Pointi<2>{half_x, half_y}, 0), canvas, COLOR_TABLE[0], false);
            //agent.drawOnCanvas(Pose<int, 2>(Pointi<2>{half_x, half_y}, 2), canvas, COLOR_TABLE[0], false);
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

            case 'q':
                start_x --;
                break;
            case 'e':
                if(start_x < -1) {
                    start_x ++;
                }
                break;
            case 'w':
                end_x ++;
                break;
            case 's':
                if(end_x > 1) {
                    end_x --;
                }
                break;

            default:
                break;
        }
    }
}

TEST(DrawOnCanvas, test) {
//    visualizeAgentCoverage<BlockAgent_2D >(agent, coverage_pair);
    Canvas canvas("Coverage visualize", 10, 10, 10., 100);
    canvas.resolution_ = 10.;
    BlockAgent_2D  block_agent(Pointf<2>{-2.3, -2}, Pointf<2>{4.1, 2}, 0, nullptr);
    CircleAgent<2> circle_agent(1.5, 0, nullptr);
//
//
    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        Pointf<3> ptf{0.3, 0.6, 0};
        //block_agent.drawOnCanvas(Pose<int, 2>(Pointi<2>{50, 50}, 0), canvas, cv::Vec3b(0,255,0));
        block_agent.drawOnCanvas(ptf, canvas, cv::Vec3b::all(0));

        //circle_agent.drawOnCanvas(Pose<int, 2>(Pointi<2>{50, 50}, 0), canvas, cv::Vec3b(255, 0, 0));
        circle_agent.drawOnCanvas(ptf, canvas, cv::Vec3b::all(0), false);

        char key = canvas.show(100);
    }
}