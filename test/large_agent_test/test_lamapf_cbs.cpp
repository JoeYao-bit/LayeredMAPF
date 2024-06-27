//
// Created by yaozhuo on 2024/5/5.
//

#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include "../../algorithm/LA-MAPF/circle_shaped_agent.h"
#include "../../algorithm/LA-MAPF/block_shaped_agent.h"

#include "../../algorithm/LA-MAPF/CBS/large_agent_CBS.h"
#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"

#include "../../freeNav-base/visualization/canvas/canvas.h"
#include "../../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../test_data.h"

using namespace freeNav;
using namespace freeNav::LayeredMAPF;
using namespace freeNav::LayeredMAPF::LA_MAPF;

struct timezone tz;
struct timeval tv_pre, tv_cur;
struct timeval tv_after;

CircleAgent<2> c1(2.5, 0);
CircleAgent<2> c2(3.5, 1);

int canvas_size_x = 1000, canvas_size_y = 700;

TEST(circleAgentIsCollide, cbs_test) {
    Pose<int, 2> p1({1,1},0), p2({1,2}, 0), p3({1,3}, 0), p4({4,1}, 0);
    std::cout << "is collide" << isCollide(c1, p1, p2, c2, p3, p4) << std::endl;
}

auto map_test_config = MAPFTestConfig_maze_32_32_4;

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};

TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto dim = loader.getDimensionInfo();

int zoom_ratio = 20;

Pointi<2> pt1;
int current_subgraph_id = 0;
bool draw_all_subgraph_node = false;
bool draw_all_instance = false;
bool draw_heuristic_table = false;
bool draw_path = true;
bool draw_full_path = true;



TEST(test, pointRotate_2D) {
    canvas_size_x = 10, canvas_size_y = 10;

    Canvas canvas("BlockRotateCoverage", canvas_size_x, canvas_size_y, 20, 10);

    Pointi<2> pt{4, 3}, pt1, pt2, pt3;
    pt1 = pointRotate_2D(pt, 1);
    pt2 = pointRotate_2D(pt, 2);
    pt3 = pointRotate_2D(pt, 3);

    std::cout << "after rotate 1 " << pt1 << std::endl;
    std::cout << "after rotate 2 " << pt2 << std::endl;
    std::cout << "after rotate 3 " << pt3 << std::endl;

    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGrid(5, 5);

        canvas.drawGridLine(canvas_size_x/2, canvas_size_y/2, pt[0]  + canvas_size_x/2, pt[1]  + canvas_size_y/2);
        canvas.drawGridLine(canvas_size_x/2, canvas_size_y/2, pt1[0] + canvas_size_x/2, pt1[1] + canvas_size_y/2);
        canvas.drawGridLine(canvas_size_x/2, canvas_size_y/2, pt2[0] + canvas_size_x/2, pt2[1] + canvas_size_y/2);
        canvas.drawGridLine(canvas_size_x/2, canvas_size_y/2, pt3[0] + canvas_size_x/2, pt3[1] + canvas_size_y/2);

        canvas.drawGrid(pt[0]  + canvas_size_x/2,  pt[1] + canvas_size_y/2,  COLOR_TABLE[0]);
        canvas.drawGrid(pt1[0] + canvas_size_x/2, pt1[1] + canvas_size_y/2, COLOR_TABLE[1]);
        canvas.drawGrid(pt2[0] + canvas_size_x/2, pt2[1] + canvas_size_y/2, COLOR_TABLE[2]);
        canvas.drawGrid(pt3[0] + canvas_size_x/2, pt3[1] + canvas_size_y/2, COLOR_TABLE[3]);


        char key = canvas.show();

    }
}

TEST(test, BlockRotateCoverage) {
    BlockAgent_2D block({-150, -100}, {200, 100}, 0, dim);
    int orient_start = 1, orient_end = 3;
    const auto& rotate_f_b = block.getRotateCoverage(orient_start, orient_end);
    bool draw_front_coverage = true, draw_backward_coverage = true, draw_rotate = true;
    int orient = 0;
    canvas_size_x = 1000, canvas_size_y = 700;
    Canvas canvas("BlockRotateCoverage", canvas_size_x, canvas_size_y, 20, 1);

    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        //canvas.drawArrowInt(0, (int)dim[1]/2, (int)dim[0], (int)dim[1]/2, 2, true, COLOR_TABLE[1]);
//        canvas.drawArrow(-2.4, 0, 0, 4.8, 2);
//        canvas.drawArrow(0, -2.4, M_PI/2, 4.8, 2);

        canvas.drawAxis(canvas_size_x/2, canvas_size_y/2, .2);
        // draw block
//        canvas.drawLine(block.min_pt_[0], block.min_pt_[1], block.max_pt_[0], block.min_pt_[1]);
//        canvas.drawLine(block.min_pt_[0], block.max_pt_[1], block.max_pt_[0], block.max_pt_[1]);
//
//        canvas.drawLine(block.min_pt_[0], block.min_pt_[1], block.min_pt_[0], block.max_pt_[1]);
//        canvas.drawLine(block.max_pt_[0], block.min_pt_[1], block.max_pt_[0], block.max_pt_[1]);

        // draw coverage grids
        if(draw_rotate) {
//            for (const auto &pt : block.grids_) {
//                canvas.drawGrid(pt[0] + canvas_size_x/2, pt[1] + canvas_size_y/2);
//            }
            for (const auto &pt : block.grids_[orient_start]) {
                canvas.drawGrid(pt[0] + canvas_size_x/2, pt[1] + canvas_size_y/2, COLOR_TABLE[orient_start+4]);
            }
            for (const auto &pt : block.grids_[orient_end]) {
                canvas.drawGrid(pt[0] + canvas_size_x/2, pt[1] + canvas_size_y/2, COLOR_TABLE[orient_end+4]);
            }
        }
        const auto& draw_which = rotate_f_b; // block.front_rotate_pts, block.backward_rotate_pts
        // draw rotate grids
        if(draw_front_coverage) {
            for (const auto &pt : draw_which.first) {
                canvas.drawGrid(pt[0] + canvas_size_x/2, pt[1] + canvas_size_y/2, COLOR_TABLE[1]);
            }
        }
        if(draw_backward_coverage) {
            for (const auto &pt : rotate_f_b.second) {
                canvas.drawGrid(pt[0] + canvas_size_x/2, pt[1] + canvas_size_y/2, COLOR_TABLE[2]);
            }
        }
        char key = canvas.show();
        switch (key) {
            case 'w':
                orient += 1;
                orient = orient % 4;
                std::cout << "-- change orient to " << orient << std::endl;
                break;
            case 's':
                orient += 3;
                orient = orient % 4;
                std::cout << "-- change orient to " << orient << std::endl;
                break;
            case 'f':
                draw_front_coverage    = !draw_front_coverage;
                break;
            case 'b':
                draw_backward_coverage = !draw_backward_coverage;
                break;
            case 'r':
                draw_rotate = !draw_rotate;
                break;
            default:
                break;
        }
    }
}




TEST(RectangleOverlap, test) {
    /*
     * rect (3.6, 12.6), (4.4, 13.4)
       rect (4.5, 21.6), (5.6, 22.4)
       rect (1.8, 12.7), (4.2, 14)
     * */
    Pointf<2> p1{3.6, 12.6}, p2{4.4, 13.4}, p3{1.8, 12.7}, p4{4.2, 14};
    std::cout << isRectangleOverlap<float, 2>(p1, p2, p3, p4) << std::endl;
}


template<typename AgentType>
void startCBSTest(const std::vector<AgentType>& agents, const InstanceOrients<2>& instances) {
    Canvas canvas("Large Agent MAPF Test", dim[0], dim[1], .1, zoom_ratio);
    auto mouse_call_back = [](int event, int x, int y, int flags, void *) {
        if(event == cv::EVENT_LBUTTONDOWN) {
            std::cout << "** get pt (" << x << ", " << y <<  ") **" << std::endl;
            pt1[0] = x;
            pt1[1] = y;
        }
    };
    canvas.setMouseCallBack(mouse_call_back);

    gettimeofday(&tv_pre, &tz);
    CBS::LargeAgentCBS<2, AgentType > lacbs(instances, agents, dim, is_occupied);
    bool solved = lacbs.solve(60, 0);
    gettimeofday(&tv_after, &tz);
    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
    std::cout << "find solution ? " << solved << " in " << time_cost << "ms " << std::endl;
    std::cout << "solution validation ? " << lacbs.solutionValidation() << std::endl;
    size_t makespan = lacbs.getMakeSpan();
    int time_index = 0;
    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied);
//        int r1 = a1.radius_*zoom_ratio, r2 = a2.radius_*zoom_ratio;
//        canvas.drawCircleInt(instances[0].first[0], instances[0].first[1], r1);
//        canvas.drawCircleInt(instances[1].first[0], instances[1].first[1], r2);
        const auto& current_subgraph = lacbs.agent_sub_graphs_[current_subgraph_id];

        if(draw_all_subgraph_node) {
            for (int i = 0; i < current_subgraph.all_nodes_.size(); i++) {
                const auto &node_ptr = current_subgraph.all_nodes_[i];
                if (node_ptr != nullptr) {
                    canvas.drawGrid(node_ptr->pt_[0], node_ptr->pt_[1], COLOR_TABLE[(current_subgraph_id + 2)%30]);
                }
            }
        }
        Id id = PointiToId(pt1, dim);
        //int orient = 0;
        for(int orient=0; orient<4; orient++)
        {
            auto current_node  = current_subgraph.all_nodes_[id * 4 + orient];
            auto current_edges = current_subgraph.all_edges_[id * 4 + orient];
            if (current_node != nullptr) {
                canvas.drawGrid(current_node->pt_[0], current_node->pt_[1], COLOR_TABLE[1]);
                for (const auto &edge_id : current_edges) {
                    if (current_subgraph.all_nodes_[edge_id]->pt_ == pt1) { continue; }
                    canvas.drawGrid(current_subgraph.all_nodes_[edge_id]->pt_[0],
                                    current_subgraph.all_nodes_[edge_id]->pt_[1],
                                    COLOR_TABLE[2]);
                }
            }
        }
        if(draw_full_path) {
            for(int i=0; i<lacbs.solutions_.size(); i++) {
                const auto& path = lacbs.solutions_[i];
                for(int t=0; t<path.size()-1; t++) {
                    Pointi<2> pt1 = lacbs.all_poses_[path[t]]->pt_, pt2 = lacbs.all_poses_[path[t+1]]->pt_;
                    canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], true, zoom_ratio/10, COLOR_TABLE[(i) % 30]);
                }
            }
        }
        if(draw_path) {
//            const auto& path = lacbs.solutions_[current_subgraph_id];
//            for (const auto &pose_id : path) {
//                canvas.drawGrid(lacbs.all_poses_[pose_id]->pt_[0],
//                                lacbs.all_poses_[pose_id]->pt_[1],
//                                COLOR_TABLE[(2+current_subgraph_id) % 30]);
//            }
            //std::cout << "draw path: " << std::endl;
            for(int i=0; i<lacbs.solutions_.size(); i++) {
                const auto& path = lacbs.solutions_[i];
                Pointi<2> pt;
                int orient = 0;
                if(time_index <= path.size() - 1) {
                    pt     = lacbs.all_poses_[path[time_index]]->pt_;
                    orient = lacbs.all_poses_[path[time_index]]->orient_;
                } else {
                    pt     = lacbs.all_poses_[path.back()]->pt_;
                    orient = lacbs.all_poses_[path.back()]->orient_;
                }
//                const auto& rect = lacbs.agents_[i].getPosedRectangle({pt, orient}); // agents
//                canvas.drawRectangleFloat(rect.first, rect.second, true, -1, COLOR_TABLE[(i) % 30]);
                //std::cout << "rect " << rect.first << ", " << rect.second << std::endl;

                DrawOnCanvas(lacbs.agents_[i], {pt, orient}, canvas, COLOR_TABLE[(i) % 30]);
                double theta = 0;
                switch (orient) {
                    case 0:
                        theta = 0;
                        break;
                    case 1:
                        theta = M_PI;
                        break;
                    case 2:
                        theta = 3*M_PI/2;
                        break;
                    case 3:
                        theta = M_PI/2;
                        break;
                    default:
                        std::cerr << "wrong 2D orient = " << orient << std::endl;
                        exit(0);
                        break;
                }
                canvas.drawArrowInt(pt[0], pt[1], theta , 1, zoom_ratio/10);

            }
        }
        if(draw_all_instance) {
            //for (int i=0; i<instances.size(); i++)
            {
                const auto &instance = instances[current_subgraph_id];
                canvas.drawGrid(instance.first.pt_[0], instance.first.pt_[1], COLOR_TABLE[(2 + current_subgraph_id)%30]);
                canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], 0 , 1, zoom_ratio/2);

                canvas.drawGrid(instance.second.pt_[0], instance.second.pt_[1], COLOR_TABLE[(2 + current_subgraph_id)%30]);
                canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], 0 , 1, zoom_ratio/2);

            }
        }
        if(draw_heuristic_table) {
            const auto& heuristic_table = lacbs.agents_heuristic_tables_[current_subgraph_id];
            Id total_index = getTotalIndexOfSpace<2>(dim);
            for(int i=0; i<total_index; i++) {
                // pick minimum heuristic value in all direction
                int value = MAX<int>;
                for(int orient=0; orient<4; orient++) {
                    if(heuristic_table[i*4 + orient] < value) {
                        value = heuristic_table[i*4 + orient];
                    }
                }
                if(value != MAX<int> && value < 10
                        ) {
                    Pointi<2> position = IdToPointi<2>(i, dim);
                    std::stringstream ss;
                    ss << value;
                    canvas.drawTextInt(position[0], position[1], ss.str().c_str(), cv::Vec3b::all(0), .5);
                }
            }
        }
        char key = canvas.show(1000);
        switch (key) {
            case 'w':
                current_subgraph_id ++;
                current_subgraph_id = current_subgraph_id % instances.size();
                std::cout << "-- switch to subgraph " << current_subgraph_id << std::endl;
                break;
            case 's':
                current_subgraph_id --;
                current_subgraph_id = (current_subgraph_id + instances.size()) % instances.size();
                std::cout << "-- switch to subgraph " << current_subgraph_id << std::endl;
                break;
            case 'a':
                draw_all_subgraph_node = !draw_all_subgraph_node;
                break;
            case 'i':
                draw_all_instance = !draw_all_instance;
                break;
            case 'h':
                draw_heuristic_table = !draw_heuristic_table;
                break;
            case 'p':
                draw_path = !draw_path;
                break;
            case 'f':
                draw_full_path = !draw_full_path;
                break;
            case 'q':
                time_index = time_index + makespan - 1;
                time_index = time_index % makespan;
                std::cout << "-- switch to time index = " << time_index << std::endl;
                break;
            case 'e':
                time_index ++;
                time_index = time_index % makespan;
                std::cout << "-- switch to time index = " << time_index << std::endl;
                break;
            default:
                break;
        }
    }
}

TEST(CircleAgentSubGraph, cbs_test) {

    // fake instances
    InstanceOrients<2> instances = {
            {{{8, 3}, 0}, {{23, 22},0} },
//            {{{9, 2}, 0}, {{5, 22}, 0}},
//            {{{2, 5}, 0}, {{17, 22}, 3}}
    };
    CircleAgents<2> agents({
                                   CircleAgent<2>(.3, 0),
//        CircleAgent<2>(.7, 1),
//        CircleAgent<2>(.6, 2)
                           });

    startCBSTest<CircleAgent<2> >(agents, instances);
}

TEST(BlockAgentSubGraph, cbs_test) {
    // fake instances
    InstanceOrients<2> instances = {
            {{{5, 3}, 0}, {{23, 22},0} },
            {{{9, 2}, 0}, {{5, 22}, 0}},
            {{{2, 5}, 0}, {{17, 22}, 3}}
    };
    Pointf<2> min_pt_0{-.4, -.4},  max_pt_0{.4, .4},
            min_pt_1{-.6, -.4},  max_pt_1{1., .4},
            min_pt_2{-.3, -1.2}, max_pt_2{1., 1.2};
    // NOTICE: initialize pt in constructor cause constant changed
    BlockAgents_2D agents({
                                        BlockAgent_2D({-.4, -.4}, {.4, .4}, 0, dim),
                                        BlockAgent_2D({-.6, -.4}, {1., .4}, 1, dim),
                                        BlockAgent_2D({-.3, -1.2},{1., 1.2}, 2, dim)
                                });

    startCBSTest<BlockAgent_2D>(agents, instances);
}