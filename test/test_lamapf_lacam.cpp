//
// Created by yaozhuo on 2024/6/21.
//


#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include "../algorithm/LA-MAPF/circle_shaped_agent.h"
#include "../algorithm/LA-MAPF/block_shaped_agent.h"

#include "../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"
#include "../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"

#include "../freeNav-base/visualization/canvas/canvas.h"
#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "test_data.h"

using namespace freeNav;
using namespace freeNav::LayeredMAPF;
using namespace freeNav::LayeredMAPF::LA_MAPF;

struct timezone tz;
struct timeval tv_pre, tv_cur;
struct timeval tv_after;

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


TEST(BlockAgentSubGraph, lacam_test) {
    Canvas canvas("Block agent", dim[0], dim[1], .1, zoom_ratio);
    auto mouse_call_back = [](int event, int x, int y, int flags, void *) {
        if(event == cv::EVENT_LBUTTONDOWN) {
            std::cout << "** get pt (" << x << ", " << y <<  ") **" << std::endl;
            pt1[0] = x;
            pt1[1] = y;
        }
    };
    canvas.setMouseCallBack(mouse_call_back);

    // fake instances
    InstanceOrients<2> instances = {
            {{{5, 3}, 0}, {{23, 22},0} },
            {{{9, 2}, 0}, {{5, 22}, 0}},
            {{{2, 5}, 0}, {{17, 22}, 3}}
    };
//    const Pointf<2> min_pt_0{-.2, -.2}, max_pt_0{.2, .2},
//                    min_pt_1{-.2, -.2}, max_pt_1{.2, .2},
//                    min_pt_2{-.4, -.4}, max_pt_2{.4, .4};
    const Pointf<2> min_pt_0{-.4, -.4}, max_pt_0{.4, .4},
            min_pt_1{-.6, -.4}, max_pt_1{1., .4},
            min_pt_2{-.3, -1.2}, max_pt_2{1., 1.2};
    // NOTICE: initialize pt in constructor cause constant changed
    const BlockAgents_2D agents({
                                        BlockAgent_2D(min_pt_0, max_pt_0, 0, dim),
                                        BlockAgent_2D(min_pt_1, max_pt_1, 1, dim),
                                        BlockAgent_2D(min_pt_2, max_pt_2, 2, dim)
                                });

    const auto seed = 0;
    auto MT = std::mt19937(seed);
    gettimeofday(&tv_pre, &tz);
    LaCAM::LargeAgentLaCAM<2, BlockAgent_2D > lacam(instances, agents, dim, is_occupied, &MT);
    bool solved = lacam.solve(60, 0);
    gettimeofday(&tv_after, &tz);
    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
    std::cout << "find solution ? " << solved << " in " << time_cost << "ms " << std::endl;
    std::cout << "solution validation ? " << lacam.solutionValidation() << std::endl;
    std::cout << "SOC / makespan = " << lacam.getSOC() << " / " <<  lacam.getMakeSpan() << std::endl;
    size_t makespan = lacam.getMakeSpan();
    int time_index = 0;
    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied);
//        int r1 = a1.radius_*zoom_ratio, r2 = a2.radius_*zoom_ratio;
//        canvas.drawCircleInt(instances[0].first[0], instances[0].first[1], r1);
//        canvas.drawCircleInt(instances[1].first[0], instances[1].first[1], r2);
        const auto& current_subgraph = lacam.agent_sub_graphs_[current_subgraph_id];

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
            for(int i=0; i<lacam.solutions_.size(); i++) {
                const auto& path = lacam.solutions_[i];
                for(int t=0; t<path.size()-1; t++) {
                    Pointi<2> pt1 = lacam.all_poses_[path[t]]->pt_, pt2 = lacam.all_poses_[path[t+1]]->pt_;
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
            for(int i=0; i<lacam.solutions_.size(); i++) {
                const auto& path = lacam.solutions_[i];
                Pointi<2> pt;
                int orient = 0;
                if(time_index <= path.size() - 1) {
                    pt     = lacam.all_poses_[path[time_index]]->pt_;
                    orient = lacam.all_poses_[path[time_index]]->orient_;
                } else {
                    pt     = lacam.all_poses_[path.back()]->pt_;
                    orient = lacam.all_poses_[path.back()]->orient_;
                }
                const auto& rect = lacam.agents_[i].getPosedRectangle({pt, orient}); // agents
                canvas.drawRectangleFloat(rect.first, rect.second, true, -1, COLOR_TABLE[(i) % 30]);
                //std::cout << "rect " << rect.first << ", " << rect.second << std::endl;

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
                const auto &instance = instances[current_subgraph_id]; // zoom_ratio/10
                canvas.drawGrid(instance.first.pt_[0], instance.first.pt_[1], COLOR_TABLE[(2 + current_subgraph_id)%30]);
                canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], 0 , 1, zoom_ratio/10);

                canvas.drawGrid(instance.second.pt_[0], instance.second.pt_[1], COLOR_TABLE[(2 + current_subgraph_id)%30]);
                canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], 0 , 1, zoom_ratio/10);

            }
        }
        if(draw_heuristic_table) {
            const auto& heuristic_table = lacam.agents_heuristic_tables_[current_subgraph_id];
            Id total_index = getTotalIndexOfSpace<2>(dim);
            for(int i=0; i<total_index; i++) {
                // pick minimum heuristic value in all direction
                int value = MAX<int>;
                for(int orient=0; orient<4; orient++) {
                    if(heuristic_table[i*4 + orient] < value) {
                        value = heuristic_table[i*4 + orient];
                    }
                }
                value = value%10;
                if(value != MAX<int> && value < 1000
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


TEST(CircleAgentSubGraph, lacam_test) {
    Canvas canvas("Circle agent", dim[0], dim[1], .1, zoom_ratio);
    auto mouse_call_back = [](int event, int x, int y, int flags, void *) {
        if(event == cv::EVENT_LBUTTONDOWN) {
            std::cout << "** get pt (" << x << ", " << y <<  ") **" << std::endl;
            pt1[0] = x;
            pt1[1] = y;
        }
    };
    canvas.setMouseCallBack(mouse_call_back);

    // fake instances
    InstanceOrients<2> instances = {
            {{{8, 3}, 0}, {{23, 22},0} },
            {{{9, 2}, 0}, {{5, 22}, 0}},
            {{{2, 5}, 0}, {{17, 22}, 3}}
    };
    CircleAgents<2> agents({
                                   CircleAgent<2>(.3, 0),
                                   CircleAgent<2>(.7, 1),
                                   CircleAgent<2>(.6, 2)
                           });

    const auto seed = 0;
    auto MT = std::mt19937(seed);
    gettimeofday(&tv_pre, &tz);
    LaCAM::LargeAgentLaCAM<2, CircleAgent<2> > lacam(instances, agents, dim, is_occupied, &MT);
    bool solved = lacam.solve(60, 0);
    gettimeofday(&tv_after, &tz);
    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
    std::cout << "find solution ? " << solved << " in " << time_cost << "ms " << std::endl;
    std::cout << "solution validation ? " << lacam.solutionValidation() << std::endl;
    std::cout << "SOC / makespan = " << lacam.getSOC() << " / " <<  lacam.getMakeSpan() << std::endl;
    size_t makespan = lacam.getMakeSpan();
    int time_index = 0;
    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied);
//        int r1 = a1.radius_*zoom_ratio, r2 = a2.radius_*zoom_ratio;
//        canvas.drawCircleInt(instances[0].first[0], instances[0].first[1], r1);
//        canvas.drawCircleInt(instances[1].first[0], instances[1].first[1], r2);
        const auto& current_subgraph = lacam.agent_sub_graphs_[current_subgraph_id];

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
            for(int i=0; i<lacam.solutions_.size(); i++) {
                const auto& path = lacam.solutions_[i];
                for(int t=0; t<path.size()-1; t++) {
                    Pointi<2> pt1 = lacam.all_poses_[path[t]]->pt_, pt2 = lacam.all_poses_[path[t+1]]->pt_;
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
            for(int i=0; i<lacam.solutions_.size(); i++) {
                const auto& path = lacam.solutions_[i];
                Pointi<2> pt;
                int orient = 0;
                if(time_index <= path.size() - 1) {
                    pt     = lacam.all_poses_[path[time_index]]->pt_;
                    orient = lacam.all_poses_[path[time_index]]->orient_;
                } else {
                    pt     = lacam.all_poses_[path.back()]->pt_;
                    orient = lacam.all_poses_[path.back()]->orient_;
                }
                canvas.drawCircleInt(pt[0], pt[1], floor(lacam.agents_[i].radius_*zoom_ratio),
                                     true, -1,
                                     COLOR_TABLE[(2+i) % 30]);

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
                const auto &instance = instances[current_subgraph_id]; // zoom_ratio/10
                canvas.drawGrid(instance.first.pt_[0], instance.first.pt_[1], COLOR_TABLE[(2 + current_subgraph_id)%30]);
                canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], 0 , 1, zoom_ratio/10);

                canvas.drawGrid(instance.second.pt_[0], instance.second.pt_[1], COLOR_TABLE[(2 + current_subgraph_id)%30]);
                canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], 0 , 1, zoom_ratio/10);

            }
        }
        if(draw_heuristic_table) {
            const auto& heuristic_table = lacam.agents_heuristic_tables_[current_subgraph_id];
            Id total_index = getTotalIndexOfSpace<2>(dim);
            for(int i=0; i<total_index; i++) {
                // pick minimum heuristic value in all direction
                int value = MAX<int>;
                for(int orient=0; orient<4; orient++) {
                    if(heuristic_table[i*4 + orient] < value) {
                        value = heuristic_table[i*4 + orient];
                    }
                }
                value = lacam.distance_to_target[current_subgraph_id][i];
                value = value%10;
                if(value != MAX<int>
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

TEST(sort, test) {
    std::vector<std::pair<size_t, int> > ids = {{0, 6}, {1, 3}, {2, 4}, {3, 5}, {4, 1}, {5, 0}};
    std::sort(ids.begin(), ids.end(), [&](const std::pair<size_t, int>& v, const std::pair<size_t, int>& u) {
        return v.second < u.second;
    });

    for(const auto& temp_pair : ids) {
        std::cout << "{" << temp_pair.first << ", " << temp_pair.second << "} ";
    }
    std::cout << std::endl;
    // {5, 0} {4, 1} {1, 3} {2, 4} {3, 5} {0, 6}
}

TEST(division, test) {
    std::cout << 0/2 << " " << 1/2 << " " << 2/2 << " " << 3/2 << std::endl;
    std::cout << 0/2 << " " << 1/2 << " " << 2/2 << " " << 3/2 << " " << 4/2 << " " << 5/2 << std::endl;

}