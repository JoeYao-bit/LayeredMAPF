//
// Created by yaozhuo on 2024/6/28.
//

#ifndef LAYEREDMAPF_COMMON_INTERFACES_H
#define LAYEREDMAPF_COMMON_INTERFACES_H
#pragma once
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include "../../algorithm/LA-MAPF/circle_shaped_agent.h"
#include "../../algorithm/LA-MAPF/block_shaped_agent.h"

#include "../../algorithm/LA-MAPF/CBS/large_agent_CBS.h"
#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"

#include "../../algorithm/LA-MAPF/large_agent_instance_generator.h"
#include "../../algorithm/LA-MAPF/instance_serialize_and_deserialize.h"

#include "../../freeNav-base/visualization/canvas/canvas.h"
#include "../../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../test_data.h"

using namespace freeNav;
using namespace freeNav::LayeredMAPF;
using namespace freeNav::LayeredMAPF::LA_MAPF;

struct timezone tz;
struct timeval tv_pre, tv_cur;
struct timeval tv_after;

int zoom_ratio = 10;

Pointi<2> pt1;
int current_subgraph_id = 0;
bool draw_all_subgraph_node = false;
bool draw_all_instance = false;
bool draw_heuristic_table = false;
bool draw_path = true;
bool draw_full_path = true;


// MAPFTestConfig_Berlin_1_256
// MAPFTestConfig_maze_32_32_4
// MAPFTestConfig_warehouse_10_20_10_2_1
// MAPFTestConfig_warehouse_10_20_10_2_2
// MAPFTestConfig_Paris_1_256
// MAPFTestConfig_simple
// MAPFTestConfig_empty_48_48
// MAPFTestConfig_warehouse_20_40_10_2_1
// MAPFTestConfig_warehouse_20_40_10_2_2
// MAPFTestConfig_room_32_32_4
// MAPFTestConfig_room_64_64_8
// MAPFTestConfig_AR0011SR
// MAPFTestConfig_AR0012SR
// MAPFTestConfig_AR0013SR
// MAPFTestConfig_AR0014SR
// MAPFTestConfig_AR0015SR
// MAPFTestConfig_AR0016SR
auto map_test_config = MAPFTestConfig_Berlin_1_256;

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};

TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto dim = loader.getDimensionInfo();


template<typename AgentType, class Method>
void startLargeAgentMAPFTest(const std::vector<AgentType>& agents, const InstanceOrients<2>& instances) {
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
    //CBS::LargeAgentCBS;
    Method lacbs(instances, agents, dim, is_occupied);
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

                lacbs.agents_[i].drawOnCanvas({pt, orient}, canvas, COLOR_TABLE[(i) % 30]);
                canvas.drawArrowInt(pt[0], pt[1], -orientToPi_2D(orient) , 1, zoom_ratio/10);

            }
        }
        if(draw_all_instance) {
            //for (int i=0; i<instances.size(); i++)
            {
                const auto &instance = instances[current_subgraph_id];
                agents[current_subgraph_id].drawOnCanvas(instance.first, canvas, COLOR_TABLE[current_subgraph_id%30]);
                canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], -orientToPi_2D(instance.first.orient_) , 1, zoom_ratio/10);

                agents[current_subgraph_id].drawOnCanvas(instance.second, canvas, COLOR_TABLE[current_subgraph_id%30]);
                canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], -orientToPi_2D(instance.second.orient_), 1, zoom_ratio/10);

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


template<typename AgentType>
void InstanceVisualization(const std::vector<AgentType>& agents,
                           const LargeAgentMAPF_InstanceGenerator<2, AgentType>& generator,
                           const std::vector<InstanceOrient<2> >& instances,
                           const std::vector<LAMAPF_Path>& solution) {

    // visualize instance
    Canvas canvas("LargeAgentMAPF InstanceGenerator", dim[0], dim[1], .1, zoom_ratio);
    int time_index = 0;


    size_t makespan = getMakeSpan(solution);

    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied);

        if(draw_full_path) {
            const auto& all_poses = generator.getAllPoses();
            for(int i=0; i<solution.size(); i++) {
                const auto& path = solution[i];
                for(int t=0; t<path.size()-1; t++) {
                    Pointi<2> pt1 = all_poses[path[t]]->pt_,
                            pt2 = all_poses[path[t+1]]->pt_;
                    canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], true, std::max(1, zoom_ratio/10), COLOR_TABLE[(i) % 30]);
                }
            }
        }
        if(draw_all_instance) {
            for (int i=0; i<instances.size(); i++)
            {
                //const auto &instance = instances[current_subgraph_id]; // zoom_ratio/10
                const auto &instance = instances[i]; // zoom_ratio/10
                DrawOnCanvas(agents[i], instance.first, canvas, COLOR_TABLE[i%30]);

                DrawOnCanvas(agents[i], instance.second, canvas, COLOR_TABLE[i%30]);

                canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], -orientToPi_2D(instance.first.orient_), 1, std::max(1, zoom_ratio/10));
                canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], -orientToPi_2D(instance.second.orient_) , 1, std::max(1, zoom_ratio/10));

            }
        }
        if(draw_path) {
            const auto& all_poses = generator.getAllPoses();
            for(int i=0; i<solution.size(); i++) {
                const auto& path = solution[i];
                Pointi<2> pt;
                int orient = 0;
                if(time_index <= path.size() - 1) {
                    pt     = all_poses[path[time_index]]->pt_;
                    orient = all_poses[path[time_index]]->orient_;
                } else {
                    pt     = all_poses[path.back()]->pt_;
                    orient = all_poses[path.back()]->orient_;
                }

                DrawOnCanvas(agents[i], {pt, orient}, canvas, COLOR_TABLE[(i) % 30]);

                canvas.drawArrowInt(pt[0], pt[1], -orientToPi_2D(orient), 1, std::max(1, zoom_ratio/10));

            }
        }
        char key = canvas.show(1000);
        switch (key) {
            case 'i':
                draw_all_instance = !draw_all_instance;
                break;
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
            case 'p':
                draw_path = !draw_path;
                break;
            case 'f':
                draw_full_path = !draw_full_path;
                break;
            case 'q':
                if(makespan > 0) {
                    time_index = time_index + makespan - 1;
                    time_index = time_index % makespan;
                    std::cout << "-- switch to time index = " << time_index << std::endl;
                }
                break;
            case 'e':
                if(makespan > 0) {
                    time_index++;
//                    if(time_index > makespan) {
//                        time_index = makespan;
//                    }
                    time_index = time_index % makespan;
                    std::cout << "-- switch to time index = " << time_index << std::endl;
                }
                break;
            default:
                break;
        }
    }
}


template<typename AgentType, typename MethodType>
void loadInstanceAndPlanning(const std::string& file_path) {
    InstanceDeserializer<2, AgentType> deserializer;
    if(deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }

    gettimeofday(&tv_pre, &tz);
    MethodType method(deserializer.getInstances(), deserializer.getAgents(), dim, is_occupied);
    bool solved = method.solve(60, 0);
    gettimeofday(&tv_after, &tz);
    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
    std::cout << "instance has " << deserializer.getAgents().size() << " agents, find solution ? " << solved << " in " << time_cost << "ms " << std::endl;
    std::cout << "solution validation ? " << method.solutionValidation() << std::endl;

//    LargeAgentMAPF_InstanceGenerator<2, AgentType> generator(deserializer.getAgents(), is_occupied, dim);
//    InstanceVisualization<AgentType>(deserializer.getAgents(), generator, deserializer.getInstances(), method.getSolution());

}


template<typename AgentType, typename MethodType>
void generateInstance(const std::vector<AgentType>& agents, const std::string& file_path) {
    gettimeofday(&tv_pre, &tz);
    LargeAgentMAPF_InstanceGenerator<2, AgentType> generator(agents, is_occupied, dim, 1e7);
    for(int i=0; i<agents.size(); i++) {
        const auto& agent = agents[i];
        std::cout << agent << std::endl;
    }
    const auto& instances_and_path = generator.getNewInstance();
    gettimeofday(&tv_after, &tz);
    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;

    std::cout << "find instance ? " << !instances_and_path.empty() << " in " << time_cost << "ms " << std::endl;

    if(instances_and_path.empty()) {
        return;
    }

    InstanceOrients<2> instances;
    for(int i=0; i<instances_and_path.size(); i++) {
        instances.push_back(instances_and_path[i].first);
    }

    std::vector<LAMAPF_Path> solution;
    for(int i=0; i<instances_and_path.size(); i++) {
        solution.push_back(instances_and_path[i].second);
    }
    InstanceSerializer<2, AgentType> serializer(agents, instances);
    if(serializer.saveToFile(file_path)) {
        std::cout << "save to path " << file_path << " success" << std::endl;
    } else {
        std::cout << "save to path " << file_path << " failed" << std::endl;
        return;
    }

//    InstanceVisualization<AgentType>(agents, generator, instances, solution);
    loadInstanceAndPlanning<AgentType, MethodType>(file_path);
}

#endif //LAYEREDMAPF_COMMON_INTERFACES_H
