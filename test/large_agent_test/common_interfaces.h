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
#include "../../algorithm/LA-MAPF/large_agent_instance_decomposition.h"

#include "../../freeNav-base/visualization/canvas/canvas.h"
#include "../../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../test_data.h"
#include "../../algorithm/LA-MAPF/CBS/layered_large_agent_CBS.h"

#include "../../algorithm/LA-MAPF/laryered_large_agent_mapf.h"


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
bool draw_heuristic_table_ignore_rotate = false;

bool draw_path = true;
bool draw_full_path = true;
bool draw_visit_grid_table = false;

// MAPFTestConfig_Berlin_1_256
// MAPFTestConfig_maze_32_32_4
// MAPFTestConfig_warehouse_10_20_10_2_1
// MAPFTestConfig_warehouse_10_20_10_2_2
// MAPFTestConfig_Paris_1_256
// MAPFTestConfig_simple
// MAPFTestConfig_empty_48_48 //
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
auto map_test_config = MAPFTestConfig_empty_48_48;
// MAPFTestConfig_Paris_1_256 //  pass
// MAPFTestConfig_Berlin_1_256; // pass
// MAPFTestConfig_maze_32_32_4; // pass
// MAPFTestConfig_simple;
// MAPFTestConfig_AR0011SR; // pass
// MAPFTestConfig_empty_48_48 // pass

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};

TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto dim = loader.getDimensionInfo();


template<Dimension N, class Method>
void startLargeAgentMAPFTest(const std::vector<AgentPtr<N> >& agents, const InstanceOrients<2>& instances) {
    zoom_ratio = std::min(2560/dim[0], 1400/dim[1]);

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
    bool solved = lacbs.solve(5, 0);
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
                    canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], true, std::max(zoom_ratio/10, 1), COLOR_TABLE[(i) % 30]);
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

                lacbs.agents_[i]->drawOnCanvas({pt, orient}, canvas, COLOR_TABLE[(i) % 30]);
                canvas.drawArrowInt(pt[0], pt[1], -orientToPi_2D(orient) , 1, std::max(zoom_ratio/10, 1));

            }
        }
        if(draw_all_instance) {
            //for (int i=0; i<instances.size(); i++)
            {
                const auto &instance = instances[current_subgraph_id];
                agents[current_subgraph_id]->drawOnCanvas(instance.first, canvas, COLOR_TABLE[current_subgraph_id%30]);
                canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], -orientToPi_2D(instance.first.orient_) , 1, std::max(zoom_ratio/10, 1));

                agents[current_subgraph_id]->drawOnCanvas(instance.second, canvas, COLOR_TABLE[current_subgraph_id%30]);
                canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], -orientToPi_2D(instance.second.orient_), 1, std::max(zoom_ratio/10, 1));

            }
        }
        // agents_heuristic_tables_ignore_rotate_
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
                if(value != MAX<int> // && value < 10
                        ) {
                    Pointi<2> position = IdToPointi<2>(i, dim);
                    std::stringstream ss;
                    ss << "h: " << value;
                    canvas.drawTextInt(position[0], position[1], ss.str().c_str(), cv::Vec3b::all(0), .5);
                }
            }
        }
        if(draw_heuristic_table_ignore_rotate) {
            const auto& heuristic_table = lacbs.agents_heuristic_tables_ignore_rotate_[current_subgraph_id];
            Id total_index = getTotalIndexOfSpace<2>(dim);
            for(int i=0; i<total_index; i++) {
                // pick minimum heuristic value in all direction
                int value = MAX<int>;
                value = heuristic_table[i];
                if(value != MAX<int>// && value < 10
                        ) {
                    Pointi<2> position = IdToPointi<2>(i, dim);
                    std::stringstream ss;
                    ss << "hir: " << value;
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
            case 'r':
                draw_heuristic_table_ignore_rotate = !draw_heuristic_table_ignore_rotate;
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


void InstanceVisualization(const std::vector<AgentPtr<2> >& agents,
                           const std::vector<PosePtr<int, 2> >& all_poses,
                           const std::vector<InstanceOrient<2> >& instances,
                           const std::vector<LAMAPF_Path>& solution,
                           const std::vector<std::vector<int> >& grid_visit_count_table = {}) {
    zoom_ratio = std::min(2560/dim[0], 1400/dim[1]);

    // visualize instance
    Canvas canvas("LargeAgentMAPF InstanceGenerator", dim[0], dim[1], .1, zoom_ratio);
    int time_index = 0;


    size_t makespan = getMakeSpan(solution);
    draw_all_instance = false;
    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied);

        if(draw_full_path) {
            for(int i=0; i<solution.size(); i++)
            {
                const auto& path = solution[i];
                if(path.empty()) { continue; }
                for(int t=0; t<path.size()-1; t++) {
                    Pointi<2> pt1 = all_poses[path[t]]->pt_,
                            pt2 = all_poses[path[t+1]]->pt_;
                    canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], true, std::max(1, zoom_ratio/10), COLOR_TABLE[(i) % 30]);
                }
            }
        } else {
            int i = current_subgraph_id;
            if(!solution.empty()) {
                //for(int i=0; i<solution.size(); i++)
                {
                    const auto &path = solution[i];
                    if (!path.empty()) {
                        for (int t = 0; t < path.size() - 1; t++) {
                            Pointi<2> pt1 = all_poses[path[t]]->pt_,
                                    pt2 = all_poses[path[t + 1]]->pt_;
                            canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], true, std::max(1, zoom_ratio / 10),
                                               COLOR_TABLE[(i) % 30]);
                        }
                    }
                }
            }
        }
        if(draw_all_instance) {
            if(draw_full_path) {
                for (int i=0; i<instances.size(); i++)
                {
                    //const auto &instance = instances[current_subgraph_id]; // zoom_ratio/10
                    const auto &instance = instances[i]; // zoom_ratio/10
                    agents[i]->drawOnCanvas(instance.first, canvas, COLOR_TABLE[i%30]);

                    agents[i]->drawOnCanvas(instance.second, canvas, COLOR_TABLE[i%30]);

                    canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], -orientToPi_2D(instance.first.orient_), 1, std::max(1, zoom_ratio/10));
                    canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], -orientToPi_2D(instance.second.orient_) , 1, std::max(1, zoom_ratio/10));

                }
            } else {
                const auto &instance = instances[current_subgraph_id]; // zoom_ratio/10
                agents[current_subgraph_id]->drawOnCanvas(instance.first, canvas, COLOR_TABLE[current_subgraph_id%30]);

                agents[current_subgraph_id]->drawOnCanvas(instance.second, canvas, COLOR_TABLE[current_subgraph_id%30]);

                canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], -orientToPi_2D(instance.first.orient_), 1, std::max(1, zoom_ratio/10));
                canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], -orientToPi_2D(instance.second.orient_) , 1, std::max(1, zoom_ratio/10));

            }
        }
        if(draw_path) {
            if(draw_full_path) {
                for(int i=0; i<solution.size(); i++)
                {
                    const auto &path = solution[i];
                    if (path.empty()) { continue; }
                    Pointi<2> pt;
                    int orient = 0;
                    if (time_index <= path.size() - 1) {
                        pt = all_poses[path[time_index]]->pt_;
                        orient = all_poses[path[time_index]]->orient_;
                    } else {
                        pt = all_poses[path.back()]->pt_;
                        orient = all_poses[path.back()]->orient_;
                    }

                    agents[i]->drawOnCanvas({pt, orient}, canvas, COLOR_TABLE[(i) % 30]);

                    canvas.drawArrowInt(pt[0], pt[1], -orientToPi_2D(orient), 1, std::max(1, zoom_ratio / 10));

                }
            } else {
                int i = current_subgraph_id;
                if(!solution.empty()) {
                    const auto &path = solution[i];
                    if (!path.empty()) {
                        Pointi<2> pt;
                        int orient = 0;
                        if (time_index <= path.size() - 1) {
                            pt = all_poses[path[time_index]]->pt_;
                            orient = all_poses[path[time_index]]->orient_;
                        } else {
                            pt = all_poses[path.back()]->pt_;
                            orient = all_poses[path.back()]->orient_;
                        }

                        agents[i]->drawOnCanvas({pt, orient}, canvas, COLOR_TABLE[(i) % 30]);

                        canvas.drawArrowInt(pt[0], pt[1], -orientToPi_2D(orient), 1, std::max(1, zoom_ratio / 10));
                    }
                }
            }
        }
        if(draw_visit_grid_table) {
            if(!grid_visit_count_table.empty()) {
                const auto& local_grid_visit_count_table = grid_visit_count_table[current_subgraph_id];
                if(!local_grid_visit_count_table.empty()){
                    Id total_index = getTotalIndexOfSpace<2>(dim);
                    for(int i=0; i<total_index; i++) {
                        Pointi<2> position = IdToPointi<2>(i, dim);
                        int value = local_grid_visit_count_table[i];
                        if(value != 0) {
                            std::stringstream ss;
                            ss << value;
                            canvas.drawTextInt(position[0], position[1], ss.str().c_str(), cv::Vec3b::all(200), .5);
                        }
                    }
                }
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
            case 'g':
                draw_visit_grid_table = !draw_visit_grid_table;
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

/*
 *     auto layered_paths = layeredLargeAgentMAPF<2, AgentType>(deserializer.getInstances(),
                                                             deserializer.getAgents(),
                                                             dim, is_occupied,
                                                             CBS::LargeAgentCBS_func<2, AgentType >,
                                                             grid_visit_count_table,
                                                             30, decomposer_ptr,
                                                             true);
 * */

template<typename MethodType>
void loadInstanceAndPlanning(const std::string& file_path, double time_limit = 30) {
    InstanceDeserializer<2> deserializer;
    if(deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << " map scale " << dim[0] << "*" << dim[1] << std::endl;

    auto start_t = clock();
    MethodType method(deserializer.getInstances(), deserializer.getAgents(), dim, is_occupied);
    auto init_t = clock();
    double init_time_cost = (((double)init_t - start_t)/CLOCKS_PER_SEC);
    if(init_time_cost >= time_limit) {
        std::cout << "NOTICE: init of large agent MAPF instance run out of time" << std::endl;
        return;
    }
    bool solved = method.solve(time_limit - init_time_cost, 0);
    auto solve_t = clock();

    double total_time_cost = (((double)solve_t - start_t)/CLOCKS_PER_SEC);

    std::cout << "instance has " << deserializer.getAgents().size() << " agents, find solution ? " << solved
              << " in " << total_time_cost << "s " << std::endl;
    std::cout << "solution validation ? " << method.solutionValidation() << std::endl;

    LargeAgentMAPF_InstanceGenerator<2> generator(deserializer.getAgents(), is_occupied, dim);
    InstanceVisualization(deserializer.getAgents(), generator.getAllPoses(), deserializer.getInstances(),
                                     method.getSolution()
    );

//    InstanceVisualization<AgentType>(deserializer.getAgents(), generator.getAllPoses(), deserializer.getInstances(), {});

}


//LaCAM::LargeAgentConstraints<2, BlockAgent_2D>
template<Dimension N>
void loadInstanceAndVisualize(const std::string& file_path) {
    InstanceDeserializer<N> deserializer;
    if(deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << " map scale " << dim[0] << "*" << dim[1] << std::endl;

    // debug: print all agents and instance
    std::cout << "Instance: " << std::endl;
    std::vector<std::string> strs = deserializer.getTextString();
    for(const auto& str : strs) {
        std::cout << str << std::endl;
    }
    std::cout << std::endl;

    LargeAgentMAPF_InstanceGenerator<N> generator(deserializer.getAgents(), is_occupied, dim);

    InstanceVisualization(deserializer.getAgents(),
                          generator.getAllPoses(),
                          deserializer.getInstances(),
                          {},
                          {});

}

//LaCAM::LargeAgentConstraints<2, BlockAgent_2D>
template<Dimension N>
void loadInstanceAndPlanningLayeredLAMAPF(const LA_MAPF_FUNC<N>& mapf_func,
                                          const std::string& file_path,
                                          double time_limit = 30,
                                          bool path_constraint = false,
                                          bool debug_mode = true,
                                          bool visualize = false) {
    InstanceDeserializer<N> deserializer;
    if(deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << " map scale " << dim[0] << "*" << dim[1] << std::endl;

    // debug: print all agents and instance
    std::cout << "Instance: " << std::endl;
    std::vector<std::string> strs = deserializer.getTextString();
    for(const auto& str : strs) {
        std::cout << str << std::endl;
    }
    std::cout << std::endl;

//    LargeAgentMAPF_InstanceGenerator<2, AgentType> generator(deserializer.getAgents(), is_occupied, dim, 1e7);
//
//    std::cout << " solvable ?  " << !((generator.getConnectionBetweenNode(7, 89773, 108968)).empty()) << std::endl;

    std::vector<std::vector<int> > grid_visit_count_table;

    LargeAgentMAPFInstanceDecompositionPtr<2> decomposer_ptr = nullptr;
    auto start_t = clock();
    auto layered_paths = layeredLargeAgentMAPF<N>(deserializer.getInstances(),
                                                             deserializer.getAgents(),
                                                             dim, is_occupied,
                                                             mapf_func, //CBS::LargeAgentCBS_func<2, AgentType >,
                                                             grid_visit_count_table,
                                                             time_limit, decomposer_ptr,
                                                             path_constraint,
                                                             debug_mode);
    auto end_t = clock();
    double total_time_cost = ((double)end_t - start_t)/CLOCKS_PER_SEC;
    std::cout << "instance has " << deserializer.getAgents().size() << " agents, find solution ? " << !layered_paths.empty()
              << " in " << total_time_cost << "s " << std::endl;

    if(visualize) {
        LargeAgentMAPF_InstanceGenerator<N> generator(deserializer.getAgents(), is_occupied, dim);

        InstanceVisualization(deserializer.getAgents(),
                                         generator.getAllPoses(),
                                         deserializer.getInstances(),
                                         layered_paths,
                                         grid_visit_count_table);
    }
}


// maximum_sample_count: max times of sample start and target for an agent
template<Dimension N>
void generateInstance(const std::vector<AgentPtr<N> >& agents,
                      const std::string& file_path,
                      const LA_MAPF_FUNC<2>& mapf_func,
                      int maximum_sample_count = 1e7) {
    gettimeofday(&tv_pre, &tz);

    // get previous texts as backup
    InstanceDeserializer<N> deserializer_1;
    deserializer_1.loadInstanceFromFile(file_path, dim);
    std::vector<std::string> backup_strs = deserializer_1.getTextString();


    // test whether serialize and deserialize will change agent's behavior
    // even we didn't change their behavior explicitly
    InstanceOrients<N> fake_instances;
    for(int i=0; i<agents.size(); i++) {
        fake_instances.push_back({Pose<int, N>{{0,0},0}, Pose<int, N>{{0,0},0}});
    }
    InstanceSerializer<N> fake_serializer(agents, fake_instances);
    if(!fake_serializer.saveToFile(file_path)) {
        std::cout << "fake_serializer save to path " << file_path << " failed" << std::endl;
        return;
    }
    InstanceDeserializer<N> fake_deserializer;
    if(!fake_deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    auto new_agents = fake_deserializer.getAgents();

    // restore backup texts
    fake_serializer.saveStrsToFile(backup_strs, file_path);

    LargeAgentMAPF_InstanceGenerator<N> generator(new_agents, is_occupied, dim, maximum_sample_count);

    // debug: print all agents
    for(int i=0; i<agents.size(); i++) {
        const auto& agent = agents[i];
        std::cout << agent->serialize() << std::endl;
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
    InstanceSerializer<N> serializer(new_agents, instances);
    if(serializer.saveToFile(file_path)) {
        std::cout << "save to path " << file_path << " success" << std::endl;
    } else {
        std::cout << "save to path " << file_path << " failed" << std::endl;
        return;
    }
}

// maximum_sample_count: max times of sample start and target for an agent
template<Dimension N>
void generateInstanceAndPlanning(const std::vector<AgentPtr<N> >& agents,
                                 const std::string& file_path,
                                 const LA_MAPF_FUNC<N>& mapf_func,
                                 int maximum_sample_count = 1e7,
                                 bool debug_mode = true,
                                 bool visualize = false) {

    //    loadInstanceAndPlanning<AgentType, MethodType>(file_path);
    generateInstance(agents, file_path, mapf_func, maximum_sample_count);
    loadInstanceAndPlanningLayeredLAMAPF<N>(mapf_func, file_path, 60, false, debug_mode, visualize);
}

template<Dimension N>
void InstanceDecompositionVisualization(const LargeAgentMAPFInstanceDecomposition<N>& decomposer) {
    zoom_ratio = std::min(2560/dim[0], 1400/dim[1]);

    // visualize instance
    Canvas canvas("LargeAgentMAPF Decomposition", dim[0], dim[1], .1, zoom_ratio);

    bool draw_related_agents_map = false;
    bool draw_hyper_node_id = false;
    bool draw_heuristic_table = false;
    int total_index = getTotalIndexOfSpace<2>(dim);
    std::vector<std::vector<std::string> > grid_strs(total_index);

    Pointi<2> pt;
    size_t node_id;
    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied);

        grid_strs = std::vector<std::vector<std::string> >(total_index);

        if(draw_all_instance) {
            const auto& instances = decomposer.instances_;
            const auto& agents    = decomposer.agents_;
            for (int current_subgraph_id=0; current_subgraph_id<instances.size(); current_subgraph_id++)
            {
                const auto &instance = instances[current_subgraph_id];
                agents[current_subgraph_id]->drawOnCanvas(instance.first, canvas, COLOR_TABLE[current_subgraph_id%30]);
                canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], -orientToPi_2D(instance.first.orient_) ,
                                    1, std::max(1,zoom_ratio/10));

                agents[current_subgraph_id]->drawOnCanvas(instance.second, canvas, COLOR_TABLE[current_subgraph_id%30]);
                canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], -orientToPi_2D(instance.second.orient_),
                                    1, std::max(1,zoom_ratio/10));

            }
        }
        if(draw_related_agents_map) {
//            std::cout << " draw_related_agents_map " << std::endl;
            const auto& hyper_graph = decomposer.connect_graphs_[current_subgraph_id];
            for(int i=0; i<total_index; i++) {
                pt = IdToPointi<2>(i, dim);
                std::set<int> related_agents;
                for(int orient=0; orient<4; orient++) {
                    node_id = i*4  + orient;
                    for(const int& related_agent : hyper_graph.related_agents_map_[node_id]) {
                        related_agents.insert(related_agent);
                    }
                }
                const auto& subgraph = decomposer.agent_sub_graphs_[current_subgraph_id];
                if(subgraph.all_nodes_[node_id] == nullptr) { continue; }
                std::stringstream ss;
                ss << "r: ";
                for(const int& related_agent : related_agents) {
                    ss << related_agent << " ";
                }
                grid_strs[i].push_back(ss.str());
            }
        }
        if(draw_hyper_node_id) {
            const auto& hyper_graph = decomposer.connect_graphs_[current_subgraph_id];
            for(int i=0; i<total_index; i++) {
                pt = IdToPointi<2>(i, dim);
                std::set<int> hyper_nodes; // a point have four direction, may be more than one
                for(int orient=0; orient<4; orient++) {
                    node_id = i*4  + orient;
                    if(hyper_graph.hyper_node_id_map_[node_id] != MAX<size_t>) {
                        hyper_nodes.insert(hyper_graph.hyper_node_id_map_[node_id]);
                    }
                }
                const auto& subgraph = decomposer.agent_sub_graphs_[current_subgraph_id];
                if(subgraph.all_nodes_[node_id] == nullptr) { continue; }
                std::stringstream ss;
                ss << "n: ";
                for(const int& related_agent : hyper_nodes) {
                    ss << related_agent << " ";
                }
                grid_strs[i].push_back(ss.str());
            }
        }
        if(draw_heuristic_table) {
            const auto& hyper_graph = decomposer.connect_graphs_[current_subgraph_id];
            const auto& heuristic_table = decomposer.heuristic_tables_[current_subgraph_id];
            const auto& subgraph = decomposer.agent_sub_graphs_[current_subgraph_id];

            for(int i=0; i<total_index; i++) {
                pt = IdToPointi<2>(i, dim);
                std::set<int> heuristic_values; // a point have four direction, may be more than one
                for(int orient=0; orient<4; orient++) {
                    node_id = i*4  + orient;
                    if(subgraph.all_nodes_[node_id] == nullptr) { continue; }
                    if(hyper_graph.hyper_node_id_map_[node_id] == MAX<size_t>) { continue; }
                    if(heuristic_table[hyper_graph.hyper_node_id_map_[node_id]] != MAX<int>) {
                        heuristic_values.insert(heuristic_table[hyper_graph.hyper_node_id_map_[node_id]]);
                    }
                }
                if(heuristic_values.empty()) { continue; }
                std::stringstream ss;
                ss << "h: ";
                for(const int& value : heuristic_values) {
                    ss << value << " ";
                }
                grid_strs[i].push_back(ss.str());
            }
        }

        for(int i=0; i<total_index; i++) {
            pt = IdToPointi<2>(i, dim);
            canvas.drawMultiTextInt(pt[0], pt[1], grid_strs[i], cv::Vec3b::all(0), 1.0);
        }

        char key = canvas.show(100);
        switch (key) {
            case 'i':
                draw_all_instance = !draw_all_instance;
                break;
            case 'r':
                draw_related_agents_map = !draw_related_agents_map;
                break;
            case 'n':
                draw_hyper_node_id = !draw_hyper_node_id;
                break;
            case 'h':
                draw_heuristic_table = !draw_heuristic_table;
                break;
            case 'w':
                current_subgraph_id = current_subgraph_id+1;
                current_subgraph_id = current_subgraph_id % decomposer.agents_.size();
                break;
            case 's':
                current_subgraph_id = current_subgraph_id + decomposer.agents_.size() - 1;
                current_subgraph_id = current_subgraph_id % decomposer.agents_.size();
                break;
            default:
                break;
        }
    }
}

template<Dimension N>
void loadInstanceAndDecomposition(const std::string& file_path) {
    InstanceDeserializer<N> deserializer;
    if (deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << " map scale ";
    for(int i=0; i<N-1; i++) {
        std::cout << dim[i] << "*";
    }
    std::cout << dim[N-1] << std::endl;

    gettimeofday(&tv_pre, &tz);
    LargeAgentMAPFInstanceDecomposition<N> decomposer(deserializer.getInstances(),
                                                                 deserializer.getAgents(),
                                                                 dim, is_occupied);
    gettimeofday(&tv_after, &tz);

    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
    std::cout << "finish decomposition in " << time_cost << "ms " << std::endl;
//    std::cout << "solution validation ? " << lacbs.solutionValidation() << std::endl;

    InstanceDecompositionVisualization(decomposer);
}


//LaCAM::LargeAgentConstraints<2, BlockAgent_2D>
template<Dimension N>
std::vector<std::string> loadInstanceAndCompareLayeredLAMAPF(const LA_MAPF_FUNC<N>& mapf_func,
                                                             const std::string& func_identifer,
                                                             const std::string& file_path,
                                                             double time_limit = 30,
                                                             bool path_constraint = false,
                                                             bool debug_mode = true) {
    InstanceDeserializer<N> deserializer;
    if(deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return {};
    }
    std::cout << " map scale ";
    for(size_t i=0; i<N-1; i++) {
        std::cout << dim[i] << "*";
    }
    std::cout << dim[N-1] << std::endl;

    // debug: print all agents and instance
    std::cout << "Instance: " << std::endl;
    std::vector<std::string> strs = deserializer.getTextString();
    for(const auto& str : strs) {
        std::cout << str << std::endl;
    }
    std::cout << std::endl;

//    LargeAgentMAPF_InstanceGenerator<2, AgentType> generator(deserializer.getAgents(), is_occupied, dim, 1e7);
//
//    std::cout << " solvable ?  " << !((generator.getConnectionBetweenNode(7, 89773, 108968)).empty()) << std::endl;

    std::vector<std::vector<int> > grid_visit_count_table_layered;

    LargeAgentMAPFInstanceDecompositionPtr<N> decomposer_ptr = nullptr;
    auto start_t = clock();
    auto layered_paths = layeredLargeAgentMAPF<N>(deserializer.getInstances(),
                                                             deserializer.getAgents(),
                                                             dim, is_occupied,
                                                             mapf_func, //CBS::LargeAgentCBS_func<2, AgentType >,
                                                             grid_visit_count_table_layered,
                                                             time_limit, decomposer_ptr,
                                                             path_constraint,
                                                             debug_mode);
    auto end_t = clock();
    double total_time_cost = ((double)end_t - start_t)/CLOCKS_PER_SEC;
    std::cout << "instance has " << deserializer.getAgents().size() << " agents, layered CBS find solution ? " << !layered_paths.empty()
              << " in " << total_time_cost << "s " << std::endl;

    // agents size / time cost / success / SOC / makespan / decom 1 time cost / decom 2 time cost / decom 3 time cost
    std::stringstream ss_layered;
    ss_layered << "LAYERED "  << func_identifer << " " << deserializer.getAgents().size() << " "
               << total_time_cost << " " << !layered_paths.empty() << " "
               << getSOC(layered_paths) << " " << getMakeSpan(layered_paths) << " "

               << decomposer_ptr->initialize_time_cost_ << " "
               << decomposer_ptr->instance_decomposition_time_cost_ << " "
               << decomposer_ptr->cluster_bipartition_time_cost_ << " "
               << decomposer_ptr->level_sorting_time_cost_;

    start_t = clock();
    std::vector<std::vector<int> > grid_visit_count_table_raw;
    auto raw_paths = mapf_func(deserializer.getInstances(),
                               deserializer.getAgents(),
                               dim, is_occupied,
                               nullptr,
                               grid_visit_count_table_raw,
                               time_limit,
                               {}, nullptr, {}, {}, {}, nullptr); // default null config for layered MAPF
    end_t = clock();
    total_time_cost = ((double)end_t - start_t)/CLOCKS_PER_SEC;
    std::cout << "instance has " << deserializer.getAgents().size() << " agents, raw CBS find solution ? " << !raw_paths.empty()
              << " in " << total_time_cost << "s " << std::endl;

    // agents size / time cost / success / SOC / makespan
    std::stringstream ss_raw;
    ss_raw << "RAW " << func_identifer << " " << deserializer.getAgents().size() << " "
           << total_time_cost << " " << !raw_paths.empty() << " "
           << getSOC(raw_paths) << " " << getMakeSpan(raw_paths);

    std::vector<std::string> retv;
    retv.push_back(ss_layered.str());
    retv.push_back(ss_raw.str());

    return retv;
}

// maximum_sample_count: max times of sample start and target for an agent
template<Dimension N>
std::vector<std::string> generateInstanceAndCompare(const std::vector<AgentPtr<N> >& agents,
                                                    const std::string& file_path,
                                                    const LA_MAPF_FUNC<N>& mapf_func,
                                                    const std::string& func_identifer,
                                                    double time_limit = 30,
                                                    bool path_constraint = false,
                                                    int maximum_sample_count = 1e7,
                                                    bool debug_mode = false) {
    generateInstance(agents, file_path, mapf_func, maximum_sample_count);
    auto retv = loadInstanceAndCompareLayeredLAMAPF<N>(mapf_func,
                                                                  func_identifer,
                                                                  file_path,
                                                                  time_limit,
                                                                  path_constraint,
                                                                  debug_mode);
    return retv;
}

void clearFile(const std::string& file_path) {
    std::ofstream os(file_path, std::ios::trunc);
    os.close();
}

void writeStrsToEndOfFile(const std::vector<std::string>& strs, const std::string& file_path) {
    std::ofstream os(file_path, std::ios::app);
    if(!os.is_open()) { return; }
    for(int i=0; i<strs.size(); i++) {
        os << strs[i] << "\n";
    }
    os.close();
}


#endif //LAYEREDMAPF_COMMON_INTERFACES_H
