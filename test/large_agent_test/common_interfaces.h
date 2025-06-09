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

//#include "../../freeNav-base/visualization/canvas/canvas.h"
#include "../../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../test_data.h"
#include "../../algorithm/LA-MAPF/CBS/layered_large_agent_CBS.h"

#include "../../algorithm/LA-MAPF/laryered_large_agent_mapf.h"
#include "../../freeNav-base/dependencies/memory_analysis.h"
#include "../../algorithm/LA-MAPF/IndependenceDetection/independence_detection.h"

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

auto map_test_config = MAPFTestConfig_AR0203SR;
// MAPFTestConfig_Paris_1_256 //  pass
// MAPFTestConfig_Berlin_1_256; // pass
// MAPFTestConfig_maze_32_32_4; // pass
// MAPFTestConfig_simple;
// MAPFTestConfig_AR0011SR; // pass
// MAPFTestConfig_empty_48_48 // pass
// MAPFTestConfig_maze_128_128_10
// MAPFTestConfig_ost003d
// MAPFTestConfig_simple_10_10

// MAPFTestConfig_Boston_2_256 // target overlap
// MAPFTestConfig_Sydney_2_256
// MAPFTestConfig_AR0044SR
// MAPFTestConfig_AR0203SR
// MAPFTestConfig_AR0072SR
// MAPFTestConfig_Denver_2_256



auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};

TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto dim = loader.getDimensionInfo();


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

    // LargeAgentMAPF_InstanceGenerator<2> generator(deserializer.getAgents(), is_occupied, dim);
    // InstanceVisualization(deserializer.getAgents(), generator.getAllPoses(), deserializer.getInstances(),
    //                                  method.getSolution()
    // );

//    InstanceVisualization<AgentType>(deserializer.getAgents(), generator.getAllPoses(), deserializer.getInstances(), {});

}


// //LaCAM::LargeAgentConstraints<2, BlockAgent_2D>
// template<Dimension N>
// void loadInstanceAndVisualize(const std::string& file_path) {
//     InstanceDeserializer<N> deserializer;
//     if(deserializer.loadInstanceFromFile(file_path, dim)) {
//         std::cout << "load from path " << file_path << " success" << std::endl;
//     } else {
//         std::cout << "load from path " << file_path << " failed" << std::endl;
//         return;
//     }
//     std::cout << " map scale " << dim[0] << "*" << dim[1] << std::endl;

//     // debug: print all agents and instance
//     std::cout << "Instance: " << std::endl;
//     std::vector<std::string> strs = deserializer.getTextString();
//     for(const auto& str : strs) {
//         std::cout << str << std::endl;
//     }
//     std::cout << std::endl;

//     LargeAgentMAPF_InstanceGenerator<N> generator(deserializer.getAgents(), is_occupied, dim);

//     InstanceVisualization(deserializer.getAgents(),
//                           generator.getAllPoses(),
//                           deserializer.getInstances(),
//                           {},
//                           {});

// }

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
    bool is_loss_of_solvability = false;
    auto layered_paths = layeredLargeAgentMAPF<N>(deserializer.getInstances(),
                                                             deserializer.getAgents(),
                                                             dim, is_occupied,
                                                             mapf_func, //CBS::LargeAgentCBS_func<2, AgentType >,
                                                             grid_visit_count_table,
                                                             is_loss_of_solvability,
                                                             time_limit, decomposer_ptr,
                                                             path_constraint,
                                                             0,
                                                             debug_mode);
    auto end_t = clock();
    double total_time_cost = ((double)end_t - start_t)/CLOCKS_PER_SEC;
    std::cout << "instance has " << deserializer.getAgents().size() << " agents, find solution ? " << !layered_paths.empty()
              << " in " << total_time_cost << "s " << std::endl;

    // if(visualize) {
    //     LargeAgentMAPF_InstanceGenerator<N> generator(deserializer.getAgents(), is_occupied, dim);

    //     InstanceVisualization(deserializer.getAgents(),
    //                                      generator.getAllPoses(),
    //                                      deserializer.getInstances(),
    //                                      layered_paths,
    //                                      grid_visit_count_table);
    // }
}


// maximum_sample_count: max times of sample start and target for an agent
template<Dimension N>
bool generateInstance(const std::vector<AgentPtr<N> >& agents,
                      const std::string& file_path,
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
        return false;
    }
    InstanceDeserializer<N> fake_deserializer;
    if(!fake_deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return false;
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
        return false;
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
        return true;
    } else {
        std::cout << "save to path " << file_path << " failed" << std::endl;
        return false;
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
    if(generateInstance(agents, file_path, maximum_sample_count)) {
        loadInstanceAndPlanningLayeredLAMAPF<N>(mapf_func, file_path, 5, false, debug_mode, visualize);
    }
}


// template<Dimension N>
// void InstanceDecompositionVisualization(const LargeAgentMAPFInstanceDecomposition<N>& decomposer) {
//     zoom_ratio = std::min(2560/dim[0], 1400/dim[1]);

//     // visualize instance
//     Canvas canvas("LargeAgentMAPF Decomposition", dim[0], dim[1], .1, zoom_ratio);

//     bool draw_related_agents_map = false;
//     bool draw_hyper_node_id = false;
//     bool draw_heuristic_table = false;
//     int total_index = getTotalIndexOfSpace<2>(dim);
//     std::vector<std::vector<std::string> > grid_strs(total_index);

//     Pointi<2> pt;
//     size_t node_id;
//     while(true) {
//         canvas.resetCanvas();
//         canvas.drawEmptyGrid();
//         canvas.drawGridMap(dim, is_occupied);

//         grid_strs = std::vector<std::vector<std::string> >(total_index);

//         if(draw_all_instance) {
//             const auto& instances = decomposer.instances_;
//             const auto& agents    = decomposer.agents_;
//             for (int current_subgraph_id=0; current_subgraph_id<instances.size(); current_subgraph_id++)
//             {
//                 const auto &instance = instances[current_subgraph_id];
//                 agents[current_subgraph_id]->drawOnCanvas(instance.first, canvas, COLOR_TABLE[current_subgraph_id%30]);
//                 canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], -orientToPi_2D(instance.first.orient_) ,
//                                     1, std::max(1,zoom_ratio/10));

//                 agents[current_subgraph_id]->drawOnCanvas(instance.second, canvas, COLOR_TABLE[current_subgraph_id%30]);
//                 canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], -orientToPi_2D(instance.second.orient_),
//                                     1, std::max(1,zoom_ratio/10));

//             }
//         }
//         if(draw_related_agents_map) {
// //            std::cout << " draw_related_agents_map " << std::endl;
//             const auto& hyper_graph = decomposer.connect_graphs_[current_subgraph_id];
//             for(int i=0; i<total_index; i++) {
//                 pt = IdToPointi<2>(i, dim);
//                 std::set<int> related_agents;
//                 for(int orient=0; orient<4; orient++) {
//                     node_id = i*4  + orient;
//                     for(const int& related_agent : hyper_graph.related_agents_map_[node_id]) {
//                         related_agents.insert(related_agent);
//                     }
//                 }
//                 const auto& subgraph = decomposer.agent_sub_graphs_[current_subgraph_id];
//                 if(subgraph.all_nodes_[node_id] == nullptr) { continue; }
//                 std::stringstream ss;
//                 ss << "r: ";
//                 for(const int& related_agent : related_agents) {
//                     ss << related_agent << " ";
//                 }
//                 grid_strs[i].push_back(ss.str());
//             }
//         }
//         if(draw_hyper_node_id) {
//             const auto& hyper_graph = decomposer.connect_graphs_[current_subgraph_id];
//             for(int i=0; i<total_index; i++) {
//                 pt = IdToPointi<2>(i, dim);
//                 std::set<int> hyper_nodes; // a point have four direction, may be more than one
//                 for(int orient=0; orient<4; orient++) {
//                     node_id = i*4  + orient;
//                     if(hyper_graph.hyper_node_id_map_[node_id] != MAX<size_t>) {
//                         hyper_nodes.insert(hyper_graph.hyper_node_id_map_[node_id]);
//                     }
//                 }
//                 const auto& subgraph = decomposer.agent_sub_graphs_[current_subgraph_id];
//                 if(subgraph.all_nodes_[node_id] == nullptr) { continue; }
//                 std::stringstream ss;
//                 ss << "n: ";
//                 for(const int& related_agent : hyper_nodes) {
//                     ss << related_agent << " ";
//                 }
//                 grid_strs[i].push_back(ss.str());
//             }
//         }
//         if(draw_heuristic_table) {
//             const auto& hyper_graph = decomposer.connect_graphs_[current_subgraph_id];
//             const auto& heuristic_table = decomposer.heuristic_tables_[current_subgraph_id];
//             const auto& subgraph = decomposer.agent_sub_graphs_[current_subgraph_id];

//             for(int i=0; i<total_index; i++) {
//                 pt = IdToPointi<2>(i, dim);
//                 std::set<int> heuristic_values; // a point have four direction, may be more than one
//                 for(int orient=0; orient<4; orient++) {
//                     node_id = i*4  + orient;
//                     if(subgraph.all_nodes_[node_id] == nullptr) { continue; }
//                     if(hyper_graph.hyper_node_id_map_[node_id] == MAX<size_t>) { continue; }
//                     if(heuristic_table[hyper_graph.hyper_node_id_map_[node_id]] != MAX<int>) {
//                         heuristic_values.insert(heuristic_table[hyper_graph.hyper_node_id_map_[node_id]]);
//                     }
//                 }
//                 if(heuristic_values.empty()) { continue; }
//                 std::stringstream ss;
//                 ss << "h: ";
//                 for(const int& value : heuristic_values) {
//                     ss << value << " ";
//                 }
//                 grid_strs[i].push_back(ss.str());
//             }
//         }

//         for(int i=0; i<total_index; i++) {
//             pt = IdToPointi<2>(i, dim);
//             canvas.drawMultiTextInt(pt[0], pt[1], grid_strs[i], cv::Vec3b::all(0), 1.0);
//         }

//         char key = canvas.show(100);
//         switch (key) {
//             case 'i':
//                 draw_all_instance = !draw_all_instance;
//                 break;
//             case 'r':
//                 draw_related_agents_map = !draw_related_agents_map;
//                 break;
//             case 'n':
//                 draw_hyper_node_id = !draw_hyper_node_id;
//                 break;
//             case 'h':
//                 draw_heuristic_table = !draw_heuristic_table;
//                 break;
//             case 'w':
//                 current_subgraph_id = current_subgraph_id+1;
//                 current_subgraph_id = current_subgraph_id % decomposer.agents_.size();
//                 break;
//             case 's':
//                 current_subgraph_id = current_subgraph_id + decomposer.agents_.size() - 1;
//                 current_subgraph_id = current_subgraph_id % decomposer.agents_.size();
//                 break;
//             default:
//                 break;
//         }
//     }
// }

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

    // InstanceDecompositionVisualization(decomposer);
}


template<Dimension N>
void generateInstanceAndDecomposition(const std::vector<AgentPtr<N> >& agents,
                                      const std::string& file_path,
                                      const LA_MAPF_FUNC<N>& mapf_func,
                                      int maximum_sample_count = 1e7,
                                      bool debug_mode = true,
                                      bool visualize = false) {
    if(generateInstance(agents, file_path, maximum_sample_count)) {
        loadInstanceAndDecomposition<N>(file_path);
    }
}



//LaCAM::LargeAgentConstraints<2, BlockAgent_2D>
template<Dimension N>
std::vector<std::string> loadInstanceAndCompareLayeredLAMAPF(const LA_MAPF_FUNC<N>& mapf_func,
                                                             const std::string& func_identifer,
                                                             const std::string& file_path,
                                                             double time_limit = 30,
                                                             bool path_constraint = false,
                                                             int level_of_decomposition = 4,
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
    return LayeredLAMAPFCompare(deserializer.getInstances(),
                                deserializer.getAgents(),
                                mapf_func,
                                func_identifer,
                                time_limit,
                                path_constraint,
                                level_of_decomposition,
                                debug_mode);
}

template<Dimension N>
std::vector<std::string> LayeredLAMAPFCompare(const InstanceOrients<N>& instances,
                                              const AgentPtrs<N>& agents,
                                              const LA_MAPF_FUNC<N>& mapf_func,
                                              const std::string& func_identifer,
                                              double time_limit = 30,
                                              bool path_constraint = false,
                                              int level_of_decomposition = 4,
                                              bool debug_mode = true) {
    //    LargeAgentMAPF_InstanceGenerator<2, AgentType> generator(deserializer.getAgents(), is_occupied, dim, 1e7);
//
//    std::cout << " solvable ?  " << !((generator.getConnectionBetweenNode(7, 89773, 108968)).empty()) << std::endl;

    std::vector<std::vector<int> > grid_visit_count_table_layered;

    LargeAgentMAPFInstanceDecompositionPtr<N> decomposer_ptr = nullptr;
    memory_recorder.clear();
    sleep(1);
    float base_usage = memory_recorder.getCurrentMemoryUsage();
    auto start_t = clock();
    bool is_loss_of_solvability = false;
    auto layered_paths = layeredLargeAgentMAPF<N>(instances,
                                                  agents,
                                                  dim, is_occupied,
                                                  mapf_func, //CBS::LargeAgentCBS_func<2, AgentType >,
                                                  grid_visit_count_table_layered,
                                                  is_loss_of_solvability,
                                                  time_limit, decomposer_ptr,
                                                  path_constraint,
                                                  level_of_decomposition,
                                                  debug_mode);
    auto end_t = clock();
    sleep(1);
    float peak_usage = memory_recorder.getMaximalMemoryUsage();
    float memory_usage = peak_usage - base_usage;

    double total_time_cost = ((double)end_t - start_t)/CLOCKS_PER_SEC;
    std::cout << "instance has " << agents.size() << " agents, layered " << func_identifer << " find solution ? " << !layered_paths.empty()
              << " in " << total_time_cost << "s " << std::endl;

    std::cout << "Layered: max subproblem / total = " << decomposer_ptr->getMaxSubProblemSize() << " / " << instances.size() << std::endl;
    std::cout << "Layered: num of subproblem = " << decomposer_ptr->getNumberOfSubProblem() << std::endl;

    // agents size / time cost / success / SOC / makespan / success / memory usage / init time cost / decom time cost / max subproblem / num of subproblems
    std::stringstream ss_layered;
    ss_layered << "LAYERED_"  << func_identifer << " " << agents.size() << " "
               << total_time_cost << " "
               << getSOC(layered_paths) << " " << getMakeSpan(layered_paths) << " "
               << !layered_paths.empty() << " " << memory_usage << " "
               << decomposer_ptr->subgraph_and_heuristic_time_cost_ << " "

               << decomposer_ptr->initialize_time_cost_  +
                  decomposer_ptr->instance_decomposition_time_cost_ +
                  decomposer_ptr->cluster_bipartition_time_cost_ +
                  decomposer_ptr->level_sorting_time_cost_ +
                  decomposer_ptr->level_bipartition_time_cost_ << " "

               << decomposer_ptr->getMaxSubProblemSize() << " "
               << decomposer_ptr->getNumberOfSubProblem() << " "
               << is_loss_of_solvability;

    memory_recorder.clear();
    sleep(1);
    base_usage = memory_recorder.getCurrentMemoryUsage();
    start_t = clock();
    std::vector<std::vector<int> > grid_visit_count_table_raw;
    auto raw_paths = mapf_func(instances,
                               agents,
                               dim, is_occupied,
                               nullptr,
                               grid_visit_count_table_raw,
                               time_limit,
                               {}, nullptr, {}, {}, {}, nullptr); // default null config for layered MAPF
    end_t = clock();
    sleep(1);
    peak_usage = memory_recorder.getMaximalMemoryUsage();
    memory_usage = peak_usage - base_usage;

    total_time_cost = ((double)end_t - start_t)/CLOCKS_PER_SEC;
    std::cout << "instance has " << agents.size() << " agents, raw " << func_identifer << " find solution ? " << !raw_paths.empty()
              << " in " << total_time_cost << "s " << std::endl;

    // agents size / time cost / success / SOC / makespan / success / memory usage
    std::stringstream ss_raw;
    ss_raw << "RAW_" << func_identifer << " " << agents.size() << " "
           << total_time_cost << " "
           << getSOC(raw_paths) << " " << getMakeSpan(raw_paths) << " "
           << !raw_paths.empty() << " " << memory_usage;

    // Independence Detection
    std::stringstream ss_id;
    if(func_identifer == "CBS") {
        memory_recorder.clear();
        sleep(1);
        base_usage = memory_recorder.getCurrentMemoryUsage();
        start_t = clock();
        auto id_solver = ID::ID<N>(instances, agents,
                                   dim, is_occupied, mapf_func, time_limit * 1e3);
        LAMAPF_Paths id_paths;
        if (id_solver.solve()) {
            id_paths = id_solver.getSolution();
        }
        end_t = clock();
        if (!id_paths.empty()) {
            std::cout << "ID: max subproblem / total = " << id_solver.getMaximalSubProblem() << " / "
                      << instances.size() << std::endl;
            std::cout << "ID: num of subproblem = " << id_solver.getNumberOfSubProblem() << std::endl;
            std::cout << "ID: is solution valid ? " << isSolutionValid<2>(id_paths, agents, id_solver.all_poses_)
                      << std::endl;
        }
        sleep(1);
        peak_usage = memory_recorder.getMaximalMemoryUsage();
        memory_usage = peak_usage - base_usage;
        total_time_cost = ((double) end_t - start_t) / CLOCKS_PER_SEC;

        // agents size / time cost / success / SOC / makespan / success / memory usage / max subproblem / num of subproblems
        ss_id << "ID_" << func_identifer << " " << agents.size() << " "
              << total_time_cost << " "
              << getSOC(id_paths) << " " << getMakeSpan(id_paths) << " "
              << !id_paths.empty() << " " << memory_usage << " "

              << id_solver.getMaximalSubProblem() << " "
              << id_solver.getNumberOfSubProblem();
    }
    std::vector<std::string> retv;
    retv.push_back(ss_layered.str());
    retv.push_back(ss_raw.str());
    if(func_identifer == "CBS") {
        retv.push_back(ss_id.str());
    }
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
    if(generateInstance(agents, file_path, maximum_sample_count)) {
        auto retv = loadInstanceAndCompareLayeredLAMAPF<N>(mapf_func,
                                                           func_identifer,
                                                           file_path,
                                                           time_limit,
                                                           path_constraint,
                                                           debug_mode);
        return retv;
    } else {
        return {};
    }
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


template<Dimension N>
bool decompositionOfSingleInstance(const InstanceOrients<N> & instances,
                                   const std::vector<AgentPtr<N> >& agents,
                                   DimensionLength* dim,
                                   const IS_OCCUPIED_FUNC<N> & isoc,
                                   OutputStream& outputStream, int level=4) {

    struct timezone tz;
    struct timeval tv_pre;
    struct timeval tv_after;

    memory_recorder.clear();
    sleep(1);
    float basic_usage = memory_recorder.getMaximalMemoryUsage();
    gettimeofday(&tv_pre, &tz);

    auto instance_decompose = std::make_shared<LargeAgentMAPFInstanceDecomposition<N> >(instances,
                                                                                        agents,
                                                                                        dim,
                                                                                        is_occupied,
                                                                                        true,
                                                                                        level,
                                                                                        true);

    gettimeofday(&tv_after, &tz);
    sleep(1);
    float peak_usage = memory_recorder.getMaximalMemoryUsage();
    float memory_usage = peak_usage - basic_usage;
    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
    bool is_legal = true;//instance_decompose->decompositionValidCheck(instance_decompose->all_clusters_);

    int max_cluster_size = 0, total_count = 0;
    for(int i=0; i<instance_decompose->all_clusters_.size(); i++) {
        total_count += instance_decompose->all_clusters_[i].size();
        if(instance_decompose->all_clusters_[i].size() > max_cluster_size) {
            max_cluster_size = instance_decompose->all_clusters_[i].size();
        }
        //if(all_clusters_[i].size() == 1) { continue; }
        //std::cout << "-- clusters " << i << " size " << all_clusters_[i].size() << ": " << all_clusters_[i] << std::endl;
    }
    assert(total_count == instances.size());
    outputStream.clear();
    std::stringstream ss;
    ss << time_cost << " "
       << max_cluster_size << " "
       << instances.size() << " "
       << is_legal << " " << level << " "
       << memory_usage << " "
       << instance_decompose->all_clusters_.size() << " "
       << instance_decompose->initialize_time_cost_ << " "
       << instance_decompose->instance_decomposition_time_cost_ << " "
       << instance_decompose->cluster_bipartition_time_cost_ << " "
       << instance_decompose->level_sorting_time_cost_ << " "
       << instance_decompose->level_bipartition_time_cost_;

    outputStream = ss.str();
    std::cout << " memory_usage = " << memory_usage << std::endl;
    return is_legal;
}

// test_count: the total count of start and target pair in the scenario file
// required_count: required
std::vector<std::set<int> > pickCasesFromScene(int test_count,
                                               const std::vector<int>& required_counts,
                                               int instance_count) {
    std::vector<std::set<int> > retv;
    for(int i=0; i<instance_count; i++) {
        for(const int& required_count : required_counts) {
            std::set<int> instance;
            while(1) {
                int current_pick = rand() % test_count;
                if(instance.find(current_pick) == instance.end()) {
                    instance.insert(current_pick);
                    if(instance.size() == required_count) {
                        retv.push_back(instance);
                        break;
                    }
                }
            }
        }
    }
    return retv;
}


#endif //LAYEREDMAPF_COMMON_INTERFACES_H
