//
// Created by yaozhuo on 2024/6/28.
//

#ifndef LAYEREDLAMAPF_COMMON_INTERFACES_H
#define LAYEREDLAMAPF_COMMON_INTERFACES_H
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
#include "../../algorithm/precomputation_for_decomposition.h"

//#include "../../freeNav-base/visualization/canvas/canvas.h"
#include "../../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../test_data.h"
#include "../../algorithm/LA-MAPF/CBS/layered_large_agent_CBS.h"

#include "../../algorithm/LA-MAPF/laryered_large_agent_mapf.h"
#include "../../freeNav-base/dependencies/memory_analysis.h"

#include "../../algorithm/LA-MAPF/IndependenceDetection/independence_detection.h"

#include "../../algorithm/break_loop_decomposition/biparition_decomposition.h"
#include "../../algorithm/break_loop_decomposition/break_loop_decomposition.h"


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

//
//TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);
//
//auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };
//
//auto dim = loader.getDimensionInfo();



// maximum_sample_count: max times of sample start and target for an agent
template<Dimension N>
bool generateLargeAgentMAPFInstance(const std::vector<AgentPtr<N> >& agents,
                                    const SingleMapTestConfig<N>& file_path,
                                    int maximum_sample_count = 1e7) {
    gettimeofday(&tv_pre, &tz);


    TextMapLoader tl(file_path.at("map_path"), is_char_occupied1);
    auto dim = tl.getDimensionInfo();
    auto is_occupied = [&tl](const freeNav::Pointi<2> &pt) -> bool { return tl.isOccupied(pt); };
    IS_OCCUPIED_FUNC<2> is_occupied_func_local = is_occupied;

    // get previous texts as backup
    InstanceDeserializer<N> deserializer;
    deserializer.loadInstanceFromFile(file_path, dim);
    std::vector<std::string> backup_strs = deserializer.getTextString();


    // test whether serialize and deserialize will change agent's behavior
    // even we didn't change their behavior explicitly
    InstanceOrients<N> fake_instances;
    for(int i=0; i<agents.size(); i++) {
        fake_instances.push_back({Pose<int, N>{{0,0},0}, Pose<int, N>{{0,0},0}});
    }
    InstanceSerializer<N> fake_serializer(agents, fake_instances);
    if(!fake_serializer.saveToFile(file_path)) {
        std::cout << "fake_serializer save to path " << file_path.at("la_ins_path") << " failed" << std::endl;
        return false;
    }
    InstanceDeserializer<N> fake_deserializer;
    if(!fake_deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path.at("la_ins_path") << " failed" << std::endl;
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
        std::cout << "save to path " << file_path.at("la_ins_path") << " success" << std::endl;
        return true;
    } else {
        std::cout << "save to path " << file_path.at("la_ins_path") << " failed" << std::endl;
        return false;
    }
}

template<Dimension N>
void loadLargeAgentInstanceAndDecomposition(const SingleMapTestConfig<N>& file_path) {

    TextMapLoader tl(file_path.at("map_path"), is_char_occupied1);
    std::cout << "start SingleMapTest from map " << file_path.at("map_path") << std::endl;
    auto dim = tl.getDimensionInfo();
    auto is_occupied = [&tl](const freeNav::Pointi<2> &pt) -> bool { return tl.isOccupied(pt); };
    IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

    InstanceDeserializer<N> deserializer;
    if (deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path.at("la_ins_path") << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path.at("la_ins_path") << " failed" << std::endl;
        return;
    }
    std::cout << " map scale ";
    for(int i=0; i<N-1; i++) {
        std::cout << dim[i] << "*";
    }
    std::cout << dim[N-1] << std::endl;

    gettimeofday(&tv_pre, &tz);
    LargeAgentMAPFInstanceDecomposition<N, HyperGraphNodeDataRaw<N>, Pose<int, N>> decomposer(deserializer.getInstances(),
                                                                                       deserializer.getAgents(),
                                                                                       dim, is_occupied);
    gettimeofday(&tv_after, &tz);

    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
    std::cout << "finish decomposition in " << time_cost << "ms " << std::endl;
//    std::cout << "solution validation ? " << lacbs.solutionValidation() << std::endl;

    // InstanceDecompositionVisualization(decomposer);
}


template<Dimension N>
void generateLargeAgentInstanceAndDecomposition(const std::vector<AgentPtr<N> >& agents,
                                      const std::string& file_path,
                                      const LA_MAPF_FUNC<N, Pose<int, N>>& mapf_func,
                                      int maximum_sample_count = 1e7,
                                      bool debug_mode = true,
                                      bool visualize = false) {
    if(generateLargeAgentMAPFInstance(agents, file_path, maximum_sample_count)) {
        loadLargeAgentInstanceAndDecomposition<N, Pose<int, N>>(file_path);
    }
}



//LaCAM::LargeAgentConstraints<2, BlockAgent_2D>
template<Dimension N>
std::vector<std::string> loadLargeAgentInstanceAndCompareLayeredLAMAPF(const LA_MAPF_FUNC<N, Pose<int, N>>& mapf_func,
                                                                       const std::string& func_identifer,
                                                                       const SingleMapTestConfig<N>& file_path,
                                                                       double time_limit = 30,
                                                                       bool path_constraint = false,
                                                                       int level_of_decomposition = 4,
                                                                       bool debug_mode = true) {


    TextMapLoader tl(file_path.at("map_path"), is_char_occupied1);
    std::cout << "start SingleMapTest from map " << map_test_config.at("map_path") << std::endl;
    auto dim = tl.getDimensionInfo();
    auto is_occupied = [&tl](const freeNav::Pointi<2> &pt) -> bool { return tl.isOccupied(pt); };
    IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

    InstanceDeserializer<2> deserializer;

    if(deserializer.loadInstanceFromFile(file_path.at("la_ins_path"), dim)) {
        std::cout << "load from path " << file_path.at("la_ins_path") << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path.at("la_ins_path") << " failed" << std::endl;
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

// maximum_sample_count: max times of sample start and target for an agent
template<Dimension N>
std::vector<std::string> generateLargeAgentInstanceAndCompare(const std::vector<AgentPtr<N> >& agents,
                                                              const std::string& file_path,
                                                              const LA_MAPF_FUNC<N, Pose<int, N>>& mapf_func,
                                                              const std::string& func_identifer,
                                                              double time_limit = 30,
                                                              bool path_constraint = false,
                                                              int maximum_sample_count = 1e7,
                                                              bool debug_mode = false) {
    if(generateLargeAgentMAPFInstance(agents, file_path, maximum_sample_count)) {
        auto retv = loadLargeAgentInstanceAndCompareLayeredLAMAPF<N>(mapf_func,
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


template<Dimension N>
std::string BIPARTITION_LAMAPF(const std::vector<std::pair<Pose<int, N>, Pose<int, N>>>& instances,
                                       const AgentPtrs<N>& agents,
                                       DimensionLength* dim,
                                       const IS_OCCUPIED_FUNC<N>& isoc,
                                       const LA_MAPF_FUNC<N, Pose<int, N>>& mapf_func,
                                       const std::string& func_identifer,
                                       double time_limit) {
    memory_recorder.clear();
    sleep(1);
    double basic_usage = memory_recorder.getCurrentMemoryUsage();
    MSTimer mst;
    auto pre_dec =
            std::make_shared<PrecomputationOfLAMAPFDecomposition<N, HyperGraphNodeDataRaw<N>>>(
                    instances,
                    agents,
                    dim, isoc, true);

    auto bi_decompose = std::make_shared<MAPFInstanceDecompositionBipartition<N, HyperGraphNodeDataRaw<N>, Pose<int, N>>>(
            dim,
            pre_dec->connect_graphs_,
            pre_dec->agent_sub_graphs_,
            pre_dec->heuristic_tables_sat_,
            pre_dec->heuristic_tables_,
            time_limit);

    LAMAPF_Paths layered_paths;
    bool detect_loss_solvability;
    std::vector<std::vector<int> > grid_visit_count_table;
    layered_paths = layeredLargeAgentMAPF<N, Pose<int, N>>(bi_decompose->all_levels_,
                                                           mapf_func, //
                                                           grid_visit_count_table,
                                                           detect_loss_solvability,
                                                           pre_dec,
                                                           time_limit - mst.elapsed()/1e3,
                                                           false);

    double total_time_cost = mst.elapsed()/1e3;

    std::cout << "instance has " << agents.size() << " agents, bipartition " << func_identifer << " find solution ? " << !layered_paths.empty()
              << " in " << total_time_cost << "s " << std::endl;

    std::cout << "bipartition: max subproblem / total = " << getMaxLevelSize(bi_decompose->all_levels_) << " / " << instances.size() << std::endl;
    std::cout << "bipartition: num of subproblem = " << bi_decompose->all_levels_.size() << std::endl;

    double max_usage = memory_recorder.getCurrentMemoryUsage();

    // agents size / time cost / success / SOC / makespan / success / memory usage / init time cost / decom time cost / max subproblem / num of subproblems
    std::stringstream ss_layered;
    ss_layered << "BP_" << func_identifer << " " << agents.size() << " "
               << total_time_cost << " "
               << getSOC(layered_paths) << " " << getMakeSpan(layered_paths) << " "
               << !layered_paths.empty() << " " << max_usage - basic_usage << " "
               << 0 << " "
               << 0 << " "
               << getMaxLevelSize(bi_decompose->all_levels_) << " "
               << bi_decompose->all_levels_.size() << " "
               << detect_loss_solvability;

    return ss_layered.str();
}

template<Dimension N>
std::string BREAKLOOP_LAMAPF(const std::vector<std::pair<Pose<int, N>, Pose<int, N>>>& instances,
                          const AgentPtrs<N>& agents,
                          DimensionLength* dim,
                          const IS_OCCUPIED_FUNC<N>& isoc,
                          const LA_MAPF_FUNC<N, Pose<int, N>>& mapf_func,
                          const std::string& func_identifer,
                          double time_limit) {
    memory_recorder.clear();
    sleep(1);
    double basic_usage = memory_recorder.getCurrentMemoryUsage();
    MSTimer mst;
    auto pre_dec =
            std::make_shared<PrecomputationOfLAMAPFDecomposition<N, HyperGraphNodeDataRaw<N>>>(
                    instances,
                    agents,
                    dim, isoc, false);

    auto bi_decompose = std::make_shared<MAPFInstanceDecompositionBreakLoop<N, HyperGraphNodeDataRaw<N>, Pose<int, N>>>(
            dim,
            pre_dec->connect_graphs_,
            pre_dec->agent_sub_graphs_,
            pre_dec->heuristic_tables_sat_,
            pre_dec->heuristic_tables_,
            time_limit);

    LAMAPF_Paths layered_paths;
    bool detect_loss_solvability;
    std::vector<std::vector<int> > grid_visit_count_table;
    layered_paths = layeredLargeAgentMAPF<N, Pose<int, N>>(bi_decompose->all_levels_,
                                                           mapf_func, //
                                                           grid_visit_count_table,
                                                           detect_loss_solvability,
                                                           pre_dec,
                                                           time_limit - mst.elapsed()/1e3,
                                                           false);

    double total_time_cost = mst.elapsed()/1e3;

    std::cout << "instance has " << agents.size() << " agents, breakloop " << func_identifer << " find solution ? " << !layered_paths.empty()
              << " in " << total_time_cost << "s " << std::endl;

    std::cout << "breakloop: max subproblem / total = " << getMaxLevelSize(bi_decompose->all_levels_) << " / " << instances.size() << std::endl;
    std::cout << "breakloop: num of subproblem = " << bi_decompose->all_levels_.size() << std::endl;

    double max_usage = memory_recorder.getCurrentMemoryUsage();

    // agents size / time cost / success / SOC / makespan / success / memory usage / init time cost / decom time cost / max subproblem / num of subproblems
    std::stringstream ss_layered;
    ss_layered << "BL_" << func_identifer << " " << agents.size() << " "
               << total_time_cost << " "
               << getSOC(layered_paths) << " " << getMakeSpan(layered_paths) << " "
               << !layered_paths.empty() << " " << max_usage - basic_usage << " "
               << 0 << " "
               << 0 << " "
               << getMaxLevelSize(bi_decompose->all_levels_) << " "
               << bi_decompose->all_levels_.size() << " "
               << detect_loss_solvability;

    return ss_layered.str();
}

template<Dimension N>
std::string ID_LAMAPF(const std::vector<std::pair<Pose<int, N>, Pose<int, N>>>& instances,
                                  const AgentPtrs<N>& agents,
                                  DimensionLength* dim,
                                  const IS_OCCUPIED_FUNC<N>& isoc,
                                  const LA_MAPF_FUNC<N, Pose<int, N>>& mapf_func,
                                  const std::string& func_identifer,
                                  double time_limit) {
    memory_recorder.clear();
    sleep(1);
    double basic_usage = memory_recorder.getCurrentMemoryUsage();
    MSTimer mst;
    auto pre =
            std::make_shared<PrecomputationOfLAMAPF<2>>(instances,
                                                        agents,
                                                        dim, isoc);

    auto id_solver = ID::ID<N, Pose<int, N>>(mapf_func, pre, (time_limit - mst.elapsed()/1e3));
    LAMAPF_Paths id_paths;
    if (id_solver.solve()) {
        id_paths = id_solver.getSolution();
    }
    double total_time_cost = mst.elapsed()/1e3;

    if (!id_paths.empty()) {
        std::cout << "ID: max subproblem / total = " << id_solver.getMaximalSubProblem() << " / "
                  << instances.size() << std::endl;
        std::cout << "ID: num of subproblem = " << id_solver.getNumberOfSubProblem() << std::endl;
        std::cout << "ID: is solution valid ? " << isSolutionValid<2>(id_paths, agents, id_solver.pre_->all_poses_)
                  << std::endl;
    } else {
        std::cout << "ID: failed when there are " << instances.size() << " agents" << std::endl;
    }

    double max_usage = memory_recorder.getCurrentMemoryUsage();

    // agents size / time cost / success / SOC / makespan / success / memory usage / init time cost / decom time cost / max subproblem / num of subproblems
    std::stringstream ss_id;
    // agents size / time cost / success / SOC / makespan / success / memory usage / max subproblem / num of subproblems
    ss_id << "ID_" << func_identifer << " " << agents.size() << " "
          << total_time_cost << " "
          << getSOC(id_paths) << " " << getMakeSpan(id_paths) << " "
          << !id_paths.empty() << " " << max_usage - basic_usage << " "

          << id_solver.getMaximalSubProblem() << " "
          << id_solver.getNumberOfSubProblem();

    return ss_id.str();
}

template<Dimension N>
std::string RAW_LAMAPF(const std::vector<std::pair<Pose<int, N>, Pose<int, N>>>& instances,
                      const AgentPtrs<N>& agents,
                      DimensionLength* dim,
                      const IS_OCCUPIED_FUNC<N>& isoc,
                      const LA_MAPF_FUNC<N, Pose<int, N>>& mapf_func,
                      const std::string& func_identifer,
                      double time_limit) {

    memory_recorder.clear();
    sleep(1);
    double basic_usage = memory_recorder.getCurrentMemoryUsage();
    MSTimer mst;
    auto pre =
            std::make_shared<PrecomputationOfLAMAPF<2>>(instances,
                                                        agents,
                                                        dim, isoc);
    std::vector<std::vector<int> > grid_visit_count_table;

    auto raw_paths = mapf_func(instances,
                               agents,
                               dim, isoc,
                               nullptr,
                               grid_visit_count_table,
                               time_limit,
                               pre->instance_node_ids_,
                               pre->all_poses_,
                               pre->distance_map_updater_,
                               pre->agent_sub_graphs_,
                               pre->agents_heuristic_tables_,
                               pre->agents_heuristic_tables_ignore_rotate_,
                               nullptr); // default null config for layered MAPF

    double total_time_cost = mst.elapsed()/1e3;

    double max_usage = memory_recorder.getCurrentMemoryUsage();

    // agents size / time cost / success / SOC / makespan / success / memory usage / init time cost / decom time cost / max subproblem / num of subproblems
    std::cout << "instance has " << agents.size() << " agents, raw " << func_identifer << " find solution ? " << !raw_paths.empty()
              << " in " << total_time_cost << "s " << std::endl;

    // agents size / time cost / success / SOC / makespan / success / memory usage
    std::stringstream ss_raw;
    ss_raw << "RAW_" << func_identifer << " " << agents.size() << " "
           << total_time_cost << " "
           << getSOC(raw_paths) << " " << getMakeSpan(raw_paths) << " "
           << !raw_paths.empty() << " " << max_usage - basic_usage;

    return ss_raw.str();
}

#endif //LAYEREDMAPF_COMMON_INTERFACES_H
