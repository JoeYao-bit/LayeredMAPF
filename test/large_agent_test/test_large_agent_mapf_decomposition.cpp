//
// Created by yaozhuo on 2024/8/10.
//

#include "../../algorithm/LA-MAPF/large_agent_instance_decomposition.h"

#include <gtest/gtest.h>

#include "common_interfaces.h"


using namespace freeNav::LayeredMAPF::LA_MAPF;


template<typename AgentType>
void InstanceDecompositionVisualization(const LargeAgentMAPFInstanceDecomposition<2, AgentType>& decomposer) {
    zoom_ratio = std::min(2560/dim[0], 1400/dim[1]);

    // visualize instance
    Canvas canvas("LargeAgentMAPF Decomposition", dim[0], dim[1], .1, zoom_ratio);

    bool draw_related_agents_map = false;
    bool draw_hyper_node_id = false;
    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied);

        if(draw_all_instance) {
            const auto& instances = decomposer.instances_;
            const auto& agents    = decomposer.agents_;
            for (int current_subgraph_id=0; current_subgraph_id<instances.size(); current_subgraph_id++)
            {
                const auto &instance = instances[current_subgraph_id];
                agents[current_subgraph_id].drawOnCanvas(instance.first, canvas, COLOR_TABLE[current_subgraph_id%30]);
                canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], -orientToPi_2D(instance.first.orient_) ,
                                    1, std::max(1,zoom_ratio/10));

                agents[current_subgraph_id].drawOnCanvas(instance.second, canvas, COLOR_TABLE[current_subgraph_id%30]);
                canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], -orientToPi_2D(instance.second.orient_),
                                    1, std::max(1,zoom_ratio/10));

            }
        }
        if(draw_related_agents_map) {
//            std::cout << " draw_related_agents_map " << std::endl;
            int total_index = getTotalIndexOfSpace<2>(dim);
            Pointi<2> pt;
            size_t node_id;
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
                canvas.drawTextInt(pt[0], pt[1], ss.str().c_str(), cv::Vec3b::all(0), 1.0);
            }
        }
        if(draw_hyper_node_id) {
            int total_index = getTotalIndexOfSpace<2>(dim);
            Pointi<2> pt;
            size_t node_id;
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
                ss << "h: ";
                for(const int& related_agent : hyper_nodes) {
                    ss << related_agent << " ";
                }
                canvas.drawTextInt(pt[0], pt[1], ss.str().c_str(), cv::Vec3b::all(0), 1.0);
            }
        }
        char key = canvas.show(100);
        switch (key) {
            case 'i':
                draw_all_instance = !draw_all_instance;
                break;
            case 'r':
                draw_related_agents_map = !draw_related_agents_map;
                break;
            case 'h':
                draw_hyper_node_id = !draw_hyper_node_id;
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

template<typename AgentType>
void loadInstanceAndDecomposition(const std::string& file_path) {
    InstanceDeserializer<2, AgentType> deserializer;
    if (deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << "map scale = " << dim[0] << "*" << dim[1] << std::endl;
    gettimeofday(&tv_pre, &tz);
    LargeAgentMAPFInstanceDecomposition<2, AgentType> decomposer(deserializer.getInstances(),
                                                                 deserializer.getAgents(),
                                                                 dim, is_occupied);
    InstanceDecompositionVisualization<AgentType>(decomposer);
}


TEST(basic_test, LA_MAPF_decomposition) {
    const std::string file_path = map_test_config.at("crc_ins_path");
    loadInstanceAndDecomposition<CircleAgent<2> >(file_path);
}
