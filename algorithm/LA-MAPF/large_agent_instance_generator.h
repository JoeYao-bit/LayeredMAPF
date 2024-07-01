//
// Created by yaozhuo on 2024/6/26.
//

#ifndef LAYEREDMAPF_LARGE_AGENT_INSTANCE_GENERATOR_H
#define LAYEREDMAPF_LARGE_AGENT_INSTANCE_GENERATOR_H

#include <random>
#include "common.h"
#include "block_shaped_agent.h"
#include "circle_shaped_agent.h"
#include "large_agent_mapf.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    template<typename T>
    T getValueWithResolution(const T& min_val, const T& max_val, const T& resolution) {
        int count = ceil((max_val - min_val)/resolution) + 1;
//        std::cout << "count = " << count << std::endl;
        std::random_device rd;
        T value = min_val + resolution * (rd() % count);
        if(value < min_val) { value = min_val; }
        if(value > max_val) { value = max_val; }
        return value;
    }

    // generator random agents and random start and target
    // interval: minimum resolution of sample
    template<Dimension N>
    CircleAgents<N> RandomCircleAgentsGenerator(const int& num_of_agents,
                                                const float& min_radius,
                                                const float& max_radius,
                                                const float& resolution,
                                                DimensionLength* dim) {
        assert(min_radius <= max_radius);

        CircleAgents<N> agents;


        float radius;
        for(int i=0; i<num_of_agents; i++) {
            radius = getValueWithResolution<float>(min_radius, max_radius, resolution);
            agents.push_back(CircleAgent<N>(radius, i, dim));
        }
        agents.shrink_to_fit();
        return agents;
    }

    BlockAgents_2D RandomBlock2DAgentsGenerator(const int& num_of_agents,
                                                const float& min_min_x,
                                                const float& max_min_x,
                                                const float& min_max_x,
                                                const float& max_max_x,
                                                const float& min_width,
                                                const float& max_width,
                                                const float& resolution,
                                                DimensionLength* dim) {
        assert(min_min_x <= 0 && max_min_x <= 0);
        assert(min_max_x >= 0 && max_max_x >= 0);
        assert(min_width > 0  && max_width > 0);

        assert(min_min_x <= max_min_x);
        assert(min_max_x <= max_max_x);
        assert(min_width <= max_width);

        BlockAgents_2D agents;
        agents.reserve(num_of_agents);
        std::random_device rd;
        std::mt19937 *MT = new std::mt19937(rd());

        float min_x, max_x, width;
        for(int i=0; i<num_of_agents; i++) {

            min_x = getValueWithResolution<float>(min_min_x, max_min_x, resolution);
//            std::cout << "min x = " << min_x << " ";
            max_x = getValueWithResolution<float>(min_max_x, max_max_x, resolution);
//            std::cout << "max x = " << max_x << " ";
            width = getValueWithResolution<float>(min_width, max_width, resolution);
//            std::cout << "width = " << width << " ";

//            std::cout << std::endl;

            Pointf<2> min_pt, max_pt;
            min_pt[0] = min_x, min_pt[1] = -width;
            max_pt[0] = max_x, max_pt[1] = width;
            BlockAgent_2D agent(min_pt, max_pt, i, dim);
            agents.push_back(agent);
        }
        delete MT;
        agents.shrink_to_fit();
        return agents;
    }

    // 1, all agent's start and target shouldn't conflict with other's start and target
    // 2, every agent can found a way to target
    // max_sample, sample for each agent, avoid infinite sample
    template<Dimension N, typename AgentType>
    struct LargeAgentMAPF_InstanceGenerator {
    public:
            LargeAgentMAPF_InstanceGenerator(const std::vector<AgentType> &agents,
                                             const IS_OCCUPIED_FUNC<N> &isoc,
                                             DimensionLength *dim,
                                             int max_sample = 100000)
                                             : agents_(agents), dim_(dim), isoc_(isoc),
                                             distance_map_updater_(DistanceMapUpdater<N>(this->isoc_, this->dim_)),
                                             max_sample_(max_sample)
                                             {

            // 1, construct all possible poses
            Id total_index = getTotalIndexOfSpace<N>(dim_);
            all_poses_.resize(total_index*2*N, nullptr); // a position with 2*N orientation
            Pointi<N> pt;
            for(Id id=0; id<total_index; id++) {
                pt = IdToPointi<N>(id, dim_);
                if(!isoc_(pt)) {
                    for(int orient=0; orient<2*N; orient++) {
                        PosePtr<int, N> pose_ptr = new Pose<int, N>(pt, orient);
                        all_poses_[id*2*N + orient] = pose_ptr;
                    }
                }
            }

            // 2, construct each agents subgraph (vertex: where it can stay, edges: whether can it move from one vertex to another)
            agent_sub_graphs_.reserve(agents_.size());
            for(const auto& agent : agents) {
                agent_sub_graphs_.push_back(constructSubGraphOfAgent(agent));
            }
        }

        ~LargeAgentMAPF_InstanceGenerator() {
            for(int i=0; i<all_poses_.size(); i++) {
                if(all_poses_[i] != nullptr) {
                    delete all_poses_[i];
                    all_poses_[i] = nullptr;
                }
            }
        }

        // if agent failed to find legal instance, repick
        std::vector< std::pair<InstanceOrient<N>, LAMAPF_Path> > getNewInstance() const {
            std::vector< std::pair<InstanceOrient<N>, LAMAPF_Path> > new_instances;
            std::random_device rd;
            int count_of_pick = 0;
            for (int i=0; i<agents_.size(); i++) {
                const auto &agent = agents_[i];
                size_t start_id, target_id;
                count_of_pick = 0;
                while (true) {
                    // 1, pick start nodes from its subgraph
                    start_id = rd() % all_poses_.size();
                    count_of_pick ++;
                    if(count_of_pick > max_sample_) { break;}
                    if(agent_sub_graphs_[i].all_nodes_[start_id] == nullptr) {
                        continue;
                    }
                    //std::cout << "pick agent " << i << "'s start " <<  std::endl;
                    // 2, if it has conflict with other agents, re-pick util no conflicts
                    bool repick = false;
                    for (int id = 0; id < new_instances.size(); id++) {
                        const auto &other_agent = agents_[id];
                        if (isCollide(agent, *all_poses_[start_id],
                                      other_agent, new_instances[id].first.first)) {
                            repick = true;
                            break;
                        }
                        if (isCollide(agent, *all_poses_[start_id],
                                      other_agent, new_instances[id].first.second)) {
                            repick = true;
                            break;
                        }
                    }
                    if(repick) {
                        continue;
                    } else {
                        break;
                    }
                }
                if(count_of_pick > max_sample_) {
                    //std::cout << "failed to get start for agent " << i << std::endl;
                    return {};
                }
                //std::cout << "find agent " << i << "'s start" << std::endl;
                while (true) {
                    // 1, pick start nodes from its subgraph
                    target_id = rd() % all_poses_.size();
                    count_of_pick ++;
                    if(count_of_pick > max_sample_) { break;}
                    if(agent_sub_graphs_[i].all_nodes_[target_id] == nullptr) {
                        continue;
                    }
                    if(target_id == start_id) {
                        continue;
                    }
                    //std::cout << "pick agent " << i << "'s target " <<  std::endl;
                    // 2, if it has conflict with other agents, re-pick util no conflicts
                    bool repick = false;
                    for (int id = 0; id < new_instances.size(); id++) {
                        const auto &other_agent = agents_[id];
                        if (isCollide(agent, *all_poses_[target_id],
                                      other_agent, new_instances[id].first.first)) {
                            repick = true;
                            break;
                        }
                        if (isCollide(agent, *all_poses_[target_id],
                                      other_agent, new_instances[id].first.second)) {
                            repick = true;
                            break;
                        }
                    }
                    if(repick) {
                        continue;
                    }
                    // 3, check whether there is a path connect them, otherwise re-pick
                    LAMAPF_Path path = getConnectionBetweenNode(i, start_id, target_id);
                    if(path.empty()) {
                        //std::cout << "find no connection between start and target, repick target" << std::endl;
                        continue;
                    } else {
                        std::cout << "find agent " << i << "'s instance success" << std::endl;
                        // 4, insert as a success instance
                        new_instances.push_back({{*all_poses_[start_id], *all_poses_[target_id]}, path});
                        break;
                    }
                }
                if(count_of_pick > max_sample_) {
                    std::cout << "failed to get target for agent " << i << std::endl;
                    return {};
                }
            }
            return new_instances;
        }

        LAMAPF_Path getConnectionBetweenNode(const int& agent_id, const size_t& start_id, const size_t& target_id) const {
            LAMAPF_Path retv;
            const auto& sub_graph = agent_sub_graphs_[agent_id];
            assert(sub_graph.all_nodes_[start_id] != nullptr && sub_graph.all_nodes_[target_id] != nullptr);
            std::vector<size_t> pre_visited(all_poses_.size(), MAX<size_t>);
            pre_visited[start_id] = start_id;
            std::vector<size_t> current_nodes = {start_id}, next_nodes;
            bool reach_target = false;
            while (!current_nodes.empty()) {
                next_nodes.clear();
                for(const auto& node : current_nodes) {
                    const auto& neighbors = sub_graph.all_edges_[node];
                    for(const auto& another_node : neighbors) {
                        if(pre_visited[another_node] != MAX<size_t>) { continue; }
                        if(another_node == target_id) {
                            pre_visited[another_node] = node;
//                            std::cout << "find path " << std::endl;
                            // retrieve path
                            LAMAPF_Path path;
                            size_t buffer_node = another_node;
                            while (buffer_node != start_id) {
//                                std::cout << "buffer_node " << buffer_node << std::endl;
                                path.push_back(buffer_node);
                                buffer_node = pre_visited[buffer_node];
                            }
                            path.push_back(start_id);
                            std::reverse(path.begin(), path.end());
//                            std::cout << "find path size = " << path.size() << std::endl;
                            return path;
                        }
                        next_nodes.push_back(another_node);
                        pre_visited[another_node] = node;
                    }
                }
                std::swap(current_nodes, next_nodes);
            }
            return {};
        }

        const std::vector<PosePtr<int, N> >& getAllPoses() const
        {
                return all_poses_;
        }

    private:

        SubGraphOfAgent<N> constructSubGraphOfAgent(const AgentType& agent) const {
            Id total_index = getTotalIndexOfSpace<N>(dim_);
            assert(all_poses_.size() == total_index*2*N);
            SubGraphOfAgent<N> sub_graph;
            sub_graph.all_nodes_.resize(total_index * 2 * N, nullptr);
            // initial nodes in subgraph
            for(size_t id=0; id<total_index; id++) {
                if(all_poses_[id*2*N] != nullptr) {
                    // if current grid is free, then check all orientation
                    for(int orient=0; orient<2*N; orient ++) {
                        const auto& current_pose = all_poses_[id * 2 * N + orient];
                        if(!agent.isCollide(*current_pose, dim_, isoc_, distance_map_updater_)) {
                            sub_graph.all_nodes_[id * 2 * N + orient] = current_pose;
                        }
                    }
                }
            }
            // when add edges, assume agent can only change position or orientation, cannot change both of them
            // and orientation can only change 90 degree at one timestep, that means the two orient must be orthogonal
            sub_graph.all_edges_.resize(total_index*2*N, {});
            sub_graph.all_backward_edges_.resize(total_index*2*N, {});
            Pointis<N> neighbors = GetNearestOffsetGrids<N>();
            for(size_t pose_id=0; pose_id < sub_graph.all_nodes_.size(); pose_id++) {
                const auto& node_ptr = sub_graph.all_nodes_[pose_id];
                if(node_ptr != nullptr) {
                    // add edges about position changing
                    size_t origin_orient = pose_id%(2*N);
                    Pointi<N> origin_pt = node_ptr->pt_;
                    Pointi<N> new_pt;
                    for(const auto& offset : neighbors) {
                        new_pt = origin_pt + offset;
                        if(isOutOfBoundary(new_pt, dim_)) { continue; }
                        Id another_node_id = PointiToId<N>(new_pt, dim_)*2*N + origin_orient;
                        PosePtr<int, N> another_node_ptr = sub_graph.all_nodes_[another_node_id];
                        if(another_node_ptr == nullptr) { continue; }
                        if(!agent.isCollide(*node_ptr, *another_node_ptr, dim_, isoc_, distance_map_updater_)) {
                            sub_graph.all_edges_[pose_id].push_back(another_node_id);
                            sub_graph.all_backward_edges_[another_node_id].push_back(pose_id);
                        }
                    }
                    // add edges about orientation changing
                    size_t base_id = pose_id / (2*N) * (2*N);
                    for(size_t orient=0; orient<2*N; orient++) {
                        // avoid repeat itself
                        if(node_ptr->orient_ == orient) { continue; }
                        // if another node in subgraph
                        size_t another_node_id = base_id + orient;
                        PosePtr<int, N> another_node_ptr = sub_graph.all_nodes_[another_node_id];
                        if(another_node_ptr == nullptr) { continue; }
                        // check whether can transfer to another node
                        if(!agent.isCollide(*node_ptr, *another_node_ptr, dim_, isoc_, distance_map_updater_)) {
                            sub_graph.all_edges_[pose_id].push_back(another_node_id);
                            sub_graph.all_backward_edges_[another_node_id].push_back(pose_id);
                        }
                    }
                }
            }
            return sub_graph;
        }

        InstanceOrients<N> instance_;

        const std::vector<AgentType>& agents_;
        DimensionLength* dim_;
        const IS_OCCUPIED_FUNC<N>& isoc_;

        // intermediate variables
        std::vector<PosePtr<int, N> > all_poses_;
        DistanceMapUpdater<N> distance_map_updater_;
        std::vector<SubGraphOfAgent<N> > agent_sub_graphs_;
        int max_sample_ = 100000;

    };

}

#endif //LAYEREDMAPF_LARGE_AGENT_INSTANCE_GENERATOR_H
