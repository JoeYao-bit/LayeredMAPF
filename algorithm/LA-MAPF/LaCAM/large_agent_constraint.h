//
// Created by yaozhuo on 2024/6/22.
//

#ifndef LAYEREDMAPF_LARGE_AGENT_CONSTRAINT_H
#define LAYEREDMAPF_LARGE_AGENT_CONSTRAINT_H

#include "utils.h"
#include "../circle_shaped_agent.h"
#include "../block_shaped_agent.h"

namespace freeNav::LayeredMAPF::LA_MAPF::LaCAM {

    template<Dimension N, typename AgentType>
    struct LargeAgentConstraints {
    public:
        explicit LargeAgentConstraints(const std::vector<PosePtr<int, N>>& all_nodes, const std::vector<AgentType>& agents, const std::vector<size_t>& occupied_now)
        : agents(agents), all_nodes(all_nodes), occupied_now(occupied_now) {
            assert(occupied_now.size() == agents.size());
            occupied_next.resize(agents.size(), MAX<size_t>);
        }

        // check what agents are collide with current agent, at this pose
        std::vector<int> collideWith(int id, const size_t& pose) const {
            std::vector<int> retv;
            for(int i=0; i<agents.size(); i++) {
                if(i == id) { continue; }
                if(occupied_next[i] == MAX<size_t>) {
                    if(isCollide(agents[i], *all_nodes[occupied_now[i]],
                                 agents[id], *all_nodes[pose])) {
                        retv.push_back(i);
                    }
                } else {
                    if(isCollide(agents[i],  *all_nodes[occupied_now[i]], *all_nodes[occupied_next[i]],
                                 agents[id], *all_nodes[pose])) {
                        retv.push_back(i);
                    }
                }
            }
            return retv;
        }

        void setOccupiedNext(const int& id, const size_t& pose) {
            occupied_next[id] = pose;
        }

        void setOccupied(const int& id, const size_t& pose_now, const size_t& pose_next) {
            occupied_now[id] = pose_now;
            occupied_next[id] = pose_next;
        }

        bool hasCollide(const int& id, const size_t& pose) const {
            for(int i=0; i<agents.size(); i++) {
                if(i == id) { continue; }
                if(occupied_next[i] == MAX<size_t>) {
                    if(isCollide(agents[i], *all_nodes[occupied_now[i]],
                                 agents[id], *all_nodes[pose])) {
                        return true;
                    }
                } else {
                    if(isCollide(agents[i],  *all_nodes[occupied_now[i]], *all_nodes[occupied_next[i]],
                                 agents[id], *all_nodes[pose])) {
                        return true;
                    }
                }
            }
            return false;
        }

        bool hasCollide(const int& id, const size_t& pose_now, const size_t& pose_next) const {
            for(int i=0; i<agents.size(); i++) {
                if(i == id) { continue; }
                if(occupied_next[i] == MAX<size_t>) {
                    if(isCollide(agents[i],  *all_nodes[occupied_now[i]],
                                 agents[id], *all_nodes[pose_now], *all_nodes[pose_next])) {
                        return true;
                    }
                } else {
                    if(isCollide(agents[i],  *all_nodes[occupied_now[i]], *all_nodes[occupied_next[i]],
                                 agents[id], *all_nodes[pose_now],        *all_nodes[pose_next])) {
                        return true;
                    }
                }
            }
            return false;
        }

    private:
        const std::vector<AgentType>& agents;
        const std::vector<PosePtr<int, N>>& all_nodes;
        std::vector<size_t> occupied_now;
        std::vector<size_t> occupied_next;

    };

}

#endif //LAYEREDMAPF_LARGE_AGENT_CONSTRAINT_H
