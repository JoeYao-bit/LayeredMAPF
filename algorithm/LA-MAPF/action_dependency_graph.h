//
// Created by yaozhuo on 2024/11/7.
//

#ifndef LAYEREDMAPF_ACTION_DEPENDENCY_GRAPH_H
#define LAYEREDMAPF_ACTION_DEPENDENCY_GRAPH_H

#include "large_agent_mapf.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    template<Dimension N>
    struct ActionDependencyGraph {

        explicit ActionDependencyGraph(const std::vector<LAMAPF_Path>& paths,
                                       const std::vector<AgentPtr<N>>& agents,
                                       const std::vector<PosePtr<int, N> >& all_poses) :
                paths_(paths), agents_(agents), all_poses_(all_poses) {
            assert(paths_.size() == agents_.size());
            // 0, construct path_index_to_node_
            path_index_to_node_.resize(paths_.size());
            int id_count = 0;
            for(int i=0; i<paths_.size(); i++) {
                path_index_to_node_[i].resize(paths_[i].size());
                for(int j=0; j<paths_[i].size(); j++) {
                    path_index_to_node_[i][j] = id_count;
                    id_count ++;
                }
            }
            ADG_.resize(id_count, {});
            action_status_.resize(id_count, UNVISITED);
            // 1, insert path to ADG
//            for(int i=0; i<paths_.size(); i++) {
//                for(int j=0; j<paths_[i].size()-1; j++) {
//                    const int& pre_id = path_index_to_node_[i][j];
//                    const int& next_id = path_index_to_node_[i][j+1];
//                    ADG_[next_id].push_back(pre_id);
//                }
//            }
            // 2, traversal all path to construct action dependency graph
            for(int i=0; i<paths_.size(); i++) {
                for(int j=0; j<paths_.size(); j++) {
                    if(i != j) {
                        for(int t1=0; t1<paths_[i].size()-1; t1++) {
                            for(int t2=t1; t2<paths_[j].size()-1; t2++) {
                                if(isCollide(agents_[i], *all_poses[paths_[i][t1]], *all_poses[paths_[i][t1+1]],
                                             agents_[j], *all_poses[paths_[j][t2]], *all_poses[paths_[j][t2+1]])) {
                                    const int& pre_id = path_index_to_node_[i][t1];
                                    const int& next_id = path_index_to_node_[j][t2];
                                    ADG_[next_id].push_back(pre_id);
                                }
                            }
                        }
                    }
                }
            }
        }

        void setActionProcessing(int action_id) {
            action_status_[action_id] = PROCESSING;
        }

        void setActionLeave(int action_id) {
            action_status_[action_id] = LEAVE;
        }

        bool isActionUnvisited(int action_id) const {
            return action_status_[action_id] == UNVISITED;
        }

        bool isActionProcessing(int action_id) const {
            return action_status_[action_id] == PROCESSING;
        }

        bool isActionLeave(int action_id) const {
            return action_status_[action_id] == LEAVE;
        }

        // the maximal actions that agent can take since time t
        // rotate and move forward's cost may not the same
        // finish all valid action avoid unnecessary wait
        int getMaximalValidAction(int agent_id, int start_t) const {
            for(int t=start_t; t<paths_[agent_id].size()-1; t++) {
                if(!isActionValid(agent_id, t)) { return t-1; }
            }
            return paths_[agent_id].size() - 2;// if all actions are valid, just finish all actions
        }

        // an action is valid after all precedent action are finished
        bool isActionValid(int path_id, int t) const {
            int action_id = path_index_to_node_[path_id][t];
            for(const auto& pre_id : ADG_[action_id]) {
                if(!isActionLeave(pre_id)) { return false; }
            }
            return true;
        }

    private:

        const std::vector<LAMAPF_Path>& paths_;
        const std::vector<AgentPtr<N> >& agents_;
        const std::vector<PosePtr<int, N> >& all_poses_;

        std::vector<std::vector<int> > path_index_to_node_;
        std::vector<std::vector<int> > ADG_; // insert all edge of action dependency graph

        enum ACTION_STATE {UNVISITED, PROCESSING, LEAVE};
        std::vector<ACTION_STATE> action_status_;

    };

}

#endif //LAYEREDMAPF_ACTION_DEPENDENCY_GRAPH_H














