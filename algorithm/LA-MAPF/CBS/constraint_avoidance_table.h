//
// Created by yaozhuo on 2024/7/8.
//

#ifndef LAYEREDMAPF_CONSTRAINT_AVOIDANCE_TABLE_H
#define LAYEREDMAPF_CONSTRAINT_AVOIDANCE_TABLE_H

#include "../common.h"
#include <unordered_set>
#include <functional>

namespace freeNav::LayeredMAPF::LA_MAPF {

    template<Dimension N, typename AgentType>
    struct ConstraintAvoidanceTable {
    public:
        struct OccGrid {
            Id grid_id = MAX<Id>;
            int agent_id;

//            static bool compareGrid(const OccGrid& g1, const OccGrid& g2) {
//                return g1.grid_id < g2.grid_id;
//            }

//            bool operator==(const OccGrid& other_grid) const
//            {
//                if (grid_id == other_grid.grid_id) return true;
//                else return false;
//            }
//
//            struct HashFunction
//            {
//                size_t operator()(const OccGrid& grid) const
//                {
//                    return std::hash<Id>()(grid.grid_id);
//                }
//            };

        };


        typedef std::vector<OccGrid> OccGridLevel;

//        typedef std::vector<OccGridLevel> OccGridss;

//        typedef std::unordered_set<OccGrid, std::function<bool(const OccGrid& g1, const OccGrid& g2)> > OccGridSet;
        // HashFunction
//        typedef std::unordered_set<OccGrid, typename OccGrid::HashFunction> OccGridSet;

        typedef std::vector<OccGridLevel> OccGridLevels;

        explicit ConstraintAvoidanceTable(DimensionLength* dim, const std::vector<PosePtr<int, N> >& all_poses, const AgentType& agent)
        : dim_(dim), all_poses_(all_poses), agent_(agent) {
            //
        }

        void setInitOccGrids(const std::vector<OccGridLevels>& init_occ_grids) {
            occ_grids_ = init_occ_grids;
            for(const auto& occ_grids : init_occ_grids) {
                insertOccGrids(occ_grids);
            }
        }

        void updateAgent(const AgentType& agent) {
            agent_ = agent;
        }

        void updateAgentPathOccGrids(const AgentType& agent, const OccGridLevels & new_occ_grids) {
            Id total_index = getTotalIndexOfSpace<N>(dim_);
            if(occ_table_.size() < new_occ_grids.size()) {
                occ_table_.resize(new_occ_grids.size(), OccGridLevel(total_index));
            }
            // clear previous occ grids
//            for(int t=0; t<occ_grids_[agent.id_].size(); t++) {
//                for(const auto& occ_grid : occ_grids_[agent.id_][t]) {
//                    occ_table_[t][occ_grid.grid_id_] = OccGrid(MAX<Id>, 0);
//                }
//            }
            clearExistingOccGrids(agent.id_);
            occ_grids_[agent.id_] = new_occ_grids;
            // set now occ grids
            for(int t=0; t<new_occ_grids.size(); t++) {
                for(const auto& occ_grid : new_occ_grids[t]) {
                    occ_table_[t][occ_grid.grid_id] = occ_grid;
                }
            }
        }

        void clearAllExistingOccGrids() {
            for(int agent=0; agent<occ_grids_.size(); agent++) {
                clearExistingOccGrids(agent);
            }
        }

        void clearExistingOccGrids(int agent_id) {
            // clear previous occ grids
            for(int t=0; t<occ_grids_[agent_id].size(); t++) {
                for(const auto& occ_grid : occ_grids_[agent_id][t]) {
                    occ_table_[t][occ_grid.grid_id] = OccGrid{MAX<Id>, 0};
                }
            }
            occ_grids_[agent_id].clear();
        }

        // for test only
        void insertAgentPathOccGrids(const AgentType& agent, const LAMAPF_Path& path) {
            // resize will keep previous element
            makespan_ = std::max(makespan_, (int)path.size());
            Id total_index = getTotalIndexOfSpace<N>(dim_);
            occ_table_.resize(makespan_, OccGridLevel(total_index));
            if(agent.id_ >= occ_grids_.size()) { occ_grids_.resize(agent.id_ + 1); }
            occ_grids_[agent.id_].clear();
            for(int t=0; t<path.size()-1; t++) {
                Pointis<N> grids = agent.getTransferOccupiedGrid(*all_poses_[path[t]], *all_poses_[path[t+1]]);
                OccGridLevel level;
                for(const auto& grid : grids) {
                    Id id = PointiToId(grid, dim_);
                    occ_table_[t][id] = OccGrid{id, agent.id_};
                    level.push_back(occ_table_[t][id]);
                }
                occ_grids_[agent.id_].push_back(level);
            }
            auto grid_pairs = agent.getPoseOccupiedGrid(*all_poses_[path.back()]);
            OccGridLevel level;
            for(const auto& grid : grid_pairs.first) {
                Id id = PointiToId(grid, dim_);
                occ_table_[path.size()-1][id] = OccGrid{id, agent.id_};
                level.push_back(occ_table_[path.size()-1][id]);
            }
            occ_grids_[agent.id_].push_back(level);
        }

        static OccGridLevels getAgentPathOccGrids(const AgentType& agent,
                                                  const LAMAPF_Path& path,
                                                  const std::vector<PosePtr<int, N> >& all_nodes,
                                                  DimensionLength* dim) {
            OccGridLevels retv;
            for(int t=0; t<path.size()-1; t++) {
                Pointis<N> grids = agent.getTransferOccupiedGrid(*all_nodes[path[t]], *all_nodes[path[t+1]]);
                OccGridLevel current_grids;
                for(const auto& grid : grids) {
                    Id id = PointiToId(grid, dim);
                    current_grids.push_back(OccGrid{id, agent.id_});
                }
                retv.push_back(current_grids);
            }
            return retv;
        }


        int getNumOfConflictsForStep(const Pose<int, N>& curr_node, const Pose<int, N>& next_node, int current_timestep) const {
            Pointis<N> occ_grids = agent_.getTransferOccupiedGrid(curr_node, next_node);
            return getNumOfConflictsForStep(occ_grids, agent_.id_, current_timestep);
        }

        // get how many agent current agent collide with
        int getNumOfConflictsForStep(const Pointis<N>& pts, int agent_id,
                                     int timestep) const {
            if(timestep + 1 > occ_table_.size()) {
                return 0;
            }
//            std::cout << "occ_table_.size() " << occ_table_.size() << std::endl;
            std::set<int> agent_ids;
            for(const auto& pt : pts) {
                Id id = PointiToId(pt, dim_);
                auto occ_grid = occ_table_[timestep][id];
                if(occ_grid.grid_id != MAX<Id>) {
                    if(occ_grid.agent_id != agent_id) {
                        agent_ids.insert(occ_grid.agent_id);
                    }
                }
            }
//            std::cout << " conf agent id: ";
//            for(const auto& id : agent_ids) {
//                std::cout << id << " ";
//            }
//            std::cout << std::endl;
            return agent_ids.size();
        }

        void printOccTable() {
            for(int t=0; t<occ_table_.size(); t++) {
                std::cout << "t = " << t << ": ";
                for(const auto& occ_grid : occ_table_[t]) {
                    if(occ_grid.grid_id != MAX<Id>) { continue; }
                    std::cout << "{" << occ_grid.grid_id << ", " << occ_grid.agent_id << "} ";
                }
                std::cout << std::endl;
            }
        }

        AgentType agent_; // current agent

    private:

        // an agent occupied at each time index
        virtual void insertOccGrids(const OccGridLevels& gridss) {
            // resize will keep previous element
            makespan_ = std::max(makespan_, int(gridss.size()));
            Id total_index = getTotalIndexOfSpace<N>(dim_);
            occ_table_.resize(makespan_, OccGridLevel(total_index));
            for(int t=0; t<gridss.size(); t++) {
                // insert grids of each time index, an agent may occupied more than one agent
                for(const auto& grid : gridss[t]) {
                    occ_table_[t][grid.grid_id] = grid;
                }
            }
        }

        const std::vector<PosePtr<int, N> >& all_poses_;

        std::vector<OccGridLevels> occ_grids_;

        OccGridLevels occ_table_;

        int makespan_ = 0;

        DimensionLength * dim_;
    };

}

#endif //LAYEREDMAPF_CONSTRAINT_AVOIDANCE_TABLE_H
