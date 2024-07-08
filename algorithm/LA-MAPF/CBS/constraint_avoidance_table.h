//
// Created by yaozhuo on 2024/7/8.
//

#ifndef LAYEREDMAPF_CONSTRAINT_AVOIDANCE_TABLE_H
#define LAYEREDMAPF_CONSTRAINT_AVOIDANCE_TABLE_H

#include "../common.h"

namespace freeNav::LayeredMAPF::LA_MAPF {

    template<Dimension N, typename AgentType>
    struct ConstraintAvoidanceTable {
    public:
        struct OccGrid {
            Id grid_id;
            int agent_id;

            static bool compareGrid(const OccGrid& g1, const OccGrid& g2) {
                return g1.grid_id < g2.grid_id;
            }

        };

        typedef std::vector<OccGrid> OccGrids;

        typedef std::vector<OccGrids> OccGridss;

        typedef std::set<OccGrid, std::function<bool(const OccGrid& g1, const OccGrid& g2)> > OccGridSet;

        ConstraintAvoidanceTable(DimensionLength* dim, const std::vector<PosePtr<int, N> >& all_poses)
        : dim_(dim), all_poses_(all_poses) {
            //
        }

        void insertAgentPathOccGrids(const AgentType& agent, const LAMAPF_Path& path) {
            // resize will keep previous element
            makespan_ = std::max(makespan_, (int)path.size());
            occ_table_.resize(makespan_, OccGridSet(OccGrid::compareGrid));
            for(int t=0; t<path.size()-1; t++) {
                Pointis<N> grids = agent.getTransferOccupiedGrid(*all_poses_[path[t]], *all_poses_[path[t+1]]);
                for(const auto& grid : grids) {
                    Id id = PointiToId(grid, dim_);
                    occ_table_[t].insert(OccGrid{id, agent.id_});
                }
            }
            auto grid_pairs = agent.getPoseOccupiedGrid(*all_poses_[path.back()]);
            for(const auto& grid : grid_pairs.first) {
                Id id = PointiToId(grid, dim_);
                occ_table_[path.size()-1].insert(OccGrid{id, agent.id_});
            }
        }

        // an agent occupied at each time index
        void insertOccGrids(const OccGridss& gridss) {
            // resize will keep previous element
            makespan_ = std::max(makespan_, gridss.size());
            occ_table_.resize(makespan_, OccGridSet(OccGrid::compareGrid));
            for(int t=0; t<gridss.size(); t++) {
                // insert grids of each time index, an agent may occupied more than one agent
                for(const auto& grid : gridss[t]) {
                    occ_table_[t].insert(grid);
                }
            }
        }

        // get how many agent current agent collide with
        int getNumOfConflictsForStep(const Pointis<N>& pts, int agent_id,
                                     int timestep) const {
            if(timestep > occ_table_.size() - 1) {
                return 0;
            }
            std::set<int> agent_ids;
            for(const auto& pt : pts) {
                Id id = PointiToId(pt, dim_);
                auto grid_iter = occ_table_[timestep].find(OccGrid{id, 0});
                if(grid_iter != occ_table_[timestep].end()) {
                    if(grid_iter->agent_id != agent_id) {
                        agent_ids.insert(grid_iter->agent_id);
                    }
                }
            }
            std::cout << " conf agent id: ";
            for(const auto& id : agent_ids) {
                std::cout << id << " ";
            }
            std::cout << std::endl;
            return agent_ids.size();
        }


        void printOccTable() {
            for(int t=0; t<occ_table_.size(); t++) {
                std::cout << "t = " << t << ": ";
                for(const auto& occ_grid : occ_table_[t]) {
                    std::cout << "{" << occ_grid.grid_id << ", " << occ_grid.agent_id << "} ";
                }
                std::cout << std::endl;
            }
        }

    private:

        const std::vector<PosePtr<int, N> >& all_poses_;

        std::vector<OccGridSet> occ_table_;

        int makespan_ = 0;

        DimensionLength * dim_;
    };

}

#endif //LAYEREDMAPF_CONSTRAINT_AVOIDANCE_TABLE_H
