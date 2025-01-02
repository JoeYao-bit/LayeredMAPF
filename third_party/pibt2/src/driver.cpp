//
// Created by yaozhuo on 2024/12/31.
//

#include "../include/driver.h"

namespace PIBT_2 {

    path_pathfinding::Grid * external_grid_ptr = nullptr;

    freeNav::Paths<2> pibt_MAPF(freeNav::DimensionLength *dim,
                                const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                                const freeNav::Instances<2> &instance_sat,
                                CBS_Li::ConstraintTable *ct,
                                int cutoff_time) {

        std::random_device rd; // 创建一个std::random_device对象
        unsigned int seed = rd(); // 生成一个随机的种子值
        std::mt19937* engine = new std::mt19937(seed); // 使用随机的种子值创建一个伪随机数生成器
        // set problem
        auto P = MAPF_Instance(dim, isoc, instance_sat, ct, MAX_TIMESTEP/2, cutoff_time*1e3, engine);
        // solve
        std::unique_ptr<MAPF_Solver> solver = std::make_unique<PIBT>(&P);
        solver->setLogShort(true);
        solver->solve();
        if (solver->succeed() && !solver->getSolution().validate(&P)
                ) {
            std::cout << "pibt_MAPF: invalid results" << std::endl;
            return {};
        }
        if(!solver->succeed()) {
            return {};
        }
        //solver->printResult();
        // output result
        //solver->makeLog(output_file);
        return solver->getResultPath(dim, instance_sat);
    }

    freeNav::Paths<2> pibt2_MAPF(freeNav::DimensionLength *dim,
                                 const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                                 const freeNav::Instances<2> &instance_sat,
                                 CBS_Li::ConstraintTable *ct,
                                 int cutoff_time) {

        std::random_device rd; // 创建一个std::random_device对象
        unsigned int seed = rd(); // 生成一个随机的种子值
        std::mt19937* engine = new std::mt19937(seed); // 使用随机的种子值创建一个伪随机数生成器
        // set problem
        MAPF_Instance* P = nullptr;
        if(external_grid_ptr != nullptr) {
//            std::cout << "use external grid ptr" << std::endl;
            P = new MAPF_Instance(external_grid_ptr, instance_sat, ct, MAX_TIMESTEP/2, cutoff_time*1e3, engine);
        } else {
            P = new MAPF_Instance(dim, isoc, instance_sat, ct, MAX_TIMESTEP/2, cutoff_time*1e3, engine);
        }
        P->ct_ = ct;

        // solve
        std::unique_ptr<MAPF_Solver> solver = std::make_unique<PIBT_PLUS>(P);
        solver->setLogShort(true);
        solver->solve();
        if (solver->succeed()  && !solver->getSolution().validate(P)
                ) {
            std::cout << "pibt2_MAPF: invalid results" << std::endl;
            delete P;
            return {};
        }
        if(!solver->succeed()) {
            delete P;
            return {};
        }
        //solver->printResult();
        // output result
        //solver->makeLog(output_file);
        auto final_solution = solver->getResultPath(dim, instance_sat);
        delete P;
        return final_solution;
    }

    freeNav::Paths<2> hca_MAPF(freeNav::DimensionLength *dim,
                               const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                               const freeNav::Instances<2> &instance_sat,
                               CBS_Li::ConstraintTable *ct,
                               int cutoff_time) {

        std::random_device rd; // 创建一个std::random_device对象

        unsigned int seed = rd(); // 生成一个随机的种子值

        std::mt19937* engine = new std::mt19937(seed); // 使用随机的种子值创建一个伪随机数生成器
        // set problem
        auto P = MAPF_Instance(dim, isoc, instance_sat, ct, MAX_TIMESTEP/2, cutoff_time*1e3, engine);
        P.ct_ = ct;

        // solve
        std::unique_ptr<MAPF_Solver> solver = std::make_unique<HCA>(&P, ct);
        solver->setLogShort(true);
        solver->solve();
        if (solver->succeed() && !solver->getSolution().validate(&P)) {
            std::cout << "hca_MAPF: invalid results" << std::endl;
            return {};
        }
        if(!solver->succeed()) {
            return {};
        }
        //solver->printResult();
        // output result
        //solver->makeLog(output_file);
        return solver->getResultPath(dim, instance_sat);
    }

    freeNav::Paths<2> push_and_swap_MAPF(freeNav::DimensionLength *dim,
                                         const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                                         const freeNav::Instances<2> &instance_sat,
                                         CBS_Li::ConstraintTable *ct,
                                         int cutoff_time) {

        std::random_device rd; // 创建一个std::random_device对象

        unsigned int seed = rd(); // 生成一个随机的种子值

        std::mt19937* engine = new std::mt19937(seed); // 使用随机的种子值创建一个伪随机数生成器
        // set problem
        auto P = MAPF_Instance(dim, isoc, instance_sat, ct, MAX_TIMESTEP/2, cutoff_time*1e3, engine);
        P.ct_ = ct;

        // solve
        std::unique_ptr<MAPF_Solver> solver = std::make_unique<PushAndSwap>(&P);
        solver->setLogShort(true);
        solver->solve();
        if (solver->succeed() && !solver->getSolution().validate(&P)) {
            std::cout << "push_and_swap_MAPF: invalid results" << std::endl;
            return {};
        }
        if(!solver->succeed()) {
            return {};
        }
        //solver->printResult();
        // output result
        //solver->makeLog(output_file);
        return solver->getResultPath(dim, instance_sat);
    }

    std::vector<int> statGraph(path_pathfinding::Grid * G) {
        std::vector<int> retv;
        for(int i=0; i<G->getV().size(); i++) {
            //
        }
        return {};
    }

    void setStatesToOccupied(path_pathfinding::Grid * G, const freeNav::Pointis<2>& grids,
                                                 const freeNav::IS_OCCUPIED_FUNC<2>& raw_isoc,
                                                 const freeNav::IS_OCCUPIED_FUNC<2>& new_isoc) {
        assert(G != nullptr);

        // set related edge to unpassable
        for(const auto& pt : grids) {
            int x = pt[0], y = pt[1];
            assert(G->V[y*G->width + x] != nullptr);
            Nodes nearby_nodes;
            // left
            if (G->existNode(x - 1, y)) nearby_nodes.push_back(G->getNode(x - 1, y));
            // right
            if (G->existNode(x + 1, y)) nearby_nodes.push_back(G->getNode(x + 1, y));
            // up
            if (G->existNode(x, y - 1)) nearby_nodes.push_back(G->getNode(x, y - 1));
            // down
            if (G->existNode(x, y + 1)) nearby_nodes.push_back(G->getNode(x, y + 1));
            // if (must) nearby node have edge to current grid, remove it
            for(const auto& node : nearby_nodes) {
                // only considering raw passable and st
                int erase_count = 0;
                for(auto iter = node->neighbor.begin(); iter != node->neighbor.end();) {
                    if((*iter)->pos.x == x && (*iter)->pos.y == y) {
                        iter = node->neighbor.erase(iter);
                        //std::cout << "delete edge " << node->pos.x << ", " << node->pos.y << " / " << x << ", " << y << std::endl;
                        erase_count ++;
                    } else  {
                        iter ++;
                    }
                }
                freeNav::Pointi<2> near_pt; near_pt[0] = node->pos.x, near_pt[1] = node->pos.y;
                //std::cout << "nearyby isoc/new_isoc = " << raw_isoc(near_pt) << "/" << new_isoc(near_pt) << std::endl;
                //std::cout << "node->neighbor size " << node->neighbor.size() << ", erase_count = " << erase_count << std::endl;
                assert(erase_count == 1);
            }
        }

        // set node to unpassable
        for(const auto& pt : grids) {
            int x = pt[0], y = pt[1];
            assert(G->V[y*G->width + x] != nullptr); // yz: other agents' start or target must be free in raw graph
            delete G->V[y*G->width + x]; // yz: remove occupied node permanently, but we will restore it
            G->V[y*G->width + x] = nullptr;
        }
        return;
    }


    void restoreStatesToPassable(path_pathfinding::Grid * G, const freeNav::Pointis<2>& grids,
                                                     const freeNav::IS_OCCUPIED_FUNC<2>& raw_isoc,
                                                     const freeNav::IS_OCCUPIED_FUNC<2>& new_isoc) {
        assert(G != nullptr);

        // restore nodes
        for(const auto& pt : grids) {
            int x = pt[0], y = pt[1];
            assert(G->V[y*G->width + x] == nullptr); // yz: other agents' start or target must be free in raw graph
            G->V[y*G->width + x] = new Node(y*G->width + x, x, y);
        }
        // restore edges of restore nodes
        for(const auto& pt : grids) {
            int x = pt[0], y = pt[1];
            assert(G->V[y*G->width + x] != nullptr); // yz: other agents' start or target must be free in raw graph
            // left
            if (G->existNode(x - 1, y)) G->V[y*G->width + x]->neighbor.push_back(G->getNode(x - 1, y));
            // right
            if (G->existNode(x + 1, y)) G->V[y*G->width + x]->neighbor.push_back(G->getNode(x + 1, y));
            // up
            if (G->existNode(x, y - 1)) G->V[y*G->width + x]->neighbor.push_back(G->getNode(x, y - 1));
            // down
            if (G->existNode(x, y + 1)) G->V[y*G->width + x]->neighbor.push_back(G->getNode(x, y + 1));
        }
        // restore related edge to passable
        for(const auto& pt : grids) {
            int x = pt[0], y = pt[1];
            assert(G->V[y*G->width + x] != nullptr);
            Nodes nearby_nodes;
            // left
            if (G->existNode(x - 1, y)) nearby_nodes.push_back(G->getNode(x - 1, y));
            // right
            if (G->existNode(x + 1, y)) nearby_nodes.push_back(G->getNode(x + 1, y));
            // up
            if (G->existNode(x, y - 1)) nearby_nodes.push_back(G->getNode(x, y - 1));
            // down
            if (G->existNode(x, y + 1)) nearby_nodes.push_back(G->getNode(x, y + 1));
            // if (must) nearby node have edge to current grid, remove it
            for(const auto& node : nearby_nodes) {
                freeNav::Pointi<2> pt; pt[0] = node->pos.x, pt[1] = node->pos.y;
                if(!raw_isoc(pt) && !new_isoc(pt)) {
                    // do not add edge to new added nodes, which already add in previous
                    int edge_count = 0;
                    // check whether already have related edges (in theory there shouldn't be)
                    for(auto iter = node->neighbor.begin(); iter != node->neighbor.end(); iter++) {
                        if((*iter)->pos.x == x && (*iter)->pos.y == y) {
                            iter = node->neighbor.erase(iter);
                            edge_count ++;
                        }
                    }
                    assert(edge_count == 0);
                    node->neighbor.push_back(G->V[y*G->width + x]);
                }
            }
        }

        return;
    }


}