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
        delete P;
        return solver->getResultPath(dim, instance_sat);
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

    path_pathfinding::Grid * setStatesToOccupied(path_pathfinding::Grid * G, const freeNav::Pointis<2>& grids) {
        assert(G != nullptr);
        // set node to unpassable
        for(const auto& pt : grids) {
            int x = pt[0], y = pt[1];
            G->V[y*G->width + x] = nullptr;
        }
        // set related edge to unpassable
        for(const auto& pt : grids) {
            int x = pt[0], y = pt[1];
            Nodes nearby_nodes;
            // left
            if (G->existNode(x - 1, y)) nearby_nodes.push_back(G->getNode(x - 1, y));
            // right
            if (G->existNode(x + 1, y)) nearby_nodes.push_back(G->getNode(x + 1, y));
            // up
            if (G->existNode(x, y - 1)) nearby_nodes.push_back(G->getNode(x, y - 1));
            // down
            if (G->existNode(x, y + 1)) nearby_nodes.push_back(G->getNode(x, y + 1));
            // if nearby node have edge to current grid, remove it
            for(const auto& node : nearby_nodes) {
                for(auto iter = node->neighbor.begin(); iter != node->neighbor.end();) {
                    if((*iter)->pos.x == x && (*iter)->pos.y == y) {
                        iter = node->neighbor.erase(iter);
                    } else  {
                        iter ++;
                    }
                }
            }
        }
        return G;
    }

}