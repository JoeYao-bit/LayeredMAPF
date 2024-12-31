//
// Created by yaozhuo on 2024/12/31.
//

#include "../include/driver.h"

namespace PIBT_2 {

    DistanceTable external_distance_table;
    bool layered_PIBT2 = false;

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
        auto P = MAPF_Instance(dim, isoc, instance_sat, ct, MAX_TIMESTEP/2, cutoff_time*1e3, engine);
        P.ct_ = ct;

        // solve
        std::unique_ptr<MAPF_Solver> solver = std::make_unique<PIBT_PLUS>(&P);
        if(layered_PIBT2 && !external_distance_table.empty()) {
            solver->setDistanceTable(external_distance_table);
        }
        solver->setLogShort(true);
        solver->solve();
        if(layered_PIBT2) { external_distance_table = solver->distance_table; }
        if (solver->succeed() //&& !solver->getSolution().validate(&P)
                ) {
            std::cout << "pibt2_MAPF: invalid results" << std::endl;
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


}