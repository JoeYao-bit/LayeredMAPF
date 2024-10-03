//
// Created by yaozhuo on 2024/1/23.
//
#pragma once
#ifndef FREENAV_PIBT2_DRIVER_H
#define FREENAV_PIBT2_DRIVER_H

#include <getopt.h>

#include <pibt2/include/default_params.hpp>
#include <pibt2/include/hca.hpp>
#include <iostream>
#include <pibt2/include/pibt.hpp>
#include <pibt2/include/pibt_plus.hpp>
#include <pibt2/include/problem.hpp>
#include <pibt2/include/push_and_swap.hpp>
#include <random>
#include <vector>
#include "../../../algorithm/layered_mapf.h"

namespace PIBT_2 {

//    std::unique_ptr<MAPF_Solver> getSolver(const std::string solver_name, MAPF_Instance *P) {
//        std::unique_ptr<MAPF_Solver> solver;
//        if (solver_name == "PIBT") {
//            solver = std::make_unique<PIBT>(P);
//        } else if (solver_name == "HCA") {
//            solver = std::make_unique<HCA>(P);
//        } else if (solver_name == "PIBT_PLUS") {
//            solver = std::make_unique<PIBT_PLUS>(P);
//        } else if (solver_name == "PushAndSwap") {
//            solver = std::make_unique<PushAndSwap>(P);
//        } else {
//            std::cout << "warn@mapf: "
//                      << "unknown solver name, " + solver_name + ", continue by PIBT"
//                      << std::endl;
//            solver = std::make_unique<PIBT>(P);
//        }
//        //solver->setParams(argc, argv);
//        solver->setVerbose(false);
//        return solver;
//    }

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
        if (solver->succeed() && !solver->getSolution().validate(&P)) {
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
        solver->setLogShort(true);
        solver->solve();
        if (solver->succeed() && !solver->getSolution().validate(&P)) {
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

#endif //FREENAV_PIBT2_DRIVER_H
