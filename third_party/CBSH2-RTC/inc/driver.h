//
// Created by yaozhuo on 2024/1/15.
//
#pragma once

#ifndef FREENAV_CBSH2_DRIVER_H
#define FREENAV_CBSH2_DRIVER_H

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "CBSH2-RTC/inc/CBS.h"
#include "EECBS/inc/ConstraintTable.h"

namespace CBSH2_RTC {

    freeNav::Paths<2> CBSH2_RTC_MAPF(freeNav::DimensionLength *dim,
                                 const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                                 const freeNav::Instances<2> &instance_sat,
                                 CBS_Li::ConstraintTable *ct,
                                 int cutoff_time) {
        namespace po = boost::program_options;
        // Declare the supported options.
        po::options_description desc("Allowed options");
        desc.add_options()
                ("help", "produce help message")

                // params for the input instance and experiment settings
                ("map,m", po::value<std::string>()->default_value(""), "input file for map")
                ("agents,a", po::value<std::string>()->default_value(""), "input file for agents")
                ("output,o", po::value<std::string>(), "output file for schedule")
                ("outputPaths", po::value<std::string>(), "output file for paths")
                ("agentNum,k", po::value<int>()->default_value(0), "number of agents")
                ("cutoffTime,t", po::value<double>()->default_value(2*cutoff_time), "cutoff time (seconds)") // *2 to get real cutoff time
                ("nodeLimit", po::value<int>()->default_value(MAX_NODES), "node limit")
                ("screen,s", po::value<int>()->default_value(0), "screen option (0: none; 1: results; 2:all)")
                ("seed,d", po::value<int>()->default_value(0), "random seed")
                ("stats", po::value<bool>()->default_value(false), "write to files some statistics")
                ("agentIdx", po::value<std::string>()->default_value(""), "customize the indices of the agents (e.g., \"0,1\")")

                // params for instance generators
                ("rows", po::value<int>()->default_value(0), "number of rows")
                ("cols", po::value<int>()->default_value(0), "number of columns")
                ("obs", po::value<int>()->default_value(0), "number of obstacles")
                ("warehouseWidth", po::value<int>()->default_value(0), "width of working stations on both sides, for generating instances")

                // params for CBS
                ("heuristics", po::value<std::string>()->default_value("WDG"), "heuristics for the high-level search (Zero, CG,DG, WDG)")
                ("prioritizingConflicts", po::value<bool>()->default_value(true), "conflict priortization. If true, conflictSelection is used as a tie-breaking rule.")
                ("bypass", po::value<bool>()->default_value(true), "Bypass1")
                ("disjointSplitting", po::value<bool>()->default_value(false), "disjoint splitting")
                ("rectangleReasoning", po::value<std::string>()->default_value("GR"), "rectangle reasoning strategy (None, R, RM, GR, Disjoint)")
                ("corridorReasoning", po::value<std::string>()->default_value("GC"), " corridor reasoning strategy (None, C, PC, STC, GC, Disjoint")
                ("mutexReasoning", po::value<bool>()->default_value(false), "Using mutex reasoning")
                ("targetReasoning", po::value<bool>()->default_value(true), "Using target reasoning")
                ("restart", po::value<int>()->default_value(1), "number of restart times (at least 1)")
                ("sipp", po::value<bool>()->default_value(false), "using sipp as the single agent solver") // yz: layered mapf support only SpaceTimeAstar now
                ;

        po::variables_map vm;
        // construct fake argument
        int fake_argc = 1;
        char arg0[] = "";
        char **fake_argv = new char *[fake_argc + 1]{arg0};
        po::store(po::parse_command_line(fake_argc, fake_argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << desc << std::endl;
            return {};
        }

        po::notify(vm);
        /////////////////////////////////////////////////////////////////////////
        /// check the correctness and consistence of params
        //////////////////////////////////////////////////////////////////////
        CBSH2_RTC::heuristics_type h;
        if (vm["heuristics"].as<std::string>() == "Zero")
            h = CBSH2_RTC::heuristics_type::ZERO;
        else if (vm["heuristics"].as<std::string>() == "CG")
            h = CBSH2_RTC::heuristics_type::CG;
        else if (vm["heuristics"].as<std::string>() == "DG")
            h = CBSH2_RTC::heuristics_type::DG;
        else if (vm["heuristics"].as<std::string>() == "WDG")
            h = CBSH2_RTC::heuristics_type::WDG;
        else
        {
            std::cout << "WRONG heuristics strategy!" << std::endl;
            return {};
        }

        CBSH2_RTC::rectangle_strategy r;
        if (vm["rectangleReasoning"].as<std::string>() == "None")
            r = CBSH2_RTC::rectangle_strategy::NR;  // no rectangle reasoning
        else if (vm["rectangleReasoning"].as<std::string>() == "R")
            r = CBSH2_RTC::rectangle_strategy::R;  // rectangle reasoning for entire paths
        else if (vm["rectangleReasoning"].as<std::string>() == "RM")
            r = CBSH2_RTC::rectangle_strategy::RM;  // rectangle reasoning for path segments
        else if (vm["rectangleReasoning"].as<std::string>() == "GR")
            r = CBSH2_RTC::rectangle_strategy::GR;  // generalized rectangle reasoning
        else if (vm["rectangleReasoning"].as<std::string>() == "Disjoint")
            r = CBSH2_RTC::rectangle_strategy::DR; // disjoint rectangle reasoning
        else
        {
            std::cout << "WRONG rectangle reasoning strategy!" << std::endl;
            return {};
        }

        CBSH2_RTC::corridor_strategy c;
        if (vm["corridorReasoning"].as<std::string>() == "None")
            c = CBSH2_RTC::corridor_strategy::NC;  // no corridor reasoning
        else if (vm["corridorReasoning"].as<std::string>() == "C")
            c = CBSH2_RTC::corridor_strategy::C;  // corridor reasoning
        else if (vm["corridorReasoning"].as<std::string>() == "PC")
            c = CBSH2_RTC::corridor_strategy::PC;  // corridor + pseudo-corridor reasoning
        else if (vm["corridorReasoning"].as<std::string>() == "STC")
            c = CBSH2_RTC::corridor_strategy::STC;  // corridor with start-target reasoning
        else if (vm["corridorReasoning"].as<std::string>() == "GC")
            c = CBSH2_RTC::corridor_strategy::GC;  // generalized corridor reasoning = corridor with start-target + pseudo-corridor
        else if (vm["corridorReasoning"].as<std::string>() == "Disjoint")
            c = CBSH2_RTC::corridor_strategy::DC; // disjoint corridor reasoning
        else
        {
            std::cout << "WRONG corridor reasoning strategy!" << std::endl;
            return {};
        }


        ///////////////////////////////////////////////////////////////////////////
        /// load the instance
        //////////////////////////////////////////////////////////////////////
//        CBSH2_RTC::Instance instance(vm["map"].as<std::string>(), vm["agents"].as<std::string>(),
//                                     vm["agentNum"].as<int>(), vm["agentIdx"].as<std::string>(),
//                                     vm["rows"].as<int>(), vm["cols"].as<int>(), vm["obs"].as<int>(), vm["warehouseWidth"].as<int>());
        CBSH2_RTC::Instance instance(dim, isoc, instance_sat);

        srand(vm["seed"].as<int>());

        int runs = vm["restart"].as<int>();


        //////////////////////////////////////////////////////////////////////
        /// initialize the solver
        //////////////////////////////////////////////////////////////////////
        CBSH2_RTC::CBS cbs(instance, ct, vm["sipp"].as<bool>(), vm["screen"].as<int>());
        cbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
        cbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
        cbs.setBypass(vm["bypass"].as<bool>());
        cbs.setRectangleReasoning(r);
        cbs.setCorridorReasoning(c);
        cbs.setHeuristicType(h);
        cbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
        cbs.setMutexReasoning(vm["mutexReasoning"].as<bool>());
        cbs.setSavingStats(vm["stats"].as<bool>());
        cbs.setNodeLimit(vm["nodeLimit"].as<int>());


        //////////////////////////////////////////////////////////////////////
        /// run
        //////////////////////////////////////////////////////////////////////
        double runtime = 0;
        int min_f_val = 0;
        for (int i = 0; i < runs; i++)
        {
            //std::cout << " std::cout << \"\"" << vm["cutoffTime"].as<double>() << std::endl;
            cbs.clear();
            cbs.solve(vm["cutoffTime"].as<double>(), min_f_val);
            runtime += cbs.runtime;
            if (cbs.solution_found)
                break;
            min_f_val = (int) cbs.min_f_val;
            cbs.randomRoot = true;
        }
        cbs.runtime = runtime;

        //////////////////////////////////////////////////////////////////////
        /// write results to files
        //////////////////////////////////////////////////////////////////////
        if (vm.count("output"))
            cbs.saveResults(vm["output"].as<std::string>(), vm["agents"].as<std::string>()+":"+ vm["agentIdx"].as<std::string>());
        // cbs.saveCT(vm["output"].as<string>() + ".tree"); // for debug
        if (vm["stats"].as<bool>())
        {
            cbs.saveStats(vm["output"].as<std::string>(), vm["agents"].as<std::string>() + ":" + vm["agentIdx"].as<std::string>());
        }
        if (cbs.solution_found && vm.count("outputPaths"))
            cbs.savePaths(vm["outputPaths"].as<std::string>());
        cbs.clearSearchEngines();
        cbs.savePaths();
        return cbs.getfreeNavPath();
    }

}

#endif //FREENAV_DRIVER_H
