#pragma once

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "ECBS.h"
#include <strstream>
#include "../../../algorithm/layered_mapf.h"

namespace CBS_Li {

//    freeNav::Paths<2> eecbs_demo(int argc, char **argv, const std::map<std::string, std::string> &map_test_config);
//
//
//    freeNav::Paths<2> eecbs_MAPF(freeNav::DimensionLength *dim,
//                                 const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
//                                 const freeNav::Instances<2> &instance_sat,
//                                 CBS_Li::ConstraintTable *ct,
//                                 int cutoff_time);



    freeNav::Paths<2> eecbs_demo(int argc, char **argv, const std::map<std::string, std::string> &map_test_config) {
        namespace po = boost::program_options;
        // Declare the supported options.
        po::options_description desc("Allowed options"); // 创建一个命令行描述
        desc.add_options() // 添加命令行参数描述,以key-value-desciption的形式添加
                ("help", "produce help message")

                // params for the input instance and experiment settings
                ("map,m", po::value<string>()->default_value(map_test_config.at("map_path")), "input file for map")
                ("agents,a", po::value<string>()->default_value(map_test_config.at("scene_path")),
                 "input file for agents")
                ("output,o",
                 po::value<string>()->default_value("/home/yaozhuo/code/free-nav/third_party/EECBS/test.csv"),
                 "output file for statistics")
                ("outputPaths",
                 po::value<string>()->default_value("/home/yaozhuo/code/free-nav/third_party/EECBS/paths.txt"),
                 "output file for paths")
                ("agentNum,k", po::value<int>()->default_value(atoi((map_test_config.at("agent_num")).c_str())),
                 "number of agents")
                ("cutoffTime,t", po::value<double>()->default_value(atof((map_test_config.at("cut_off_time")).c_str())),
                 "cutoff time (seconds)")
                ("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
                ("stats", po::value<bool>()->default_value(false), "write to files some detailed statistics")

                // params for CBS node selection strategies
                ("highLevelSolver", po::value<string>()->default_value("EES"),
                 "the high-level solver (A*, A*eps, EES, NEW)")
                ("lowLevelSolver", po::value<bool>()->default_value(true), "using suboptimal solver in the low level")
                ("inadmissibleH", po::value<string>()->default_value("Global"),
                 "inadmissible heuristics (Zero, Global, Path, Local, Conflict)")
                ("suboptimality", po::value<double>()->default_value(1.2), "suboptimality bound")

                // params for CBS improvement
                ("heuristics", po::value<string>()->default_value("WDG"),
                 "admissible heuristics for the high-level search (Zero, CG,DG, WDG)")
                ("prioritizingConflicts", po::value<bool>()->default_value(true),
                 "conflict prioirtization. If true, conflictSelection is used as a tie-breaking rule.")
                ("bypass", po::value<bool>()->default_value(true), "Bypass1")
                ("disjointSplitting", po::value<bool>()->default_value(false), "disjoint splitting")
                ("rectangleReasoning", po::value<bool>()->default_value(true), "rectangle reasoning")
                ("corridorReasoning", po::value<bool>()->default_value(true), "corridor reasoning")
                ("targetReasoning", po::value<bool>()->default_value(true), "target reasoning")
                ("sipp", po::value<bool>()->default_value(false), "using SIPPS as the low-level solver")
                ("restart", po::value<int>()->default_value(0), "rapid random restart times");
        po::variables_map vm;    //用来保存命令行的数据
        po::store(po::parse_command_line(argc, argv, desc), vm);     //读取输入的命令行参数

        if (vm.count("help")) {
            cout << desc << endl;
            return {};
        }

        po::notify(vm);
        if (vm["suboptimality"].as<double>() < 1) {
            cerr << "Suboptimal bound should be at least 1!" << endl;
            return {};
        }

        high_level_solver_type s;
        if (vm["highLevelSolver"].as<string>() == "A*")
            s = high_level_solver_type::ASTAR;
        else if (vm["highLevelSolver"].as<string>() == "A*eps")
            s = high_level_solver_type::ASTAREPS;
        else if (vm["highLevelSolver"].as<string>() == "EES")
            s = high_level_solver_type::EES;
        else if (vm["highLevelSolver"].as<string>() == "NEW")
            s = high_level_solver_type::NEW;
        else {
            cout << "WRONG high level solver!" << endl;
            return {};
        }

        if (s == high_level_solver_type::ASTAR && vm["suboptimality"].as<double>() > 1) {
            cerr << "A* cannot perform suboptimal search!" << endl;
            return {};
        }

        heuristics_type h;
        if (vm["heuristics"].as<string>() == "Zero")
            h = heuristics_type::ZERO;
        else if (vm["heuristics"].as<string>() == "CG")
            h = heuristics_type::CG;
        else if (vm["heuristics"].as<string>() == "DG")
            h = heuristics_type::DG;
        else if (vm["heuristics"].as<string>() == "WDG")
            h = heuristics_type::WDG;
        else {
            cout << "WRONG heuristics strategy!" << endl;
            return {};
        }

        if ((h == heuristics_type::CG || h == heuristics_type::DG) && vm["lowLevelSolver"].as<bool>()) {
            cerr << "CG or DG heuristics do not work with low level of suboptimal search!" << endl;
            return {};
        }

        heuristics_type h_hat; // inadmissible heuristics
        if (s == high_level_solver_type::ASTAR ||
            s == high_level_solver_type::ASTAREPS ||
            vm["inadmissibleH"].as<string>() == "Zero")
            h_hat = heuristics_type::ZERO;
        else if (vm["inadmissibleH"].as<string>() == "Global")
            h_hat = heuristics_type::GLOBAL;
        else if (vm["inadmissibleH"].as<string>() == "Path")
            h_hat = heuristics_type::PATH;
        else if (vm["inadmissibleH"].as<string>() == "Local")
            h_hat = heuristics_type::LOCAL;
        else if (vm["inadmissibleH"].as<string>() == "Conflict")
            h_hat = heuristics_type::CONFLICT;
        else {
            cout << "WRONG inadmissible heuristics strategy!" << endl;
            return {};
        }

        conflict_selection conflict = conflict_selection::EARLIEST;
        node_selection n = node_selection::NODE_CONFLICTPAIRS;


        srand((int) time(0));

        ///////////////////////////////////////////////////////////////////////////
        // load the instance
        Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
                          vm["agentNum"].as<int>());

        ConstraintTable ct(instance.num_of_cols, instance.map_size);

        srand(0);
        int runs = 1;// + vm["restart"].as<int>();
        //////////////////////////////////////////////////////////////////////
        // initialize the solver
        if (vm["lowLevelSolver"].as<bool>()) {
            ECBS ecbs(instance, ct, vm["sipp"].as<bool>(), vm["screen"].as<int>());
            ecbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
            ecbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
            ecbs.setBypass(vm["bypass"].as<bool>());
            ecbs.setRectangleReasoning(vm["rectangleReasoning"].as<bool>());
            ecbs.setCorridorReasoning(vm["corridorReasoning"].as<bool>());
            ecbs.setHeuristicType(h, h_hat);
            ecbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
            ecbs.setMutexReasoning(false);
            ecbs.setConflictSelectionRule(conflict);
            ecbs.setNodeSelectionRule(n);
            ecbs.setSavingStats(vm["stats"].as<bool>());
            ecbs.setHighLevelSolver(s, vm["suboptimality"].as<double>());
            //////////////////////////////////////////////////////////////////////
            // run
            double runtime = 0;
            int lowerbound = 0;
            for (int i = 0; i < runs; i++) {
                ecbs.clear();
                ecbs.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
                runtime += ecbs.runtime;
                if (ecbs.solution_found)
                    break;
                lowerbound = ecbs.getLowerBound();
                ecbs.randomRoot = true;
                cout << "Failed to find solutions in Run " << i << endl;
            }
            ecbs.runtime = runtime;
            if (vm.count("output"))
                ecbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
            if (ecbs.solution_found && vm.count("outputPaths"))
                ecbs.savePaths(vm["outputPaths"].as<string>());
            /*size_t pos = vm["output"].as<string>().rfind('.');      // position of the file extension
            string output_name = vm["output"].as<string>().substr(0, pos);     // get the name without extension
            cbs.saveCT(output_name); // for debug*/
            if (vm["stats"].as<bool>())
                ecbs.saveStats(vm["output"].as<string>(), vm["agents"].as<string>());
            ecbs.clearSearchEngines();
            ecbs.saveCT(map_test_config.at("ct_path"));
            return ecbs.getfreeNavPath();
        } else {
            CBS cbs(instance, ct, vm["sipp"].as<bool>(), vm["screen"].as<int>());
            cbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
            cbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
            cbs.setBypass(vm["bypass"].as<bool>());
            cbs.setRectangleReasoning(vm["rectangleReasoning"].as<bool>());
            cbs.setCorridorReasoning(vm["corridorReasoning"].as<bool>());
            cbs.setHeuristicType(h, h_hat);
            cbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
            cbs.setMutexReasoning(false);
            cbs.setConflictSelectionRule(conflict);
            cbs.setNodeSelectionRule(n);
            cbs.setSavingStats(vm["stats"].as<bool>());
            cbs.setHighLevelSolver(s, vm["suboptimality"].as<double>());
            //////////////////////////////////////////////////////////////////////
            // run
            double runtime = 0;
            int lowerbound = 0;
            for (int i = 0; i < runs; i++) {
                cbs.clear();
                cbs.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
                runtime += cbs.runtime;
                if (cbs.solution_found)
                    break;
                lowerbound = cbs.getLowerBound();
                cbs.randomRoot = true;
                cout << "Failed to find solutions in Run " << i << endl;
            }
            cbs.runtime = runtime;
            if (vm.count("output"))
                cbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
            if (cbs.solution_found && vm.count("outputPaths"))
                cbs.savePaths(vm["outputPaths"].as<string>());
            if (vm["stats"].as<bool>())
                cbs.saveStats(vm["output"].as<string>(), vm["agents"].as<string>());
            cbs.clearSearchEngines();
            cbs.saveCT(map_test_config.at("ct_path"));
            return cbs.getfreeNavPath();
        }
    }


    freeNav::Paths<2> eecbs_MAPF(freeNav::DimensionLength *dim,
                                 const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                                 const freeNav::Instances<2> &instance_sat,
                                 CBS_Li::ConstraintTable *ct,
                                 int cutoff_time) {

        namespace po = boost::program_options;
        // Declare the supported options.
        po::options_description desc("Allowed options"); // 创建一个命令行描述
        desc.add_options() // 添加命令行参数描述,以key-value-desciption的形式添加
                ("help", "produce help message")

                // params for the input instance and experiment settings
                //("map,m", po::value<string>()->default_value(map_test_config.at("map_path")), "input file for map")
                //("agents,a", po::value<string>()->default_value(map_test_config.at("scene_path")), "input file for agents")

                ("output,o",
                 po::value<string>()->default_value("../third_party/EECBS/test.csv"),
                 "output file for statistics")
                ("outputPaths",
                 po::value<string>()->default_value("../third_party/EECBS/paths.txt"),
                 "output file for paths")

                ("agentNum,k", po::value<int>()->default_value(instance_sat.size()), "number of agents")
                ("cutoffTime,t", po::value<double>()->default_value(2*cutoff_time),
                 "cutoff time (seconds)") // yz: the real cut off time is halfed

                ("screen,s", po::value<int>()->default_value(0), "screen option (0: none; 1: results; 2:all)")
                ("stats", po::value<bool>()->default_value(false), "write to files some detailed statistics")

                // params for CBS node selection strategies
                ("highLevelSolver", po::value<string>()->default_value("EES"),
                 "the high-level solver (A*, A*eps, EES, NEW)")
                ("lowLevelSolver", po::value<bool>()->default_value(true), "using suboptimal solver in the low level")
                ("inadmissibleH", po::value<string>()->default_value("Global"),
                 "inadmissible heuristics (Zero, Global, Path, Local, Conflict)")
                ("suboptimality", po::value<double>()->default_value(1.2), "suboptimality bound")

                // params for CBS improvement
                ("heuristics", po::value<string>()->default_value("WDG"),
                 "admissible heuristics for the high-level search (Zero, CG,DG, WDG)")
                ("prioritizingConflicts", po::value<bool>()->default_value(false),
                 "conflict prioirtization. If true, conflictSelection is used as a tie-breaking rule.")
                ("bypass", po::value<bool>()->default_value(false), "Bypass1")
                ("disjointSplitting", po::value<bool>()->default_value(false), "disjoint splitting")
                ("rectangleReasoning", po::value<bool>()->default_value(false), "rectangle reasoning")
                ("corridorReasoning", po::value<bool>()->default_value(false), "corridor reasoning")
                ("targetReasoning", po::value<bool>()->default_value(false), "target reasoning")
                ("sipp", po::value<bool>()->default_value(false), "using SIPPS as the low-level solver")
                ("restart", po::value<int>()->default_value(0), "rapid random restart times");

        po::variables_map vm;    //用来保存命令行的数据
        // construct fake argument
        int fake_argc = 1;
        char arg0[] = "";
        char **fake_argv = new char *[fake_argc + 1]{arg0};
        po::store(po::parse_command_line(fake_argc, fake_argv, desc), vm);     //读取输入的命令行参数
        //po::store(po::parse_command_line(argc, argv, desc), vm);     //读取输入的命令行参数
        if (vm.count("help")) {
            cout << desc << endl;
            return {};
        }
        po::notify(vm);
        if (vm["suboptimality"].as<double>() < 1) {
            cerr << "Suboptimal bound should be at least 1!" << endl;
            return {};
        }

        high_level_solver_type s;
        if (vm["highLevelSolver"].as<string>() == "A*")
            s = high_level_solver_type::ASTAR;
        else if (vm["highLevelSolver"].as<string>() == "A*eps")
            s = high_level_solver_type::ASTAREPS;
        else if (vm["highLevelSolver"].as<string>() == "EES")
            s = high_level_solver_type::EES;
        else if (vm["highLevelSolver"].as<string>() == "NEW")
            s = high_level_solver_type::NEW;
        else {
            cout << "WRONG high level solver!" << endl;
            return {};
        }

        if (s == high_level_solver_type::ASTAR && vm["suboptimality"].as<double>() > 1) {
            cerr << "A* cannot perform suboptimal search!" << endl;
            return {};
        }

        heuristics_type h;
        if (vm["heuristics"].as<string>() == "Zero")
            h = heuristics_type::ZERO;
        else if (vm["heuristics"].as<string>() == "CG")
            h = heuristics_type::CG;
        else if (vm["heuristics"].as<string>() == "DG")
            h = heuristics_type::DG;
        else if (vm["heuristics"].as<string>() == "WDG")
            h = heuristics_type::WDG;
        else {
            cout << "WRONG heuristics strategy!" << endl;
            return {};
        }

        if ((h == heuristics_type::CG || h == heuristics_type::DG) && vm["lowLevelSolver"].as<bool>()) {
            cerr << "CG or DG heuristics do not work with low level of suboptimal search!" << endl;
            return {};
        }

        heuristics_type h_hat; // inadmissible heuristics
        if (s == high_level_solver_type::ASTAR ||
            s == high_level_solver_type::ASTAREPS ||
            vm["inadmissibleH"].as<string>() == "Zero")
            h_hat = heuristics_type::ZERO;
        else if (vm["inadmissibleH"].as<string>() == "Global")
            h_hat = heuristics_type::GLOBAL;
        else if (vm["inadmissibleH"].as<string>() == "Path")
            h_hat = heuristics_type::PATH;
        else if (vm["inadmissibleH"].as<string>() == "Local")
            h_hat = heuristics_type::LOCAL;
        else if (vm["inadmissibleH"].as<string>() == "Conflict")
            h_hat = heuristics_type::CONFLICT;
        else {
            cout << "WRONG inadmissible heuristics strategy!" << endl;
            return {};
        }

        conflict_selection conflict = conflict_selection::EARLIEST;
        node_selection n = node_selection::NODE_CONFLICTPAIRS;


        srand((int) time(0));

        ///////////////////////////////////////////////////////////////////////////
        // load the instance

//    Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
//                      vm["agentNum"].as<int>());

        // create instance from freeNav style data
        //std::cout << "instance_eecbs " << instance_sat << std::endl;
        Instance instance(dim, isoc, instance_sat);
        // construct constraint table from previous path
//        if (previous_paths.empty()) {
//            if (ct != nullptr) { delete ct; ct = nullptr; }
//            ct = new ConstraintTable(instance.num_of_cols, instance.map_size);
//        }
//        if(ct == nullptr) {
//            ct = new ConstraintTable(instance.num_of_cols, instance.map_size);
//        }
        //std::cout << " finish instance " << std::endl;
        bool new_ct = false;
        if(ct == nullptr) {
            ct = new ConstraintTable(instance.num_of_cols, instance.map_size);
            new_ct = true;
        }
        srand(0);
        int runs = 1;// + vm["restart"].as<int>();
        //////////////////////////////////////////////////////////////////////
        // initialize the solver
        if (vm["lowLevelSolver"].as<bool>()) {
            ECBS ecbs(instance, *ct, vm["sipp"].as<bool>(), vm["screen"].as<int>());
            ecbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
            ecbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
            ecbs.setBypass(vm["bypass"].as<bool>());
            ecbs.setRectangleReasoning(vm["rectangleReasoning"].as<bool>());
            ecbs.setCorridorReasoning(vm["corridorReasoning"].as<bool>());
            ecbs.setHeuristicType(h, h_hat);
            ecbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
            ecbs.setMutexReasoning(false);
            ecbs.setConflictSelectionRule(conflict);
            ecbs.setNodeSelectionRule(n);
            ecbs.setSavingStats(vm["stats"].as<bool>());
            ecbs.setHighLevelSolver(s, vm["suboptimality"].as<double>());
            //////////////////////////////////////////////////////////////////////
            // run
            double runtime = 0;
            int lowerbound = 0;
            for (int i = 0; i < runs; i++) {
                ecbs.clear();
                ecbs.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
                runtime += ecbs.runtime;
                if (ecbs.solution_found)
                    break;
                lowerbound = ecbs.getLowerBound();
                ecbs.randomRoot = true;
                cout << "Failed to find solutions in Run " << i << endl;
            }
            ecbs.runtime = runtime;
//        if (vm.count("output"))
//            ecbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
//        if (ecbs.solution_found && vm.count("outputPaths"))
//            ecbs.savePaths(vm["outputPaths"].as<string>());
            /*size_t pos = vm["output"].as<string>().rfind('.');      // position of the file extension
            string output_name = vm["output"].as<string>().substr(0, pos);     // get the name without extension
            cbs.saveCT(output_name); // for debug*/
//        if (vm["stats"].as<bool>())
//            ecbs.saveStats(vm["output"].as<string>(), vm["agents"].as<string>());
            if(ecbs.solution_found) { ecbs.savePaths(); }
            ecbs.clearSearchEngines();
            if (ecbs.solution_found) {
                for (const auto &previous_path : ecbs.getfreeNavPath()) {
                    MAPFPath path_eecbs;
                    for (int i = 0; i < previous_path.size(); i++) {
                        path_eecbs.push_back(
                                PathEntry(instance.num_of_cols * previous_path[i][1] + previous_path[i][0]));
                    }
                    //ct->insert2CT(path_eecbs);
                }
            }
            delete [] fake_argv;
            if(new_ct) { delete ct; }
//            if(ct != nullptr) {
//                delete ct;
//                ct = nullptr;
//            }
            return ecbs.solution_found ? ecbs.getfreeNavPath() : freeNav::Paths<2>();
        } else {
            CBS cbs(instance, *ct, vm["sipp"].as<bool>(), vm["screen"].as<int>());
            cbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
            cbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
            cbs.setBypass(vm["bypass"].as<bool>());
            cbs.setRectangleReasoning(vm["rectangleReasoning"].as<bool>());
            cbs.setCorridorReasoning(vm["corridorReasoning"].as<bool>());
            cbs.setHeuristicType(h, h_hat);
            cbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
            cbs.setMutexReasoning(false);
            cbs.setConflictSelectionRule(conflict);
            cbs.setNodeSelectionRule(n);
            cbs.setSavingStats(vm["stats"].as<bool>());
            cbs.setHighLevelSolver(s, vm["suboptimality"].as<double>());
            //////////////////////////////////////////////////////////////////////
            // run
            double runtime = 0;
            int lowerbound = 0;
            for (int i = 0; i < runs; i++) {
                cbs.clear();
                cbs.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
                runtime += cbs.runtime;
                if (cbs.solution_found)
                    break;
                lowerbound = cbs.getLowerBound();
                cbs.randomRoot = true;
                cout << "Failed to find solutions in Run " << i << endl;
            }
            cbs.runtime = runtime;
            if (vm.count("output"))
                cbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
            if (cbs.solution_found && vm.count("outputPaths"))
                cbs.savePaths(vm["outputPaths"].as<string>());
            if (vm["stats"].as<bool>())
                cbs.saveStats(vm["output"].as<string>(), vm["agents"].as<string>());
            if(cbs.solution_found) { cbs.savePaths(); }
            cbs.clearSearchEngines();
            if (cbs.solution_found) {
                for (const auto &previous_path : cbs.getfreeNavPath()) {
                    MAPFPath path_eecbs;
                    for (int i = 0; i < previous_path.size(); i++) {
                        path_eecbs.push_back(
                                PathEntry(instance.num_of_cols * previous_path[i][1] + previous_path[i][0]));
                    }
                    //ct->insert2CT(path_eecbs);
                }
            }
            delete [] fake_argv;
            if(new_ct) { delete ct; }
//            if(ct != nullptr) {
//                delete ct;
//                ct = nullptr;
//            }
            return cbs.solution_found ? cbs.getfreeNavPath() : freeNav::Paths<2>();
        }
    }


}



























