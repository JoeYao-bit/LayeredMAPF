//
// Created by yaozhuo on 2024/1/17.
//

#ifndef FREENAV_MAPF_LNS_MAPF_DRIVER_H
#define FREENAV_MAPF_LNS_MAPF_DRIVER_H

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "MAPF-LNS2/inc/LNS.h"
#include "MAPF-LNS2/inc/AnytimeBCBS.h"
#include "MAPF-LNS2/inc/AnytimeEECBS.h"
#include "MAPF-LNS2/inc/PIBT/pibt.h"
#include "../../../freeNav-base/basic_elements/point.h"
#include "EECBS/inc/ConstraintTable.h"

namespace MAPF_LNS {

    freeNav::Paths<2> LNS_MAPF(freeNav::DimensionLength *dim,
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
                ("map,m", po::value<string>()->default_value(""), "input file for map")
                ("agents,a", po::value<string>()->default_value(""), "input file for agents")
                ("agentNum,k", po::value<int>()->default_value(0), "number of agents")
                ("output,o", po::value<string>(), "output file name (no extension)")
                ("outputPaths", po::value<string>(), "output file for paths")
                ("cutoffTime,t", po::value<double>()->default_value(cutoff_time/2), "cutoff time (seconds)")
                ("screen,s", po::value<int>()->default_value(0),
                 "screen option (0: none; 1: LNS results; 2:LNS detailed results; 3: MAPF detailed results)")
                ("stats", po::value<string>(), "output stats file")

                // solver
                ("solver", po::value<string>()->default_value("LNS"), "solver (LNS, A-BCBS, A-EECBS)")
                ("sipp", po::value<bool>()->default_value(false), "Use SIPP as the single-agent solver")
                ("seed", po::value<int>()->default_value(0), "Random seed")

                // params for LNS
                ("initLNS", po::value<bool>()->default_value(true),
                 "use LNS to find initial solutions if the initial sovler fails")
                ("neighborSize", po::value<int>()->default_value(8), "Size of the neighborhood")
                ("maxIterations", po::value<int>()->default_value(0), "maximum number of iterations")
                ("initAlgo", po::value<string>()->default_value("EECBS"), // yz: PP，PPS，PIBT, winPIBT cause failed in layered mapf, need map file path
                 "MAPF algorithm for finding the initial solution (EECBS, PP, PPS, CBS, PIBT, winPIBT)")
                ("replanAlgo", po::value<string>()->default_value("PP"),
                 "MAPF algorithm for replanning (EECBS, CBS, PP)")
                ("destoryStrategy", po::value<string>()->default_value("Adaptive"),
                 "Heuristics for finding subgroups (Random, RandomWalk, Intersection, Adaptive)")
                ("pibtWindow", po::value<int>()->default_value(5),
                 "window size for winPIBT")
                ("winPibtSoftmode", po::value<bool>()->default_value(true),
                 "winPIBT soft mode")

                // params for initLNS
                ("initDestoryStrategy", po::value<string>()->default_value("Adaptive"),
                 "Heuristics for finding subgroups (Target, Collision, Random, Adaptive)")
                ;
        po::variables_map vm;
        // construct fake argument
        int fake_argc = 1;
        char arg0[] = "";
        char **fake_argv = new char *[fake_argc + 1]{arg0};
        po::store(po::parse_command_line(fake_argc, fake_argv, desc), vm);

        if (vm.count("help")) {
            cout << desc << endl;
            return {};
        }

        PIBTPPS_option pipp_option;
        pipp_option.windowSize = vm["pibtWindow"].as<int>();
        pipp_option.winPIBTSoft = vm["winPibtSoftmode"].as<bool>();

        po::notify(vm);

        srand((int)time(0));
        delete[] fake_argv;
//        Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
//                          vm["agentNum"].as<int>());
        Instance instance(dim, isoc, instance_sat);

        double time_limit = vm["cutoffTime"].as<double>();
        int screen = vm["screen"].as<int>();
        srand(vm["seed"].as<int>());

        LNS lns(instance, time_limit,
                vm["initAlgo"].as<string>(),
                vm["replanAlgo"].as<string>(),
                vm["destoryStrategy"].as<string>(),
                vm["neighborSize"].as<int>(),
                vm["maxIterations"].as<int>(),
                vm["initLNS"].as<bool>(),
                vm["initDestoryStrategy"].as<string>(),
                vm["sipp"].as<bool>(),
                screen, pipp_option, ct);
        bool succ = lns.run();
        if (succ)
        {
            lns.validateSolution();
            if (vm.count("outputPaths"))
                lns.writePathsToFile(vm["outputPaths"].as<string>());
        }
        if (vm.count("output"))
            lns.writeResultToFile(vm["output"].as<string>());
        if (vm.count("stats"))
            lns.writeIterStatsToFile(vm["stats"].as<string>());
        if(succ) {
            lns.savePaths();
            // lns.writePathsToFile("path.txt");
            return lns.getfreeNavPath();
        } else {
            return {};
        }
    }

    freeNav::Paths<2> AnytimeBCBS_MAPF(freeNav::DimensionLength *dim,
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
                ("map,m", po::value<string>()->default_value(""), "input file for map")
                ("agents,a", po::value<string>()->default_value(""), "input file for agents")
                ("agentNum,k", po::value<int>()->default_value(0), "number of agents")
                ("output,o", po::value<string>(), "output file name (no extension)")
                ("outputPaths", po::value<string>(), "output file for paths")
                ("cutoffTime,t", po::value<double>()->default_value(cutoff_time), "cutoff time (seconds)")
                ("screen,s", po::value<int>()->default_value(0),
                 "screen option (0: none; 1: LNS results; 2:LNS detailed results; 3: MAPF detailed results)")
                ("stats", po::value<string>(), "output stats file")

                // solver
                ("solver", po::value<string>()->default_value("A-BCBS"), "solver (LNS, A-BCBS, A-EECBS)")
                ("sipp", po::value<bool>()->default_value(false), "Use SIPP as the single-agent solver")
                ("seed", po::value<int>()->default_value(0), "Random seed")

                // params for LNS
                ("initLNS", po::value<bool>()->default_value(true),
                 "use LNS to find initial solutions if the initial sovler fails")
                ("neighborSize", po::value<int>()->default_value(8), "Size of the neighborhood")
                ("maxIterations", po::value<int>()->default_value(0), "maximum number of iterations")
                ("initAlgo", po::value<string>()->default_value("EECBS"), // yz: PP，PPS，PIBT, winPIBT cause failed in layered mapf, need map file path
                 "MAPF algorithm for finding the initial solution (EECBS, PP, PPS, CBS, PIBT, winPIBT)")
                ("replanAlgo", po::value<string>()->default_value("PP"),
                 "MAPF algorithm for replanning (EECBS, CBS, PP)")
                ("destoryStrategy", po::value<string>()->default_value("Adaptive"),
                 "Heuristics for finding subgroups (Random, RandomWalk, Intersection, Adaptive)")
                ("pibtWindow", po::value<int>()->default_value(5),
                 "window size for winPIBT")
                ("winPibtSoftmode", po::value<bool>()->default_value(true),
                 "winPIBT soft mode")

                // params for initLNS
                ("initDestoryStrategy", po::value<string>()->default_value("Adaptive"),
                 "Heuristics for finding subgroups (Target, Collision, Random, Adaptive)")
                ;
        po::variables_map vm;
        // construct fake argument
        int fake_argc = 1;
        char arg0[] = "";
        char **fake_argv = new char *[fake_argc + 1]{arg0};
        po::store(po::parse_command_line(fake_argc, fake_argv, desc), vm);

        if (vm.count("help")) {
            cout << desc << endl;
            return {};
        }

        PIBTPPS_option pipp_option;
        pipp_option.windowSize = vm["pibtWindow"].as<int>();
        pipp_option.winPIBTSoft = vm["winPibtSoftmode"].as<bool>();

        po::notify(vm);

        srand((int)time(0));
        delete fake_argv;
//        Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
//                          vm["agentNum"].as<int>());
        Instance instance(dim, isoc, instance_sat);

        double time_limit = vm["cutoffTime"].as<double>();
        int screen = vm["screen"].as<int>();
        srand(vm["seed"].as<int>());
        bool new_ct=false;
        if(ct == nullptr) {
            ct = new CBS_Li::ConstraintTable(instance.num_of_cols, instance.map_size);
            new_ct = true;
        }
        AnytimeBCBS bcbs(instance, time_limit, screen, ct);
        bcbs.run();
        bcbs.validateSolution();
//        if (vm.count("output"))
//            bcbs.writeResultToFile(vm["output"].as<string>() + ".csv");
//        if (vm.count("stats"))
//            bcbs.writeIterStatsToFile(vm["stats"].as<string>());

        bcbs.savePaths();
        // lns.writePathsToFile("path.txt");
        if(new_ct) { delete ct; }
        return bcbs.getfreeNavPath();
    }

    freeNav::Paths<2> AnytimeEECBS_MAPF(freeNav::DimensionLength *dim,
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
                ("map,m", po::value<string>()->default_value(""), "input file for map")
                ("agents,a", po::value<string>()->default_value(""), "input file for agents")
                ("agentNum,k", po::value<int>()->default_value(0), "number of agents")
                ("output,o", po::value<string>(), "output file name (no extension)")
                ("outputPaths", po::value<string>(), "output file for paths")
                ("cutoffTime,t", po::value<double>()->default_value(2*cutoff_time), "cutoff time (seconds)")
                ("screen,s", po::value<int>()->default_value(0),
                 "screen option (0: none; 1: LNS results; 2:LNS detailed results; 3: MAPF detailed results)")
                ("stats", po::value<string>(), "output stats file")

                // solver
                ("solver", po::value<string>()->default_value("A-EECBS"), "solver (LNS, A-BCBS, A-EECBS)")
                ("sipp", po::value<bool>()->default_value(false), "Use SIPP as the single-agent solver")
                ("seed", po::value<int>()->default_value(0), "Random seed")

                // params for LNS
                ("initLNS", po::value<bool>()->default_value(true),
                 "use LNS to find initial solutions if the initial sovler fails")
                ("neighborSize", po::value<int>()->default_value(8), "Size of the neighborhood")
                ("maxIterations", po::value<int>()->default_value(0), "maximum number of iterations")
                ("initAlgo", po::value<string>()->default_value("EECBS"), // yz: PP，PPS，PIBT, winPIBT cause failed in layered mapf, need map file path
                 "MAPF algorithm for finding the initial solution (EECBS, PP, PPS, CBS, PIBT, winPIBT)")
                ("replanAlgo", po::value<string>()->default_value("PP"),
                 "MAPF algorithm for replanning (EECBS, CBS, PP)")
                ("destoryStrategy", po::value<string>()->default_value("Adaptive"),
                 "Heuristics for finding subgroups (Random, RandomWalk, Intersection, Adaptive)")
                ("pibtWindow", po::value<int>()->default_value(5),
                 "window size for winPIBT")
                ("winPibtSoftmode", po::value<bool>()->default_value(true),
                 "winPIBT soft mode")

                // params for initLNS
                ("initDestoryStrategy", po::value<string>()->default_value("Adaptive"),
                 "Heuristics for finding subgroups (Target, Collision, Random, Adaptive)")
                ;
        po::variables_map vm;
        // construct fake argument
        int fake_argc = 1;
        char arg0[] = "";
        char **fake_argv = new char *[fake_argc + 1]{arg0};
        po::store(po::parse_command_line(fake_argc, fake_argv, desc), vm);

        if (vm.count("help")) {
            cout << desc << endl;
            return {};
        }

        PIBTPPS_option pipp_option;
        pipp_option.windowSize = vm["pibtWindow"].as<int>();
        pipp_option.winPIBTSoft = vm["winPibtSoftmode"].as<bool>();

        po::notify(vm);

        srand((int)time(0));
        delete fake_argv;
//        Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
//                          vm["agentNum"].as<int>());
        CBS_Li::Instance instance(dim, isoc, instance_sat);

        double time_limit = vm["cutoffTime"].as<double>();
        int screen = vm["screen"].as<int>();
        srand(vm["seed"].as<int>());
        bool new_ct=false;
        if(ct == nullptr) {
            ct = new CBS_Li::ConstraintTable(instance.num_of_cols, instance.map_size);
            new_ct = true;
        }
        AnytimeEECBS eecbs(instance, time_limit, screen, ct);
        eecbs.run();
        eecbs.validateSolution();
        if (vm.count("output"))
            eecbs.writeResultToFile(vm["output"].as<string>() + ".csv");
        if (vm.count("stats"))
            eecbs.writeIterStatsToFile(vm["stats"].as<string>());

        // lns.writePathsToFile("path.txt");
        if(!eecbs.solution.empty()) eecbs.savePaths();
        if(new_ct) { delete ct; }
        return eecbs.getfreeNavPath();
    }

}
#endif //FREENAV_DRIVER_H
