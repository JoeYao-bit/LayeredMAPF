/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, May 2020
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "PBS/inc/PBS.h"

namespace PBS_Li {

/* Main function */
    int main(int argc, char **argv) {
        namespace po = boost::program_options;
        // Declare the supported options.
        po::options_description desc("Allowed options");
        desc.add_options()
                ("help", "produce help message")

                // params for the input instance and experiment settings
                ("map,m", po::value<std::string>()->required(), "input file for map")
                ("agents,a", po::value<std::string>()->required(), "input file for agents")
                ("output,o", po::value<std::string>(), "output file for statistics")
                ("outputPaths", po::value<std::string>(), "output file for paths")
                ("agentNum,k", po::value<int>()->default_value(0), "number of agents")
                ("cutoffTime,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
                ("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
                ("stats", po::value<bool>()->default_value(false), "write to files some detailed statistics")

                ("sipp", po::value<bool>()->default_value(1), "using SIPP as the low-level solver");
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 1;
        }

        po::notify(vm);

        srand((int) time(0));

        ///////////////////////////////////////////////////////////////////////////
        // load the instance
        CBS_Li::Instance instance(vm["map"].as<std::string>(), vm["agents"].as<std::string>(),
                          vm["agentNum"].as<int>());
        auto ct = CBS_Li::ConstraintTable(instance.num_of_cols, instance.map_size);
        srand(0);
        PBS pbs(instance, ct, vm["sipp"].as<bool>(), vm["screen"].as<int>());
        // run
        double runtime = 0;
        pbs.solve(vm["cutoffTime"].as<double>());
        if (vm.count("output"))
            pbs.saveResults(vm["output"].as<std::string>(), vm["agents"].as<std::string>());
        if (pbs.solution_found && vm.count("outputPaths"))
            pbs.savePaths(vm["outputPaths"].as<std::string>());
        /*size_t pos = vm["output"].as<string>().rfind('.');      // position of the file extension
        string output_name = vm["output"].as<string>().substr(0, pos);     // get the name without extension
        cbs.saveCT(output_name); // for debug*/
        pbs.clearSearchEngines();

        return 0;

    }

}