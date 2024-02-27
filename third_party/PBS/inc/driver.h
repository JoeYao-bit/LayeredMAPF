//
// Created by yaozhuo on 2024/1/13.
//
#pragma once

#ifndef FREENAV_DRIVER_H
#define FREENAV_DRIVER_H

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "PBS/inc/PBS.h"
#include "../../../algorithm/layered_mapf.h"

namespace PBS_Li {


    freeNav::Paths<2> pbs_MAPF(freeNav::DimensionLength *dim,
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
                ("output,o", po::value<string>(), "output file for statistics")
                ("outputPaths", po::value<string>(), "output file for paths")
                ("agentNum,k", po::value<int>()->default_value(0), "number of agents")
                ("cutoffTime,t", po::value<double>()->default_value(cutoff_time), "cutoff time (seconds)")
                ("screen,s", po::value<int>()->default_value(0), "screen option (0: none; 1: results; 2:all)")
                ("stats", po::value<bool>()->default_value(false), "write to files some detailed statistics")

                ("sipp", po::value<bool>()->default_value(true), "using SIPP as the low-level solver");

        po::variables_map vm;
        // construct fake argument
        int fake_argc = 1;
        char arg0[] = "";
        char **fake_argv = new char *[fake_argc + 1]{arg0};
        po::store(po::parse_command_line(fake_argc, fake_argv, desc), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return {};
        }

        po::notify(vm);

        srand((int) time(0));

        ///////////////////////////////////////////////////////////////////////////
        // load the instance
        //Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
        //                  vm["agentNum"].as<int>());

        CBS_Li::Instance instance(dim, isoc, instance_sat);
        if(ct == nullptr) {
            ct = new CBS_Li::ConstraintTable(instance.num_of_cols, instance.map_size);
        }
        srand(0);
        PBS pbs(instance, *ct, vm["sipp"].as<bool>(), vm["screen"].as<int>());
        // run
        double runtime = 0;
        pbs.solve(vm["cutoffTime"].as<double>());
        if (vm.count("output"))
            pbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
        if (pbs.solution_found && vm.count("outputPaths"))
            pbs.savePaths(vm["outputPaths"].as<string>());
        /*size_t pos = vm["output"].as<string>().rfind('.');      // position of the file extension
        string output_name = vm["output"].as<string>().substr(0, pos);     // get the name without extension
        cbs.saveCT(output_name); // for debug*/
        pbs.clearSearchEngines();
        pbs.savePaths();
        return pbs.solution_found ? pbs.getfreeNavPath() : freeNav::Paths<2>();
    }

}
#endif //FREENAV_DRIVER_H
