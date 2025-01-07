#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <sstream>
#include <algorithm>
#include <iterator>
#ifndef INSTACE_H
#define INSTACE_H
#include "../algorithm/basic.h"
namespace Hybird_MAPF {


    class Instance {
    public:
        Instance() {}

        Instance(freeNav::DimensionLength *dim, const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                 const freeNav::Instances<2> &instance_sat) {}

        void ReadInput(std::string);

        void IncreaseAgentsNumber();

        void ResetAgentsNumber(int = 1);

        void ParseMap(std::string);

        std::string input_filename;
        std::string statistic_file;

        int timeout;        //timeout in s
        int nodes; // yz: number of all passable
        int agents;

        int rows;
        int columns;

        std::vector<int> start; // yz: all start id in graph
        std::vector<int> goal; // yz: all target id in graph

        std::vector<std::vector<int> > graph;        // list of neighbours // yz: connection between node
        std::vector<std::vector<int> > int_graph;    // input grid // yz: map

        std::vector<std::vector<int> > distance;

        /* DEBUG */

        void PrintPlan(std::vector<std::vector<int> > &);

        void CheckPlan(std::vector<std::vector<int> > &);

        int GetPlanMakespan(std::vector<std::vector<int> > &);

        int GetPlanSoC(std::vector<std::vector<int> > &);
    };
}
#endif /* INSTACE_H */