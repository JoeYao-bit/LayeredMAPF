#ifndef HYBIRD_MAPF_INSTACE_H
#define HYBIRD_MAPF_INSTACE_H \
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <sstream>
#include <algorithm>
#include <iterator>

#include "../algorithm/layered_mapf.h"
namespace Hybird_MAPF {


    class Instance {
    public:
        Instance() {}

        // yz: freeNav style interfaces
        Instance(freeNav::DimensionLength *dim, const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                 const freeNav::Instances<2> &instance_sat);

        void ReadInput(std::string);

        void IncreaseAgentsNumber();

        void ResetAgentsNumber(int = 1);

        void ParseMap(std::string);

        std::string input_filename;
        std::string statistic_file;

        int timeout;        //timeout in s // yz: according to other comment, there should be ms
        int nodes; // yz: number of all passable
        freeNav::Pointis<2> node_to_pt_map; // yz: node corresponding to which pt
        int agents;

        int rows;
        int columns;

        std::vector<int> start; // yz: all start id in graph
        std::vector<int> goal; // yz: all target id in graph

        std::vector<std::vector<int> > graph;        // list of neighbours // yz: connection between node
        std::vector<std::vector<int> > int_graph;    // input grid // yz: map

        std::vector<std::vector<int> > distance;

        // yz: only for freeNav style interfaces
        freeNav::DimensionLength *dim_;
        freeNav::IS_OCCUPIED_FUNC<2> isoc_;
        freeNav::Instances<2> instance_sat_;

        /* DEBUG */

        void PrintPlan(std::vector<std::vector<int> > &);

        bool CheckPlan(std::vector<std::vector<int> > &);

        int GetPlanMakespan(std::vector<std::vector<int> > &);

        int GetPlanSoC(std::vector<std::vector<int> > &);
    };
}
#endif /* INSTACE_H */