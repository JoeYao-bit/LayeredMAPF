#pragma once
#include "grid-pathfinding/graph/include/graph.hpp"
#include <random>

#include "default_params.hpp"
#include "task.hpp"
#include "util.hpp"
#include "../../../freeNav-base/basic_elements/point.h" // yz: add for freeNav style interfaces
#include "EECBS/inc/ConstraintTable.h" // yz: introduce static constraint table

namespace PIBT_2 {

    using Config = std::vector<Node *>;  // < loc_0[t], loc_1[t], ... >
    using Configs = std::vector<Config>;

    extern bool solution_valid_flag;// yz: add to avoid halt

// check two configurations are same or not
    [[maybe_unused]] static bool sameConfig(const Config &config_i,
                                            const Config &config_j) {
        if (config_i.size() != config_j.size()) return false;
        const int size_i = config_i.size();
        for (int k = 0; k < size_i; ++k) {
            if (config_i[k] != config_j[k]) return false;
        }
        return true;
    }

    [[maybe_unused]] static int getPathCost(const path_pathfinding::Path &path) {
        int cost = path.size() - 1;
        auto itr = path.end() - 1;
        while (itr != path.begin() && *itr == *(itr - 1)) {
            --cost;
            --itr;
        }
        return cost;
    }

    class Problem {
    protected:
        std::string instance;  // instance name
        path_pathfinding::Graph *G;              // graph
        std::mt19937 *MT;      // seed
        Config config_s;       // initial configuration
        Config config_g;       // goal configuration
        int num_agents;        // number of agents
        int max_timestep;      // timestep limit
        int max_comp_time;     // comp_time limit, ms

        // utilities
        void halt(const std::string &msg) const;

        void warn(const std::string &msg) const;

    public:
        Problem() {};

        Problem(const std::string &_instance) : instance(_instance) {}

        Problem(std::string _instance, path_pathfinding::Graph *_G, std::mt19937 *_MT, Config _config_s,
                Config _config_g, int _num_agents, int _max_timestep,
                int _max_comp_time);

        // yz: freeNav style interfaces
        Problem(freeNav::DimensionLength *dim, const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                const freeNav::Instances<2> &instance_sat,
                CBS_Li::ConstraintTable* ct,
                int _max_timestep,
                int _max_comp_time, std::mt19937 *_MT);

        Problem(path_pathfinding::Graph * external_graph,
                const freeNav::Instances<2> &instance_sat, CBS_Li::ConstraintTable* ct,
                int _max_timestep, int _max_comp_time,
                std::mt19937 *_MT);

        // yz: generate only grid to avoid repeat in Layered PIBT
        static path_pathfinding::Grid * generateGridPtr(freeNav::DimensionLength *dim,
                                                         const freeNav::IS_OCCUPIED_FUNC<2> &isoc) {
            path_pathfinding::Grid* G = new path_pathfinding::Grid(); // yz: when to release resource ?
            auto raw_G = (path_pathfinding::Grid *)(G);
            raw_G->width = dim[0];
            raw_G->height = dim[1];
            G->V = Nodes(dim[0] * dim[1], nullptr);
            freeNav::Pointi<2> pt;
            for(int x=0; x<dim[0]; x++) {
                for(int y=0; y<dim[1]; y++) {
                    pt[0] = x, pt[1] = y;
                    int id = dim[0] * y + x;
                    if(!isoc(pt)) {
                        Node* v = new Node(id, x, y);
                        G->V[id] = v;
                    }
                }
            }
            // create edges
            for (int y = 0; y < dim[1]; ++y) {
                for (int x = 0; x < dim[0]; ++x) {
                    if (!G->existNode(x, y)) continue;
                    Node* v = G->getNode(x, y);
                    // left
                    if (G->existNode(x - 1, y)) v->neighbor.push_back(G->getNode(x - 1, y));
                    // right
                    if (G->existNode(x + 1, y)) v->neighbor.push_back(G->getNode(x + 1, y));
                    // up
                    if (G->existNode(x, y - 1)) v->neighbor.push_back(G->getNode(x, y - 1));
                    // down
                    if (G->existNode(x, y + 1)) v->neighbor.push_back(G->getNode(x, y + 1));
                }
            }
            return G;
        }

        ~Problem() {};

        path_pathfinding::Graph *getG() { return G; }

        int getNum() { return num_agents; }

        std::mt19937 *getMT() { return MT; }

        Node *getStart(int i) const;  // return start of a_i
        Node *getGoal(int i) const;   // return  goal of a_i
        Config getConfigStart() const { return config_s; };

        Config getConfigGoal() const { return config_g; };

        int getMaxTimestep() { return max_timestep; };

        int getMaxCompTime() { return max_comp_time; };

        std::string getInstanceFileName() { return instance; };

        void setMaxCompTime(const int t) { max_comp_time = t; }

        CBS_Li::ConstraintTable* ct_ = nullptr; // yz: introduce as static constraint created by previous paths
        bool using_external_grid_ = false; // yz: whether using external grid to save time, if is, de not delete it

    };

    class MAPF_Instance : public Problem {
    private:
        const bool instance_initialized;  // for memory manage

        // set starts and goals randomly
        void setRandomStartsGoals();

        // set well-formed instance
        void setWellFormedInstance();

    public:
        MAPF_Instance(const std::string &_instance);

        MAPF_Instance(MAPF_Instance *P, Config _config_s, Config _config_g,
                      int _max_comp_time, int _max_timestep);

        MAPF_Instance(MAPF_Instance *P, int _max_comp_time);

        MAPF_Instance(freeNav::DimensionLength *dim, const freeNav::IS_OCCUPIED_FUNC<2> &isoc,
                      const freeNav::Instances<2> &instance_sat, CBS_Li::ConstraintTable* ct,
                      int _max_timestep, int _max_comp_time,
                      std::mt19937 *_MT);

        MAPF_Instance(path_pathfinding::Graph * external_grid,
                      const freeNav::Instances<2> &instance_sat, CBS_Li::ConstraintTable* ct,
                      int _max_timestep, int _max_comp_time,
                      std::mt19937 *_MT);

        ~MAPF_Instance();

        bool isInitializedInstance() const { return instance_initialized; }

        // used when making new instance file
        void makeScenFile(const std::string &output_file);
    };

    class MAPD_Instance : public Problem {
    private:
        float task_frequency;
        int task_num;

        int current_timestep;  // current timestep
        Tasks TASKS_OPEN;
        Tasks TASKS_CLOSED;

        Nodes LOCS_PICKUP;             // candidates of pickup locations
        Nodes LOCS_DELIVERY;           // candidates of delivery locations
        Nodes LOCS_NONTASK_ENDPOINTS;  // endpoints, not necessary for PIBT
        Nodes LOCS_ENDPOINTS;          // pickup, delivery, nontasks

        bool specify_pickup_deliv_locs;

        void setupSpetialNodes();

    public:
        MAPD_Instance(const std::string &_instance);

        ~MAPD_Instance();

        void update();

        int getCurrentTimestep() const { return current_timestep; }

        float getTaskFrequency() const { return task_frequency; }

        float getTaskNum() const { return task_num; }

        Tasks getOpenTasks() { return TASKS_OPEN; }

        Tasks getClosedTasks() { return TASKS_CLOSED; }

        Nodes getEndpoints() { return LOCS_ENDPOINTS; }
    };
}