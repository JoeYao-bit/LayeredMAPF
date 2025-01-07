#include "ICTS.h"

using namespace std;
namespace Hybird_MAPF {

    ICTSSolver::ICTSSolver(Instance *in, int cf) {
        // parameters
        solver_input = "icts_instance.in";
        solver_output = "icts_solution.out";
        solver_name = "icts.exe";

        cost_function = cf;

        inst = in;
    }

    void ICTSSolver::Solve(vector<int> &agents_to_plan, vector<vector<int> > &agents_to_avoid, int Cost,
                           long long timeout_ms) {
        ofstream ICTS;
        ICTS.open(solver_input);
        if (!ICTS.is_open())
            cout << "fail" << endl;

        // graph
        ICTS << "0" << endl << "Grid:" << endl;
        ICTS << inst->int_graph.size() << "," << inst->int_graph[0].size() << endl;
        for (size_t i = 0; i < inst->int_graph.size(); i++) {
            for (size_t j = 0; j < inst->int_graph[i].size(); j++) {
                if (inst->int_graph[i][j] == -1)
                    ICTS << "@";
                else
                    ICTS << ".";
            }
            ICTS << endl;
        }


        // agents
        ICTS << "Agents:" << endl << agents_to_plan.size() << endl;
        for (size_t i = 0; i < agents_to_plan.size(); i++) {
            ICTS << i << ",";
            // goal
            pair<int, int> goal = FindCoordinates(inst->int_graph, inst->goal[agents_to_plan[i]]);
            ICTS << goal.first << "," << goal.second << ",";
            // start
            pair<int, int> start = FindCoordinates(inst->int_graph, inst->start[agents_to_plan[i]]);
            ICTS << start.first << "," << start.second;

            ICTS << endl;
        }

        // probably not needed to print the agents to avoid
        /*for (size_t i = 0; i < agents_to_avoid.size(); i++)
        {
            CBS << i + agents_to_plan.size() << ",";
            // goal
            pair<int, int> goal = FindCoordinates(inst->int_graph, agents_to_avoid[i][agents_to_avoid[i].size() - 1]);
            CBS << goal.first << "," << goal.second << ",";
            // start
            pair<int, int> start = FindCoordinates(inst->int_graph, agents_to_avoid[i][0]);
            CBS << start.first << "," << start.second;

            CBS << endl;
        }*/

        // avoid - DOES NOT WOKR YET!!!!!
        ICTS << "CurrentPlan:" << endl;

        if (Cost < 0)
            ICTS << "0,0" << endl;
        else {
            ICTS << agents_to_avoid.size() << "," << agents_to_avoid[0].size() << endl;

            for (size_t i = 0; i < agents_to_avoid.size(); i++) {
                ICTS << i + agents_to_plan.size() << "#";
                for (size_t j = 0; j < agents_to_avoid[i].size(); j++) {
                    pair<int, int> position = FindCoordinates(inst->int_graph, agents_to_avoid[i][j]);
                    ICTS << ";" << position.first << "," << position.second;
                }
                ICTS << endl;
            }
        }

        ICTS.close();

        // solve by ICTS
        stringstream ss_exec;
        if (cost_function == 1) // makespan
            ss_exec << solver_name << " " << solver_input << " " << timeout_ms << " makespan > " << solver_output;
        else if (cost_function == 2) // SoC
            ss_exec << solver_name << " " << solver_input << " " << timeout_ms << " > " << solver_output;
        string execute = ss_exec.str();
        cout << execute << endl;
        system(execute.c_str());

    }

    int ICTSSolver::ReadResults(vector<vector<int> > &plan, int cost) {
        // read output
        string line;
        string s_dump;
        float elapsed_time;
        int agents, makespan;
        bool valid_solution = false;
        ifstream input(solver_output);
        if (input.is_open()) {
            while (getline(input, line)) {
                // ok solution
                if (line.compare("agents | timesteps") == 0) {
                    valid_solution = true;
                    break;
                }
            }

            // timeout or error
            if (!valid_solution) {
                input.close();
                return -1;
            }

            getline(input, line);
            stringstream ss(line);
            ss >> agents;
            ss >> makespan;

            plan = vector<vector<int> >(agents, vector<int>(makespan));

            for (int i = 0; i < agents; i++) {
                getline(input, line);
                stringstream single_agent_plan(line);
                for (int j = 0; j < makespan; j++) {
                    int x, y;
                    single_agent_plan >> x;
                    single_agent_plan >> y;
                    plan[i][j] = inst->int_graph[x][y];
                }
            }
            input.close();

            // check if the solution satisfies cost constraints
            // TODO: may be done in CBS to return only valid solutions

            int SoC = 0;

            for (size_t i = 0; i < plan.size(); i++) {
                for (size_t j = plan[i].size() - 1; j > 0; j--) {
                    if (plan[i][j] != plan[i][j - 1]) {
                        SoC += j;
                        break;
                    }
                }
            }

            if (cost != -1 && cost_function == 1 && makespan > cost)
                return 1;

            if (cost != -1 && cost_function == 2 && SoC > cost)
                return 1;

            return 0;

        }

        return -1;
    }

    pair<int, int> ICTSSolver::FindCoordinates(vector<vector<int> > graph, int node_id) {
        for (size_t i = 0; i < graph.size(); i++) {
            for (size_t j = 0; j < graph[i].size(); j++) {
                if (graph[i][j] == node_id)
                    return make_pair(i, j);
            }
        }
    }
}