#include "Picat.h"

using namespace std;
namespace Hybird_MAPF {

    PicatSolver::PicatSolver(Instance *in, int cf) {
        // parameters
        solver_input = "picat_instance.pi";
        solver_output = "picat_solution.out";

        cost_function = cf;

        if (cost_function == 1)
            solver_name = "mks";
        if (cost_function == 2)
            solver_name = "soc";

        inst = in;
    }

    void PicatSolver::Solve(vector<int> &agents_to_plan, vector<vector<int> > &agents_to_avoid, int Cost,
                            long long timeout_ms) {
        ofstream picat;
        picat.open(solver_input);
        if (!picat.is_open())
            cout << "fail" << endl;
        picat << "ins(Graph, As, Avoid, Makespan, SumOfCosts) =>" << endl;
        picat << "    Graph = [" << endl;
        for (size_t i = 0; i < inst->graph.size(); i++) {
            if (i != 0)
                picat << "," << endl;
            picat << "    $neibs(" << i + 1 << ",[" << i + 1;

            for (size_t j = 0; j < inst->graph[i].size(); j++)
                picat << "," << inst->graph[i][j] + 1;

            picat << "])";
        }

        picat << endl << "    ]," << endl;
        picat << "    As = [";
        for (size_t i = 0; i < agents_to_plan.size(); i++) {
            if (i != 0)
                picat << ",";
            picat << "(" << inst->start[agents_to_plan[i]] + 1 << "," << inst->goal[agents_to_plan[i]] + 1 << ")";
        }

        picat << "]," << endl;

        // avoid
        if (Cost < 0) {
            picat << "    Avoid = new_array(0,0)," << endl;
            picat << "    Makespan = -1," << endl;
            picat << "    SumOfCosts = -1." << endl;
        } else if (cost_function == 1) // makespan optimal
        {
            picat << "    Avoid = new_array(" << Cost << "," << agents_to_avoid.size() << ")," << endl;
            for (size_t i = 0; i < agents_to_avoid.size(); i++) {
                for (size_t j = 0; j < Cost; j++) {
                    picat << "    Avoid[" << j + 1 << "," << i + 1 << "] = " << agents_to_avoid[i][j] + 1 << ","
                          << endl;
                }
            }
            picat << "    Makespan = " << Cost << "," << endl;
            picat << "    SumOfCosts = -1." << endl;
        } else // SoC optimal
        {
            int max_timestep_to_avoid = 0;
            for (size_t i = 0; i < agents_to_avoid.size(); i++) {
                for (size_t j = agents_to_avoid[i].size() - 1; j > 0; j--) {
                    if (agents_to_avoid[i][j] != agents_to_avoid[i][j - 1]) {
                        max_timestep_to_avoid = max(max_timestep_to_avoid, (int) j + 1);
                        break;
                    }
                }
            }

            int mksLB = 0;
            int socLB = 0;
            for (size_t i = 0; i < agents_to_plan.size(); i++) {
                mksLB = max(mksLB, inst->distance[inst->start[agents_to_plan[i]]][inst->goal[agents_to_plan[i]]]) + 1;
                socLB += inst->distance[inst->start[agents_to_plan[i]]][inst->goal[agents_to_plan[i]]];
            }
            int makespan = max(max_timestep_to_avoid, mksLB + Cost - socLB);

            picat << "    Avoid = new_array(" << makespan << "," << agents_to_avoid.size() << ")," << endl;
            for (size_t i = 0; i < agents_to_avoid.size(); i++) {
                for (size_t j = 0; j < max_timestep_to_avoid; j++) {
                    picat << "    Avoid[" << j + 1 << "," << i + 1 << "] = " << agents_to_avoid[i][j] + 1 << ","
                          << endl;
                }
                for (size_t j = max_timestep_to_avoid; j < makespan; j++) {
                    picat << "    Avoid[" << j + 1 << "," << i + 1 << "] = "
                          << agents_to_avoid[i][agents_to_avoid[i].size() - 1] + 1 << "," << endl;
                }
            }

            picat << "    Makespan = " << makespan << "," << endl;
            picat << "    SumOfCosts = " << Cost << "." << endl;
        }

        picat.close();

        // solve by picat
        stringstream ss_exec;
        ss_exec << "bash -c \'timeout " << (int) (timeout_ms / 1000) << " ./picat " << solver_name << " "
                << solver_input << "' > " << solver_output << " 2>&1";
        string execute = ss_exec.str();
        cout << execute << endl;
        system(execute.c_str());
    }


    int PicatSolver::ReadResults(vector<vector<int> > &plan, int cost) {
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
                // no solution
                if (line.compare("*** error(failed,main/1)") == 0) {
                    input.close();
                    return 1;
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
                    int location;
                    single_agent_plan >> location;
                    plan[i][j] = location - 1;
                }
            }
            input.close();

            return 0;

        }

        return -1;
    }
}