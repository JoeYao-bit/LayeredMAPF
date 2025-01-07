#include "instance.h"

using namespace std;
namespace Hybird_MAPF {

    void Instance::ReadInput(string f) {
        // Parameters for solving
        statistic_file = "../statistics/stat.txt";
        timeout = 600000; // timeout in milliseconds // yz: 600s = 10min

        // Read input
        input_filename = f;
        bool map_loaded = false;
        ifstream in;
        in.open(input_filename);
        if (!in.is_open())
            return;

        char c_dump;
        string line;
        getline(in, line); // first line - version

        while (getline(in, line)) {
            stringstream ssline(line);
            string part;
            vector<string> parsed_line;
            while (getline(ssline, part, '\t')) {
                parsed_line.push_back(part);
            }

            if (!map_loaded) {
                ParseMap(parsed_line[1]);
                map_loaded = true;
            }

            start.push_back(int_graph[stoi(parsed_line[5])][stoi(parsed_line[4])]);
            goal.push_back(int_graph[stoi(parsed_line[7])][stoi(parsed_line[6])]);
        }

        in.close();

        agents = 0;
    }

    void Instance::ParseMap(string filename) {
        ifstream in;
        in.open(string("../instances/maps/").append(filename));
        if (!in.is_open())
            return;

        char c_dump;
        string s_dump;
        getline(in, s_dump); // first line - type

        in >> s_dump >> rows;
        in >> s_dump >> columns;
        in >> s_dump; // map

        // graph
        int_graph = vector<vector<int> >(rows, vector<int>(columns, -1)); // yz: default to -1
        nodes = 0;

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < columns; j++) {
                in >> c_dump;
                if (c_dump == '.') {
                    int_graph[i][j] = nodes; // yz: passable node have monotonically increasing node id
                    nodes++;
                }
            }
        }

        // yz: nodes = number of passable grids, graph: grid connect to which edges
        graph = vector<vector<int> >(nodes, vector<int>()); // yz: graph store which node are connected to this node

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < columns; j++) {
                if (int_graph[i][j] != -1) // yz: if node (i, j) is passable
                {
                    // yz: adding edge is two direction, so only need check in one direction
                    // down
                    if (i < rows - 1 && int_graph[i + 1][j] != -1) {
                        graph[int_graph[i][j]].push_back(int_graph[i + 1][j]);
                        graph[int_graph[i + 1][j]].push_back(int_graph[i][j]);
                    }

                    // left
                    if (j > 0 && int_graph[i][j - 1] != -1) {
                        graph[int_graph[i][j]].push_back(int_graph[i][j - 1]);
                        graph[int_graph[i][j - 1]].push_back(int_graph[i][j]);
                    }
                }
            }
        }

        in.close();
        // yz: each node's distance to other nodes
        distance = vector<vector<int> >(graph.size(), vector<int>(graph.size(), graph.size()));
    }

    void Instance::IncreaseAgentsNumber() {
        agents++;
    }

    void Instance::ResetAgentsNumber(int a) {
        agents = a;
    }

/* DEBUG */

    void Instance::PrintPlan(vector<vector<int> > &plan) {
        for (size_t i = 0; i < plan.size(); i++) {
            cout << "agent " << i << ": ";
            for (size_t j = 0; j < plan[i].size(); j++) {
                if (plan[i][j] == -1)
                    cout << "X";
                else if (plan[i][j] == -2)
                    cout << "G";
                else
                    cout << plan[i][j];
                if (j != plan[i].size() - 1)
                    cout << "=>";
            }
            cout << endl;
        }
    }

    void Instance::CheckPlan(vector<vector<int> > &plan) {
        size_t plan_length = 0;
        for (size_t i = 0; i < plan.size(); i++)
            plan_length = max(plan_length, plan[i].size());

        if (plan_length == 0) {
            cout << "Empty plan!" << endl;
            return;
        }

        for (size_t i = 0; i < plan_length; i++) {
            for (size_t j = 0; j < plan.size(); j++) {
                for (size_t k = 0; k < plan.size(); k++) {
                    if (j == k)
                        continue;
                    if (plan[j][i] != -1 && plan[j][i] != -2 && plan[j][i] == plan[k][i])
                        cout << "Node collision!!! Agents [" << j << "], [" << k << "] at time [" << i << "]" << endl;
                    if (i > 0 && plan[j][i] != -1 && plan[j][i] != -2 && plan[j][i - 1] == plan[k][i] &&
                        plan[k][i - 1] == plan[j][i])
                        cout << "Swap collision!!! Agents [" << j << "], [" << k << "] at times [" << i - 1 << "," << i
                             << "]" << endl;
                }
            }
        }
    }

    int Instance::GetPlanMakespan(vector<vector<int> > &plan) {
        int mks = 0;
        for (size_t i = 0; i < plan.size(); i++) {
            for (size_t j = plan[i].size() - 1; j > mks; j--) {
                if (plan[i][j] != plan[i][j - 1]) {
                    mks = j;
                    break;
                }
            }
        }
        mks++;
        return mks;
    }

    int Instance::GetPlanSoC(vector<vector<int> > &plan) {
        int soc = 0;

        for (size_t i = 0; i < plan.size(); i++) {
            for (int j = plan[i].size() - 1; j >= 0; j--) {
                if (plan[i][j] != goal[i]) {
                    soc += j + 1;
                    break;
                }
            }
        }
        return soc;
    }


/* OLD READ INPUT */

/*

char c_dump;
string s_dump;

// grid size
in >> rows >> c_dump >> columns;

getline(in, s_dump);

// graph
int_graph = vector<vector<int> >(rows, vector<int>(columns, -1));
nodes = 0;

for (int i = 0; i < rows; i++)
{
	for (int j = 0; j < columns; j++)
	{
		in >> c_dump;
		if (c_dump == '.')
		{
			int_graph[i][j] = nodes;
			nodes++;
		}
	}
}

graph = vector<vector<int> >(nodes, vector<int>());

for (int i = 0; i < rows; i++)
{
	for (int j = 0; j < columns; j++)
	{
		if (int_graph[i][j] != -1)
		{
			// down
			if (i < rows - 1 && int_graph[i + 1][j] != -1)
			{
				graph[int_graph[i][j]].push_back(int_graph[i + 1][j]);
				graph[int_graph[i + 1][j]].push_back(int_graph[i][j]);
			}

			// left
			if (j > 0 && int_graph[i][j - 1] != -1)
			{
				graph[int_graph[i][j]].push_back(int_graph[i][j - 1]);
				graph[int_graph[i][j - 1]].push_back(int_graph[i][j]);
			}
		}
	}
}

// agents
getline(in, s_dump);	//empty line
getline(in, s_dump);	//"Agents:"

in >> agents;

for (int i = 0; i < agents; i++)
{
	int s, g;
	in >> s >> g;
	start.push_back(s);
	goal.push_back(g);
}

*/
}