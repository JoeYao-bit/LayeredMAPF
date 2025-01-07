#include <numeric>

//#include "dirent.h"
#include "CBS.h"
#include "Picat.h"
#include "ID.h"

#define MAKESPAN 1
#define SOC 2

#define FULL_ID 1
#define SIMPLE_ID 0
using namespace Hybird_MAPF;

using namespace std;

void PrintStatistic(int, ID*, Instance*, string, string);

int main()
{
//	DIR *dir1;
//	struct dirent *ent1;
	int ID_type = FULL_ID;
	int cost_function = SOC;
//	if ((dir1 = opendir("../instances")) != NULL)
//	{
//		while ((ent1 = readdir(dir1)) != NULL)
//		{
//			if (string(ent1->d_name).compare(".") == 0 || string(ent1->d_name).compare("..") == 0 || string(ent1->d_name).compare("solved") == 0 || string(ent1->d_name).compare("maps") == 0)
//				continue;

			// Read input
			Instance* inst = new Instance();
			//inst->ReadInput(string("../instances/").append(string(ent1->d_name)));

			// Create solver
			ID* Solver = new ID(inst, cost_function, ID_type);
			int ret_val;

			string function_name;
			if (cost_function == SOC)
				function_name = "soc";
			else
				function_name = "mks";
			

			// Solve the instance - use only Picat
			ret_val = 0;
			inst->ResetAgentsNumber();
			while (ret_val == 0 && inst->agents <= inst->start.size())
			{
				ret_val = Solver->SolveProblem(vector<bool> {true,false,false});
				PrintStatistic(ret_val, Solver, inst, "Picat", function_name);
				inst->IncreaseAgentsNumber();
			}

			// Solve the instance - use only CBS
			ret_val = 0;
			inst->ResetAgentsNumber();
			while (ret_val == 0 && inst->agents <= inst->start.size())
			{
				ret_val = Solver->SolveProblem(vector<bool> {false, true, false});
				PrintStatistic(ret_val, Solver, inst, "CBS", function_name);
				inst->IncreaseAgentsNumber();
			}

			// Solve the instance - use only ICTS
			ret_val = 0;
			inst->ResetAgentsNumber();
			while (ret_val == 0 && inst->agents <= inst->start.size())
			{
				ret_val = Solver->SolveProblem(vector<bool> {false, false, true});
				PrintStatistic(ret_val, Solver, inst, "ICTS", function_name);
				inst->IncreaseAgentsNumber();
			}
			
			// Solve the instance - use all algorithms
			ret_val = 0;
			inst->ResetAgentsNumber();
			while (ret_val == 0 && inst->agents <= inst->start.size())
			{
				ret_val = Solver->SolveProblem(vector<bool> {true, true, true});
				PrintStatistic(ret_val, Solver, inst, "Hybrid", function_name);
				inst->IncreaseAgentsNumber();
			}		

			delete inst;
			delete Solver;

//			string executable;
//			executable = string("move ../instances/").append(string(ent1->d_name)).append(" ../instances/solved");
//			system(executable.c_str());
//		}
//		closedir(dir1);
//	}

}


void PrintStatistic(int ret_val, ID* Solver, Instance* inst, string solver_type, string note)
{
	string delimiter = "_";
	ofstream statistic;

	int cbs_computed = 0;
	int picat_computed = 0;
	int icts_computed = 0;
	int cbs_used = 0;
	int picat_used = 0;
	int icts_used = 0;
	long long cbs_time = 0;
	long long picat_time = 0;
	long long icts_time = 0;

	if (solver_type.compare("Hybrid") == 0)
	{
		picat_computed = Solver->solver_computed[0];
		cbs_computed = Solver->solver_computed[1];
		icts_computed = Solver->solver_computed[2];

		picat_used = Solver->solver_used[0];
		cbs_used = Solver->solver_used[1];
		icts_used = Solver->solver_used[2];

		picat_time = accumulate(Solver->solver_time[0].begin(), Solver->solver_time[0].end(), 0);
		cbs_time = accumulate(Solver->solver_time[1].begin(), Solver->solver_time[1].end(), 0);
		icts_time = accumulate(Solver->solver_time[2].begin(), Solver->solver_time[2].end(), 0);
	}
	else if (solver_type.compare("CBS") == 0)
	{
		cbs_computed = Solver->solver_computed[0];

		cbs_used = Solver->solver_used[0];

		cbs_time = accumulate(Solver->solver_time[0].begin(), Solver->solver_time[0].end(), 0);
	}
	else if (solver_type.compare("Picat") == 0)
	{
		picat_computed = Solver->solver_computed[0];

		picat_used = Solver->solver_used[0];

		picat_time = accumulate(Solver->solver_time[0].begin(), Solver->solver_time[0].end(), 0);
	}
	else if (solver_type.compare("ICTS") == 0)
	{
		icts_computed = Solver->solver_computed[0];

		icts_used = Solver->solver_used[0];

		icts_time = accumulate(Solver->solver_time[0].begin(), Solver->solver_time[0].end(), 0);
	}

	statistic.open(inst->statistic_file, ofstream::out | ofstream::app);
	if (statistic.is_open())
	{
		// name of the instance
		statistic << inst->input_filename << delimiter;

		// number of agents used
		statistic << inst->agents << delimiter;

		// what solver was used
		statistic << solver_type << delimiter;

		// what was the objective function
		statistic << note << delimiter;

		// final mks
		statistic << Solver->final_makespan << delimiter;

		// final soc
		statistic << Solver->final_soc << delimiter;

		// was the computation successful
		statistic << (ret_val == 0 ? "success" : "fail") << delimiter;

		// how many times each solver was computed
		statistic << cbs_computed << delimiter;
		statistic << picat_computed << delimiter;
		statistic << icts_computed << delimiter;

		// how many times each solver was used
		statistic << cbs_used << delimiter;
		statistic << picat_used << delimiter;
		statistic << icts_used << delimiter;

		// how much time each solver take
		statistic << cbs_time << delimiter;
		statistic << picat_time << delimiter;
		statistic << icts_time << delimiter;

		// how much time it take together - should be just added the two previous
		statistic << cbs_time + picat_time + icts_time;

		statistic << endl;

		statistic.close();
	}
}