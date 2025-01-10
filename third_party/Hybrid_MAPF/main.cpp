#include <numeric>

//#include "dirent.h"
#include "CBS.h"
#include "Picat.h"
#include "ID.h"
#include "../third_party/EECBS/inc/driver.h"
#include "../test/test_data.h"
#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"

struct timezone tz;
struct timeval tv_pre;
struct timeval tv_after;

// MAPFTestConfig_random_32_32_20 413.8 ms
// MAPFTestConfig_maze_32_32_2 25.42 ms
// MAPFTestConfig_maze_32_32_4 54.914 ms
// MAPFTestConfig_den312d  314.776 ms
// MAPFTestConfig_Berlin_1_256 56.385 ms
// MAPFTestConfig_Paris_1_256 1005.82 ms
// MAPFTestConfig_warehouse_10_20_10_2_1
// MAPFTestConfig_den520d 237.842 ms
// MAPFTestConfig_empty_32_32 2872.3 ms
// MAPFTestConfig_ht_chantry
// MAPFTestConfig_lak303d
// MAPFTestConfig_simple
// MAPFTestConfig_empty_48_48
auto map_test_config = freeNav::LayeredMAPF::MAPFTestConfig_empty_48_48;//MAPFTestConfig_Boston_0_256;

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};

freeNav::TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);

auto is_occupied = [](const freeNav::Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto set_occupied = [](const freeNav::Pointi<2> & pt) { loader.setOccupied(pt); };

freeNav::IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

freeNav::SET_OCCUPIED_FUNC<2> set_occupied_func = set_occupied;


#define MAKESPAN 1
#define SOC 2

#define FULL_ID 1
#define SIMPLE_ID 0
using namespace Hybird_MAPF;

using namespace std;

void PrintStatistic(int, ID*, Instance*, string, string);

int main()
{

    //MemoryRecorder memory_recorder(50);
    std::cout << " map name " << map_test_config.at("map_path")  << std::endl;
    // load mapf scene
    const auto& dim = loader.getDimensionInfo();
    freeNav::Instances<2> ists;
    ScenarioLoader2D sl(map_test_config.at("scene_path").c_str());
    int max_count_of_case = atoi((map_test_config.at("agent_num")).c_str());
    int count_of_experiments = sl.GetNumExperiments();

    // load experiment data
    for (int i = 0; i < std::min(max_count_of_case, count_of_experiments); i++) {
        const auto &experiment = sl.GetNthExperiment(i);
        freeNav::Pointi<2> pt1({experiment.GetStartX(), experiment.GetStartY()});
        freeNav::Pointi<2> pt2({experiment.GetGoalX(), experiment.GetGoalY()});
        freeNav::Instance<2> ist = {pt1, pt2};
        //std::cout << " start, target = " << pt1 << ", " << pt2 << std::endl;
        ists.push_back(ist);
    }
    std::cout << "get " << ists.size() << " instances" << std::endl;

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
			Instance* inst = new Instance(dim, is_occupied, ists);
			//inst->ReadInput(string("../instances/").append(string(ent1->d_name)));

			// Create solver
			ID* Solver = new ID(inst, cost_function, ID_type);
			Solver->mapf_func_ = CBS_Li::eecbs_MAPF;//CBS_Li::cbs_MAPF;
			int ret_val;

			string function_name;
			if (cost_function == SOC)
				function_name = "soc";
			else
				function_name = "mks";
            freeNav::Paths<2> paths;
            gettimeofday(&tv_pre, &tz);

            ret_val = Solver->SolveProblem(vector<bool> {true,false,false});
            if(Solver->solved) {
                paths = Solver->paths_fr;
            }
            gettimeofday(&tv_after, &tz);
            double time_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
            std::cout << paths.size() << " paths " << " / agents " << ists.size() << std::endl;
            std::cout << "-- Independence Detection end in " << time_cost << "ms" << std::endl;
            std::cout << " is solution valid ? " << freeNav::LayeredMAPF::validateSolution<2>(paths) << std::endl;
			// Solve the instance - use only Picat
//			ret_val = 0;
//			inst->ResetAgentsNumber();
//			while (ret_val == 0 && inst->agents <= inst->start.size())
//			{
//				ret_val = Solver->SolveProblem(vector<bool> {true,false,false});
//				PrintStatistic(ret_val, Solver, inst, "Picat", function_name);
//				inst->IncreaseAgentsNumber();
//			}
//
//			// Solve the instance - use only CBS
//			ret_val = 0;
//			inst->ResetAgentsNumber();
//			while (ret_val == 0 && inst->agents <= inst->start.size())
//			{
//				ret_val = Solver->SolveProblem(vector<bool> {false, true, false});
//				PrintStatistic(ret_val, Solver, inst, "CBS", function_name);
//				inst->IncreaseAgentsNumber();
//			}
//
//			// Solve the instance - use only ICTS
//			ret_val = 0;
//			inst->ResetAgentsNumber();
//			while (ret_val == 0 && inst->agents <= inst->start.size())
//			{
//				ret_val = Solver->SolveProblem(vector<bool> {false, false, true});
//				PrintStatistic(ret_val, Solver, inst, "ICTS", function_name);
//				inst->IncreaseAgentsNumber();
//			}
//
//			// Solve the instance - use all algorithms
//			ret_val = 0;
//			inst->ResetAgentsNumber();
//			while (ret_val == 0 && inst->agents <= inst->start.size())
//			{
//				ret_val = Solver->SolveProblem(vector<bool> {true, true, true});
//				PrintStatistic(ret_val, Solver, inst, "Hybrid", function_name);
//				inst->IncreaseAgentsNumber();
//			}

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