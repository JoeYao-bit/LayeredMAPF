//
// Created by yaozhuo on 6/29/25.
//



#include "../../algorithm/LA-MAPF/LaCAM/layered_large_agent_LaCAM.h"
#include "../../algorithm/LA-MAPF/CBS/layered_large_agent_CBS.h"

#include "common_interfaces.h"

#include "../test_data.h"
#include "../../algorithm/break_loop_decomposition/biparition_decomposition.h"
#include "../../algorithm/break_loop_decomposition/break_loop_decomposition.h"
#include <mutex>
#include <thread>
using namespace freeNav::LayeredMAPF::LA_MAPF;




void multiLoadAgentAndCompare(const SingleMapTestConfig<2>& map_file,
                              int count_of_test,
                              int maximum_agents,
                              int minimum_agents,
                              int agent_interval,
                              double time_limit = 60) {

    TextMapLoader loader = TextMapLoader(map_file.at("map_path"), is_char_occupied1);

    auto dim = loader.getDimensionInfo();
    auto is_occupied = [&loader](const freeNav::Pointi<2> &pt) -> bool { return loader.isOccupied(pt); };
    IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;


    InstanceDeserializer<2> deserializer;
    if(deserializer.loadInstanceFromFile(map_file.at("la_ins_path"), dim)) {
        std::cout << "load from path " << map_file.at("la_ins_path") << " success" << std::endl;
    } else {
        std::cout << "load from path " << map_file.at("la_ins_path") << " failed" << std::endl;
        return;
    }
    assert(!deserializer.getAgents().empty());
    std::vector<int> required_counts;
    for(int c=minimum_agents; c<=std::min(maximum_agents, (int)deserializer.getAgents().size()); c+=agent_interval) {
        required_counts.push_back(c);
    }
    auto agent_and_instances = deserializer.getTestInstance(required_counts, count_of_test);

    std::cout << " agent_and_instances size " << agent_and_instances.size() << std::endl;

    for (int i = 0; i < agent_and_instances.size(); i++) {

        const auto& instances_local = agent_and_instances[i].second;
        const auto& agents_local = agent_and_instances[i].first;

        std::vector<std::string> strs;

        auto str = BIPARTITION_LAMAPF<2>(
                instances_local,
                agents_local,
                dim,
                is_occupied,
                LaCAM::LargeAgentLaCAMPose_func<2>,
                "LaCAM",
                time_limit);
        strs.push_back(str);

        str = BIPARTITION_LAMAPF<2>(
                instances_local,
                agents_local,
                dim,
                is_occupied,
                CBS::LargeAgentCBS_func<2, Pose<int, 2> >,
                "CBS",
                time_limit);
        strs.push_back(str);

        str = BREAKLOOP_LAMAPF<2>(
                instances_local,
                agents_local,
                dim,
                is_occupied,
                LaCAM::LargeAgentLaCAMPose_func<2>,
                "LaCAM",
                time_limit);
        strs.push_back(str);

        str = BREAKLOOP_LAMAPF<2>(
                instances_local,
                agents_local,
                dim,
                is_occupied,
                //LaCAM::LargeAgentLaCAM_func<2, Pose<int, 2> >,
                CBS::LargeAgentCBS_func<2, Pose<int, 2> >,
                "CBS",
                time_limit);
        strs.push_back(str);

        str = RAW_LAMAPF<2>(
                instances_local,
                agents_local,
                dim,
                is_occupied,
                //LaCAM::LargeAgentLaCAM_func<2, Pose<int, 2> >,
                LaCAM::LargeAgentLaCAMPose_func<2>,
                "LaCAM",
                time_limit);
        strs.push_back(str);


        str = RAW_LAMAPF<2>(
                instances_local,
                agents_local,
                dim,
                is_occupied,
                //LaCAM::LargeAgentLaCAM_func<2, Pose<int, 2> >,
                CBS::LargeAgentCBS_func<2, Pose<int, 2> >,
                "CBS",
                time_limit);
        strs.push_back(str);

        str = ID_LAMAPF<2>(
                instances_local,
                agents_local,
                dim,
                is_occupied,
                //LaCAM::LargeAgentLaCAM_func<2, Pose<int, 2> >,
                CBS::LargeAgentCBS_func<2, Pose<int, 2> >,
                "CBS",
                time_limit);
        strs.push_back(str);

        for(const auto& str : strs) {
            std::cout << str << std::endl;
        }

//        IDLAMAPF<2>();
//        RAWLAMAPF<2>();

        writeStrsToEndOfFile(strs, map_file.at("la_comp_path"));
    }

}



//TEST(Multi_Generate_Agent_And_Compare, test) {
int main() {
    // file_path, count_of_test, max_agent_count, min_agent_count, interval, max_sample
    std::vector<std::tuple<SingleMapTestConfig<2>, int, int, int, int> >
            map_configs = {
        {MAPFTestConfig_empty_48_48,     1, 60, 10, 10}, // 50, 10, 10
       {MAPFTestConfig_Berlin_1_256,    1, 80, 10, 10}, // 80, 10, 10
    {MAPFTestConfig_ost003d,         1, 100, 10, 10},// 100, 10, 10
    {MAPFTestConfig_AR0044SR, 1, 140, 10, 10}, // 50, 5, 5
    {MAPFTestConfig_AR0203SR, 1, 40, 5, 5}, // 40, 5, 5

            //      {MAPFTestConfig_Paris_1_256,     1, 80, 10, 10}, // 80, 10, 10 / 20, 2, 2s

            //    {MAPFTestConfig_maze_128_128_10, 1, 60, 10, 10}, // 60, 10, 10

            // {MAPFTestConfig_den520d,         1, 100, 10, 10},// 100, 10, 10

              //{MAPFTestConfig_Boston_2_256, 1, 70, 10, 10}, //  70, 10, 10
            //   {MAPFTestConfig_Sydney_2_256, 1, 70, 10, 10}, // 70, 10, 10


//            {MAPFTestConfig_AR0072SR, 1, 30, 5, 5}, // 30, 5, 5
//            {MAPFTestConfig_Denver_2_256, 1, 80, 10, 10}, // 80, 10, 10

            // not in test
            //        {MAPFTestConfig_Boston_2_256, 1, 20, 2, 2}, // ok
            //        {MAPFTestConfig_Sydney_2_256, 1, 20, 2, 2}, // ok
            //        {MAPFTestConfig_AR0044SR, 1, 20, 2, 2}, // ok
            //        {MAPFTestConfig_AR0203SR, 1, 20, 2, 2}, // ok
            //    {MAPFTestConfig_AR0072SR, 1, 20, 2, 2}, // ok
                // {MAPFTestConfig_Denver_2_256, 1, 20, 2, 2} // ok

    };
    std::vector<bool> finished(map_configs.size(), false);
    std::mutex lock_1, lock_2;
    int map_id = 0;
	for(int i=0; i<map_configs.size(); i++) {
	auto lambda_func = [&]() {
		lock_1.lock();
		const auto& file_config = map_configs[map_id];
		map_id ++;
		lock_1.unlock();
	    for(int i=0; i<100;i++)
	    {
		std::cout << "global layered" << i << std::endl;

		    multiLoadAgentAndCompare(std::get<0>(file_config),
			                     std::get<1>(file_config),
			                     std::get<2>(file_config),
			                     std::get<3>(file_config),
			                     std::get<4>(file_config),
			                     60);
	    }
	    lock_2.lock();
	    finished[i] = true;
	    lock_2.unlock();
		};
		std::thread t(lambda_func);
		t.detach();
		sleep(1);
	}
	while(finished != std::vector<bool>(map_configs.size(), true)) {
        sleep(1);
    	}
    return 0;
}
