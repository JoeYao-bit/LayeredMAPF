
#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../freeNav-base/dependencies/memory_analysis.h"
#include "../algorithm/general_mapf_scene.h"
#include "../test/test_data.h"

#include "common_interfaces.h"



using namespace freeNav;
using namespace freeNav::LayeredMAPF;
using namespace freeNav::LayeredMAPF::LA_MAPF;

// test_count: the total count of start and target pair in the scenario file
// required_count: required
std::vector<std::set<int> > pickCasesFromScene(int test_count,
                                               const std::vector<int>& required_counts,
                                               int instance_count) {
    std::vector<std::set<int> > retv;
    for(int i=0; i<instance_count; i++) {
        for(const int& required_count : required_counts) {
            std::set<int> instance;
            while(1) {
                int current_pick = rand() % test_count;
                if(instance.find(current_pick) == instance.end()) {
                    instance.insert(current_pick);
                    if(instance.size() == required_count) {
                        retv.push_back(instance);
                        break;
                    }
                }
            }
        }
    }
    return retv;
}


auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};

void multiLoadAgentAndCompare(const SingleMapTestConfig<2>& map_file,
                              const std::vector<int>& agent_in_instances,
                              int count_of_instance,
                              double time_limit = 60) {

    TextMapLoader loader = TextMapLoader(map_file.at("map_path"), is_char_occupied1);

    auto dim = loader.getDimensionInfo();
    auto is_occupied = [&loader](const freeNav::Pointi<2> &pt) -> bool { return loader.isOccupied(pt); };
    IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

    ScenarioLoader2D sl(map_file.at("scene_path").c_str());
    int count_of_experiments = sl.GetNumExperiments();


    const auto& insts_ids = pickCasesFromScene(count_of_experiments, agent_in_instances, count_of_instance);
    InstancesS<2> istss;
    for(const auto& ids : insts_ids) {
        Instances<2> ists;
        for(const int& id : ids) {
            const auto &experiment = sl.GetNthExperiment(id);
            Pointi<2> pt1({experiment.GetStartX(), experiment.GetStartY()});
            Pointi<2> pt2({experiment.GetGoalX(), experiment.GetGoalY()});
            Instance<2> ist = {pt1, pt2};
            //std::cout << " start, target = " << pt1 << ", " << pt2 << std::endl;
            ists.push_back(ist);
        }
        istss.push_back(ists);
    }

    std::cout << " istss size " << istss.size() << std::endl;

    for (int i = 0; i < istss.size(); i++) {

        const auto& instances = istss[i];

        std::vector<std::string> strs;

//        auto str1 = BIPARTITION_MAPF<2>(
//                instances,
//                dim,
//                is_occupied,
//                LaCAM::LargeAgentLaCAM_func<2, Pointi<2> >,
//                "LaCAM",
//                time_limit);
//
//        strs.push_back(str1);

//        auto str2 = BREAKLOOP_MAPF<2>(
//                instances,
//                dim,
//                is_occupied,
//                //LaCAM::LargeAgentLaCAM_func<2, Pose<int, 2> >,
//                CBS::LargeAgentCBS_func<2, Pointi<2> >,
//                "CBS",
//                time_limit);
//
//        strs.push_back(str2);
//
        auto str3 = RAW_MAPF<2>(
                instances,
                dim,
                is_occupied,
                //LaCAM::LargeAgentLaCAM_func<2, Pose<int, 2> >,
                CBS::LargeAgentCBS_func<2, Pointi<2> >,
                "CBS",
                time_limit);

        strs.push_back(str3);
//
//        auto str4 = ID_MAPF<2>(
//                instances,
//                dim,
//                is_occupied,
//                //LaCAM::LargeAgentLaCAM_func<2, Pose<int, 2> >,
//                CBS::LargeAgentCBS_func<2, Pointi<2> >,
//                "CBS",
//                time_limit);
//
//        strs.push_back(str4);

        for(const auto& str : strs) {
            std::cout << str << std::endl;
        }

//        IDLAMAPF<2>();
//        RAWLAMAPF<2>();

        //writeStrsToEndOfFile(strs, map_test_config.at("la_comp_path"));
    }

}



//TEST(Multi_Generate_Agent_And_Compare, test) {
int main() {
    // file_path, count_of_test, max_agent_count, min_agent_count, interval, max_sample
//    std::vector<std::tuple<SingleMapTestConfig<2>, int, int, int, int> >
//    map_configs = {
//            //      {MAPFTestConfig_Paris_1_256,     1, 80, 10, 10}, // 80, 10, 10 / 20, 2, 2s
//            //     {MAPFTestConfig_empty_48_48,     1, 50, 10, 10}, // 50, 10, 10
//            //     {MAPFTestConfig_Berlin_1_256,    1, 80, 10, 10}, // 80, 10, 10
//            //    {MAPFTestConfig_maze_128_128_10, 1, 60, 10, 10}, // 60, 10, 10
//
//            // {MAPFTestConfig_den520d,         1, 100, 10, 10},// 100, 10, 10
//            // {MAPFTestConfig_ost003d,         1, 100, 10, 10},// 100, 10, 10
//            //  {MAPFTestConfig_Boston_2_256, 1, 70, 10, 10}, //  70, 10, 10
//            //   {MAPFTestConfig_Sydney_2_256, 1, 70, 10, 10}, // 70, 10, 10
//
//            {MAPFTestConfig_AR0044SR, 1, 6, 2, 2}, // 50, 5, 5
//            //{MAPFTestConfig_AR0203SR, 1, 6, 2, 2}, // 40, 5, 5
////            {MAPFTestConfig_AR0072SR, 1, 30, 5, 5}, // 30, 5, 5
////            {MAPFTestConfig_Denver_2_256, 1, 80, 10, 10}, // 80, 10, 10
//
//            // not in test
//            //        {MAPFTestConfig_Boston_2_256, 1, 20, 2, 2}, // ok
//            //        {MAPFTestConfig_Sydney_2_256, 1, 20, 2, 2}, // ok
//            //        {MAPFTestConfig_AR0044SR, 1, 20, 2, 2}, // ok
//            //        {MAPFTestConfig_AR0203SR, 1, 20, 2, 2}, // ok
//            //    {MAPFTestConfig_AR0072SR, 1, 20, 2, 2}, // ok
//            //     {MAPFTestConfig_Denver_2_256, 1, 20, 2, 2} // ok
//
//    };
//    for(int i=0; i<1;i++)
//    {
//        std::cout << "global layered" << i << std::endl;
//        for(const auto& file_config : map_configs) {
//            multiLoadAgentAndCompare(std::get<0>(file_config),
//                                     std::get<1>(file_config),
//                                     std::get<2>(file_config),
//                                     std::get<3>(file_config),
//                                     std::get<4>(file_config),
//                                     60);
//        }
//    }
    return 0;
}

