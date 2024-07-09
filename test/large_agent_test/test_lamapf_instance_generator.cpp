//
// Created by yaozhuo on 2024/6/26.
//
#include <gtest/gtest.h>
#include <random>
#include "../../algorithm/LA-MAPF/large_agent_instance_generator.h"
#include "../../algorithm/LA-MAPF/instance_serialize_and_deserialize.h"
#include "../../freeNav-base/visualization/canvas/canvas.h"
#include "../../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../../algorithm/LA-MAPF/CBS/large_agent_CBS.h"
#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"
#include "../../algorithm/LA-MAPF/CBS/constraint_avoidance_table.h"

#include "../test_data.h"

using namespace freeNav;
using namespace freeNav::LayeredMAPF;
using namespace freeNav::LayeredMAPF::LA_MAPF;

struct timezone tz;
struct timeval tv_pre, tv_cur;
struct timeval tv_after;

TEST(CircleAgentTest, generator_test) {
//    std::random_device rd;
//    std::mt19937 *MT = new std::mt19937(rd());
//    std::cout << get_random_float(MT, .1, 2.4) << std::endl;
//    delete MT;

    CircleAgents<2> agents = RandomCircleAgentsGenerator<2>(5,
                                                            .4, 2.3,
                                                            .1, nullptr);
    for(int i=0; i<agents.size(); i++) {
        const auto& agent = agents[i];
        std::cout << agent << std::endl;
    }
}

TEST(Block2DAgentTest, generator_test) {
//    std::cout << getValueWithInterval<float>(-.5, -.1, .1) << std::endl;

    const BlockAgents_2D& agents = RandomBlock2DAgentsGenerator(5,
                                                          -.4, -.2,
                                                          .2, 2.,
                                                          .2, 1.4,
                                                          .1, nullptr);
    // NOTICE: must declared then print, otherwise value will be change
    //         even declared, the first print not change but following print change
    //         (-0.2, -0.2) (1.8, 1)
    //         (0, 1.4013e-45) (1.8, 1)

    for(int i=0; i<agents.size(); i++) {
        const auto& agent = agents[i];
        std::cout << agent << std::endl;
    }
}
// MAPFTestConfig_Berlin_1_256
// MAPFTestConfig_maze_32_32_4
// MAPFTestConfig_warehouse_10_20_10_2_1
// MAPFTestConfig_warehouse_10_20_10_2_2
// MAPFTestConfig_Paris_1_256
// MAPFTestConfig_simple
// MAPFTestConfig_empty_48_48
// MAPFTestConfig_warehouse_20_40_10_2_1
// MAPFTestConfig_warehouse_20_40_10_2_2
// MAPFTestConfig_room_32_32_4
// MAPFTestConfig_room_64_64_8
auto map_test_config = MAPFTestConfig_Paris_1_256;

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};

TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto dim = loader.getDimensionInfo();

int canvas_size_x = 1000, canvas_size_y = 700;

int zoom_ratio = 5;

Pointi<2> pt1;
int current_subgraph_id = 0;
bool draw_all_subgraph_node = false;
bool draw_all_instance = false;
bool draw_path = true;
bool draw_full_path = true;

template<typename AgentType>
void InstanceVisualization(const std::vector<AgentType>& agents,
                           const LargeAgentMAPF_InstanceGenerator<2, AgentType>& generator,
                           const std::vector<InstanceOrient<2> >& instances,
                           const std::vector<LAMAPF_Path>& solution) {

    // visualize instance
    Canvas canvas("LargeAgentMAPF InstanceGenerator", dim[0], dim[1], .1, zoom_ratio);
    int time_index = 0;


    size_t makespan = getMakeSpan(solution);

    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied);

        if(draw_full_path) {
            const auto& all_poses = generator.getAllPoses();
            for(int i=0; i<solution.size(); i++) {
                const auto& path = solution[i];
                for(int t=0; t<path.size()-1; t++) {
                    Pointi<2> pt1 = all_poses[path[t]]->pt_,
                            pt2 = all_poses[path[t+1]]->pt_;
                    canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], true, std::max(1, zoom_ratio/10), COLOR_TABLE[(i) % 30]);
                }
            }
        }
        if(draw_all_instance) {
            for (int i=0; i<instances.size(); i++)
            {
                //const auto &instance = instances[current_subgraph_id]; // zoom_ratio/10
                const auto &instance = instances[i]; // zoom_ratio/10
                DrawOnCanvas(agents[i], instance.first, canvas, COLOR_TABLE[i%30]);

                DrawOnCanvas(agents[i], instance.second, canvas, COLOR_TABLE[i%30]);

                canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], -orientToPi_2D(instance.first.orient_), 1, std::max(1, zoom_ratio/10));
                canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], -orientToPi_2D(instance.second.orient_) , 1, std::max(1, zoom_ratio/10));

            }
        }
        if(draw_path) {
            const auto& all_poses = generator.getAllPoses();
            for(int i=0; i<solution.size(); i++) {
                const auto& path = solution[i];
                Pointi<2> pt;
                int orient = 0;
                if(time_index <= path.size() - 1) {
                    pt     = all_poses[path[time_index]]->pt_;
                    orient = all_poses[path[time_index]]->orient_;
                } else {
                    pt     = all_poses[path.back()]->pt_;
                    orient = all_poses[path.back()]->orient_;
                }

                DrawOnCanvas(agents[i], {pt, orient}, canvas, COLOR_TABLE[(i) % 30]);

                canvas.drawArrowInt(pt[0], pt[1], -orientToPi_2D(orient), 1, std::max(1, zoom_ratio/10));

            }
        }
        char key = canvas.show(1000);
        switch (key) {
            case 'i':
                draw_all_instance = !draw_all_instance;
                break;
            case 'w':
                current_subgraph_id ++;
                current_subgraph_id = current_subgraph_id % instances.size();
                std::cout << "-- switch to subgraph " << current_subgraph_id << std::endl;
                break;
            case 's':
                current_subgraph_id --;
                current_subgraph_id = (current_subgraph_id + instances.size()) % instances.size();
                std::cout << "-- switch to subgraph " << current_subgraph_id << std::endl;
                break;
            case 'p':
                draw_path = !draw_path;
                break;
            case 'f':
                draw_full_path = !draw_full_path;
                break;
            case 'q':
                if(makespan > 0) {
                    time_index = time_index + makespan - 1;
                    time_index = time_index % makespan;
                    std::cout << "-- switch to time index = " << time_index << std::endl;
                }
                break;
            case 'e':
                if(makespan > 0) {
                    time_index++;
//                    if(time_index > makespan) {
//                        time_index = makespan;
//                    }
                    time_index = time_index % makespan;
                    std::cout << "-- switch to time index = " << time_index << std::endl;
                }
                break;
            default:
                break;
        }
    }
}




template<typename AgentType, typename MethodType>
void loadInstanceAndPlanning(const std::string& file_path) {
    InstanceDeserializer<2, AgentType> deserializer;
    if(deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    LargeAgentMAPF_InstanceGenerator<2, AgentType> generator(deserializer.getAgents(), is_occupied, dim);

    gettimeofday(&tv_pre, &tz);
    MethodType method(deserializer.getInstances(), deserializer.getAgents(), dim, is_occupied);
    bool solved = method.solve(60, 0);
    gettimeofday(&tv_after, &tz);
    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
    std::cout << "instance has " << deserializer.getAgents().size() << " agents, find solution ? " << solved << " in " << time_cost << "ms " << std::endl;
    std::cout << "solution validation ? " << method.solutionValidation() << std::endl;

    InstanceVisualization<AgentType>(deserializer.getAgents(), generator, deserializer.getInstances(), method.getSolution());

}

template<typename AgentType, typename MethodType>
void generateInstance(const std::vector<AgentType>& agents, const std::string& file_path) {
    gettimeofday(&tv_pre, &tz);
    LargeAgentMAPF_InstanceGenerator<2, AgentType> generator(agents, is_occupied, dim, 1e7);
    for(int i=0; i<agents.size(); i++) {
        const auto& agent = agents[i];
        std::cout << agent << std::endl;
    }
    const auto& instances_and_path = generator.getNewInstance();
    gettimeofday(&tv_after, &tz);
    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;

    std::cout << "find instance ? " << !instances_and_path.empty() << " in " << time_cost << "ms " << std::endl;

    if(instances_and_path.empty()) {
        return;
    }

    InstanceOrients<2> instances;
    for(int i=0; i<instances_and_path.size(); i++) {
        instances.push_back(instances_and_path[i].first);
    }

    std::vector<LAMAPF_Path> solution;
    for(int i=0; i<instances_and_path.size(); i++) {
        solution.push_back(instances_and_path[i].second);
    }
    InstanceSerializer<2, AgentType> serializer(agents, instances);
    if(serializer.saveToFile(file_path)) {
        std::cout << "save to path " << file_path << " success" << std::endl;
    } else {
        std::cout << "save to path " << file_path << " failed" << std::endl;
        return;
    }

//    InstanceVisualization<AgentType>(agents, generator, instances, solution);
    loadInstanceAndPlanning<AgentType, MethodType>(file_path);
}

TEST(GenerateCircleInstance, test)
{
    const CircleAgents<2>& agents = RandomCircleAgentsGenerator<2>(10,
                                                                   .2, 1.4,
                                                                   .1,
                                                                   dim);
    generateInstance<CircleAgent<2>, CBS::LargeAgentCBS<2, CircleAgent<2> > > (agents, map_test_config.at("crc_ins_path"));

};

TEST(GenerateBlock_2DInstance, test)
{
    const BlockAgents_2D& agents = RandomBlock2DAgentsGenerator(10,
                                                                -1.4, -.2,
                                                                .2, 1.4,
                                                                .2, 1.4,
                                                                .1, dim);
    generateInstance<BlockAgent_2D, CBS::LargeAgentCBS<2, BlockAgent_2D > >(agents, map_test_config.at("blk_ins_path"));

};

TEST(LoadCircleInstance, test)
{
    const std::string file_path = map_test_config.at("crc_ins_path");
//    loadInstanceAndPlanning<CircleAgent<2>,
//                            LaCAM::LargeAgentLaCAM<2, CircleAgent<2>,
//                            LaCAM::LargeAgentConstraintTable<2, CircleAgent<2> > > >(file_path);
    loadInstanceAndPlanning<CircleAgent<2>,
                            CBS::LargeAgentCBS<2, CircleAgent<2> > >(file_path);
};

TEST(LoadBlock_2DInstance, test)
{
    const std::string file_path = map_test_config.at("blk_ins_path");
//    loadInstanceAndPlanning<BlockAgent_2D,
//                            LaCAM::LargeAgentLaCAM<2,
//                            BlockAgent_2D, LaCAM::LargeAgentConstraintTable<2, BlockAgent_2D > > >(file_path);
    loadInstanceAndPlanning<BlockAgent_2D,
            CBS::LargeAgentCBS<2, BlockAgent_2D > >(file_path);
};

TEST(resize, test)
{
    std::vector<int> values = {1, 2, 3};
    values.resize(5, 10);
    for(const auto& value : values) {
        std::cout << value << " ";
    }
    std::cout << std::endl;
    // 1 2 3 10 10
}

