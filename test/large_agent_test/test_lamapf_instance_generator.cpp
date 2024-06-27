//
// Created by yaozhuo on 2024/6/26.
//
#include <gtest/gtest.h>
#include <random>
#include "../../algorithm/LA-MAPF/large_agent_instance_generator.h"
#include "../../algorithm/LA-MAPF/instance_serialize_and_deserialize.h"
#include "../../freeNav-base/visualization/canvas/canvas.h"
#include "../../freeNav-base/dependencies/2d_grid/text_map_loader.h"
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
                                                            .1);
    for(int i=0; i<agents.size(); i++) {
        const auto& agent = agents[i];
        std::cout << "CircleAgent " << i << ": "<< agent.radius_ << " " << std::endl;
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
        std::cout << "BlockAgent " << i << ": " << agent.min_pt_ << " " << agent.max_pt_ << std::endl;
    }
}

auto map_test_config = MAPFTestConfig_maze_32_32_4;

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};

TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto dim = loader.getDimensionInfo();

int canvas_size_x = 1000, canvas_size_y = 700;

int zoom_ratio = 20;

Pointi<2> pt1;
int current_subgraph_id = 0;
bool draw_all_subgraph_node = false;
bool draw_all_instance = false;
bool draw_path = true;
bool draw_full_path = true;

#define CIRCLE_AGENT 0

TEST(LargeAgentMAPF_InstanceGenerator, test) {

#if CIRCLE_AGENT
    const CircleAgents<2>& agents = RandomCircleAgentsGenerator<2>(5,
                                                            .2, 1.,
                                                            .1);
#else
    const BlockAgents_2D& agents = RandomBlock2DAgentsGenerator(8,
                                                                -.4, -.2,
                                                                .2, 1.2,
                                                                .2, 1.2,
                                                                .1, nullptr);
#endif

    gettimeofday(&tv_pre, &tz);

#if CIRCLE_AGENT
    LargeAgentMAPF_InstanceGenerator<2, CircleAgent<2> > generator(agents, is_occupied, dim);
    for(int i=0; i<agents.size(); i++) {
        const auto& agent = agents[i];
        std::cout << "CircleAgent " << i << ": "<< agent.radius_ << " " << std::endl;
    }
#else
    LargeAgentMAPF_InstanceGenerator<2, BlockAgent_2D > generator(agents, is_occupied, dim);
    for(int i=0; i<agents.size(); i++) {
        const auto& agent = agents[i];
        std::cout << "BlockAgent " << i << ": " << agent.min_pt_ << " " << agent.max_pt_ << std::endl;
    }
#endif
    const auto& instances_and_path = generator.getNewInstance();
    gettimeofday(&tv_after, &tz);
    double time_cost = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;

    std::cout << "find instance ? " << !instances_and_path.empty() << " in " << time_cost << "ms " << std::endl;

    if(instances_and_path.empty()) {
        return;
    }

    std::vector<LAMAPF_Path> solution;
    for(int i=0; i<instances_and_path.size(); i++) {
        solution.push_back(instances_and_path[i].second);
    }
    size_t makespan = getMakeSpan(solution);

    // visualize instance
    Canvas canvas("LargeAgentMAPF InstanceGenerator", dim[0], dim[1], .1, zoom_ratio);
    int time_index = 0;
    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied);
        if(draw_all_instance) {
            for (int i=0; i<instances_and_path.size(); i++)
            {
                //const auto &instance = instances[current_subgraph_id]; // zoom_ratio/10
                const auto &instance = instances_and_path[i].first; // zoom_ratio/10
                canvas.drawGrid(instance.first.pt_[0], instance.first.pt_[1], COLOR_TABLE[(2 + i)%30]);
                canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], 0 , 1, zoom_ratio/10);

                canvas.drawGrid(instance.second.pt_[0], instance.second.pt_[1], COLOR_TABLE[(2 + i)%30]);
                canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], 0 , 1, zoom_ratio/10);

            }
        }
        if(draw_full_path) {
            const auto& all_poses = generator.getAllPoses();
            for(int i=0; i<instances_and_path.size(); i++) {
                const auto& path = instances_and_path[i].second;
                for(int t=0; t<path.size()-1; t++) {
                    Pointi<2> pt1 = all_poses[path[t]]->pt_,
                              pt2 = all_poses[path[t+1]]->pt_;
                    canvas.drawLineInt(pt1[0], pt1[1], pt2[0], pt2[1], true, zoom_ratio/10, COLOR_TABLE[(i) % 30]);
                }
            }
        }
        if(draw_path) {
            const auto& all_poses = generator.getAllPoses();
            for(int i=0; i<instances_and_path.size(); i++) {
                const auto& path = instances_and_path[i].second;
                Pointi<2> pt;
                int orient = 0;
                if(time_index <= path.size() - 1) {
                    pt     = all_poses[path[time_index]]->pt_;
                    orient = all_poses[path[time_index]]->orient_;
                } else {
                    pt     = all_poses[path.back()]->pt_;
                    orient = all_poses[path.back()]->orient_;
                }
//                const auto& rect = lacam.agents_[i].getPosedRectangle({pt, orient}); // agents
//                canvas.drawRectangleFloat(rect.first, rect.second, true, -1, COLOR_TABLE[(i) % 30]);
//                //std::cout << "rect " << rect.first << ", " << rect.second << std::endl;
#if CIRCLE_AGENT
                canvas.drawCircleInt(pt[0], pt[1], floor(agents[i].radius_*zoom_ratio),
                                     true, -1,
                                     COLOR_TABLE[(2+i) % 30]);
#else
                const auto& rect = agents[i].getPosedRectangle({pt, orient}); // agents
                canvas.drawRectangleFloat(rect.first, rect.second, true, -1, COLOR_TABLE[(i) % 30]);
#endif
                double theta = 0;
                switch (orient) {
                    case 0:
                        theta = 0;
                        break;
                    case 1:
                        theta = M_PI;
                        break;
                    case 2:
                        theta = 3*M_PI/2;
                        break;
                    case 3:
                        theta = M_PI/2;
                        break;
                    default:
                        std::cerr << "wrong 2D orient = " << orient << std::endl;
                        exit(0);
                        break;
                }
                canvas.drawArrowInt(pt[0], pt[1], theta , 1, zoom_ratio/10);

            }
        }
        char key = canvas.show(1000);
        switch (key) {
            case 'i':
                draw_all_instance = !draw_all_instance;
                break;
            case 'w':
                current_subgraph_id ++;
                current_subgraph_id = current_subgraph_id % instances_and_path.size();
                std::cout << "-- switch to subgraph " << current_subgraph_id << std::endl;
                break;
            case 's':
                current_subgraph_id --;
                current_subgraph_id = (current_subgraph_id + instances_and_path.size()) % instances_and_path.size();
                std::cout << "-- switch to subgraph " << current_subgraph_id << std::endl;
                break;
            case 'p':
                draw_path = !draw_path;
                break;
            case 'f':
                draw_full_path = !draw_full_path;
                break;
            case 'q':
                time_index = time_index + makespan - 1;
                time_index = time_index % makespan;
                std::cout << "-- switch to time index = " << time_index << std::endl;
                break;
            case 'e':
                time_index ++;
                time_index = time_index % makespan;
                std::cout << "-- switch to time index = " << time_index << std::endl;
                break;
            default:
                break;
        }
    }
}