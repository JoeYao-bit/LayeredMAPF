//
// Created by yaozhuo on 2024/6/26.
//
#include <gtest/gtest.h>
#include <random>
#include "../../algorithm/LA-MAPF/large_agent_instance_generator.h"

#include "../../freeNav-base/visualization/canvas/canvas.h"
#include "../../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../test_data.h"

using namespace freeNav;
using namespace freeNav::LayeredMAPF;
using namespace freeNav::LayeredMAPF::LA_MAPF;

TEST(CircleAgentTest, generator_test) {
//    std::random_device rd;
//    std::mt19937 *MT = new std::mt19937(rd());
//    std::cout << get_random_float(MT, .1, 2.4) << std::endl;
//    delete MT;

    CircleAgents<2> agents = RandomCircleAgentsGenerator<2>(5,
                                                            .4, 2.3,
                                                            .1);
    for(const auto& agent : agents) {
        std::cout << agent.radius_ << " ";
    }
    std::cout << std::endl;
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


//    const freeNav::Pointf<2> min_pt = agents[0].min_pt_, max_pt = agents[0].max_pt_;
//    std::cout << min_pt << " " << max_pt << std::endl;
//    for(const BlockAgent_2D agent : agents) {
//        const freeNav::Pointf<2> min_pt = agent.min_pt_, max_pt = agent.max_pt_;
//        std::cout << min_pt << " " << max_pt << std::endl;
//    }
    freeNav::Pointf<2> min_pt = agents[0].min_pt_, max_pt = agents[0].max_pt_;

//    std::cout << std::endl;
                       min_pt = agents[0].min_pt_, max_pt = agents[0].max_pt_;
    std::cout << min_pt << " " << max_pt << std::endl;

                       min_pt = agents[0].min_pt_, max_pt = agents[0].max_pt_;
    std::cout << min_pt << " " << max_pt << std::endl;

                       min_pt = agents[2].min_pt_, max_pt = agents[2].max_pt_;
    std::cout << min_pt << " " << max_pt << std::endl;

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

TEST(LargeAgentMAPF_InstanceGenerator, test) {
    CircleAgents<2> agents = RandomCircleAgentsGenerator<2>(3,
                                                            .2, 1.,
                                                            .1);
    for(const auto& agent : agents) {
        std::cout << agent.radius_ << " ";
    }
    std::cout << std::endl;

    LargeAgentMAPF_InstanceGenerator<2, CircleAgent<2> > generator(agents, is_occupied, dim);
    const auto& instances = generator.getNewInstance();
    if(instances.empty()) {
        return;
    }
    // visualize instance
    Canvas canvas("LargeAgentMAPF InstanceGenerator", dim[0], dim[1], .1, zoom_ratio);
    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGridMap(dim, is_occupied);
        if(draw_all_instance) {
            //for (int i=0; i<instances.size(); i++)
            {
                const auto &instance = instances[current_subgraph_id]; // zoom_ratio/10
                canvas.drawGrid(instance.first.pt_[0], instance.first.pt_[1], COLOR_TABLE[(2 + current_subgraph_id)%30]);
                canvas.drawArrowInt(instance.first.pt_[0], instance.first.pt_[1], 0 , 1, zoom_ratio/10);

                canvas.drawGrid(instance.second.pt_[0], instance.second.pt_[1], COLOR_TABLE[(2 + current_subgraph_id)%30]);
                canvas.drawArrowInt(instance.second.pt_[0], instance.second.pt_[1], 0 , 1, zoom_ratio/10);

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
            default:
                break;
        }
    }
}