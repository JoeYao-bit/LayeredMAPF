//
// Created by yaozhuo on 2024/5/5.
//

#include <gtest/gtest.h>
#include "../algorithm/LA-MAPF/shaped_agent.h"
#include "../algorithm/LA-MAPF/large_agent_CBS.h"
#include "../freeNav-base/visualization/canvas/canvas.h"
#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "test_data.h"

using namespace freeNav;
using namespace freeNav::LayeredMAPF;
using namespace freeNav::LayeredMAPF::LA_MAPF;

CircleAgent<2> c1(2.5);
CircleAgent<2> c2(3.5);

TEST(circleAgentIsCollide, test) {
    Pose<2> p1({1,1},0), p2({1,2}, 0), p3({1,3}, 0), p4({4,1}, 0);
    std::cout << "is collide" << isCollide(c1, p1, p2, c2, p3, p4) << std::endl;
}

auto map_test_config = MAPFTestConfig_maze_32_32_4;

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};

TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto dim = loader.getDimensionInfo();

int zoom_ratio = 20;
TEST(circleAgentSubGraph, test) {
    Canvas canvas("circle agent", dim[0], dim[1], .1, zoom_ratio);
    // fake instances
    Instances<2> instances = {{{28, 13}, {27, 15}}, {{27, 21}, {6, 2}} };
    Agents<2> agents;
    CircleAgent<2> a1(.5), a2(1.5);
    agents.push_back((Agent<2>*)(&a1));
    agents.push_back((Agent<2>*)(&a2));
    LargeAgentCBS<2> lacbs(instances, agents, dim, is_occupied);

    while(true) {
        canvas.resetCanvas();
        canvas.drawGridMap(dim, is_occupied);
//        int r1 = a1.radius_*zoom_ratio, r2 = a2.radius_*zoom_ratio;
//        canvas.drawCircleInt(instances[0].first[0], instances[0].first[1], r1);
//        canvas.drawCircleInt(instances[1].first[0], instances[1].first[1], r2);
        for(int i=0; i<lacbs.agent_sub_graphs_[0].all_poses_.size(); i++) {
            const auto& node_ptr = lacbs.agent_sub_graphs_[0].all_poses_[i];
            if(node_ptr != nullptr) {
                canvas.drawGrid(node_ptr->pt_[0], node_ptr->pt_[1], COLOR_TABLE[0]);
            }
        }
        canvas.show();
    }

}