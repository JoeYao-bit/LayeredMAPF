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

Pointi<2> pt1;
int current_subgraph_id = 0;
bool draw_all_subgraph_node = false;

TEST(circleAgentSubGraph, test) {
    Canvas canvas("circle agent", dim[0], dim[1], .1, zoom_ratio);
    auto mouse_call_back = [](int event, int x, int y, int flags, void *) {
        if(event == cv::EVENT_LBUTTONDOWN) {
            std::cout << "** get pt (" << x << ", " << y <<  ") **" << std::endl;
            pt1[0] = x;
            pt1[1] = y;
        }
    };
    canvas.setMouseCallBack(mouse_call_back);

    // fake instances
    Instances<2> instances = {{{28, 13}, {27, 15}}, {{27, 21}, {6, 2}} };
    Agents<2> agents;
    CircleAgent<2> a1(.5), a2(.4);
    agents.push_back((Agent<2>*)(&a1));
    agents.push_back((Agent<2>*)(&a2));
    LargeAgentCBS<2> lacbs(instances, agents, dim, is_occupied);

    while(true) {
        canvas.resetCanvas();
        canvas.drawGridMap(dim, is_occupied);
//        int r1 = a1.radius_*zoom_ratio, r2 = a2.radius_*zoom_ratio;
//        canvas.drawCircleInt(instances[0].first[0], instances[0].first[1], r1);
//        canvas.drawCircleInt(instances[1].first[0], instances[1].first[1], r2);
        const auto& current_subgraph = lacbs.agent_sub_graphs_[current_subgraph_id];

        if(draw_all_subgraph_node) {
            for (int i = 0; i < current_subgraph.all_poses_.size(); i++) {
                const auto &node_ptr = current_subgraph.all_poses_[i];
                if (node_ptr != nullptr) {
                    canvas.drawGrid(node_ptr->pt_[0], node_ptr->pt_[1], COLOR_TABLE[0]);
                }
            }
        }
        Id id = PointiToId(pt1, dim);
        int orient = 0;
        //for(int orient=0; orient<4; orient++)
        {
            auto current_node  = current_subgraph.all_poses_[id*4 + orient];
            auto current_edges = current_subgraph.all_edges_[id*4 + orient];
            if (current_node != nullptr) {
                canvas.drawGrid(current_node->pt_[0], current_node->pt_[1], COLOR_TABLE[1]);
                for (const auto &edge_id : current_edges) {
                    if (current_subgraph.all_poses_[edge_id]->pt_ == pt1) { continue; }
                    canvas.drawGrid(current_subgraph.all_poses_[edge_id]->pt_[0],
                                    current_subgraph.all_poses_[edge_id]->pt_[1],
                                    COLOR_TABLE[2]);
                }
            }
        }
        char key = canvas.show();
        if(key == 'w') {
            current_subgraph_id ++;
            current_subgraph_id = current_subgraph_id % instances.size();
            std::cout << " switch to subgraph " << current_subgraph_id << std::endl;
        } else if(key == 's') {
            current_subgraph_id --;
            current_subgraph_id = (current_subgraph_id + instances.size()) % instances.size();
            std::cout << " switch to subgraph " << current_subgraph_id << std::endl;
        } else if(key == 'a') {
            draw_all_subgraph_node = !draw_all_subgraph_node;
        }
    }

}