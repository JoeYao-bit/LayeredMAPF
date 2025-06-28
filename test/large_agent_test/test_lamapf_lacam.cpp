//
// Created by yaozhuo on 2024/6/21.
//


#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include "../../algorithm/LA-MAPF/circle_shaped_agent.h"
#include "../../algorithm/LA-MAPF/block_shaped_agent.h"

#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"
#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"

#include "../../freeNav-base/visualization/canvas/canvas.h"
#include "../../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../test_data.h"
#include "common_interfaces.h"

using namespace freeNav;
using namespace freeNav::LayeredMAPF;
using namespace freeNav::LayeredMAPF::LA_MAPF;


TEST(sort, test) {
    std::vector<std::pair<size_t, int> > ids = {{0, 6}, {1, 3}, {2, 4}, {3, 5}, {4, 1}, {5, 0}};
    std::sort(ids.begin(), ids.end(), [&](const std::pair<size_t, int>& v, const std::pair<size_t, int>& u) {
        return v.second < u.second;
    });

    for(const auto& temp_pair : ids) {
        std::cout << "{" << temp_pair.first << ", " << temp_pair.second << "} ";
    }
    std::cout << std::endl;
    // {5, 0} {4, 1} {1, 3} {2, 4} {3, 5} {0, 6}
}

TEST(division, test) {
    std::cout << 0/2 << " " << 1/2 << " " << 2/2 << " " << 3/2 << std::endl;
    std::cout << 0/2 << " " << 1/2 << " " << 2/2 << " " << 3/2 << " " << 4/2 << " " << 5/2 << std::endl;

}

int main() {
    return 0;
}