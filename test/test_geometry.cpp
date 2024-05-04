//
// Created by yaozhuo on 2024/5/5.
//

#include <gtest/gtest.h>
#include "../algorithm/LA-MAPF/shaped_agent.h"

using namespace freeNav;
using namespace freeNav::LayeredMAPF::LA_MAPF;

CircleAgent<2> c1(2.5);
CircleAgent<2> c2(3.5);

int main(void) {
    Pose<2> p1, p2, p3, p4;
    p1.pt_ = Pointi<2>({1,1});
    p2.pt_ = Pointi<2>({1,2});
    p3.pt_ = Pointi<2>({1,3});
    p4.pt_ = Pointi<2>({4,1});
    std::cout << "is collide" << isCollide(c1, p1, p2, c2, p3, p4) << std::endl;



    return 0;
}