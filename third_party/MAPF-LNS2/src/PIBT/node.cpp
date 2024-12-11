/*
 * node.cpp
 *
 * Purpose: Node
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */


#include "MAPF-LNS2/inc/PIBT/node.h"
namespace MAPF_LNS2 {

    int Node::cntIndex = 0;

    Node::Node(int _id) : id(_id), index(cntIndex) {
        ++cntIndex;
        pos = Vec2f(0, 0);
    }
}