//
// Created by yaozhuo on 2024/8/10.
//

#include "../../algorithm/LA-MAPF/large_agent_instance_decomposition.h"

#include <gtest/gtest.h>

#include "common_interfaces.h"

#include <boost/graph/subgraph.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <boost/config.hpp>
#include <vector>
#include <boost/graph/connected_components.hpp>

#include <iostream>

#include <boost/graph/strong_components.hpp>

#include <boost/graph/graphviz.hpp>
#include <boost/graph/graph_utility.hpp>

using namespace freeNav::LayeredMAPF::LA_MAPF;


TEST(basic_test, LA_MAPF_decomposition) {
    const std::string file_path = map_test_config.at("la_ins_path");
    loadInstanceAndDecomposition<2>(file_path);
}

TEST(boost_strong_componnet, test) {

    using namespace boost;
    using Vertex = size_t;

    // 定义图的类型
    typedef adjacency_list<vecS, vecS, directedS, Vertex> Graph;

    // 创建一个有向图
    Graph g;

    //add_edge(Vertex(13), Vertex(13),g);
    add_vertex(Vertex(13),g);

    // 添加边
    add_edge(Vertex(11), Vertex(1), g);
    add_edge(Vertex(1), Vertex(2), g);
    add_edge(Vertex(2), Vertex(11), g);

    add_edge(Vertex(3), Vertex(4), g);
    add_edge(Vertex(4), Vertex(5), g);
    add_edge(Vertex(5), Vertex(3), g);

    add_edge(Vertex(5), Vertex(6), g);

    add_edge(Vertex(6), Vertex(7), g);

    add_edge(Vertex(15), Vertex(7), g);
    add_edge(Vertex(15), Vertex(7), g);

    add_edge(Vertex(7), Vertex(7), g);
    add_edge(Vertex(15), Vertex(15), g);



    // 向量存储每个顶点的强连通分量编号
    std::vector<int> component(num_vertices(g));

    // 计算强连通分量，并返回分量的数量
    int num = strong_components(g, &component[0]);

    // 输出结果
    std::cout << "Total number of strong components: " << num << std::endl;
    for (size_t i = 0; i < component.size(); ++i) {
        std::cout << "Vertex " << i << " is in component " << component[i] << std::endl;
    }

/*
  Sample output:
  A directed graph:
  a --> b f h
  b --> c a
  c --> d b
  d --> e
  e --> d
  f --> g
  g --> f d
  h --> i
  i --> h j e c
  j -->

  Total number of components: 4
  Vertex a is in component 3
  Vertex b is in component 3
  Vertex c is in component 3
  Vertex d is in component 0
  Vertex e is in component 0
  Vertex f is in component 1
  Vertex g is in component 1
  Vertex h is in component 3
  Vertex i is in component 3
  Vertex j is in component 2
 */

}

