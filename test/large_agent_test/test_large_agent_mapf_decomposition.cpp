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
    const std::string file_path = map_test_config.at("crc_ins_path");
    loadInstanceAndDecomposition<CircleAgent<2> >(file_path);
}

TEST(boost_strong_componnet, test) {

//    using namespace std;
//    using namespace boost;
//
//// Underlying graph representation and implementation
////typedef adjacency_list_traits<vecS, vecS, directedS> Traits;
//
//// Graph representation
//    typedef subgraph< adjacency_list<vecS, vecS, directedS,
//            property<vertex_color_t, int>, property<edge_index_t, int> > > Graph;
//
//// Iterating over vertices and edges
//    typedef graph_traits<Graph>::vertex_iterator vertex_iter;
////typedef graph_traits<Graph>::edge_iterator edge_iter;
//
//    Graph g;
//    add_edge(0,1, g);
//    add_edge(1,4, g);
//    add_edge(4,0, g);
//    add_edge(3,0, g);
//
//    add_edge(2,5, g);
//
//    std::vector<int> comp(num_vertices(g));
//    int num = connected_components (g, comp.data());
//    std::cout << std::endl;
////    std::vector < int >::iterator i;
////    std::cout << "Total number of components: " << num << std::endl;
////    for (i = comp.begin(); i != comp.end(); ++i)
////        std::cout << "Vertex " << i - comp.begin()
////                  << " is in component " << *i << std::endl;
//
//    std::vector<Graph* > comps(num);
//    for(size_t i=0;i<num;++i) {
//        comps[i] = & g.create_subgraph();
//    }
//    for(size_t i=0;i<num_vertices(g);++i) {
//        cout<<"add vetex "<<i<<"to sub graph "<<comp[i]<<endl;
//        add_vertex(i, *comps[comp[i]]);
//    }
////    pair<vertex_iter, vertex_iter> vip;
//
////    cout << "Vertices in g  = [ ";
////    vip = vertices(g);
////    for(vertex_iter vi = vip.first; vi != vip.second; ++vi) {
////        cout << *vi << " ";
////    }
////    cout << "]" << endl;
//    for(size_t i=0;i<num;i++)
//    {
//        cout << "Vertices (local) in comps[i]' = [ ";
//        pair<vertex_iter, vertex_iter> lvip;
//        lvip = vertices(*comps[i]);
//
//        for(vertex_iter vi = lvip.first; vi != lvip.second; ++vi)
//        {
//            cout << (*comps[i]).local_to_global(*vi) << " ";
//        }
//
//        cout << "]" << endl;
//    }


// following code is more efficient than previous code
//    using namespace boost;
//    using Vertex = size_t;
//
//    // 定义图的类型
//    typedef adjacency_list<vecS, vecS, directedS, Vertex> Graph;
//
//    // 创建一个有向图
//    Graph g;
//
//    //add_edge(Vertex(13), Vertex(13),g);
//    add_vertex(Vertex(13),g);
//
//    // 添加边
//    add_edge(Vertex(11), Vertex(1), g);
//    add_edge(Vertex(1), Vertex(2), g);
//    add_edge(Vertex(2), Vertex(11), g);
//
//    add_edge(Vertex(3), Vertex(4), g);
//    add_edge(Vertex(4), Vertex(5), g);
//    add_edge(Vertex(5), Vertex(3), g);
//
//    add_edge(Vertex(5), Vertex(6), g);
//
//    add_edge(Vertex(6), Vertex(7), g);
//
//    add_edge(Vertex(15), Vertex(7), g);
//    add_edge(Vertex(15), Vertex(7), g);
//
//    add_edge(Vertex(7), Vertex(7), g);
//    add_edge(Vertex(15), Vertex(15), g);
//
//
//
//    // 向量存储每个顶点的强连通分量编号
//    std::vector<int> component(num_vertices(g));
//
//    // 计算强连通分量，并返回分量的数量
//    int num = strong_components(g, &component[0]);
//
//    // 输出结果
//    std::cout << "Total number of strong components: " << num << std::endl;
//    for (size_t i = 0; i < component.size(); ++i) {
//        std::cout << "Vertex " << i << " is in component " << component[i] << std::endl;
//    }

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

    using namespace boost;
    const char* name = "abcdefghij";

//    GraphvizDigraph G;
//    read_graphviz("scc.dot", G);
//
//    std::cout << "A directed graph:" << std::endl;
//    print_graph(G, name);
//    std::cout << std::endl;
//
//    typedef graph_traits<GraphvizGraph>::vertex_descriptor Vertex;
//
//    std::vector<int> component(num_vertices(G)), discover_time(num_vertices(G));
//    std::vector<default_color_type> color(num_vertices(G));
//    std::vector<Vertex> root(num_vertices(G));
//    int num = strong_components(G, &component[0],
//                                root_map(&root[0]).
//                                        color_map(&color[0]).
//                                        discover_time_map(&discover_time[0]));
//
//    std::cout << "Total number of components: " << num << std::endl;
//    std::vector<int>::size_type i;
//    for (i = 0; i != component.size(); ++i)
//        std::cout << "Vertex " << name[i]
//                  <<" is in component " << component[i] << std::endl;

}

TEST(boost_strong_componnet_new, test) {
    GraphForTest graph;
//    graph.addEdge(1, 2);
//    graph.addEdge(2, 3);
//    graph.addEdge(3, 1);
//
//    graph.addEdge(2, 4);
//
//    graph.addEdge(4, 5);
//    graph.addEdge(5, 6);
//    graph.addEdge(6, 4);


//    graph.addEdge(11, 1);
//    graph.addEdge(1, 2);
//    graph.addEdge(2, 11);
//
//    graph.addEdge(3, 4);
//    graph.addEdge(4, 5);
//    graph.addEdge(5, 3);
//
//    graph.addEdge(5, 6);
//
//    graph.addEdge(6, 7);
//
//    graph.addEdge(15, 7);
//    graph.addEdge(15, 7);
//
//    graph.addEdge(7, 7);
//    graph.addEdge(15, 15);
//
//    TarjanForSCC tarjan_solver(graph.all_edges_);
//    tarjan_solver.tarjanForSCC();

    getStrongComponentFromGraph(graph.all_edges_, graph.all_backward_edges_);
}