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
    using namespace boost;
    using Vertex = size_t;

    // 定义图的类型
    typedef adjacency_list<vecS, vecS, directedS, Vertex> Graph;

    // 创建一个有向图
    Graph g(8);

    // 添加边
    add_edge(Vertex(0), Vertex(1), g);
    add_edge(Vertex(1), Vertex(2), g);
    add_edge(Vertex(2), Vertex(0), g);
    add_edge(Vertex(3), Vertex(4), g);
    add_edge(Vertex(4), Vertex(5), g);
    add_edge(Vertex(5), Vertex(3), g);
    add_edge(Vertex(6), Vertex(7), g);

    // 向量存储每个顶点的强连通分量编号
    std::vector<int> component(num_vertices(g));

    // 计算强连通分量，并返回分量的数量
    int num = strong_components(g, &component[0]);

    // 输出结果
    std::cout << "Total number of strong components: " << num << std::endl;
    for (size_t i = 0; i < component.size(); ++i) {
        std::cout << "Vertex " << i << " is in component " << component[i] << std::endl;
    }

}