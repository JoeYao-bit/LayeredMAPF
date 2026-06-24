//
// Created by yaozhuo on 6/25/26.
//

#ifndef LAYEREDMAPF_NETWORK_DISMANTLING_H
#define LAYEREDMAPF_NETWORK_DISMANTLING_H

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <climits>
#include <stack>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/copy.hpp>

namespace ND {

    using namespace std;
    using namespace boost;

    typedef adjacency_list<
            vecS,
            vecS,
            directedS
    > Graph;

    typedef graph_traits<Graph>::vertex_descriptor Vertex;
    typedef graph_traits<Graph>::edge_descriptor Edge;
    typedef graph_traits<Graph>::out_edge_iterator OutEdgeIter;

    vector<vector<Vertex>> GetAllCyclesContainingVertex(const Graph &g, Vertex v);

    int GetLongestCycleOfVertex(const Graph &g, Vertex v);

    struct EdgeScore {
        Vertex u, v;
        bool is_bi;
        int cycle_len;
        double inner_out_ratio;

        bool operator<(const EdgeScore &other) const {
            if (is_bi != other.is_bi)
                return !is_bi;
            if (cycle_len != other.cycle_len)
                return cycle_len > other.cycle_len;
            return inner_out_ratio < other.inner_out_ratio;
        }
    };


    int BfsShortestInGiant(const Graph &g, Vertex src, Vertex dst, const boost::unordered_set<Vertex> &giant) {
        boost::unordered_map<Vertex, int> dist;
        boost::queue<Vertex> q;
        dist[src] = 0;
        q.push(src);

        while (!q.empty()) {
            Vertex u = q.front();
            q.pop();
            OutEdgeIter ei, ei_end;
            for (tie(ei, ei_end) = out_edges(u, g); ei != ei_end; ++ei) {
                Vertex v = target(*ei, g);
                if (!giant.count(v)) continue;
                if (dist.find(v) == dist.end()) {
                    dist[v] = dist[u] + 1;
                    q.push(v);
                    if (v == dst)
                        return dist[v];
                }
            }
        }
        return INT_MAX;
    }

    bool IsBiEdge(const Graph &g, Vertex u, Vertex v) {
        Edge e;
        bool exist;
        tie(e, exist) = edge(v, u, g);
        return exist;
    }

    void GetAllSCC(const Graph &g, vector<int> &comp, boost::unordered_map<int, vector<Vertex>> &comp_map) {
        comp.assign(num_vertices(g), 0);
        strong_components(g, comp.data());
        comp_map.clear();
        for (Vertex v = 0; v < num_vertices(g); ++v) {
            comp_map[comp[v]].push_back(v);
        }
    }

    boost::unordered_set<Vertex> GetGiantSCC(const Graph &g) {
        vector<int> comp;
        boost::unordered_map<int, vector<Vertex>> comp_map;
        GetAllSCC(g, comp, comp_map);

        int max_cid = 0;
        size_t max_size = 0;
        for (auto &p : comp_map) {
            if (p.second.size() > max_size) {
                max_size = p.second.size();
                max_cid = p.first;
            }
        }

        boost::unordered_set<Vertex> giant;
        for (Vertex v : comp_map[max_cid])
            giant.insert(v);
        return giant;
    }

    size_t GetComponentSize(const Graph &g, Vertex u) {
        vector<int> comp;
        boost::unordered_map<int, vector<Vertex>> comp_map;
        GetAllSCC(g, comp, comp_map);
        int cid = comp[u];
        return comp_map[cid].size();
    }

    vector<EdgeScore> ScoreOnlyGiantEdges(const Graph &g, const boost::unordered_set<Vertex> &giant) {
        vector<EdgeScore> edge_list;
        boost::unordered_map<Vertex, int> inner_out_cnt;

// 遍历巨型SCC里每一个顶点u
        for (Vertex u : giant) {
            int cnt = 0;
            OutEdgeIter ei, ei_end;
            // 遍历u所有向外的出边 u→*
            for (tie(ei, ei_end) = out_edges(u, g); ei != ei_end; ++ei) {
                Vertex v = target(*ei, g);
                // 如果这条边的终点v也属于巨型SCC内部，计数+1
                if (giant.count(v)) cnt++;
            }
            // 记录u在giant内部的出边总数
            inner_out_cnt[u] = cnt;
        }


        for (Vertex u : giant) {
            OutEdgeIter ei, ei_end;
            // 遍历u全部出边
            for (tie(ei, ei_end) = out_edges(u, g); ei != ei_end; ++ei) {
                Vertex v = target(*ei, g);
                // 只保留【起点u、终点v都在巨型SCC内部】的边，跳过向外的边
                if (!giant.count(v)) continue;

                EdgeScore es;
                es.u = u;
                es.v = v;
                // 判断 u→v 和 v→u 是否同时存在双向边
                es.is_bi = IsBiEdge(g, u, v);

                // BFS：在巨型SCC子图里，求v到u的最短路径长度 dist_vu
                int dist_vu = BfsShortestInGiant(g, v, u, giant);
                // 环长度打分：
                // 若v无法走到u → 不存在环，cycle_len赋值惩罚值10000（不适合删这条边）
                // 若存在路径，环长 = 1（当前边u→v） + v到u的最短距离
                es.cycle_len = (dist_vu == INT_MAX) ? 10000 : (1 + dist_vu);

                // u总出边数（内部+指向外部）
                int total_out = out_degree(u, g);
                // 内部出边占全部出边的比值 [0,1]
                es.inner_out_ratio = (total_out == 0) ? 0.0 : static_cast<double>(inner_out_cnt[u]) / total_out;
                // 将这条边的所有特征存入候选边列表，后续排序选最小cycle_len的边断开
                edge_list.push_back(es);
            }
        }

        sort(edge_list.rbegin(), edge_list.rend());
        return edge_list;
    }

}
#endif //LAYEREDMAPF_NETWORK_DISMANTLING_H
