//
// Created by yaozhuo on 6/23/26.
//
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <climits>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/copy.hpp>


using namespace std;
//using namespace boost;

// 有向图定义，标准兼容写法
typedef boost::adjacency_list<
        boost::vecS,        // 出边容器
        boost::vecS,        // 顶点容器
        boost::directedS    // 有向图
> Graph;

// 基础类型萃取，杜绝 category 相关报错
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::graph_traits<Graph>::out_edge_iterator OutEdgeIter;

struct EdgeScore
{
    Vertex u, v;
    bool is_bi;
    int cycle_len;
    double inner_out_ratio;

    bool operator<(const EdgeScore& other) const
    {
        if (is_bi != other.is_bi)
            return !is_bi;
        if (cycle_len != other.cycle_len)
            return cycle_len > other.cycle_len;
        return inner_out_ratio < other.inner_out_ratio;
    }
};

// BFS 仅在最大SCC内部沿有向边搜索最短路径
int BfsShortestInGiant(const Graph& g, Vertex src, Vertex dst, const unordered_set<Vertex>& giant)
{
    unordered_map<Vertex, int> dist;
    queue<Vertex> q;
    dist[src] = 0;
    q.push(src);

    while (!q.empty())
    {
        Vertex u = q.front();
        q.pop();
        OutEdgeIter ei, ei_end;
        for (tie(ei, ei_end) = out_edges(u, g); ei != ei_end; ++ei)
        {
            Vertex v = target(*ei, g);
            if (!giant.count(v)) continue;
            if (dist.find(v) == dist.end())
            {
                dist[v] = dist[u] + 1;
                q.push(v);
                if (v == dst)
                    return dist[v];
            }
        }
    }
    return INT_MAX;
}

// 判断双向边：同时存在 u->v 与 v->u
bool IsBiEdge(const Graph& g, Vertex u, Vertex v)
{
    Edge e;
    bool exist;
    tie(e, exist) = edge(v, u, g);
    return exist;
}

// 提取全局最大强连通分量
unordered_set<Vertex> GetGiantSCC(const Graph& g)
{
    vector<int> comp(num_vertices(g));
    strong_components(g, comp.data());

    unordered_map<int, vector<Vertex>> comp_map;
    for (Vertex v = 0; v < num_vertices(g); ++v)
    {
        comp_map[comp[v]].push_back(v);
    }

    int max_cid = 0;
    size_t max_size = 0;
    for (auto& p : comp_map)
    {
        if (p.second.size() > max_size)
        {
            max_size = p.second.size();
            max_cid = p.first;
        }
    }

    unordered_set<Vertex> giant;
    for (Vertex v : comp_map[max_cid])
        giant.insert(v);
    return giant;
}

// 仅对最大SCC内部边打分排序
vector<EdgeScore> ScoreOnlyGiantEdges(const Graph& g, const unordered_set<Vertex>& giant)
{
    vector<EdgeScore> edge_list;
    unordered_map<Vertex, int> inner_out_cnt;

    // 统计SCC内部出边数量
    for (Vertex u : giant)
    {
        int cnt = 0;
        OutEdgeIter ei, ei_end;
        for (tie(ei, ei_end) = out_edges(u, g); ei != ei_end; ++ei)
        {
            Vertex v = target(*ei, g);
            if (giant.count(v)) cnt++;
        }
        inner_out_cnt[u] = cnt;
    }

    // 遍历最大SCC内所有出边
    for (Vertex u : giant)
    {
        OutEdgeIter ei, ei_end;
        for (tie(ei, ei_end) = out_edges(u, g); ei != ei_end; ++ei)
        {
            Vertex v = target(*ei, g);
            if (!giant.count(v)) continue;

            EdgeScore es;
            es.u = u;
            es.v = v;
            es.is_bi = IsBiEdge(g, u, v);

            int dist_vu = BfsShortestInGiant(g, v, u, giant);
            es.cycle_len = (dist_vu == INT_MAX) ? 10000 : (1 + dist_vu);

            int total_out = out_degree(u, g);
            es.inner_out_ratio = (total_out == 0) ? 0.0 : static_cast<double>(inner_out_cnt[u]) / total_out;
            edge_list.push_back(es);
        }
    }

    sort(edge_list.rbegin(), edge_list.rend());
    return edge_list;
}

// 标准ND贪心主循环
void RunNetworkDismantling(Graph& g)
{
    int iter = 0;
    while (true)
    {
        iter++;
        unordered_set<Vertex> giant = GetGiantSCC(g);
        size_t gscc_size = giant.size();
        if (gscc_size <= 1)
        {
            cout << "\n[Finish] All SCC size <= 1, total iter: " << iter << "\n";
            break;
        }
        cout << "\nIter " << iter << " | Max SCC size = " << gscc_size << "\n";

        vector<EdgeScore> sorted_edges = ScoreOnlyGiantEdges(g, giant);
        bool cut_ok = false;

        for (const EdgeScore& e : sorted_edges)
        {
            Graph tempG;
            copy_graph(g, tempG);
            remove_edge(e.u, e.v, tempG);

            unordered_set<Vertex> new_giant = GetGiantSCC(tempG);
            if (new_giant.size() < gscc_size)
            {
                g.swap(tempG);
                cut_ok = true;
                cout << "  Remove edge: " << e.u << " -> " << e.v << "\n";
                break;
            }
        }
        if (!cut_ok)
        {
            cout << "[Stop] No edge can shrink max SCC\n";
            break;
        }
    }
}

// 测试用例1：单一有向大环
void BuildTestGraph1(Graph& g)
{
    add_edge(0, 1, g);
    add_edge(1, 0, g);
    add_edge(1, 2, g);
    add_edge(2, 3, g);
    add_edge(3, 1, g);
}

// 测试用例2：多独立SCC，仅最大分量参与打分
void BuildTestGraph2(Graph& g)
{
    // 最大SCC {0,1,2,3}
    add_edge(0, 1, g); add_edge(1, 0, g);
    add_edge(1, 2, g); add_edge(2, 3, g); add_edge(3, 1, g);
    // 小二元环
    add_edge(4, 5, g); add_edge(5, 4, g);
    add_edge(6, 7, g); add_edge(7, 6, g);
}

int main()
{
    cout << "===== Test Case 1: Single Giant SCC ====\n";
    {
        Graph g1;
        BuildTestGraph1(g1);
        RunNetworkDismantling(g1);
    }

    cout << "\n=========================================\n";
    cout << "===== Test Case 2: Multiple Separate SCC ====\n";
    {
        Graph g2;
        BuildTestGraph2(g2);
        RunNetworkDismantling(g2);
    }
    return 0;
}
