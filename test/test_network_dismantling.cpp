//
// Created by yaozhuo on 6/23/26.
//
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <climits>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/copy.hpp>

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

int BfsShortestInGiant(const Graph& g, Vertex src, Vertex dst, const boost::unordered_set<Vertex>& giant)
{
    boost::unordered_map<Vertex, int> dist;
    boost::queue<Vertex> q;
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

bool IsBiEdge(const Graph& g, Vertex u, Vertex v)
{
    Edge e;
    bool exist;
    tie(e, exist) = edge(v, u, g);
    return exist;
}

// 获取所有SCC，返回 顶点->分量ID、分量ID->顶点列表
void GetAllSCC(const Graph& g, vector<int>& comp, boost::unordered_map<int, vector<Vertex>>& comp_map)
{
    comp.assign(num_vertices(g), 0);
    strong_components(g, comp.data());
    comp_map.clear();
    for (Vertex v = 0; v < num_vertices(g); ++v)
    {
        comp_map[comp[v]].push_back(v);
    }
}

// 获取全局最大SCC集合
boost::unordered_set<Vertex> GetGiantSCC(const Graph& g)
{
    vector<int> comp;
    boost::unordered_map<int, vector<Vertex>> comp_map;
    GetAllSCC(g, comp, comp_map);

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

    boost::unordered_set<Vertex> giant;
    for (Vertex v : comp_map[max_cid])
        giant.insert(v);
    return giant;
}

// 获取顶点u当前所属分量的大小
size_t GetComponentSize(const Graph& g, Vertex u)
{
    vector<int> comp;
    boost::unordered_map<int, vector<Vertex>> comp_map;
    GetAllSCC(g, comp, comp_map);
    int cid = comp[u];
    return comp_map[cid].size();
}

vector<EdgeScore> ScoreOnlyGiantEdges(const Graph& g, const boost::unordered_set<Vertex>& giant)
{
    vector<EdgeScore> edge_list;
    boost::unordered_map<Vertex, int> inner_out_cnt;

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

void RunNetworkDismantling(Graph& g)
{
    int iter = 0;
    while (true)
    {
        iter++;
        boost::unordered_set<Vertex> giant = GetGiantSCC(g);
        size_t gscc_size = giant.size();
        if (gscc_size <= 1)
        {
            cout << "\n[Finish] All SCC size <= 1, total iter: " << iter << "\n";
            break;
        }
        cout << "\nIter " << iter << " | Current max SCC size = " << gscc_size << "\n";

        vector<EdgeScore> sorted_edges = ScoreOnlyGiantEdges(g, giant);
        bool cut_ok = false;

        for (const EdgeScore& e : sorted_edges)
        {
            // 记录这条边所在分量原始大小
            size_t origin_comp_size = GetComponentSize(g, e.u);

            Graph tempG;
            copy_graph(g, tempG);
            remove_edge(e.u, e.v, tempG);

            // 修复核心判断：不再对比全局最大，对比本条边所属分量的尺寸
            size_t new_comp_size = GetComponentSize(tempG, e.u);
            if (new_comp_size < origin_comp_size)
            {
                g.swap(tempG);
                cut_ok = true;
                cout << "  Remove edge: " << e.u << " -> " << e.v
                     << " | Component shrank: " << origin_comp_size << " -> " << new_comp_size << "\n";
                break;
            }
        }
        if (!cut_ok)
        {
            cout << "[Stop] No edge can shrink any giant component\n";
            break;
        }
    }
}

// 测试用例：两个完全相同的最大SCC（复现你遇到的bug）
void BuildTestGraph_TwoEqualMax(Graph& g)
{
    // 第一个4节点环 0,1,2,3
    add_edge(0,1,g); add_edge(1,0,g);
    add_edge(1,2,g); add_edge(2,3,g); add_edge(3,1,g);
    // 第二个完全一样的4节点环 4,5,6,7，并列全局最大
    add_edge(4,5,g); add_edge(5,4,g);
    add_edge(5,6,g); add_edge(6,7,g); add_edge(7,5,g);
}

void BuildTestGraph1(Graph& g)
{
    add_edge(0, 1, g);
    add_edge(1, 0, g);
    add_edge(1, 2, g);
    add_edge(2, 3, g);
    add_edge(3, 1, g);
}

void BuildTestGraph2(Graph& g)
{
    add_edge(0, 1, g); add_edge(1, 0, g);
    add_edge(1, 2, g); add_edge(2, 3, g); add_edge(3, 1, g);
    add_edge(4, 5, g); add_edge(5, 4, g);
    add_edge(6, 7, g); add_edge(7, 6, g);
}

int main()
{
    cout << "===== Test Case: Two equal largest SCC (bug reproduce & fix) ====\n";
    {
        Graph g;
        BuildTestGraph_TwoEqualMax(g);
        RunNetworkDismantling(g);
    }

    cout << "\n=========================================\n";
    cout << "===== Test Case 1: Single Giant SCC ====\n";
    {
        Graph g1;
        BuildTestGraph1(g1);
        RunNetworkDismantling(g1);
    }

    cout << "\n=========================================\n";
    cout << "===== Test Case 2: Multiple small SCC ====\n";
    {
        Graph g2;
        BuildTestGraph2(g2);
        RunNetworkDismantling(g2);
    }
    return 0;
}
