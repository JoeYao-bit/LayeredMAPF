//
// Created by yaozhuo on 6/23/26.
//
#include "../algorithm/network_dismantling.h"

void RunNetworkDismantling(ND::Graph& g)
{
    int iter = 0;
    while (true)
    {
        iter++;
        boost::unordered_set<ND::Vertex> giant = ND::GetGiantSCC(g);
        size_t gscc_size = giant.size();
        if (gscc_size <= 1)
        {
            std::cout << "\n[Finish] All SCC size <= 1, total iter: " << iter << "\n";
            break;
        }
        std::cout << "\nIter " << iter << " | Current max SCC size = " << gscc_size << "\n";

        std::vector<ND::EdgeScore> sorted_edges = ND::ScoreOnlyGiantEdges(g, giant);
        bool cut_ok = false;

        for (const ND::EdgeScore& e : sorted_edges)
        {
            size_t origin_comp_size = ND::GetComponentSize(g, e.u);

            ND::Graph tempG;
            copy_graph(g, tempG);
            remove_edge(e.u, e.v, tempG);

            //size_t new_comp_size = GetComponentSize(tempG, e.u);
            size_t new_comp_size = ND::GetLongestCycleOfVertex(tempG, e.u);
            if (new_comp_size < origin_comp_size)
            {
                g.swap(tempG);
                cut_ok = true;
                std::cout << "  Remove edge: " << e.u << " -> " << e.v
                     << " | Component shrank: " << origin_comp_size << " -> " << new_comp_size << "\n";
                break;
            }
        }
        if (!cut_ok)
        {
            std::cout << "[Stop] No edge can shrink any giant component\n";
            break;
        }
    }
}

// ===================== 新增：节点所在最长环路工具函数（仅定义，主流程不调用） =====================
// DFS 回溯枚举所有包含start的简单有向环
void DfsFindCycles(
        const ND::Graph& g,
        ND::Vertex cur,
        ND::Vertex start,
        std::vector<ND::Vertex>& path,
        boost::unordered_set<ND::Vertex>& visited,
        std::vector<std::vector<ND::Vertex>>& all_cycles
)
{
    if (cur == start && path.size() > 1)
    {
        all_cycles.push_back(path);
        return;
    }
    if (visited.count(cur))
        return;

    visited.insert(cur);
    path.push_back(cur);

    ND::OutEdgeIter ei, ei_end;
    for (tie(ei, ei_end) = out_edges(cur, g); ei != ei_end; ++ei)
    {
        ND::Vertex nxt = target(*ei, g);
        DfsFindCycles(g, nxt, start, path, visited, all_cycles);
    }

    path.pop_back();
    visited.erase(cur);
}

// 获取所有包含顶点v的简单有向环
std::vector<std::vector<ND::Vertex>> GetAllCyclesContainingVertex(const ND::Graph& g, ND::Vertex v)
{
    std::vector<std::vector<ND::Vertex>> cycles;
    std::vector<ND::Vertex> path;
    boost::unordered_set<ND::Vertex> vis;
    DfsFindCycles(g, v, v, path, vis, cycles);
    return cycles;
}

// 返回：该节点参与的最长环路 {长度, 环顶点集合}
//pair<int, vector<Vertex>>
int GetLongestCycleOfVertex(const ND::Graph& g, ND::Vertex v)
{
    auto cycles = GetAllCyclesContainingVertex(g, v);
    if (cycles.empty())
        return 1;

    int max_len = 0;
    std::vector<ND::Vertex> longest_cycle;
    for (auto& cy : cycles)
    {
        if ((int)cy.size() > max_len)
        {
            max_len = (int)cy.size();
            longest_cycle = cy;
        }
    }
//    return {max_len, longest_cycle};
    return max_len;
}
// ==========================================================================================

void BuildTestGraph_TwoEqualMax(ND::Graph& g)
{
    add_edge(0,1,g); add_edge(1,0,g);
    add_edge(1,2,g); add_edge(2,3,g); add_edge(3,1,g);
    add_edge(4,5,g); add_edge(5,4,g);
    add_edge(5,6,g); add_edge(6,7,g); add_edge(7,5,g);
}

void BuildTestGraph1(ND::Graph& g)
{
    add_edge(0, 1, g);
    add_edge(1, 0, g);
    add_edge(1, 2, g);
    add_edge(2, 3, g);
    add_edge(3, 1, g);
}

void BuildTestGraph2(ND::Graph& g)
{
    add_edge(0, 1, g); add_edge(1, 0, g);
    add_edge(1, 2, g); add_edge(2, 3, g); add_edge(3, 1, g);
    add_edge(4, 5, g); add_edge(5, 4, g);
    add_edge(6, 7, g); add_edge(7, 6, g);
}

int main()
{
    std::cout << "===== Test Case: Two equal largest SCC ====\n";
    {
        ND::Graph g;
        BuildTestGraph_TwoEqualMax(g);
        RunNetworkDismantling(g);

        // 示例：如需使用环路检测，取消下面注释即可调用
        /*
        auto [len, cycle] = GetLongestCycleOfVertex(g, 1);
        cout << "\nVertex 1 longest cycle length: " << len << "\n";
        for (auto v : cycle) cout << v << " ";
        cout << "\n";
        */
    }

    std::cout << "\n=========================================\n";
    std::cout << "===== Test Case 1: Single Giant SCC ====\n";
    {
        ND::Graph g1;
        BuildTestGraph1(g1);
        RunNetworkDismantling(g1);
    }

    std::cout << "\n=========================================\n";
    std::cout << "===== Test Case 2: Multiple small SCC ====\n";
    {
        ND::Graph g2;
        BuildTestGraph2(g2);
        RunNetworkDismantling(g2);
    }
    return 0;
}
