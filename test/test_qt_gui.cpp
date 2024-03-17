#include <QApplication>

#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../freeNav-base/dependencies/massive_scene_loader/ScenarioLoader2D.h"
#include "../freeNav-base/dependencies/memory_analysis.h"

#include "../test/visualization/qt_gui/mainwindow.h"
#include "../test/test_data.h"

#include "../algorithm/instance_decomposition.h"

#include "EECBS/inc/driver.h"
#include "PBS/inc/driver.h"
#include "CBSH2-RTC/inc/driver.h"
#include "MAPF-LNS2/inc/driver.h"
#include "lacam/include/planner.hpp"
#include "lacam2/include/lacam2.hpp"
#include "pibt2/include/driver.h"
#include <libkahypar.h>

using namespace freeNav;
using namespace freeNav::LayeredMAPF;

// MAPFTestConfig_random_32_32_20 413.8 ms / layered slower, but faster after 160 agent
// MAPFTestConfig_maze_32_32_2 25.42 ms / layered faster, after 58 agent
// MAPFTestConfig_maze_32_32_4 54.914 ms / layered faster
// MAPFTestConfig_den312d  314.776 ms / layered faster，after 250 agent
// MAPFTestConfig_Berlin_1_256 56.385 ms / layered faster
// MAPFTestConfig_Paris_1_256 1005.82 ms / layered faster
// MAPFTestConfig_warehouse_10_20_10_2_1  5726.96 ms / layered faster， after 500 agent
// MAPFTestConfig_den520d 237.842 ms / layered faster， after 150 agent
// MAPFTestConfig_empty_32_32 2872.3 ms / layered faster
// MAPFTestConfig_simple
auto map_test_config = MAPFTestConfig_Paris_1_256;

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') { return false; }
    return true;
};

freeNav::TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);

// instance loader

auto is_occupied = [](const freeNav::Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto set_occupied = [](const freeNav::Pointi<2> & pt) { loader.setOccupied(pt); };

freeNav::IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

freeNav::SET_OCCUPIED_FUNC<2> set_occupied_func = set_occupied;

struct timezone tz;
struct timeval tv_pre;
struct timeval tv_after;

using namespace std;
//using namespace boost;

// Underlying graph representation and implementation
//typedef adjacency_list_traits<vecS, vecS, directedS> Traits;

//// Graph representation
//typedef boost::subgraph< boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
//        boost::property<vertex_color_t, int>, boost::property<edge_index_t, int> > > Graph;
//
//// Iterating over vertices and edges
//typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
////typedef graph_traits<Graph>::edge_iterator edge_iter;
//
//int main1(void) {
//    Graph g;
//    add_edge(0, 1, g);
//    add_edge(1, 4, g);
//    add_edge(4, 0, g);
//    add_edge(3, 2, g);
//    add_edge(2, 0, g);
//    //add_edge(1, 3, g);
//
//    std::vector<int> comp(num_vertices(g));
//
//    int num = boost::strong_components(g, comp.data());
//
//    std::cout << std::endl;
//    std::vector<int>::iterator i;
//    std::cout << "Total number of components: " << num << std::endl;
//    for (i = comp.begin(); i != comp.end(); ++i)
//        std::cout << "Vertex " << i - comp.begin()
//                  << " is in component " << *i << std::endl;
//
//    std::vector<Graph *> comps(num);
//    for (size_t i = 0; i < num; ++i) {
//        comps[i] = &g.create_subgraph();
//    }
//    for (size_t i = 0; i < num_vertices(g); ++i) {
//        //cout << "add vertex " << i << " to sub graph " << comp[i] << endl;
//        add_vertex(i, *comps[comp[i]]);
//    }
//
////    pair<vertex_iter, vertex_iter> vip;
////
////    cout << "Vertices in g  = [ ";
////    vip = vertices(g);
////    for (vertex_iter vi = vip.first; vi != vip.second; ++vi) {
////        cout << *vi << " ";
////    }
////    cout << "]" << endl;
//
//    for (size_t i = 0; i < num; i++) {
//        cout << "Vertices (local) in comps[i]' = [ ";
//        pair<vertex_iter, vertex_iter> lvip;
//        lvip = vertices(*comps[i]);
//
//        for (vertex_iter vi = lvip.first; vi != lvip.second; ++vi) {
//            cout << (*comps[i]).local_to_global(*vi) << " ";
//        }
//
//        cout << "]" << endl;
//    }
//
//    return 0;
//}

int main(int argc, char *argv[])
{
    MemoryRecorder memory_recorder(50);

    // start conflict based search
    std::cout << "start Conflict Based Search" << std::endl;
    const auto& dim = loader.getDimensionInfo();
    freeNav::Instances<2> ists;
    ScenarioLoader2D sl(map_test_config.at("scene_path").c_str());
    int max_count_of_case = atoi((map_test_config.at("agent_num")).c_str());
    int count_of_experiments = sl.GetNumExperiments();


    // load experiment data
    for (int i = 0; i < std::min(max_count_of_case, count_of_experiments); i++) {
        const auto &experiment = sl.GetNthExperiment(i);
        freeNav::Pointi<2> pt1({experiment.GetStartX(), experiment.GetStartY()});
        freeNav::Pointi<2> pt2({experiment.GetGoalX(), experiment.GetGoalY()});
        freeNav::Instance<2> ist = {pt1, pt2};
        //std::cout << " start, target = " << pt1 << ", " << pt2 << std::endl;
        ists.push_back(ist);
    }
    //std::cout << "get " << ists.size() << " instances" << std::endl;
    //std::cout << " agent 428 " << ists[428].first << ", " << ists[428].second << std::endl;
    freeNav::Paths<2> multiple_paths;
    memory_recorder.clear();
    sleep(1);
    float base_usage = memory_recorder.getCurrentMemoryUsage();

    gettimeofday(&tv_pre, &tz);
    // comparing to the raw version, the layered vision will add more static constraint
    // so avoid the copy of static constraint table, will increase the performance of layered mapf
    //if(CBS_Li::ct != nullptr) { delete CBS_Li::ct; }
    multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, CBS_Li::eecbs_MAPF, CBS_Li::eecbs_MAPF);
    //multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, LaCAM::lacam_MAPF, CBS_Li::eecbs_MAPF, false, 60);
    //multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, PBS_Li::pbs_MAPF, CBS_Li::eecbs_MAPF, true, 60);
    //multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, CBSH2_RTC::CBSH2_RTC_MAPF, CBS_Li::eecbs_MAPF, true, 60);
    //multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, MAPF_LNS::MAPF_LNS_MAPF, CBS_Li::eecbs_MAPF, true, 60);
    //multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, LaCAM2::lacam2_MAPF, CBS_Li::eecbs_MAPF, false, 60);
    //multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, PIBT_2::pibt_MAPF, CBS_Li::eecbs_MAPF, false, 60);
    gettimeofday(&tv_after, &tz);
    double layered_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << multiple_paths.size() << " agents " << std::endl;
    std::cout << "-- layered lacam end in " << layered_cost << "ms" << std::endl << std::endl;
    std::cout << " is solution valid ? " << validateSolution<2>(multiple_paths) << std::endl;

    std::set<int> visited_grid_during_lacam = PIBT_2::visited_grid_;
    std::cout << " visited_grid_during_lacam size " << visited_grid_during_lacam.size() << std::endl;
    sleep(1);
    float maximal_usage = memory_recorder.getMaximalMemoryUsage();
    {
        std::cout << "layered mapf maximal usage = " << maximal_usage - base_usage << " MB" << std::endl;
    }

    return 0;
    // start GUI for visualization
//    Q_INIT_RESOURCE(images);
//    QApplication app(argc, argv);
//    app.setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);
//
//    MainWindow window(is_occupied, loader.getDimensionInfo(), ists, visited_grid_during_lacam);
//    window.show();
//
//    return app.exec();
}






























