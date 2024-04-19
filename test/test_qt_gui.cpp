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

// MAPFTestConfig_random_32_32_20
// MAPFTestConfig_maze_32_32_2
// MAPFTestConfig_maze_32_32_4
// MAPFTestConfig_den312d
// MAPFTestConfig_Berlin_1_256
// MAPFTestConfig_Paris_1_256
// MAPFTestConfig_warehouse_10_20_10_2_1
// MAPFTestConfig_den520d
// MAPFTestConfig_empty_32_32
// MAPFTestConfig_simple
// MAPFTestConfig_ht_chantry
// MAPFTestConfig_lak303d
// MAPFTestConfig_maze_128_128_2
// MAPFTestConfig_maze_128_128_10
// MAPFTestConfig_random_64_64_10
// MAPFTestConfig_random_64_64_20
// MAPFTestConfig_room_64_64_16
// MAPFTestConfig_room_64_64_8
// MAPFTestConfig_room_32_32_4
// MAPFTestConfig_warehouse_10_20_10_2_2
// MAPFTestConfig_warehouse_20_40_10_2_1
// MAPFTestConfig_warehouse_20_40_10_2_2
auto map_test_config = MAPFTestConfig_lak303d;

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
        ists.push_back(ist);
    }
    freeNav::Paths<2> multiple_paths;
    memory_recorder.clear();
    sleep(1);
    float base_usage = memory_recorder.getCurrentMemoryUsage();

    gettimeofday(&tv_pre, &tz);
    // comparing to the raw version, the layered vision will add more static constraint
    // so avoid the copy of static constraint table, will increase the performance of layered mapf

    multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, CBS_Li::eecbs_MAPF, CBS_Li::eecbs_MAPF, true);

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

    // start GUI for visualization
    Q_INIT_RESOURCE(images);
    QApplication app(argc, argv);
    app.setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);

    MainWindow window(is_occupied, loader.getDimensionInfo(), ists, visited_grid_during_lacam);
    window.show();

    return app.exec();
}






























