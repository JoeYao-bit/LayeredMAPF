//
// Created by yaozhuo on 2023/6/7.
//
#include "../freeNav-base/dependencies/thread_pool.h"
#include "../freeNav-base/dependencies/color_table.h"
#include "../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../freeNav-base/visualization/3d_viewer/3d_viewer.h"
#include "../freeNav-base/dependencies/memory_analysis.h"
#include "../freeNav-base/basic_elements/surface_process.h"

#include "../algorithm/layered_mapf.h"
#include "../algorithm/general_mapf_scene.h"

#include "../test/test_data.h"

#include "../third_party/EECBS/inc/driver.h"
#include "../third_party/PBS/inc/driver.h"
#include "../third_party/CBSH2-RTC/inc/driver.h"
#include "../third_party/MAPF-LNS2/inc/driver.h"
#include "../third_party/lacam/include/planner.hpp"
#include "../third_party/lacam2/include/lacam2.hpp"
#include "../third_party/pibt2/include/driver.h"

#include "../test/visualization/3d_viewer.h"

using namespace freeNav;
using namespace freeNav::LayeredMAPF;

Viewer3D* viewer_3d;
ThreadPool viewer_thread(1);

struct timezone tz;
struct timeval tv_pre;
struct timeval tv_after;

bool plan_finish = false;

Pointi<3> pt1, pt2;
GridPtr<3> sg1 = std::make_shared<Grid<3>>(),
           sg2 = std::make_shared<Grid<3>>();

// MAPFTestConfig_random_32_32_20 413.8 ms / layered slower, but faster after 160 agent
// MAPFTestConfig_maze_32_32_2 25.42 ms / layered faster, after 58 agent
// MAPFTestConfig_maze_32_32_4 54.914 ms / layered faster
// MAPFTestConfig_den312d  314.776 ms / layered faster，after 250 agent
// MAPFTestConfig_Berlin_1_256 56.385 ms / layered faster
// MAPFTestConfig_Paris_1_256 1005.82 ms / layered faster
// MAPFTestConfig_warehouse_10_20_10_2_1  5726.96 ms / layered faster， after 500 agent
// MAPFTestConfig_den520d 237.842 ms / layered faster， after 150 agent
// MAPFTestConfig_empty_32_32 2872.3 ms / layered faster
auto map_test_config = MAPFTestConfig_empty_32_32;

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};

TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto set_occupied = [](const Pointi<2> & pt) { loader.setOccupied(pt); };

IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

SET_OCCUPIED_FUNC<2> set_occupied_func = set_occupied;

//int current_time_index = 0;
std::set<int> visited_grid_during_lacam;
int main(int argc, char** argv) {
    MemoryRecorder memory_recorder(50);
    std::cout << " map name " << map_test_config.at("map_path") << std::endl;
    // load mapf scene
    //GeneralMAPFScenePtr<2> scene_ptr;
    const auto& dim = loader.getDimensionInfo();
    freeNav::Instances<2> ists;
    ScenarioLoader2D sl(map_test_config.at("scene_path").c_str());
    int max_count_of_case = atoi((map_test_config.at("agent_num")).c_str());
    int count_of_experiments = sl.GetNumExperiments();

    // load experiment data
    for (int i = 0; i < std::min(max_count_of_case, count_of_experiments); i++) {
        const auto &experiment = sl.GetNthExperiment(i);
        Pointi<2> pt1({experiment.GetStartX(), experiment.GetStartY()});
        Pointi<2> pt2({experiment.GetGoalX(), experiment.GetGoalY()});
        freeNav::Instance<2> ist = {pt1, pt2};
        //std::cout << " start, target = " << pt1 << ", " << pt2 << std::endl;
        ists.push_back(ist);
    }
    std::cout << "get " << ists.size() << " instances" << std::endl;
    Paths<2> multiple_paths;
    // decomposition
    //freeNav::TCBS::MAPFInstanceDecomposition<2> decompose(ists, dim, is_occupied);
    sleep(1);
    memory_recorder.clear();
    float base_usage = memory_recorder.getCurrentMemoryUsage();
    int agent_num = 100;
    gettimeofday(&tv_pre, &tz);
    // comparing to the raw version, the layered vision will add more static constraint
    // so avoid the copy of static constraint table, will increase the performance of layered mapf
    //if(CBS_Li::ct != nullptr) { delete CBS_Li::ct; }
    multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, CBS_Li::eecbs_MAPF, CBS_Li::eecbs_MAPF, false, agent_num);

//    multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, LaCAM::lacam_MAPF, CBS_Li::eecbs_MAPF, false, agent_num);
//    multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, LaCAM2::lacam2_MAPF, CBS_Li::eecbs_MAPF, false, agent_num);

//    multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, PBS_Li::pbs_MAPF, CBS_Li::eecbs_MAPF, true, agent_num);
//    multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, CBSH2_RTC::CBSH2_RTC_MAPF, CBS_Li::eecbs_MAPF, true, agent_num);

//    multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, MAPF_LNS::LNS_MAPF, CBS_Li::eecbs_MAPF, true, agent_num);
//    multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, MAPF_LNS::AnytimeBCBS_MAPF, CBS_Li::eecbs_MAPF, true, agent_num);
//    multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, MAPF_LNS::AnytimeEECBS_MAPF, CBS_Li::eecbs_MAPF, true, agent_num);

//    multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, PIBT_2::pibt_MAPF, CBS_Li::eecbs_MAPF, false, agent_num);
//    multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, PIBT_2::pibt2_MAPF, CBS_Li::eecbs_MAPF, false, agent_num);
//    multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, PIBT_2::hca_MAPF, CBS_Li::eecbs_MAPF, false, agent_num);
//    multiple_paths = layeredMAPF<2>(ists, dim, is_occupied, PIBT_2::push_and_swap_MAPF, CBS_Li::eecbs_MAPF, false, agent_num);

    gettimeofday(&tv_after, &tz);
    double layered_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << multiple_paths.size() << " agents " << std::endl;
    std::cout << "-- layered mapf end in " << layered_cost << "ms" << std::endl << std::endl;
    std::cout << " is solution valid ? " << validateSolution<2>(multiple_paths) << std::endl;
    sleep(1);
    float maximal_usage = memory_recorder.getMaximalMemoryUsage();
    {
        std::cout << "layered mapf maximal usage = " << maximal_usage - base_usage << " MB" << std::endl;
    }


    int total_cost = 0, maximum_single_cost = 0;
    for(const auto& path : multiple_paths) {
        //std::cout << path << std::endl;
        total_cost += path.size();
        maximum_single_cost = std::max(maximum_single_cost, (int)path.size());
    }
    std::cout << "layered total cost          = " << total_cost << std::endl;
    std::cout << "layered maximum_single_cost = " << maximum_single_cost << std::endl;
    std::cout << std::endl;
    sleep(1);
    memory_recorder.clear();
    base_usage = memory_recorder.getCurrentMemoryUsage();

    gettimeofday(&tv_pre, &tz);
    //auto multiple_paths = my_eecbs::eecbs_demo(argc, argv, map_test_config); // time cost increasing sharply as number of agents increasing
    //auto multiple_paths = second_eecbs::eecbs_demo(argc, argv, map_test_config); // time cost increasing sharply as number of agents increasing

    multiple_paths = CBS_Li::eecbs_MAPF(dim, is_occupied_func, ists, nullptr, agent_num);

    //multiple_paths = LaCAM::lacam_MAPF(dim, is_occupied_func, ists, nullptr, agent_num);

    //multiple_paths = LaCAM2::lacam2_MAPF(dim, is_occupied_func, ists, nullptr, agent_num);

    //multiple_paths = PBS_Li::pbs_MAPF(dim, is_occupied_func, ists, nullptr, agent_num);

    //multiple_paths = CBSH2_RTC::CBSH2_RTC_MAPF(dim, is_occupied_func, ists, nullptr, agent_num);

    //multiple_paths = MAPF_LNS::LNS_MAPF(dim, is_occupied_func, ists, nullptr, agent_num);
    //multiple_paths = MAPF_LNS::AnytimeBCBS_MAPF(dim, is_occupied_func, ists, nullptr, agent_num);
    //multiple_paths = MAPF_LNS::AnytimeEECBS_MAPF(dim, is_occupied_func, ists, nullptr, agent_num);

    //multiple_paths = PIBT_2::pibt_MAPF(dim, is_occupied_func, ists, nullptr, agent_num);
    //multiple_paths = PIBT_2::pibt2_MAPF(dim, is_occupied_func, ists, nullptr, agent_num);
    //multiple_paths = PIBT_2::hca_MAPF(dim, is_occupied_func, ists, nullptr, agent_num);
    //multiple_paths = PIBT_2::push_and_swap_MAPF(dim, is_occupied_func, ists, nullptr, agent_num);

    gettimeofday(&tv_after, &tz);
    //assert(multiple_paths.size() == ists.size());
    visited_grid_during_lacam = LaCAM::visited_grid_;
    //std::cout << " visited_grid_during_lacam size " << visited_grid_during_lacam.size() << std::endl;
    double build_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
    std::cout << multiple_paths.size() << " paths " << " / agents " << ists.size() << std::endl;
    std::cout << "-- raw mapf end in " << build_cost << "ms" << std::endl;
    std::cout << " is solution valid ? " << validateSolution<2>(multiple_paths) << std::endl;

    std::cout << "--variation of memory " << memory_recorder.getCurrentMemoryUsage() - base_usage << " MB" << std::endl;
    sleep(1);
    maximal_usage = memory_recorder.getMaximalMemoryUsage();
    {
        std::cout << "--raw mapf maximal usage = " << maximal_usage - base_usage << " MB" << std::endl;
    }
    total_cost = 0;
    maximum_single_cost = 0;
    for(const auto& path : multiple_paths) {
        //std::cout << path << std::endl;
        total_cost += path.size();
        maximum_single_cost = std::max(maximum_single_cost, (int)path.size());
    }
    std::cout << "total cost          = " << total_cost << std::endl;
    std::cout << "maximum_single_cost = " << maximum_single_cost << std::endl;

    /* initialize my own CBS */
//    freeNav::CBS::Instances<2> ists;
//    ConflictBasedSearchPtr<2> my_cbs = std::make_shared<ConflictBasedSearch<2> >(loader.getDimensionInfo(), is_occupied);
//
//    /* load test data */
//    ScenarioLoader2D sl(map_test_config.at("scene_path").c_str());
//    int max_count_of_case = atoi((map_test_config.at("agent_num")).c_str());
//    int count_of_experiments = sl.GetNumExperiments();
//
//    // load experiment data
//    for (int i = 0; i < std::min(max_count_of_case, count_of_experiments); i++) {
//        const auto &experiment = sl.GetNthExperiment(i);
//        Pointi<2> pt1({experiment.GetStartX(), experiment.GetStartY()});
//        Pointi<2> pt2({experiment.GetGoalX(), experiment.GetGoalY()});
//        freeNav::CBS::Instance<2> ist = {pt1, pt2};
//        //std::cout << " start, target = " << pt1 << ", " << pt2 << std::endl;
//        ists.push_back(ist);
//    }
//// //    ists = {{{61, 40}, {8, 14}}};
//    std::cout << "get " << ists.size() << " instances" << std::endl;
//    /* start my own CBS */
//    gettimeofday(&tv_pre, &tz);
//    auto multiple_paths_mycbs = my_cbs->solve(ists, atoi((map_test_config.at("cut_off_time")).c_str()));
//    std::cout << multiple_paths_mycbs.size() << " agents " << std::endl;
//    gettimeofday(&tv_after, &tz);
//    double cbs_cost = (tv_after.tv_sec - tv_pre.tv_sec)*1e3 + (tv_after.tv_usec - tv_pre.tv_usec)/1e3;
//    std::cout << "-- my CBS end in " << cbs_cost << "ms" << std::endl;
//    total_cost = 0, maximum_single_cost = 0;
//    for(const auto& path : multiple_paths_mycbs) {
//        //std::cout << path << std::endl;
//        total_cost += path.size();
//        maximum_single_cost = std::max(maximum_single_cost, (int)path.size());
//    }
//    std::cout << "total cost          = " << total_cost << std::endl;
//    std::cout << "maximum_single_cost = " << maximum_single_cost << std::endl;

    /* end my CBS */
    ThreadPool tp(1);

    // set viewer
    viewer_3d = new Viewer3D();
    viewer_3d->SetMapfPath(multiple_paths);
    viewer_thread.Schedule([&]{
        viewer_3d->init();

        pangolin::Var<bool> menu_draw_map               = pangolin::Var<bool>("menu.DrawMap",true, true);
        pangolin::Var<bool> menu_vlo                    = pangolin::Var<bool>("menu.DrawVoxelLine",true, true);
        pangolin::Var<bool> menu_occupied_surface       = pangolin::Var<bool>("menu.DrawOctomap",true, true);
        pangolin::Var<bool> menu_draw_path              = pangolin::Var<bool>("menu.DrawPath",true, true);
        pangolin::Var<bool> menu_bound                  = pangolin::Var<bool>("menu.DrawBoundary",false, true);
        pangolin::Var<bool> menu_tangent_candidate_grid = pangolin::Var<bool>("menu.DrawTangentCandidate",false, true);
        pangolin::Var<bool> menu_set_state              = pangolin::Var<bool>("menu.SetStartNow",true, true);
        pangolin::Var<bool> menu_start_plan             = pangolin::Var<bool>("menu.StartPlan",false, false);
        pangolin::Var<bool> menu_reset_view             = pangolin::Var<bool>("menu.AutoResetView",false, true);
        pangolin::Var<bool> menu_reset_top_level        = pangolin::Var<bool>("menu.ResetTopLevel",false, false);
        pangolin::Var<bool> menu_fixed_height           = pangolin::Var<bool>("menu.FixedHeight",true, true);
        pangolin::Var<bool> menu_draw_specific_path     = pangolin::Var<bool>("menu.DrawSpecificPath",true, true);
        pangolin::Var<bool> menu_reset_draw_path_id     = pangolin::Var<bool>("menu.ResetDrawPathId",false, false);
        pangolin::Var<bool> menu_instance               = pangolin::Var<bool>("menu.DrawInstance",false, false);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);
        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        // Define Camera Render Object (for view / scene browsing)
        viewer_3d->mViewpointX = loader.getDimensionInfo()[0]/2;
        viewer_3d->mViewpointY = loader.getDimensionInfo()[1]/2;
        double basic_view_z = 3 * std::max(loader.getDimensionInfo()[0], loader.getDimensionInfo()[1]);
        // set parameters for the window
        auto modeview_matrix = pangolin::ModelViewLookAt(loader.getDimensionInfo()[0]/2,
                                                         loader.getDimensionInfo()[1]/2,
                                                         basic_view_z + viewer_3d->current_top_level,//2*planner_3d->space_3d->max_z,
                                                         loader.getDimensionInfo()[0]/2,
                                                         loader.getDimensionInfo()[1]/2,
                                                         0, 0.0, -1.0, 0.0);
        // set parameters for the window
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(768,768,viewer_3d->mViewpointF,viewer_3d->mViewpointF,384,384,0.1,basic_view_z + 1000),
                modeview_matrix
        );
        MyHandler3D* handle_3d = new MyHandler3D(s_cam);
        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -768.0f/768.0f)
                .SetHandler(handle_3d);
        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        while( !pangolin::ShouldQuit() )
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            modeview_matrix = pangolin::ModelViewLookAt(loader.getDimensionInfo()[0]/2,
                                                             loader.getDimensionInfo()[1]/2,
                                                        basic_view_z + (menu_fixed_height ? 0 : viewer_3d->current_top_level),//2*planner_3d->space_3d->max_z,
                                                             loader.getDimensionInfo()[0]/2,
                                                             loader.getDimensionInfo()[1]/2,
                                                        0, 0.0, -1.0, 0.0);
            if(menu_reset_view) {
                s_cam.SetModelViewMatrix(modeview_matrix);
                //menu_reset_view = false;
            }
            d_cam.Activate(s_cam);
            glClearColor(1.0f,1.0f,1.0f,1.0f);
            if(menu_vlo) viewer_3d->show_line_for_voxel = true;
            else viewer_3d->show_line_for_voxel = false;
            if(menu_draw_map) {
                if(menu_fixed_height) {
                    viewer_3d->DrawGridMapAtTimeIndex(loader.getDimensionInfo(), is_occupied_func, 0);
                } else {
                    viewer_3d->DrawGridMapAtTimeIndex(loader.getDimensionInfo(), is_occupied_func, viewer_3d->current_top_level);
                }
            }
            if(menu_instance) {
                for(int i=0; i<ists.size(); i++) {
                    auto color = COLOR_TABLE[i % 30];
                    double r = ((int) color[0]) / 255., g = ((int) color[1]) / 255., b = ((int) color[2]) / 255.;
                    viewer_3d->DrawVoxel(ists[i].first[0], ists[i].first[1], 0, 0.5, r, g, b, true);
                    viewer_3d->DrawVoxel(ists[i].second[0], ists[i].second[0], 0, 0.5, r, g, b, true);
                }
            }
            if(menu_reset_top_level) {
                viewer_3d->current_top_level = 0;
                menu_reset_top_level = false;
            }
            if(menu_reset_draw_path_id) {
                viewer_3d->current_path_id = 0;
                menu_reset_draw_path_id = false;
            }
            if(menu_draw_path) {
                viewer_3d->DrawMAPFPathAtTimeIndex(viewer_3d->current_top_level,
                                                   menu_draw_specific_path,
                                                   viewer_3d->current_path_id,
                                                   menu_fixed_height);
            }
            if(menu_start_plan) {
                menu_start_plan = false;
                viewer_3d->under_planning = true;
                tp.Schedule([&]{

                    // line of sight check comparison

                    std::cout << "-- start planning " << std::endl;
                    std::cout << "start " << viewer_3d->start_ << " target" << viewer_3d->target_ << std::endl;
                    plan_finish = false;
                    std::cout << "-- finish planning" << std::endl;
                    plan_finish = true;
                });
            }
            if(menu_set_state) viewer_3d->viewer_set_start = true;
            else viewer_3d->viewer_set_start = false;
            if(menu_bound) {
                viewer_3d->DrawBound(0, loader.getDimensionInfo()[0],
                                     0, loader.getDimensionInfo()[1],
                                     0, basic_view_z + viewer_3d->current_top_level);
            }
            pangolin::FinishFrame();
        }

    });
    return 0;
}

