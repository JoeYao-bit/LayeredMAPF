//
// Created by yaozhuo on 2023/6/18.
//

//
// Created by yaozhuo on 2023/6/7.
//

#include "3d_viewer/3d_viewer.h"
#include "dependencies/test_data.h"
#include "multi-agent-path-finding/general_mapf_scene.h"
#include "dependencies/color_table.h"
#include <2d_grid/text_map_loader.h>
#include "EECBS/inc/Instance.h"
#include "dependencies/test_data.h"

Viewer3D* viewer_3d;
ThreadPool viewer_thread(1);

// MapTestConfig_Full4 // run out of space, shrink space 4
// MapTestConfig_Simple // success
// MapTestConfig_DA2
// MapTestConfig_DC1 // one bywave, need 8~14s
// MapTestConfig_Complex
// MapTestConfig_A1
// MapTestConfig_FA2
// MapTestConfig_A5
auto map_test_config = freeNav::RimJump::MAPFTestConfig_random_32_32_20;

// generate random grid maps and agents
CBS_Li::Instance instance(map_test_config.at("map_path"),
                  map_test_config.at("scene_path"),
                  atoi((map_test_config.at("agent_num")).c_str()),
                  5, 5, 2, 0
);

using namespace freeNav::RimJump;
using namespace freeNav;

struct timezone tz;
struct timeval tv_pre;
struct timeval tv_after;

std::shared_ptr<RoadMapGraphBuilder<3> > tgb = nullptr;

bool plan_finish = false;

Pointi<3> pt1, pt2;
GridPtr<3> sg1 = std::make_shared<Grid<3>>(), sg2 = std::make_shared<Grid<3>>();



auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') return false;
    return true;
};


// load mapf scene
TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);

auto is_occupied = [](const freeNav::Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto set_occupied = [](const freeNav::Pointi<2> & pt) { loader.setOccupied(pt); };

IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

SET_OCCUPIED_FUNC<2> set_occupied_func = set_occupied;

//int current_time_index = 0;

int main(int argc, char** argv) {
    ThreadPool tp(1);

    // set viewer
    viewer_3d = new Viewer3D();
    viewer_thread.Schedule([&]{
        viewer_3d->init();

        pangolin::Var<bool> menu_draw_map               = pangolin::Var<bool>("menu.DrawMap",true, true);
        pangolin::Var<bool> menu_vlo                    = pangolin::Var<bool>("menu.DrawVoxelLine",true, true);
        pangolin::Var<bool> menu_occupied_surface       = pangolin::Var<bool>("menu.DrawOctomap",true, true);
        pangolin::Var<bool> menu_draw_path              = pangolin::Var<bool>("menu.DrawPath",true, true);
        pangolin::Var<bool> menu_bound                  = pangolin::Var<bool>("menu.DrawBoundary",false, true);
        pangolin::Var<bool> menu_tangent_candidate_grid = pangolin::Var<bool>("menu.DrawTangentCandidate",false, true);
        pangolin::Var<bool> menu_set_state              = pangolin::Var<bool>("menu.SetStartNow",true, true);
        pangolin::Var<bool> menu_reset_view             = pangolin::Var<bool>("menu.AutoResetView",false, true);

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
                                                         basic_view_z,//2*planner_3d->space_3d->max_z,
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
                                                        basic_view_z,//2*planner_3d->space_3d->max_z,
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
                viewer_3d->DrawGridMapAtTimeIndex(loader.getDimensionInfo(), is_occupied_func, 0);
            }
            if(menu_set_state) viewer_3d->viewer_set_start = true;
            else viewer_3d->viewer_set_start = false;
            pangolin::FinishFrame();
        }

    });
    return 0;
}

