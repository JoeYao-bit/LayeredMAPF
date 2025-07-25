//
// Created by yaozhuo on 2024/5/5.
//

#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include "../../algorithm/LA-MAPF/circle_shaped_agent.h"
#include "../../algorithm/LA-MAPF/block_shaped_agent.h"

#include "../../algorithm/LA-MAPF/CBS/large_agent_CBS.h"
#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"
#include "../../algorithm/LA-MAPF/CBS/constraint_avoidance_table.h"

#include "../../freeNav-base/visualization/canvas/canvas.h"
#include "../../freeNav-base/dependencies/2d_grid/text_map_loader.h"
#include "../test_data.h"
#include "common_interfaces.h"
#include "../../algorithm/LA-MAPF/CBS/layered_large_agent_CBS.h"

#include "../../algorithm/precomputation_for_decomposition.h"


int canvas_size_x = 1000, canvas_size_y = 700;


TEST(test, pointRotate_2D) {
    canvas_size_x = 10, canvas_size_y = 10;

    Canvas canvas("BlockRotateCoverage", canvas_size_x, canvas_size_y, 20, 10);

    Pointi<2> pt{4, 3}, pt1, pt2, pt3;
    pt1 = pointRotate_2D(pt, 1);
    pt2 = pointRotate_2D(pt, 2);
    pt3 = pointRotate_2D(pt, 3);

    std::cout << "after rotate 1 " << pt1 << std::endl;
    std::cout << "after rotate 2 " << pt2 << std::endl;
    std::cout << "after rotate 3 " << pt3 << std::endl;

    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        canvas.drawGrid(5, 5);

        canvas.drawGridLine(canvas_size_x/2, canvas_size_y/2, pt[0]  + canvas_size_x/2, pt[1]  + canvas_size_y/2);
        canvas.drawGridLine(canvas_size_x/2, canvas_size_y/2, pt1[0] + canvas_size_x/2, pt1[1] + canvas_size_y/2);
        canvas.drawGridLine(canvas_size_x/2, canvas_size_y/2, pt2[0] + canvas_size_x/2, pt2[1] + canvas_size_y/2);
        canvas.drawGridLine(canvas_size_x/2, canvas_size_y/2, pt3[0] + canvas_size_x/2, pt3[1] + canvas_size_y/2);

        canvas.drawGrid(pt[0]  + canvas_size_x/2,  pt[1] + canvas_size_y/2,  COLOR_TABLE[0]);
        canvas.drawGrid(pt1[0] + canvas_size_x/2, pt1[1] + canvas_size_y/2, COLOR_TABLE[1]);
        canvas.drawGrid(pt2[0] + canvas_size_x/2, pt2[1] + canvas_size_y/2, COLOR_TABLE[2]);
        canvas.drawGrid(pt3[0] + canvas_size_x/2, pt3[1] + canvas_size_y/2, COLOR_TABLE[3]);


        char key = canvas.show();

    }
}

TEST(test, BlockRotateCoverage) {
    auto file_path = MAPFTestConfig_empty_48_48;

    TextMapLoader tl(file_path.at("map_path"), is_char_occupied1);
    std::cout << "start SingleMapTest from map " << file_path.at("map_path") << std::endl;
    auto dim = tl.getDimensionInfo();
    auto is_occupied = [&tl](const freeNav::Pointi<2> &pt) -> bool { return tl.isOccupied(pt); };
    IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

    BlockAgent_2D block({-15, -10}, {20, 10}, 0, dim);
    int orient_start = 1, orient_end = 3;
    const auto& rotate_f_b = block.getRotateCoverage(orient_start, orient_end);
    bool draw_front_coverage = true, draw_backward_coverage = true, draw_rotate = true;
    int orient = 0;
    canvas_size_x = 100, canvas_size_y = 70;
    Canvas canvas("BlockRotateCoverage", canvas_size_x, canvas_size_y, 20, 1);

    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();
        //canvas.drawArrowInt(0, (int)dim[1]/2, (int)dim[0], (int)dim[1]/2, 2, true, COLOR_TABLE[1]);
//        canvas.drawArrow(-2.4, 0, 0, 4.8, 2);
//        canvas.drawArrow(0, -2.4, M_PI/2, 4.8, 2);

        canvas.drawAxis(canvas_size_x/2, canvas_size_y/2, .2);
        // draw block
//        canvas.drawLine(block.min_pt_[0], block.min_pt_[1], block.max_pt_[0], block.min_pt_[1]);
//        canvas.drawLine(block.min_pt_[0], block.max_pt_[1], block.max_pt_[0], block.max_pt_[1]);
//
//        canvas.drawLine(block.min_pt_[0], block.min_pt_[1], block.min_pt_[0], block.max_pt_[1]);
//        canvas.drawLine(block.max_pt_[0], block.min_pt_[1], block.max_pt_[0], block.max_pt_[1]);

        // draw coverage grids
        if(draw_rotate) {
//            for (const auto &pt : block.grids_) {
//                canvas.drawGrid(pt[0] + canvas_size_x/2, pt[1] + canvas_size_y/2);
//            }
            for (const auto &pt : block.grids_[orient_start].first) {
                canvas.drawGrid(pt[0] + canvas_size_x/2, pt[1] + canvas_size_y/2, COLOR_TABLE[orient_start+4]);
            }
            for (const auto &pt : block.grids_[orient_start].second) {
                canvas.drawGrid(pt[0] + canvas_size_x/2, pt[1] + canvas_size_y/2, (COLOR_TABLE[orient_start+4] + cv::Vec3b::all(255))/2);
            }
            for (const auto &pt : block.grids_[orient_end].first) {
                canvas.drawGrid(pt[0] + canvas_size_x/2, pt[1] + canvas_size_y/2, COLOR_TABLE[orient_end+4]);
            }
            for (const auto &pt : block.grids_[orient_end].second) {
                canvas.drawGrid(pt[0] + canvas_size_x/2, pt[1] + canvas_size_y/2, (COLOR_TABLE[orient_start+4] + cv::Vec3b::all(255))/2);
            }
        }
        const auto& draw_which = rotate_f_b; // block.front_rotate_pts, block.backward_rotate_pts
        // draw rotate grids
        if(draw_front_coverage) {
            for (const auto &pt : draw_which.first) {
                canvas.drawGrid(pt[0] + canvas_size_x/2, pt[1] + canvas_size_y/2, COLOR_TABLE[1]);
            }
        }
        if(draw_backward_coverage) {
            for (const auto &pt : rotate_f_b.second) {
                canvas.drawGrid(pt[0] + canvas_size_x/2, pt[1] + canvas_size_y/2, COLOR_TABLE[2]);
            }
        }
        char key = canvas.show();
        switch (key) {
            case 'w':
                orient += 1;
                orient = orient % 4;
                std::cout << "-- change orient to " << orient << std::endl;
                break;
            case 's':
                orient += 3;
                orient = orient % 4;
                std::cout << "-- change orient to " << orient << std::endl;
                break;
            case 'f':
                draw_front_coverage    = !draw_front_coverage;
                break;
            case 'b':
                draw_backward_coverage = !draw_backward_coverage;
                break;
            case 'r':
                draw_rotate = !draw_rotate;
                break;
            default:
                break;
        }
    }
}

TEST(RectangleOverlap, test) {
    /*
     * rect (3.6, 12.6), (4.4, 13.4)
       rect (4.5, 21.6), (5.6, 22.4)
       rect (1.8, 12.7), (4.2, 14)
     * */
    Pointf<2> p1{3.6, 12.6}, p2{4.4, 13.4}, p3{1.8, 12.7}, p4{4.2, 14};
    std::cout << isRectangleOverlap<float, 2>(p1, p2, p3, p4) << std::endl;
}

TEST(pointer, test) {
    DimensionLength dim[2];
    dim[0] = 1, dim[1] = 3;
    DimensionLength* dim1 = dim;
    std::cout << "dim  " << dim << std::endl;

    std::cout << "dim1 " << dim1 << std::endl;
    std::cout << "dim1 " << dim1[0] << ", " << dim1[1] << std::endl;

}


TEST(constraint_avoidance_table, test) {
    auto file_path = MAPFTestConfig_empty_48_48;

    TextMapLoader tl(file_path.at("map_path"), is_char_occupied1);
    std::cout << "start SingleMapTest from map " << file_path.at("map_path") << std::endl;
    auto dim = tl.getDimensionInfo();
    auto is_occupied = [&tl](const freeNav::Pointi<2> &pt) -> bool { return tl.isOccupied(pt); };
    IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;


    // fake instances
    InstanceOrients<2> instances = {
            {{{5, 3}, 0}, {{23, 22},0} },
            {{{9, 2}, 0}, {{5, 22}, 0}},
            {{{2, 5}, 0}, {{17, 22}, 3}}
    };
    AgentPtrs<2> agents({
                                   std::make_shared<CircleAgent<2> >(.3, 0, dim),
                                   std::make_shared<CircleAgent<2> >(.7, 1, dim),
                                   std::make_shared<CircleAgent<2> >(.6, 2, dim)
                           });

    InstanceDeserializer<2> deserializer_local;
    const std::string file_path_local = file_path.at("la_ins_path");
    if(deserializer_local.loadInstanceFromFile(file_path_local, dim)) {
        std::cout << "load from path " << file_path_local << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path_local << " failed" << std::endl;
        return;
    }

    PrecomputationOfLAMAPFDecomposition<2, HyperGraphNodeDataRaw<2>> pre(instances,
                                                                         agents,
                                                                         dim, is_occupied, true);
    std::vector<std::vector<int> > grid_visit_count_table;

    CBS::LargeAgentCBS<2, Pose<int, 2>> lacbs(instances,
                                              agents,
                                              dim, is_occupied,
                                              nullptr,
                                              pre.instance_node_ids_,
                                              pre.all_poses_,
                                              pre.distance_map_updater_,
                                              pre.agent_sub_graphs_,
                                              pre.agents_heuristic_tables_,
                                              pre.agents_heuristic_tables_ignore_rotate_,
                                              nullptr);

    if(!lacbs.solve(30)) {
        std::cout << "failed to solve" << std::endl;
    }
    std::cout << "validation of solution " << lacbs.solutionValidation() << std::endl;
    int agent_id = 2;

    ConstraintAvoidanceTable<2, Pose<int, 2>> table(dim, lacbs.all_poses_, agents[0]);

    table.insertAgentPathOccGrids(agents[0], lacbs.getSolution()[0]);
    table.insertAgentPathOccGrids(agents[1], lacbs.getSolution()[1]);
    table.insertAgentPathOccGrids(agents[2], lacbs.getSolution()[2]);

    table.printOccTable();

    LAMAPF_Path path = lacbs.getSolution()[agent_id];
    for(int t=0; t<path.size()-1; t++) {
        const auto& curr_node_id = path[t];
        const auto& next_node_id = path[t+1];
//        std::cout << " curr_node_id " << curr_node_id << ", next_node_id " << next_node_id << std::endl;
//        std::cout << " curr_node_id node " << *lacbs.all_poses_[curr_node_id]
//                  << ", next_node_id node " << *lacbs.all_poses_[next_node_id] << std::endl;

        Pointis<2> occ_grids = agents[agent_id]->getTransferOccupiedGrid(*lacbs.all_poses_[curr_node_id],
                                                                        *lacbs.all_poses_[next_node_id]);

        int num_of_conf = table.getNumOfConflictsForStep(occ_grids, 3, t);
        std::cout << "t = " << t << " have " << num_of_conf << " conflicts " << std::endl;
    }


}

int main() {
    return 0;
}