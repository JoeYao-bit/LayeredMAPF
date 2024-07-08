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



CircleAgent<2> c1(2.5, 0, dim);
CircleAgent<2> c2(3.5, 1, dim);

int canvas_size_x = 1000, canvas_size_y = 700;

TEST(circleAgentIsCollide, cbs_test) {
    Pose<int, 2> p1({1,1},0), p2({1,2}, 0), p3({1,3}, 0), p4({4,1}, 0);
    std::cout << "is collide" << isCollide(c1, p1, p2, c2, p3, p4) << std::endl;
}

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

TEST(CircleAgentSubGraph, cbs_test) {

    // fake instances
    InstanceOrients<2> instances = {
            {{{8, 3}, 0}, {{23, 22},0} },
            {{{9, 2}, 0}, {{5, 22}, 0}},
            {{{2, 5}, 0}, {{17, 22}, 3}}
    };
    CircleAgents<2> agents({
                            CircleAgent<2>(.3, 0, dim),
                            CircleAgent<2>(.7, 1, dim),
                            CircleAgent<2>(.6, 2, dim)
                           });
//    std::cout << "dim = " << dim << std::endl;
//    for(const auto& agent : agents) {
//        std::cout << "agent " << agent.id_ << ", dim = " << agent.dim_ << std::endl;
//        std::cout << "agent " << agent.id_ << ", dim = " << agent.dim_[0] << ", " << agent.dim_[1] << std::endl;
//    }

    startLargeAgentMAPFTest<CircleAgent<2>, CBS::LargeAgentCBS<2, CircleAgent<2>> >(agents, instances);
}

TEST(BlockAgentSubGraph, cbs_test) {
    // fake instances
    InstanceOrients<2> instances = {
            {{{5, 3}, 0}, {{23, 22},0} },
            {{{9, 2}, 0}, {{5, 22}, 0}},
            {{{2, 5}, 0}, {{17, 22}, 3}}
    };
    Pointf<2> min_pt_0{-.4, -.4},  max_pt_0{.4, .4},
            min_pt_1{-.6, -.4},  max_pt_1{1., .4},
            min_pt_2{-.3, -1.2}, max_pt_2{1., 1.2};
    // NOTICE: initialize pt in constructor cause constant changed
    BlockAgents_2D agents({
                                        BlockAgent_2D({-.4, -.4}, {.4, .4}, 0, dim),
                                        BlockAgent_2D({-.6, -.4}, {1.3, .4}, 1, dim),
                                        BlockAgent_2D({-.3, -.8},{1., .8}, 2, dim)
                                });

    startLargeAgentMAPFTest<BlockAgent_2D, CBS::LargeAgentCBS<2, BlockAgent_2D> >(agents, instances);
}

TEST(constraint_avoidance_table, test) {
    // fake instances
    InstanceOrients<2> instances = {
            {{{5, 3}, 0}, {{23, 22},0} },
            {{{9, 2}, 0}, {{5, 22}, 0}},
            {{{2, 5}, 0}, {{17, 22}, 3}}
    };
    CircleAgents<2> agents({
                                   CircleAgent<2>(.3, 0, dim),
                                   CircleAgent<2>(.7, 1, dim),
                                   CircleAgent<2>(.6, 2, dim)
                           });

    CBS::LargeAgentCBS<2, CircleAgent<2> > lacbs(instances, agents, dim, is_occupied);

    if(!lacbs.solve(30)) {
        std::cout << "failed to solve" << std::endl;
    }
    std::cout << "validation of solution " << lacbs.solutionValidation() << std::endl;
    ConstraintAvoidanceTable<2, CircleAgent<2> > table(dim, lacbs.all_poses_);

    table.insertAgentPathOccGrids(agents[0], lacbs.getSolution()[0]);
    table.insertAgentPathOccGrids(agents[1], lacbs.getSolution()[1]);
    table.insertAgentPathOccGrids(agents[2], lacbs.getSolution()[2]);

    table.printOccTable();

    int agent_id = 2;
    LAMAPF_Path path = lacbs.getSolution()[agent_id];
    for(int t=0; t<path.size()-1; t++) {
        const auto& curr_node_id = path[t];
        const auto& next_node_id = path[t+1];
//        std::cout << " curr_node_id " << curr_node_id << ", next_node_id " << next_node_id << std::endl;
//        std::cout << " curr_node_id node " << *lacbs.all_poses_[curr_node_id]
//                  << ", next_node_id node " << *lacbs.all_poses_[next_node_id] << std::endl;

        Pointis<2> occ_grids = agents[agent_id].getTransferOccupiedGrid(*lacbs.all_poses_[curr_node_id],
                                                                        *lacbs.all_poses_[next_node_id]);

        int num_of_conf = table.getNumOfConflictsForStep(occ_grids, 3, t);
        std::cout << "t = " << t << " have " << num_of_conf << " conflicts " << std::endl;
    }


}