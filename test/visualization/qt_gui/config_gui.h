//
// Created by yaozhuo on 2023/10/7.
//

#ifndef FREENAV_COMMOM_QTGUI_H
#define FREENAV_COMMOM_QTGUI_H

// common data for CBS GUI
#include <vector>
#include <QColor>

const int tree_node_rec_size_x = 70;
const int tree_node_rec_size_y = 50;

const int tree_node_horizon_interval = 10;
const int tree_node_vertical_interval = 40;

// node's height should be higher than edge
const int tree_node_z = 5;
const int tree_edge_z = 0;

const int node_boundary_pen_width = 4;
const int edge_pen_width = 5;

const int GRID_WIDTH = 50;

const int GRID_INTERVAL = 1;

const int GRID_FONT_SIZE = 12;

typedef enum {
    G_VAL = 0,
    H_VAL,
    DISTANCE_TO_GO,
    CONFLICTS_SIZE,
    COST_TO_GOAL,
    DEPTH
} ValType;

extern std::vector<std::vector<int> > ColorType;



#endif //FREENAV_COMMOM_H
