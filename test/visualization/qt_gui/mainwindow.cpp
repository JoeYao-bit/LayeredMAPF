#include "mainwindow.h"
#include "view.h"

#include <QHBoxLayout>
#include <QSplitter>
#include "config_gui.h"
#include <sstream>
#include "../algorithm/instance_decomposition.h"

using namespace freeNav;

MainWindow::MainWindow(freeNav::IS_OCCUPIED_FUNC<2> is_occupied,
                       freeNav::DimensionLength* dim,
                       const freeNav::Instances<2>& instances,
                       const std::set<int>& ids_to_draw,
                       QWidget *parent)
    : QMainWindow(parent)
{
    view = new View("Constraint Tree & Grid Map");
    view->is_occupied_ = is_occupied;
    view->dim_ = dim;
    view->instance_ = instances;
    view->ids_to_draw_ = ids_to_draw;
    view->layered_mapf = std::make_shared<freeNav::LayeredMAPF::MAPFInstanceDecomposition<2> >(instances, dim, is_occupied, 2);
    view->initGridMap();
    view->initAgentRelations();
    view->initAgentNameText();
    populateScene();

    view->view()->setScene(scene); // only content add before set will be visualized
    view->second_view()->setScene(scene_second);

    this->setCentralWidget(view);
    setWindowTitle(tr("Conflict Based Search GUI"));
}

void MainWindow::populateScene()
{
    scene = new QGraphicsScene(this);
    QColor color = QColor::fromRgb(ColorType[ValType::G_VAL][0],
                                   ColorType[ValType::G_VAL][1],
                                   ColorType[ValType::G_VAL][2]);
    addAgentRelations();
    addAgentText();

    // add item about the second scene
    scene_second = new QGraphicsScene(this);
    addGridMap();

}

void MainWindow::addAgentRelations() {
    for(const auto& agent : view->all_agent_relations_) {
        scene->addItem(agent);
    }
}

void MainWindow::addGridMap() {
    // draw grid map
    for(const auto& grid : view->all_grid_) {
        scene_second->addItem(grid);
    }
    scene_second->addItem(view->agent_start_);
    scene_second->addItem(view->agent_target_);
}

//void MainWindow::traversalTree(TreeNode* node) {
//    if(node == nullptr) return;
//    if(node->pa_ == nullptr) {
//        // initialize
//        max_g_val_            = 0, min_g_val_            = freeNav::MAX<int>;
//        max_h_val_            = 0, min_h_val_            = freeNav::MAX<int>;
//        max_conflict_size_    = 0, min_conflict_size_    = freeNav::MAX<int>;
//        max_distance_to_goal_ = 0, min_distance_to_goal_ = freeNav::MAX<int>;
//        max_cost_to_goal_     = 0, min_cost_to_goal_     = freeNav::MAX<int>;
//        max_depth_            = 0;
//
//        view->index_of_key_nodes_.clear();
//        view->all_level_nodes_.clear();
//    }
//
//    max_depth_ = max(max_depth_, node->node_->depth_);
//
//    max_g_val_ = max(max_g_val_, node->node_->g_val_);
//    min_g_val_ = min(max_g_val_, node->node_->g_val_);
//
//    max_h_val_ = max(max_h_val_, node->node_->h_val_);
//    min_h_val_ = min(max_h_val_, node->node_->h_val_);
//
//    max_conflict_size_ = max(max_conflict_size_, (int)node->node_->conflict_size_);
//    min_conflict_size_ = min(min_conflict_size_, (int)node->node_->conflict_size_);
//
//    max_distance_to_goal_ = max(max_distance_to_goal_, node->node_->distance_to_go_);
//    min_distance_to_goal_ = min(min_distance_to_goal_, node->node_->distance_to_go_);
//
//    max_cost_to_goal_ = max(max_cost_to_goal_, node->node_->cost_to_go_);
//    min_cost_to_goal_ = min(min_cost_to_goal_, node->node_->cost_to_go_);
//
//    if(node->node_->depth_ >= view->all_level_nodes_.size()) {
//        view->all_level_nodes_.push_back({});
//        assert(node->node_->depth_ + 1 == view->all_level_nodes_.size());
//    }
//
//    if(node->node_->in_final_path_) {
//        view->index_of_key_nodes_.push_back(view->all_level_nodes_[node->node_->depth_].size());
//    }
//
//    view->all_level_nodes_[node->node_->depth_].push_back(node);
//
//    if(node->node_->ch_.empty()) return;
//    //not set x, only set y
//    TreeNode *left_child = new TreeNode(node->color_, 0, node->node_->depth_*tree_node_vertical_interval,
//                                        tree_node_z, node->node_->ch_[0], node);
//    node->ch_.push_back(left_child);
//    traversalTree(node->ch_[0]);
//    if(node->node_->ch_.size() == 2) {
//        TreeNode *right_child = new TreeNode(node->color_, 0, node->node_->depth_*tree_node_vertical_interval,
//                                            tree_node_z, node->node_->ch_[1], node);
//        node->ch_.push_back(right_child);
//        traversalTree(node->ch_[1]);
//    }
//}

//void MainWindow::addBinaryTreeEdges(TreeNode* node_item) {
//    if(node_item == nullptr) return;
//    if(node_item->ch_.empty()) return;
//    QColor color = QColor::fromRgb(200, 200, 200);
//    TreeEdge *edge_item = new TreeEdge(color, node_item, node_item->ch_[0]);
//    scene->addItem(edge_item);
//    view->all_edges_.push_back(edge_item);
//    addBinaryTreeEdges(node_item->ch_[0]);
//    if(node_item->ch_.size() == 2) {
//        TreeEdge *edge_item = new TreeEdge(color, node_item, node_item->ch_[1]);
//        scene->addItem(edge_item);
//        view->all_edges_.push_back(edge_item);
//        addBinaryTreeEdges(node_item->ch_[1]);
//    }
//}

//void MainWindow::setTreeNodes() {
//    for(int level=0; level<view->all_level_nodes_.size(); level++) {
//        for(int i=0; i<view->all_level_nodes_[level].size(); i++) {
//            int x;
//            // considering the final path may not in the deepest tree
//            if(!view->index_of_key_nodes_.empty() && (view->index_of_key_nodes_.size() > level)) {
//                x = (i - view->index_of_key_nodes_[level]) * (tree_node_horizon_interval + tree_node_rec_size_x);
//            } else {
//                x = (i - view->all_level_nodes_[level].size()/2)* (tree_node_horizon_interval + tree_node_rec_size_x);
//            }
//            int y = level*(tree_node_vertical_interval + tree_node_rec_size_y);
//            view->all_level_nodes_[level][i]->x_ = x;
//            view->all_level_nodes_[level][i]->y_ = y;
//            view->all_level_nodes_[level][i]->setPos(x, y);
//            scene->addItem(view->all_level_nodes_[level][i]);
//        }
//    }
//}


void MainWindow::addAgentText() {
    for(const auto& text : view->agent_name_row_) {
        scene->addItem(text);
    }
    for(const auto& text : view->agent_name_column_) {
        scene->addItem(text);
    }
}


























