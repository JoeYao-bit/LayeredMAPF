#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QMainWindow>
#include "view.h"
#include "../algorithm/layered_mapf.h"

QT_BEGIN_NAMESPACE
class QGraphicsScene;
class QSplitter;
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(freeNav::IS_OCCUPIED_FUNC<2> is_occupied,
               freeNav::DimensionLength* dim,
               const freeNav::Instances<2>& instances,
               const std::set<int>& ids_to_draw,
               QWidget *parent = 0);

    View* view;

private:

    void populateScene();

    // call after addBinaryTreeNodes
    //void addBinaryTreeEdges(TreeNode* node_item);

    void addGridMap();

    void addAgentRelations();

    void addAgentText();

    // update value boundary for customized visualization
    //void traversalTree(TreeNode* root_node);

    // call after traversalTree
    //void setTreeNodes();

    // called after travelsalTree
//    void getAllDepthNodes();

    // variable for visualization
    int max_g_val_,            min_g_val_; // for successful CBS search, min value is 0, but for failed CBS, it may not equal to 0
    int max_h_val_,            min_h_val_;
    int max_conflict_size_,    min_conflict_size_;
    int max_distance_to_goal_, min_distance_to_goal_;
    int max_cost_to_goal_,     min_cost_to_goal_;
    int max_depth_; // max depth of tree, and min_depth is 0, no matter success or failed

    QGraphicsScene *scene, *scene_second;


};

#endif // MAINWINDOW_H
