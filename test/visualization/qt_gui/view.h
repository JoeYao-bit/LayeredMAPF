#ifndef VIEW_H
#define VIEW_H

#include <QFrame>
#include <QGraphicsView>
#include <QKeyEvent>
#include <QLineEdit>
#include <QGridLayout>

#include "grid_element.h"
#include "../algorithm/instance_decomposition.h"
#include "../freeNav-base/dependencies/color_table.h"
QT_BEGIN_NAMESPACE
class QLabel;
class QSlider;
class QToolButton;
QT_END_NAMESPACE

class View;

class GraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    GraphicsView(View *v) : QGraphicsView(), view(v) { }

protected:
#if QT_CONFIG(wheelevent)
    void wheelEvent(QWheelEvent *) override;
#endif

private:
    View *view;
};

class View : public QFrame
{
    Q_OBJECT
public:
    explicit View(const QString &name, QWidget *parent = 0);

    QGraphicsView *view() const;

    QGraphicsView *second_view() const;

    void setAllNodesColor(int r, int g, int b);

    void initGridMap();

    void initAgentRelations();

    void initAgentNameText();

    void resetGridMapToEmpty();

    void drawInstancesOfMAPFColor();
    void drawInstancesOfMAPFText();
    void drawFreeGridGroupColor();
    void drawFreeGridGroupText();
    void drawNearbyHyperNodes();
    void drawHeuristicTable();
    void drawAllCluster();

    int current_shown_agent_ = 0;

    freeNav::IS_OCCUPIED_FUNC<2> is_occupied_;

    freeNav::DimensionLength* dim_;

    freeNav::Instances<2> instance_;

    std::vector<int> index_of_key_nodes_; // index of nodes that in the final path that in each level

    std::vector<Grid*> all_grid_;

    // draw the dependency relation between agents
    std::vector<Grid*> all_agent_relations_;

    freeNav::LayeredMAPF::MAPFInstanceDecompositionPtr<2> layered_mapf;

    CircleOfGrid *agent_start_, *agent_target_;

    std::vector<QGraphicsTextItem*> agent_name_row_;

    std::vector<QGraphicsTextItem*> agent_name_column_;

    std::set<int> ids_to_draw_; // a set of point to draw, only for debug

protected:
    virtual void keyPressEvent(QKeyEvent *ev);
    virtual void keyReleaseEvent(QKeyEvent *ev);

public slots:
    void zoomIn(int level = 1);
    void zoomOut(int level = 1);
    void zoomInSecond(int level = 1);
    void zoomOutSecond(int level = 1);

private slots:
    void resetView();
    void resetViewSecond();

    void setResetButtonEnabled();
    void setResetButtonEnabledSecond();

    void setupMatrix();
    void setupMatrixSecond();

    void togglePointerMode();
    void togglePointerModeSecond();

    void toggleOpenGL();
    void toggleAntialiasing(); // 是否抗锯齿

    void toggleOpenGLSecond();
    void toggleAntialiasingSecond(); // 是否抗锯齿

    void print();

    void setColorToGreen();
    void setColorToRed();
    void setDrawNodeInPathSpecific(); // whether draw hyper node that form final path specificly

    void setHyperGraphGridMap();
    void setEmptyGridMap();
    void setInstanceGridMap();
    void setHeuristicTable();
    void setAllCluster();

    void agentNumEditFinished();

    void toggleDrawCurrentInstance(); // 是否抗锯齿

    void updateWindowRatio();


private:
    GraphicsView *graphicsView, *graphicsView_second;
    QLabel *label;

    QToolButton *selectModeButton, *selectModeButton_second;
    QToolButton *dragModeButton, *dragModeButton_second;
    QToolButton *openGlButton, *openGlButtonSecond;
    QToolButton *antialiasButton, *antialiasButtonSecond;
    QToolButton *printButton;
    QToolButton *resetButton, *resetButton_second;
    QToolButton *drawFinalPathSpecificButton;
    QToolButton *drawCurrentInstance_second;

    QLabel* current_agent_text;

    QSlider *zoomSlider, *zoomSlider_second;
    QSlider *windowRatioSlider;

    QGridLayout *topLayout;// Layout that contain two windows

    QMenu *colorValueMenu; // draw tree node in different color for different value type

    QMenu *gridMapValueMenu; // draw tree node in different color for different value type

    QLineEdit* agent_num_edit;// get current agent num

};

#endif // VIEW_H
