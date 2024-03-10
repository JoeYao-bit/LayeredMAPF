#include "view.h"
//#define QT_NO_OPENGL

#if defined(QT_PRINTSUPPORT_LIB)
#include <QtPrintSupport/qtprintsupportglobal.h>
#if QT_CONFIG(printdialog)
#include <QPrinter>
#include <QPrintDialog>
#endif
#endif
#ifndef QT_NO_OPENGL
#include <QtOpenGL>
#else
#include <QtWidgets>
#endif
#include <qmath.h>

#if QT_CONFIG(wheelevent)
void GraphicsView::wheelEvent(QWheelEvent *e)
{
    if (e->modifiers() & Qt::ControlModifier) {
        if (e->delta() > 0) {
            view->zoomIn(6);
        } else {
            view->zoomOut(6);
        }
        e->accept();
    } else {
        QGraphicsView::wheelEvent(e);
    }
}
#endif

View::View(const QString &name, QWidget *parent)
    : QFrame(parent)
{
    setFrameStyle(Sunken | StyledPanel);
    graphicsView = new GraphicsView(this);
    graphicsView->setRenderHint(QPainter::Antialiasing, false);
    graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
    graphicsView->setOptimizationFlags(QGraphicsView::DontSavePainterState);
    graphicsView->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    graphicsView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

    graphicsView_second = new GraphicsView(this);
    graphicsView_second->setRenderHint(QPainter::Antialiasing, false);
    graphicsView_second->setDragMode(QGraphicsView::RubberBandDrag);
    graphicsView_second->setOptimizationFlags(QGraphicsView::DontSavePainterState);
    graphicsView_second->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    graphicsView_second->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);


    int size = style()->pixelMetric(QStyle::PM_ToolBarIconSize);
    QSize iconSize(size, size);

    QToolButton *zoomInIcon = new QToolButton;
    zoomInIcon->setAutoRepeat(true);
    zoomInIcon->setAutoRepeatInterval(33);
    zoomInIcon->setAutoRepeatDelay(0);
    zoomInIcon->setIcon(QPixmap(":/zoomin.png"));
    zoomInIcon->setIconSize(iconSize);

    QToolButton *zoomInIcon_second = new QToolButton;
    zoomInIcon_second->setAutoRepeat(true);
    zoomInIcon_second->setAutoRepeatInterval(33);
    zoomInIcon_second->setAutoRepeatDelay(0);
    zoomInIcon_second->setIcon(QPixmap(":/zoomin.png"));
    zoomInIcon_second->setIconSize(iconSize);

    QToolButton *zoomOutIcon = new QToolButton;
    zoomOutIcon->setAutoRepeat(true);
    zoomOutIcon->setAutoRepeatInterval(33);
    zoomOutIcon->setAutoRepeatDelay(0);
    zoomOutIcon->setIcon(QPixmap(":/zoomout.png"));
    zoomOutIcon->setIconSize(iconSize);

    QToolButton *zoomOutIcon_second = new QToolButton;
    zoomOutIcon_second->setAutoRepeat(true);
    zoomOutIcon_second->setAutoRepeatInterval(33);
    zoomOutIcon_second->setAutoRepeatDelay(0);
    zoomOutIcon_second->setIcon(QPixmap(":/zoomout.png"));
    zoomOutIcon_second->setIconSize(iconSize);

    zoomSlider = new QSlider;
    zoomSlider->setMinimum(0);
    zoomSlider->setMaximum(1000); // 最大放大倍数
    zoomSlider->setValue(300);
    zoomSlider->setTickPosition(QSlider::TicksRight);

    zoomSlider_second = new QSlider;
    zoomSlider_second->setMinimum(0);
    zoomSlider_second->setMaximum(1000); // 最大放大倍数
    zoomSlider_second->setValue(300);
    zoomSlider_second->setTickPosition(QSlider::TicksRight);

    // Zoom slider layout
    QVBoxLayout *zoomSliderLayout = new QVBoxLayout;
    zoomSliderLayout->addWidget(zoomInIcon);
    zoomSliderLayout->addWidget(zoomSlider);
    zoomSliderLayout->addWidget(zoomOutIcon);

    QVBoxLayout *zoomSliderLayout_second = new QVBoxLayout;
    zoomSliderLayout_second->addWidget(zoomInIcon_second);
    zoomSliderLayout_second->addWidget(zoomSlider_second);
    zoomSliderLayout_second->addWidget(zoomOutIcon_second);

    resetButton = new QToolButton;
    resetButton->setText(tr("reset"));
    resetButton->setEnabled(false);

    resetButton_second = new QToolButton;
    resetButton_second->setText(tr("reset"));
    resetButton_second->setEnabled(false);

    // Label layout
    QHBoxLayout *labelLayout = new QHBoxLayout;
    label = new QLabel(name);
    selectModeButton = new QToolButton;
    selectModeButton->setText(tr("Select"));
    selectModeButton->setCheckable(true);
    selectModeButton->setChecked(true);

    selectModeButton_second = new QToolButton;
    selectModeButton_second->setText(tr("Select"));
    selectModeButton_second->setCheckable(true);
    selectModeButton_second->setChecked(true);

    dragModeButton = new QToolButton;
    dragModeButton->setText(tr("Drag"));
    dragModeButton->setCheckable(true);
    dragModeButton->setChecked(false);

    dragModeButton_second = new QToolButton;
    dragModeButton_second->setText(tr("Drag"));
    dragModeButton_second->setCheckable(true);
    dragModeButton_second->setChecked(false);

    QMenuBar *bar = new QMenuBar(this);

    drawFinalPathSpecificButton = new QToolButton;
    drawFinalPathSpecificButton->setText(tr("SpecificFinalPath"));
    drawFinalPathSpecificButton->setCheckable(true);
    drawFinalPathSpecificButton->setChecked(true);

    current_agent_text = new QLabel(this);
    current_agent_text->setText(tr("Current Agent:"));

    drawCurrentInstance_second = new QToolButton;
    drawCurrentInstance_second->setText(QString("Whether draw current agent"));
    drawCurrentInstance_second->setCheckable(true);
    drawCurrentInstance_second->setChecked(true);

    colorValueMenu       = new QMenu(QStringLiteral("SetColorType: Default"), this);
    QAction *pOpenAction = new QAction(QStringLiteral("Green"), this);
    QAction *pSaveAction = new QAction(QStringLiteral("Red"), this);

    connect(pOpenAction, &QAction::triggered, this, &View::setColorToGreen);
    connect(pSaveAction, &QAction::triggered, this, &View::setColorToRed);

    QMenuBar *bar_grid_map = new QMenuBar(this);

    gridMapValueMenu              = new QMenu(QStringLiteral("SetGridMapContent: Empty"), this);
    QAction *drawHyperGraphAction = new QAction(QStringLiteral("HyperGraph"), this);
    QAction *drawInstance         = new QAction(QStringLiteral("Instances"), this);
    QAction *drawHeuristic        = new QAction(QStringLiteral("Heuristic"), this);
    QAction *drawCluster          = new QAction(QStringLiteral("AllClusters"), this);
    QAction *drawEmptyAction      = new QAction(QStringLiteral("Empty"), this);

    gridMapValueMenu->addAction(drawHyperGraphAction);
    gridMapValueMenu->addAction(drawInstance);
    gridMapValueMenu->addAction(drawHeuristic);
    gridMapValueMenu->addAction(drawCluster);
    gridMapValueMenu->addAction(drawEmptyAction);

    bar_grid_map->addMenu(gridMapValueMenu);

    connect(drawHyperGraphAction, &QAction::triggered, this, &View::setHyperGraphGridMap);
    connect(drawHeuristic,        &QAction::triggered, this, &View::setHeuristicTable);
    connect(drawInstance,         &QAction::triggered, this, &View::setInstanceGridMap);
    connect(drawCluster,          &QAction::triggered, this, &View::setAllCluster);
    connect(drawEmptyAction,      &QAction::triggered, this, &View::setEmptyGridMap);

    bar->addMenu(colorValueMenu);

    QList<QAction*> fileAactions;
    fileAactions << pOpenAction << pSaveAction;
    colorValueMenu->addActions(fileAactions);

    antialiasButton = new QToolButton;
    antialiasButton->setText(tr("Antialiasing"));
    antialiasButton->setCheckable(true);
    antialiasButton->setChecked(false);

    openGlButton = new QToolButton;
    openGlButton->setText(tr("OpenGL"));
    openGlButton->setCheckable(true);

    antialiasButtonSecond = new QToolButton;
    antialiasButtonSecond->setText(tr("Antialiasing"));
    antialiasButtonSecond->setCheckable(true);
    antialiasButtonSecond->setChecked(true);
    graphicsView_second->setRenderHint(QPainter::Antialiasing, true);


    openGlButtonSecond = new QToolButton;
    openGlButtonSecond->setText(tr("OpenGL"));
    openGlButtonSecond->setCheckable(true);
    openGlButtonSecond->setChecked(true);
    graphicsView_second->setViewport( new QGLWidget(QGLFormat(QGL::SampleBuffers)));

#ifndef QT_NO_OPENGL
    openGlButton->setEnabled(QGLFormat::hasOpenGL());
#else
    openGlButton->setEnabled(false);
#endif
    printButton = new QToolButton;
    printButton->setIcon(QIcon(QPixmap(":/fileprint.png")));

    QButtonGroup *pointerModeGroup = new QButtonGroup(this);
    pointerModeGroup->setExclusive(true);
    pointerModeGroup->addButton(selectModeButton);
    pointerModeGroup->addButton(dragModeButton);

    labelLayout->addWidget(label);
    labelLayout->addStretch();
    labelLayout->addWidget(selectModeButton);
    labelLayout->addWidget(dragModeButton);

    labelLayout->addStretch();
    labelLayout->addWidget(bar);
    labelLayout->addWidget(drawFinalPathSpecificButton);

    labelLayout->addStretch();
    labelLayout->addWidget(antialiasButton);
    labelLayout->addWidget(openGlButton);
    labelLayout->addWidget(printButton);

//    labelLayout->addStretch();
//    labelLayout->addWidget(selectModeButton_second);
//    labelLayout->addWidget(dragModeButton_second);



    QGridLayout *topLayout = new QGridLayout;
    topLayout->addLayout(labelLayout, 0, 0);

    QButtonGroup *pointerModeGroup_second = new QButtonGroup(this);
    pointerModeGroup_second->setExclusive(true);
    pointerModeGroup_second->addButton(selectModeButton_second);
    pointerModeGroup_second->addButton(dragModeButton_second);

    agent_num_edit = new QLineEdit(this);
    agent_num_edit->setPlaceholderText(QString(std::to_string(current_shown_agent_).c_str()));
    agent_num_edit->setClearButtonEnabled(true);
    agent_num_edit->setValidator(new QIntValidator(agent_num_edit));
    agent_num_edit->setEchoMode(QLineEdit::Normal);

    connect(agent_num_edit, SIGNAL(editingFinished()), this, SLOT(agentNumEditFinished()));

    // 只允许输入整型
    // ui->lineEdit->setValidator(new QIntValidator(ui->lineEdit));

    // 只允许输入数字
    // ui->lineEdit->setValidator(new QRegExpValidator(QRegExp("[0-9]+$")));

    // 只能输入字母和数字
    // ui->lineEdit->setValidator(new QRegExpValidator(QRegExp("[a-zA-Z0-9]+$")));

    //agent_num_edit->setEchoMode();

    QHBoxLayout *labelLayout_second = new QHBoxLayout;
    labelLayout_second->addWidget(selectModeButton_second);
    labelLayout_second->addWidget(dragModeButton_second);
    labelLayout_second->addStretch();
    labelLayout_second->addWidget(bar_grid_map);
    labelLayout_second->addStretch();
    labelLayout_second->addWidget(current_agent_text);
    labelLayout_second->addWidget(agent_num_edit);
    labelLayout_second->addStretch();
    labelLayout_second->addWidget(drawCurrentInstance_second);
    labelLayout_second->addStretch();
    labelLayout_second->addWidget(antialiasButtonSecond);
    labelLayout_second->addWidget(openGlButtonSecond);

    topLayout->addLayout(labelLayout_second, 0, 1);

    QGridLayout *window_first = new QGridLayout;
    window_first->addWidget(graphicsView, 0, 0);
    window_first->addLayout(zoomSliderLayout, 0, 1);
    topLayout->addLayout(window_first, 1, 0);

    QGridLayout *window_second = new QGridLayout;
    window_second->addWidget(graphicsView_second, 0, 0);
    window_second->addLayout(zoomSliderLayout_second, 0, 1);
    topLayout->addLayout(window_second, 1, 1);


    topLayout->addWidget(resetButton, 2, 0);
    topLayout->addWidget(resetButton_second, 2, 1);

    topLayout->setColumnStretch(0, 2);
    topLayout->setColumnStretch(1, 3);

    QGridLayout *on_top_Layout = new QGridLayout;
    on_top_Layout->addLayout(topLayout, 0, 0);

    windowRatioSlider = new QSlider;
    windowRatioSlider->setMinimum(1);
    windowRatioSlider->setMaximum(1000); // 最大放大倍数
    windowRatioSlider->setValue(500);
    windowRatioSlider->setOrientation(Qt::Horizontal);

    on_top_Layout->addWidget(windowRatioSlider, 1, 0);

    setLayout(on_top_Layout);

    connect(resetButton, SIGNAL(clicked()), this, SLOT(resetView()));
    connect(resetButton_second, SIGNAL(clicked()), this, SLOT(resetViewSecond()));

    connect(zoomSlider, SIGNAL(valueChanged(int)), this, SLOT(setupMatrix()));
    connect(zoomSlider_second, SIGNAL(valueChanged(int)), this, SLOT(setupMatrixSecond()));

    connect(windowRatioSlider, SIGNAL(valueChanged(int)), this, SLOT(updateWindowRatio()));

    connect(graphicsView->verticalScrollBar(), SIGNAL(valueChanged(int)),
            this, SLOT(setResetButtonEnabled()));

    connect(graphicsView->horizontalScrollBar(), SIGNAL(valueChanged(int)),
            this, SLOT(setResetButtonEnabled()));

    connect(selectModeButton, SIGNAL(toggled(bool)), this, SLOT(togglePointerMode()));
    connect(drawFinalPathSpecificButton, SIGNAL(toggled(bool)), this, SLOT(setDrawNodeInPathSpecific()));

    connect(dragModeButton, SIGNAL(toggled(bool)), this, SLOT(togglePointerMode()));
    connect(dragModeButton_second, SIGNAL(toggled(bool)), this, SLOT(togglePointerModeSecond()));


    connect(antialiasButton, SIGNAL(toggled(bool)), this, SLOT(toggleAntialiasing()));
    connect(openGlButton, SIGNAL(toggled(bool)), this, SLOT(toggleOpenGL()));

    connect(antialiasButtonSecond, SIGNAL(toggled(bool)), this, SLOT(toggleAntialiasingSecond()));
    connect(openGlButtonSecond, SIGNAL(toggled(bool)), this, SLOT(toggleOpenGLSecond()));

    connect(zoomInIcon, SIGNAL(clicked()), this, SLOT(zoomIn()));
    connect(zoomOutIcon, SIGNAL(clicked()), this, SLOT(zoomOut()));

    connect(zoomInIcon_second, SIGNAL(clicked()), this, SLOT(zoomInSecond()));
    connect(zoomOutIcon_second, SIGNAL(clicked()), this, SLOT(zoomOutSecond()));

    connect(printButton, SIGNAL(clicked()), this, SLOT(print()));

    connect(drawCurrentInstance_second, SIGNAL(toggled(bool)), this, SLOT(toggleDrawCurrentInstance()));

    setupMatrix();

}

QGraphicsView *View::view() const
{
    return static_cast<QGraphicsView *>(graphicsView);
}

void View::updateWindowRatio() {
    //topLayout->setSizeConstraint(QLayout::SetNoConstraint);
//    topLayout->setColumnStretch(0, windowRatioSlider->value());
//    topLayout->setColumnStretch(1, windowRatioSlider->maximum() - windowRatioSlider->value());
    //std::cout << " windowRatioSlider->value() " << windowRatioSlider->value() << " / windowRatioSlider->maximum() " << windowRatioSlider->maximum() << std::endl;
    //topLayout->update();
}

QGraphicsView *View::second_view() const {
    return static_cast<QGraphicsView *>(graphicsView_second);
}

void View::keyPressEvent(QKeyEvent *ev) {
    //std::cout << ev->key() << " is pressed " << std::endl;
}

void View::keyReleaseEvent(QKeyEvent *ev) {
    //std::cout << ev->key() << " is released " << std::endl;
}

void View::resetView()
{
    zoomSlider->setValue(250);
    setupMatrix();
    graphicsView->ensureVisible(QRectF(0, 0, 0, 0));
    resetButton->setEnabled(false);
}

void View::resetViewSecond()
{
    zoomSlider_second->setValue(250);
    setupMatrixSecond();
    graphicsView_second->ensureVisible(QRectF(0, 0, 0, 0));
    resetButton_second->setEnabled(false);
}

void View::setResetButtonEnabled()
{
    resetButton->setEnabled(true);
}

void View::setResetButtonEnabledSecond()
{
    resetButton_second->setEnabled(true);
}


void View::setupMatrix()
{
    qreal scale = qPow(qreal(2), (zoomSlider->value() - 250) / qreal(50));

    QMatrix matrix;
    matrix.scale(scale, scale);

    graphicsView->setMatrix(matrix);
    setResetButtonEnabled();
}

void View::setupMatrixSecond()
{
    qreal scale = qPow(qreal(2), (zoomSlider_second->value() - 250) / qreal(50));
    QMatrix matrix;
    matrix.scale(scale, scale);
    graphicsView_second->setMatrix(matrix);
    setResetButtonEnabledSecond();
}

void View::togglePointerMode()
{
    graphicsView->setDragMode(selectModeButton->isChecked()
                              ? QGraphicsView::RubberBandDrag
                              : QGraphicsView::ScrollHandDrag);
    graphicsView->setInteractive(selectModeButton->isChecked());
}

void View::togglePointerModeSecond()
{
    graphicsView_second->setDragMode(selectModeButton_second->isChecked()
                              ? QGraphicsView::RubberBandDrag
                              : QGraphicsView::ScrollHandDrag);
    graphicsView_second->setInteractive(selectModeButton_second->isChecked());
}

void View::toggleOpenGL()
{
#ifndef QT_NO_OPENGL
    graphicsView->setViewport(openGlButton->isChecked() ? new QGLWidget(QGLFormat(QGL::SampleBuffers)) : new QWidget);
#endif
}


void View::toggleAntialiasing()
{
    graphicsView->setRenderHint(QPainter::Antialiasing, antialiasButton->isChecked());
}

void View::toggleOpenGLSecond()
{
#ifndef QT_NO_OPENGL
    graphicsView_second->setViewport(openGlButtonSecond->isChecked() ? new QGLWidget(QGLFormat(QGL::SampleBuffers)) : new QWidget);
#endif
}


void View::toggleAntialiasingSecond()
{
    graphicsView_second->setRenderHint(QPainter::Antialiasing, antialiasButtonSecond->isChecked());
}

// TODO: considering print to SVG picture in the future
void View::print()
{
#if QT_CONFIG(printdialog)
    QPrinter printer;
    QPrintDialog dialog(&printer, this);
    if (dialog.exec() == QDialog::Accepted) {
        // in the final result, at least one orientation reach boundary of scene
        QPainter painter(&printer);
        //graphicsView->render(&painter); // render just what in view and print it on file
        graphicsView->scene()->render(&painter); // render the whole scene and print it on file
    }
#endif
}

void View::zoomIn(int level)
{
    zoomSlider->setValue(zoomSlider->value() + level);
}

void View::zoomOut(int level)
{
    zoomSlider->setValue(zoomSlider->value() - level);
}

void View::zoomInSecond(int level)
{
    zoomSlider_second->setValue(zoomSlider_second->value() + level);
}

void View::zoomOutSecond(int level)
{
    zoomSlider_second->setValue(zoomSlider_second->value() - level);
}

void View::setAllNodesColor(int r, int g, int b) {
//    for(int level=0; level<all_level_nodes_.size(); level++) {
//        for(int i=0; i<all_level_nodes_[level].size(); i++) {
//            all_level_nodes_[level][i]->setColorRGB(r, g, b);
//        }
//    }
}

void View::setColorToGreen() {
    colorValueMenu->setTitle(tr("SetColorType: Green"));
    colorValueMenu->update();
    setAllNodesColor(0, 255, 0);
}

void View::setColorToRed() {
    colorValueMenu->setTitle(tr("SetColorType: Red"));
    colorValueMenu->update();
    setAllNodesColor(255, 0, 0);
}

void View::setHyperGraphGridMap() {
    gridMapValueMenu->setTitle(tr("SetGridMapContent: HyperGraph"));
    gridMapValueMenu->update();
    resetGridMapToEmpty();
    drawInstancesOfMAPFColor();
    drawFreeGridGroupColor();
    drawInstancesOfMAPFText();
    drawNearbyHyperNodes();
}

void View::setEmptyGridMap() {
    gridMapValueMenu->setTitle(tr("SetGridMapContent: Empty"));
    gridMapValueMenu->update();
    resetGridMapToEmpty();
}

void View::setDrawNodeInPathSpecific() {
//    for(int level=0; level<all_level_nodes_.size(); level++) {
//        for(int i=0; i<all_level_nodes_[level].size(); i++) {
//            if(all_level_nodes_[level][i]->node_->in_final_path_) {
//                all_level_nodes_[level][i]->setSpecific(drawFinalPathSpecificButton->isChecked());
//            }
//        }
//    }
}

void View::setInstanceGridMap() {
    gridMapValueMenu->setTitle(tr("SetGridMapContent: Instances"));
    gridMapValueMenu->update();
    resetGridMapToEmpty();
    drawInstancesOfMAPFColor();
    drawInstancesOfMAPFText();
    //drawFreeGridGroupColor();
}

void View::setAllCluster() {
    gridMapValueMenu->setTitle(tr("SetGridMapContent: AllClusters"));
    gridMapValueMenu->update();
    resetGridMapToEmpty();
    drawAllCluster();
}

void View::setHeuristicTable() {
    gridMapValueMenu->setTitle(tr("SetGridMapContent: HeuristicTable"));
    gridMapValueMenu->update();
    resetGridMapToEmpty();
    drawInstancesOfMAPFColor();
    drawInstancesOfMAPFText();
    drawFreeGridGroupColor();
    drawHeuristicTable();
    std::cout << " draw heuristic table of agent " << current_shown_agent_ << std::endl;
}

void View::initGridMap() {
    // draw grid map
    int block_total_width = GRID_WIDTH + 2 * GRID_INTERVAL;
    all_grid_.clear();
    for(int j=0; j<dim_[1]; j++) {
        for(int i=0; i<dim_[0]; i++) {
            Grid *grid = new Grid(10);
            //std::cout << "i*block_total_width, j*block_total_width" << i*block_total_width << ", " << j*block_total_width << std::endl;
            grid->setPos(i*block_total_width, j*block_total_width);
            freeNav::Pointi<2> pt({i, j});
            if(is_occupied_(pt)) {
                grid->setColorRGB(150, 150,150);
            } else {
                grid->setColorRGB(255, 255, 255);
            }
            all_grid_.push_back(grid);
        }
    }
    if(instance_.size() > 0) {
        agent_start_ = new CircleOfGrid(15);
        agent_start_->setColorRGB(0, 255, 0);
        agent_start_->setPos(instance_[current_shown_agent_].first[0]*block_total_width,
                             instance_[current_shown_agent_].first[1]*block_total_width);
        agent_target_ = new CircleOfGrid(15);
        agent_target_->setColorRGB(255, 0,0);
        agent_target_->setPos(instance_[current_shown_agent_].second[0]*block_total_width,
                              instance_[current_shown_agent_].second[1]*block_total_width);
    }
}

void View::initAgentRelations() {
    // draw grid map
    int block_total_width = GRID_WIDTH + 2 * GRID_INTERVAL;
    all_agent_relations_.clear();
    all_agent_relations_.clear();
    for(int i=0; i<instance_.size(); i++) {
        for(int j=0; j<instance_.size(); j++) {
            Grid *grid = new Grid(10);
            //std::cout << "i*block_total_width, j*block_total_width" << i*block_total_width << ", " << j*block_total_width << std::endl;
            grid->setPos(i*block_total_width, j*block_total_width);
            if(layered_mapf->all_passing_agent_[i].find(j) == layered_mapf->all_passing_agent_[i].end())
            //if(layered_mapf->all_related_agent_[i].find(j) == layered_mapf->all_related_agent_[i].end())
            {
                grid->setColorRGB(255, 255,255);
            } else {
                grid->setColorRGB(COLOR_TABLE[i%30][0], COLOR_TABLE[i%30][1], COLOR_TABLE[i%30][2]);
            }
            all_agent_relations_.push_back(grid);
        }
    }
}

void View::initAgentNameText() {
    agent_name_column_.clear();
    agent_name_row_.clear();
    int block_total_width = GRID_WIDTH + 2 * GRID_INTERVAL;
    QFont font("TimesNewRoman", 20, QFont::Bold);
    for(int i=0; i<instance_.size(); i++) {
        QGraphicsTextItem * agent_text_row = new QGraphicsTextItem(QString(std::to_string(i).c_str()));
        agent_text_row->setPos(i*block_total_width-block_total_width*1/4, -block_total_width-block_total_width/4);
        agent_text_row->setZValue(10);
        agent_text_row->setFont(font);
        agent_name_row_.push_back(agent_text_row);
        QGraphicsTextItem * agent_text_col = new QGraphicsTextItem(QString(std::to_string(i).c_str()));
        agent_text_col->setPos(-block_total_width-block_total_width/4, i*block_total_width-block_total_width/2);
        agent_text_col->setZValue(10);
        agent_text_col->setFont(font);
        agent_name_column_.push_back(agent_text_col);
    }
//    agent_name_row_;
//    agent_name_column_;
}

void View::resetGridMapToEmpty() {
    for(auto& grid : all_grid_) {
        grid->strs_.clear();
    }
    for(int j=0; j<dim_[1]; j++) {
        for(int i=0; i<dim_[0]; i++) {
            freeNav::Pointi<2> pt({i, j});
            freeNav::Id grid_id = freeNav::PointiToId(pt, dim_);
            Grid *grid = all_grid_[grid_id];
            if(is_occupied_(pt)) {
                grid->setColorRGB(150, 150,150);
            } else {
                grid->setColorRGB(255, 255, 255);
            }
        }
    }
}

void View::drawHeuristicTable() {
    for(int j=0; j<dim_[1]; j++) {
        for(int i=0; i<dim_[0]; i++) {
            freeNav::Pointi<2> pt({i, j});
            if(is_occupied_(pt)) continue;
            freeNav::Id grid_id = freeNav::PointiToId(pt, dim_);
            Grid *grid = all_grid_[grid_id];
            const int& hyper_node_id = layered_mapf->grid_map_[grid_id]->hyper_node_id_;
            std::stringstream ss_heuristic_value;
            ss_heuristic_value << layered_mapf->all_heuristic_table_agent_[current_shown_agent_][hyper_node_id];
            grid->strs_.push_back(ss_heuristic_value.str());
        }
    }
}

void View::drawAllCluster() {
    // 1, pick largest cluster
    const auto& all_clusters = layered_mapf->all_clusters_;//layered_mapf->selectLargestCluster(layered_mapf->all_related_agent_);
    // 2, draw largest cluster's instance
    for(int cid=0; cid<all_clusters.size(); cid++) {
        for(const auto& agent_id : all_clusters[cid]) {
            auto fill_color = COLOR_TABLE[cid % 30]; // cv::Vec3b(200, 200, 200);

            freeNav::Id start_id = PointiToId(instance_[agent_id].first, dim_),
                    target_id = PointiToId(instance_[agent_id].second, dim_);
            all_grid_[start_id]->text_color_ = Qt::white;
            all_grid_[start_id]->setColorRGB(fill_color[0], fill_color[1], fill_color[2]);

            all_grid_[target_id]->text_color_ = Qt::white;
            all_grid_[target_id]->setColorRGB(fill_color[0], fill_color[1], fill_color[2]);

            int mean_color = fill_color[0] + fill_color[1] + fill_color[2];
            if (mean_color > 450) {
                all_grid_[start_id]->text_color_ = Qt::black;
                all_grid_[target_id]->text_color_ = Qt::black;
            } else {
                all_grid_[start_id]->text_color_ = Qt::white;
                all_grid_[target_id]->text_color_ = Qt::white;
            }

            std::stringstream ss_start;
            ss_start << agent_id << "s";
            all_grid_[start_id]->strs_.push_back(ss_start.str());

            std::stringstream ss_target;
            ss_target << agent_id << "t";
            all_grid_[target_id]->strs_.push_back(ss_target.str());

            std::stringstream ss_cluster_id;
            ss_cluster_id << cid << "c";
            all_grid_[start_id]->strs_.push_back(ss_cluster_id.str());
            all_grid_[target_id]->strs_.push_back(ss_cluster_id.str());
        }
    }
}

void View::drawInstancesOfMAPFColor() {
//    for(int i=0; i<instance_.size(); i++) {
//        const auto& ist = instance_[i];
//        freeNav::Pointi<2> start = ist.first, target = ist.second;
//        freeNav::Id start_id = PointiToId(start, dim_), target_id = PointiToId(target, dim_);
//        if(is_occupied_(start) || is_occupied_(target)) {
//            std::cout << " start or target is occupied " << std::endl;
//        }
//
//        auto fill_color = COLOR_TABLE[i % 30]; // cv::Vec3b(200, 200, 200);
//        all_grid_[start_id]->text_color_ = Qt::white;
//        all_grid_[start_id]->setColorRGB(fill_color[0], fill_color[1], fill_color[2]);
//
//        all_grid_[target_id]->text_color_ = Qt::white;
//        all_grid_[target_id]->setColorRGB(fill_color[0], fill_color[1], fill_color[2]);
//        int mean_color = fill_color[0] + fill_color[1] + fill_color[2];
//        if(mean_color > 450) {
//            all_grid_[start_id]->text_color_ = Qt::black;
//            all_grid_[target_id]->text_color_ = Qt::black;
//        } else {
//            all_grid_[start_id]->text_color_ = Qt::white;
//            all_grid_[target_id]->text_color_ = Qt::white;
//        }
//    }

    // draw low level search traveled grid
    int failed_instance_id = 10;
    for(const int& id : ids_to_draw_) {
        all_grid_[id]->setColorRGB(0, 0, 0);
        all_grid_[id]->text_color_ = Qt::white;
    }
    std::cout << " cluster failed_instance_id agent id = "  << *layered_mapf->all_clusters_[failed_instance_id].begin() << std::endl;
    for(int i=0; i<layered_mapf->all_clusters_.size(); i++) {
        const auto& current_agents = layered_mapf->all_clusters_[i];
        if(i==failed_instance_id) {
            freeNav::Pointi<2> start = instance_[*current_agents.begin()].first, target = instance_[*current_agents.begin()].second;
            freeNav::Id start_id = PointiToId(start, dim_), target_id = PointiToId(target, dim_);
            std::cout << " ids_to_draw_ contain failed_instance_id start/target: " << (ids_to_draw_.find(start_id) != ids_to_draw_.end()) << " / "
                    << (ids_to_draw_.find(target_id) != ids_to_draw_.end()) << std::endl;
            continue;
        }
        for(const int& agent_id : current_agents) {
            const auto& ist = instance_[agent_id];
            freeNav::Pointi<2> start = ist.first, target = ist.second;
            freeNav::Id start_id = PointiToId(start, dim_), target_id = PointiToId(target, dim_);
            auto fill_color = COLOR_TABLE[i % 30]; // cv::Vec3b(200, 200, 200);
            if (i < failed_instance_id) {
                all_grid_[target_id]->text_color_ = Qt::white;
                all_grid_[target_id]->setColorRGB(fill_color[0], fill_color[1], fill_color[2]);
            } else {
                all_grid_[start_id]->text_color_ = Qt::white;
                all_grid_[start_id]->setColorRGB(fill_color[0], fill_color[1], fill_color[2]);
            }
            int mean_color = fill_color[0] + fill_color[1] + fill_color[2];
            if(mean_color > 450) {
                all_grid_[start_id]->text_color_ = Qt::black;
                all_grid_[target_id]->text_color_ = Qt::black;
            } else {
                all_grid_[start_id]->text_color_ = Qt::white;
                all_grid_[target_id]->text_color_ = Qt::white;
            }
        }
    }
}

void View::drawInstancesOfMAPFText() {
    for(int i=0; i<instance_.size(); i++) {
        const auto& ist = instance_[i];
        freeNav::Pointi<2> start = ist.first, target = ist.second;
        freeNav::Id start_id = PointiToId(start, dim_), target_id = PointiToId(target, dim_);
        if(is_occupied_(start) || is_occupied_(target)) {
            std::cout << " start or target is occupied " << std::endl;
        }
        auto fill_color = COLOR_TABLE[i % 30]; // cv::Vec3b(200, 200, 200);
        std::stringstream ss_start; ss_start << i << "s";
        all_grid_[start_id]->strs_.push_back(ss_start.str());

        std::stringstream ss_target; ss_target << i << "t";
        all_grid_[target_id]->strs_.push_back(ss_target.str());
    }
}

void View::drawFreeGridGroupColor() {
    if(layered_mapf == nullptr) { return; }
    const auto& hyper_nodes = layered_mapf->all_hyper_nodes_;
    for(int i=0; i<hyper_nodes.size(); i++) {
        if(hyper_nodes[i]->free_grid_group_.free_grids_.empty()) { continue; }
        for(const auto& grid : hyper_nodes[i]->free_grid_group_.free_grids_) {
            auto fill_color = COLOR_TABLE[i % 30];
            all_grid_[grid->id_]->setColorRGB(fill_color[0], fill_color[1], fill_color[2]);
        }
    }
}

void View::drawFreeGridGroupText() {
    if(layered_mapf == nullptr) { return; }
    const auto& hyper_nodes = layered_mapf->all_hyper_nodes_;
    for(int i=0; i<hyper_nodes.size(); i++) {
        if(hyper_nodes[i]->free_grid_group_.free_grids_.empty()) { continue; }
        for(const auto& grid : hyper_nodes[i]->free_grid_group_.free_grids_) {
            std::stringstream ss;
            ss << i << "g";
            all_grid_[grid->id_]->strs_.push_back(ss.str());
        }
    }
}

void View::drawNearbyHyperNodes() {
    if(layered_mapf == nullptr) { return; }
    const auto& hyper_nodes = layered_mapf->all_hyper_nodes_;
    // 1, draw neighbor of agent's start and target
    for(int i=0; i<2*instance_.size(); i++) {
        if(hyper_nodes[i]->agent_grid_ptr_ == nullptr) { continue; }
        std::stringstream ss;
        ss << hyper_nodes[i]->connecting_nodes_.size() << "nf";
        all_grid_[hyper_nodes[i]->agent_grid_ptr_->id_]->strs_.push_back(ss.str());
    }
    // 2, draw neighbor of free grid group
    for(int i=2*instance_.size(); i<hyper_nodes.size(); i++) {
        const auto& free_grids = hyper_nodes[i]->free_grid_group_.free_grids_;
        std::stringstream ss;
        ss << hyper_nodes[i]->connecting_nodes_.size() << "na";
        for(const auto& grid : free_grids) {
            all_grid_[grid->id_]->strs_.push_back(ss.str());
        }
    }
}

void View::agentNumEditFinished() {
    if( agent_num_edit->text().toInt() >= instance_.size() ||  agent_num_edit->text().toInt() < 0) { return; }
    current_shown_agent_ = agent_num_edit->text().toInt();
    agent_num_edit->setText(QString(std::to_string(current_shown_agent_).c_str()));
    agent_num_edit->setPlaceholderText(QString(std::to_string(current_shown_agent_).c_str()));
    //setHeuristicTable();
    int block_total_width = GRID_WIDTH + 2 * GRID_INTERVAL;
    agent_start_->setPos(instance_[current_shown_agent_].first[0]*block_total_width,
                         instance_[current_shown_agent_].first[1]*block_total_width);
    agent_start_->update();
    agent_target_->setPos(instance_[current_shown_agent_].second[0]*block_total_width,
                          instance_[current_shown_agent_].second[1]*block_total_width);
    agent_target_->update();
}

void View::toggleDrawCurrentInstance() {
    agent_start_->setVisible(!agent_start_->isVisible());
    agent_target_->setVisible(!agent_target_->isVisible());
}