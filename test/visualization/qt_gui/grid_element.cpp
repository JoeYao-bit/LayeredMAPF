//
// Created by yaozhuo on 2023/10/28.
//

#include "grid_element.h"
#include <QtWidgets>

Grid::Grid(int z)
{
    QColor default_color;
    default_color.setRgb(0, 255, 0);
    this->color_ = default_color;

    setZValue(z);

    setFlags(ItemIsSelectable);

    // QGraphicsItem要实现鼠标不按下的情况下，获取鼠标move事件，
    // 可通过**hoverMoveEvent()来实现，可通过设置setAcceptHoverEvents(true)**使其生效
    setAcceptHoverEvents(true);
}

// 图像的绘制必须在boundingRect()函数所确定的Rect之中
QRectF Grid::boundingRect() const
{
    return QRectF(-GRID_WIDTH/2 - GRID_INTERVAL,
                  -GRID_WIDTH/2 - GRID_INTERVAL,
                  GRID_WIDTH + 2*GRID_INTERVAL,
                  GRID_WIDTH + 2*GRID_INTERVAL);
}

// 用来确定哪些区域需要重构（repaint）
// 用来检测碰撞
QPainterPath Grid::shape() const
{
    QPainterPath path;
    path.addRect(-GRID_WIDTH/2 - GRID_INTERVAL,
                 -GRID_WIDTH/2 - GRID_INTERVAL,
                 GRID_WIDTH + 2*GRID_INTERVAL,
                 GRID_WIDTH + 2*GRID_INTERVAL);
    return path;
}

void Grid::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget);

    QColor fillColor = (option->state & QStyle::State_Selected) ? color_.light(50) : color_;

    if (option->state & QStyle::State_MouseOver)
        fillColor = fillColor.light(125);

    // 缩放比例
    const qreal lod = option->levelOfDetailFromTransform(painter->worldTransform());

    int width = 2;
    if (option->state & QStyle::State_Selected)
        width = 4;
    painter->setPen(QPen(fillColor, width));

    QPainterPath path;
    path.addRect(QRect(-GRID_WIDTH/2-GRID_INTERVAL,
                       -GRID_WIDTH/2-GRID_INTERVAL,
                       GRID_WIDTH,
                       GRID_WIDTH));

    // 绘制带有圆角的矩形 r,xRnd和yRnd参数指定了圆角的圆度。0是有角的角，99是最大圆度
    //painter->drawRect();
    painter->fillPath(path, fillColor);
    // Draw text, but do not draw if too far
    if (lod >= .5)
    {
        painter->setPen(QPen(Qt::black, width));
        painter->drawPath(path);
        QFont font("Times", GRID_FONT_SIZE);
        font.setStyleStrategy(QFont::ForceOutline);
        painter->setFont(font);
        painter->save(); // 保存当前Painter状态
        painter->scale(1, 1);
        painter->setPen(text_color_); // pen color is text color
        int index = 1;
//        painter->drawText(-GRID_WIDTH/2-GRID_INTERVAL + 10,
//                          -GRID_WIDTH/2-GRID_INTERVAL + 15*(index++),
//                          QString("x=%1").arg(this->x()));//
//        painter->drawText(-GRID_WIDTH/2-GRID_INTERVAL + 10,
//                          -GRID_WIDTH/2-GRID_INTERVAL + 15*(index++),
//                          QString("y=%1").arg(this->y()));

        for(const auto& str : strs_) {
            painter->drawText(-GRID_WIDTH/2-GRID_INTERVAL + 3,
                              -GRID_WIDTH/2-GRID_INTERVAL + 3 + font.pointSize()*1.25*(index++),
                              QString(str.c_str()));//
        }
        painter->restore(); //恢复上面保存的状态
    }
}

void Grid::setColorRGB(int r, int g, int b) {
    this->color_ = QColor::fromRgb(r, g, b);
    update();
}

CircleOfGrid::CircleOfGrid(int z) {
    QColor default_color;
    default_color.setRgb(0, 255, 0);
    this->color_ = default_color;

    setZValue(z);
}

// 图像的绘制必须在boundingRect()函数所确定的Rect之中
QRectF CircleOfGrid::boundingRect() const
{
    int radius_of_circle = ceil((GRID_WIDTH + 2*GRID_INTERVAL)*sqrt(2));
    return QRectF(-radius_of_circle, -radius_of_circle, radius_of_circle, radius_of_circle);
}

// 用来确定哪些区域需要重构（repaint）
// 用来检测碰撞
QPainterPath CircleOfGrid::shape() const
{
    int radius_of_circle = ceil((GRID_WIDTH + 2*GRID_INTERVAL)*sqrt(2));
    QPainterPath path;
    path.addRect(-radius_of_circle, -radius_of_circle, radius_of_circle, radius_of_circle);
    return path;
}

void CircleOfGrid::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget);

    QColor fillColor = (option->state & QStyle::State_Selected) ? color_.light(50) : color_;
    painter->setPen(QPen(fillColor, 30));
    int radius_of_circle = ceil(4*(GRID_WIDTH + 2*GRID_INTERVAL));
    painter->drawEllipse(-radius_of_circle,-radius_of_circle,2*radius_of_circle,2*radius_of_circle);
}

void CircleOfGrid::setColorRGB(int r, int g, int b) {
    this->color_ = QColor::fromRgb(r, g, b);
}