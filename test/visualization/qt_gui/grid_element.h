//
// Created by yaozhuo on 2023/10/28.
//

#ifndef FREENAV_GRID_ELEMENT_H
#define FREENAV_GRID_ELEMENT_H

#include <QColor>
#include <QGraphicsItem>
#include "config_gui.h"

class Grid : public QGraphicsItem
{
public:
    Grid(int z);

    QRectF boundingRect() const override;

    QPainterPath shape() const override;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *item, QWidget *widget) override;

    void setColorRGB(int r, int g, int b);

    std::vector<std::string> strs_;

    QColor text_color_;

private:
    QColor color_;

    friend class MainWindow;
    friend class View;

};

class CircleOfGrid : public QGraphicsItem
{
public:
    CircleOfGrid(int z);

    QRectF boundingRect() const override;

    QPainterPath shape() const override;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *item, QWidget *widget) override;

    void setColorRGB(int r, int g, int b);

private:
    QColor color_;

    friend class MainWindow;
    friend class View;

};

//class TextOfAgent : public QGraphicsTextItem {
//public:
//    TextOfAgent(const QString& str, int z);
//
//    QRectF boundingRect() const override;
//
//    QPainterPath shape() const override;
//
//    void paint(QPainter *painter, const QStyleOptionGraphicsItem *item, QWidget *widget) override;
//
//    void setColorRGB(int r, int g, int b);
//
//private:
//    QColor color_;
//
//    QString str_;
//
//    friend class MainWindow;
//    friend class View;
//};

#endif //FREENAV_GRID_ELEMENT_H
