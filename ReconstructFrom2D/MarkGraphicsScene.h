#pragma once

#include <QGraphicsView>
#include <QGraphicsLineItem>
#include <QPointF>
#include <QGraphicsSceneMouseEvent>
#include <QList>
#include <QGraphicsPixmapItem>
#include <QLine>
#include <QApplication>
#include <QStyle>
#include <QMouseEvent>

#include <memory>
#include <iostream>

#include "Line.h"

class MarkGraphicsScene : public QGraphicsView
{
	Q_OBJECT

public:
	MarkGraphicsScene(QWidget *parent = 0);
	~MarkGraphicsScene();

	public slots:
	void undo();
	void reset();
	void set_image(QString image_path);

signals:
	void resize_main_window(int dw, int dh);

protected:
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	//void paintEvent(QPaintEvent *event);
	//void keyPressEvent(QKeyEvent *event);

private:
	const int STACK_SIZE = 5;
	const int IMAGE_WIDTH = 1024;
	const int IMAGE_HEIGHT = 768;

	bool perm_;
	QPixmap image_;
	QPen line_pen_;
	QPen vertex_pen_;

	std::shared_ptr<QGraphicsScene> graphics_scene_;
	std::shared_ptr<QGraphicsPixmapItem> pixmap_item_;
	std::shared_ptr<QLineF> line_;
	QGraphicsLineItem * line_item_;

	std::shared_ptr<QPointF> vertex_;
	QGraphicsEllipseItem * vertex_item_;

	std::list<Line> line_list_;
	std::list<QGraphicsLineItem *> line_item_stack_;
	std::list<QPointF> vertex_list_;
	std::list<QGraphicsEllipseItem *> vertex_item_stack_;
	std::list<int> mode_stack_;

	//void makeItemsControllable(bool areControllable);
	int find_nearest_vertex(const QPointF &point, QPointF &nearest_vertex);
};
