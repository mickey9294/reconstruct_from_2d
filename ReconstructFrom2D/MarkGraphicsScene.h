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
#include <QVector>
#include <QMouseEvent>
#include <unordered_set>

#include <memory>
#include <iostream>
#include <time.h>
#include <string>
#include <fstream>
#include <iostream>

#include <Eigen\Core>

#include <boost\algorithm\string.hpp>

#include "Line.h"

class MarkGraphicsScene : public QGraphicsView
{
	Q_OBJECT

public:
	MarkGraphicsScene(QWidget *parent = 0);
	~MarkGraphicsScene();

	enum State {
		LabelSilhouette,
		LabelParallel
	};

	std::list<QPointF> & get_vertices();
	std::list<Line> & get_edges();
	std::list<std::vector<int>> & get_parallel_groups();
	const std::string & get_image_path() const;
	const std::vector<std::vector<int>> &const_face_circuits() const;

	public slots:
	void undo();
	void reset();
	void set_image(QString image_path);
	void set_faces(const std::vector<std::vector<int>> &face_circuits);
	void change_state();
	void finish_label_parallelism();
	void save_current_state();
	void load_current_state();

signals:
	void resize_main_window(int dw, int dh);

protected:
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	//void paintEvent(QPaintEvent *event);
	void keyPressEvent(QKeyEvent *event);

private:
	const int STACK_SIZE = 5;
	const int IMAGE_WIDTH = 1024;
	const int IMAGE_HEIGHT = 768;

	std::string image_path_;

	bool perm_;
	QPixmap image_;
	QPen line_pen_;
	QPen vertex_pen_;
	QPen focus_pen_;
	QPen parallel_pen_;

	std::list<std::vector<int>> parallel_lines_group_;
	int focused_line_id_;
	std::vector<bool> line_chosen_before_;
	std::unordered_set<int> current_parallel_group_;

	State state_;

	Eigen::MatrixXf N_;  /* Each column represents the direction vector of an edge */
	Eigen::MatrixXf A_;  /* Each column represents the first end point of an edge */
	Eigen::MatrixXf B_; /* Each column represents the second end point of an edge */
	 
	std::shared_ptr<QGraphicsScene> graphics_scene_;
	std::shared_ptr<QGraphicsPixmapItem> pixmap_item_;
	std::shared_ptr<QLineF> line_;
	QGraphicsLineItem * line_item_;

	std::shared_ptr<QPointF> vertex_;
	QGraphicsEllipseItem * vertex_item_;

	QGraphicsTextItem *number_item_;

	std::list<Line> line_list_;
	std::list<QGraphicsLineItem *> line_item_stack_;
	std::list<QPointF> vertex_list_;
	std::list<QGraphicsEllipseItem *> vertex_item_stack_;
	std::list<QGraphicsPolygonItem *> face_item_stack_;
	std::list<QGraphicsTextItem *> number_item_stack_;
	std::list<int> mode_stack_;

	std::vector<std::vector<int>> face_circuits_;

	//void makeItemsControllable(bool areControllable);
	int find_nearest_vertex(const QPointF &point, QPointF &nearest_vertex);
	bool find_nearest_edge(const QPoint &point, int &nearest_edge_id, float &distance);
	QColor random_color();
	QPen random_pen();
};
