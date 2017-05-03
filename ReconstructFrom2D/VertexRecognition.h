#pragma once

#include <QPixmap>
#include <QImage>
#include <qdebug.h>
#include <QLine>

#include <iostream>
#include <fstream>
#include <cmath>

#include <Eigen\Core>

#include "lsd.h"

class VertexRecognition
{
public:
	VertexRecognition();
	~VertexRecognition();

	void set_image(const QPixmap &image);
	void set_sketch_vertices(const std::list<QPointF> &sketch_vertices);

	void get_precise_vertices(std::vector<int> &precise_vertices_id,
		std::vector<QPointF> &precies_vertices);

private:
	QImage image_;
	std::vector<QPointF> sketch_vertices_;

	double distance(const QPointF &p1, const QPointF &p2);
	double distance(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);
	double min_lines_dist(const QLineF &line1, const QLineF & line2, const QPointF &intersect_point);
	Eigen::Vector3d line_equation(const QPointF &p1, const QPointF &p2);
	Eigen::Vector3d line_equation(const QLineF &line);

};

