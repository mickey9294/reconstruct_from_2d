#pragma once

#include <QPixmap>
#include <QPointF>
#include <QImage>
#include <qline.h>

#include <list>
#include <vector>
#include <fstream>
#include <cmath>

#include <Eigen\Core>

#include "Line.h"
#include "lsd.h"

class VerticesConnector
{
public:
	VerticesConnector();
	VerticesConnector(const QPixmap &image, const std::list<QPointF> &vertices);
	~VerticesConnector();

	typedef std::pair<int, int> Interval;

	void set_image(const QPixmap &image);

	void get_connection_lines(std::list<Line> &connection_lines);

	bool is_connected(const QPointF &v1, const QPointF &v2);

private:
	QImage image_;
	std::vector<QPointF> vertices_;
	int count_;

	void insert_interval(std::list<Interval> &intervals, Interval new_interval);
};

