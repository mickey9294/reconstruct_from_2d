#pragma once

#include <engine.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <assert.h>

#include <Eigen\Core>
#include <boost\algorithm\string.hpp>

#include <QPointF>

#include "Line.h"

class CameraClibrator
{
public:
	CameraClibrator();
	CameraClibrator(const std::list<QPointF> &vertices, const std::list<Line> &lines,
		const std::list<std::vector<int>> &parallel_groups, int w, int h);
	CameraClibrator(const std::vector<Eigen::Vector2d> &vertices, const std::vector<Line> &lines,
		const std::vector<std::vector<int>> &parallel_groups, int w, int h);
	~CameraClibrator();

	void calibrate(double &focal_length, Eigen::Vector2d &primary_point);

private:
	std::vector<Eigen::Vector2d> vertices_;
	std::vector<Line> lines_;
	std::vector<std::vector<int>> parallel_groups_;
	int width_;
	int height_;

	Eigen::Vector3d get_line_equation(int line_id);
	double point_line_distance(const Eigen::Vector3d &line, const Eigen::Vector2d &point);
};

