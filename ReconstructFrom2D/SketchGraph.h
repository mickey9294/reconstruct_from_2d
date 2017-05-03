#pragma once

#include <QPoint>

#include <vector>
#include <list>
#include <algorithm>

#include <Eigen\Core>
#include <Eigen\Dense>

#include "Line.h"

class SketchGraph
{
public:
	SketchGraph();
	SketchGraph(const std::list<QPointF> &vertices, const std::list<Line> &edges);
	SketchGraph(const SketchGraph &another);
	~SketchGraph();

	SketchGraph & operator=(const SketchGraph &another);

	std::vector<Eigen::Vector2d>::const_iterator const_vertices_begin() const;
	std::vector<Eigen::Vector2d>::const_iterator const_vertices_end() const;
	const std::vector<Eigen::Vector2d> & const_vertices();

	std::vector<Line>::const_iterator const_edges_begin() const;
	std::vector<Line>::const_iterator const_edges_end() const;

	std::vector<std::list<int>>::const_iterator const_adjacency_list_begin() const;
	std::vector<std::list<int>>::const_iterator const_adjacency_list_end() const;

	std::list<int> & get_vertex_adjacency(int v);
	Eigen::MatrixXi get_adjacency_mat() const;
	Eigen::MatrixXi & adjacency_mat();

	int num_vertices() const;
	int num_edges() const;

	bool edges_intersect(int v11, int v12, int v21, int v22);
	bool are_adjacent(int v1, int v2);

	void remove_vertex_in_adjacency_list(int v);
	void rebuild_adjacency_list();

private:
	std::vector<Eigen::Vector2d> vertices_;
	std::vector<Line> edges_;
	std::vector<std::list<int>> adjacency_list_;
	Eigen::MatrixXi adjacency_mat_;

	int orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r);
};

