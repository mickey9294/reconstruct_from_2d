#pragma once

#include <QPoint>

#include <vector>
#include <list>
#include <algorithm>

#include <Eigen\Core>

#include "Line.h"

class SketchGraph
{
public:
	SketchGraph();
	SketchGraph(std::list<QPointF> vertices, std::list<Line> edges);
	SketchGraph(const SketchGraph &another);
	~SketchGraph();

	std::vector<Eigen::Vector2f>::const_iterator const_vertices_begin() const;
	std::vector<Eigen::Vector2f>::const_iterator const_vertices_end() const;
	const std::vector<Eigen::Vector2f> & const_vertices();

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
	std::vector<Eigen::Vector2f> vertices_;
	std::vector<Line> edges_;
	std::vector<std::list<int>> adjacency_list_;
	Eigen::MatrixXi adjacency_mat_;

	int orientation(Eigen::Vector2f p, Eigen::Vector2f q, Eigen::Vector2f r);
};

