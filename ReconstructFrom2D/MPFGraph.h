#pragma once

#include <vector>
#include <list>
#include <algorithm>

#include <Eigen\Core>

#include "MPFVertex.h"
#include "Line.h"

class MPFGraph
{
public:
	MPFGraph();
	MPFGraph(const std::vector<std::vector<int>> &circuits, const std::vector<Eigen::Vector2f> &points);
	MPFGraph(const MPFGraph &another);
	~MPFGraph();

	std::vector<MPFVertex>::const_iterator const_vertices_begin() const;
	std::vector<MPFVertex>::const_iterator const_vertices_end() const;
	std::vector<std::list<int>>::const_iterator const_adj_list_begin() const;
	std::vector<std::list<int>>::const_iterator const_adj_list_end() const;
	std::vector<Eigen::Vector2f>::const_iterator const_points_begin() const;
	std::vector<Eigen::Vector2f>::const_iterator const_points_end() const;

	int num_vertices() const;
	int num_points() const;

	int get_vertex_weigth(int index);

	bool are_adjacent(int v1, int v2);

	MPFVertex & operator[](int index);

private:
	std::vector<MPFVertex> vertices_;
	std::vector<Eigen::Vector2f> points_;
	//std::vector<Line> edges_;
	std::vector<std::list<int>> adjacency_list_;
	Eigen::MatrixXi adjacency_mat_;
	

	bool coexist(MPFVertex &v1, MPFVertex &v2);
	void get_common_edges(std::vector<int> &circuit1, std::vector<int> &circuit2, std::list<Line> &common);
};

