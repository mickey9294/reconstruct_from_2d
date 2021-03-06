#include "MPFGraph.h"



MPFGraph::MPFGraph()
{
}

MPFGraph::MPFGraph(const std::vector<std::vector<int>> &circuits, const std::vector<Eigen::Vector2d> &points)
{
	int num_vertices = circuits.size();
	vertices_.reserve(num_vertices);
	std::vector<int> weights(num_vertices);
	for (int i = 0; i < num_vertices; i++)
	{
		vertices_.push_back(MPFVertex(circuits[i]));
		weights[i] = vertices_[i].weight();
	}

	struct vertex_comperator
	{
		inline bool operator() (const MPFVertex &vert1, const MPFVertex &vert2)
		{
			return vert1.weight() > vert2.weight();
		}
	};

	std::sort(vertices_.begin(), vertices_.end(), vertex_comperator());

	for (int i = 0; i < num_vertices; i++)
		vertices_[i].set_id(i);

	points_.resize(points.size());
	for (int i = 0; i < points.size(); i++)
	{
		points_[i][0] = points[i].x();
		points_[i][1] = points[i].y();
	}

	adjacency_mat_.resize(num_vertices, num_vertices);
	adjacency_mat_.setZero();
	adjacency_list_.resize(num_vertices);
	for (int i = 0; i < num_vertices; i++)
	{
		for (int j = i + 1; j < num_vertices; j++)
		{
			if (coexist(vertices_[i], vertices_[j]))
			{
				adjacency_list_[i].push_back(j);
				adjacency_list_[j].push_front(i);
				adjacency_mat_(i, j) = 1;
				adjacency_mat_(j, i) = 1;
			}
		}
	}
}

MPFGraph::MPFGraph(const MPFGraph & another)
{
	int num_vertices = another.num_vertices();
	int num_points = another.num_points();

	vertices_.reserve(num_vertices);
	for (std::vector<MPFVertex>::const_iterator vert_it = another.const_vertices_begin();
		vert_it != another.const_vertices_end(); ++vert_it)
	{
		vertices_.push_back(MPFVertex(*vert_it));
	}

	adjacency_list_.resize(num_vertices);
	adjacency_mat_.resize(num_vertices, num_vertices);
	adjacency_mat_.setZero();
	int adj_idx = 0;
	for (std::vector<std::list<int>>::const_iterator adj_it = another.const_adj_list_begin();
		adj_it != another.const_adj_list_end(); ++adj_it, ++adj_idx)
	{
		for (std::list<int>::const_iterator jt = adj_it->begin(); jt != adj_it->end(); ++jt)
		{
			adjacency_list_[adj_idx].push_back(*jt);
			adjacency_mat_(adj_idx, *jt) = 1;
		}
	}

	points_.resize(num_points);
	int p_idx = 0;
	for (std::vector<Eigen::Vector2d>::const_iterator p_it = another.const_points_begin();
		p_it != another.const_points_end(); ++p_it, ++p_idx)
	{
		points_[p_idx][0] = p_it->x();
		points_[p_idx][1] = p_it->y();
	}
}


MPFGraph::~MPFGraph()
{
}

MPFGraph & MPFGraph::operator=(const MPFGraph & another)
{
	int num_vertices = another.num_vertices();
	int num_points = another.num_points();

	vertices_.reserve(num_vertices);
	for (std::vector<MPFVertex>::const_iterator vert_it = another.const_vertices_begin();
		vert_it != another.const_vertices_end(); ++vert_it)
	{
		vertices_.push_back(MPFVertex(*vert_it));
	}

	adjacency_list_.resize(num_vertices);
	adjacency_mat_.resize(num_vertices, num_vertices);
	adjacency_mat_.setZero();
	int adj_idx = 0;
	for (std::vector<std::list<int>>::const_iterator adj_it = another.const_adj_list_begin();
		adj_it != another.const_adj_list_end(); ++adj_it, ++adj_idx)
	{
		for (std::list<int>::const_iterator jt = adj_it->begin(); jt != adj_it->end(); ++jt)
		{
			adjacency_list_[adj_idx].push_back(*jt);
			adjacency_mat_(adj_idx, *jt) = 1;
		}
	}

	points_.resize(num_points);
	int p_idx = 0;
	for (std::vector<Eigen::Vector2d>::const_iterator p_it = another.const_points_begin();
		p_it != another.const_points_end(); ++p_it, ++p_idx)
	{
		points_[p_idx][0] = p_it->x();
		points_[p_idx][1] = p_it->y();
	}

	return *this;
}

std::vector<MPFVertex>::const_iterator MPFGraph::const_vertices_begin() const
{
	return vertices_.begin();
}

std::vector<MPFVertex>::const_iterator MPFGraph::const_vertices_end() const
{
	return vertices_.end();
}

std::vector<std::list<int>>::const_iterator MPFGraph::const_adj_list_begin() const
{
	return adjacency_list_.begin();
}

std::vector<std::list<int>>::const_iterator MPFGraph::const_adj_list_end() const
{
	return adjacency_list_.end();
}

std::vector<Eigen::Vector2d>::const_iterator MPFGraph::const_points_begin() const
{
	return points_.begin();
}

std::vector<Eigen::Vector2d>::const_iterator MPFGraph::const_points_end() const
{
	return points_.end();
}

int MPFGraph::num_vertices() const
{
	return vertices_.size();
}

int MPFGraph::num_points() const
{
	return points_.size();
}

int MPFGraph::get_vertex_weigth(int index)
{
	return vertices_[index].weight();
}

bool MPFGraph::are_adjacent(int v1, int v2)
{
	return adjacency_mat_(v1, v2) == 0 ? false : true;
}

MPFVertex & MPFGraph::operator[](int index)
{
	return vertices_[index];
}

bool MPFGraph::coexist(MPFVertex & v1, MPFVertex & v2)
{
	std::list<Line> common_edges;
	get_common_edges(v1.get_circuit(), v2.get_circuit(), common_edges);
	if (common_edges.size() < 2)
		return true;

	Line e0 = common_edges.front();
	Eigen::Vector2d p0 = points_[e0.p1()];
	Eigen::Vector2d q0 = points_[e0.p2()];
	bool vertical0 = std::abs(q0[0] - p0[0]) < 1e-3;
	float slope0;
	float intercept0;
	if (vertical0)
	{
		slope0 = std::numeric_limits<float>::max();
		intercept0 = std::numeric_limits<float>::max();
	}
	else
	{
		slope0 = (q0[1] - p0[1]) / (q0[0] - p0[0]);
		intercept0 = -slope0 * p0[0] + p0[1];
	}
	std::list<Line>::iterator com_it = common_edges.begin();
	std::advance(com_it, 1);
	for (; com_it != common_edges.end(); ++com_it)
	{
		Eigen::Vector2d p = points_[com_it->p1()];
		Eigen::Vector2d q = points_[com_it->p2()];
		if (std::abs(q[0] - p[0]) < 1e-3)
		{
			if (vertical0)
			{
				if (std::abs(p0[0] - p[0]) > 1e-3)
					return false;
			}
			else
				return false;
		}
		else
		{
			if (vertical0)
				return false;
			else
			{
				float slope = (q[1] - p[1]) / (q[0] - p[0]);
				float intercept = -slope * p[0] + p[1];

				if (std::abs(slope - slope0) > 1e-3 || std::abs(intercept - intercept0) > 1e-3)
					return false;
			}
		}
	}
	return true;
}

void MPFGraph::get_common_edges(std::vector<int>& circuit1, std::vector<int>& circuit2, std::list<Line>& common)
{
	int n1 = circuit1.size();
	int n2 = circuit2.size();

	for (int i = 0; i < n1; i++)
	{
		int start1 = circuit1[i];
		int end1 = circuit1[(i + 1) % n1];
		if (start1 > end1)
			std::swap(start1, end1);
		for (int j = 0; j < n2; j++)
		{
			int start2 = circuit2[j];
			int end2 = circuit2[(j + 1) % n2];
			if (start2 > end2)
				std::swap(start2, end2);

			if (start1 == start2 && end1 == end2)
				common.push_back(Line(start1, end1));
		}
	}
}


