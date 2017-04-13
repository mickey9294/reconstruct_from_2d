#include "SketchGraph.h"



SketchGraph::SketchGraph()
{
}

SketchGraph::SketchGraph(std::list<QPointF> vertices, std::list<Line> edges)
{
	int num_vertices = vertices.size();
	vertices_.resize(num_vertices);
	int vert_idx = 0;
	for (std::list<QPointF>::iterator vert_it = vertices.begin(); vert_it != vertices.end(); ++vert_it, ++vert_idx)
	{
		vertices_[vert_idx][0] = vert_it->x();
		vertices_[vert_idx][1] = vert_it->y();
	}

	edges_.resize(edges.size());
	adjacency_list_.resize(num_vertices);
	adjacency_mat_.resize(num_vertices, num_vertices);
	adjacency_mat_.setZero();
	int edge_idx = 0;
	for (std::list<Line>::iterator edge_it = edges.begin(); edge_it != edges.end(); ++edge_it, ++edge_idx)
	{
		int p1 = edge_it->p1();
		int p2 = edge_it->p2();
		adjacency_list_[p1].push_back(p2);
		adjacency_list_[p2].push_front(p1);
		adjacency_mat_(p1, p2) = 1;
		adjacency_mat_(p2, p1) = 1;

		edges_[edge_idx][0] = std::min(p1, p2);
		edges_[edge_idx][1] = std::max(p1, p2);
	}
}

SketchGraph::SketchGraph(const SketchGraph & another)
{
	int num_vertices = another.num_vertices();
	vertices_.resize(num_vertices);
	int vert_idx = 0;
	for (std::vector<Eigen::Vector2f>::const_iterator vert_it = another.const_vertices_begin();
		vert_it != another.const_vertices_end(); ++vert_it, ++vert_idx)
	{
		vertices_[vert_idx][0] = vert_it->x();
		vertices_[vert_idx][1] = vert_it->y();
	}

	adjacency_list_.resize(num_vertices);
	adjacency_mat_.resize(num_vertices, num_vertices);
	adjacency_mat_.setZero();
	int adj_idx = 0;
	for (std::vector<std::list<int>>::const_iterator adj_it = another.const_adjacency_list_begin();
		adj_it != another.const_adjacency_list_end(); ++adj_it, ++adj_idx)
	{
		for (std::list<int>::const_iterator jt = adj_it->begin(); jt != adj_it->end(); jt++)
		{
			adjacency_list_[adj_idx].push_back(*jt);
			adjacency_mat_(adj_idx, *jt) = 1;
		}
	}

	edges_.resize(another.num_edges());
	int edge_idx = 0;
	for (std::vector<Line>::const_iterator edge_it = another.const_edges_begin();
		edge_it != another.const_edges_end(); ++edge_it, ++edge_idx)
	{
		edges_[edge_idx].setP1(edge_it->p1());
		edges_[edge_idx].setP2(edge_it->p2());
	}
}


SketchGraph::~SketchGraph()
{
}

std::vector<Eigen::Vector2f>::const_iterator SketchGraph::const_vertices_begin() const
{
	return vertices_.begin();
}

std::vector<Eigen::Vector2f>::const_iterator SketchGraph::const_vertices_end() const
{
	return vertices_.end();
}

const std::vector<Eigen::Vector2f>& SketchGraph::const_vertices()
{
	return vertices_;
}

std::vector<Line>::const_iterator SketchGraph::const_edges_begin() const
{
	return edges_.begin();
}

std::vector<Line>::const_iterator SketchGraph::const_edges_end() const
{
	return edges_.end();
}

std::vector<std::list<int>>::const_iterator SketchGraph::const_adjacency_list_begin() const
{
	return adjacency_list_.begin();
}

std::vector<std::list<int>>::const_iterator SketchGraph::const_adjacency_list_end() const
{
	return adjacency_list_.end();
}

std::list<int>& SketchGraph::get_vertex_adjacency(int v)
{
	return adjacency_list_[v];
}

Eigen::MatrixXi SketchGraph::get_adjacency_mat() const
{
	return adjacency_mat_;
}

Eigen::MatrixXi & SketchGraph::adjacency_mat()
{
	return adjacency_mat_;
}

bool SketchGraph::are_adjacent(int v1, int v2)
{
	if (adjacency_mat_(v1, v2) == 0)
		return false;
	else
		return true;
}

void SketchGraph::remove_vertex_in_adjacency_list(int v)
{
	for (int i = v + 1; i < num_vertices(); i++)
	{
		for (std::list<int>::iterator it = adjacency_list_[i].begin();
			it != adjacency_list_[i].end(); ++it)
		{
			if (*it == v)
			{
				adjacency_list_[i].erase(it);
				break;
			}
		}
	}
}

void SketchGraph::rebuild_adjacency_list()
{
	adjacency_list_.clear();
	adjacency_list_.resize(num_vertices());
	for (std::vector<Line>::iterator edge_it = edges_.begin(); edge_it != edges_.end(); ++edge_it)
	{
		adjacency_list_[edge_it->p1()].push_back(edge_it->p2());
		adjacency_list_[edge_it->p2()].push_front(edge_it->p1());
	}
}

int SketchGraph::num_vertices() const
{
	return vertices_.size();
}

int SketchGraph::num_edges() const
{
	return edges_.size();
}

bool SketchGraph::edges_intersect(int v11, int v12, int v21, int v22)
{
	Eigen::Vector2f &p1 = vertices_[v11];
	Eigen::Vector2f &q1 = vertices_[v12];
	Eigen::Vector2f &p2 = vertices_[v21];
	Eigen::Vector2f &q2 = vertices_[v22];

	/* Check wheter two lines are parallel */
	Eigen::Vector2f v1 = p1 - q1;
	Eigen::Vector2f v2 = p2 - q2;
	if (v1.cross(v2).norm() < 1e-3)
		return false;

	/* Check whether two line segments are end to end */
	if (v11 == v21 || v11 == v22 || v12 == v21 || v12 == v22)
		return false;

	/* Check whether two line segments intersect */
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);
	if (o1 != o2 && o3 != o4)
		return true;

	return false;
}

int SketchGraph::orientation(Eigen::Vector2f p, Eigen::Vector2f q, Eigen::Vector2f r)
{
	float val = (q[1] - p[1]) * (r[0] - q[0]) -
		(q[0] - p[0]) * (r[1] - q[1]);
	if (std::abs(val) < 1e-4)
		return 0;
	return val > 0 ? 1 : 2;
}
