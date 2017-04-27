#pragma once

#include <vector>
#include <list>

#include <Eigen\Core>

class PlanarFace
{
public:
	PlanarFace();
	PlanarFace(int id, const std::vector<int> &circuit);
	PlanarFace(const PlanarFace &another);
	~PlanarFace();

	PlanarFace &operator=(const PlanarFace &another);

	void set_id(int id);
	int id() const;

	void set_normal(const Eigen::Vector3f &normal);
	Eigen::Vector3f get_normal() const;

	const std::vector<int> &const_circuit() const;
	std::vector<int> & get_circuit();
	std::string circuit_string() const;

	void add_edge(int line_id);
	std::list<int> &get_edges();
	const std::list<int> &const_edges() const;

private:
	int id_;
	std::vector<int> circuit_;
	Eigen::Vector3f normal_;
	std::list<int> edges_list_;
};

