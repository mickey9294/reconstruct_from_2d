#pragma once

#include <vector>

#include <Eigen\Core>

class PlanarFace
{
public:
	PlanarFace();
	PlanarFace(int id, const std::vector<int> &circuit);
	PlanarFace(const PlanarFace &another);
	~PlanarFace();

	void set_id(int id);
	int id() const;

	void set_normal(const Eigen::Vector3f &normal);
	Eigen::Vector3f get_normal() const;

	const std::vector<int> &const_circuit() const;
	std::vector<int> & get_circuit();

private:
	int id_;
	std::vector<int> circuit_;
	Eigen::Vector3f normal_;
};

