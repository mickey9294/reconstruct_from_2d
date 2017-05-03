#include "PlanarFace.h"



PlanarFace::PlanarFace()
{
}

PlanarFace::PlanarFace(int id, const std::vector<int>& circuit)
{
	id_ = id;
	circuit_ = circuit;
}

PlanarFace::PlanarFace(const PlanarFace & another)
{
	id_ = another.id();
	const std::vector<int> &circuit = another.const_circuit();
	circuit_.resize(circuit.size());
	for (int i = 0; i < circuit.size(); i++)
		circuit_[i] = circuit[i];

	normal_ = another.get_normal();

	const std::list<int> &edges = another.const_edges();
	for (std::list<int>::const_iterator e_it = edges.begin(); e_it != edges.end(); ++e_it)
		edges_list_.push_back(*e_it);
}


PlanarFace::~PlanarFace()
{
}

PlanarFace & PlanarFace::operator=(const PlanarFace & another)
{
	id_ = another.id();
	const std::vector<int> &circuit = another.const_circuit();
	circuit_.resize(circuit.size());
	for (int i = 0; i < circuit.size(); i++)
		circuit_[i] = circuit[i];

	normal_ = another.get_normal();

	const std::list<int> &edges = another.const_edges();
	for (std::list<int>::const_iterator e_it = edges.begin(); e_it != edges.end(); ++e_it)
		edges_list_.push_back(*e_it);

	return *this;
}

void PlanarFace::set_id(int id)
{
	id_ = id;
}

int PlanarFace::id() const
{
	return id_;
}

void PlanarFace::set_normal(const Eigen::Vector3d & normal)
{
	normal_ = normal;
}

Eigen::Vector3d PlanarFace::get_normal() const
{
	return normal_;
}

const std::vector<int> & PlanarFace::const_circuit() const
{
	return circuit_;
}

std::vector<int>& PlanarFace::get_circuit()
{
	return circuit_;
}

std::string PlanarFace::circuit_string() const
{
	if (circuit_.empty())
		return "";

	std::stringstream ss;
	ss << circuit_.front();
	for (int i = 1; i < circuit_.size(); i++)
		ss << " " << circuit_[i];

	return ss.str();
}

void PlanarFace::add_edge(int line_id)
{
	edges_list_.push_back(line_id);
}

std::list<int>& PlanarFace::get_edges()
{
	return edges_list_;
}

const std::list<int>& PlanarFace::const_edges() const
{
	return edges_list_;
}
