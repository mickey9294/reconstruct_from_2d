#include "MPFVertex.h"



MPFVertex::MPFVertex()
{
	id_ = 0;
	weight_ = 0;
}

MPFVertex::MPFVertex(const std::vector<int> circuit)
{
	id_ = 0;
	weight_ = circuit.size();
	circuit_.resize(circuit.size());
	for (int i = 0; i < circuit.size(); i++)
		circuit_[i] = circuit[i];
}

MPFVertex::MPFVertex(const MPFVertex & another)
{
	id_ = another.id();
	weight_ = another.weight();
	circuit_.resize(weight_);
	int idx = 0;
	for (std::vector<int>::const_iterator it = another.const_circuit_begin();
		it != another.const_circuit_end; ++it)
		circuit_[idx++] = *it;
}


MPFVertex::~MPFVertex()
{
}

int MPFVertex::id() const
{
	return id_;
}

void MPFVertex::set_id(int id)
{
	id_ = id;
}

int MPFVertex::weight() const
{
	return weight_;
}

std::vector<int>::const_iterator MPFVertex::const_circuit_begin() const
{
	return circuit_.begin();
}

std::vector<int>::const_iterator MPFVertex::const_circuit_end() const
{
	return circuit_.end();
}

std::vector<int>& MPFVertex::get_circuit()
{
	return circuit_;
}