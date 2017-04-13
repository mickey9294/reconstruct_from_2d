#pragma once

#include <vector>

class MPFVertex
{
public:
	MPFVertex();
	MPFVertex(const std::vector<int> circuit);
	MPFVertex(const MPFVertex &another);
	~MPFVertex();

	int id() const;
	void set_id(int id);

	int weight() const;

	std::vector<int>::const_iterator const_circuit_begin() const;
	std::vector<int>::const_iterator const_circuit_end() const;

	std::vector<int> & get_circuit();

private:
	int id_;
	int weight_;
	std::vector<int> circuit_;
};

