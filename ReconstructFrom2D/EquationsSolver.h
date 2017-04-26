#pragma once

#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

#include <Eigen\Core>

#include <boost\algorithm\string.hpp>

#include <engine.h>

class EquationsSolver
{
public:
	EquationsSolver();
	~EquationsSolver();

	void solve(int Nf, int Nv, 
		std::vector<Eigen::Vector2f> &refined_vertices, Eigen::VectorXf &refinded_q);
};