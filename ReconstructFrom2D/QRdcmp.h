#pragma once

#include <Eigen\Core>

#include "nr3.h"

class QRdcmp
{
public:
	int n;
	Eigen::MatrixXd qt, r;
	bool sing;

	QRdcmp(Eigen::MatrixXd &a);
	~QRdcmp();

	void solve(Eigen::VectorXd &b, Eigen::VectorXd &x);
	void qtmult(Eigen::VectorXd &b, Eigen::VectorXd &x);
	void rsolve(Eigen::VectorXd &b, Eigen::VectorXd &x);
	void update(Eigen::VectorXd &u, Eigen::VectorXd &v);
	void rotate(const int i, const double a, const double b);
};

