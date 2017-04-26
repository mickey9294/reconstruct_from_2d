#pragma once

#include <Eigen\Core>
#include <assert.h>
#include <iostream>

class LUdcmp
{
public:
	LUdcmp(Eigen::MatrixXd &a);
	~LUdcmp();

	int n;
	Eigen::MatrixXd lu;
	Eigen::VectorXi indx;
	float d;

	void solve(Eigen::VectorXd &b, Eigen::VectorXd &x);
	void solve(Eigen::MatrixXd &b, Eigen::MatrixXd &x);
	void inverse(Eigen::MatrixXd &ainv);
	float det();
	void mprove(Eigen::VectorXd &b, Eigen::VectorXd &x);
	Eigen::MatrixXd &aref;
};

