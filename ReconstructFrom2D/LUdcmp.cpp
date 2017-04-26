#include "LUdcmp.h"



LUdcmp::LUdcmp(Eigen::MatrixXd & a)
	: n(a.rows()), lu(a), aref(a), indx(n)
{
	const double TINY = 1.0e-8;
	int i, imax, j, k;
	double big, temp;
	Eigen::VectorXd vv(n);
	d = 1.0;

	for (i = 0; i < n; i++)
	{
		big = 0;
		for (j = 0; j < n; j++)
		{
			if ((temp = std::abs(lu(i,j))) > big)
				big = temp;
		}
		if (std::abs(big) < 1.0e-20)
		{
			std::cerr << "Singular matrix in LUdcmp" << std::endl;
			assert(false);
		}
		vv[i] = 1.0 / big;
	}

	for (k = 0; k < n; k++)
	{
		big = 0;
		for (i = k; i < n; i++)
		{
			temp = vv[i] * abs(lu(i, k));
			if (temp > big)
			{
				big = temp;
				imax = i;
			}
		}
		if (k != imax)
		{
			for (j = 0; j < n; j++)
			{
				temp = lu(imax, j);
				lu(imax, j) = lu(k, j);
				lu(k, j) = temp;
			}
			d = -d;
			vv[imax] = vv[k];
		}
		indx[k] = imax;
		if (std::abs(lu(k, k)) < 1.0e-20)
			lu(k, k) = TINY;

		for (i = k + 1; i < n; i++)
		{
			temp = lu(i, k) /= lu(k, k);
			for (j = k + 1; j < n; j++)
				lu(i, j) -= temp * lu(k, j);
		}
	}

	std::cout << "lu: \n" << lu << std::endl;
}


LUdcmp::~LUdcmp()
{
}

void LUdcmp::solve(Eigen::VectorXd & b, Eigen::VectorXd & x)
{
	int i, ii = 0, ip, j;
	double sum;
	if (b.rows() != n || x.rows() != n)
	{
		std::cerr << "LUdcmp::solve bad sizes" << std::endl;
		assert(false);
	}
	for (i = 0; i < n; i++)
		x[i] = b[i];
	for (i = 0; i < n; i++)
	{
		ip = indx[i];
		sum = x[ip];
		x[ip] = x[i];
		if (ii != 0)
		{
			for (j = ii - 1; j < i; j++)
				sum -= lu(i, j) * x[j];
		}
		else if (std::abs(sum) > 1.0e-8)
			ii = i + 1;
		x[i] = sum;
	}
	std::cout << "x:\n" << x << std::endl;
	for (i = n - 1; i >= 0; i--)
	{
		sum = x[i];
		for (j = i + 1; j < n; j++)
			sum -= lu(i, j) * x[j];
		double luii = lu(i, i);
		x[i] = sum / luii;
	}
}

void LUdcmp::solve(Eigen::MatrixXd & b, Eigen::MatrixXd & x)
{
	int i, j, m = b.cols();
	if (b.rows() != n || x.rows() != n || b.cols() != x.cols())
	{
		std::cerr << "LUdcmp::solve bad sizes" << std::endl;
		assert(false);
	}

	Eigen::VectorXd xx(n);
	for (j = 0; j < m; j++)
	{
		for (i = 0; i < n; i++)
			xx[i] = b(i, j);
		solve(xx, xx);
		for (i = 0; i < n; i++)
			x(i, j) = xx[i];
	}
}

void LUdcmp::inverse(Eigen::MatrixXd & ainv)
{
	int i, j;
	ainv.resize(n, n);
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
			ainv(i, j) = 0.0;
		ainv(i, i) = 1.0;
	}
	solve(ainv, ainv);
}

float LUdcmp::det()
{
	double dd = d;
	for (int i = 0; i < n; i++)
		dd *= lu(i, i);
	return dd;
}

void LUdcmp::mprove(Eigen::VectorXd & b, Eigen::VectorXd & x)
{
	int i, j;
	Eigen::VectorXd r(n);

	for (i = 0; i < n; i++)
	{
		double sdp = -b[i];
		for (j = 0; j < n; j++)
			sdp += (double)aref(i, j) * (double)x[j];
		r[i] = sdp;
	}
	solve(r, r);
	for (i = 0; i < n; i++)
		x[i] -= r[i];
}
