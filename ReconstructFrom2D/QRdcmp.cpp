#include "QRdcmp.h"


QRdcmp::QRdcmp(Eigen::MatrixXd & a)
	: n(a.rows()),  r(a), sing(false)
{
	qt.resize(n, n);

	int i, j, k;
	Eigen::VectorXd c(n), d(n);
	double scale, sigma, sum, tau;

	for (k = 0; k < n - 1; k++)
	{
		scale = 0;
		for (i = k; i < n; i++)
			scale = std::max(scale, std::abs(r(i, k)));
		if (std::abs(scale) < 1.0e-20)
		{
			sing = true;
			c[k] = 0;
			d[k] = 0;
		}
		else
		{
			for (i = k; i < n; i++)
				r(i, k) /= scale;
			for (sum = 0, i = k; i < n; i++)
				sum += SQR(r(i, k));
			sigma = SIGN(std::sqrt(sum), r(k, k));
			r(k, k) += sigma;
			c[k] = sigma * r(k, k);
			d[k] = -scale * sigma;
			for (j = k + 1; j < n; j++)
			{
				for (sum = 0, i = k; i < n; i++)
					sum += r(i, k) * r(i, j);
				tau = sum / c[k];
				for (i = k; i < n; i++)
					r(i, j) -= tau * r(i, k);
			}
		}
	}
	d[n - 1] = r(n - 1, n - 1);
	if (std::abs(d[n - 1]) < 1.0e-20)
		sing = true;
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
			qt(i, j) = 0;
		qt(i, i) = 1.0;
	}
	for (k = 0; k < n - 1; k++)
	{
		if (std::abs(c[k]) > 1.0e-8)
		{
			for (j = 0; j < n; j++)
			{
				sum = 0;
				for (i = k; i < n; i++)
					sum += r(i, k) * qt(i, j);
				sum /= c[k];
				for (i = k; i < n; i++)
					qt(i, j) -= sum * r(i, k);
			}
		}
	}
	for (i = 0; i < n; i++)
	{
		r(i, i) = d[i];
		for (j = 0; j < i; j++)
			r(i, j) = 0;
	}
}

QRdcmp::~QRdcmp()
{
}

void QRdcmp::solve(Eigen::VectorXd & b, Eigen::VectorXd & x)
{
	qtmult(b, x);
	rsolve(x, x);
}

void QRdcmp::qtmult(Eigen::VectorXd & b, Eigen::VectorXd & x)
{
	int i, j;
	double sum;
	for (i = 0; i < n; i++)
	{
		sum = 0;
		for (j = 0; j < n; j++)
			sum += qt(i, j) * b[j];
		x[i] = sum;
	}
}

void QRdcmp::rsolve(Eigen::VectorXd & b, Eigen::VectorXd & x)
{
	int i, j;
	double sum;
	if (sing)
	{
		std::cerr << "attempting solve in a singular QR" << std::endl;
		assert(false);
	}
	for (i = n - 1; i >= 0; i--)
	{
		sum = b[i];
		for (j = i + 1; j < n; j++)
			sum -= r(i, j) * x[j];
		x[i] = sum / r(i, i);
	}
}

void QRdcmp::update(Eigen::VectorXd & u, Eigen::VectorXd & v)
{
	int i, k;
	Eigen::VectorXd w(u);

	for (k = n - 1; k >= 0; k--)
	{
		if (std::abs(w[k]) > 1.0e-8)
			break;
	}
	if (k < 0)
		k = 0;
	for (i = k - 1; i >= 0; i--)
	{
		rotate(i, w[i], -w[i + 1]);
		if (std::abs(w[i]) < 1.0e-20)
			w[i] = std::abs(w[i + 1]);
		else if (std::abs(w[i]) > std::abs(w[i + 1]))
			w[i] = std::abs(w[i]) * std::sqrt(1.0 + SQR(w[i + 1] / w[i]));
		else
			w[i] = std::abs(w[i + 1]) * std::sqrt(1.0 + SQR(w[i] / w[i + 1]));
	}
	for (i = 0; i < n; i++)
		r(0, i) += w[0] * v[i];
	for (i = 0; i < k; i++)
		rotate(i, r(i, i), -r(i + 1, i));
	for (i = 0; i < n; i++)
	{
		if (std::abs(r(i, i)) < 1.0e-20)
			sing = true;
	}
}

void QRdcmp::rotate(const int i, const double a, const double b)
{
	int j;
	double c, fact, s, w, y;
	if (std::abs(a) < 1.0e-20)
	{
		c = 0;
		s = (b > 0 || std::abs(b) < 1.0e-20) ? 1.0 : -1.0;
	}
	else if (std::abs(a) > std::abs(b))
	{
		fact = b / a;
		c = SIGN(1.0 / std::sqrt(1.0 + (fact*fact)), a);
		s = fact * c;
	}
	else
	{
		fact = a / b;
		s = SIGN(1.0 / std::sqrt(1.0 + (fact * fact)), b);
		c = fact * s;
	}

	for (j = i; j < n; j++)
	{
		y = r(i, j);
		w = r(i + 1, j);
		r(i, j) = c * y - s * w;
		r(i + 1, j) = s * y + c * w;
	}

	for (j = 0; j < n; j++)
	{
		y = qt(i, j);
		w = qt(i + 1, j);
		qt(i, j) = c * y - s * w;
		qt(i + 1, j) = s * y + c * w;
	}
}
