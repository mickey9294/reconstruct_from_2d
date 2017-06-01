#include "CameraClibrator.h"



CameraClibrator::CameraClibrator()
{
}

CameraClibrator::CameraClibrator(const std::list<QPointF> &vertices, const std::list<Line> &lines,
	const std::list<std::vector<int>> &parallel_groups, int w, int h)
{
	vertices_.resize(vertices.size());
	int idx = 0;
	for (std::list<QPointF>::const_iterator v_it = vertices.begin(); v_it != vertices.end(); ++v_it, ++idx)
	{
		vertices_[idx][0] = v_it->x();
		vertices_[idx][1] = v_it->y();
	}

	lines_.resize(lines.size());
	idx = 0;
	for (std::list<Line>::const_iterator l_it = lines.begin(); l_it != lines.end(); ++l_it, ++idx)
	{
		lines_[idx].setP1(l_it->p1());
		lines_[idx].setP2(l_it->p2());
	}

	parallel_groups_.resize(parallel_groups.size());
	idx = 0;
	for (std::list<std::vector<int>>::const_iterator it = parallel_groups.begin(); it != parallel_groups.end(); ++it, ++idx)
	{
		int j = 0;
		parallel_groups_[idx].resize(it->size());
		for (std::vector<int>::const_iterator jt = it->begin(); jt != it->end(); ++jt, ++j)
			parallel_groups_[idx][j] = *jt;
	}

	width_ = w;
	height_ = h;
}

CameraClibrator::CameraClibrator(const std::vector<Eigen::Vector2d>& vertices, const std::vector<Line>& lines, 
	const std::vector<std::vector<int>>& parallel_groups, int w, int h)
{
	vertices_.resize(vertices.size());
	int idx = 0;
	for (std::vector<Eigen::Vector2d>::const_iterator v_it = vertices.begin(); v_it != vertices.end(); ++v_it, ++idx)
	{
		vertices_[idx][0] = v_it->x();
		vertices_[idx][1] = v_it->y();
	}

	lines_.resize(lines.size());
	idx = 0;
	for (std::vector<Line>::const_iterator l_it = lines.begin(); l_it != lines.end(); ++l_it, ++idx)
	{
		lines_[idx].setP1(l_it->p1());
		lines_[idx].setP2(l_it->p2());
	}

	parallel_groups_.resize(parallel_groups.size());
	idx = 0;
	for (std::vector<std::vector<int>>::const_iterator it = parallel_groups.begin(); it != parallel_groups.end(); ++it, ++idx)
	{
		int j = 0;
		parallel_groups_[idx].resize(it->size());
		for (std::vector<int>::const_iterator jt = it->begin(); jt != it->end(); ++jt, ++j)
			parallel_groups_[idx][j] = *jt;
	}

	width_ = w;
	height_ = h;
}


CameraClibrator::~CameraClibrator()
{
}

void CameraClibrator::calibrate(double & focal_length, Eigen::Vector2d & primary_point)
{
	focal_length = 0;
	primary_point.setZero();

	if (parallel_groups_.size() < 2)
		return;

	/* Get the equations of all lines */
	std::vector<Eigen::Vector3d> line_equations(lines_.size());
	for (int i = 0; i < lines_.size(); i++)
		line_equations[i] = get_line_equation(i);

	std::vector<Eigen::Vector3d> vp_list;

	for (std::vector<std::vector<int>>::iterator group_it = parallel_groups_.begin();
		group_it != parallel_groups_.end(); ++group_it)
	{
		if (group_it->size() < 2)
			continue;

		Eigen::Vector3d vanishing_point;
		vanishing_point.setZero();
		//int vp_count = 0;
		double min_sum_dist = std::numeric_limits<double>::max();

		for (int i = 0; i < group_it->size(); i++)
		{
			/* Pick the first line from the current parallel group */
			int line_id_1 = group_it->operator[](i);
			Eigen::Vector3d &line_1 = line_equations[line_id_1];

			for (int j = i + 1; j < group_it->size(); j++)
			{
				/* Pick the second line from the current parallel group */
				int line_id_2 = group_it->operator[](j);
				Eigen::Vector3d &line_2 = line_equations[line_id_2];

				/* Compute a vanishing point from these two parallel lines */
				Eigen::Vector3d vp = line_1.cross(line_2);
				assert(std::abs(vp[2]) > 1e-10);

				vp /= vp[2];

				/* Sum the distances from the vanishing points to all the lines in the parallel group */
				double sum_dist = 0;
				for (int k = 0; k < group_it->size(); k++)
				{
					Eigen::Vector3d &line = line_equations[group_it->operator[](k)];
					double dist = point_line_distance(line, vp.head(2));
					sum_dist += dist;
				}
				/* Check whether the current vp is the most plausible one */
				if (sum_dist < min_sum_dist)
				{
					min_sum_dist = sum_dist;
					vanishing_point = vp;
				}
			}
		}

		vp_list.push_back(vanishing_point);
	}

	int count = 0;
	for (int i = 0; i < vp_list.size(); i++)
	{
		Eigen::Vector2d vp1 = vp_list[i].head(2);
		for (int j = i + 1; j < vp_list.size(); j++)
		{
			Eigen::Vector2d vp2 = vp_list[j].head(2);

			double mid = -vp1.dot(vp2);
			if (mid >= 0)
			{
				focal_length += std::sqrt(mid);
				count++;
			}
		}
	}

	if (count > 0)
		focal_length /= count;
	else
		focal_length = (width_ + height_) / 2.0;

	primary_point[0] = width_ / 2.0;
	primary_point[1] = height_ / 2.0;
}

Eigen::Vector3d CameraClibrator::get_line_equation(int line_id)
{
	int v1_idx = lines_[line_id].p1();
	int v2_idx = lines_[line_id].p2();
	Eigen::Vector2d &v1 = vertices_[v1_idx];
	Eigen::Vector2d &v2 = vertices_[v2_idx];

	if (std::abs(v1.x() - v2.x()) < 1e-4)
	{
		return Eigen::Vector3d(1, 0, -v1.x());
	}

	double slope = (v2.y() - v1.y()) / (v2.x() - v1.x());
	double a = slope;
	double b = -1;
	double c = -slope * v1.x() + v1.y();
	
	if(std::abs(c) > 1.0e-8)
	{
		a /= c;
		b /= c;
		c = 1.0;
	}

	return Eigen::Vector3d(a, b, c);
}

double CameraClibrator::point_line_distance(const Eigen::Vector3d & line, const Eigen::Vector2d & point)
{
	double dist = abs(line[0] * point[0] + line[1] * point[1] + line[2])
		/ std::sqrt(std::pow(line[0], 2) + std::pow(line[1], 2));
	return dist;
}
