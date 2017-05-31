#include "VerticesConnector.h"

VerticesConnector::VerticesConnector()
{
}

VerticesConnector::VerticesConnector(const QPixmap & image, const std::list<QPointF>& vertices)
{
	image_ = image.toImage().convertToFormat(QImage::Format_Grayscale8);

	vertices_.resize(vertices.size());
	int idx = 0;
	for (std::list<QPointF>::const_iterator it = vertices.begin();
		it != vertices.end(); ++it, ++idx)
	{
		vertices_[idx].setX(it->x());
		vertices_[idx].setY(it->y());
	}

	count_ = 0;
}


VerticesConnector::~VerticesConnector()
{
}

void VerticesConnector::set_image(const QPixmap & image)
{
	image_ = image.toImage().convertToFormat(QImage::Format_Grayscale8);
}

void VerticesConnector::get_connection_lines(std::list<Line>& connection_lines)
{
	for (int i = 0; i < vertices_.size(); i++)
	{
		for (int j = i + 1; j < vertices_.size(); j++)
		{
			bool connected = is_connected(vertices_[i], vertices_[j]);
			if (connected)
				connection_lines.push_back(Line(i, j));
		}
	}
}

bool VerticesConnector::is_connected(const QPointF & v1, const QPointF & v2)
{
	QPointF vleft, vright;
	if (v1.x() < v2.x())
	{
		vleft = v1;
		vright = v2;
	}
	else
	{
		vleft = v2;
		vright = v1;
	}
		
	QLineF true_connection(vleft, vright);
	Eigen::Vector2d true_direction(vright.x() - vleft.x(), vright.y() - vleft.y());
	true_direction.normalize();

	int x_start = std::min(std::round(v1.x()), std::round(v2.x()));
	int x_end = std::max(std::round(v1.x()), std::round(v2.x()));
	int y_start = std::min(std::round(v1.y()), std::round(v2.y()));
	int y_end = std::max(std::round(v1.y()), std::round(v2.y()));

	if (x_start - 10 >= 0)
		x_start -= 10;
	else
		x_start = 0;
	if (x_end + 10 < image_.width())
		x_end += 10;
	else
		x_end = image_.width();
	if (y_start - 10 >= 0)
		y_start -= 10;
	else
		y_start = 0;
	if (y_end + 10 < image_.height())
		y_end += 10;
	else
		y_end = image_.height();

	/* Grab a patch around the sketch vertex */
	int width = x_end - x_start;
	int height = y_end - y_start;
	image_double patch = new_image_double(x_end - x_start, y_end - y_start);
	for (int x = 0; x + x_start < x_end; x++)
	{
		for (int y = 0; y_start + y < y_end; y++)
		{
			QRgb pixel = image_.pixel(x_start + x, y_start + y);
			int gray = qRed(pixel);

			patch->data[x + y * width] = gray;
		}
	}
	/* Detect line segments */
	ntuple_list ntl = lsd(patch);

	
	//std::vector<QLineF> lines(ntl->size);
	std::list<Interval> continuous_lines;
	Eigen::Vector2d vert1(vleft.x(), vleft.y());
	Eigen::Vector2d vert2(vright.x(), vright.y());
	double proj_range = (vert2 - vert1).norm();
	//std::string out_path = "..\\lines_" + std::to_string(count_++) + ".csv";
	//std::ofstream out(out_path.c_str());
	for (int j = 0; j < ntl->size; j++)
	{
		/* Extract line segments information */
		double x1 = ntl->values[0 + j * ntl->dim] + x_start;
		double y1 = ntl->values[1 + j * ntl->dim] + y_start;
		double x2 = ntl->values[2 + j * ntl->dim] + x_start;
		double y2 = ntl->values[3 + j * ntl->dim] + y_start;

		//out << x1 << "," << y1 << "," << x2 << "," << y2 << std::endl;

		QLineF line;
		line.setP1(QPointF(x1, y1));
		line.setP2(QPointF(x2, y2));

		double angle = true_connection.angleTo(line);
		double cos_angle = std::cos(angle * M_PI / 180.0);
		if (cos_angle >=0.9976 && cos_angle <= 1.0 || cos_angle <= -0.9976 && cos_angle >= -1.0)  /* if two lines are parallel */
		{
			/* Compute distance between true connection line and current line segment */
			Eigen::Vector2d p1(x1, y1);
			Eigen::Vector2d vec = (vert1 - p1) - (vert1 - p1).dot(true_direction) * true_direction;
			double dist = vec.norm();

			if (dist < 3.0)
			{
				Eigen::Vector2d p2(x2, y2);

				double proj1 = (p1 - vert1).dot(true_direction);
				double proj2 = (p2 - vert1).dot(true_direction);
				if (proj1 < 0)
					proj1 = 0;
				if (proj2 < 0)
					proj2 = 0;
				if (proj1 > proj_range)
					proj1 = proj_range;
				if (proj2 > proj_range)
					proj2 = proj_range;
				if (proj1 > proj2)
					std::swap(proj1, proj2);

				insert_interval(continuous_lines, Interval(proj1, proj2));
			}
		}
	}

	//out.close();

	double coverage = 0;
	for (std::list<Interval>::iterator it = continuous_lines.begin(); it != continuous_lines.end(); ++it)
	{
		coverage += std::abs(it->second - it->first);
	}
	coverage /= proj_range;

	if (coverage > 0.6)
		return true;
	else
		return false;
}

void VerticesConnector::insert_interval(std::list<Interval>& intervals, Interval new_interval)
{
	std::list<Interval> result;
	int i = 0;

	/* Add all the intervals ending before new_interval starts */
	std::list<Interval>::iterator it = intervals.begin();
	for (; it != intervals.end(); ++it)
	{
		if (new_interval.first <= it->second)
			break;
		result.push_back(*it);
	}

	for (; it != intervals.end(); ++it)
	{
		if (it->first > new_interval.second)
			break;

		new_interval = Interval(std::min(new_interval.first, it->first),
			std::max(new_interval.second, it->second));
	}
	result.push_back(new_interval);

	/* add all the rest */
	for (; it != intervals.end(); ++it)
	{
		result.push_back(*it);
	}

	intervals = result;
}
