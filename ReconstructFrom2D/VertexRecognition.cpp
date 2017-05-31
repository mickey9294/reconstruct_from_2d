#include "VertexRecognition.h"



VertexRecognition::VertexRecognition()
{
}


VertexRecognition::~VertexRecognition()
{
}

void VertexRecognition::set_image(const QPixmap & image)
{
	image_ = image.toImage().convertToFormat(QImage::Format_Grayscale8);
}

void VertexRecognition::set_sketch_vertices(const std::list<QPointF>& sketch_vertices)
{
	sketch_vertices_.resize(sketch_vertices.size());
	int idx = 0;
	for (std::list<QPointF>::const_iterator it = sketch_vertices.begin();
		it != sketch_vertices.end(); ++it, ++idx)
	{
		sketch_vertices_[idx].setX(it->x());
		sketch_vertices_[idx].setY(it->y());
	}
}

void VertexRecognition::get_precise_vertices(std::vector<int>& precise_vertices_id,
	std::vector<QPointF> &precise_vertices, std::vector<std::vector<QLineF>> &line_segments)
{
	const int patch_size = 50;
	line_segments.clear();
	line_segments.reserve(sketch_vertices_.size());

	for (int i = 0; i < sketch_vertices_.size(); i++)
	{
		QPointF &sketch_vertex = sketch_vertices_[i];
		int x_start, y_start, x_end, y_end;
		if (sketch_vertex.x() > patch_size / 2)
			x_start = std::round(sketch_vertex.x()) - patch_size / 2;
		else
			x_start = 0;
		if (sketch_vertex.y() > patch_size / 2)
			y_start = std::round(sketch_vertex.y()) - patch_size / 2;
		else
			y_start = 0;
		if (sketch_vertex.x() < image_.width() - patch_size / 2)
			x_end = std::round(sketch_vertex.x()) + patch_size / 2;
		else
			x_end = image_.width();
		if (sketch_vertex.y() < image_.height() - patch_size / 2)
			y_end = std::round(sketch_vertex.y()) + patch_size / 2;
		else
			y_end = image_.height();

		
		/* Grab a patch around the sketch vertex */
		image_double patch = new_image_double(x_end - x_start, y_end - y_start);
		for (int x = 0; x + x_start < x_end; x++)
		{
			for (int y = 0; y_start + y < y_end; y++)
			{
				QRgb pixel = image_.pixel(x_start + x, y_start + y);
				int gray = qRed(pixel);
				
				patch->data[x + y * patch_size] = gray;
			}
		}

		ntuple_list ntl = lsd(patch);
		std::vector<QLineF> lines(ntl->size);
		//std::string path = "..\\lines_" + std::to_string(i) + ".csv";
		//std::ofstream out(path.c_str());
		for (int j = 0; j < ntl->size; j++)
		{
			double x1 = ntl->values[0 + j * ntl->dim] + x_start;
			double y1 = ntl->values[1 + j * ntl->dim] + y_start;
			double x2 = ntl->values[2 + j * ntl->dim] + x_start;
			double y2 = ntl->values[3 + j * ntl->dim] + y_start;
			lines[j].setP1(QPointF(x1, y1));
			lines[j].setP2(QPointF(x2, y2));

			//out << lines[j].x1() << "," << lines[j].y1() << "," << lines[j].x2() << "," << lines[j].y2() << std::endl;
		}
		//out.close();

		line_segments.push_back(lines);

		QPointF precise_vertex(0, 0);
		double min_dist = std::numeric_limits<double>::max();
		int count = 0;
		for (int j = 0; j < lines.size(); j++)
		{
			QLineF &line1 = lines[j];
			for (int k = j + 1; k < lines.size(); k++)
			{
				QLineF &line2 = lines[k];
				
				QPointF intersect_point;
				QLineF::IntersectType intersect_type = line1.intersect(line2, &intersect_point);
				if (intersect_type == QLineF::UnboundedIntersection)
				{
					if (min_lines_dist(line1, line2, intersect_point) <= 4.0)
					{
						double dist = distance(intersect_point, sketch_vertex);
						if (dist <= 4.0)
						{
							if(dist < min_dist)
								precise_vertex = intersect_point;
							count++;
						}
					}
				}
				else if (intersect_type == QLineF::BoundedIntersection)
				{
					double dist = distance(intersect_point, sketch_vertex);
					if (dist <= 4.0)
					{
						if(dist < min_dist)
							precise_vertex = intersect_point;
						count++;
					}
				}
			}
		}

		if (count > 0)
		{
			precise_vertices_id.push_back(i);
			precise_vertices.push_back(precise_vertex);
		}
	}

	//image_double patch = new_image_double(image_.width(), image_.height());
	//for (int x = 0; x < image_.width(); x++)
	//{
	//	for (int y = 0; y < image_.height(); y++)
	//	{
	//		QRgb pixel = image_.pixel(x, y);
	//		int gray = qRed(pixel);
	//		patch->data[x + y * image_.width()] = (double)gray;
	//	}
	//}
	//ntuple_list ntl = lsd(patch);
	//std::ofstream out("..\\lines.csv");
	//for (int j = 0; j < ntl->size; j++)
	//{
	//	float x1 = ntl->values[0 + j * ntl->dim];
	//	float y1 = ntl->values[1 + j * ntl->dim];
	//	float x2 = ntl->values[2 + j * ntl->dim];
	//	float y2 = ntl->values[3 + j * ntl->dim];

	//	out << x1 << "," << y1 << "," << x2 << "," << y2 << std::endl;
	//}
	//out.close();
}

bool VertexRecognition::get_precise_vertices(const QPointF & sketch_vertex, QPointF & precise_vertex)
{
	assert(image_.width() > 0);
	const int patch_size = 50;

	int x_start, y_start, x_end, y_end;
	if (sketch_vertex.x() > patch_size / 2)
		x_start = std::round(sketch_vertex.x()) - patch_size / 2;
	else
		x_start = 0;
	if (sketch_vertex.y() > patch_size / 2)
		y_start = std::round(sketch_vertex.y()) - patch_size / 2;
	else
		y_start = 0;
	if (sketch_vertex.x() < image_.width() - patch_size / 2)
		x_end = std::round(sketch_vertex.x()) + patch_size / 2;
	else
		x_end = image_.width();
	if (sketch_vertex.y() < image_.height() - patch_size / 2)
		y_end = std::round(sketch_vertex.y()) + patch_size / 2;
	else
		y_end = image_.height();

	/* Grab a patch around the sketch vertex */
	image_double patch = new_image_double(x_end - x_start, y_end - y_start);
	for (int x = 0; x + x_start < x_end; x++)
	{
		for (int y = 0; y_start + y < y_end; y++)
		{
			QRgb pixel = image_.pixel(x_start + x, y_start + y);
			int gray = qRed(pixel);

			patch->data[x + y * patch_size] = gray;
		}
	}
	/* Detect line segments */
	ntuple_list ntl = lsd(patch);

	/* Extract line segments information */
	std::vector<QLineF> lines(ntl->size);
	for (int j = 0; j < ntl->size; j++)
	{
		double x1 = ntl->values[0 + j * ntl->dim] + x_start;
		double y1 = ntl->values[1 + j * ntl->dim] + y_start;
		double x2 = ntl->values[2 + j * ntl->dim] + x_start;
		double y2 = ntl->values[3 + j * ntl->dim] + y_start;
		lines[j].setP1(QPointF(x1, y1));
		lines[j].setP2(QPointF(x2, y2));
	}

	precise_vertex.setX(0);
	precise_vertex.setY(0);
	double min_dist = std::numeric_limits<double>::max();
	int count = 0;
	for (int j = 0; j < lines.size(); j++)
	{
		QLineF &line1 = lines[j];
		for (int k = j + 1; k < lines.size(); k++)
		{
			QLineF &line2 = lines[k];

			QPointF intersect_point;
			QLineF::IntersectType intersect_type = line1.intersect(line2, &intersect_point);
			if (intersect_type == QLineF::UnboundedIntersection)
			{
				if (min_lines_dist(line1, line2, intersect_point) <= 4.0)
				{
					double dist = distance(intersect_point, sketch_vertex);
					if (dist <= 4.0)
					{
						if (dist < min_dist)
							precise_vertex = intersect_point;
						count++;
					}
				}
			}
			else if (intersect_type == QLineF::BoundedIntersection)
			{
				double dist = distance(intersect_point, sketch_vertex);
				if (dist <= 4.0)
				{
					if (dist < min_dist)
						precise_vertex = intersect_point;
					count++;
				}
			}
		}
	}
	if (count == 0)
	{
		precise_vertex = sketch_vertex;
		return false;
	}
	else
		return true;
}

double VertexRecognition::distance(const QPointF & p1, const QPointF & p2)
{
	double dist = std::sqrt(std::pow(p1.x() - p2.x(), 2) + std::pow(p1.y() - p2.y(), 2));
	return dist;
}

double VertexRecognition::distance(const Eigen::Vector2d & p1, const Eigen::Vector2d & p2)
{
	double dist = std::sqrt(std::pow(p1.x() - p2.x(), 2) + std::pow(p1.y() - p2.y(), 2));
	return dist;
}

double VertexRecognition::min_lines_dist(const QLineF & line1, const QLineF & line2, const QPointF &intersect_point)
{
	Eigen::Vector2d p1(line1.p1().x(), line1.p1().y());
	Eigen::Vector2d p2(line1.p2().x(), line1.p2().y());
	Eigen::Vector2d q1(line2.p1().x(), line2.p1().y());
	Eigen::Vector2d q2(line2.p2().x(), line2.p2().y());

	Eigen::Vector2d inter(intersect_point.x(), intersect_point.y());
	Eigen::Vector2d tp1 = p1 - inter;
	Eigen::Vector2d tp2 = p2 - inter;
	Eigen::Vector2d tq1 = q1 - inter;
	Eigen::Vector2d tq2 = q2 - inter;

	bool on_l1 = tp1.dot(tp2) < 0;
	bool on_l2 = tq1.dot(tq2) < 0;

	double min_dist;
	if (on_l1 && !on_l2)
	{
		min_dist = std::min(tq1.norm(), tq2.norm());
	}
	else if (!on_l1 && on_l2)
		min_dist = std::min(tp1.norm(), tp2.norm());
	else if (!on_l1 && !on_l2)
	{
		min_dist = std::max(std::min(tp1.norm(), tp2.norm()), std::min(tq1.norm(), tq2.norm()));
	}
	return min_dist;
}

Eigen::Vector3d VertexRecognition::line_equation(const QPointF & p1, const QPointF & p2)
{
	Eigen::Vector3d line;
	if (std::abs(p1.x() - p2.x()) < 1e-6)
	{
		line[0] = 1;
		line[1] = 0;
		line[2] = -p1.x();
	}
	else
	{
		float slope = (p2.y() - p1.y()) / (p2.x() - p1.x());
		line[0] = slope;
		line[1] = -1;
		line[2] = -slope * p1.x() + p1.y();

		if (std::abs(line[2]) > 1e-8)
			line /= line[2];
	}

	return line;
}

Eigen::Vector3d VertexRecognition::line_equation(const QLineF & line)
{
	Eigen::Vector3d line_equ;
	QPointF p1 = line.p1();
	QPointF p2 = line.p2();
	if (std::abs(p1.x() - p2.x()) < 1e-6)
	{
		if (std::abs(p1.x() > 1e-6))
		{
			line_equ[0] = -1.0 / p1.x();
			line_equ[1] = 0;
			line_equ[2] = 1.0;
		}
		else
		{
			line_equ[0] = 1;
			line_equ[1] = 0;
			line_equ[2] = -p1.x();
		}
	}
	else
	{
		float slope = (p2.y() - p1.y()) / (p2.x() - p1.x());
		line_equ[0] = slope;
		line_equ[1] = -1;
		line_equ[2] = -slope * p1.x() + p1.y();

		if (std::abs(line_equ[2]) > 1e-8)
			line_equ /= line_equ[2];
	}

	return line_equ;
}


