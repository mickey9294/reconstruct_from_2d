#include "PCModel.h"

PCModel::PCModel(QObject *parent)
	: QObject(parent), ShapeModel(ShapeModel::ModelType::PointCloud)
{
	centroid_.setZero();
}

PCModel::PCModel(const PCModel &pc)
	: QObject(pc.parent()), ShapeModel(ShapeModel::ModelType::PointCloud)
{
	vertices_.reserve(pc.num_vertices());
	normals_.reserve(pc.num_vertices());
	for (std::vector<Eigen::Vector3d>::const_iterator vert_it = pc.vertices_const_begin();
		vert_it != pc.vertices_const_end(); ++vert_it)
		vertices_.push_back(*vert_it);
	
	for (std::vector<Eigen::Vector3d>::const_iterator norm_it = pc.normals_const_begin();
		norm_it != pc.vertices_const_end(); ++norm_it)
		normals_.push_back(*norm_it);

	radius_ = pc.get_radius();
	centroid_ = pc.get_centroid();
	input_filepath_ = pc.get_input_filepath();
}

PCModel::PCModel(const char *file_path)
{
	input_filepath_ = std::string(file_path);

	load_from_file(file_path);

	normalize();

	if (normals_.empty())
		estimate_normals();
}

PCModel::~PCModel()
{
	
}

PCModel & PCModel::operator=(const PCModel & pc)
{
	vertices_.reserve(pc.num_vertices());
	normals_.reserve(pc.num_vertices());
	for (std::vector<Eigen::Vector3d>::const_iterator vert_it = pc.vertices_const_begin();
		vert_it != pc.vertices_const_end(); ++vert_it)
		vertices_.push_back(*vert_it);

	for (std::vector<Eigen::Vector3d>::const_iterator norm_it = pc.normals_const_begin();
		norm_it != pc.vertices_const_end(); ++norm_it)
		normals_.push_back(*norm_it);

	radius_ = pc.get_radius();
	centroid_ = pc.get_centroid();
	input_filepath_ = pc.get_input_filepath();

	return *this;
}

Eigen::Vector3d PCModel::get_centroid() const
{
	return centroid_;
}

void PCModel::load_from_file(const char *file_path)
{
	boost::filesystem::path shape_path(file_path);
	std::string shape_name = shape_path.stem().string();

	std::ifstream pc_in(shape_path.string());
	if (pc_in.is_open())
	{
		std::string format = boost::filesystem::extension(shape_path);
		if (format.compare(".off") == 0 || format.compare(".OFF") == 0)
		{
			std::string line;
			
			std::getline(pc_in, line);
			if (line.compare("OFF") == 0)
			{
				std::getline(pc_in, line);

				std::vector<std::string> line_split;
				boost::split(line_split, line, boost::is_any_of(" "), boost::token_compress_on);

				int nvertices = std::stoi(line_split[0]);

				vertices_.reserve(nvertices);
				//normals_.reserve(nvertices);

				for (int i = 0; i < nvertices; i++)
				{
					std::getline(pc_in, line);
					boost::split(line_split, line, boost::is_any_of(" "), boost::token_compress_on);

					double x = std::stod(line_split[0]);
					double y = std::stod(line_split[1]);
					double z = std::stod(line_split[2]);

					vertices_.push_back(Eigen::Vector3d(x, y, z));
					//normals_.push_back(Eigen::Vector3d(0, 0, 1));
				}
			}
		}
		else if (format.compare(".ply") == 0 || format.compare(".PLY") == 0)
		{
			std::string line;

			std::getline(pc_in, line);
			if (line.compare("ply") == 0 || line.compare("PLY") == 0)
			{
				int nvertices;
				bool read_normals = false;
				bool read_colors = false;
				std::vector<std::string> line_split;

				std::getline(pc_in, line);
				while (line.compare("end_header") != 0)
				{
					if (line.length() > 14 && line.substr(0, 14).compare("element vertex") == 0)
					{
						boost::split(line_split, line, boost::is_any_of(" "), boost::token_compress_on);
						nvertices = std::stoi(line_split[2]);
						vertices_.reserve(nvertices);
						normals_.reserve(nvertices);
						colors_.reserve(nvertices);
					}
					else if (line.compare("property float nx") == 0)
					{
						read_normals = true;
					}
					else if (line.compare("property uchar red") == 0)
						read_colors = true;
					std::getline(pc_in, line);
				}

				for (int i = 0; i < nvertices; i++)
				{
					std::getline(pc_in, line);
					boost::split(line_split, line, boost::is_any_of(" "), boost::token_compress_on);

					Eigen::Vector3d vertex, normal;

					for (int j = 0; j < 3; j++)
						vertex[j] = std::stod(line_split[j]);
					vertices_.push_back(vertex);

					if (line_split.size() >= 6 && read_normals)
					{
						for (int j = 0; j < 3; j++)
							normal[j] = std::stod(line_split[3 + j]);
						normals_.push_back(normal);
					}
					else if (line_split.size() >= 6 && read_colors)
					{
						Eigen::Vector3i color;
						for (int j = 0; j < 3; j++)
							color[j] = std::stoi(line_split[3 + j]);
						colors_.push_back(color);
					}
					//else
						//normals_.push_back(Eigen::Vector3d(0, 0, 1));
				}
			}
		}

		pc_in.close();
	}
}

void PCModel::output(const char *filename)
{
	boost::filesystem::path out_path(filename);

	std::ofstream out(filename);
	if (out.is_open())
	{
		std::string file_format = boost::filesystem::extension(out_path);

		if (file_format.compare(".ply") == 0 || file_format.compare(".PLY") == 0)
		{
			/* Count the number of actual points with high confidence */
			int nvertices = num_vertices();

			/* Write the header */
			out << "ply" << std::endl;
			out << "format ascii 1.0" << std::endl;
			std::string pointsNum = std::to_string(nvertices);
			std::string elementVertex = "element vertex " + pointsNum;
			out << elementVertex << std::endl;
			out << "property float x" << std::endl;
			out << "property float y" << std::endl;
			out << "property float z" << std::endl;
			out << "property float nx" << std::endl;
			out << "property float ny" << std::endl;
			out << "property float nz" << std::endl;
			out << "end_header" << std::endl;

			/* Write the vertices and normals */
			for (int i = 0; i < nvertices; i++)
			{
				Eigen::Vector3d &vertex = vertices_[i];
				Eigen::Vector3d &normal = normals_[i];

				out << vertex.x() << " " << vertex.y() << " " << vertex.z() << " "
					<< normal.x() << " " << normal.y() << " " << normal.z() << std::endl;
			}

		}
		else if (file_format.compare("off") == 0 || file_format.compare("OFF") == 0)
		{
			/* Write the header */
			out << "OFF" << endl;
			out << num_vertices() << " 0 0" << endl;

			/* Write the vertices */
			for (std::vector<Eigen::Vector3d>::iterator vertex_it = vertices_.begin(); vertex_it != vertices_.end(); ++vertex_it)
			{
				out << vertex_it->x() << " " << vertex_it->y() << " " << vertex_it->z() << endl;
			}
		}
		else if (file_format.compare("pts") == 0 || file_format.compare("PTS") == 0)
		{
			for (std::vector<Eigen::Vector3d>::iterator vertex_it = vertices_.begin(); vertex_it != vertices_.end(); ++vertex_it)
			{
				out << "0 0.333 0.333 0.333 " <<
					vertex_it->x() << " " << vertex_it->y() << " " << vertex_it->z() << std::endl;
			}
		}

		out.close();
	}
}

void PCModel::normalize()
{
	normalize_to_barycenter();
}

void PCModel::normalize_to_centre()
{
	compute_bounding_sphere(radius_, centroid_);

	for (std::vector<Eigen::Vector3d>::iterator vertex_it = vertices_.begin(); vertex_it != vertices_.end(); ++vertex_it)
	{
		vertex_it->operator-=(centroid_);
		vertex_it->operator/=(2 * radius_);
	}

	centroid_.setZero();
	radius_ = 0.5;
}

void PCModel::normalize_to_barycenter()
{
	const int dimen = 3;
	PointVector S;
	std::vector<double> coords(dimen);

	Eigen::Vector3d center;
	center.setZero();

	for (std::vector<Eigen::Vector3d>::iterator vertex_it = vertices_.begin(); vertex_it != vertices_.end(); ++vertex_it)
	{
		coords[0] = vertex_it->x();
		coords[1] = vertex_it->y();
		coords[2] = vertex_it->z();

		S.push_back(MiniPoint(dimen, coords.begin()));
		
		center += *vertex_it;
	}

	if (num_vertices() > 0)
		center /= num_vertices();

	for (std::vector<Eigen::Vector3d>::iterator vertex_it = vertices_.begin(); vertex_it != vertices_.end(); ++vertex_it)
		vertex_it->operator-=(center);

	/* Compute the Miniball of the mesh */
	Miniball mb(dimen, S);
	radius_ = mb.radius();

	centroid_.setZero();
}

void PCModel::compute_bounding_sphere(double &radius, Eigen::Vector3d &center)
{
	const int dimen = 3;
	PointVector S;
	std::vector<double> coords(dimen);

	for (std::vector<Eigen::Vector3d>::iterator vertex_it = vertices_.begin(); vertex_it != vertices_.end(); ++vertex_it)
	{
		coords[0] = vertex_it->x();
		coords[1] = vertex_it->y();
		coords[2] = vertex_it->z();

		S.push_back(MiniPoint(dimen, coords.begin()));
	}

	/* Compute the Miniball of the mesh */
	Miniball mb(dimen, S);
	radius = mb.radius();
	Miniball::Coordinate_iterator center_it = mb.center_begin();
	center[0] = center_it[0];
	center[1] = center_it[1];
	center[2] = center_it[2];
}

void PCModel::set_input_filepath(const char *input_filepath)
{
	input_filepath_ = std::string(input_filepath);
}

void PCModel::set_input_filepath(std::string input_filepath)
{
	input_filepath_ = input_filepath;
}

void PCModel::add_frame(const asdk::IFrameMesh * mesh, Eigen::Matrix4d trans)
{
	using namespace asdk;

	double xmean = centroid_.x() * num_vertices();
	double ymean = centroid_.y() * num_vertices();
	double zmean = centroid_.z() * num_vertices();

	/* get points */
	IArrayPoint3F * modelPoints = mesh->getPoints();
	int pointsNum = modelPoints->getSize();
	Point3F * points = modelPoints->getPointer();
	/* get normal vectors of the points */
	TArrayPoint3F normals = mesh->getPointsNormals();

	std::vector<Eigen::Vector3d> new_point_cluster;
	std::vector<Eigen::Vector3d> new_norm_cluster;
	new_point_cluster.reserve(pointsNum);
	new_norm_cluster.reserve(pointsNum);

	std::vector<Eigen::Vector3d> temp_points;
	temp_points.reserve(pointsNum);

	for (int i = 0; i < pointsNum; i++)
	{
		Point3F point = points[i];
		
		Eigen::Vector4d v(point.x, point.y, point.z, 1.0);
		v = trans * v;

		temp_points.push_back(v.head(3));

		xmean += point.x;
		ymean += point.y;
		zmean += point.z;
	}

	vertices_.insert(vertices_.end(), temp_points.begin(), temp_points.end());

	centroid_[0] = xmean / (double)num_vertices();
	centroid_[1] = ymean / (double)num_vertices();
	centroid_[2] = zmean / (double)num_vertices();
}

std::string PCModel::get_input_filepath() const
{
	return input_filepath_;
}

void PCModel::rotate(float angle, float x, float y, float z)
{
	using namespace Eigen;

	float _angle = angle / 180.0 * PI;
	AngleAxis<double> rotation(_angle, Vector3d(x, y, z));

	for (std::vector<Vector3d>::iterator vertex_it = vertices_.begin(); vertex_it != vertices_.end(); ++vertex_it)
		*vertex_it = rotation * (*vertex_it);

	for (std::vector<Vector3d>::iterator norm_it = normals_.begin(); norm_it != normals_.end(); ++norm_it)
		*norm_it = rotation * (*norm_it);

	centroid_ = rotation * centroid_;
}

void PCModel::transform(Eigen::Matrix4d mat)
{
	using namespace Eigen;

	for (std::vector<Vector3d>::iterator vertex_it = vertices_.begin(); vertex_it != vertices_.end(); ++vertex_it)
	{
		Vector4d vert_aug;
		vert_aug.head(3) = *vertex_it;
		vert_aug[3] = 1.0;

		vert_aug = mat * vert_aug;
		vertex_it->head(3) = vert_aug.head(3);
	}

	for (std::vector<Vector3d>::iterator norm_it = normals_.begin(); norm_it != normals_.end(); ++norm_it)
	{
		Vector4d norm_aug = Vector4d::Zero();
		norm_aug.head(3) = *norm_it;

		norm_aug = mat * norm_aug;
		norm_it->head(3) = norm_aug.head(3);
	}

	Vector4d center_aug;
	center_aug.head(3) = centroid_;
	center_aug[3] = 1.0;
	center_aug = mat * center_aug;
	centroid_.head(3) = center_aug.head(3);
}

Eigen::Vector3d & PCModel::operator[](int index)
{
	return vertices_[index];
}

Eigen::Vector3d PCModel::get_vertex(int index)
{
	return vertices_.at(index);
}

Eigen::Vector3d PCModel::get_vertex_normal(int index)
{
	return normals_.at(index);
}

void PCModel::estimate_normals()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cloud->width = num_vertices();
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);
	int i = 0;
	for (std::vector<Eigen::Vector3d>::iterator it = vertices_.begin(); it != vertices_.end(); ++it, ++i)
	{
		cloud->points[i].x = it->x();
		cloud->points[i].y = it->y();
		cloud->points[i].z = it->z();
	}

	/* Create the normal estimation class, and pass the input dataset to it */
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	/* Create an empty kdtree representation, and pass it to the normal estimation object.
	Its content will be filled inside the object, based on the given input dataset (as no other search surface is given). */
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);    /* The output normals container */

	/* Use all neighbors in a sphere of 0.1 radius */
	ne.setRadiusSearch(0.1);
	/* Compute the normals (as well as curvatures, but not used here) */
	ne.compute(*cloud_normals);

	normals_.resize(num_vertices());
	int normal_idx = 0;
	for (pcl::PointCloud<pcl::Normal>::iterator norm_it = cloud_normals->begin(); 
		norm_it != cloud_normals->end(); ++norm_it, ++normal_idx)
	{
		normals_[normal_idx][0] = norm_it->normal_x;
		normals_[normal_idx][1] = norm_it->normal_y;
		normals_[normal_idx][2] = norm_it->normal_z;
	}
}

void PCModel::draw(float scale)
{
	assert(vertices_.size() == normals_.size());

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glPointSize(2.5);
	glBegin(GL_POINTS);

	std::vector<Eigen::Vector3d>::iterator vertex_it, normal_it;
	int idx = 0;
	for (vertex_it = vertices_.begin(), normal_it = normals_.begin();
		vertex_it != vertices_.end() && normal_it != normals_.end(); ++vertex_it, ++normal_it, ++idx)
	{
		float transp = 1.0;

		if (idx < colors_.size())
			glColor4f(colors_[idx][0], colors_[idx][1], colors_[idx][2], 1.0);
		else
			glColor4f(COLORS[10][0], COLORS[10][1], COLORS[10][2], transp);
		glNormal3f(normal_it->x(), normal_it->y(), normal_it->z());
		glVertex3f(scale * vertex_it->x(), scale * vertex_it->y(), scale * vertex_it->z());
	}

	glEnd();
	glDisable(GL_BLEND);
}