#include "MeshModel.h"

MeshModel::MeshModel(QObject *parent)
	: QObject(parent),
	ShapeModel(ShapeModel::ModelType::Mesh)
{
	centroid_.setZero();
	radius_ = 0;
}

MeshModel::MeshModel(const std::vector<Eigen::Vector3d>& vertices, 
	const std::vector<Eigen::Vector3i>& triangles, const std::vector<std::vector<int>>& faces)
{
	vertices_.resize(vertices.size());
	for (int i = 0; i < vertices.size(); i++)
	{
		vertices_[i][0] = vertices[i][0];
		vertices_[i][1] = vertices[i][1];
		vertices_[i][2] = vertices[i][2];
	}

	triangles_.resize(triangles.size());
	normals_.resize(triangles.size());
	for (int i = 0;i < triangles.size(); i++)
	{
		triangles_[i][0] = triangles[i][0];
		triangles_[i][1] = triangles[i][1];
		triangles_[i][2] = triangles[i][2];

		Eigen::Vector3d vec1 = vertices_[triangles_[i][0]] - vertices_[triangles_[i][1]];
		Eigen::Vector3d vec2 = vertices_[triangles_[i][0]] - vertices_[triangles_[i][2]];
		Eigen::Vector3d norm = vec1.cross(vec2);
		normals_[i][0] = norm[0];
		normals_[i][1] = norm[1];
		normals_[i][2] = norm[2];
	}
	
	faces_.resize(faces.size());
	for (int i = 0; i < faces.size(); i++)
	{
		faces_[i].resize(faces[i].size());
		for (int j = 0; j < faces[i].size(); j++)
			faces_[i][j] = faces[i][j];
	}

	normalize();
}

MeshModel::MeshModel(const std::vector<Eigen::Vector3d>& vertices,
	const std::vector<Eigen::Vector3i>& triangles, const std::vector<PlanarFace>& faces)
{
	vertices_.resize(vertices.size());
	for (int i = 0; i < vertices.size(); i++)
	{
		vertices_[i][0] = vertices[i][0];
		vertices_[i][1] = vertices[i][1];
		vertices_[i][2] = vertices[i][2];
	}

	triangles_.resize(triangles.size());
	normals_.resize(triangles.size());
	for (int i = 0;i < triangles.size(); i++)
	{
		triangles_[i][0] = triangles[i][0];
		triangles_[i][1] = triangles[i][1];
		triangles_[i][2] = triangles[i][2];

		Eigen::Vector3d vec1 = vertices_[triangles_[i][0]] - vertices_[triangles_[i][1]];
		Eigen::Vector3d vec2 = vertices_[triangles_[i][0]] - vertices_[triangles_[i][2]];
		Eigen::Vector3d norm = vec1.cross(vec2);
		normals_[i][0] = norm[0];
		normals_[i][1] = norm[1];
		normals_[i][2] = norm[2];
	}

	faces_.resize(faces.size());
	for (int i = 0; i < faces.size(); i++)
	{
		const std::vector<int> circuit = faces[i].const_circuit();
		faces_[i].resize(circuit.size());
		for (int j = 0; j < circuit.size(); j++)
			faces_[i][j] = circuit[j];
	}

	normalize();
}

MeshModel::MeshModel(const MeshModel & mesh)
{
	const std::vector<Eigen::Vector3d> &verts = mesh.const_vertices();
	vertices_.resize(verts.size());
	for (int i = 0; i < verts.size(); i++)
	{
		vertices_[i][0] = verts[i][0];
		vertices_[i][1] = verts[i][1];
		vertices_[i][2] = verts[i][2];
	}

	const std::vector<Eigen::Vector3i> &tris = mesh.const_triangles();
	triangles_.resize(tris.size());
	for (int i = 0; i < tris.size(); i++)
	{
		triangles_[i][0] = tris[i][0];
		triangles_[i][1] = tris[i][1];
		triangles_[i][2] = tris[i][2];
	}

	const std::vector<std::vector<int>> &faces = mesh.const_faces();
	faces_.resize(faces.size());
	for (int i = 0; i < faces.size(); i++)
	{
		faces_[i].resize(faces[i].size());
		for (int j = 0; j < faces[i].size(); j++)
			faces_[i][j] = faces[i][j];
	}

	const std::vector<Eigen::Vector3d> &norms = mesh.const_normals();
	normals_.resize(norms.size());
	for (int i = 0; i < norms.size(); i++)
	{
		normals_[i][0] = norms[i][0];
		normals_[i][1] = norms[i][1];
		normals_[i][2] = norms[i][2];
	}

	centroid_ = mesh.get_centroid();
	radius_ = mesh.get_radius();
}

MeshModel::~MeshModel()
{
}

MeshModel & MeshModel::operator=(const MeshModel & mesh)
{
	const std::vector<Eigen::Vector3d> &verts = mesh.const_vertices();
	vertices_.resize(verts.size());
	for (int i = 0; i < verts.size(); i++)
	{
		vertices_[i][0] = verts[i][0];
		vertices_[i][1] = verts[i][1];
		vertices_[i][2] = verts[i][2];
	}

	const std::vector<Eigen::Vector3i> &tris = mesh.const_triangles();
	triangles_.resize(tris.size());
	for (int i = 0; i < tris.size(); i++)
	{
		triangles_[i][0] = tris[i][0];
		triangles_[i][1] = tris[i][1];
		triangles_[i][2] = tris[i][2];
	}

	const std::vector<std::vector<int>> &faces = mesh.const_faces();
	faces_.resize(faces.size());
	for (int i = 0; i < faces.size(); i++)
	{
		faces_[i].resize(faces[i].size());
		for (int j = 0; j < faces[i].size(); j++)
			faces_[i][j] = faces[i][j];
	}

	const std::vector<Eigen::Vector3d> &norms = mesh.const_normals();
	normals_.resize(norms.size());
	for (int i = 0; i < norms.size(); i++)
	{
		normals_[i][0] = norms[i][0];
		normals_[i][1] = norms[i][1];
		normals_[i][2] = norms[i][2];
	}

	centroid_ = mesh.get_centroid();
	radius_ = mesh.get_radius();

	return *this;
}

Eigen::Vector3d MeshModel::get_vertex(int index)
{
	return vertices_[index];
}

Eigen::Vector3d MeshModel::get_centroid() const
{
	return centroid_;
}

Eigen::Vector3d & MeshModel::operator[](int index)
{
	return vertices_[index];
}

void MeshModel::output(const char * file_path)
{
	boost::filesystem::path out_path(file_path);

	std::ofstream out(file_path);
	if (out.is_open())
	{
		std::string file_format = boost::filesystem::extension(out_path);
		if (file_format.compare(".ply") == 0 || file_format.compare(".PLY") == 0)
		{
			int nvertices = num_vertices();

			out << "ply" << std::endl;
			out << "format ascii 1.0" << std::endl;
			std::string elementVertex = "element vertex " + std::to_string(nvertices);
			out << elementVertex << std::endl;
			out << "property float x" << std::endl;
			out << "property float y" << std::endl;
			out << "property float z" << std::endl;
			std::string elementFace = "element face " + std::to_string(triangles_.size());
			out << elementFace << std::endl;
			out << "property list uchar int vertex_indices" << std::endl;
			out << "end_header" << std::endl;

			for (int i = 0; i < nvertices; i++)
			{
				Eigen::Vector3d &vertex = vertices_[i];
				out << vertex.x() << " " << vertex.y() << " " << vertex.z() << std::endl;
			}

			for (int i = 0; i < triangles_.size(); i++)
			{
				Eigen::Vector3i &tri = triangles_[i];
				out << "3 " << tri[0] << " " << tri[1] << " " << tri[2] << std::endl;
			}
		}
		out.close();
	}
}

void MeshModel::normalize()
{
	normalize_to_centre();
}

void MeshModel::draw(float scale)
{
}

void MeshModel::rotate(float angle, float x, float y, float z)
{
	using namespace Eigen;

	float _angle = angle / 180.0 * M_PI;
	AngleAxis<double> rotation(_angle, Vector3d(x, y, z));

	for (std::vector<Vector3d>::iterator vertex_it = vertices_.begin(); vertex_it != vertices_.end(); ++vertex_it)
		*vertex_it = rotation * (*vertex_it);

	for (std::vector<Vector3d>::iterator norm_it = normals_.begin(); norm_it != normals_.end(); ++norm_it)
		*norm_it = rotation * (*norm_it);

	centroid_ = rotation * centroid_;
}

void MeshModel::transform(Eigen::Matrix4d mat)
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

const std::vector<Eigen::Vector3d>& MeshModel::const_vertices() const
{
	return vertices_;
}

const std::vector<Eigen::Vector3i>& MeshModel::const_triangles() const
{
	return triangles_;
}

const std::vector<std::vector<int>>& MeshModel::const_faces() const
{
	return faces_;
}

const std::vector<Eigen::Vector3d>& MeshModel::const_normals() const
{
	return normals_;
}

void MeshModel::normalize_to_centre()
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

void MeshModel::compute_bounding_sphere(double & radius, Eigen::Vector3d & center)
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
