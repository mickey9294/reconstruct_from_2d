#pragma once

#include <QObject>

#include "ShapeModel.h"
#include "PlanarFace.h"

class MeshModel : public QObject, public ShapeModel
{
	Q_OBJECT

public:
	MeshModel(QObject *parent = 0);
	MeshModel(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3i> &triangles,
		const std::vector<std::vector<int>> &faces);
	MeshModel(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3i> &triangles,
		const std::vector<PlanarFace> &faces);
	MeshModel(const MeshModel &mesh);
	~MeshModel();

	MeshModel & operator=(const MeshModel &mesh);
	
	int num_vertices() const { return vertices_.size(); }
	//std::string get_input_filepath() const;
	Eigen::Vector3d get_vertex(int index);
	//Eigen::Vector3d get_vertex_normal(int index);
	Eigen::Vector3d get_centroid() const;
	double get_radius() const { return radius_; }
	Eigen::Vector3d & operator[](int index);

	void output(const char *file_path);
	void normalize();
	void draw(float scale);
	void rotate(float angle, float x, float y, float z);
	void transform(Eigen::Matrix4d mat);

	const std::vector<Eigen::Vector3d> &const_vertices() const;
	const std::vector<Eigen::Vector3i> &const_triangles() const;
	const std::vector<std::vector<int>> &const_faces() const;
	const std::vector<Eigen::Vector3d> &const_normals() const;

private:
	double radius_;
	Eigen::Vector3d centroid_;
	//std::string input_filepath_;

	std::vector<Eigen::Vector3d> vertices_;
	std::vector<Eigen::Vector3i> triangles_;
	std::vector<std::vector<int>> faces_;
	std::vector<Eigen::Vector3d> normals_;

	void normalize_to_centre();
	void compute_bounding_sphere(double &radius, Eigen::Vector3d &center);
};
