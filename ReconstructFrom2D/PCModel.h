#ifndef PCMODEL_H
#define PCMODEL_H

#include <QObject>
#include <QtOpenGL>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <cmath>
#include <fstream>
#include <iostream>

#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/RefBase.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/TArrayRef.h>

#include "ShapeModel.h"

#define PI 3.14159265359

namespace asdk {
	using namespace artec::sdk::base;
	//using namespace artec::sdk::capturing;
	//using namespace artec::sdk::scanning;
	//using namespace artec::sdk::algorithms;
};

class PCModel : public QObject, public ShapeModel
{
	Q_OBJECT

public:
	PCModel(QObject *parent = NULL);
	PCModel(const PCModel &pc);
	PCModel(const char *file_path);
	~PCModel();

	PCModel & operator=(const PCModel &pc);

	int num_vertices() const { return vertices_.size(); }
	std::string get_input_filepath() const;
	Eigen::Vector3d get_vertex(int index);
	Eigen::Vector3d get_vertex_normal(int index);
	void output(const char *file_path);
	Eigen::Vector3d get_centroid() const;
	double get_radius() const { return radius_; }
	Eigen::Vector3d & operator[](int index);

	std::vector<Eigen::Vector3d>::const_iterator vertices_const_begin() const { return vertices_.begin(); }
	std::vector<Eigen::Vector3d>::const_iterator normals_const_begin() const { return normals_.begin(); }
	std::vector<Eigen::Vector3d>::const_iterator vertices_const_end() const { return vertices_.end(); }
	std::vector<Eigen::Vector3d>::const_iterator normals_const_end() const { return normals_.end(); }

	void set_input_filepath(const char *input_filepath);
	void set_input_filepath(std::string input_filepath);

	void add_frame(const asdk::IFrameMesh *mesh, Eigen::Matrix4d);

	void normalize();
	void estimate_normals();
	void draw(float scale);
	void rotate(float angle, float x, float y, float z);
	void transform(Eigen::Matrix4d mat);

private:
	double radius_;
	Eigen::Vector3d centroid_;
	std::string input_filepath_;

	std::vector<Eigen::Vector3d> vertices_;
	std::vector<Eigen::Vector3d> normals_;
	std::vector<Eigen::Vector3i> colors_;

	void load_from_file(const char *file_path);
	void normalize_to_centre();
	void normalize_to_barycenter();
	void compute_bounding_sphere(double &radius, Eigen::Vector3d &center);
};

#endif // !PCMODEL_H