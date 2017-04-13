#ifndef SHAPEMODEL_H
#define SHAPEMODEL_H

#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <assert.h>
#include <boost\filesystem.hpp>
#include <boost\algorithm\string.hpp>
#include <memory>
#include "Seb.h"

typedef Seb::Point<float> MiniPoint;
typedef std::vector<MiniPoint> PointVector;
typedef Seb::Smallest_enclosing_ball<float> Miniball;

/* Define 11 different colors */
const float COLORS[11][3] = {
	{ 1.0, 0.0, 0.0 },    /* 红色 */
	{ 0.0, 1.0, 0.0 },    /* 绿色 */
	{ 0.0, 0.0, 1.0 },    /* 蓝色 */
	{ 1.0, 1.0, 0.0 },    /* 黄色 */
	{ 0.0, 1.0, 1.0 },    /* 天蓝色 */
	{ 1.0, 0.0, 1.0 },    /* 淡紫色*/
	{ 0.5, 0.0, 0.5 },    /* 紫色 */
	{ 1.0, 0.5, 0.25 },   /* 橘黄色 */
	{ 0.5, 0.5, 0.0 },
	{ 0.0, 0.5, 0.5 },
	{ 0.5, 0.5, 0.5 }     /* 灰色 */
};

class ShapeModel
{
public:
	enum ModelType {
		Mesh,
		PointCloud
	};

	ShapeModel();
	ShapeModel(ModelType type);
	virtual ~ShapeModel();
	
	ModelType get_type();
	
	virtual std::string get_input_filepath() const = 0;
	virtual Eigen::Vector3d get_vertex(int index) = 0;
	virtual Eigen::Vector3d & operator[](int index) = 0;
	virtual Eigen::Vector3d get_vertex_normal(int index) = 0;
	virtual int num_vertices() const = 0;
	virtual Eigen::Vector3d get_centroid() const = 0;
	virtual double get_radius() const = 0;

	virtual void output(const char *file_path) = 0;
	virtual void normalize() = 0;
	virtual void draw(float scale) = 0;
	virtual void rotate(float angle, float x, float y, float z) = 0;
	virtual void transform(Eigen::Matrix4d mat) = 0;

protected:
	ModelType type_;
};

#endif