#pragma once

#include <qpoint.h>

#include <list>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>

#include <Eigen\Core>
#include <Eigen\Eigenvalues>

#include "PlanarFace.h"
#include "Line.h"

class ConstraintsGenerator
{
public:
	ConstraintsGenerator();
	ConstraintsGenerator(float focal_length, float w, float h, const std::list<QPointF> &vertices, 
		const std::list<Line> &edges, const std::vector<std::vector<int>> & face_circuits,
		const std::list<std::vector<int>> &parallel_group);
	~ConstraintsGenerator();

	static const float t;
	float Z0;

	void add_connectivity_constraint();
	void add_perspective_symmetry_constraint();
	void add_fixing_vertex_contraint();
	void add_line_parallelism_constrain();

	void detect_symmetric_faces();

	void set_parallel_groups(const std::list<std::vector<int>> &parallel_group);

private:
	float focal_length_;
	Eigen::Matrix3f calib_mat_;

	std::vector<Eigen::Vector2f> vertices_;
	std::vector<Line> edges_;
	std::vector<PlanarFace> faces_;
	std::vector<std::vector<int>> parallel_groups_;
	std::vector<std::list<int>> edge_to_face_map_;

	/* Constraints matrices */
	Eigen::MatrixXf A_;
	Eigen::MatrixXf B_;
	Eigen::MatrixXf C_;
	Eigen::MatrixXf E_;
	Eigen::MatrixXf G_;

	void map_faces_to_verts(const std::vector<PlanarFace> &faces, int num_vertices,
		std::vector<std::vector<int>> &face_vert_map);

	bool perspective_symmetry_in_face(const PlanarFace &face, Eigen::Vector3f &perspective_point, 
		Eigen::Vector3f &sym_axis_start, Eigen::Vector3f &sym_axis_end);

	Eigen::MatrixXf form_S(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2);

	void map_edges_to_face();

	int get_line_id(int v1, int v2);

	bool in_same_face(int line_id_1, int line_id_2);
	bool is_in_face(int line_id, int face_id);

	void map_edges_to_parallel_group(std::vector<int> &edge_parallel_group_map);

	Eigen::Vector3f get_line_equation(int line_id);
};

