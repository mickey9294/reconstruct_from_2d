#pragma once

#include <QObject>
#include <qpoint.h>

#include <list>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <unordered_set>

#include <Eigen\Core>
#include <Eigen\Eigenvalues>

#include <engine.h>

#include "PlanarFace.h"
#include "Line.h"
#include "CameraClibrator.h"
#include "EquationsSolver.h"
#include "Reconstructor.h"

class ConstraintsGenerator : public QObject
{
	Q_OBJECT

public:
	ConstraintsGenerator(QObject *parent = 0);
	ConstraintsGenerator(const std::string &image_path, float w, float h, const std::list<QPointF> &vertices,
		const std::list<Line> &edges, const std::vector<std::vector<int>> & face_circuits,
		const std::list<std::vector<int>> &parallel_group, QObject *parent = 0);
	~ConstraintsGenerator();

	static const float t;
	float Z0;

	void add_constraints(std::vector<Eigen::Vector2f> &refined_vertices, Eigen::VectorXf &refined_q);

	void add_connectivity_constraint();
	void add_perspective_symmetry_constraint();
	void add_fixing_vertex_contraint();
	void add_line_parallelism_constrain();
	void add_orhogonal_corner_constraint();

	void set_parallel_groups(const std::list<std::vector<int>> &parallel_group);

	float get_focal_length() const;

signals:
	void report_status(QString msg);

private:
	float focal_length_;
	float width_;
	float height_;
	Eigen::Vector2f primary_point_;
	Eigen::Matrix3f calib_mat_;

	std::vector<Eigen::Vector2f> vertices_;
	std::vector<Line> edges_;
	std::vector<PlanarFace> faces_;
	std::vector<std::vector<int>> parallel_groups_;
	std::vector<std::list<std::pair<int, int>>> lines_parallel_to_faces_;
	std::vector<std::list<int>> edge_to_face_map_;
	std::vector<std::vector<int>> vert_to_face_map_;


	/* Constraints matrices */
	Eigen::MatrixXf A_;
	Eigen::MatrixXf B_;
	Eigen::MatrixXf C_;
	Eigen::MatrixXf E_;
	std::vector<std::pair<int, int>> G_;

	std::shared_ptr<EquationsSolver> equations_solver_;


	void map_verts_to_face(const std::vector<PlanarFace> &faces, int num_vertices);

	Eigen::MatrixXf form_S(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2);

	void map_edges_to_face();

	int get_line_id(int v1, int v2);

	bool in_same_face(int line_id_1, int line_id_2);
	bool is_in_face(int line_id, int face_id);

	void map_edges_to_parallel_group(std::vector<int> &edge_parallel_group_map);

	Eigen::Vector3f get_line_equation(int line_id);

	void map_verts_to_edge(std::vector<std::vector<int>> &vert_to_edge_map);

	Eigen::Vector2f get_line_direction(int line_id);

	void output_constraints();
	void output_environment();

	int factorial(int n);

	void update_environment(const std::vector<Eigen::Vector2f> &refined_vertices);
};
