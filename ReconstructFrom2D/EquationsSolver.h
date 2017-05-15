#pragma once

#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>

#include <Eigen\Core>
#include <Eigen\Dense>

#include <boost\algorithm\string.hpp>
#include <boost\filesystem.hpp>

#include <engine.h>

//#include <NLF.h>
//#include <LinearEquation.h>
//#include <OptQNIPS.h>
//#include <newmat.h>
#include <armadillo>
#include <lindo.h>

#include "PlanarFace.h"
#include "Line.h"

//using NEWMAT::ColumnVector;
//using NEWMAT::Matrix;
//using NEWMAT::SymmetricMatrix;

//using namespace OPTPP;
struct ConstraintsEnvironment {
	Eigen::MatrixXd A;
	Eigen::MatrixXd B;
	Eigen::MatrixXd C;
	Eigen::MatrixXd E;
	std::vector<std::pair<int, int>> G;
	Eigen::MatrixXd Ad;
	Eigen::MatrixXd Bd;
	Eigen::MatrixXd Cd;

	std::vector<Eigen::Vector2d> verts_2d;
	std::vector<Line> edges;
	std::vector<PlanarFace> faces;
	std::vector<std::vector<int>> vert_to_face_map;
	std::vector<int> imprecise_vertices;
	std::vector<int> precise_sym_faces;
	std::vector<int> perspective_syms;
	std::vector<std::list<std::pair<int, int>>> face_parallel_groups;

	Eigen::MatrixXd xi;
	Eigen::VectorXd qi;
	Eigen::MatrixXd xiplus1;
	Eigen::VectorXd qiplus1;
	double f;

	ConstraintsEnvironment();
	ConstraintsEnvironment(const Eigen::MatrixXd &_A, const Eigen::MatrixXd _B,
		const Eigen::MatrixXd &_C, const Eigen::MatrixXd &_E, const std::vector<std::pair<int, int>> &_G,
		const Eigen::MatrixXd &_Ad, const Eigen::MatrixXd &_Bd, const Eigen::MatrixXd &_Cd,
		const std::vector<Eigen::Vector2d> &_verts_2d, const std::vector<Line> &_edges,
		const std::vector<PlanarFace> _faces, const std::vector<std::vector<int>>  &_vert_to_face_map,
		const std::vector<int> &_imprecise_vertices, const std::vector<int> &_precise_sym_faces,
		const std::vector<int> &_perspective_syms, const std::vector<std::list<std::pair<int, int>>> &_face_parallel_groups,
		double _f);
};

class EquationsSolver
{
public:
	EquationsSolver();
	EquationsSolver(const std::string &image_name);
	~EquationsSolver();

	void set_constraints_environment(const Eigen::MatrixXd &_A, const Eigen::MatrixXd _B,
		const Eigen::MatrixXd &_C, const Eigen::MatrixXd &_E, const std::vector<std::pair<int, int>> &_G,
		const Eigen::MatrixXd &_Ad, const Eigen::MatrixXd &_Bd, const Eigen::MatrixXd &_Cd,
		const std::vector<Eigen::Vector2d> &_verts_2d, const std::vector<Line> &_edges,
		const std::vector<PlanarFace> _faces, const std::vector<std::vector<int>>  &_vert_to_face_map,
		const std::vector<int> &_imprecise_vertices, const std::vector<int> &_precise_sym_faces,
		const std::vector<int> &_perspective_syms, const std::vector<std::list<std::pair<int, int>>> &_face_parallel_groups,
		double _f);

	void solve(int Nf, int Nv, 
		std::vector<Eigen::Vector2d> &refined_vertices, Eigen::VectorXd &refined_q);

	int lindo_solve_q();
	int lindo_solve_x();
	void lindo_solve(std::vector<Eigen::Vector2d> &refined_vertices, Eigen::VectorXd &refined_q);

private:
	std::string image_name_;
	ConstraintsEnvironment * environment_;

	static double fixed_depth;

	//void init_q_function(int ndim, ColumnVector &q);
	//void q_function(int mode, int ndim, const ColumnVector &q, double &fq, ColumnVector &gx, int &result);

	static void form_unfix_constraints(Eigen::MatrixXd &Au, Eigen::MatrixXd &Bu, Eigen::MatrixXd &Cu, 
		const Eigen::MatrixXd &xi, ConstraintsEnvironment *environment);
	static void form_constraints(Eigen::MatrixXd &Au, Eigen::MatrixXd &Bu, Eigen::MatrixXd &Cu,
		const Eigen::MatrixXd &x, ConstraintsEnvironment *environment);

	static Eigen::Vector3d line_equation(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);
	static Eigen::Vector3d line_equation(const Eigen::Vector2d &v1, const Eigen::Vector2d &v2);

	static Eigen::MatrixXd form_S(const Eigen::Vector3d & v1, const Eigen::Vector3d & v2);

	static int find_in_imprecise(int vert_id, const std::vector<int> &imprecise_vertices);

	int num_contraints_rows();

	static int  LS_CALLTYPE local_sol_log(pLSmodel model, int iLoc, void *cbData);
	static int  CALLBACKTYPE Funcalcq(pLSmodel pModel, void    *pUserData,
		int      nRow, double  *pdX,
		int      nJDiff, double  dXJBase,
		double   *pdFuncVal, int  *pReserved);
	static int  CALLBACKTYPE Funcalcx(pLSmodel pModel, void    *pUserData,
		int      nRow, double  *pdX,
		int      nJDiff, double  dXJBase,
		double   *pdFuncVal, int  *pReserved);

	void get_sparse_info(std::vector<int> &Abegcol, std::vector<int> &Alencol, std::vector<int> &Arowndx,
		std::vector<double> &A);

	double F_q_x(const Eigen::VectorXd &q, const Eigen::MatrixXd &x);
};