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

#include <armadillo>
#include <lindo.h>
#include <optimization.h>

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
	Eigen::MatrixXd Au;
	Eigen::MatrixXd Bu;
	Eigen::MatrixXd Cu;

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
	bool lindo_solve(std::vector<Eigen::Vector2d> &refined_vertices, Eigen::VectorXd &refined_q);

	void alg_solve_q();
	void alg_solve_q_nc();
	void alg_solve_x();
	void alg_solve_x_nc();
	void alg_solve_quadratic_q();
	bool alg_solve(std::vector<Eigen::Vector2d> &refined_vertices, Eigen::VectorXd &refined_q);

	void midaco_solve_q();
	void madaco_solve_initial_q();

private:
	std::string image_name_;
	ConstraintsEnvironment * environment_;

	static const double FIXED_DEPTH;
	static const double NEIGHBORHOOD;

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

	int num_constraints_rows();
	int num_q_equations();
	int num_x_equations();

	static int  LS_CALLTYPE local_sol_log(pLSmodel model, int iLoc, void *cbData);
	static int  CALLBACKTYPE Funcalcq(pLSmodel pModel, void    *pUserData,
		int      nRow, double  *pdX,
		int      nJDiff, double  dXJBase,
		double   *pdFuncVal, int  *pReserved);
	static double simple_Funcalcq(const Eigen::VectorXd &q, ConstraintsEnvironment *cons_env);
	static double simple_Funcalcq(const alglib::real_1d_array &qarr, ConstraintsEnvironment *cons_env);
	static double simple_Funcalcx(const Eigen::MatrixXd &xarr, ConstraintsEnvironment *cons_env);
	static double simple_Funcalcx(const alglib::real_1d_array &xarr, ConstraintsEnvironment *cons_env);
	static int CALLBACKTYPE Gradcalcq(pLSmodel pModel, void *pUserData,
		int nRow, double *pdX, double *lb,
		double *ub, int nNewPnt, int nNPar,
		int *parlist, double *partial);
	static int  CALLBACKTYPE Funcalcx(pLSmodel pModel, void    *pUserData,
		int      nRow, double  *pdX,
		int      nJDiff, double  dXJBase,
		double   *pdFuncVal, int  *pReserved);

	static void alg_funcq(const alglib::real_1d_array &qarr, double &func, alglib::real_1d_array &grad, void *ptr);
	static void alg_funcx(const alglib::real_1d_array &x, double &func, void *ptr);
	static void alg_fvecq(const alglib::real_1d_array &x, alglib::real_1d_array &fi, void *ptr);
	static void alg_jacq(const alglib::real_1d_array &x, alglib::real_1d_array &fi, alglib::real_2d_array &jac, void *ptr);
	static void alg_fvecx(const alglib::real_1d_array &x, alglib::real_1d_array &fi, void *ptr);

	static void nonl_grad(const Eigen::Vector3d &n1, const Eigen::Vector3d &n2, Eigen::VectorXd &grad);
	static void nonl_grad2(const Eigen::Vector3d &n1, const Eigen::Vector3d &n2, Eigen::VectorXd &grad);

	void midaco_q_function(double *f, double *g, double *qarr);
	void midaco_init_q_func(double *f, double *g, double *qarr);

	void get_sparse_info(std::vector<int> &Abegcol, std::vector<int> &Alencol, std::vector<int> &Arowndx,
		std::vector<double> &A);

	void form_quadratic_coefficients(const Eigen::MatrixXd &Au, const Eigen::MatrixXd &Bu, const Eigen::MatrixXd &Cu,
		Eigen::MatrixXd &QA);

	double F_q_x(const Eigen::VectorXd &q, const Eigen::MatrixXd &x);

	std::string constraints_to_string();
	void get_alg_constraints(alglib::real_2d_array &constraints, alglib::integer_1d_array &type);
	void get_alg_strict_constraints(alglib::real_2d_array & constraints, alglib::integer_1d_array &type);
	std::string matrix_to_string(const Eigen::MatrixXd &mat);
	void print_constraints_func_values(const Eigen::VectorXd &q);
	
	//std::string matrix_to_string(const Eigen::VectorXd &vec);
};