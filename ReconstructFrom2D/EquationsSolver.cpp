#include "EquationsSolver.h"

const double EquationsSolver::FIXED_DEPTH = -2.0 / 3.0;
const double EquationsSolver::NEIGHBORHOOD = 49.0;

#define APIVERSION \
{\
    char szVersion[255], szBuild[255];\
    LSgetVersionInfo(szVersion,szBuild);\
    printf("\nLINDO API Version %s built on %s\n",szVersion,szBuild);\
}\

EquationsSolver::EquationsSolver()
	: environment_(NULL)
{
}

EquationsSolver::EquationsSolver(const std::string & image_name)
	: environment_(NULL), image_name_(image_name)
{
}


EquationsSolver::~EquationsSolver()
{
	if (environment_)
		delete(environment_);
}

void EquationsSolver::set_constraints_environment(const Eigen::MatrixXd & _A, const Eigen::MatrixXd _B, 
	const Eigen::MatrixXd & _C, const Eigen::MatrixXd & _E, const std::vector<std::pair<int, int>>& _G, 
	const Eigen::MatrixXd & _Ad, const Eigen::MatrixXd & _Bd, const Eigen::MatrixXd & _Cd, 
	const std::vector<Eigen::Vector2d>& _verts_2d, const std::vector<Line>& _edges, 
	const std::vector<PlanarFace> _faces, const std::vector<std::vector<int>>& _vert_to_face_map, 
	const std::vector<int>& _imprecise_vertices, const std::vector<int>& _precise_sym_faces, 
	const std::vector<int>& _perspective_syms, const std::vector<std::list<std::pair<int, int>>>& _face_parallel_groups, double _f)
{
	environment_ = new ConstraintsEnvironment(_A, _B, _C, _E, _G, _Ad, _Bd, _Cd,
		_verts_2d, _edges, _faces, _vert_to_face_map, _imprecise_vertices, 
		_precise_sym_faces, _perspective_syms, _face_parallel_groups, _f);
}

void EquationsSolver::solve(int Nf, int Nv, 
	std::vector<Eigen::Vector2d> &refined_vertices, Eigen::VectorXd &refined_q)
{
	Engine *ep;
	if (!(ep = engOpen("\0"))) //启动matlab 引擎
	{
		std::cerr << "Initilizing Reconstructor failed." << std::endl;
		return;
	}
	else
	{
		engSetVisible(ep, false); // 设置窗口不可见
	}

	engEvalString(ep, "cd \'D:\\Projects\\reconstruct_from_2d\\matlab\';");

	/* Reconstruct the rough 3D shape */
	std::string start_point_path = "..\\start_points\\" + image_name_ + ".csv";
	if (!boost::filesystem::exists(boost::filesystem::path(start_point_path)))
	{
		std::string rough_recon_cmd = "solve_O3dr(" + std::to_string(Nf) + ");";
		engEvalString(ep, rough_recon_cmd.c_str());
	}
	else
	{
		std::string load_q0_cmd = "load_start_point(\'" + image_name_ + "\');";
		engEvalString(ep, load_q0_cmd.c_str());
	}

	/* Constraint refinement and joint optimization */
	engEvalString(ep, "solve_q_x();");

	/* Load the result of optimization */
	refined_vertices.resize(Nv);
	std::ifstream in;
	in.open("..\\matlab\\xi_1.csv");
	if (in.is_open())
	{
		std::string line;
		std::vector<std::string> line_split;
		for (int i = 0; i < Nv; i++)
		{
			std::getline(in, line);
			if (line.length() > 0)
			{
				boost::split(line_split, line, boost::is_any_of(","), boost::token_compress_on);
				float x = std::stof(line_split[0]);
				float y = std::stof(line_split[1]);
				refined_vertices[i][0] = x;
				refined_vertices[i][1] = y;
			}
		}
		in.close();
	}

	in.open("..\\matlab\\qi_1.csv");
	if (in.is_open())
	{
		refined_q.resize(3 * Nf);
		std::string line;
		for (int i = 0; i < 3 * Nf; i++)
		{
			std::getline(in, line);
			if(line.length() > 0)
				refined_q[i] = std::stof(line);
		}

		in.close();
	}

	engClose(ep);
}

int EquationsSolver::lindo_solve_q()
{
	pLSenv env = NULL;
	pLSmodel model = NULL;
	FILE *logfile = stdout;
	int errors = 0, nErrorCode = LSERR_NO_ERROR;
	double *lb = NULL, *ub = NULL;
	int Nnlobj, howmany = 0;
	char MY_LICENSE_KEY[1024];
	int Nf = environment_->faces.size();

	/*****************************************************************
	* Step 1: Create a model in the environment.
	*****************************************************************/
	nErrorCode = LSloadLicenseString("D:\\ProgrammingTools\\Lindoapi\\license\\lndapi100.lic", MY_LICENSE_KEY);
	if (nErrorCode != LSERR_NO_ERROR)
	{
		printf("Failed to load license key (error %d)\n", nErrorCode);
		exit(1);
	}

	APIVERSION;
	env = LScreateEnv(&nErrorCode, MY_LICENSE_KEY);
	if (nErrorCode != LSERR_NO_ERROR) 
		return nErrorCode;

	model = LScreateModel(env, &nErrorCode);
	if (nErrorCode != LSERR_NO_ERROR) 
		return nErrorCode;

	/*****************************************************************
	* Step 2: Specify the LP portion of the model.
	*****************************************************************/
	/* model dimensions */
	int m = num_constraints_rows();
	int n = 3 * Nf;

	double *primal = new double[n];

	std::vector<int> VAbegcol, VAlencol, VArowndx;
	std::vector<double> VA;
	get_sparse_info(VAbegcol, VAlencol, VArowndx, VA);
	int nz = VA.size();

	int *Abegcol = new int[VAbegcol.size()];
	int *Alencol = new int[VAlencol.size()];
	int *Arowndx = new int[VArowndx.size()];
	double *A = new double[VA.size()];

	for (int i = 0; i < VAbegcol.size(); i++)
		Abegcol[i] = VAbegcol[i];
	for (int i = 0; i < VAlencol.size(); i++)
		Alencol[i] = VAlencol[i];
	for (int i = 0; i < VArowndx.size(); i++)
		Arowndx[i] = VArowndx[i];
	for (int i = 0; i < VA.size(); i++)
		A[i] = VA[i];

	double *rhs = new double[m];
	std::memset(rhs, 0, m * sizeof(double));
	rhs[m - 1] = FIXED_DEPTH;
	
	double *cost = new double[n];
	std::memset(cost, 0, n * sizeof(double));

	char * contype = new char[m];
	for (int i = 0; i < m; i++)
		contype[i] = 'E';

	char *vartype = new char[n];
	for (int i = 0; i < n; i++)
		vartype[i] = 'C';

	for (int i = 0; i < VAbegcol.size(); i++)
		Abegcol[i] = VAbegcol[i];
	for (int i = 0; i < VAlencol.size(); i++)
		Alencol[i] = VAlencol[i];
	for (int i = 0; i < VArowndx.size(); i++)
		Arowndx[i] = VArowndx[i];
	for (int i = 0; i < VA.size(); i++)
		A[i] = VA[i];

	lb = new double[n];
	ub = new double[n];
	for (int i = 0; i < n; i++)
	{
		lb[i] = -1.0;
		ub[i] = 1.0;
	}

	nErrorCode = LSloadLPData(model, m, n, LS_MIN, 0.0, cost, rhs, contype, nz,
		Abegcol, Alencol, A, Arowndx, lb, ub);
	if (nErrorCode != LSERR_NO_ERROR) 
		return nErrorCode;

	/*****************************************************************
	* Step 3: Specify the NLP portion of the model.
	*****************************************************************/
	/* The number of nonlinear variables in each column */
	std::memset(Alencol, 0, n * sizeof(int));

	/* The indices of the first nonlinear variable in each column */
	std::memset(Abegcol, 0, (n + 1) * sizeof(int));

	/* The indices of nonlinear constraints */
	std::memset(Arowndx, 0, VArowndx.size() * sizeof(int));

	/* The indices of variables that are nonlinear in the objective*/
	int *Nobjndx = new int[n];
	for (int i = 0; i < n; i++)
		Nobjndx[i] = i;

	/* Number nonlinear variables in cost. */
	Nnlobj = n;
	/* Load the nonlinear structure */
	nErrorCode = LSloadNLPData(model, Abegcol, Alencol,
		NULL, Arowndx, Nnlobj, Nobjndx, NULL);

	if (nErrorCode != LSERR_NO_ERROR) return nErrorCode;

	/*****************************************************************
	* Step 4: Set up callback functions
	*****************************************************************/
	/* Install the routine that will calculate the function values. */
	nErrorCode = LSsetFuncalc(model, (Funcalc_type)Funcalcq, environment_);
	if (nErrorCode != LSERR_NO_ERROR) return nErrorCode;

	nErrorCode = LSsetGradcalc(model, Gradcalcq, environment_, 0, NULL);
	if (nErrorCode != LSERR_NO_ERROR) return nErrorCode;

	/* Install the callback function to call at every local solution */
	LSsetCallback(model, (cbFunc_t)local_sol_log, &howmany);

	/* Set the print level to 1 */
	nErrorCode = LSsetModelIntParameter(model, LS_IPARAM_NLP_PRINTLEVEL, 1);

	/* Set the NLP prelevel to 126 */
	//nErrorCode = LSsetModelIntParameter(model, LS_IPARAM_NLP_PRELEVEL, 0);

	/*****************************************************************
	* Step 5: Solve the model
	*****************************************************************/
	/* Turn multi-start search on */
	//LSsetModelIntParameter(model, LS_IPARAM_NLP_SOLVER, LS_NMETHOD_MSW_GRG);

	/* Set maximum number of local optimizations */
	LSsetModelIntParameter(model, LS_IPARAM_NLP_MAXLOCALSEARCH, 5);

	/* Load an initial starting point */
	for (int i = 0; i < n; i++)
		primal[i] = environment_->qi[i];
	std::cout << "Before opt, qi: " << environment_->qi.transpose() << std::endl;
	double fval;
	Funcalcq(model, environment_, -1, primal, 0, 0, &fval, NULL);
	std::cout << "Funcq = " << fval << std::endl;
	nErrorCode = LSloadVarStartPoint(model, primal);

	printf("\n\tSolving the NLP using Multi-Start Approach.\n\n");
	int status;
	nErrorCode = LSoptimize(model, LS_METHOD_FREE, &status);

	if (nErrorCode != LSERR_NO_ERROR) 
		return nErrorCode;
	
	int i;
	double objval, dobjval, pinf, dinf;
	nErrorCode = LSgetPrimalSolution(model, primal);
	nErrorCode = LSgetInfo(model, LS_DINFO_POBJ, &objval);
	nErrorCode = LSgetInfo(model, LS_DINFO_DOBJ, &dobjval);
	nErrorCode = LSgetInfo(model, LS_DINFO_PINFEAS, &pinf);
	nErrorCode = LSgetInfo(model, LS_DINFO_DINFEAS, &dinf);

	Funcalcq(model, environment_, -1, primal, 0, 0, &fval, NULL);
	std::cout << "fval = " << fval;
	if (nErrorCode == LSERR_NO_ERROR)
	{
		printf("\n\n\n");
		printf("obj  = %15.7f \n", objval);
		for (i = 0; i < n; i++)
		{
			printf("qi+1[%d] = %15.7f \n", i, primal[i]);
			environment_->qiplus1[i] = primal[i];
		}
		printf("bound = %f \n", dobjval);
		printf("pinf = %16e \n", pinf);
		printf("dinf = %16e \n", dinf);
		printf("stat = %d \n", status);
	}
	else
	{
		printf("Error %d occured\n\n\n", nErrorCode);
	}


	/*****************************************************************
	* Step 6: Delete the model & env space
	*****************************************************************/
	LSdeleteModel(&model);
	LSdeleteEnv(&env);

	delete(Abegcol);
	delete(Alencol);
	delete(Arowndx);
	delete(A);
	delete(rhs);
	delete(cost);
	delete(contype);
	delete(vartype);
	delete(Nobjndx);
	delete(primal);
	delete(lb);
	delete(ub);

	return nErrorCode;
}

int EquationsSolver::lindo_solve_x()
{
	if (environment_->imprecise_vertices.empty())
		return 0;

	/* main entry point*/
	pLSenv env = NULL;
	pLSmodel model = NULL;
	int errors = 0, nErrorCode = LSERR_NO_ERROR;
	char MY_LICENSE_KEY[1024];

	/*****************************************************************
	* Step 1: Create a model in the environment.
	*****************************************************************/
	nErrorCode = LSloadLicenseString("D:\\ProgrammingTools\\Lindoapi\\license\\lndapi100.lic", MY_LICENSE_KEY);
	if (nErrorCode != LSERR_NO_ERROR)
	{
		printf("Failed to load license key (error %d)\n", nErrorCode);
		exit(1);
	}

	APIVERSION;
	env = LScreateEnv(&nErrorCode, MY_LICENSE_KEY);
	if (nErrorCode != LSERR_NO_ERROR) 
		return nErrorCode;

	model = LScreateModel(env, &nErrorCode);
	if (nErrorCode != LSERR_NO_ERROR) 
		return nErrorCode;

	/*****************************************************************
	* Step 2: Specify the LP portion of the model.
	*****************************************************************/
	/* model dimensions */
	int m = environment_->imprecise_vertices.size();
	int n = 2 * m;
	int nz = 0;

	/* The indices of the first nonzero in each column */
	int *Abegcol = new int[n + 1];
	std::memset(Abegcol, 0, (n + 1) * sizeof(int));

	/* The indices of the first nonzero in each column */
	int *Alencol = new int[n];
	std::memset(Alencol, 0, n * sizeof(int));

	/* The nonzero coefficients of the LP portion of the model*/
	double *A = NULL;

	/* The objective coefficients of the linear portion of the model*/
	/* and lower/upper bounds on variables */
	double *cost = new double[n];
	memset(cost, 0, n * sizeof(double));
	double *lb = new double[n];
	double *ub = new double[n];
	for (int i = 0; i < m; i++)
	{
		int vert_id = environment_->imprecise_vertices[i];
		Eigen::Vector2d &x0 = environment_->verts_2d[vert_id];
		lb[2 * i] = x0.x() - 8.0;
		ub[2 * i] = x0.x() + 8.0;
		lb[2 * i + 1] = x0.y() - 8.0;
		ub[2 * i + 1] = x0.y() + 8.0;
	}

	/* The right-hand sides of the constraints */
	double *rhs = new double[m];
	for (int i = 0; i < m; i++)
	{
		//int vert_id = environment_->imprecise_vertices[i];
		//Eigen::Vector2d fixed_vert = environment_->verts_2d[vert_id];
		//rhs[i] = 49.0 - std::pow(fixed_vert.x(), 2) - std::pow(fixed_vert.y(), 2);
		rhs[i] = 0;
	}

	/* The constraint types */
	char *contype = new char[m];
	for (int i = 0; i < m; i++)
		contype[i] = 'L';
	char *vartype = new char[n];
	for (int i = 0; i < n; i++)
		vartype[i] = 'C';

	/* Load in nonzero structure and linear/constant terms.  */
	nErrorCode = LSloadLPData(model, m, n, LS_MIN, 0.0, cost, rhs, contype, nz,
		Abegcol, Alencol, A, NULL, lb, ub);

	if (nErrorCode != LSERR_NO_ERROR) 
		return nErrorCode;

	nErrorCode = LSloadVarType(model, vartype);
	if (nErrorCode != LSERR_NO_ERROR) 
		return nErrorCode;

	/*****************************************************************
	* Step 3: Specify the NLP portion of the model.
	*****************************************************************/
	/* The number of nonlinear variables in each column */
	for (int i = 0; i < n; i++)
		Alencol[i] = 1;

	/* The indices of the first nonlinear variable in each column */
	for (int i = 0; i <= n; i++)
		Abegcol[i] = i;

	/* The indices of nonlinear constraints */
	int *Arowndx = new int[n];
	for (int i = 0; i < m; i++)
	{
		Arowndx[2 * i] = i;
		Arowndx[2 * i + 1] = i;
	}

	/* The indices of variables that are nonlinear in the objective*/
	int *Nobjndx = new int[n];
	for (int i = 0; i < n; i++)
		Nobjndx[i] = i;

	/* Number nonlinear variables in cost. */
	int Nnlobj = n;

	/* Load the nonlinear structure */
	nErrorCode = LSloadNLPData(model, Abegcol, Alencol,
		NULL, Arowndx, Nnlobj, Nobjndx, NULL);

	if (nErrorCode != LSERR_NO_ERROR) 
		return nErrorCode;
	printf("\nThe model is installed successfully...\n");

	/*****************************************************************
	* Step 4: Set up callback functions
	*****************************************************************/
	/* Install the routine that will calculate the function values. */
	nErrorCode = LSsetFuncalc(model, (Funcalc_type)Funcalcx, environment_);
	if (nErrorCode != LSERR_NO_ERROR) 
		return nErrorCode;
	/* Install a callback function */
	int counter = 0;
	//LSsetCallback(model, (cbFunc_t)local_sol_log, &counter);

	/* Set the print level to 1 */
	nErrorCode = LSsetModelIntParameter(model, LS_IPARAM_NLP_PRINTLEVEL, 1);

	/* Turn multi-start search on */
	LSsetModelIntParameter(model, LS_IPARAM_NLP_SOLVER, LS_NMETHOD_MSW_GRG);

	/* Set maximum number of local optimizations */
	LSsetModelIntParameter(model, LS_IPARAM_NLP_MAXLOCALSEARCH, 2);

	/*****************************************************************
	* Step 5: Solve the model
	*****************************************************************/
	/* load an initial starting point */
	double *primal = new double[n];
	for (int i = 0; i < m; i++)
	{
		primal[2 * i] = environment_->xi(i, 0);
		primal[2 * i + 1] = environment_->xi(i, 1);
	}
	nErrorCode = LSloadVarStartPoint(model, primal);

	/* set up as NLP and optimize */
	int status;
	double objval, dobjval, dinf, pinf;
	nErrorCode = LSoptimize(model, LS_METHOD_FREE, &status);
	if (nErrorCode != LSERR_NO_ERROR)
		return nErrorCode;
	nErrorCode = LSgetInfo(model, LS_DINFO_POBJ, &objval);
	nErrorCode = LSgetInfo(model, LS_DINFO_DOBJ, &dobjval);
	nErrorCode = LSgetInfo(model, LS_DINFO_PINFEAS, &pinf);
	nErrorCode = LSgetInfo(model, LS_DINFO_DINFEAS, &dinf);

	nErrorCode = LSgetPrimalSolution(model, primal);
	nErrorCode = LSgetSlacks(model, rhs);

	double fval;
	Funcalcx(model, environment_, -1, primal, 0, 0, &fval, NULL);
	std::cout << "fval = " << fval;
	if (nErrorCode == LSERR_NO_ERROR)
	{
		printf("\n\n\n");
		printf("obj  = %15.7f \n", objval);
		for (int i = 0; i < m; i++)
		{
			printf("xi+1[%d] = %15.7f \n", i, primal[2 * i]);
			printf("xi+1[%d] = %15.7f \n", i, primal[2 * i + 1]);
			environment_->xiplus1(i, 0) = primal[2 * i];
			environment_->xiplus1(i, 1) = primal[2 * i + 1];
		}
		printf("bound = %f \n", dobjval);
		printf("pinf = %16e \n", pinf);
		printf("dinf = %16e \n", dinf);
		printf("stat = %d \n", status);
	}
	else
	{
		printf("Error %d occured\n\n\n", nErrorCode);
	}

	/*****************************************************************
	* Step 6: Delete the model & env space
	*****************************************************************/
	LSdeleteModel(&model);
	LSdeleteEnv(&env);

	delete(Abegcol);
	delete(Alencol);
	delete(cost);
	delete(rhs);
	delete(contype);
	delete(vartype);
	delete(Arowndx);
	delete(Nobjndx);
	delete(primal);
	delete(lb);
	delete(ub);

	return nErrorCode;
}

bool EquationsSolver::lindo_solve(std::vector<Eigen::Vector2d> &refined_vertices, Eigen::VectorXd &refined_q)
{
	std::ifstream in;
	int Nf = environment_->faces.size();
	std::string start_point_path = "..\\start_points\\" + image_name_ + ".csv";
	if (!boost::filesystem::exists(boost::filesystem::path(start_point_path)))
	{
		Engine *ep;
		if (!(ep = engOpen("\0"))) //启动matlab 引擎
		{
			std::cerr << "Initilizing Reconstructor failed." << std::endl;
			return false;
		}
		else
			engSetVisible(ep, false); // 设置窗口不可见
		engEvalString(ep, "cd \'D:\\Projects\\reconstruct_from_2d\\matlab\';");
		/* Reconstruct the rough 3D shape */
		std::string rough_recon_cmd = "solve_O3dr(" + std::to_string(Nf) + ");";
		engEvalString(ep, rough_recon_cmd.c_str());
		engClose(ep);

		in.open("..\\matlab\\q0.csv");
	}
	else
	{
		in.open(start_point_path.c_str());
	}
	if (in.is_open())
	{
		std::string line;
		for (int i = 0; i < 3 * Nf; i++)
		{
			std::getline(in, line);
			environment_->qi[i] = std::stod(line);
		}
		in.close();
	}
	else
	{
		std::cerr << "q0.csv does not exist." << std::endl;
		return false;
	}
	//madaco_solve_initial_q();

	int idx = 0;
	double Fi = 0;
	const double sigma = 1e-6;

	while (true)
	{
		form_unfix_constraints(environment_->Au, environment_->Bu, environment_->Cu,
			environment_->xi, environment_);
		//lindo_solve_q();
		midaco_solve_q();
		lindo_solve_x();

		Eigen::MatrixXd full_x(environment_->verts_2d.size(), 2);
		for (int i = 0; i < full_x.rows(); i++)
		{
			full_x(i, 0) = environment_->verts_2d[i].x();
			full_x(i, 1) = environment_->verts_2d[i].y();
		}
		for (int i = 0; i < environment_->imprecise_vertices.size(); i++)
		{
			int vert_id = environment_->imprecise_vertices[i];
			full_x(vert_id, 0) = environment_->xiplus1(i, 0);
			full_x(vert_id, 1) = environment_->xiplus1(i, 1);
		}

		double Fiplus1 = F_q_x(environment_->qiplus1, full_x);
		double delta = std::abs(Fiplus1 - Fi);
		if (delta < sigma)
			break;
		else
		{
			environment_->qi = environment_->qiplus1;
			environment_->xi = environment_->xiplus1;
			Fi = Fiplus1;
		}
		idx++;
	}

	refined_vertices = environment_->verts_2d;
	for (int i = 0; i < environment_->imprecise_vertices.size(); i++)
	{
		int vert_id = environment_->imprecise_vertices[i];
		refined_vertices[vert_id][0] = environment_->xiplus1(i, 0);
		refined_vertices[vert_id][1] = environment_->xiplus1(i, 1);
	}

	refined_q = environment_->qiplus1;

	return true;
}

void EquationsSolver::alg_solve_q()
{
	alglib::real_1d_array qi;
	alglib::real_1d_array scale;
	qi.setlength(environment_->qi.size());
	scale.setlength(qi.length());
	for (int i = 0; i < environment_->qi.size(); i++)
	{
		qi[i] = environment_->qi[i];
		scale[i] = 1.0e-4;
	}

	alglib::real_2d_array c;
	//std::string ct_str = "[0";
	//for (int i = 1; i < c.rows(); i++)
	//	ct_str += ",0";
	//ct_str += "]";
	alglib::integer_1d_array ct;
	get_alg_constraints(c, ct);
	//alglib::real_2d_array c_check = constraints_to_string().c_str();

	alglib::minbleicstate state;
	alglib::minbleicreport rep;

	double epsg = 0.000001;
	double epsf = 0;
	double epsx = 0;
	alglib::ae_int_t maxits = 0;

	//std::cout << "qi start :";
	//for (int i = 0; i < qi.length(); i++)
	//	std::cout << qi[i] << " ";
	//std::cout << std::endl;
	double func_val;
	//alglib::real_1d_array grad;
	//grad.setlength(qi.length());
	//alg_funcq(qi, func_val, grad, environment_);
	//print_constraints_func_values(environment_->qi);

	alglib::minbleiccreate(qi, state);
	alglib::minbleicsetlc(state, c, ct);
	alglib::minbleicsetcond(state, epsg, epsf, epsx, maxits);
	alglib::minbleicsetscale(state, scale);
	alglib::minbleicoptimize(state, alg_funcq, NULL, environment_);
	alglib::minbleicresults(state, qi, rep);
	int term_type = rep.terminationtype;
	
	for (int i = 0; i < environment_->qiplus1.size(); i++)
		environment_->qiplus1[i] = qi[i];

	//func_val = simple_Funcalcq(environment_->qiplus1, environment_);
	//std::cout << "funcq_val = " << func_val << std::endl;
}

void EquationsSolver::alg_solve_q_nc()
{
	std::string qstr = matrix_to_string(environment_->qi);
	alglib::real_1d_array qi = qstr.c_str();
	
	std::string lb_str = "[-1.0", ub_str = "[1.0";
	for (int i = 1; i < qi.length(); i++)
	{
		lb_str += ",-1.0";
		ub_str += ",1.0";
	}
	lb_str += "]";
	ub_str += "]";
	alglib::real_1d_array lb = lb_str.c_str();
	alglib::real_1d_array ub = ub_str.c_str();
	
	double epsx = 1.0e-8;
	double epsf = 1.0e-8;
	double epsg = 1.0e-8;
	alglib::ae_int_t maxits = 0;
	alglib::minlmstate state;
	alglib::minlmreport rep;

	std::cout << "Before opt, fvalq = " << simple_Funcalcq(qi, environment_) << std::endl;
	std::cout << "qi: " << qi.tostring(5) << std::endl;

	alglib::real_1d_array scale;
	scale.setlength(qi.length());
	for (int i = 0; i < qi.length(); i++)
		scale[i] = 1.0e-4;

	alglib::minlmcreatevj(qi.length(), num_q_equations(), qi, state);
	alglib::minlmsetcond(state, epsg, epsf, epsx, maxits);
	alglib::minlmsetscale(state, scale);
	alglib::minlmsetbc(state, lb, ub);
	alglib::minlmoptimize(state, alg_fvecq, alg_jacq, NULL, environment_);
	alglib::minlmresults(state, qi, rep);

	int term_type = rep.terminationtype;

	std::cout << "After opt, fvalq = " << simple_Funcalcq(qi, environment_) << std::endl;
	std::cout << "qi+1: " << qi.tostring(5) << std::endl;

	for (int i = 0; i < environment_->qiplus1.size(); i++)
		environment_->qiplus1[i] = qi[i];
}

void EquationsSolver::alg_solve_x()
{
	std::string xi_str = "[" + std::to_string(environment_->xi(0, 0)) + ","
		+ std::to_string(environment_->xi(0, 1));
	std::string scale_str = "[100, 100";
	for (int i = 1; i < environment_->xi.rows(); i++)
	{
		xi_str += "," + std::to_string(environment_->xi(i, 0))
			+ "," + std::to_string(environment_->xi(i, 1));
		scale_str += ",100,100";
	}
	xi_str += "]";
	scale_str += "]";

	alglib::real_1d_array xi = xi_str.c_str();
	alglib::real_1d_array scale = scale_str.c_str();
	double epsg = 1.0e-6;
	double epsf = 1.0e-6;
	double epsx = 1.0e-6;
	double diffstep = 1.0e-4;
	alglib::ae_int_t maxits = 0;
	alglib::ae_int_t outerits = 5;
	alglib::ae_int_t updatefreq = 10;
	double rho = 10;
	alglib::minbleicstate state;
	alglib::minbleicreport rep;

	alglib::real_1d_array lb, ub;
	lb.setlength(xi.length());
	ub.setlength(xi.length());
	for (int i = 0; i < environment_->imprecise_vertices.size(); i++)
	{
		int vert_id = environment_->imprecise_vertices[i];
		lb[2 * i] = environment_->verts_2d[vert_id][0] - 8.0;
		ub[2 * i] = environment_->verts_2d[vert_id][0] + 8.0;
		lb[2 * i + 1] = environment_->verts_2d[vert_id][1] - 8.0;
		ub[2 * i + 1] = environment_->verts_2d[vert_id][1] + 8.0;
	}
	
	alglib::minbleiccreatef(environment_->xi.rows() * 2, xi, diffstep, state);
	alglib::minbleicsetscale(state, scale);
	alglib::minbleicsetbc(state, lb, ub);
	alglib::minbleicsetcond(state, epsg, epsf, epsx, maxits);
	alglib::minbleicoptimize(state, alg_funcx, NULL, environment_);
	alglib::minbleicresults(state, xi, rep);
	int term_type = rep.terminationtype;

	//std::cout << "xi:" << std::endl;
	//std::cout << xi.tostring(5) << std::endl;


	for (int i = 0; i < environment_->xiplus1.rows(); i++)
	{
		environment_->xiplus1(i, 0) = xi[2 * i];
		environment_->xiplus1(i, 1) = xi[2 * i + 1];
	}
}

void EquationsSolver::alg_solve_x_nc()
{
	alglib::real_1d_array x;
	x.setlength(environment_->xi.rows() * 2);
	for (int i = 0; i < environment_->xi.rows(); i++)
	{
		x[2 * i] = environment_->xi(i, 0);
		x[2 * i + 1] = environment_->xi(i, 1);
	}
	double epsg = 1.0e-8;
	double epsf = 1.0e-8;
	double epsx = 1.0e-8;
	alglib::ae_int_t maxits = 0;
	alglib::minlmstate state;
	alglib::minlmreport rep;
	alglib::real_1d_array lb, ub;
	lb.setlength(x.length());
	ub.setlength(x.length());
	for (int i = 0; i < environment_->verts_2d.size(); i++)
	{
		lb[2 * i] = environment_->verts_2d[i][0] - 8.0;
		ub[2 * i] = environment_->verts_2d[i][0] + 8.0;
		lb[2 * i + 1] = environment_->verts_2d[i][1] - 8.0;
		ub[2 * i + 1] = environment_->verts_2d[i][1] + 8.0;
	}

	std::cout << "Before opt, fvalx = " << simple_Funcalcx(x, environment_) << std::endl;

	alglib::minlmcreatev(x.length(), num_x_equations(), x, 1.0e-4, state);
	alglib::minlmsetbc(state, lb, ub);
	alglib::minlmsetcond(state, epsg, epsf, epsx, maxits);
	alglib::minlmoptimize(state, alg_fvecx, NULL, environment_);
	alglib::minlmresults(state, x, rep);

	int term_type = rep.terminationtype;

	std::cout << "After opt, fvalx = " << simple_Funcalcx(x, environment_) << std::endl;

	for (int i = 0; i < environment_->xiplus1.rows(); i++)
	{
		environment_->xiplus1(i, 0) = x[2 * i];
		environment_->xiplus1(i, 1) = x[2 * i + 1];
	}
}

void EquationsSolver::alg_solve_quadratic_q()
{
	Eigen::MatrixXd QA;
	form_quadratic_coefficients(environment_->Au, environment_->Bu, environment_->Cu, QA);

	//std::ofstream out("..\\QA.csv");
	//const Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n");
	//out << QA.format(CSVFormat);
	//out.close();
	
	alglib::real_2d_array a = matrix_to_string(QA).c_str();
	alglib::real_1d_array b = matrix_to_string(Eigen::VectorXd::Zero(QA.rows())).c_str();

	std::string qstr = "[";
	std::string scale_str = "[0.001";
	qstr += std::to_string(environment_->qi[0]);
	for (int i = 1; i < environment_->qi.size(); i++)
	{
		qstr += "," + std::to_string(environment_->qi[i]);
		scale_str += ",0.001";
	}
	qstr += "]";
	scale_str += "]";

	alglib::real_1d_array qi = qstr.c_str();
	alglib::real_2d_array c = constraints_to_string().c_str();
	alglib::real_1d_array scale = scale_str.c_str();

	std::string ct_str = "[0";
	for (int i = 1; i < c.rows(); i++)
		ct_str += ",0";
	ct_str += "]";
	alglib::integer_1d_array ct = ct_str.c_str();

	alglib::minqpstate state;
	alglib::minqpreport rep;

	alglib::minqpcreate(QA.rows(), state);
	alglib::minqpsetquadraticterm(state, a);
	alglib::minqpsetlinearterm(state, b);
	alglib::minqpsetlc(state, c, ct);

	alglib::minqpsetscale(state, scale);

	alglib::minqpsetalgobleic(state, 0.0, 0.0, 0.0, 0);
	alglib::minqpoptimize(state);
	alglib::minqpresults(state, qi, rep);

	for (int i = 0; i < environment_->qiplus1.size(); i++)
		environment_->qiplus1[i] = qi[i];

	double func_val = simple_Funcalcq(environment_->qiplus1, environment_);
	std::cout << "func_val = " << func_val << std::endl;
	std::cout << "qi+1: " << environment_->qiplus1.transpose() << std::endl;
}

bool EquationsSolver::alg_solve(std::vector<Eigen::Vector2d>& refined_vertices, Eigen::VectorXd & refined_q)
{
	std::ifstream in;
	int Nf = environment_->faces.size();
	std::string start_point_path = "..\\start_points\\" + image_name_ + ".csv";
	if (!boost::filesystem::exists(boost::filesystem::path(start_point_path)))
	{
		Engine *ep;
		if (!(ep = engOpen("\0"))) //启动matlab 引擎
		{
			std::cerr << "Initilizing Reconstructor failed." << std::endl;
			return false;
		}
		else
			engSetVisible(ep, false); // 设置窗口不可见
		engEvalString(ep, "cd \'D:\\Projects\\reconstruct_from_2d\\matlab\';");
		/* Reconstruct the rough 3D shape */
		std::string rough_recon_cmd = "solve_O3dr(" + std::to_string(Nf) + ");";
		engEvalString(ep, rough_recon_cmd.c_str());
		engClose(ep);

		in.open("..\\matlab\\q0.csv");
	}
	else
	{
		in.open(start_point_path.c_str());
	}
	if (in.is_open())
	{
		std::string line;
		for (int i = 0; i < 3 * Nf; i++)
		{
			std::getline(in, line);
			if(line.length() > 0)
				environment_->qi[i] = std::stod(line);
		}
		in.close();
	}
	else
	{
		std::cerr << "q0.csv does not exist." << std::endl;
		return false;
	}

	int idx = 0;
	double Fi = 0;
	const double sigma = 1e-6;

	//environment_->xi.resize(environment_->verts_2d.size(), 2);
	//environment_->xiplus1.resize(environment_->verts_2d.size(), 2);
	//for (int i = 0; i < environment_->verts_2d.size(); i++)
	//{
	//	environment_->xi(i, 0) = environment_->verts_2d[i][0];
	//	environment_->xi(i, 1) = environment_->verts_2d[i][1];
	//	environment_->xiplus1(i, 0) = environment_->verts_2d[i][0];
	//	environment_->xiplus1(i, 1) = environment_->verts_2d[i][1];
	//}
	const Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n");
	//std::ofstream out("..\\Ad.csv");
	//out << environment_->Ad.format(CSVFormat);
	//out.close();
	//out.open("..\\Bd.csv");
	//out << environment_->Bd.format(CSVFormat);
	//out.close();
	//out.open("..\\Cd.csv");
	//out << environment_->Cd.format(CSVFormat);
	//out.close();


	while (true)
	{
		form_unfix_constraints(environment_->Au, environment_->Bu, environment_->Cu,
			environment_->xi, environment_);

		alg_solve_q();
		double fval = simple_Funcalcq(environment_->qiplus1, environment_);
		std::cout << "Funcq_val = " << fval << std::endl;
		alg_solve_x();
		fval = simple_Funcalcx(environment_->xiplus1, environment_);
		std::cout << "Funcx_val = " << fval << std::endl;

		Eigen::MatrixXd full_x(environment_->verts_2d.size(), 2);
		for (int i = 0; i < full_x.rows(); i++)
		{
			full_x(i, 0) = environment_->verts_2d[i].x();
			full_x(i, 1) = environment_->verts_2d[i].y();
		}
		for (int i = 0; i < environment_->imprecise_vertices.size(); i++)
		{
			int vert_id = environment_->imprecise_vertices[i];
			full_x(vert_id, 0) = environment_->xiplus1(i, 0);
			full_x(vert_id, 1) = environment_->xiplus1(i, 1);
		}

		double Fiplus1 = F_q_x(environment_->qiplus1, full_x);

		//form_constraints(environment_->A, environment_->B, environment_->C, environment_->xi, environment_);

		//alg_solve_q_nc();
		//alg_solve_x_nc();
		//double Fiplus1 = F_q_x(environment_->qiplus1, environment_->xiplus1);
		double delta = std::abs(Fiplus1 - Fi);
		if (delta < sigma)
			break;
		else
		{
			environment_->qi = environment_->qiplus1;
			environment_->xi = environment_->xiplus1;
			Fi = Fiplus1;
		}
		idx++;
	}

	refined_vertices = environment_->verts_2d;
	for (int i = 0; i < environment_->imprecise_vertices.size(); i++)
	{
		int vert_id = environment_->imprecise_vertices[i];
		refined_vertices[vert_id][0] = environment_->xiplus1(i, 0);
		refined_vertices[vert_id][1] = environment_->xiplus1(i, 1);
	}
	/*refined_vertices.resize(environment_->xi.rows());
	for (int i = 0; i < environment_->xi.rows(); i++)
	{
		refined_vertices[i][0] = environment_->xiplus1(i, 0);
		refined_vertices[i][1] = environment_->xiplus1(i, 1);
	}*/

	refined_q = environment_->qiplus1;
	std::ofstream qout("..\\matlab\\qi_1.csv");
	qout << environment_->qiplus1.format(CSVFormat);
	qout.close();
	std::ofstream out("..\\matlab\\Au.csv");
	out << environment_->Au.format(CSVFormat);
	out.close();
	out.open("..\\matlab\\Bu.csv");
	out << environment_->Bu.format(CSVFormat);
	out.close();
	out.open("..\\matlab\\Cu.csv");
	out << environment_->Cu.format(CSVFormat);
	out.close();

	return true;
}

/***********************************************************************/
extern"C" { int midaco(long int*, long int*, long int*, long int*, long int*,
	long int*, double*, double*, double*, double*, double*,
	long int*, long int*, double*, double*, long int*,
	long int*, long int*, double*, long int*, char*);}
/***********************************************************************/
extern"C" { int midaco_print(int, long int, long int, long int*, long int*, double*,
	double*, double*, double*, double*, long int, long int,
	long int, long int, long int, double*, double*,
	long int, long int, double*, long int, char*);}

void EquationsSolver::midaco_solve_q()
{
	/* Variable and Workspace Declarations */
	long int o, n, ni, m, me, maxeval, maxtime, printeval, save2file, iflag, istop;
	long int liw, lrw, lpf, i, iw[10000], p = 1; double rw[10000], pf[10000];
	double   f[10], g[1000], x[1000], xl[1000], xu[1000], param[12];
	char key[] = "MIDACO_LIMITED_VERSION___[CREATIVE_COMMONS_BY-NC-ND_LICENSE]";

	/*****************************************************************/
	/***  Step 1: Problem definition  ********************************/
	/*****************************************************************/

	/* STEP 1.A: Problem dimensions
	******************************/
	o = 1; /* Number of objectives                          */
	int Nf = environment_->faces.size();
	n = 3 * Nf; /* Number of variables (in total)                */
	ni = 0; /* Number of integer variables (0 <= ni <= n)    */
	m = num_constraints_rows(); /* Number of constraints (in total)              */
	me = m; /* Number of equality constraints (0 <= me <= m) */

	/* STEP 1.B: Lower and upper bounds 'xl' & 'xu'
	**********************************************/
	for (i = 0; i<n; i++)
	{
		xl[i] = -1.0;
		xu[i] = 1.0;
	}

	/* STEP 1.C: Starting point 'x'
	******************************/
	for (i = 0; i<n; i++)
	{
		x[i] = environment_->qi[i]; /* Here for example: starting point = lower bounds */
	}

	/*****************************************************************/
	/***  Step 2: Choose stopping criteria and printing options   ****/
	/*****************************************************************/

	/* STEP 2.A: Stopping criteria
	*****************************/
	maxeval = 10000;    /* Maximum number of function evaluation (e.g. 1000000)  */
	maxtime = 60 * 60 * 24; /* Maximum time limit in Seconds (e.g. 1 Day = 60*60*24) */

	/* STEP 2.B: Printing options
	****************************/
	printeval = 1000; /* Print-Frequency for current best solution (e.g. 1000) */
	save2file = 1;    /* Save SCREEN and SOLUTION to TXT-files [ 0=NO/ 1=YES]  */

	/*****************************************************************/
	/***  Step 3: Choose MIDACO parameters (FOR ADVANCED USERS)    ***/
	/*****************************************************************/

	param[0] = 0.0;  /* ACCURACY  */
	param[1] = 0.0;  /* SEED      */
	param[2] = 0.0;  /* FSTOP     */
	param[3] = 0.0;  /* ALGOSTOP  */
	param[4] = 0.0;  /* EVALSTOP  */
	param[5] = 0.0;  /* FOCUS     */
	param[6] = 0.0;  /* ANTS      */
	param[7] = 0.0;  /* KERNEL    */
	param[8] = 0.0;  /* ORACLE    */
	param[9] = 0.0;  /* PARETOMAX */
	param[10] = 0.0;  /* EPSILON   */
	param[11] = 0.0;  /* CHARACTER */

	/*****************************************************************/
	/*
	Call MIDACO by Reverse Communication
	*/
	/*****************************************************************/
	/* Workspace length calculation */
	lrw = sizeof(rw) / sizeof(double);
	lpf = sizeof(pf) / sizeof(double);
	liw = sizeof(iw) / sizeof(long int);
	/* Print midaco headline and basic information */
	midaco_print(1, printeval, save2file, &iflag, &istop, &*f, &*g, &*x, &*xl, &*xu,
		o, n, ni, m, me, &*rw, &*pf, maxeval, maxtime, &*param, p, &*key);
	while (istop == 0) /*~~~ Start of the reverse communication loop ~~~*/
	{
		/* Evaluate objective function */
		midaco_q_function(&*f, &*g, &*x);

		/* Call MIDACO */
		midaco(&p, &o, &n, &ni, &m, &me, &*x, &*f, &*g, &*xl, &*xu, &iflag,
			&istop, &*param, &*rw, &lrw, &*iw, &liw, &*pf, &lpf, &*key);
		/* Call MIDACO printing routine */
		midaco_print(2, printeval, save2file, &iflag, &istop, &*f, &*g, &*x, &*xl, &*xu,
			o, n, ni, m, me, &*rw, &*pf, maxeval, maxtime, &*param, p, &*key);
	} /*~~~End of the reverse communication loop ~~~*/
	std::cout << "returned_func_val = " << f[0] << std::endl;
	std::cout << "iflag = " << iflag << std::endl;

	for (i = 0; i < n; i++)
		environment_->qiplus1[i] = x[i];
	std::cout << "qi+1: " << environment_->qiplus1.transpose() << std::endl;
}

void EquationsSolver::madaco_solve_initial_q()
{
	/* Variable and Workspace Declarations */
	long int o, n, ni, m, me, maxeval, maxtime, printeval, save2file, iflag, istop;
	long int liw, lrw, lpf, i, iw[10000], p = 1; double rw[10000], pf[10000];
	double   f[10], g[1000], x[1000], xl[1000], xu[1000], param[12];
	char key[] = "MIDACO_LIMITED_VERSION___[CREATIVE_COMMONS_BY-NC-ND_LICENSE]";

	/*****************************************************************/
	/***  Step 1: Problem definition  ********************************/
	/*****************************************************************/

	/* STEP 1.A: Problem dimensions
	******************************/
	o = 5; /* Number of objectives                          */
	int Nf = environment_->faces.size();
	n = 3 * Nf; /* Number of variables (in total)                */
	ni = 0; /* Number of integer variables (0 <= ni <= n)    */
	m = 0; /* Number of constraints (in total)              */
	me = 0; /* Number of equality constraints (0 <= me <= m) */

	/* STEP 1.B: Lower and upper bounds 'xl' & 'xu'
	**********************************************/
	for (i = 0; i<n; i++)
	{
		xl[i] = -1.0;
		xu[i] = 1.0;
	}

	/* STEP 1.C: Starting point 'x'
	******************************/
	for (i = 0; i<n; i++)
	{
		x[i] = xl[i]; /* Here for example: starting point = lower bounds */
	}

	/*****************************************************************/
	/***  Step 2: Choose stopping criteria and printing options   ****/
	/*****************************************************************/

	/* STEP 2.A: Stopping criteria
	*****************************/
	maxeval = 10000;    /* Maximum number of function evaluation (e.g. 1000000)  */
	maxtime = 60 * 60 * 24; /* Maximum time limit in Seconds (e.g. 1 Day = 60*60*24) */

	/* STEP 2.B: Printing options
	****************************/
	printeval = 1000; /* Print-Frequency for current best solution (e.g. 1000) */
	save2file = 1;    /* Save SCREEN and SOLUTION to TXT-files [ 0=NO/ 1=YES]  */

	/*****************************************************************/
	/***  Step 3: Choose MIDACO parameters (FOR ADVANCED USERS)    ***/
	/*****************************************************************/

	param[0] = 0.0;  /* ACCURACY  */
	param[1] = 0.0;  /* SEED      */
	param[2] = 0.0;  /* FSTOP     */
	param[3] = 0.0;  /* ALGOSTOP  */
	param[4] = 0.0;  /* EVALSTOP  */
	param[5] = 0.0;  /* FOCUS     */
	param[6] = 0.0;  /* ANTS      */
	param[7] = 0.0;  /* KERNEL    */
	param[8] = 0.0;  /* ORACLE    */
	param[9] = 0.0;  /* PARETOMAX */
	param[10] = 0.0;  /* EPSILON   */
	param[11] = 0.0;  /* CHARACTER */

	/*****************************************************************/
	/*
	Call MIDACO by Reverse Communication
	*/
	/*****************************************************************/
	/* Workspace length calculation */
	lrw = sizeof(rw) / sizeof(double);
	lpf = sizeof(pf) / sizeof(double);
	liw = sizeof(iw) / sizeof(long int);
	/* Print midaco headline and basic information */
	midaco_print(1, printeval, save2file, &iflag, &istop, &*f, &*g, &*x, &*xl, &*xu,
		o, n, ni, m, me, &*rw, &*pf, maxeval, maxtime, &*param, p, &*key);
	while (istop == 0) /*~~~ Start of the reverse communication loop ~~~*/
	{
		/* Evaluate objective function */
		midaco_init_q_func(&*f, &*g, &*x);

		/* Call MIDACO */
		midaco(&p, &o, &n, &ni, &m, &me, &*x, &*f, &*g, &*xl, &*xu, &iflag,
			&istop, &*param, &*rw, &lrw, &*iw, &liw, &*pf, &lpf, &*key);
		/* Call MIDACO printing routine */
		midaco_print(2, printeval, save2file, &iflag, &istop, &*f, &*g, &*x, &*xl, &*xu,
			o, n, ni, m, me, &*rw, &*pf, maxeval, maxtime, &*param, p, &*key);
	} /*~~~End of the reverse communication loop ~~~*/
	  /*****************************************************************/
	  // printf("\n Solution f[0] = %f ", f[0]);
	  // printf("\n Solution g[0] = %f ", g[0]);
	  // printf("\n Solution x[0] = %f ", x[0]);            
	  /*****************************************************************/
	/*****************************************************************/
	/***********************  END OF MAIN  ***************************/
	/*****************************************************************/
	for (i = 0; i < n; i++)
		environment_->qi[i] = x[i];
}

//void EquationsSolver::init_q_function(int ndim, ColumnVector & q)
//{
//
//}
//
//void EquationsSolver::q_function(int mode, int ndim, const ColumnVector & q, double & fq, ColumnVector & gx, int & result)
//{
//	fq = 0;
//
//	Eigen::VectorXd eigen_q(q.nrows());
//	for (int i = 0; i < q.nrows(); i++)
//		eigen_q[i] = q(i);
//
//	Eigen::MatrixXd Au, Bu, Cu;
//	form_unfix_constraints(Au, Bu, Cu);
//
//	fq += (Au * eigen_q).squaredNorm();
//	fq += (Bu * eigen_q).squaredNorm();
//	fq += (Cu * eigen_q).squaredNorm();
//}

void EquationsSolver::form_unfix_constraints(Eigen::MatrixXd & Au, Eigen::MatrixXd & Bu, Eigen::MatrixXd & Cu,
	const Eigen::MatrixXd &xi, ConstraintsEnvironment *environment)
{
	int Nf = environment->faces.size();

	/* Form Au */
	int num_rows = 0;
	for (std::vector<int>::iterator it = environment->imprecise_vertices.begin(); 
		it != environment->imprecise_vertices.end(); ++it)
		num_rows += environment->vert_to_face_map[*it].size() - 1;
	Au.resize(num_rows, 3 * Nf);
	Au.setZero();
	int idx = 0;
	for (int i = 0; i < xi.rows(); i++)
	{
		int vert_id = environment->imprecise_vertices[i];

		Eigen::Vector3d x_aug;
		x_aug << xi.row(i).transpose(), -environment->f;

		for (int j = 0; j < environment->vert_to_face_map[vert_id].size() - 1; j++)
		{
			int face_id_1 = environment->vert_to_face_map[vert_id][j] * 3;
			int face_id_2 = environment->vert_to_face_map[vert_id][j + 1] * 3;
			Au.block(idx, face_id_1, 1, 3) = x_aug.transpose();
			Au.block(idx, face_id_2, 1, 3) = -x_aug.transpose();
			idx++;
		}
	}

	/* Form Bu */
	Eigen::Matrix3d K;
	K << -environment->f, 0, 0, 0, -environment->f, 0, 0, 0, 1;
	Eigen::Matrix3d K_inv = K.inverse();
	num_rows = 0;
	for (int i = 0; i < environment->perspective_syms.size(); i++)
		if (environment->perspective_syms[i] >= 0)
			num_rows++;
	num_rows -= environment->precise_sym_faces.size();
	num_rows *= 2;
	Bu.resize(num_rows, 3 * Nf);
	Bu.setZero();
	int count = 0;
	int precise_face = -1;
	int precise_face_id = 0;
	if (!environment->precise_sym_faces.empty())
	{
		precise_face = environment->precise_sym_faces.front();
		precise_face_id = 1;
	}
	for (int i = 0; i < environment->faces.size(); i++)
	{
		int sym_idx = environment->perspective_syms[i];
		if (sym_idx < 0)
			continue;
		if (precise_face >= 0 && i == precise_face)
		{
			if(precise_face_id < environment->precise_sym_faces.size())
				precise_face = environment->precise_sym_faces[precise_face_id++];
			continue;
		}

		const std::vector<int> &circuit = environment->faces[i].const_circuit();
		int N = circuit.size();
		Eigen::MatrixXd Si(3 * N, 9);
		
		for (int k = 0; k < N; k++)
		{
			int x1_idx = (1 + k) % N;
			int vert_idx_1 = circuit[x1_idx];
			int imprecise_idx_1 = find_in_imprecise(vert_idx_1, environment->imprecise_vertices);
			Eigen::Vector3d x1;
			if (imprecise_idx_1 < 0)
				x1 << environment->verts_2d[vert_idx_1], 1.0;
			else
				x1 << xi.row(imprecise_idx_1).transpose(), 1.0;

			int x2_idx = (sym_idx - k) % N;
			if (x2_idx < 0)
				x2_idx += N;
			int vert_idx_2 = circuit[x2_idx];
			int imprecise_idx_2 = find_in_imprecise(vert_idx_2, environment->imprecise_vertices);
			Eigen::Vector3d x2;
			if (imprecise_idx_2 < 0)
				x2 << environment->verts_2d[vert_idx_2], 1.0;
			else
				x2 << xi.row(imprecise_idx_2).transpose(), 1.0;

			Si.block(3 * k, 0, 3, 9) = form_S(x1, x2);
		}

		arma::mat arma_Si(Si.rows(), Si.cols());
		for (int r = 0; r < arma_Si.n_rows; r++)
		{
			for (int c = 0; c < arma_Si.n_cols; c++)
				arma_Si(r, c) = Si(r, c);
		}
		//arma_Si.print("Si: ");
		arma::cx_vec eigval;
		arma::cx_mat eigvec;
		arma::eig_gen(eigval, eigvec, arma_Si.t() * arma_Si);

		//eigval.print("eigval: ");
		//eigvec.print("eigvec: ");
		
		std::vector<double> evals(eigval.size());
		std::vector<int> eval_idx(eigval.size());
		for (int j = 0; j < eigval.size(); j++)
		{
			evals[j] = eigval[j].real();
			eval_idx[j] = j;
		}
		std::sort(eval_idx.begin(), eval_idx.end(), [&evals](int a, int b) {
			return evals[a] < evals[b];
		});
		int num_eig = eigval.size();
		arma::cx_colvec hi = eigvec.col(eval_idx.front());
		arma::mat33 Hi;
		//Eigen::Matrix3d H;
		for (int j = 0; j < 3; j++)
		{
			for (int t = 0; t < 3; t++)
			{
				Hi(j, t) = hi[3 * j + t].real();
				//H(j, t) = hi[3 * j + t].real();
			}
		}

		//Hi.print("Hi: ");

		arma::cx_vec heval;
		arma::cx_mat hevec;
		arma::eig_gen(heval, hevec, Hi);

		//heval.print("heval: ");
		//hevec.print("hevec: ");

		Eigen::Vector3d psx, psl_p1, psl_p2;
		if (std::abs(heval[1].real() - heval[2].real()) < std::abs(heval[1].real() - heval[0].real()) &&
			std::abs(heval[1].real() - heval[2].real()) < std::abs(heval[2].real() - heval[0].real()))
		{
			for (int j = 0; j < 3; j++)
			{
				psx[j] = hevec(j, 0).real();
				psl_p1[j] = hevec(j, 1).real();
				psl_p2[j] = hevec(j, 2).real();
			}
		}
		else if (std::abs(heval[0].real() - heval[2].real()) < std::abs(heval[1].real() - heval[0].real()) &&
			std::abs(heval[0].real() - heval[2].real()) < std::abs(heval[2].real() - heval[1].real()))
		{
			for (int j = 0; j < 3; j++)
			{
				psx[j] = hevec(j, 1).real();
				psl_p1[j] = hevec(j, 0).real();
				psl_p2[j] = hevec(j, 2).real();
			}
		}
		else
		{
			for (int j = 0; j < 3; j++)
			{
				psx[j] = hevec(j, 2).real();
				psl_p1[j] = hevec(j, 0).real();
				psl_p2[j] = hevec(j, 1).real();
			}
		}

		psx /= psx[2];
		psl_p1 /= psl_p1[2];
		psl_p2 /= psl_p2[2];
		Eigen::Vector3d psl = line_equation(psl_p1, psl_p2);

		Eigen::Vector3d ci1 = K_inv * psx;
		Eigen::Vector3d ci2 = ci1.cross(K.transpose() * psl);
		Eigen::MatrixXd Ci(3, 2);
		Ci << ci1, ci2;
		Eigen::MatrixXd Bi(2, 3 * Nf);
		Bi.setZero();
		Bi.block(0, 3 * i, 2, 3) = Ci.transpose();
		Bu.block(2 * count, 0, 2, 3 * Nf) = Bi;
		count++;
	}

	/* Form Cu */
	num_rows = 0;
	for (int i = 0; i < environment->face_parallel_groups.size(); i++)
		num_rows += environment->face_parallel_groups[i].size();
	num_rows -= environment->Cd.rows();
	Cu.resize(num_rows, 3 * Nf);
	Cu.setZero();
	idx = 0;
	for (int i = 0; i < Nf; i++)
	{
		for (std::list<std::pair<int, int>>::iterator pair_it = environment->face_parallel_groups[i].begin();
			pair_it != environment->face_parallel_groups[i].end(); ++pair_it)
		{
			int line_1_id = pair_it->first;
			int line_2_id = pair_it->second;
			Line &line_1 = environment->edges[line_1_id];
			Line &line_2 = environment->edges[line_2_id];

			int imprecise_p1 = find_in_imprecise(line_1.p1(), environment->imprecise_vertices);
			int imprecise_q1 = find_in_imprecise(line_1.p2(), environment->imprecise_vertices);
			int imprecise_p2 = find_in_imprecise(line_2.p1(), environment->imprecise_vertices);
			int imprecise_q2 = find_in_imprecise(line_2.p2(), environment->imprecise_vertices);
			if (imprecise_p1 >= 0 || imprecise_p2 >= 0 || imprecise_q1 >= 0 || imprecise_q2 >= 0)
			{
				Eigen::Vector2d p1, q1, p2, q2;

				if (imprecise_p1 < 0)
					p1 = environment->verts_2d[line_1.p1()];
				else
					p1 = xi.row(imprecise_p1).transpose();
				if (imprecise_q1 < 0)
					q1 = environment->verts_2d[line_1.p2()];
				else
					q1 = xi.row(imprecise_q1).transpose();
				if (imprecise_p2 < 0)
					p2 = environment->verts_2d[line_2.p1()];
				else
					p2 = xi.row(imprecise_p2).transpose();
				if (imprecise_q2 < 0)
					q2 = environment->verts_2d[line_2.p2()];
				else
					q2 = xi.row(imprecise_q2).transpose();

				Eigen::Vector3d l1 = line_equation(p1, q1);
				Eigen::Vector3d l2 = line_equation(p2, q2);

				Eigen::Vector3d v = l1.cross(l2);
				v /= v[2];
				Eigen::Vector3d R = K_inv * v;
				Cu.block(idx, 3 * i, 1, 3) = R.transpose();
				idx++;
			}
		}
	}
}

void EquationsSolver::form_constraints(Eigen::MatrixXd & Au, Eigen::MatrixXd & Bu, Eigen::MatrixXd & Cu, 
	const Eigen::MatrixXd & x, ConstraintsEnvironment * environment)
{
	int Nf = environment->faces.size();

	/* Form Au */
	int num_rows = 0;
	for (std::vector<std::vector<int>>::iterator v_it = environment->vert_to_face_map.begin();
		v_it != environment->vert_to_face_map.end(); ++v_it)
		num_rows += v_it->size() - 1;

	Au.resize(num_rows, 3 * Nf);
	Au.setZero();
	int idx = 0;
	for (int i = 0; i < x.rows(); i++)
	{
		Eigen::Vector3d x_aug;
		x_aug << x.row(i).transpose(), -environment->f;

		for (int j = 0; j < environment->vert_to_face_map[i].size() - 1; j++)
		{
			int face_id_1 = environment->vert_to_face_map[i][j] * 3;
			int face_id_2 = environment->vert_to_face_map[i][j + 1] * 3;
			Au.block(idx, face_id_1, 1, 3) = x_aug.transpose();
			Au.block(idx, face_id_2, 1, 3) = -x_aug.transpose();
			idx++;
		}
	}

	/* Form Bu */
	Eigen::Matrix3d K;
	K << -environment->f, 0, 0, 0, -environment->f, 0, 0, 0, 1;
	Eigen::Matrix3d K_inv = K.inverse();
	num_rows = 0;
	for (int i = 0; i < environment->perspective_syms.size(); i++)
		if (environment->perspective_syms[i] >= 0)
			num_rows++;
	
	num_rows *= 2;
	Bu.resize(num_rows, 3 * Nf);
	Bu.setZero();
	int count = 0;

	for (int i = 0; i < environment->faces.size(); i++)
	{
		int sym_idx = environment->perspective_syms[i];
		if (sym_idx < 0)
		{
			continue;
		}

		const std::vector<int> &circuit = environment->faces[i].const_circuit();
		int N = circuit.size();
		Eigen::MatrixXd Si(3 * N, 9);

		for (int k = 0; k < N; k++)
		{
			int x1_idx = (1 + k) % N;
			int vert_idx_1 = circuit[x1_idx];
			
			Eigen::Vector3d x1;
			x1 << x.row(vert_idx_1).transpose(), 1.0;

			int x2_idx = (sym_idx - k) % N;
			if (x2_idx < 0)
				x2_idx += N;
			int vert_idx_2 = circuit[x2_idx];
			Eigen::Vector3d x2;
			x2 << x.row(vert_idx_2).transpose(), 1.0;

			Si.block(3 * k, 0, 3, 9) = form_S(x1, x2);
		}

		arma::mat arma_Si(Si.rows(), Si.cols());
		//arma_Si.print("Si: ");
		for (int r = 0; r < arma_Si.n_rows; r++)
		{
			for (int c = 0; c < arma_Si.n_cols; c++)
				arma_Si(r, c) = Si(r, c);
		}
		arma::cx_vec eigval;
		arma::cx_mat eigvec;
		arma::eig_gen(eigval, eigvec, arma_Si.t() * arma_Si);

		std::vector<double> evals(eigval.size());
		std::vector<int> eval_idx(eigval.size());
		for (int j = 0; j < eigval.size(); j++)
		{
			evals[j] = eigval[j].real();
			eval_idx[j] = j;
		}
		std::sort(eval_idx.begin(), eval_idx.end(), [&evals](int a, int b) {
			return evals[a] < evals[b];
		});
		arma::cx_colvec hi = eigvec.col(eval_idx.front());
		arma::mat33 Hi;
		for (int j = 0; j < 3; j++)
		{
			for (int t = 0; t < 3; t++)
				Hi(j, t) = hi[3 * j + t].real();
		}

		arma::cx_vec heval;
		arma::cx_mat hevec;
		arma::eig_gen(heval, hevec, Hi);
		Eigen::Vector3d psx, psl_p1, psl_p2;
		if (std::abs(heval[1].real() - heval[2].real()) < std::abs(heval[1].real() - heval[0].real()) &&
			std::abs(heval[1].real() - heval[2].real()) < std::abs(heval[2].real() - heval[0].real()))
		{
			for (int j = 0; j < 3; j++)
			{
				psx[j] = hevec(j, 0).real();
				psl_p1[j] = hevec(j, 1).real();
				psl_p2[j] = hevec(j, 2).real();
			}
		}
		else if (std::abs(heval[0].real() - heval[2].real()) < std::abs(heval[1].real() - heval[0].real()) &&
			std::abs(heval[0].real() - heval[2].real()) < std::abs(heval[2].real() - heval[1].real()))
		{
			for (int j = 0; j < 3; j++)
			{
				psx[j] = hevec(j, 1).real();
				psl_p1[j] = hevec(j, 0).real();
				psl_p2[j] = hevec(j, 2).real();
			}
		}
		else
		{
			for (int j = 0; j < 3; j++)
			{
				psx[j] = hevec(j, 2).real();
				psl_p1[j] = hevec(j, 0).real();
				psl_p2[j] = hevec(j, 1).real();
			}
		}

		psx /= psx[2];
		psl_p1 /= psl_p1[2];
		psl_p2 /= psl_p2[2];
		Eigen::Vector3d psl = line_equation(psl_p1, psl_p2);

		Eigen::Vector3d ci1 = K_inv * psx;
		Eigen::Vector3d ci2 = ci1.cross(K.transpose() * psl);
		Eigen::MatrixXd Ci(3, 2);
		Ci << ci1, ci2;
		Eigen::MatrixXd Bi(2, 3 * Nf);
		Bi.setZero();
		Bi.block(0, 3 * i, 2, 3) = Ci.transpose();

		Bu.block(2 * count, 0, 2, 3 * Nf) = Bi;
		count++;
	}

	/* Form Cu */
	num_rows = 0;
	for (int i = 0; i < environment->face_parallel_groups.size(); i++)
		num_rows += environment->face_parallel_groups[i].size();
	Cu.resize(num_rows, 3 * Nf);
	Cu.setZero();
	idx = 0;
	for (int i = 0; i < Nf; i++)
	{
		for (std::list<std::pair<int, int>>::iterator pair_it = environment->face_parallel_groups[i].begin();
			pair_it != environment->face_parallel_groups[i].end(); ++pair_it)
		{
			int line_1_id = pair_it->first;
			int line_2_id = pair_it->second;
			Line &line_1 = environment->edges[line_1_id];
			Line &line_2 = environment->edges[line_2_id];

			Eigen::Vector2d p1, q1, p2, q2;

			p1 = x.row(line_1.p1()).transpose();
			q1 = x.row(line_1.p2()).transpose();
			p2 = x.row(line_2.p1()).transpose();
			q2 = x.row(line_2.p2()).transpose();

			Eigen::Vector3d l1 = line_equation(p1, q1);
			Eigen::Vector3d l2 = line_equation(p2, q2);

			Eigen::Vector3d v = l1.cross(l2);
			v /= v[2];
			Eigen::Vector3d R = K_inv * v;
			Cu.block(idx, 3 * i, 1, 3) = R.transpose();
			idx++;
		}
	}
}

int EquationsSolver::find_in_imprecise(int vert_id, const std::vector<int> &imprecise_vertices)
{
	int front = 0, back = imprecise_vertices.size() - 1;

	while (front <= back)
	{
		if (vert_id == imprecise_vertices[front])
			return front;
		if (vert_id == imprecise_vertices[back])
			return back;

		int mid = (front + back) / 2;
		if (vert_id == imprecise_vertices[mid])
			return mid;
		if (vert_id < imprecise_vertices[mid])
		{
			front++;
			back = mid - 1;
		}
		else
		{
			front = mid + 1;
			back--;
		}
	}

	return -1;
}

Eigen::MatrixXd EquationsSolver::form_S(const Eigen::Vector3d & v1, const Eigen::Vector3d & v2)
{
	Eigen::MatrixXd s(3, 9);
	float x1 = v1[0], y1 = v1[1], z1 = v1[2];
	float x2 = v2[0], y2 = v2[1], z2 = v2[2];
	s.block<1, 9>(0, 0) << 0, 0, 0, -x2*z1, -y2*z1, -z1*z2, x2*y1, y1*y2, y1 * z2;
	s.block<1, 9>(1, 0) << x2 * z1, y2*z1, z1*z2, 0, 0, 0, -x1*x2, -x1*y2, -x1*z2;
	s.block<1, 9>(2, 0) << -x2*y1, -y1*y2, -y1*z2, x1*x2, x1*y2, x1*z2, 0, 0, 0;

	return s;
}

Eigen::Vector3d EquationsSolver::line_equation(const Eigen::Vector3d & v1, const Eigen::Vector3d & v2)
{
	if (std::abs(v1.x() - v2.x()) < 1e-6)
	{
		if (std::abs(v1.x()) > 1e-6)
			return Eigen::Vector3d(-1.0 / v1.x(), 0, 1.0);
		else
			return Eigen::Vector3d(1.0, 0, -v1.x());
	}

	double slope = (v2.y() - v1.y()) / (v2.x() - v1.x());
	double a = slope;
	double b = -1;
	double c = -slope * v1.x() + v1.y();

	if (std::abs(c) > 1e-8)
	{
		a /= c;
		b /= c;
		c = 1.0;
	}

	return Eigen::Vector3d(a, b, c);
}

Eigen::Vector3d EquationsSolver::line_equation(const Eigen::Vector2d & v1, const Eigen::Vector2d & v2)
{
	if (std::abs(v1.x() - v2.x()) < 1e-6)
	{
		if (std::abs(v1.x()) > 1e-6)
			return Eigen::Vector3d(-1.0 / v1.x(), 0, 1.0);
		else
			return Eigen::Vector3d(1.0, 0, -v1.x());
	}

	double slope = (v2.y() - v1.y()) / (v2.x() - v1.x());
	double a = slope;
	double b = -1;
	double c = -slope * v1.x() + v1.y();

	if (std::abs(c) > 1e-8)
	{
		a /= c;
		b /= c;
		c = 1.0;
	}

	return Eigen::Vector3d(a, b, c);
}

int EquationsSolver::num_constraints_rows()
{
	int num_rows = 0;
	num_rows += environment_->Ad.rows();
	num_rows += environment_->Bd.rows();
	num_rows += environment_->Cd.rows();
	num_rows += environment_->E.rows();

	return num_rows;
}

int EquationsSolver::num_x_equations()
{
	int num = 0;
	num += environment_->A.rows();
	num += environment_->B.rows();
	num += environment_->C.rows();
	return num;
}

int EquationsSolver::num_q_equations()
{
	int num = 0;
	num += num_x_equations();
	num += environment_->G.size();

	return num;
}

int LS_CALLTYPE EquationsSolver::local_sol_log(pLSmodel model, int iLoc, void * cbData)
{
	int iter = 0, niter, biter, siter;
	int *nKKT = (int *)cbData, npass, nbrn;
	double pfeas = 0.0, pobj = 0.0, dfeas = 0.0;
	double bestobj;
	if (iLoc == LSLOC_LOCAL_OPT)
	{
		if (*nKKT == 0) {
			printf(" %5s %11s %11s %11s %10s\n",
				"Iter", "Objective", "Infeas", "Best", "Branches");
		}
		LSgetCallbackInfo(model, iLoc, LS_IINFO_MIP_NLP_ITER, &niter);
		LSgetCallbackInfo(model, iLoc, LS_IINFO_MIP_SIM_ITER, &siter);
		LSgetCallbackInfo(model, iLoc, LS_IINFO_MIP_BAR_ITER, &biter);
		LSgetCallbackInfo(model, iLoc, LS_DINFO_POBJ, &pobj);
		LSgetCallbackInfo(model, iLoc, LS_DINFO_PINFEAS, &pfeas);
		LSgetCallbackInfo(model, iLoc, LS_DINFO_MSW_POBJ, &bestobj);
		LSgetCallbackInfo(model, iLoc, LS_IINFO_MIP_BRANCHCOUNT, &nbrn);
		iter = niter + siter + biter;
		printf(" %5d %11.3f %11.3f %11.3f %10d\n", iter, pobj, pfeas,
			bestobj, nbrn);
		(*nKKT)++;
	}
	else if (iLoc == LSLOC_CONOPT)
	{
		printf("%5s %5s %16s %16s %16s %16s\n",
			"PASS", "ITER", "POBJ", "PINFEAS", "DINFEAS", "BESTOBJ");

		LSgetCallbackInfo(model, iLoc, LS_IINFO_NLP_ITER, &iter);
		LSgetCallbackInfo(model, iLoc, LS_DINFO_POBJ, &pobj);
		LSgetCallbackInfo(model, iLoc, LS_DINFO_PINFEAS, &pfeas);
		LSgetCallbackInfo(model, iLoc, LS_DINFO_DINFEAS, &dfeas);
		printf("%5s %5d %16.5e %16.5e %16.5e %16s\n",
			"", iter, pobj, pfeas, dfeas, "");

	}

	return 0;
}

int CALLBACKTYPE EquationsSolver::Funcalcq(pLSmodel pModel, void * pUserData, int nRow, double * pdX,
	int nJDiff, double dXJBase, double * pdFuncVal, int * pReserved)
{
	ConstraintsEnvironment *cons_env = (ConstraintsEnvironment *)pUserData;

	int    nerr = 0;
	double fq = 0;
	int Nf = cons_env->faces.size();
	Eigen::VectorXd q(3 * Nf);
	for (int i = 0; i < 3 * Nf; i++)
		q[i] = pdX[i];

	//std::cout << "q:\n" << q.transpose() << std::endl;

	int Ad_start = 0;
	int Bd_start = Ad_start + cons_env->Ad.rows();
	int Cd_start = Bd_start + cons_env->Bd.rows();
	int E_start = Cd_start + cons_env->Cd.rows();

	if (nRow == -1)
	{
		fq += (cons_env->Au * q).squaredNorm();
		fq += (cons_env->Bu * q).squaredNorm();
		fq += (cons_env->Cu * q).squaredNorm();

		//if (fq > 1.0e10)
		//{
		//	std::cout << "Au:\n" << Au << std::endl;
		//	std::cout << "Bu:\n" << Bu << std::endl;
		//	std::cout << "Cu:\n" << Cu << std::endl;
		//	std::cout << "q:\n" << q.transpose() << std::endl;
		//	std::cout << "xi:\n" << cons_env->xi << std::endl;
		//	std::cout << "Au * q:\n" << (Au * q).transpose() << std::endl;
		//	std::cout << "Bu * q:\n" << (Bu * q).transpose() << std::endl;
		//	std::cout << "Cu * q:\n" << (Cu * q).transpose() << std::endl;
		//}
		double gsum = 0;
		for (std::vector<std::pair<int, int>>::iterator pair_it = cons_env->G.begin();
			pair_it != cons_env->G.end(); ++pair_it)
		{
			int face_idx_1 = pair_it->first;
			int face_idx_2 = pair_it->second;
			double a1 = pdX[3 * face_idx_1 + 0];
			double a2 = pdX[3 * face_idx_1 + 1];
			double a3 = pdX[3 * face_idx_1 + 2];
			double b1 = pdX[3 * face_idx_2 + 0];
			double b2 = pdX[3 * face_idx_2 + 1];
			double b3 = pdX[3 * face_idx_2 + 2];

			Eigen::Vector3d ni(a1, a2, a3);
			Eigen::Vector3d nj(b1, b2, b3);
			gsum += std::pow(ni.normalized().dot(nj.normalized()), 2);
		}
		fq += gsum;
	}
	else if (nRow >= Ad_start && nRow < Bd_start)
	{
		fq += cons_env->Ad.row(nRow) * q;
	}
	else if (nRow >= Bd_start && nRow < Cd_start)
	{
		fq += cons_env->Bd.row(nRow - Bd_start) * q;
	}
	else if (nRow >= Cd_start && nRow < E_start)
	{
		fq += cons_env->Cd.row(nRow - Cd_start) * q;
	}
	else
	{
		Eigen::MatrixXd e = cons_env->E * q;
		fq += e(0, 0) - FIXED_DEPTH;
	}

	*pdFuncVal = fq;
	//if (fq > 100)
	//{
	//	std::cout << "Au:\n" << cons_env->Au << std::endl;
	//	std::cout << "Bu:\n" << cons_env->Bu << std::endl;
	//	std::cout << "Cu:\n" << cons_env->Cu << std::endl;
	//	std::cout << "q:\n" << q.transpose() << std::endl;
	//	std::cout << "xi:\n" << cons_env->xi << std::endl;
	//}
	return nerr;
}

double EquationsSolver::simple_Funcalcq(const Eigen::VectorXd & q, ConstraintsEnvironment * cons_env)
{
	double fq = 0;
	fq += (cons_env->Au * q).squaredNorm();
	fq += (cons_env->Bu * q).squaredNorm();
	fq += (cons_env->Cu * q).squaredNorm();

	double gsum = 0;
	for (std::vector<std::pair<int, int>>::iterator pair_it = cons_env->G.begin();
		pair_it != cons_env->G.end(); ++pair_it)
	{
		int face_idx_1 = pair_it->first;
		int face_idx_2 = pair_it->second;

		Eigen::Vector3d ni = q.segment(3 * face_idx_1, 3);
		Eigen::Vector3d nj = q.segment(3 * face_idx_2, 3);
		gsum += std::pow(ni.normalized().dot(nj.normalized()), 2);
	}
	fq += gsum;

	return fq;
}

double EquationsSolver::simple_Funcalcq(const alglib::real_1d_array & qarr, ConstraintsEnvironment * cons_env)
{
	double fq = 0;
	Eigen::VectorXd q(qarr.length());
	for (int i = 0; i < qarr.length(); i++)
		q[i] = qarr[i];

	fq += (cons_env->A * q).squaredNorm();
	fq += (cons_env->B * q).squaredNorm();
	fq += (cons_env->C * q).squaredNorm();

	double gsum = 0;
	for (std::vector<std::pair<int, int>>::iterator pair_it = cons_env->G.begin();
		pair_it != cons_env->G.end(); ++pair_it)
	{
		int face_idx_1 = pair_it->first;
		int face_idx_2 = pair_it->second;

		Eigen::Vector3d ni = q.segment(3 * face_idx_1, 3);
		Eigen::Vector3d nj = q.segment(3 * face_idx_2, 3);
		gsum += std::pow(ni.normalized().dot(nj.normalized()), 2);
	}
	fq += gsum;

	return fq;
}

double EquationsSolver::simple_Funcalcx(const Eigen::MatrixXd & x, ConstraintsEnvironment * cons_env)
{
	Eigen::MatrixXd Au, Bu, Cu;
	form_unfix_constraints(Au, Bu, Cu, x, cons_env);

	double fx = 0;
	fx += (Au * cons_env->qiplus1).squaredNorm();
	fx += (Bu * cons_env->qiplus1).squaredNorm();
	fx += (Cu * cons_env->qiplus1).squaredNorm();

	return fx;
}

double EquationsSolver::simple_Funcalcx(const alglib::real_1d_array & xarr, ConstraintsEnvironment * cons_env)
{
	Eigen::MatrixXd x(xarr.length() / 2, 2);
	for (int i = 0; i < x.rows(); i++)
	{
		x(i, 0) = xarr[2 * i];
		x(i, 1) = xarr[2 * i + 1];
	}

	Eigen::MatrixXd Au, Bu, Cu;
	form_constraints(Au, Bu, Cu, x, cons_env);

	double fx = 0;
	fx += (Au * cons_env->qiplus1).squaredNorm();
	fx += (Bu * cons_env->qiplus1).squaredNorm();
	fx += (Cu * cons_env->qiplus1).squaredNorm();

	return fx;
}

int CALLBACKTYPE EquationsSolver::Gradcalcq(pLSmodel pModel, void * pUserData, int nRow, double * pdX, 
	double * lb, double * ub, int nNewPnt, int nNPar, int * parlist, double * partial)
{
	ConstraintsEnvironment *cons_env = (ConstraintsEnvironment *)pUserData;
	Eigen::VectorXd q(cons_env->qi.size());
	for (int i = 0; i < q.size(); i++)
		q[i] = pdX[i];

	assert(q.size() == nNPar);

	for (int i = 0; i < nNPar; i++)
		partial[i] = 0;

	int Ad_start = 0;
	int Bd_start = Ad_start + cons_env->Ad.rows();
	int Cd_start = Bd_start + cons_env->Bd.rows();
	int E_start = Cd_start + cons_env->Cd.rows();

	if (nRow == -1)
	{
		for (std::vector<std::pair<int, int>>::iterator pair_it = cons_env->G.begin();
			pair_it != cons_env->G.end(); ++pair_it)
		{
			int face_idx_1 = pair_it->first;
			int face_idx_2 = pair_it->second;
			Eigen::Vector3d ni = q.segment(3 * face_idx_1, 3);
			Eigen::Vector3d nj = q.segment(3 * face_idx_2, 3);
			Eigen::VectorXd ggrad;
			nonl_grad(ni, nj, ggrad);
			for (int i = 0; i < 3; i++)
			{
				partial[3 * face_idx_1 + i] += ggrad[i];
				partial[3 * face_idx_2 + i] += ggrad[i + 3];
			}
		}
		for (int i = 0; i < q.size(); i++)
		{
			partial[i] += 2 * cons_env->Au.col(i).transpose() * (cons_env->Au * q);
			partial[i] += 2 * cons_env->Bu.col(i).transpose() * (cons_env->Bu * q);
			partial[i] += 2 * cons_env->Cu.col(i).transpose() * (cons_env->Cu * q);
		}
	}
	else if (nRow >= Ad_start && nRow < Bd_start)
	{
		for (int i = 0; i < q.size(); i++)
			partial[i] = cons_env->Ad(nRow, i);
	}
	else if (nRow >= Bd_start && nRow < Cd_start)
	{
		for (int i = 0; i < q.size(); i++)
			partial[i] = cons_env->Bd(nRow - Bd_start, i);
	}
	else if (nRow >= Cd_start && nRow < E_start)
	{
		for (int i = 0; i < q.size(); i++)
			partial[i] = cons_env->Cd(nRow - Cd_start, i);
	}
	else
	{
		for (int i = 0; i < q.size(); i++)
			partial[i] = cons_env->E(0, i);
	}

	return 0;
}

int CALLBACKTYPE EquationsSolver::Funcalcx(pLSmodel pModel, void * pUserData, int nRow, double * pdX, int nJDiff, 
	double dXJBase, double * pdFuncVal, int * pReserved)
{
	ConstraintsEnvironment *cons_env = (ConstraintsEnvironment *)pUserData;
	
	double fx = 0;
	int    nerr = 0;
	Eigen::MatrixXd xi(cons_env->imprecise_vertices.size(), 2);
	for (int i = 0; i < xi.rows(); i++)
	{
		xi(i, 0) = pdX[2 * i];
		xi(i, 1) = pdX[2 * i + 1];
	}
	if (nRow == -1)
	{
		Eigen::MatrixXd Au, Bu, Cu;
		form_unfix_constraints(Au, Bu, Cu, xi, cons_env);

		fx += (Au * cons_env->qiplus1).squaredNorm();
		fx += (Bu * cons_env->qiplus1).squaredNorm();
		fx += (Cu * cons_env->qiplus1).squaredNorm();
	}
	else
	{
		int vert_id = cons_env->imprecise_vertices[nRow];
		Eigen::Vector2d &fixed_vert = cons_env->verts_2d[vert_id];
		fx += (xi.row(nRow).transpose() - fixed_vert).squaredNorm() - 49.0;
	}

	*pdFuncVal = fx;
	return nerr;
}

void EquationsSolver::alg_funcq(const alglib::real_1d_array & qarr, double & func, alglib::real_1d_array & grad, void * ptr)
{
	ConstraintsEnvironment *cons_env = (ConstraintsEnvironment *)ptr;

	func = 0;
	int Nf = cons_env->faces.size();
	Eigen::VectorXd q(3 * Nf);
	for (int i = 0; i < 3 * Nf; i++)
	{
		q[i] = qarr[i];
		grad[i] = 0;
	}

	//std::cout << "q: " << q.transpose() << std::endl;

	func += (cons_env->Au * q).squaredNorm();
	func += (cons_env->Bu * q).squaredNorm();
	func += (cons_env->Cu * q).squaredNorm();

	double gsum = 0;
	for (std::vector<std::pair<int, int>>::iterator pair_it = cons_env->G.begin();
		pair_it != cons_env->G.end(); ++pair_it)
	{
		int face_idx_1 = pair_it->first;
		int face_idx_2 = pair_it->second;
		double a1 = qarr[3 * face_idx_1 + 0];
		double a2 = qarr[3 * face_idx_1 + 1];
		double a3 = qarr[3 * face_idx_1 + 2];
		double b1 = qarr[3 * face_idx_2 + 0];
		double b2 = qarr[3 * face_idx_2 + 1];
		double b3 = qarr[3 * face_idx_2 + 2];

		Eigen::Vector3d ni(a1, a2, a3);
		Eigen::Vector3d nj(b1, b2, b3);
		gsum += std::pow(ni.normalized().dot(nj.normalized()), 2);

		if (gsum != gsum)
		{
			std::cout << "Au:\n" << cons_env->Au << std::endl;
			std::cout << "Bu:\n" << cons_env->Bu << std::endl;
			std::cout << "Cu:\n" << cons_env->Cu << std::endl;
			std::cout << "q: " << q.transpose() << std::endl;
		}
		Eigen::VectorXd ggrad;
		nonl_grad(ni, nj, ggrad);
		for (int i = 0; i < 3; i++)
		{
			grad[3 * face_idx_1 + i] += ggrad[i];
			grad[3 * face_idx_2 + i] += ggrad[i + 3];
		}
	}
	func += gsum;

	//std::cout << "funcq = " << func << std::endl;
	for (int i = 0; i < 3 * Nf; i++)
	{
		grad[i] += 2 * cons_env->Au.col(i).transpose() * (cons_env->Au * q);
		grad[i] += 2 * cons_env->Bu.col(i).transpose() * (cons_env->Bu * q);
		grad[i] += 2 * cons_env->Cu.col(i).transpose() * (cons_env->Cu * q);
	}
}

void EquationsSolver::alg_funcx(const alglib::real_1d_array &xarr, double &fx, void *ptr)
{
	//std::cout << "xarr:" << std::endl;
	//std::cout << xarr.tostring(5) << std::endl;
	ConstraintsEnvironment *cons_env = (ConstraintsEnvironment *)ptr;

	fx = 0;
	Eigen::MatrixXd xi(cons_env->xi.rows(), 2);
	for (int i = 0; i < xi.rows(); i++)
	{
		xi(i, 0) = xarr[2 * i];
		xi(i, 1) = xarr[2 * i + 1];
	}

	Eigen::MatrixXd Au, Bu, Cu;
	form_unfix_constraints(Au, Bu, Cu, xi, cons_env);

	fx += (Au * cons_env->qiplus1).squaredNorm();
	fx += (Bu * cons_env->qiplus1).squaredNorm();
	fx += (Cu * cons_env->qiplus1).squaredNorm();

	if (fx != fx || fx < -1.0e10 || fx > 1.0e10)
	{
		std::cout << "Au:\n" << Au << std::endl;
		std::cout << "Bu:\n" << Bu << std::endl;
		std::cout << "Cu:\n" << Cu << std::endl;
		std::cout << "qi+1: " << cons_env->qiplus1.transpose() << std::endl;
	}
}

void EquationsSolver::alg_fvecq(const alglib::real_1d_array & qarr, alglib::real_1d_array & fi, void * ptr)
{
	ConstraintsEnvironment *cons_env = (ConstraintsEnvironment *)ptr;

	Eigen::VectorXd q(qarr.length());
	for (int i = 0; i < qarr.length(); i++)
		q[i] = qarr[i];

	int idx = 0;
	for (int i = 0; i < cons_env->A.rows(); i++)
	{
		fi[idx] = cons_env->A.row(i) * q;
		idx++;
	}
	for (int i = 0; i < cons_env->B.rows(); i++)
	{
		fi[idx] = cons_env->B.row(i) * q;
		idx++;
	}
	for (int i = 0; i < cons_env->C.rows(); i++)
	{
		fi[idx] = cons_env->C.row(i) * q;
		idx++;
	}

	for (std::vector<std::pair<int, int>>::iterator g_it = cons_env->G.begin();
		g_it != cons_env->G.end(); ++g_it)
	{
		int face_idx_1 = g_it->first;
		int face_idx_2 = g_it->second;
		Eigen::Vector3d n1 = q.segment(3 * face_idx_1, 3);
		Eigen::Vector3d n2 = q.segment(3 * face_idx_2, 3);
		fi[idx++] = n1.normalized().dot(n2.normalized());
	}
}

void EquationsSolver::alg_jacq(const alglib::real_1d_array & qarr, alglib::real_1d_array & fi, alglib::real_2d_array & jac, void * ptr)
{
	ConstraintsEnvironment *cons_env = (ConstraintsEnvironment *)ptr;
	

	Eigen::VectorXd q(qarr.length());
	for (int i = 0; i < qarr.length(); i++)
		q[i] = qarr[i];

	int idx = 0;
	for (int i = 0; i < cons_env->A.rows(); i++)
	{
		fi[idx] = cons_env->A.row(i) * q;
		for (int j = 0; j < q.size(); j++)
			jac[idx][j] = cons_env->A(i, j);
		idx++;
	}
	
	for (int i = 0; i < cons_env->B.rows(); i++)
	{
		fi[idx] = cons_env->B.row(i) * q;
		for (int j = 0; j < q.size(); j++)
			jac[idx][j] = cons_env->B(i, j);
		idx++;
	}

	for (int i = 0; i < cons_env->C.rows(); i++)
	{
		fi[idx] = cons_env->C.row(i) * q;
		for (int j = 0; j < q.size(); j++)
			jac[idx][j] = cons_env->C(i, j);
		idx++;
	}

	for (std::vector<std::pair<int, int>>::iterator g_it = cons_env->G.begin();
		g_it != cons_env->G.end(); ++g_it)
	{
		int face_idx_1 = g_it->first;
		int face_idx_2 = g_it->second;
		Eigen::Vector3d n1 = q.segment(3 * face_idx_1, 3);
		Eigen::Vector3d n2 = q.segment(3 * face_idx_2, 3);
		fi[idx] = n1.normalized().dot(n2.normalized());

		for (int i = 0; i < q.size(); i++)
			jac[idx][i] = 0;

		Eigen::VectorXd ggrad;
		nonl_grad2(n1, n2, ggrad);
		jac[idx][3 * face_idx_1] = ggrad[0];
		jac[idx][3 * face_idx_1 + 1] = ggrad[1];
		jac[idx][3 * face_idx_1 + 2] = ggrad[2];
		jac[idx][3 * face_idx_2] = ggrad[3];
		jac[idx][3 * face_idx_2 + 1] = ggrad[4];
		jac[idx][3 * face_idx_2 + 2] = ggrad[5];
		idx++;
	}
}

void EquationsSolver::alg_fvecx(const alglib::real_1d_array & x, alglib::real_1d_array & fi, void * ptr)
{
	ConstraintsEnvironment *cons_env = (ConstraintsEnvironment *)ptr;

	Eigen::MatrixXd xi(x.length() / 2, 2);
	for (int i = 0; i < xi.rows(); i++)
	{
		xi(i, 0) = x[2 * i];
		xi(i, 1) = x[2 * i + 1];
	}

	Eigen::MatrixXd Au, Bu, Cu;
	form_constraints(Au, Bu, Cu, xi, cons_env);

	int idx = 0;
	for (int i = 0; i < Au.rows(); i++)
	{
		fi[idx] = Au.row(i) * cons_env->qiplus1;
		idx++;
	}
	for (int i = 0; i < Bu.rows(); i++)
	{
		fi[idx] = Bu.row(i) * cons_env->qiplus1;
		idx++;
	}
	for (int i = 0; i < Cu.rows(); i++)
	{
		fi[idx] = Cu.row(i) * cons_env->qiplus1;
		idx++;
	}
}

void EquationsSolver::nonl_grad(const Eigen::Vector3d & n1, const Eigen::Vector3d & n2, Eigen::VectorXd & grad)
{
	grad.resize(6);
	double a1 = n1[0], a2 = n1[1], a3 = n1[2];
	double b1 = n2[0], b2 = n2[1], b3 = n2[2];

	grad[0] = 2.0 * (a1 * b1 + a2 * b2 + a3 * b3) * (b1 * b1 + b2 * b2 + b3 * b3)
		* (b1 * (a1 * a1 + a2 * a2 + a3 * a3) - a1 * (a1 * b1 + a2 * b2 + a3 * b3))
		/ (std::pow(a1 * a1 + a2 * a2 + a3 * a3, 2) * std::pow(b1 * b1 + b2 * b2 + b3 * b3, 2));
	grad[1] = 2.0 * (a1 * b1 + a2 * b2 + a3 * b3) * (b1 * b1 + b2 * b2 + b3 * b3)
		* (b2 * (a1 * a1 + a2 * a2 + a3 * a3) - a2 * (a1 * b1 + a2 * b2 + a3 * b3))
		/ (std::pow(a1 * a1 + a2 * a2 + a3 * a3, 2) * std::pow(b1 * b1 + b2 * b2 + b3 * b3, 2));
	grad[2] = 2.0 * (a1 * b1 + a2 * b2 + a3 * b3) * (b1 * b1 + b2 * b2 + b3 * b3)
		* (b3 * (a1 * a1 + a2 * a2 + a3 * a3) - a3 * (a1 * b1 + a2 * b2 + a3 * b3))
		/ (std::pow(a1 * a1 + a2 * a2 + a3 * a3, 2) * std::pow(b1 * b1 + b2 * b2 + b3 * b3, 2));
	grad[3] = 2.0 * (a1 * b1 + a2 * b2 + a3 * b3) * (b1 * b1 + b2 * b2 + b3 * b3)
		* (a1 * (a1 * a1 + a2 * a2 + a3 * a3) - b1 * (a1 * b1 + a2 * b2 + a3 * b3))
		/ (std::pow(a1 * a1 + a2 * a2 + a3 * a3, 2) * std::pow(b1 * b1 + b2 * b2 + b3 * b3, 2));
	grad[4] = 2.0 * (a1 * b1 + a2 * b2 + a3 * b3) * (b1 * b1 + b2 * b2 + b3 * b3)
		* (a2 * (a1 * a1 + a2 * a2 + a3 * a3) - b2 * (a1 * b1 + a2 * b2 + a3 * b3))
		/ (std::pow(a1 * a1 + a2 * a2 + a3 * a3, 2) * std::pow(b1 * b1 + b2 * b2 + b3 * b3, 2));
	grad[5] = 2.0 * (a1 * b1 + a2 * b2 + a3 * b3) * (b1 * b1 + b2 * b2 + b3 * b3)
		* (a3 * (a1 * a1 + a2 * a2 + a3 * a3) - b3 * (a1 * b1 + a2 * b2 + a3 * b3))
		/ (std::pow(a1 * a1 + a2 * a2 + a3 * a3, 2) * std::pow(b1 * b1 + b2 * b2 + b3 * b3, 2));
}

void EquationsSolver::nonl_grad2(const Eigen::Vector3d & n1, const Eigen::Vector3d & n2, Eigen::VectorXd & grad)
{
	grad.resize(6);
	double x1 = n1[0], y1 = n1[1], z1 = n1[2];
	double x2 = n2[0], y2 = n2[1], z2 = n2[2];

	grad[0] = x2 / (std::sqrt(x1 * x1 + y1 * y1 + z1 * z1)*std::sqrt(x2 *x2 + y2 * y2 + z2 * z2))
		- x1 * (x1 * x2 + y1 * y2 + z1 * z2) /
		(std::pow(x1 * x1 + y1 * y1 + z1 * z1, 3.0 / 2.0) * std::sqrt(x2 * x2 + y2 * y2 + z2 * z2));
	grad[1] = y2 / (std::sqrt(x1 * x1 + y1 * y1 + z1 * z1)*std::sqrt(x2 *x2 + y2 * y2 + z2 * z2))
		- y1 * (x1 * x2 + y1 * y2 + z1 * z2) /
		(std::pow(x1 * x1 + y1 * y1 + z1 * z1, 3.0 / 2.0) * std::sqrt(x2 * x2 + y2 * y2 + z2 * z2));
	grad[2] = z2 / (std::sqrt(x1 * x1 + y1 * y1 + z1 * z1)*std::sqrt(x2 *x2 + y2 * y2 + z2 * z2))
		- z1 * (x1 * x2 + y1 * y2 + z1 * z2) /
		(std::pow(x1 * x1 + y1 * y1 + z1 * z1, 3.0 / 2.0) * std::sqrt(x2 * x2 + y2 * y2 + z2 * z2));
	grad[3] = x1 / (std::sqrt(x1 * x1 + y1 * y1 + z1 * z1)*std::sqrt(x2 *x2 + y2 * y2 + z2 * z2))
		- x2 * (x1 * x2 + y1 * y2 + z1 * z2) /
		(std::pow(x2 * x2 + y2 * y2 + z2 * z2, 3.0 / 2.0) * std::sqrt(x1 * x1 + y1 * y1 + z1 * z1));
	grad[4] = y1 / (std::sqrt(x1 * x1 + y1 * y1 + z1 * z1)*std::sqrt(x2 *x2 + y2 * y2 + z2 * z2))
		- y2 * (x1 * x2 + y1 * y2 + z1 * z2) /
		(std::pow(x2 * x2 + y2 * y2 + z2 * z2, 3.0 / 2.0) * std::sqrt(x1 * x1 + y1 * y1 + z1 * z1));
	grad[5] = z1 / (std::sqrt(x1 * x1 + y1 * y1 + z1 * z1)*std::sqrt(x2 *x2 + y2 * y2 + z2 * z2))
		- z2 * (x1 * x2 + y1 * y2 + z1 * z2) /
		(std::pow(x2 * x2 + y2 * y2 + z2 * z2, 3.0 / 2.0) * std::sqrt(x1 * x1 + y1 * y1 + z1 * z1));
}

void EquationsSolver::midaco_q_function(double * f, double * g, double * qarr)
{
	double func = 0;
	int Nf = environment_->faces.size();
	Eigen::VectorXd q(3 * Nf);
	for (int i = 0; i < 3 * Nf; i++)
	{
		q[i] = qarr[i];
	}

	//std::cout << "q: " << q.transpose() << std::endl;

	func += (environment_->Au * q).squaredNorm();
	func += (environment_->Bu * q).squaredNorm();
	func += (environment_->Cu * q).squaredNorm();

	double gsum = 0;
	for (std::vector<std::pair<int, int>>::iterator pair_it = environment_->G.begin();
		pair_it != environment_->G.end(); ++pair_it)
	{
		int face_idx_1 = pair_it->first;
		int face_idx_2 = pair_it->second;
		double a1 = qarr[3 * face_idx_1 + 0];
		double a2 = qarr[3 * face_idx_1 + 1];
		double a3 = qarr[3 * face_idx_1 + 2];
		double b1 = qarr[3 * face_idx_2 + 0];
		double b2 = qarr[3 * face_idx_2 + 1];
		double b3 = qarr[3 * face_idx_2 + 2];

		Eigen::Vector3d ni(a1, a2, a3);
		Eigen::Vector3d nj(b1, b2, b3);
		gsum += std::pow(ni.normalized().dot(nj.normalized()), 2);
	}
	func += gsum;
	f[0] = func;

	Eigen::VectorXd Adq = environment_->Ad * q;
	Eigen::VectorXd Bdq = environment_->Bd * q;
	Eigen::VectorXd Cdq = environment_->Cd * q;
	int idx = 0;
	for (int i = 0; i < Adq.size(); i++)
		g[idx++] = Adq[i];
	for (int i = 0; i < Bdq.size(); i++)
		g[idx++] = Bdq[i];
	for (int i = 0; i < Cdq.size(); i++)
		g[idx++] = Cdq[i];
	g[idx] = (environment_->E * q)(0, 0) - FIXED_DEPTH;
}

void EquationsSolver::midaco_init_q_func(double * f, double * g, double * qarr)
{
	//double func = 0;
	int Nf = environment_->faces.size();
	Eigen::VectorXd q(3 * Nf);
	for (int i = 0; i < 3 * Nf; i++)
	{
		q[i] = qarr[i];
	}

	f[0] += (environment_->A * q).squaredNorm();
	f[1] += (environment_->B * q).squaredNorm();
	f[2] += (environment_->C * q).squaredNorm();
	f[3] += std::pow((environment_->E * q)(0,0) - FIXED_DEPTH, 2.0);
	std::cout << "A:\n" << environment_->A << std::endl;
	double gsum = 0;
	for (std::vector<std::pair<int, int>>::iterator pair_it = environment_->G.begin();
		pair_it != environment_->G.end(); ++pair_it)
	{
		int face_idx_1 = pair_it->first;
		int face_idx_2 = pair_it->second;
		double a1 = qarr[3 * face_idx_1 + 0];
		double a2 = qarr[3 * face_idx_1 + 1];
		double a3 = qarr[3 * face_idx_1 + 2];
		double b1 = qarr[3 * face_idx_2 + 0];
		double b2 = qarr[3 * face_idx_2 + 1];
		double b3 = qarr[3 * face_idx_2 + 2];

		Eigen::Vector3d ni(a1, a2, a3);
		Eigen::Vector3d nj(b1, b2, b3);
		gsum += std::pow(ni.normalized().dot(nj.normalized()), 2);
	}
	f[4] = gsum;
}

void EquationsSolver::get_sparse_info(std::vector<int>& Abegcol, std::vector<int>& Alencol, std::vector<int>& Arowndx, std::vector<double>& A)
{
	int Nf = environment_->faces.size();
	Eigen::MatrixXd Cons(num_constraints_rows(), 3 * Nf);
	//int row_start = 0;
	//Cons.block(0, 0, environment_->Ad.rows(), 3 * Nf) = environment_->Ad;
	//row_start += environment_->Ad.rows();
	//Cons.block(row_start, 0, environment_->Bd.rows(), 3 * Nf) = environment_->Bd;
	//row_start += environment_->Bd.rows();
	//Cons.block(row_start, 0, environment_->Cd.rows(), 3 * Nf) = environment_->Cd;
	//row_start += environment_->Cd.rows();
	//Cons.block(row_start, 0, environment_->E.rows(), 3 * Nf) = environment_->E.block(0, 0, 1, 3 * Nf);
	Cons << environment_->Ad, environment_->Bd, environment_->Cd, 
		environment_->E;
	Alencol.resize(Cons.cols(), 0);
	Abegcol.resize(Cons.cols() + 1, 0);
	int nz_idx = 0;
	for (int i = 0; i < Cons.cols(); i++)
	{
		bool first_in_col = true;
		int count_col = 0;
		for (int j = 0; j < Cons.rows(); j++)
		{
			double element = Cons(j, i);
			if (std::abs(element) > 1e-6)
			{
				if (first_in_col)
				{
					Abegcol[i] = nz_idx;
					first_in_col = false;
				}
				count_col++;
				Arowndx.push_back(j);
				A.push_back(element);
				nz_idx++;
			}
		}
		Alencol[i] = count_col;
	}
	Abegcol[Cons.cols()] = A.size();
	Arowndx.shrink_to_fit();
	A.shrink_to_fit();
}

void EquationsSolver::form_quadratic_coefficients(const Eigen::MatrixXd & Au, const Eigen::MatrixXd & Bu, 
	const Eigen::MatrixXd & Cu, Eigen::MatrixXd & QA)
{
	int Nf = environment_->faces.size();
	QA.resize(3 * Nf, 3 * Nf);
	QA.setZero();

	for (int i = 0; i < 3 * Nf; i++)
	{
		for (int j = 0; j <= i; j++)
		{
			double QAij = 0;
			for (int k = 0; k < Au.rows(); k++)
				QAij += Au(k, i) * Au(k, j);
			for (int k = 0; k < Bu.rows(); k++)
				QAij += Bu(k, i) * Bu(k, j);
			for (int k = 0; k < Cu.rows(); k++)
				QAij += Cu(k, i) * Cu(k, j);

			QA(i, j) = 2.0 * QAij;
			if(i != j)
				QA(j, i) = 2.0 * QAij;
		}
	}
}

double EquationsSolver::F_q_x(const Eigen::VectorXd & q, const Eigen::MatrixXd & x)
{
	Eigen::MatrixXd Au, Bu, Cu;
	form_constraints(Au, Bu, Cu, x, environment_);

	double F = 0;
	F += (Au * q).squaredNorm();
	F += (Bu * q).squaredNorm();
	F += (Cu * q).squaredNorm();

	double gsum = 0;
	for (std::vector<std::pair<int, int>>::iterator pair_it = environment_->G.begin();
		pair_it != environment_->G.end(); ++pair_it)
	{
		int face_idx_1 = pair_it->first;
		int face_idx_2 = pair_it->second;
		double a1 = q[3 * face_idx_1 + 0];
		double a2 = q[3 * face_idx_1 + 1];
		double a3 = q[3 * face_idx_1 + 2];
		double b1 = q[3 * face_idx_2 + 0];
		double b2 = q[3 * face_idx_2 + 1];
		double b3 = q[3 * face_idx_2 + 2];

		Eigen::Vector3d ni(a1, a2, a3);
		Eigen::Vector3d nj(b1, b2, b3);
		gsum += std::pow(ni.normalized().dot(nj.normalized()), 2);
	}
	F += gsum;

	return F;
}

std::string EquationsSolver::constraints_to_string()
{
	std::string res = "[";
	int Nf = environment_->faces.size();
	for (int i = 0; i < environment_->Ad.rows(); i++)
	{
		res += "[";
		for (int j = 0; j < 3 * Nf; j++)
			res += std::to_string(environment_->Ad(i, j)) + ",";
		res += "0],";
	}

	for (int i = 0; i < environment_->Bd.rows(); i++)
	{
		res += "[";
		for (int j = 0; j < 3 * Nf; j++)
			res += std::to_string(environment_->Bd(i, j)) + ",";
		res += "0],";
	}

	for (int i = 0; i < environment_->Cd.rows(); i++)
	{
		res += "[";
		for (int j = 0; j < 3 * Nf; j++)
			res += std::to_string(environment_->Cd(i, j)) + ",";
		res += "0],";
	}

	res += "[";
	for (int j = 0; j < 3 * Nf; j++)
		res += std::to_string(environment_->E(0, j)) + ",";
	res += std::to_string(FIXED_DEPTH) + "]]";

	//std::ofstream out("..\\constraints.txt");
	//out << res << std::endl;
	//out.close();

	return res;
}

void EquationsSolver::get_alg_constraints(alglib::real_2d_array & constraints, alglib::integer_1d_array &type)
{
	int Nf = environment_->faces.size();
	int num_constraints = num_constraints_rows() * 2;
	constraints.setlength(num_constraints, 3 * Nf + 1);
	type.setlength(num_constraints);

	const double threshold = 0.1;
	int idx = 0;
	for (int i = 0; i < environment_->Ad.rows(); i++)
	{
		int j;
		for (j = 0; j < 3 * Nf; j++)
		{
			constraints[2 * idx][j] = environment_->Ad(i, j);
			constraints[2 * idx + 1][j] = environment_->Ad(i, j);
		}
		constraints[2 * idx][j] = threshold;
		constraints[2 * idx + 1][j] = -threshold;
		type[2 * idx] = -1;
		type[2 * idx + 1] = 1;
		idx++;
	}

	for (int i = 0; i < environment_->Bd.rows(); i++)
	{
		int j;
		for (j = 0; j < 3 * Nf; j++)
		{
			constraints[2 * idx][j] = environment_->Bd(i, j);
			constraints[2 * idx + 1][j] = environment_->Bd(i, j);
		}
		constraints[2 * idx][j] = threshold;
		constraints[2 * idx + 1][j] = -threshold;
		type[2 * idx] = -1;
		type[2 * idx + 1] = 1;
		idx++;
	}

	for (int i = 0; i < environment_->Cd.rows(); i++)
	{
		int j;
		for (j = 0; j < 3 * Nf; j++)
		{
			constraints[2 * idx][j] = environment_->Cd(i, j);
			constraints[2 * idx + 1][j] = environment_->Cd(i, j);
		}
		constraints[2 * idx][j] = threshold;
		constraints[2 * idx + 1][j] = -threshold;
		type[2 * idx] = -1;
		type[2 * idx + 1] = 1;
		idx++;
	}

	int j;
	for (j = 0; j < 3 * Nf; j++)
	{
		constraints[2 * idx][j] = environment_->E(0, j);
		constraints[2 * idx + 1][j] = environment_->E(0, j);
	}
	constraints[2 * idx][j] = FIXED_DEPTH + threshold;
	constraints[2 * idx + 1][j] = FIXED_DEPTH - threshold;
	type[2 * idx] = -1;
	type[2 * idx + 1] = 1;
}

void EquationsSolver::
get_alg_strict_constraints(alglib::real_2d_array & constraints, alglib::integer_1d_array & type)
{
	int Nf = environment_->faces.size();
	int num_constraints = num_constraints_rows();
	constraints.setlength(num_constraints, 3 * Nf + 1);
	type.setlength(num_constraints);

	int idx = 0;
	for (int i = 0; i < environment_->Ad.rows(); i++)
	{
		int j;
		for (j = 0; j < 3 * Nf; j++)
		{
			constraints[idx][j] = environment_->Ad(i, j);
		}
		constraints[idx][j] = 0;
		type[idx] = 0;
		idx++;
	}

	for (int i = 0; i < environment_->Bd.rows(); i++)
	{
		int j;
		for (j = 0; j < 3 * Nf; j++)
		{
			constraints[idx][j] = environment_->Bd(i, j);
		}
		constraints[idx][j] = 0;
		type[idx] = 0;
		idx++;
	}

	for (int i = 0; i < environment_->Cd.rows(); i++)
	{
		int j;
		for (j = 0; j < 3 * Nf; j++)
		{
			constraints[idx][j] = environment_->Cd(i, j);
		}
		constraints[idx][j] = 0;
		type[idx] = 0;
		idx++;
	}

	int j;
	for (j = 0; j < 3 * Nf; j++)
	{
		constraints[idx][j] = environment_->E(0, j);
	}
	constraints[idx][j] = FIXED_DEPTH;
	type[idx] = 0;
}

std::string EquationsSolver::matrix_to_string(const Eigen::MatrixXd & mat)
{
	std::string res = "[";
	if (mat.rows() == 1 || mat.cols() == 1)
	{
		int i;
		if (mat.rows() == 1)
		{
			for (i = 0; i < mat.size() - 1; i++)
				res += std::to_string(mat(0, i)) + ",";
			res += std::to_string(mat(0, i)) + "]";
		}
		if (mat.cols() == 1)
		{
			for (i = 0; i < mat.size() - 1; i++)
				res += std::to_string(mat(i, 0)) + ",";
			res += std::to_string(mat(i, 0)) + "]";
		}
		return res;
	}
	else
	{
		for (int i = 0; i < mat.rows(); i++)
		{
			res += "[";
			int j = 0;
			for (j = 0; j < mat.cols() - 1; j++)
				res += std::to_string(mat(i, j)) + ",";
			res += std::to_string(mat(i, j)) + "]";
			if (i < mat.rows() - 1)
				res += ",";
		}

		res += "]";
		//std::ofstream out("..\\QAstr.txt");
		//out << res << std::endl;
		return res;
	}
}

void EquationsSolver::print_constraints_func_values(const Eigen::VectorXd & q)
{
	std::ofstream out("..\\constraints_values.csv");
	std::cout << "Constraints functions values:" << std::endl;
	Eigen::VectorXd Adq = environment_->Ad * q;
	for (int i = 0; i < Adq.size(); i++)
		std::cout << Adq[i] << " ";
	std::cout << std::endl;
	Eigen::VectorXd Bdq = environment_->Bd * q;
	for (int i = 0; i < Bdq.size(); i++)
		std::cout << Bdq[i] << " ";
	std::cout << std::endl;
	Eigen::VectorXd Cdq = environment_->Cd * q;
	for (int i = 0; i < Cdq.size(); i++)
		std::cout << Cdq[i] << " ";
	std::cout << std::endl;

	int max_rows = std::max(Adq.size(), std::max(Bdq.size(), Cdq.size()));
	for (int i = 0; i < max_rows; i++)
	{
		if (i < Adq.size())
			out << Adq[i];
		out << ",";
		if (i < Bdq.size())
			out << Bdq[i];
		out << ",";
		if (i < Cdq.size())
			out << Cdq[i];
		out << std::endl;
	}
	out.close();
}

//std::string EquationsSolver::matrix_to_string(const Eigen::VectorXd & vec)
//{
//	std::string res = "[";
//	int i = 0;
//	for (i = 0; i < vec.size() - 1; i++)
//		res += std::to_string(vec[i]) + ",";
//	res += std::to_string(vec[i]) + "]";
//	return res;
//}

ConstraintsEnvironment::ConstraintsEnvironment()
{
	A.setZero();
	B.setZero();
	C.setZero();
	E.setZero();
	Ad.setZero();
	Bd.setZero();
	Cd.setZero();
	xi.setZero();
	qi.setZero();
	xiplus1.setZero();
	qiplus1.setZero();
	f = 1000.0;
}

ConstraintsEnvironment::ConstraintsEnvironment(const Eigen::MatrixXd & _A, const Eigen::MatrixXd _B,
	const Eigen::MatrixXd & _C, const Eigen::MatrixXd & _E, const std::vector<std::pair<int, int>>& _G,
	const Eigen::MatrixXd & _Ad, const Eigen::MatrixXd & _Bd, const Eigen::MatrixXd & _Cd,
	const std::vector<Eigen::Vector2d>& _verts_2d, const std::vector<Line>& _edges, const std::vector<PlanarFace> _faces,
	const std::vector<std::vector<int>>& _vert_to_face_map, const std::vector<int>& _imprecise_vertices,
	const std::vector<int>& _precise_sym_faces, const std::vector<int>& _perspective_syms,
	const std::vector<std::list<std::pair<int, int>>>& _face_parallel_groups, double _f)
{
	A = _A;
	B = _B;
	C = _C;
	G.resize(_G.size());
	for (int i = 0; i < _G.size(); i++)
	{
		G[i].first = _G[i].first;
		G[i].second = _G[i].second;
	}
	E = _E;

	Ad = _Ad;
	Bd = _Bd;
	Cd = _Cd;

	verts_2d.resize(_verts_2d.size());
	for (int i = 0; i < _verts_2d.size(); i++)
	{
		verts_2d[i][0] = _verts_2d[i][0];
		verts_2d[i][1] = _verts_2d[i][1];
	}

	edges.resize(_edges.size());
	for (int i = 0; i < _edges.size(); i++)
	{
		edges[i].setP1(_edges[i].p1());
		edges[i].setP2(_edges[i].p2());
	}

	faces.resize(_faces.size());
	int idx = 0;
	for (std::vector<PlanarFace>::const_iterator f_it = _faces.begin(); f_it != _faces.end(); ++f_it)
	{
		faces[idx++] = *f_it;
	}

	vert_to_face_map.resize(_vert_to_face_map.size());
	for (int i = 0; i < _vert_to_face_map.size(); i++)
	{
		vert_to_face_map[i].resize(_vert_to_face_map[i].size());
		for (int j = 0; j < _vert_to_face_map[i].size(); j++)
		{
			vert_to_face_map[i][j] = _vert_to_face_map[i][j];
		}
	}

	imprecise_vertices.resize(_imprecise_vertices.size());
	for (int i = 0; i < _imprecise_vertices.size(); i++)
		imprecise_vertices[i] = _imprecise_vertices[i];

	precise_sym_faces.resize(_precise_sym_faces.size());
	for (int i = 0; i < _precise_sym_faces.size(); i++)
		precise_sym_faces[i] = _precise_sym_faces[i];

	perspective_syms.resize(_perspective_syms.size());
	for (int i = 0; i < _perspective_syms.size(); i++)
		perspective_syms[i] = _perspective_syms[i];

	face_parallel_groups.resize(_face_parallel_groups.size());
	idx = 0;
	for (std::vector<std::list<std::pair<int, int>>>::const_iterator it = _face_parallel_groups.begin();
		it != _face_parallel_groups.end(); ++it)
	{
		for (std::list<std::pair<int, int>>::const_iterator pair_it = it->begin();
			pair_it != it->end(); ++pair_it)
			face_parallel_groups[idx].push_back(std::pair<int, int>(pair_it->first, pair_it->second));
		idx++;
	}

	f = _f;

	xi.resize(imprecise_vertices.size(), 2);
	xiplus1.resize(imprecise_vertices.size(), 2);
	for (int i = 0; i < imprecise_vertices.size(); i++)
	{
		int vert_id = imprecise_vertices[i];
		xi(i, 0) = verts_2d[vert_id].x();
		xi(i, 1) = verts_2d[vert_id].y();
		xiplus1(i, 0) = verts_2d[vert_id].x();
		xiplus1(i, 1) = verts_2d[vert_id].y();
	}

	qi.resize(3 * faces.size(), 1);
	qi.setOnes();
	qiplus1.resize(3 * faces.size(), 1);
	qiplus1.setOnes();
}
