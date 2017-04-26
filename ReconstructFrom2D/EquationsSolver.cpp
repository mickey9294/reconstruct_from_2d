#include "EquationsSolver.h"



EquationsSolver::EquationsSolver()
{
}


EquationsSolver::~EquationsSolver()
{
}

void EquationsSolver::solve(int Nf, int Nv, 
	std::vector<Eigen::Vector2f> &refined_vertices, Eigen::VectorXf &refined_q)
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

	engEvalString(ep, "cd \'D:\\Libraries\\matlab_tools\\broyden\';");

	/* Reconstruct the rough 3D shape */
	std::string rough_recon_cmd = "solve_O3dr(" + std::to_string(Nf) + ");";
	engEvalString(ep, rough_recon_cmd.c_str());

	/* Constraint refinement and joint optimization */
	engEvalString(ep, "solve_q_x();");

	/* Load the result of optimization */
	refined_vertices.resize(Nv);
	std::ifstream in;
	in.open("D:\\Libraries\\matlab_tools\\broyden\\xi+1.csv");
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

	in.open("D:\\Libraries\\matlab_tools\\broyden\\qi+1.csv");
	if (in.is_open())
	{
		refined_q.resize(3 * Nf);
		std::string line;
		for (int i = 0; i < 3 * Nf; i++)
		{
			std::getline(in, line);
			refined_q[i] = std::stof(line);
		}

		in.close();
	}
}
