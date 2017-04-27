#include "ConstraintsGenerator.h"

const float ConstraintsGenerator::t = 100;

ConstraintsGenerator::ConstraintsGenerator(QObject *parent)
	: QObject(parent)
{
}

ConstraintsGenerator::ConstraintsGenerator(const std::string &image_path, float w, float h, const std::list<QPointF>& vertices,
	const std::list<Line>& edges, const std::vector<std::vector<int>> & face_circuits, const std::list<std::vector<int>> &parallel_group,
	QObject *parent)
	: QObject(parent)
{
	std::shared_ptr<CameraClibrator> calibrator(new CameraClibrator(vertices, edges, parallel_group, w, h));

	calibrator->calibrate(focal_length_, primary_point_);
	Z0 = -2 * focal_length_;

	width_ = w;
	height_ = h;

	calib_mat_.setIdentity();
	calib_mat_(0, 0) = focal_length_;
	calib_mat_(1, 1) = focal_length_;

	vertices_.resize(vertices.size());
	edges_.resize(edges.size());

	int idx = 0;
	for (std::list<QPointF>::const_iterator it = vertices.begin(); it != vertices.end(); ++it)
	{
		vertices_[idx][0] = it->x() - primary_point_.x();
		vertices_[idx++][1] = primary_point_.y() - it->y();
	}

	idx = 0;
	for (std::list<Line>::const_iterator it = edges.begin(); it != edges.end(); ++it)
	{
		edges_[idx].setP1(it->p1());
		edges_[idx++].setP2(it->p2());
	}

	idx = 0;
	faces_.reserve(face_circuits.size());
	for (std::vector<std::vector<int>>::const_iterator it = face_circuits.begin();
		it != face_circuits.end(); ++it)
	{
		faces_.push_back(PlanarFace(idx++, *it));
	}

	map_verts_to_face(faces_, vertices_.size());

	set_parallel_groups(parallel_group);

	map_edges_to_face();
}


ConstraintsGenerator::~ConstraintsGenerator()
{
}

void ConstraintsGenerator::add_constraints(std::vector<Eigen::Vector2f> &refined_vertices, Eigen::VectorXf &refined_q)
{
	output_environment();

	add_connectivity_constraint();
	add_perspective_symmetry_constraint();
	add_fixing_vertex_contraint();
	add_line_parallelism_constrain();
	add_orhogonal_corner_constraint();

	output_constraints();

	emit report_status("Adding constraints done.");

	if (!equations_solver_)
		equations_solver_.reset(new EquationsSolver());

	equations_solver_->solve(faces_.size(), vertices_.size(), refined_vertices, refined_q);

	update_environment(refined_vertices);

	emit report_status("Constraint refinement and joint optimization done.");

	Reconstructor recon(focal_length_);
	recon.reconstruct(vertices_, refined_q, vert_to_face_map_, faces_);

	emit report_status("Reconstruction done.");
}

void ConstraintsGenerator::add_connectivity_constraint()
{
	int num_vertices = vertices_.size();
	int num_faces = faces_.size();

	std::list<Eigen::MatrixXf> As;
	int num_rows = 0;
	for (int i = 0; i < num_vertices; i++)
	{
		Eigen::Vector3f homo_vert;
		homo_vert.head(2) = vertices_[i];
		homo_vert[2] = -focal_length_;

		std::vector<int> &related_faces_id = vert_to_face_map_[i];
		int num_related_faces = related_faces_id.size();
		num_rows += num_related_faces - 1;

		if (num_rows > 0)
		{
			Eigen::MatrixXf Ai(num_related_faces - 1, 3 * num_faces);
			Ai.setZero();

			for (int j = 0; j < num_related_faces - 1; j++)
			{
				int face_id = related_faces_id[j];
				int next_face_id = related_faces_id[j + 1];
				Ai.block<1, 3>(j, 3 * face_id) = homo_vert.transpose();
				Ai.block<1, 3>(j, 3 * next_face_id) = -homo_vert.transpose();
			}

			As.push_back(Ai);
		}
	}

	A_.resize(num_rows, 3 * num_faces);
	int start_row = 0;
	for (std::list<Eigen::MatrixXf>::iterator A_it = As.begin(); A_it != As.end(); ++A_it)
	{
		A_.block(start_row, 0, A_it->rows(), 3 * num_faces) = *A_it;
		start_row += A_it->rows();
	}
}

void ConstraintsGenerator::add_perspective_symmetry_constraint()
{
	std::ofstream out("D:\\Libraries\\matlab_tools\\broyden\\circuits.txt");
	if (out.is_open())
	{
		for (std::vector<PlanarFace>::iterator f_it = faces_.begin(); f_it != faces_.end(); ++f_it)
			out << f_it->circuit_string() << std::endl;

		out.close();
	}

	Engine *ep;
	if (!(ep = engOpen("\0"))) //启动matlab 引擎
	{
		std::cerr << "Initilizing Reconstructor failed." << std::endl;
		return;
	}
	else
		engSetVisible(ep, false); // 设置窗口不可见
	
	engEvalString(ep, "cd \'D:\\Libraries\\matlab_tools\\broyden\';");
	std::string sym_detect_cmd = "detect_perspective_syms(" + std::to_string(faces_.size()) + ");";
	engEvalString(ep, sym_detect_cmd.c_str());

	engClose(ep);

	std::ifstream in("D:\\Libraries\\matlab_tools\\broyden\\B.csv");
	if (in.is_open())
	{
		std::list<Eigen::VectorXf> rows;
		std::string line;
		std::vector<std::string> line_split;
		while (!in.eof())
		{
			std::getline(in, line);
			if (line.length() > 0)
			{
				boost::split(line_split, line, boost::is_any_of(","), boost::token_compress_on);
				Eigen::VectorXf row(line_split.size());
				for (int i = 0; i < line_split.size(); i++)
					row[i] = std::stof(line_split[i]);

				rows.push_back(row);
			}
		}

		B_.resize(rows.size(), 3 * faces_.size());
		int row_idx = 0;
		for (std::list<Eigen::VectorXf>::iterator row_it = rows.begin(); row_it != rows.end(); ++row_it, ++row_idx)
			B_.row(row_idx) = *row_it;

		in.close();
	}
}

void ConstraintsGenerator::add_fixing_vertex_contraint()
{
	Eigen::Vector3f x0;
	x0.head(2) = vertices_[faces_.front().const_circuit().front()];
	x0[2] = -focal_length_;

	int num_faces = faces_.size();

	E_.setZero(1, 3 * num_faces);
	E_.block<1, 3>(0, 0) = x0.transpose();
}

void ConstraintsGenerator::add_line_parallelism_constrain()
{
	std::vector<int> edge_parallel_group_map;
	map_edges_to_parallel_group(edge_parallel_group_map);

	int Nf = faces_.size();
	lines_parallel_to_faces_.resize(Nf);

	std::list<Eigen::VectorXf> Crows;

	for (std::vector<PlanarFace>::iterator face_it = faces_.begin(); face_it != faces_.end(); ++face_it)
	{
		int face_id = face_it->id();
		const std::list<int> & edges_list = face_it->const_edges();

		std::unordered_set<int> used_parallel_groups;

		for (std::list<int>::const_iterator e_it = edges_list.begin(); e_it != edges_list.end(); ++e_it)
		{
			int line_id = *e_it;
			int parallel_group_id = edge_parallel_group_map[line_id];

			if (used_parallel_groups.find(parallel_group_id) != used_parallel_groups.end())
				continue;
			else
				used_parallel_groups.insert(parallel_group_id);

			if (parallel_group_id >= 0)
			{
				std::vector<int> &parallel_group = parallel_groups_[parallel_group_id];
				if (parallel_group.size() < 3)
					continue;

				for (int i = 0; i < parallel_group.size(); i++)
				{
					int paraline_1 = parallel_group[i];
					if (paraline_1 != line_id && !is_in_face(paraline_1, face_id))
					{
						for (int j = i + 1; j < parallel_group.size(); j++)
						{
							int paraline_2 = parallel_group[j];
							if (paraline_2 != line_id && !is_in_face(paraline_2, face_id))
							{
								lines_parallel_to_faces_[face_id].push_back(std::pair<int, int>(paraline_1, paraline_2));

								Eigen::VectorXf Crow(3 * Nf);
								Crow.setZero();

								Eigen::Vector3f line_1 = get_line_equation(paraline_1);
								Eigen::Vector3f line_2 = get_line_equation(paraline_2);
								Eigen::Vector3f v = line_1.cross(line_2);
								assert(std::abs(v[2]) > 1e-4);
								v[0] /= v[2];
								v[1] /= v[2];
								v[2] = 1.0;

								Eigen::Vector3f R = calib_mat_.inverse() * v;
								Crow.segment(3 * face_id, 3) = R;

								Crows.push_back(Crow);
							}
						}
					}
				}
			}
		}
	}

	C_.resize(Crows.size(), 3 * Nf);
	int row_idx = 0;
	for (std::list<Eigen::VectorXf>::iterator row_it = Crows.begin(); row_it != Crows.end(); ++row_it, ++row_idx)
		C_.block(row_idx, 0, 1, 3 * Nf) = row_it->transpose();

	//std::cout << "C:\n" << C_ << std::endl;
}

void ConstraintsGenerator::add_orhogonal_corner_constraint()
{
	std::vector<std::vector<int>> vert_to_edge_map;
	map_verts_to_edge(vert_to_edge_map);

	std::list<std::pair<int, int>> orthogonal_faces;
	std::unordered_set<int> added_faces_pair;

	for (int i = 0; i < vertices_.size(); i++)
	{
		if (vert_to_edge_map[i].size() == 3)
		{
			std::vector<Eigen::Vector2f> lines(6);
			for (int j = 0; j < 3; j++)
			{
				Eigen::Vector2f line_direction = get_line_direction(vert_to_edge_map[i][j]);
				lines[2 * j] = line_direction;
				lines[2 * j + 1] = -line_direction;
			}

			bool ortho_possible = true;
			for (int j = 0; j < 2; j++)
			{
				for (int k = 2; k < 4; k++)
				{
					for (int l = 4; l < 6; l++)
					{
						if (lines[j].dot(lines[k]) > 0
							&& lines[j].dot(lines[l]) > 0
							&& lines[k].dot(lines[l]) > 0)
						{
							ortho_possible = false;
							break;
						}
					}
					if (!ortho_possible)
						break;
				}
				if (!ortho_possible)
					break;
			}

			if (ortho_possible)
			{
				std::vector<int> &related_faces = vert_to_face_map_[i];
				if (related_faces.size() > 1)
				{
					int num_pairs = factorial(related_faces.size()) / 2;
					std::vector<std::pair<int, int>> face_pairs(num_pairs);
					int pair_count = 0;
					for (int j = 0; j < related_faces.size(); j++)
					{
						int face_id_1 = related_faces[j];
						for (int k = j + 1; k < related_faces.size(); k++)
						{
							int face_id_2 = related_faces[k];
							assert(face_id_1 != face_id_2);
							face_pairs[pair_count].first = std::min(face_id_1, face_id_2);
							face_pairs[pair_count].second = std::max(face_id_1, face_id_2);
							pair_count++;
						}
					}

					for (int j = 0; j < face_pairs.size(); j++)
					{
						std::pair<int, int> &face_pair = face_pairs[j];
						int pair_number = std::pow(2, face_pair.first) + std::pow(3, face_pair.second);
						if (added_faces_pair.find(pair_number) == added_faces_pair.end())
						{
							orthogonal_faces.push_back(face_pair);
							added_faces_pair.insert(pair_number);
						}
					}
				}
			}
		}
	}

	G_.resize(orthogonal_faces.size());
	int idx = 0;
	for (std::list<std::pair<int, int>>::iterator pair_it = orthogonal_faces.begin();
		pair_it != orthogonal_faces.end(); ++pair_it, ++idx)
	{
		G_[idx].first = pair_it->first;
		G_[idx].second = pair_it->second;
	}
}

void ConstraintsGenerator::set_parallel_groups(const std::list<std::vector<int>>& parallel_groups)
{
	parallel_groups_.reserve(parallel_groups.size());
	for (std::list<std::vector<int>>::const_iterator g_it = parallel_groups.begin(); g_it != parallel_groups.end(); ++g_it)
	{
		std::vector<int> group = *g_it;
		parallel_groups_.push_back(group);
	}
}

float ConstraintsGenerator::get_focal_length() const
{
	return focal_length_;
}

void ConstraintsGenerator::map_verts_to_face(const std::vector<PlanarFace>& faces, int num_vertices)
{
	vert_to_face_map_.clear();
	vert_to_face_map_.resize(num_vertices);

	for (std::vector<PlanarFace>::const_iterator it = faces.begin();
		it != faces.end(); ++it)
	{
		int id = it->id();
		const std::vector<int> &circuit = it->const_circuit();

		for (std::vector<int>::const_iterator jt = circuit.begin(); jt != circuit.end(); ++jt)
			vert_to_face_map_[*jt].push_back(id);
	}

	for (std::vector<std::vector<int>>::iterator it = vert_to_face_map_.begin();
		it != vert_to_face_map_.end(); ++it)
		it->shrink_to_fit();
}

Eigen::MatrixXf ConstraintsGenerator::form_S(const Eigen::Vector3f & v1, const Eigen::Vector3f & v2)
{
	Eigen::MatrixXf s(3, 9);
	float x1 = v1[0], y1 = v1[1], z1 = v1[2];
	float x2 = v2[0], y2 = v2[1], z2 = v2[2];
	s.block<1, 9>(0, 0) << 0, 0, 0, -x2*z1, -y2*z1, -z1*z2, x2*y1, y1*y2, y1 * z2;
	s.block<1, 9>(1, 0) << x2 * z1, y2*z1, z1*z2, 0, 0, 0, -x1*x2, -x1*y2, -x1*z2;
	s.block<1, 9>(2, 0) << -x2*y1, -y1*y2, -y1*z2, x1*x2, x1*y2, x1*z2, 0, 0, 0;
	//s.block<1, 9>(3, 0) << 0, 0, 0, -x1*z2, -y1*z2, -z1*z2, x1*y2, y1*y2, y2*z1;
	//s.block<1, 9>(4, 0) << x1*z2, y1*z2, z1*z2, 0, 0, 0, -x1*x2, -x2*y1, -x2*z1;
	//s.block<1, 9>(5, 0) << -x1*y2, -y1*y2, -y2*z1, x1*x2, x2*y1, x2*z1, 0, 0, 0;

	return s;
}

void ConstraintsGenerator::map_edges_to_face()
{
	edge_to_face_map_.resize(edges_.size());

	for (std::vector<PlanarFace>::iterator face_it = faces_.begin(); face_it != faces_.end(); ++face_it)
	{
		const std::vector<int> &circuit = face_it->const_circuit();
		int circuit_size = circuit.size();

		for (int i = 0; i < circuit_size; i++)
		{
			int edge_id = get_line_id(circuit[i], circuit[(i + 1) % circuit_size]);
			face_it->add_edge(edge_id);
			edge_to_face_map_[edge_id].push_back(face_it->id());
		}
	}
}

int ConstraintsGenerator::get_line_id(int v1, int v2)
{
	for (int i = 0; i < edges_.size(); i++)
	{
		if (edges_[i].same_line(v1, v2))
			return i;
	}

	return -1;
}

bool ConstraintsGenerator::in_same_face(int line_id_1, int line_id_2)
{
	std::list<int> &related_faces_1 = edge_to_face_map_[line_id_1];
	std::list<int> &related_faces_2 = edge_to_face_map_[line_id_2];
	for (std::list<int>::iterator it = related_faces_1.begin(); it != related_faces_1.end(); ++it)
	{
		for (std::list<int>::iterator jt = related_faces_2.begin(); jt != related_faces_2.end(); ++jt)
		{
			if (*it == *jt)
				return true;
		}
	}
	return false;
}

bool ConstraintsGenerator::is_in_face(int line_id, int face_id)
{
	std::list<int> &related_faces = edge_to_face_map_[line_id];
	for (std::list<int>::iterator it = related_faces.begin(); it != related_faces.end(); ++it)
	{
		if (face_id == *it)
			return true;
	}

	return false;
}

void ConstraintsGenerator::map_edges_to_parallel_group(std::vector<int>& edge_parallel_group_map)
{
	if (parallel_groups_.empty())
		return;

	edge_parallel_group_map.resize(edges_.size(), -1);

	int group_idx = 0;
	for (std::vector<std::vector<int>>::iterator group_it = parallel_groups_.begin();
		group_it != parallel_groups_.end(); ++group_it, ++group_idx)
	{
		for (std::vector<int>::iterator e_it = group_it->begin(); e_it != group_it->end(); ++e_it)
			edge_parallel_group_map[*e_it] = group_idx;
	}
}

Eigen::Vector3f ConstraintsGenerator::get_line_equation(int line_id)
{
	int v1_idx = edges_[line_id].p1();
	int v2_idx = edges_[line_id].p2();
	Eigen::Vector2f &v1 = vertices_[v1_idx];
	Eigen::Vector2f &v2 = vertices_[v2_idx];

	if (std::abs(v1.x() - v2.x()) < 1e-4)
	{
		return Eigen::Vector3f(1, 0, -v1.x());
	}

	float slope = (v2.y() - v1.y()) / (v2.x() - v1.x());
	float a = slope;
	float b = -1;
	float c = -slope * v1.x() + v1.y();

	return Eigen::Vector3f(a, b, c);
}

void ConstraintsGenerator::map_verts_to_edge(std::vector<std::vector<int>>& vert_to_edge_map)
{
	vert_to_edge_map.resize(vertices_.size());

	std::vector<std::unordered_set<int>> temp_map(vertices_.size());

	int l_idx = 0;
	for (std::vector<Line>::iterator l_it = edges_.begin(); l_it != edges_.end(); ++l_it, ++l_idx)
	{
		int v1 = l_it->p1();
		int v2 = l_it->p2();

		if (temp_map[v1].find(l_idx) == temp_map[v1].end())
			temp_map[v1].insert(l_idx);
		if (temp_map[v2].find(l_idx) == temp_map[v2].end())
			temp_map[v2].insert(l_idx);
	}

	for (int i = 0; i < vertices_.size(); i++)
	{
		vert_to_edge_map[i].resize(temp_map[i].size());
		int idx = 0;
		for (std::unordered_set<int>::iterator it = temp_map[i].begin();
			it != temp_map[i].end(); ++it, ++idx)
		{
			vert_to_edge_map[i][idx] = *it;
		}
	}
}

Eigen::Vector2f ConstraintsGenerator::get_line_direction(int line_id)
{
	Line &line = edges_[line_id];
	Eigen::Vector2f &v1 = vertices_[line.p1()];
	Eigen::Vector2f &v2 = vertices_[line.p2()];

	return v2 - v1;
}

void ConstraintsGenerator::output_constraints()
{
	std::ofstream out;
	std::string output_dir = "D:\\Libraries\\matlab_tools\\broyden\\";
	Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n");

	std::string A_path = output_dir + "A.csv";
	out.open(A_path.c_str());
	if (out.is_open())
	{
		out << A_.format(CSVFormat);
		out.close();
	}

	//std::string B_path = output_dir + "B.csv";
	//out.open(B_path.c_str());
	//if (out.is_open())
	//{
	//	out << B_.format(CSVFormat);
	//	out.close();
	//}

	std::string C_path = output_dir + "C.csv";
	out.open(C_path.c_str());
	if (out.is_open())
	{
		out << C_.format(CSVFormat);
		out.close();
	}

	Eigen::MatrixXf E_aug(E_.rows(), E_.cols() + 1);
	E_aug.block(0, 0, E_.rows(), E_.cols()) = E_;
	Eigen::VectorXf constants(E_.rows());
	constants.setConstant(focal_length_ / Z0);
	E_aug.block(0, E_.cols(), E_.rows(), 1) = constants;

	std::string E_path = output_dir + "E.csv";
	out.open(E_path.c_str());
	if (out.is_open())
	{
		out << E_aug.format(CSVFormat);
		out.close();
	}

	std::string G_path = output_dir + "G.csv";
	out.open(G_path.c_str());
	if (out.is_open())
	{
		for (std::vector<std::pair<int, int>>::iterator it = G_.begin(); it != G_.end(); ++it)
		{
			out << it->first << "," << it->second << std::endl;
		}
		out.close();
	}
}

void ConstraintsGenerator::output_environment()
{
	std::ofstream out;
	out.open("D:\\Libraries\\matlab_tools\\broyden\\x0.csv");
	if (out.is_open())
	{
		for (std::vector<Eigen::Vector2f>::iterator vert_it = vertices_.begin(); vert_it != vertices_.end(); ++vert_it)
			out << vert_it->x() << "," << vert_it->y() << std::endl;

		out.close();
	}

	out.open("D:\\Libraries\\matlab_tools\\broyden\\edges.csv");
	if (out.is_open())
	{
		for (std::vector<Line>::iterator e_it = edges_.begin(); e_it != edges_.end(); ++e_it)
			out << e_it->p1() << "," << e_it->p2() << std::endl;
		out.close();
	}

	out.open("D:\\Libraries\\matlab_tools\\broyden\\vert_face_map.txt");
	if (out.is_open())
	{
		for (std::vector<std::vector<int>>::iterator v_it = vert_to_face_map_.begin();
			v_it != vert_to_face_map_.end(); ++v_it)
		{
			if (!v_it->empty())
			{
				out << v_it->front();
				for (int i = 1; i < v_it->size(); i++)
					out << " " << v_it->operator[](i);
				out << std::endl;
			}
		}
		out.close();
	}

	out.open("D:\\Libraries\\matlab_tools\\broyden\\face_parallel_groups.txt");
	if (out.is_open())
	{
		for (std::vector<std::list<std::pair<int, int>>>::iterator f_it = lines_parallel_to_faces_.begin();
			f_it != lines_parallel_to_faces_.end(); ++f_it)
		{
			if (!f_it->empty())
			{
				out << f_it->front().first << " " << f_it->front().second;
				std::list<std::pair<int, int>>::iterator l_it = f_it->begin();
				std::advance(l_it, 1);
				for (; l_it != f_it->end(); ++l_it)
					out << " " << l_it->first << " " << l_it->second;
				out << std::endl;
			}
		}
		out.close();
	}

	out.open("D:\\Libraries\\matlab_tools\\broyden\\f.csv");
	if (out.is_open())
	{
		out << focal_length_ << std::endl;
		out.close();
	}
}

int ConstraintsGenerator::factorial(int n)
{
	int fact = 1;
	for (int i = 2; i <= n; i++)
		fact *= i;

	return fact;
}

void ConstraintsGenerator::update_environment(const std::vector<Eigen::Vector2f>& refined_vertices)
{
	for (int i = 0; i < vertices_.size(); i++)
	{
		vertices_[i][0] = refined_vertices[i][0];
		vertices_[i][1] = refined_vertices[i][1];
	}

	std::shared_ptr<CameraClibrator> calibrator(new CameraClibrator(vertices_, edges_, parallel_groups_, width_, height_));
	calibrator->calibrate(focal_length_, primary_point_);
}

