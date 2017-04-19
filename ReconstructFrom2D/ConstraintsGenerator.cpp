#include "ConstraintsGenerator.h"

const float ConstraintsGenerator::t = 100;

ConstraintsGenerator::ConstraintsGenerator()
{
}

ConstraintsGenerator::ConstraintsGenerator(float focal_length, float w, float h, const std::list<QPointF>& vertices,
	const std::list<Line>& edges, const std::vector<std::vector<int>> & face_circuits, const std::list<std::vector<int>> &parallel_group)
{
	focal_length_ = focal_length;
	Z0 = -2 * focal_length_;

	calib_mat_.setIdentity();
	calib_mat_(0, 0) = focal_length_;
	calib_mat_(1, 1) = focal_length_;

	vertices_.resize(vertices.size());
	edges_.resize(edges.size());

	int idx = 0;
	for (std::list<QPointF>::const_iterator it = vertices.begin(); it != vertices.end(); ++it)
	{
		vertices_[idx][0] = it->x() - w / 2.0;
		vertices_[idx++][1] = h / 2.0 - it->y();
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

	set_parallel_groups(parallel_group);

	map_edges_to_face();
}


ConstraintsGenerator::~ConstraintsGenerator()
{
}

void ConstraintsGenerator::add_connectivity_constraint()
{
	int num_vertices = vertices_.size();
	int num_faces = faces_.size();

	std::vector<std::vector<int>> face_vert_map;
	map_faces_to_verts(faces_, num_vertices, face_vert_map);

	std::list<Eigen::MatrixXf> As;
	int num_cols = 0;
	for (int i = 0; i < num_vertices; i++)
	{
		Eigen::Vector3f homo_vert;
		homo_vert.head(2) = vertices_[i];
		homo_vert[2] = -focal_length_;

		std::vector<int> &related_faces_id = face_vert_map[i];
		int num_related_faces = related_faces_id.size();
		num_cols += num_related_faces - 1;

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

	A_.resize(3 * num_faces, num_cols);
	int start_col = 0;
	for (std::list<Eigen::MatrixXf>::iterator A_it = As.begin(); A_it != As.end(); ++A_it)
	{
		A_.block(0, start_col, 3 * num_faces, A_it->cols()) = *A_it;
		start_col += A_it->cols();
	}

	A_ = A_.transpose();
}

void ConstraintsGenerator::add_perspective_symmetry_constraint()
{
	for (std::vector<PlanarFace>::iterator face_it = faces_.begin(); face_it != faces_.end(); ++face_it)
	{
		Eigen::Vector3f pp, pas, pae;
		perspective_symmetry_in_face(*face_it, pp, pas, pae);
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

	std::list<Eigen::VectorXf> Crows;

	for (std::vector<PlanarFace>::iterator face_it = faces_.begin(); face_it != faces_.end(); ++face_it)
	{
		int face_id = face_it->id();
		const std::list<int> & edges_list = face_it->const_edges();

		for (std::list<int>::const_iterator e_it = edges_list.begin(); e_it != edges_list.end(); ++e_it)
		{
			int line_id = *e_it;
			int parallel_group_id = edge_parallel_group_map[line_id];

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
								Eigen::VectorXf Crow(3 * Nf);
								Crow.setZero();

								Eigen::Vector3f line_1 = get_line_equation(paraline_1);
								Eigen::Vector3f line_2 = get_line_equation(paraline_2);
								Eigen::Vector3f v = line_1.cross(line_2);
								assert(std::abs(v[2]) > 1e-4);
								v[0] /= v[2];
								v[1] /= v[2];

								Eigen::Vector3f R = calib_mat_.inverse() * v;
								Crow.block<1, 3>(0, face_id * 3) = R.transpose();

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
		C_.block(row_idx, 0, 1, 3 * Nf) = *row_it;

	std::cout << "C:\n" << C_ << std::endl;
}

void ConstraintsGenerator::detect_symmetric_faces()
{
	for (int i = 0; i < faces_.size(); i++)
	{
		Eigen::Vector3f pp, as, ae;
		bool ret = perspective_symmetry_in_face(faces_[i], pp, as, ae);
	}
}

void ConstraintsGenerator::set_parallel_groups(const std::list<std::vector<int>>& parallel_groups)
{
	parallel_groups_.resize(parallel_groups.size());
	for (std::list<std::vector<int>>::const_iterator g_it = parallel_groups.begin(); g_it != parallel_groups.end(); ++g_it)
	{
		std::vector<int> group = *g_it;
		parallel_groups_.push_back(group);
	}
}

void ConstraintsGenerator::map_faces_to_verts(const std::vector<PlanarFace>& faces, int num_vertices,
	std::vector<std::vector<int>>& face_vert_map)
{
	face_vert_map.clear();
	face_vert_map.resize(num_vertices);

	for (std::vector<PlanarFace>::const_iterator it = faces.begin();
		it != faces.end(); ++it)
	{
		int id = it->id();
		const std::vector<int> &circuit = it->const_circuit();

		for (std::vector<int>::const_iterator jt = circuit.begin(); jt != circuit.end(); ++jt)
			face_vert_map[*jt].push_back(id);
	}
	
	for (std::vector<std::vector<int>>::iterator it = face_vert_map.begin();
		it != face_vert_map.end(); ++it)
		it->shrink_to_fit();
}

bool ConstraintsGenerator::perspective_symmetry_in_face(const PlanarFace & face, 
	Eigen::Vector3f & perspective_point, Eigen::Vector3f & sym_axis_start, Eigen::Vector3f & sym_axis_end)
{
	const std::vector<int> &circuit = face.const_circuit();
	int N = circuit.size();

	float min_ci = std::numeric_limits<float>::max();
	Eigen::Matrix3f min_Hi;
	for (int i = 0; i < N; i++)
	{
		if (i == 1)
			continue;
		Eigen::MatrixXf Si(3 * N, 9);
		Eigen::MatrixXf X(3, N);
		Eigen::MatrixXf X_star(3, N);
		for (int k = 0; k < N; k++)
		{
			Eigen::Vector3f x1, x2;
			int x1_idx = (1 + k) % N;
			x1.head(2) = vertices_[circuit[x1_idx]];
			x1[2] = 1.0;
			int x2_idx = (i - k) % N;
			if (x2_idx < 0)
				x2_idx = N + x2_idx;
			x2.head(2) = vertices_[circuit[x2_idx]];
			x2[2] = 1.0;

			Si.block<3, 9>(3 * k, 0) = form_S(x1, x2);
			X.block<3, 1>(0, k) = x1;
			X_star.block<3, 1>(0, k) = x2;
		}

		Eigen::MatrixXf solution = Si.transpose() * Si;
		Eigen::EigenSolver<Eigen::MatrixXf> es(solution);

		Eigen::MatrixXf evalues = es.eigenvalues().real();
		Eigen::MatrixXf evectors = es.eigenvectors().real();

		std::vector<float> evals(evalues.rows());
		std::vector<int> eval_idx(evalues.rows());
		for (int j = 0; j < evalues.rows(); j++)
		{
			evals[j] = evalues(j, 0);
			eval_idx[j] = j;
		}

		std::sort(eval_idx.begin(), eval_idx.end(), [&evals](int a, int b) {
			return evals[a] < evals[b];
		});

		Eigen::Matrix3f Hi;
		Eigen::VectorXf hi = evectors.col(eval_idx.front());
		
		for (int j = 0; j < 3; j++)
		{
			for (int t = 0; t < 3; t++)
				Hi(j, t) = hi[3 * j + t];
		}

		//Hi = X_star * X.transpose() * ((X * X.transpose()).inverse());
 
		float ci = 0;
		for (int k = 0; k < N; k++)
		{
			Eigen::Vector3f x1, x2;
			x1.head(2) = vertices_[circuit[(1 + k) % N]];
			x1[2] = 1.0;
			int x2_idx = (i - k) % N;
			if (x2_idx < 0)
				x2_idx = N + x2_idx;
			x2.head(2) = vertices_[circuit[x2_idx]];
			x2[2] = 1.0;

			Eigen::Vector3f vec = Hi * x1 - x2;
			ci += vec.norm();
		}

		if (ci < min_ci)
		{
			min_ci = ci;
			min_Hi = Hi;

			Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n");
			std::ofstream out("../X.csv");
			out << X.format(CSVFormat);
			out.close();

			out.open("../X_star.csv");
			out << X_star.format(CSVFormat);
			out.close();

			out.open("../H.csv");
			out << Hi.format(CSVFormat);
			out.close();
		}
	}

	//if(min_ci >= t)
		//return false;

	Eigen::EigenSolver<Eigen::Matrix3f> es(min_Hi);
	Eigen::MatrixXcf eigen_values = es.eigenvalues();
	Eigen::Matrix3cf eigen_vectors = es.eigenvectors();

	std::cout << "Eigen values:\n" << eigen_values << std::endl;
	std::cout << "Eigen vectors:\n" << eigen_vectors << std::endl;

	return true;
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
	for(int i = 0; i < edges_.size(); i++)
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
