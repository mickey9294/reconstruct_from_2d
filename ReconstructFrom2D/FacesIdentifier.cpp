#include "FacesIdentifier.h"



FacesIdentifier::FacesIdentifier()
{
}

FacesIdentifier::FacesIdentifier(std::list<QPointF> vertices, std::list<Line> edges)
{
	sketch_graph_.reset(new SketchGraph(vertices, edges));
}


FacesIdentifier::~FacesIdentifier()
{
}

void FacesIdentifier::identify_all_faces(std::vector<std::vector<int>>& face_circuits)
{
	/* Find out all minimal potential faces */
	std::vector<std::vector<int>> potential_face_circuits;
	generate_min_potential_faces(potential_face_circuits);

	MPFGraph mpf_graph(potential_face_circuits, sketch_graph_->const_vertices());

	std::list<std::list<int>> cmwc;
	find_max_weight_clique(mpf_graph, cmwc);

	assert(!cmwc.empty());
	std::list<int> &max_weight_clique = cmwc.front();
	face_circuits.clear();
	face_circuits.reserve(max_weight_clique.size());

	for (std::list<int>::iterator it = max_weight_clique.begin();
		it != max_weight_clique.end(); ++it)
	{
		MPFVertex &vert = mpf_graph[*it];
		std::vector<int> face_circuit = vert.get_circuit();

		face_circuits.push_back(face_circuit);
	}
}

void FacesIdentifier::generate_min_potential_faces(std::vector<std::vector<int>>& face_circuits)
{
	int num_vertices = sketch_graph_->num_vertices();

	int index = 0;
	std::vector<bool> label(num_vertices, false);
	std::vector<int> path(num_vertices);
	std::vector<std::vector<int>> circuits;

	for (int i = 0; i < num_vertices - 1; i++)
	{
		int start = i;
		circuit(i, start, index, path, label, circuits);
		sketch_graph_->remove_vertex_in_adjacency_list(i);
	}

	sketch_graph_->rebuild_adjacency_list();
}

int FacesIdentifier::find_max_weight_clique(MPFGraph & mpf_graph, std::list<std::list<int>> &cmwc)
{
	int num_vertices = mpf_graph.num_vertices();

	/* Initialization */
	int cmw = 0;   /* The current maximum weight of a clique */
	int level_id = 1;  /* The current level */
	std::vector<int> current_level;
	current_level.reserve(num_vertices);
	for (int i = 0; i < num_vertices; i++)
		current_level.push_back(i);

	int ns = 0;
	std::list<int> current_clique;

	int ret = find_max_weight_clique(mpf_graph, level_id, current_level, -1, num_vertices - 1,
		current_clique, ns, cmw, cmwc);
}


void FacesIdentifier::circuit(int v, int start, int & index, std::vector<int> &path, 
	std::vector<bool> &label, std::vector<std::vector<int>> &circuits)
{
	index++;
	//std::vector<bool> label(sketch_graph_->num_vertices(), false);
	//std::vector<int> path;
	label[v] = true;
	path[index - 1] = v;

	std::list<int> &adj = sketch_graph_->get_vertex_adjacency(v);
	for (std::list<int>::iterator adj_it = adj.begin(); adj_it != adj.end(); ++adj_it)
	{
		int u = *adj_it;
		if (u == start && index >= 3)
		{
			/* Check whether edge(u,v) interects any edge in the current path */
			bool exist_intersection = false;
			for (int i = 0; i < index - 1; i++)
			{
				exist_intersection = exist_intersection
					|| sketch_graph_->edges_intersect(u, v, path[i], path[i + 1]);

				if (exist_intersection)
					break;
			}
			if (exist_intersection)
				break;

			/* To avoid double the circuits */
			if (path[1] < path[index - 1])
			{
				std::vector<int> acircuit(index);
				for (int i = 0; i < index; i++)
					acircuit[i] = path[i];
				circuits.push_back(acircuit);
				break;
			}
		}
		else
		{
			if (label[u])  /* To ensure the current vertex has not been visited */
				continue;

			/* Check whether edge(u,v) interects any edge in the current path */
			bool exist_intersection = false;
			for (int i = 0; i < index - 1; i++)
			{
				exist_intersection = exist_intersection
					|| sketch_graph_->edges_intersect(u, v, path[i], path[i + 1]);

				if (exist_intersection)
					break;
			}
			if (exist_intersection)
				continue;

			/* Check whether there is no edge in the graph connecting any two nonadjacent vertices in the current path */
			bool flag = false;
			for (int i = 0; i < index; i++)
			{
				for (int j = i + 2; j < index; j++)
				{
					bool adjacent = sketch_graph_->are_adjacent(path[i], path[j]);
					if (adjacent)
					{
						flag = true;
						break;
					}
				}
				if (flag)
					break;
			}
			
			if (flag)
				continue;

			circuit(u, start, index, path, label, circuits);
		}
	}

	index--;
	label[v] = false;
}

int FacesIdentifier::compute_ccw(MPFGraph & mpf_graph, std::list<int>& clique)
{
	int weight_sum = 0;
	for (std::list<int>::iterator it = clique.begin(); it != clique.end(); ++it)
		weight_sum += mpf_graph.get_vertex_weigth(*it);

	return weight_sum;
}

int FacesIdentifier::compute_pvw(MPFGraph & mpf_graph, std::list<int> &level, int first_of_level)
{
	int num_vertices = mpf_graph.num_vertices();
	int pvw = 0;
	std::list<int>::iterator it = level.begin();
	std::advance(it, first_of_level + 1);
	for (; it != level.end(); it++)
		pvw += mpf_graph.get_vertex_weigth(*it);

	return pvw;
}

int FacesIdentifier::compute_pvw(MPFGraph & mpf_graph, std::vector<int>& level, int first_of_level)
{
	int num_vertices = mpf_graph.num_vertices();
	int pvw = 0;
	for (int i = first_of_level + 1; i < level.size(); i++)
		pvw += mpf_graph.get_vertex_weigth(i);

	return pvw;
}

int FacesIdentifier::find_max_weight_clique(MPFGraph & mpf_graph, int level_id, std::vector<int> level,
	int first, int last, std::list<int> current_clique, int &ns, int & cmw, std::list<std::list<int>>& cmwc)
{
	while (true)
	{
		first++;
		current_clique.push_back(level[first]);
		int ccw = compute_ccw(mpf_graph, current_clique);
		int pvw = compute_pvw(mpf_graph, level, first);

		if (ccw + pvw < cmw)
		{
			if (level_id == 1)
				return 0;
			else
				return 1;
		}

		std::vector<int> next_level;
		int first_vert = level[first];
		for (int i = first + 1; i < last; i++)
		{
			if (mpf_graph.are_adjacent(first_vert, level[i]))
				next_level.push_back(level[i]);
		}
		if (!next_level.empty())
		{
			next_level.shrink_to_fit();
			int ret_state = find_max_weight_clique(mpf_graph, level_id + 1, next_level, -1, 
				next_level.size() - 1, current_clique, ns, cmw, cmwc);
			if (ret_state == 0)
				return 0;
			if (ret_state == 1)
			{
				current_clique.pop_back();
				continue;
			}
		}

		if (ccw > cmw)
		{
			cmw = ccw;
			ns = 1;
			cmwc.push_back(current_clique);
		}
		else if (ccw == cmw)
		{
			ns++;
			cmwc.push_back(current_clique);
		}

		current_clique.pop_back(); 
		if (level_id > 1 && first == last)
			return 2;
	}
}
