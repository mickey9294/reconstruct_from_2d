#pragma once

#include <memory>
#include <assert.h>
#include <unordered_map>

#include "SketchGraph.h"
#include "MPFGraph.h"

class FacesIdentifier
{
public:
	FacesIdentifier();
	FacesIdentifier(const std::list<QPointF> &vertices, const std::list<Line> &edges);
	~FacesIdentifier();

	void identify_all_faces(std::vector<std::vector<int>> &face_circuits);
	void generate_min_potential_faces(std::vector<std::vector<int>> &face_circuits);
	int find_max_weight_clique(MPFGraph &mpf_graph, std::list<std::list<int>> &cmwc);

private:
	std::shared_ptr<SketchGraph> sketch_graph_;

	void circuit(int v, int start, int &index, std::vector<int> &path, 
		std::vector<bool> &label, std::vector<std::vector<int>> &circuits);

	int compute_ccw(MPFGraph &mpf_graph, std::list<int> &clique);
	int compute_pvw(MPFGraph &mpf_graph, std::list<int> &level, int first_of_level);
	int compute_pvw(MPFGraph &mpf_graph, std::vector<int> &level, int first_of_level);

	int find_max_weight_clique(MPFGraph &mpf_graph, int level_id, std::vector<int> level,
		int first, int last, std::list<int> current_clique, int &ns, int &cmw, std::list<std::list<int>> &cmwc);
};

