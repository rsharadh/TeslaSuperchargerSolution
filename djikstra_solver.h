// Tesla SuperCharger Challenge Solution by Sharadh Ramaswamy
// Solution based on 
// A) The Shortest Path with Charging Problem: A Scalable, Discretized Approach
// https://pdfs.semanticscholar.org/dd62/0e8e55dc27ec660b763a4d3a21af5940b66f.pdf
// B) Djikstra algorithm for single source shortest paths.
#pragma once

#include <algorithm>
#include <float.h>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>
#include "network.h"
#include "path_finder_util.h"

namespace challenge {


struct DjikstraNode {
	// Fixed properties.
	row charger_properties;
	double output_charge_level;
	std::vector<std::pair<DjikstraNode*, double>> neighbors;
	std::unordered_map<DjikstraNode*, double> neighbor_recharging_times;
	// Path finder algorithms properties;
 	DjikstraNode* prev;	
  	double travel_time_to_get_here;
};

class DjikstraComparator {
	public:
		bool operator()(
			const DjikstraNode* lhs, 
			const DjikstraNode* rhs) const {
			return lhs->travel_time_to_get_here >= 
				rhs->travel_time_to_get_here; 
		}
};

class DjikstraHeap {
	public:
		DjikstraHeap(
			const std::vector<DjikstraNode*>& nodes) 
				: heap_nodes_(nodes) {
			std::make_heap(
				heap_nodes_.begin(),
				heap_nodes_.end(), 
				comparator_);
		}

		bool Empty() {
			return heap_nodes_.empty();
		}

		DjikstraNode* Top() {
			if (Empty()) {
				return nullptr;
			}
			return heap_nodes_[0];
		}

		void Pop() {
			if (Empty()) {
				return;
			}
			auto temp = heap_nodes_.front();
			heap_nodes_.front() = heap_nodes_.back();
			heap_nodes_.back() = temp;

			heap_nodes_.pop_back();
			std::make_heap(
				heap_nodes_.begin(),
				heap_nodes_.end(), 
				comparator_);
		}
	private:
		std::vector<DjikstraNode*> heap_nodes_;	
		DjikstraComparator comparator_;
};

class DjikstraSolver {
public:
	DjikstraSolver(const int num_charge_levels);

	std::string GetPath(
		const std::string& initial_charger_name,
		const std::string& final_charger_name);

	double GetDistance(const std::string& start, const std::string& end);
	void SetIsDebug(bool is_debug) {
		is_debug_ = is_debug;
	}
	~DjikstraSolver();
private:
	void PrepareNodes();
	void PrepareSourceNodes(
		const std::string& initial_charger_name,
		const std::vector<int>& start_charge_indices);
	void FindNeighbors();
	void FindNeighborsHeavy();
	void SingleSourceShortestPaths(
		const std::string& initial_charger_name);
	std::string GetPath(DjikstraNode*);
	std::string GetDebugString(DjikstraNode*);
	const int num_charge_levels_;
	std::vector<DjikstraNode*> nodes_;
	std::unordered_map<std::string, DjikstraNode*> name_to_node_map_;
	std::unordered_map<std::string, const row*> name_to_row_map_;
	bool is_debug_;
};

}  // namespace challenge 