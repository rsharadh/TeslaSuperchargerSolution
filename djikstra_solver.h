// Tesla SuperCharger Challenge Solution by Sharadh Ramaswamy
// Solution based on 
// A) The Shortest Path with Charging Problem: A Scalable, Discretized Approach
// https://pdfs.semanticscholar.org/dd62/0e8e55dc27ec660b763a4d3a21af5940b66f.pdf
// B) Djikstra algorithm for single source shortest paths.
#pragma once

#include <algorithm>
#include <float.h>
#include <string>
#include <unordered_map>
#include <vector>
#include "network.h"
#include "path_finder_util.h"

namespace challenge {
// In order to find the optimal routing + charging policy, we create an
// artificial charger network, where each charging station is replaced by
// a pre-specified (fixed) number of charging stations, N, each of which charges
// the car to a unique pre-determined, fixed (output) charge level equalling
// k * / N * kCarCapacityKm, where 1 <= k <= N and kCarCapacityKm = 320. 
//
// In this augmented network, given two nodes A and B, B is a neighbor of A iff 
// 1) distance between A and B or distance (A, B) <= 320 km AND
// 2) output charge level of A >= distance (A, B) AND
// 3) output charge level of A <= distance (A, B) + output charge level of B
// Note that
// i) the recharging time at B, when the car arrives from A can be calculated as
//   recharge-time(B | A) =
//		(output-charge-level(B) - 
//			(output-charge-level(A) - distance(A, B))) 
//	     / charging-rate(B) 
// ii) effective travel time from A-to-B = 
// 			recharge-time(B | A) + distance(A, B) / kCarSpeedKmPh
//	
// For any fixed N, the shortest path between any two nodes can be computed with
// the Djikstra algorithm, which also identifies an optimal recharging policy. 
// As N gets larger, the shortest path and the discretized charging policy also
// approach the continous space optimum. In practice, N = 10 works very well.

// Node representing augmented graph where each node is represented
// by the corresponding charger properties (name, lat, long, chargin rate)
// and an output charge level.
struct DjikstraNode {
	// Fixed properties.
	row charger_properties;
	double output_charge_level;

	// List of pointers to neighbors and their distances.
	std::vector<std::pair<DjikstraNode*, double>> neighbors;
	std::unordered_map<DjikstraNode*, double> neighbor_recharging_times;

	// Path finder algorithm properties.
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

// Class to find the shortest path between a pair of charging stations
// in an augmented graph with 'num_charge_levels' charge levels, using
// the Djikstra algorithm run on a graph of DjistraNode's.
class DjikstraSolver {
public:
	DjikstraSolver(const int num_charge_levels, const bool is_multi_threaded_init = true);

	// Computes path string consisting of a sequence of charging station plus time
	// spent at charging station (output format set in challenge question).
	std::string GetPath(
		const std::string& initial_charger_name,
		const std::string& final_charger_name);

	// Returns distance between start and end charging stations in Km.
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

	// Initializes one node and its neighbors.
	void InitializeNodeAndNeighbors(DjikstraNode* node);
	// Initializes each node and its neighbors, for given vector of nodes.
	void InitializeNodesAndNeighbors(std::vector<DjikstraNode*>* nodes);	
	// Populates neighborhood of each DjikstraNode in graph using
	// pre-computed node-pair distance map. Uses std::thread for parallelized
	// initialization.
	void FindNeighbors();
	
	// Same as FindNeighbors, but with unoptimized for-loops (hence slower).
	void FindNeighborsHeavy();

	// Computes shortest path from given charger to all other chargers, which
	// also computes
	void SingleSourceShortestPaths(
		const std::string& initial_charger_name);

	// Gets optimal path to a specific destination node from provided
	// source node (specified in initial_charger_name), through Djikstra
	// algorithm.
	std::string GetPath(DjikstraNode*);
	std::string GetDebugString(DjikstraNode*);

	// Number of discrete charge levels in the augmented network.
	const int num_charge_levels_;

	// Set of DjikstraNodes in the augmented network.
	std::vector<DjikstraNode*> nodes_;

	// Map from a string representing the DjikstraNode to a pointer to
	// to DjikstraNode, where the "name" of a DjikstraNode A is constructed
	// as A.charger_properties.name + ':' + std::to_string(A.output_charge_level).
	std::unordered_map<std::string, DjikstraNode*> name_to_node_map_;


	// Map from a name of the charger to a pointer to the corresponding
	// row element.
	std::unordered_map<std::string, const row*> name_to_row_map_;

	// Map from charger name to the name of its neighbors and their distances.	
	std::unordered_map<std::string, std::vector<std::pair<std::string, double>>> charger_name_to_neighors_map_;	

	// Flag used to decide if a debug string is to be output as a part of
	// the final path string.
	bool is_debug_;

	// Flag to indicate if initialization is multi-threaded.
	bool is_multi_threaded_init_;
};

}  // namespace challenge 