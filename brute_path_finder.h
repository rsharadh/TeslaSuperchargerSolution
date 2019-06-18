// Tesla SuperCharger Challenge Solution by Sharadh Ramaswamy
#pragma once
#include "network.h"

#include <string>
#include <unordered_map>
#include <vector>
#include "path_finder_util.h"

namespace challenge {

enum Color {
	BLACK = 0,
	GRAY,
	WHITE
};

// Node in network.
struct Node {
  row charger_properties;
  // List of neighbor names and their distances, where two
  // nodes are neighbors if the distance between them <=
  // kCarCapacityKm (320 km).
  std::vector<std::pair<std::string, double>> neighbors;
  double distance_to_farthest_neighbor;

  // Path finder algorithms properties.
  Color color;
  Node* prev;
  double car_range_left;
  double travel_time_to_get_here;
  double local_charging_time;
};


// Fuel recharge policy
// A) MINIMAL_RECHARGE: recharges car to level required for the car to reach the
//    furthest of the neighbors from any given charging station OR
// B) FULL_RECHARGE: fully recharges car at every station OR
enum FuelRechargePolicy {
	MINIMAL_RECHARGE = 0,
	FULL_RECHARGE
};

// "Brute" path finder that finds the path between two nodes
// using breadth first search. 
class BruteForcePathFinder {
public:
	BruteForcePathFinder(const FuelRechargePolicy policy);

	// Returns path from source to destination charger in the
	// format required by challenge.
	virtual std::string GetPath(
		const std::string& initial_charger_name, 
		const std::string& final_charger_name);

	void SetIsDebug(const bool is_debug) {
		is_debug_ = is_debug;
	}

	// Returns great-circle distance between start and end chargers.
	double GetDistance(const std::string& start, const std::string& end);
	
	~BruteForcePathFinder();

private:
	// Prepares all Node's for breadth first path search from start charger
	// to end charger.
	void PrepareNodes();

	// Gets path from super charger rows a and b.
	std::string GetPath(const row& a, const row& b);

	// Gets path string for a path ending in node, for a pre-provided
	// starting charger station/node. 
	std::string GetPathString(Node* node);

	// Generates a debug string at each stage of journey.
	std::string GetDebugString(Node* node);

	// Recharging policy at each charger station.
	const FuelRechargePolicy policy_;

	// Nodes in super charger network.
	std::vector<Node*> nodes_;

	// Map from charger name to corresponding Node pointer.
	std::unordered_map<std::string, Node*> name_to_node_map_;

	// Map from charger name to corresponding row pointer.
	std::unordered_map<std::string, const row*> name_to_row_map_;

	// Flag to generate debug string while generating path string
	// from source to destination.
	bool is_debug_;
};

}  // namespace challenge