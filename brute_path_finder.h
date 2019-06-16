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

struct Node {
  row charger_properties;
  std::vector<std::pair<std::string, double>> neighbors;
  double distance_to_farthest_neighbor;
  // Path finder algorithms properties;
  Color color;
  Node* prev;	
  double car_range_left;
  double travel_time_to_get_here;
  double local_charging_time;
};


enum FuelRechargePolicy {
	MINIMAL_RECHARGE = 0,
	FULL_RECHARGE
};

class BruteForcePathFinder {
public:
	BruteForcePathFinder(const FuelRechargePolicy policy);

	virtual std::string GetPath(
		const std::string& initial_charger_name, 
		const std::string& final_charger_name);

	void SetIsDebug(const bool is_debug) {
		is_debug_ = is_debug;
	}

	double GetDistance(const std::string& start, const std::string& end);
	
	~BruteForcePathFinder();
private:
	void PrepareNodes();
	virtual std::string GetPath(const row& a, const row& b);
	virtual std::string GetPathString(Node* node);
	std::string GetDebugString(Node* node);
	const FuelRechargePolicy policy_;
	std::vector<Node*> nodes_;
	std::unordered_map<std::string, Node*> name_to_node_map_;
	std::unordered_map<std::string, const row*> name_to_row_map_;
	bool is_debug_;
};

}  // namespace challenge