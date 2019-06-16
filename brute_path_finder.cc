// Tesla SuperCharger Challenge Solution by Sharadh Ramaswamy
#include "brute_path_finder.h"

#include <array>
#include <algorithm>
#include <float.h>
#include <deque>          // std::deque
#include <string>
#include <unordered_map>
#include <vector>
#include "network.h"
#include "path_finder_util.h"

namespace challenge {
namespace {
constexpr double kErrorDiscounter = 1.0;

typedef std::pair<std::string, double> NameDistance;	

void PrintProperties(const row& a) {
	std::cout<<"Name "<<a.name<<std::endl;
	std::cout<<"Latitude "<<a.lat<<std::endl;
	std::cout<<"Longitude "<<a.lon<<std::endl;
	std::cout<<"Rate "<<a.rate<<std::endl;
}

void PrintProperties(const Node& a) {
	PrintProperties(a.charger_properties);
	for (const auto& neighbor: a.neighbors) {
		std::cout<<"Neighbor "<<neighbor.first<<" at a distance of "<<neighbor.second<<std::endl;
	}
}

void Recharge(Node* node, const double current_range,
	const double target_range) {
	if (node == nullptr) {
		return;
	}
	if (current_range >= target_range) {
		return;
	}
	double range_extension_needed = target_range - current_range;
	node->local_charging_time = range_extension_needed / node->charger_properties.rate;
	node->car_range_left = target_range;
}
}  // namespace

BruteForcePathFinder::BruteForcePathFinder(const FuelRechargePolicy policy) : policy_(policy) {
	for (const auto& charging_station: network) {
		Node* node = new Node();
		node->charger_properties = charging_station;
		node->neighbors.clear();
		name_to_node_map_[node->charger_properties.name] = node;
		name_to_row_map_[node->charger_properties.name] = &charging_station;
		nodes_.push_back(node);
	}

	for (const auto& node: nodes_) {
		node->neighbors = FindNeighbors(
			*name_to_row_map_[node->charger_properties.name], kErrorDiscounter);
		const auto& max_element = std::max_element(
			node->neighbors.begin(),node->neighbors.end(), 
			[](const NameDistance& a , const NameDistance& b) {
				return a.second < b.second;
			});
		node->distance_to_farthest_neighbor = (*max_element).second;
	}
	is_debug_ = false;
};


double BruteForcePathFinder::GetDistance(const std::string& start, const std::string& end) {
	if (is_debug_) {
		PrintProperties(*name_to_row_map_[start]);
		PrintProperties(*name_to_row_map_[end]);
	}	
	return GreatCircleDistanceKm(
		*name_to_row_map_[start], 
		*name_to_row_map_[end]);
}

std::string BruteForcePathFinder::GetPath(
	const std::string & initial_charger_name, 
	const std::string & final_charger_name) {
	if (initial_charger_name == final_charger_name) {
		return initial_charger_name + ", " + final_charger_name;
	}
	return GetPath(
		*name_to_row_map_[initial_charger_name], 
		*name_to_row_map_[final_charger_name]);
}

std::string BruteForcePathFinder::GetPath(const row& a, const row& b){
	PrepareNodes();
	Node* start_node = name_to_node_map_[a.name];
	start_node->travel_time_to_get_here = 0.0;
	start_node->car_range_left = kCarCapacityKm;
	// Since car starts with full range, there is no need to set
	// start_node->local_charing_time = kCarCapacityKm / start_node->charger_properties.rate;
	start_node->local_charging_time = 0;
	std::deque<Node*> node_processing_queue;
	node_processing_queue.push_back(start_node);

	while (!node_processing_queue.empty()) {
		Node* front = node_processing_queue.front();
		node_processing_queue.pop_front();
		if (front->charger_properties.name == b.name) {
			return GetPathString(front->prev) + ", " + b.name;
		}
		if (front->color != BLACK) {
			continue;
		}
		front->color = Color::GRAY;
		// visit neighbors;
		for (const auto& neighbor : front->neighbors) {
			Node* neighbor_node = name_to_node_map_[neighbor.first];
			if (neighbor_node->color != Color::BLACK) {
				continue;
			}

			const double distance_to_neighbor = neighbor.second;
			const double time_to_neighbor = 
				front->travel_time_to_get_here + 
				distance_to_neighbor / kCarSpeedKmPh + 
				front->local_charging_time;
			if (time_to_neighbor < neighbor_node->travel_time_to_get_here) {
				neighbor_node->travel_time_to_get_here = time_to_neighbor;
				neighbor_node->prev = front;
				double car_range_left = front->car_range_left - distance_to_neighbor;
				switch(policy_) {
					case MINIMAL_RECHARGE:
						Recharge(neighbor_node, car_range_left, 
							neighbor_node->distance_to_farthest_neighbor);
						break;
					case FULL_RECHARGE:
					default:
						Recharge(neighbor_node, car_range_left, kCarCapacityKm);
				}
			}
			node_processing_queue.push_back(neighbor_node);
		}
		front->color = WHITE;
	}

	return "No path available";
}

std::string BruteForcePathFinder::GetPathString(Node* node) {
	std::string path_string_prev;
	if (node == nullptr) {
		return path_string_prev;
	}
	if (node->prev == nullptr) {
		//path_string_prev = node->charger_properties.name;
		path_string_prev = 
			node->charger_properties.name + ", " +
			node->charger_properties.name + ", " + 
			std::to_string(node->local_charging_time);
		if (is_debug_) {
			path_string_prev += GetDebugString(node);
		}	
		return path_string_prev;
	}

	path_string_prev = GetPathString(node->prev);
	if (!path_string_prev.empty()) {
		path_string_prev += ", ";
	}
	path_string_prev += node->charger_properties.name; 
	path_string_prev += ", ";
	path_string_prev += std::to_string(node->local_charging_time);

	if (is_debug_) {
		path_string_prev += GetDebugString(node);
	}
	return path_string_prev;
}

std::string BruteForcePathFinder::GetDebugString(Node* node) {
	std::string path_string_prev = "";
	if (node == nullptr) {
		return path_string_prev;
	}
	if (node->prev == nullptr) {
		path_string_prev += 
			" " + std::to_string(node->local_charging_time * node->charger_properties.rate);
		return path_string_prev;
	}
	double distance_from_prev = 
		GetDistance(
			node->prev->charger_properties.name, 
			node->charger_properties.name);
	double car_range_left_at_prev = node->prev->car_range_left;
	double car_range_left_enter_here = 	
		car_range_left_at_prev - distance_from_prev;
	double car_range_left_leave_here = 
		car_range_left_enter_here + 
		node->local_charging_time * node->charger_properties.rate;

	path_string_prev += 
		" " + std::to_string(node->charger_properties.rate) + 
		" " + node->prev->charger_properties.name + "<->" + node->charger_properties.name +
		" " + std::to_string(distance_from_prev) + 
		" " + std::to_string(car_range_left_enter_here) + 
		" " + std::to_string(car_range_left_leave_here) + " ";

	return path_string_prev;
}

void BruteForcePathFinder::PrepareNodes() {
	for (auto& node: nodes_) {
		node->color = Color::BLACK;
		node->prev = nullptr;
		node->travel_time_to_get_here = DBL_MAX;
		node->local_charging_time = 0.0;
		node->car_range_left = 0.0;
	}
}

BruteForcePathFinder::~BruteForcePathFinder() {
	for (auto node: nodes_) {
		delete node;
	}
}
}  // namespace challenge