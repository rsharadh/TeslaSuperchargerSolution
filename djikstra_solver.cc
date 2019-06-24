// Tesla SuperCharger Challenge Solution by Sharadh Ramaswamy
#include <algorithm>
#include <math.h> 
#include <queue> 
#include <thread>

#include "network.h"
#include "djikstra_solver.h"
#include "node_distance_pair_map_pruned.h"

namespace challenge {
namespace {
constexpr double kLargeTimeValue = 1e6;

std::string GetDjikstraNodeName(const row& a, const int charge_level_index) {
	return a.name + ":" + std::to_string(charge_level_index);
}

}  // namespace

void DjikstraSolver::FindNeighborsHeavy() {
  for (auto node: nodes_) {
  	for (auto candidate: nodes_) {
  		if (node->charger_properties.name == candidate->charger_properties.name) {
  			continue;
  		}
  		const auto& key_node_candidate = GenerateNodePairKey(
  					node->charger_properties.name, 
  					candidate->charger_properties.name);
  		if (node_pair_distance_map.find(key_node_candidate) == node_pair_distance_map.end()) {
  			continue;
  		}
  		double distance_to_candidate = 
  			node_pair_distance_map[key_node_candidate];

  		if (distance_to_candidate < kCarCapacityKm && 
  			distance_to_candidate < node->output_charge_level) {
  			double destination_car_charge_level =
  				node->output_charge_level - distance_to_candidate;
	  		if (destination_car_charge_level > candidate->output_charge_level) {
	  			// Since we cannot discharge excess charge.
	  			continue;
	  		}
  			double travel_time = 
  				distance_to_candidate / kCarSpeedKmPh;
			double recharge_time = 
				(candidate->output_charge_level - 
					destination_car_charge_level) / candidate->charger_properties.rate;
			double effective_travel_time = 	
				travel_time + recharge_time;
			node->neighbors.push_back(
				std::make_pair(candidate, effective_travel_time));	
			node->neighbor_recharging_times[candidate] = recharge_time;
  		}
  	}
  }	
}

void DjikstraSolver::InitializeNodeAndNeighbors(DjikstraNode* node) {
  double delta_charge_level = kCarCapacityKm / num_charge_levels_;
  const auto& neighbors = 
  		charger_name_to_neighors_map_[node->charger_properties.name];
  	for (const auto& candidate: neighbors) {
  		if (node->output_charge_level < candidate.second) {
  			continue;
  		}
		double travel_time = 
  				candidate.second / kCarSpeedKmPh;
  		double destination_car_charge_level =
  				node->output_charge_level - candidate.second;
  		int start_charge_level_index = 	
  		 	std::max(
  		 		std::min(
  		 			static_cast<int>(
  		 				std::ceil(
  		 					destination_car_charge_level / 
  		 					delta_charge_level)), 
  		 			num_charge_levels_), 
  		 		1);	
	 	for (; start_charge_level_index <= num_charge_levels_; 
	 		start_charge_level_index++) {
			double recharge_time = 
				(start_charge_level_index * delta_charge_level - 
					destination_car_charge_level) / 
						name_to_row_map_[candidate.first]->rate;
			double effective_travel_time = 	
				travel_time + recharge_time;
			auto candidate_node = 
				name_to_node_map_[
					GetDjikstraNodeName(
						*name_to_row_map_[candidate.first], 
						start_charge_level_index)];	
			node->neighbors.push_back(
				std::make_pair(
					candidate_node, 
					effective_travel_time));	
			node->neighbor_recharging_times[candidate_node] = 
				recharge_time;
	 	}
  	}
}
void DjikstraSolver::InitializeNodesAndNeighbors(std::vector<DjikstraNode*>* nodes) {
	for (auto& node: *nodes) {
		InitializeNodeAndNeighbors(node);
	}
}
void DjikstraSolver::FindNeighbors() {
	for (const auto& charger: network) {
		charger_name_to_neighors_map_[charger.name] = 
			FindNeighborsPreComputed(charger);
  	}	
	if (false) {
		// Single threaded solution.
		for (auto node: nodes_) {
			InitializeNodeAndNeighbors(node);
		}
	} else {
		// Multi-threaded solution.
		std::vector<std::thread> thread_vector;
		if (false) {
			// One thread per Djikstra node <=> num_charge_levels_ * 303 threads.
		  for (auto node: nodes_) {
		  	thread_vector.push_back(
		  		std::thread(
		  			&DjikstraSolver::InitializeNodeAndNeighbors, 
		  			this, 
		  			node));
		  }
		  for (auto& t : thread_vector) {
		  	t.join();
		  }
		} else {
			if (false) {
				// Two threads.
			  	std::vector<DjikstraNode*> first_half(nodes_.begin(), nodes_.begin() + nodes_.size() / 2 + 1);
			  	std::vector<DjikstraNode*> second_half(nodes_.begin() + nodes_.size() / 2 + 1, nodes_.end());
			
			  	std::thread t1 (&DjikstraSolver::InitializeNodesAndNeighbors, this, &first_half);
			  	std::thread t2 (&DjikstraSolver::InitializeNodesAndNeighbors, this, &second_half);
			  	t1.join();
			  	t2.join();
		  	} else {
				// num_charge_levels_ threads.
		  		int num_splits = num_charge_levels_ ;
		  		int num_per_split = nodes_.size() / num_splits;
		  		std::vector<std::vector<DjikstraNode*>> node_splits;
		  		node_splits.reserve(num_splits);
		  		for (int node_split_index = 0; node_split_index < num_splits; node_split_index++) {
		  			int start_position = node_split_index * num_per_split;
		  			int end_position = start_position + num_per_split; 
		  			node_splits.push_back(
		  				std::vector<DjikstraNode*>(
		  					nodes_.begin() + start_position, 
		  					nodes_.begin() + end_position));
		  			thread_vector.push_back(
		  				std::thread(
		  					&DjikstraSolver::InitializeNodesAndNeighbors, 
		  					this, 
		  					&node_splits[node_split_index]));
		  		}
		  		for (auto& t : thread_vector) {
		      		t.join();
		      	}
		  	}
		} 
	}
}

DjikstraSolver::DjikstraSolver(const int num_charge_levels) 
	: num_charge_levels_(num_charge_levels) {
	double delta_charge_level = kCarCapacityKm / num_charge_levels_;	
	for (const auto& charging_station: network) {
		name_to_row_map_[charging_station.name] = &charging_station;
		for (int charge_level_index = 1; 
			charge_level_index * delta_charge_level <= kCarCapacityKm;
			charge_level_index++) {
			double charge_level = charge_level_index * delta_charge_level;
			DjikstraNode* node = new DjikstraNode();
			node->charger_properties = charging_station;
			node->output_charge_level = charge_level;
			node->neighbors.clear();
			nodes_.push_back(node);
			name_to_node_map_[
				GetDjikstraNodeName(
					node->charger_properties, 
					charge_level_index)] = node;
		}
	}
	FindNeighbors();
	is_debug_ = false;
}

double DjikstraSolver::GetDistance(
	const std::string& start, const std::string& end) {
	return GreatCircleDistanceKm(
		*name_to_row_map_[start], 
		*name_to_row_map_[end]);
}


std::string DjikstraSolver::GetPath(
		const std::string& initial_charger_name,
		const std::string& final_charger_name) {
	SingleSourceShortestPaths(initial_charger_name);

	DjikstraNode* actual_destination_node = 
		name_to_node_map_[
			GetDjikstraNodeName(
				*name_to_row_map_[final_charger_name], 
				num_charge_levels_)];
	for (int charge_level_index = 1; 
		charge_level_index <= num_charge_levels_; 
		charge_level_index++) {
		DjikstraNode* potential_destination_node = 
			name_to_node_map_[
				GetDjikstraNodeName(
					*name_to_row_map_[final_charger_name], 
					charge_level_index)];
		if (potential_destination_node->travel_time_to_get_here < 
			actual_destination_node->travel_time_to_get_here) {
			actual_destination_node = potential_destination_node;
		}	
	}

	if (is_debug_) {
		std::cout<<"actual_destination_node->travel_time_to_get_here "
			<<actual_destination_node->travel_time_to_get_here<<std::endl;
		std::cout<<"actual_destination_node->output_charge_level "
			<<actual_destination_node->output_charge_level<<std::endl;
		std::cout<<"GetPath(actual_destination_node) "
			<<GetPath(actual_destination_node)<<std::endl;
	}
	return GetPath(actual_destination_node->prev) + ", " + final_charger_name;
}

std::string DjikstraSolver::GetPath(DjikstraNode* node) {
	std::string path_string_prev;
	if (node == nullptr) {
		return path_string_prev;
	}

	if (node->prev == nullptr) {
		path_string_prev =  
			node->charger_properties.name + ", " +
			node->charger_properties.name + ", ";
		path_string_prev += "0.0";

		if (is_debug_) {
			path_string_prev += GetDebugString(node);
		}

		return path_string_prev;	
	}

	path_string_prev = GetPath(node->prev);
	if (!path_string_prev.empty()) {
		path_string_prev += ", ";
	}
	path_string_prev += node->charger_properties.name; 
	path_string_prev += ", ";
	auto& neighbor_recharging_time_map = 
		node->prev->neighbor_recharging_times;
	path_string_prev += 
		std::to_string(neighbor_recharging_time_map[node]);

	if (is_debug_) {
		path_string_prev += GetDebugString(node);
	}
	return path_string_prev;
}

std::string DjikstraSolver::GetDebugString(DjikstraNode* node) {
	std::string debug_string;
	if (node == nullptr) {
		return debug_string;
	}

	if (node->prev == nullptr) {
		debug_string += 
			" " + std::to_string(node->output_charge_level);
		return debug_string;
	}	

	double distance_from_prev = 
		GetDistance(
			node->prev->charger_properties.name, 
			node->charger_properties.name);
	double car_range_left_at_prev = node->prev->output_charge_level;
	double car_range_left_enter_here = 	
		car_range_left_at_prev - distance_from_prev;
	double car_range_left_leave_here = 
		node->output_charge_level;
	debug_string += 
		" " + std::to_string(node->charger_properties.rate) + 
		" " + std::to_string(distance_from_prev) + 
		" " + std::to_string(car_range_left_enter_here) + 
		" " + std::to_string(car_range_left_leave_here) + " ";
	return debug_string;	
}

void DjikstraSolver::SingleSourceShortestPaths(
	const std::string& initial_charger_name) {

	PrepareNodes();
	PrepareSourceNodes(initial_charger_name, {num_charge_levels_});

	std::priority_queue<DjikstraNode*, std::vector<DjikstraNode*>, DjikstraComparator> djikstra_queue; 
	djikstra_queue.push(
		name_to_node_map_[
			GetDjikstraNodeName(
				*name_to_row_map_[initial_charger_name], 
				num_charge_levels_)]);
	while (!djikstra_queue.empty()) {
		DjikstraNode* min_element = djikstra_queue.top();
		djikstra_queue.pop();
		for (auto& neighbor : min_element->neighbors) {
			if (neighbor.first->travel_time_to_get_here >
				min_element->travel_time_to_get_here + 
				neighbor.second) {
				neighbor.first->prev = min_element;
				neighbor.first->travel_time_to_get_here = 
					min_element->travel_time_to_get_here + 
					neighbor.second;
				djikstra_queue.push(neighbor.first);
			}
		}
	}
}

void DjikstraSolver::PrepareNodes() {
	for (const auto& node : nodes_) {
		node->travel_time_to_get_here = kLargeTimeValue;
		node->prev = nullptr;
	}
}	

void DjikstraSolver::PrepareSourceNodes(
	const std::string& initial_charger_name,
	const std::vector<int>& start_charge_indices) {
	for (const auto charge_level_index : start_charge_indices) {
		DjikstraNode* potential_source = 
			name_to_node_map_[
				GetDjikstraNodeName(
					*name_to_row_map_[initial_charger_name], 
					charge_level_index)];
		potential_source->travel_time_to_get_here = 0.0;
		potential_source->output_charge_level = kCarCapacityKm;
	}
}

DjikstraSolver::~DjikstraSolver() {
	for (auto node: nodes_) {
		delete node;
	}
}
}  // namespace challenge 