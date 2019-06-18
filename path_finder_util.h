// Tesla SuperCharger Challenge Solution by Sharadh Ramaswamy
#pragma once
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>
#include "network.h"

namespace challenge {
constexpr double kEarthRadiusKm = 6356.752;
constexpr double kCarSpeedKmPh = 105;
constexpr double kCarCapacityKm = 320;
constexpr char kNodePairMapFile[] = "node_pair_distance_map.txt";	

// Given row a and b, computes great-circle distance assuming that
// the earth is a sphere of radius kEarthRadiusKm. 
// Note :-
// 1. If is_haversine is true, the Haversine formula is used. 
// 2. If is_haversine is false, the central angle formulation is used.
// 3. If the absolute difference in latitudes and longitudes is < kMinLatDiff,
//    the great-circle approximated by the Pythagoras theorem (in order to
//    have high sensitivity for close enough points). The minimum sensitivity
//    is restricted to be  kMinDistance (5 km).
double GreatCircleDistanceKm(
  const row& a, 
  const row& b, 
  const bool is_haversine = true);

// Finds all neighbors for a given row "a" within the network, where two
// row elements are neighbors if they lie within kCarCapacityKm (320 km)
// of each other.
std::vector<std::pair<std::string, double>> 
FindNeighbors(const row& a, float error_discounter);

// Same as FindNeighbors, but uses pre-computed inter-row distances
// for inter-row distances <= kCarCapacityKm (320 km). The pre-computed
// inter-row distances <= kCarCapacityKm (320 km) are accessed through
// a node-pair distance map, declared in node_distance_pair_map_pruned.h
// and defined in node_distance_pair_map_pruned.cc.
std::vector<std::pair<std::string, double>> FindNeighborsPreComputed(
	const row& a);

// Generates a key given the names of two charging stations, in order to
// obtain the distance between them from (pre-computed) node-pair distance map.
std::string GenerateNodePairKey(
	const std::string& charger_a, const std::string& charger_b);

// Util method to read the node-pair distance map from the raw node-pair
// distance map .txt file.
std::unordered_map<std::string, double> ReadNodePairMapFile(
	const std::string& node_pair_distance_map_file);

// Util method to read the node-pair distance map from the raw node-pair
// distance map .txt file, if it exists. Else create it.
std::unordered_map<std::string, double> CreateNodePairMapFile(
	const std::string& node_pair_distance_map_file);

}  // namespace challenge
