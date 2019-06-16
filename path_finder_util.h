// Tesla SuperCharger Challenge Solution by Sharadh Ramaswamy
#pragma once
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>
#include "network.h"

namespace challenge {
constexpr double kMinDistance = 5;
constexpr double kEarthRadiusKm = 6356.752;
constexpr double kCarSpeedKmPh = 105;
constexpr double kCarCapacityKm = 320;
constexpr char kNodePairMapFile[] = "node_pair_distance_map.txt";	

double GreatCircleDistanceKm(
  const row& a, 
  const row& b, 
  const bool is_haversine = true);

std::vector<std::pair<std::string, double>> 
FindNeighbors(const row& a, float error_discounter);

std::vector<std::pair<std::string, double>> FindNeighborsPreComputed(
	const row& a);

std::string GenerateNodePairKey(const std::string& string_a, const std::string& string_b);

std::unordered_map<std::string, double> ReadNodePairMapFile(
	const std::string& node_pair_distance_map_file);

std::unordered_map<std::string, double> CreateNodePairMapFile(
	const std::string& node_pair_distance_map_file);

}  // namespace challenge
