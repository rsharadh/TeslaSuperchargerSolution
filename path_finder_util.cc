// Tesla SuperCharger Challenge Solution by Sharadh Ramaswamy
#include <algorithm>
#include <math.h>
#include <fstream>

#include "network.h"
#include "path_finder_util.h"
#include "node_distance_pair_map_pruned.h"

namespace challenge {
namespace {
constexpr double kPI = 3.14159265359;
constexpr double kMinLatDiff = 0.5;
constexpr double kMinLongDiff = 0.5;
constexpr double kMinDistance = 5;
constexpr int kMaxPowerOdd = 15;
constexpr double kFactorial[] = {
	1, 1, 2, 6, 24, 120, 
	720, 5040, 40320, 362880, 3628800, 39916800,
	479001600, 6227020800, 87178291200, 1307674368000
};

double CustomSin(double angle_radian) {
	double square = angle_radian * angle_radian;
	double prod = angle_radian;
	double sum = 0;
	int sign_prod = -1;
	for (int index = 1; index <= kMaxPowerOdd; index += 2) {
		sign_prod = -sign_prod;
		sum += sign_prod * prod / kFactorial[index];
		prod *= square;		
	}
	return sum;
}

double CustomCos(double angle_radian) {
	double square = angle_radian * angle_radian;
	double prod = square;
	double sum = 1;

	int sign_prod = 1;
	for (int index = 2; index < kMaxPowerOdd; index += 2) {
		sign_prod = -sign_prod;
		sum += sign_prod * prod / kFactorial[index];
		prod *= square;		
	}
	return sum;
}

double CustomASin(double value) {
	if (value >= 1) {
		value = 1.0;
	}
	if (value <= -1) {
		value = -1.0;
	}
	double square = value * value;
	double prod = value;
	double sum = 0;
	sum += prod;
	prod *= square;
	sum += prod / 6;
	prod *= square;
	sum += 3 * prod / 40;
	prod *= square;
	sum += 5 * prod / 112; 
	prod *= square;
	sum += 35 * prod / 1152; 
	return sum;
}

double DegreeToRadian(const double degree) {
	return degree * kPI / 180.0;
}

double SinDegree(const double degree) {
#ifdef CUSTOM_TRIG	
	return CustomSin(DegreeToRadian(degree));
#else
   return sin(DegreeToRadian(degree));
#endif	
}

double CosDegree(const double degree) {
#ifdef CUSTOM_TRIG	
	return CustomCos(DegreeToRadian(degree));
#else
	return cos(DegreeToRadian(degree));
#endif	
}

double ComputeASin(double value) {
	return asin(value);	
}

double Square(const double x) {
	return x * x;
}

double ComputeACos(double value) {
	return kPI / 2.0 - ComputeASin(value);
}

// https://en.wikipedia.org/wiki/Great-circle_distance#Formulae
double GreatCircleDistanceKmSimple(const row& a, const row& b) {
	double central_angle = ComputeACos(
		(SinDegree(a.lat) * SinDegree(b.lat)) + 
		(CosDegree(a.lat) * CosDegree(b.lat) * CosDegree(a.lon - b.lon)));
	return kEarthRadiusKm * central_angle;
}


// https://en.wikipedia.org/wiki/Great-circle_distance#Formulae
double GreatCircleDistanceKmHaversine(const row& a, const row& b) {
	double delta_lon = a.lon - b.lon;
	double delta_lat = a.lat - b.lat;
	double central_angle = 
		2.0 * 
		ComputeASin(
			sqrt(
				Square(SinDegree(delta_lat / 2.0)) + 
				Square(SinDegree(delta_lon / 2.0)) * 
				CosDegree(a.lat) * CosDegree(b.lat)
				)
			);
	return kEarthRadiusKm * central_angle;
}

double HypotenuseApproximation(const row& a, const row& b) {
	if (a.name == b.name) {
		return 0.0;
	}
	// Use pythogoras theorem and approximate distances.
	double abs_delta_lon = fabs(a.lon - b.lon);
	double abs_delta_lat = fabs(a.lat - b.lat);
	auto abs_delta_lon_double = kEarthRadiusKm * 1000 * abs_delta_lon / 180.0 * kPI;
	auto abs_delta_lat_double = kEarthRadiusKm * 1000 * abs_delta_lat / 180.0 * kPI;
	auto hypotenuse = sqrt(Square(abs_delta_lon_double) + Square(abs_delta_lat_double));
	return std::max(hypotenuse / 1000, kMinDistance);	
}
}  // namespace

double GreatCircleDistanceKm(
	const row& a, const row& b, const bool is_haversine) {
	double abs_delta_lon = fabs(a.lon - b.lon);
	double abs_delta_lat = fabs(a.lat - b.lat);
	if (abs_delta_lon < kMinLatDiff && 
		abs_delta_lat < kMinLongDiff) {
		return HypotenuseApproximation(a, b);
	}

	double distance =
		   is_haversine 
		   ? GreatCircleDistanceKmSimple(a, b) 
		   : GreatCircleDistanceKmHaversine(a, b);

   return distance;
} 

std::vector<std::pair<std::string, double>> FindNeighbors(const row& a, float error_discounter) {
	error_discounter = fmax(0.0, fmin(error_discounter, 1.0));
	std::vector<std::pair<std::string, double>> neighbors;
	for (const auto& candidate: network) {
		if (candidate.name == a.name) {
			continue;
		}
		double distance_to_candidate = GreatCircleDistanceKm(a, candidate);
		if (distance_to_candidate < error_discounter * kCarCapacityKm) {
			neighbors.push_back(std::make_pair(candidate.name, distance_to_candidate));
		}
	}
	return neighbors;
}

std::vector<std::pair<std::string, double>> FindNeighborsPreComputed(
	const row& a) {
	std::vector<std::pair<std::string, double>> neighbors;

	for (const auto& candidate: network) {
		if (candidate.name == a.name) {
			continue;
		}
		const auto& key_a_candidate = GenerateNodePairKey(a.name, candidate.name);
		if (node_pair_distance_map.find(key_a_candidate) 
			!= node_pair_distance_map.end()) {
			const auto distance_to_candidate = node_pair_distance_map[key_a_candidate];
			neighbors.push_back(std::make_pair(candidate.name, distance_to_candidate));
		}
	}
	return neighbors;
}


std::string GenerateNodePairKey(
	const std::string& charger_a, const std::string& charger_b) {
	std::string key = charger_a;
	key += ":";
	key += charger_b;
	return key;	
}

std::unordered_map<std::string, double> ReadNodePairMapFile(
	const std::string& node_pair_distance_map_file) {
  std::unordered_map<std::string, double> node_pair_distance_map;
  std::ifstream distance_map_file(node_pair_distance_map_file);
  if (!distance_map_file.good()) {
  	return node_pair_distance_map;
  }
  std::string one_line; 
  while (std::getline(distance_map_file, one_line)) {
  	std::string key_ab = one_line.substr(0,	one_line.find(' '));
  	double distance_ab = 
  		std::stof(
  			one_line.substr(
  				one_line.find(' ') + 1, 
  				one_line.size() - one_line.find(' '))); 
	node_pair_distance_map[key_ab] = distance_ab;
  }
  distance_map_file.close();	
  return node_pair_distance_map;
}

std::unordered_map<std::string, double> CreateNodePairMapFile(
	const std::string& node_pair_distance_map_file) {
  std::unordered_map<std::string, double> node_pair_distance_map = 
	  ReadNodePairMapFile(kNodePairMapFile);
  if (node_pair_distance_map.empty()) {
  	std::cout<<kNodePairMapFile<<" does not exist! Will create now."<<std::endl;
  } else {
  	std::cout<<kNodePairMapFile<<" exists!"<<std::endl;
  	return node_pair_distance_map;
  }		
  std::ofstream distance_map_file(node_pair_distance_map_file);
  for (const auto& a: network) {
  	for (const auto& b: network) {
  		if (a.name == b.name) {
  			continue;
  		}
  		const auto& key_ab = GenerateNodePairKey(a.name, b.name);
  		if (node_pair_distance_map.find(key_ab) == 
  			node_pair_distance_map.end()) {
	  		const auto& key_ba = GenerateNodePairKey(b.name, a.name);
	  		double distance_ab = GreatCircleDistanceKm(a, b);
	  		node_pair_distance_map[key_ab] = distance_ab;
	  		node_pair_distance_map[key_ba] = distance_ab;
			if (distance_map_file.is_open()) {
		    	distance_map_file << key_ab << " " << distance_ab << "\n";
		    	distance_map_file << key_ba << " " << distance_ab << "\n";
		  	}
		}
  	}
  }
  distance_map_file.close();	
  return node_pair_distance_map;
}

}  // namespace challenge 
