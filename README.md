Build candidate_solution with
g++ main.cpp network.cpp path_finder_util.cc brute_path_finder.cc djikstra_solver.cc node_distance_pair_map_pruned.cc -I=$PWD --std=c++11 -O1 -o candidate_solution

Solution based on

A) The Shortest Path with Charging Problem: A Scalable, Discretized Approach https://pdfs.semanticscholar.org/dd62/0e8e55dc27ec660b763a4d3a21af5940b66f.pdf

B) Djikstra algorithm for single source shortest paths.
