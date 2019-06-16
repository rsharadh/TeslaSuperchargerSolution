Build candidate_solution with

g++ main.cpp network.cpp path_finder_util.cc brute_path_finder.cc djikstra_solver.cc node_distance_pair_map_pruned.cc -I=$PWD --std=c++11 -O1 -o candidate_solution

Solution based on

A) The Shortest Path with Charging Problem: A Scalable, Discretized Approach https://pdfs.semanticscholar.org/dd62/0e8e55dc27ec660b763a4d3a21af5940b66f.pdf

B) Djikstra algorithm for single source shortest paths.

Sample Run:
$ time ./candidate_solution Mountain_View_CA Albany_NY
Mountain_View_CA, Mountain_View_CA, 0.0, Fresno_CA, 1.505389, Beatty_NV, 2.206551, St._George_UT, 1.706328, Richfield_UT, 1.290381, Green_River_UT, 1.160740, Glenwood_Springs_CO, 0.450984, Silverthorne_CO, 0.667939, Lone_Tree_CO, 1.566213, Goodland_KS, 1.251407, Hays_KS, 0.951429, Salina_KS, 1.733701, Topeka_KS, 1.322869, Columbia_MO, 2.775697, Effingham_IL, 0.965519, Terre_Haute_IN, 2.075938, Lima_OH, 1.353195, Macedonia_OH, 2.009742, Buffalo_NY, 2.026842, Liverpool_NY, 0.782145, Albany_NY

real	0m0.319s
user	0m0.288s
sys	0m0.019s

$ ./checker_osx "Mountain_View_CA, Mountain_View_CA, 0.0, Fresno_CA, 1.505389, Beatty_NV, 2.206551, St._George_UT, 1.706328, Richfield_UT, 1.290381, Green_River_UT, 1.160740, Glenwood_Springs_CO, 0.450984, Silverthorne_CO, 0.667939, Lone_Tree_CO, 1.566213, Goodland_KS, 1.251407, Hays_KS, 0.951429, Salina_KS, 1.733701, Topeka_KS, 1.322869, Columbia_MO, 2.775697, Effingham_IL, 0.965519, Terre_Haute_IN, 2.075938, Lima_OH, 1.353195, Macedonia_OH, 2.009742, Buffalo_NY, 2.026842, Liverpool_NY, 0.782145, Albany_NY"
Finding Path Between Mountain_View_CA and Albany_NY
Reference result: Success, cost was 70.1723
Candidate result: Success, cost was 69.4671


