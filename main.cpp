// Tesla SuperCharger Challenge Solution by Sharadh Ramaswamy
#include "network.h"
#include "path_finder_util.h"
#include "brute_path_finder.h"
#include "djikstra_solver.h"


int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Error: requires initial and final supercharger names" << std::endl;        
        return -1;
    }
    
    std::string initial_charger_name = argv[1];
    std::string goal_charger_name = argv[2];

    challenge::DjikstraSolver djikstra_solver(/*num_charge_levels=*/ 10);
    std::cout<<djikstra_solver.GetPath(
    	initial_charger_name, 
    	goal_charger_name)<<std::endl;
    return 0;
}
