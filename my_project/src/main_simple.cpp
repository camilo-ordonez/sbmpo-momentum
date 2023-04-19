#include <sbmpo/sbmpo.hpp>
#include <my_project/MomentumModel.hpp>
#include <iostream>

using namespace my_namespace;

int main(int argc, char ** argv) {

  MomentumModel model;

  sbmpo::SBMPOParameters params;
  /* Add in parameters here */


  params.max_iterations = 50000;
  params.max_generations = 100;
  params.sample_time = 0.1;
  params.grid_resolution = {0.005, 0.0005};
  params.start_state = {0, 0};
  params.goal_state = {15, 0};
  params.samples = {
    {-1},
    {1}
  };


  
  sbmpo::SBMPO planner(model, params);
  planner.run();
  
  std::cout << "---- Planner Results ----" << std::endl;
  std::cout << "Iterations: " << planner.iterations() << std::endl;
  std::cout << "Exit code: " << planner.exit_code() << std::endl;
  std::cout << "Computation Time: " << planner.time_us() << "us" << std::endl;
  std::cout << "Path cost: " << planner.cost() << std::endl;
  std::cout << "Number of nodes: " << planner.size() << std::endl;
  
  std::cout << "-- State Path --" << std::endl;
  for (sbmpo::State state : planner.state_path()) {
    std::cout << "  - [ ";
    for (float s : state) {
      std::cout << s << " ";
    }
    std::cout << "]" << std::endl;
  }
  
  std::cout << "-- Control Path --" << std::endl;
  for (sbmpo::Control control : planner.control_path()) {
    std::cout << "  - [ ";
    for (float c : control) {
      std::cout << c << " ";
    }
    std::cout << "]" << std::endl;
  }
  
  return 0;
}







