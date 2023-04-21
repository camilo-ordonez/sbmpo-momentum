#include <my_project/MomentumModelUnicycle.hpp>
#include <sbmpo_benchmarking/benchmark.hpp>

using namespace sbmpo;
//using namespace sbmpo_models;
using namespace my_namespace;
using namespace sbmpo_benchmarking;

int main (int argc, char ** argv) {

    // Path to csv workspace
    std::string csv_folder = "/home/camilo/Desktop/sbmpo-momentum/my_project_ws/my_project/csv/";

/*
    // Check arguments
    if(argc > 1){
            csv_folder = argv[1];
    } else{
        printf("\nMIssing CSV folder path .\n");
        return 0;
    }
*/

    // Create new benchmark
    Benchmark<MomentumModelUnicycle> benchmarker(csv_folder);

    // Run benchmark on the model (saves to csv folder)
    benchmarker.benchmark();

    return 0;


}