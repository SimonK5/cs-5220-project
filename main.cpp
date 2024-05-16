#include "common.h"
#include "graph.hpp"
#include "serial.hpp"
//#include "mpi.hpp"
#include <sstream>
#include <string>
#include "stdio.h"
#include <iostream>
#include "upcxx.hpp"
#include <upcxx/upcxx.hpp>

int main(int argc, char** argv){

    auto start_time = std::chrono::steady_clock::now();
    upcxx::init(); 
    upcxx_astar(100, std::vector<Obstacle>(),5,5,95,95); 
    upcxx::finalize();
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    double seconds = diff.count();
    if(upcxx::rank_me()==0)std::cout << upcxx::rank_me()<<" Time taken (s): " << seconds << std::endl; 
   
    return 0;
}
