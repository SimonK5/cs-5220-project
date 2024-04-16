#include "common.h"
#include "graph.hpp"
#include "serial.hpp"
#include "mpi.hpp"


int main(int argc, char* argv[]){
    // std::vector<Obstacle> obstacleList = {{{1, 1}, {5, 5}}, {{7, 7}, {9, 9}}};
    std::vector<Obstacle> obstacleList = {};
    // serial_astar(10);
    AStarMap map = AStarMap(10, obstacleList);
    map.print_assignments(10);

    // int num_procs, rank;
    // MPI_Init(&argc, &argv);
    // MPI_Comm_size(MPI_COMM_WORLD, &num_procs);
    // MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    

    return 0;
}