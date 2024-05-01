#include "common.h"
#include "graph.hpp"
// #include "serial.hpp"
#include "mpi.hpp"
#include <sstream>
#include <string>
#include "stdio.h"
#include <iostream>
#include "upcxx.hpp"

int main(int argc, char** argv){
   // mpi_astar(argc, argv);
    std::vector<Obstacle> obstacleList = {}; 
  //  AStarMap map = AStarMap(10, obstacleList);
    upcxx_astar(10,obstacleList);
    //process input
    //assume that all input files are of the form F:= H\n(C\n)*
    //H := <grid size> <solution value>
    //C := <point 1 x val> <point 1 y val> <point 2 x val> <point 2 y val>

    // std::string nextline; 
    // int index = 0; 
    // int grid_size = -1; 
    // int solution =-1; 
    // int startX, startY;
    // int endX, endY;
    // while (std::getline(std::cin, nextline)){
    //     int x1, y1, x2, y2; 
    //     std::istringstream iss(nextline); 
    //     if(index<1){
    //         iss>>grid_size>>solution>>x1>>y1>>x2>>y2; 
    //         startX = x1;
    //         startY = y1;
    //         endX = x2;
    //         endY = y2;
    //     }
    //     else{
    //         iss>>x1>>y1>>x2>>y2;
    //         obstacleList.push_back(Obstacle(x1,y1,x2,y2));         
    //     }
    //     index++; 
    // }
    // AStarMap map = AStarMap(grid_size, obstacleList, startX, startY, endX, endY);
    // int result = serial_astar(map); 
    // if (result!= solution){
    //     printf("expected  %d, got %d ", solution, result); 
    //     return 1; 
    // } 
    // printf("dist %d", result); 


    // int result = serial_astar(map);

    return 0;
}
