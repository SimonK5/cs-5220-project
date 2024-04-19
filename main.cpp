#include "common.h"
#include "graph.hpp"
#include "serial.hpp"
#include <sstream>
#include <string>
#include "stdio.h"
#include <iostream>

int main(){
    std::vector<Obstacle> obstacleList = {};
    
    //process input
    //assume that all input files are of the form F:= H\n(C\n)*
    //H := <grid size> <solution value>
    //C := <point 1 x val> <point 1 y val> <point 2 x val> <point 2 y val>

    std::string nextline; 
    int index = 0; 
    int grid_size = -1; 
    int solution =-1; 
    Point startPoint;
    Point endPoint; 
    while (std::getline(std::cin, nextline)){
        int x1, y1,x2, y2; 
        std::istringstream iss(nextline); 
        if(index<1){
            iss>>grid_size>>solution>>x1>>y1>>x2>>y2; 
            startPoint = {x1,y1}; 
            endPoint = {x2, y2}; 
        }
        else{
        iss>>x1>>y1>>x2>>y2; 
        obstacleList.push_back( {{x1,y1},{x2,y2}});         
        }
        index++; 
    }
    int result =serial_astar(grid_size, obstacleList, startPoint, endPoint); 
    if (result!= solution){
        printf("expected  %d, got %d ", solution, result); 
        return 1; 
    } 
    printf("dist %d", result); 


    // int result = serial_astar(10, obstacleList);

    return 0;
}