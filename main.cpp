#include "common.h"
#include "graph.hpp"
#include "serial.hpp"

int main(){
    // std::vector<Obstacle> obstacleList = {{{1, 1}, {5, 5}}, {{7, 7}, {9, 9}}};
    std::vector<Obstacle> obstacleList = {};
    serial_astar(10);
    return 0;
}