#include "common.h"
#include "graph.hpp"
#include "serial.hpp"

int main(){
    std::vector<Obstacle> obstacleList = {{{1, 1}, {5, 5}}, {{7, 7}, {9, 9}}};
    AStarMap map = AStarMap(10, obstacleList);
    map.render();
    serial_astar(10);
    return 0;
}