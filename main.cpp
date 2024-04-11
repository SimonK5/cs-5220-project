#include "common.h"
#include "graph.hpp"

int main(){
    std::vector<Obstacle> obstacleList = {{{1, 1}, {5, 5}}, {{7, 7}, {9, 9}}};
    AStarMap map = AStarMap(10, obstacleList);
    map.render();
    return 0;
}