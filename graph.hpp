#ifndef GRAPH
#define GRAPH
#include <vector>
#include "common.h"
#include <iostream>


class Point{
public:
    Point(int xCoord, int yCoord) : x(xCoord), y(yCoord) {}
    int x, y;
    float dist(Point other) const{
        return sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
    }
};

class Node{
public:
    Node(int x, int y) : pos(Point(x, y)), parent(nullptr), cost_to_come(0.0), heuristic_cost(0.0) {}
    Node(Point p) : pos(p), parent(nullptr), cost_to_come(0.0), heuristic_cost(0.0) {}
    Point pos;
    Node* parent;
    float cost_to_come;
    float heuristic_cost;

    std::vector<std::vector<int>> get_neighbor_directions(){
        std::vector<std::vector<int>> dirn = {{1, 0}, {0, 1}, {0, -1}, {-1, 0}};
        return dirn;
    }

    float heuristic(Node other){
        return pos.dist(other.pos);
    }
};

bool operator<(const Node& a, const Node& b) {
    return a.heuristic_cost < b.heuristic_cost;
}
bool operator==(const Node& a, const Node& b) {
    return a.pos.x == b.pos.x && a.pos.y == b.pos.y;
}

class Obstacle{
public:
    Point p1; // bottom left corner (low x, low y)
    Point p2; // top right corner (high x, high y)
    bool contains(Point p){
        return p.x >= p1.x && p.x <= p2.x && p.y >= p1.y && p.y <= p2.y;
    }
};

class AStarMap{
public:
    Node start;
    Node goal;
    int size;
    std::vector<Obstacle> obstacles;
    std::vector<std::vector<char>> grid;

    AStarMap(int s, std::vector<Obstacle> obstacleList) : size(s), start(Node(-1, -1)), goal(Node(-1, -1)){
        std::vector<std::vector<char>> initGrid(size, std::vector<char>(size));
        for(int i = 0; i < size; i++){
            for(int j = 0; j < size; j++){
                Point p = Point(i, j);
                initGrid[i][j] = '.';
                for(Obstacle o : obstacleList){
                    if(o.contains(p)){
                        initGrid[i][j] = 'X';
                    }
                }
            }
        }
        std::uniform_int_distribution<> distrib(0, s-1);
        Point startPoint = Point(distrib(gen), distrib(gen));
        while(initGrid[startPoint.x][startPoint.y] == 'X'){
            startPoint = Point(distrib(gen), distrib(gen));
        }
        initGrid[startPoint.x][startPoint.y] = 'S';
        start = Node(startPoint);

        Point endPoint = Point(distrib(gen), distrib(gen));
        while(initGrid[endPoint.x][endPoint.y] == 'X'){
            endPoint = Point(distrib(gen), distrib(gen));
        }
        initGrid[endPoint.x][endPoint.y] = 'G';
        goal = Node(endPoint);
        grid = initGrid;
    }
    
    void render(){
        for(int i = 0; i < size; i++){
            for(int j = 0; j < size; j++){
                std::cout << grid[i][j];
            }
            std::cout << std::endl;
        }
    }
};

#endif