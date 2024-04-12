#ifndef GRAPH
#define GRAPH
#include <vector>
#include "common.h"
#include <iostream>
#include <cmath>


class Point{
public:
    Point(int xCoord, int yCoord) : x(xCoord), y(yCoord) {}
    int x, y;
    float eucliden_dist(Point other) const{
        int dx = x - other.x;
        int dy = y - other.y;
        return sqrt(dx * dx + dy * dy);
    }
    float manhattan_dist(Point other) const{
        int dx = x - other.x;
        int dy = y - other.y;
        return std::abs(dx) + std::abs(dy);
    }
};

/**
 * A node in the graph generated by A*.
*/
class Node{
public:
    Node(int x, int y) : pos(Point(x, y)), parent(nullptr), cost_to_come(0.0), heuristic_cost(0.0) {}
    Node(Point p) : pos(p), parent(nullptr), cost_to_come(0.0), heuristic_cost(0.0) {}
    Node(const Node& other) : pos(other.pos), parent(other.parent) {}
    Point pos;
    Node* parent;
    float cost_to_come;
    float heuristic_cost;

    std::vector<std::vector<int>> get_neighbor_directions(){
        std::vector<std::vector<int>> dirn = {{1, 0}, {0, 1}, {0, -1}, {-1, 0}};
        return dirn;
    }

    float heuristic(Node other){
        return pos.manhattan_dist(other.pos);
    }

    bool operator==(const Node& other) const {
        return pos.x == other.pos.x && pos.y == other.pos.y;
    }
};

// Necessary for using an unordered_set of Nodes
struct NodeHash {
    std::size_t operator()(const Node* n) const {
        std::size_t hashX = std::hash<int>{}(n->pos.x);
        std::size_t hashY = std::hash<int>{}(n->pos.y);
        return hashX ^ (hashY + 0x9e3779b9 + (hashX << 6) + (hashX >> 2));
    }
};

struct NodeEqual {
    bool operator()(const Node* a, const Node* b) const {
        return a->pos.x == b->pos.x && a->pos.y == b->pos.y;
    }
};

// Necessary for using a priority_queue of Nodes
struct NodeCompare {
    bool operator()(const Node* n1, const Node* n2) const {
        return n1->heuristic_cost > n2->heuristic_cost;
    }
};

// bool operator<(const Node& a, const Node& b) {
//     return a.heuristic_cost < b.heuristic_cost;
// }

/**
 * A rectangular obstacle.
*/
class Obstacle{
public:
    Point p1; // bottom left corner (low x, low y)
    Point p2; // top right corner (high x, high y)
    bool contains(Point p){
        return p.x >= p1.x && p.x <= p2.x && p.y >= p1.y && p.y <= p2.y;
    }
};

/**
 * A grid containing obstacles, nodes, and empty space.
*/
class AStarMap{
public:
    Node *start;
    Node *goal;
    int size;
    std::vector<Obstacle> obstacles;
    std::vector<std::vector<char>> grid;

    AStarMap(int s, std::vector<Obstacle> obstacleList) : size(s), start(nullptr), goal(nullptr){
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
        start = new Node(startPoint);

        Point endPoint = Point(distrib(gen), distrib(gen));
        while(initGrid[endPoint.x][endPoint.y] == 'X' || endPoint.x == startPoint.x && endPoint.y == startPoint.y){
            endPoint = Point(distrib(gen), distrib(gen));
        }
        initGrid[endPoint.x][endPoint.y] = 'G';
        goal = new Node(endPoint);
        grid = initGrid;
    }

    bool in_bounds(Node n){
        return n.pos.x >= 0 && n.pos.y >= 0 && n.pos.x < size && n.pos.y < size;
    }

    bool is_valid_node(Node n){
        return in_bounds(n) && grid[n.pos.x][n.pos.y] != 'X';
    }

    void add_to_path(int i, int j){
        if(grid[i][j] == 'S' || grid[i][j] == 'G') return;
        grid[i][j] = 'P';
    }

    void open_node(int i, int j){
        if(grid[i][j] == 'S' || grid[i][j] == 'G') return;
        grid[i][j] = 'E';
    }

    void close_node(int i, int j){
        if(grid[i][j] == 'S' || grid[i][j] == 'G') return;
        grid[i][j] = 'C';
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