#ifndef GRAPH
#define GRAPH
#include <vector>
#include "common.h"
#include <iostream>
#include <cmath>

/**
 * A node in the graph generated by A*.
*/
class Node{
public:
    int x, y;
    int parentX, parentY; // for MPI sending purposes. Actual node objects will be stored in a separate data structure
    float cost_to_come;
    float heuristic_cost;
    
    Node(int xPos, int yPos) : x(xPos), y(yPos), parentX(0), parentY(0), cost_to_come(0.0), heuristic_cost(0.0) {}
    Node(int xPos, int yPos, int xParent, int yParent) : x(xPos), y(yPos), parentX(xParent), parentY(yParent), cost_to_come(0.0), heuristic_cost(0.0) {}
    Node(const Node& other) : x(other.x), y(other.y), parentX(other.parentX), parentY(other.parentY), cost_to_come(other.cost_to_come), heuristic_cost(other.heuristic_cost) {}
    Node() : x(0), y(0), parentX(0), parentY(0), cost_to_come(0.0), heuristic_cost(0.0) {}

    std::vector<std::vector<int>> get_neighbor_directions(){
        std::vector<std::vector<int>> dirn = {{1, 0}, {0, 1}, {0, -1}, {-1, 0}};
        return dirn;
    }

    float heuristic(Node other){
        return manhattan_dist(other);
    }

    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }

    float euclidean_dist(Node other) const{
        int dx = x - other.x;
        int dy = y - other.y;
        return sqrt(dx * dx + dy * dy);
    }

    float manhattan_dist(Node other) const{
        int dx = x - other.x;
        int dy = y - other.y;
        return std::abs(dx) + std::abs(dy);
    }

    float get_weight(Node other, unsigned int seed) {
        std::size_t hash = std::hash<int>{}(x) ^ std::hash<int>{}(y) ^
                        std::hash<int>{}(other.x) ^ std::hash<int>{}(other.y) ^ std::hash<unsigned int>{}(seed);
        std::mt19937 rng(hash);
        std::uniform_real_distribution<float> dist(1, 10);
        float weight = dist(rng);
        return weight;
    }
};

// Necessary for using an unordered_set of Nodes
struct NodeHash {
    std::size_t operator()(const Node &n) const {
        std::size_t hashX = std::hash<int>{}(n.x);
        std::size_t hashY = std::hash<int>{}(n.y);
        return hashX ^ (hashY + 0x9e3779b9 + (hashX << 6) + (hashX >> 2));
    }
};

struct NodeEqual {
    bool operator()(const Node &a, const Node &b) const {
        return a.x == b.x && a.y == b.y;
    }
};

// Necessary for using a priority_queue of Nodes
struct NodeCompare {
    bool operator()(const Node &n1, const Node &n2) const {
        return n1.heuristic_cost > n2.heuristic_cost;
    }
};

/**
 * A rectangular obstacle.
*/
class Obstacle{
public:
    int x1, y1; // bottom left corner (low x, low y)
    int x2, y2; // top right corner (high x, high y)
    Obstacle(int xl, int yl, int xr, int yr) : x1(xl), y1(yl), x2(xr), y2(yr) {}
    bool contains(int x, int y){
        return x >= x1 && x <= x2 && y >= y1 && y <= y2;
    }
};

/**
 * A grid containing obstacles, nodes, and empty space.
*/
class AStarMap{
public:
    int startX;
    int startY;
    int endX;
    int endY;

    int size;
    std::vector<Obstacle> obstacles;
    std::vector<std::vector<char>> grid;
    int seed = 5;

    AStarMap(int s, std::vector<Obstacle> obstacleList) : size(s){
        std::vector<std::vector<char>> initGrid(size, std::vector<char>(size));
        for(int i = 0; i < size; i++){
            for(int j = 0; j < size; j++){
                initGrid[i][j] = '.';
                for(Obstacle o : obstacleList){
                    if(o.contains(i, j)){
                        initGrid[i][j] = 'X';
                    }
                }
            }
        }
        std::uniform_int_distribution<> distrib(0, s-1);
        startX = distrib(gen);
        startY = distrib(gen);
        while(initGrid[startX][startY] == 'X'){
            startX = distrib(gen);
            startY = distrib(gen);
        }
        initGrid[startX][startY] = 'S';

        endX = distrib(gen);
        endY = distrib(gen);
        while(initGrid[endX][endY] == 'X' || initGrid[endX][endY] == 'S'){
            endX = distrib(gen);
            endY = distrib(gen);
        }
        initGrid[endX][endY] = 'G';
        grid = initGrid;
    }
     AStarMap(int s, std::vector<Obstacle> obstacleList, int sX, int sY, int eX, int eY) : size(s), startX(sX), startY(sY), endX(eX), endY(eY){
        std::vector<std::vector<char>> initGrid(size, std::vector<char>(size));
        for(int i = 0; i < size; i++){
            for(int j = 0; j < size; j++){
                initGrid[i][j] = '.';
                for(Obstacle o : obstacleList){
                    if(o.contains(i, j)){
                        initGrid[i][j] = 'X';
                    }
                }
            }
        } 
        initGrid[startX][startY] = 'S';
        initGrid[endX][endY] = 'G';
        grid = initGrid;
    }
    bool in_bounds(Node n){
        return n.x >= 0 && n.y >= 0 && n.x < size && n.y < size;
    }

    bool is_valid_node(Node n){
        return in_bounds(n) && grid[n.x][n.y] != 'X';
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

    std::size_t get_proc(Node n, int N){
        NodeHash nh;
        return nh(n) % N;
    }

    void print_assignments(int N){
        for(int i = 0; i < size; i++){
            for(int j = 0; j < size; j++){
                if(grid[i][j] == 'X'){
                    std::cout << 'X';
                }
                else{
                    NodeHash nh;
                    Node n = Node(i, j);
                    std::size_t test = nh(n);

                    std::cout << test % N;
                }
                
            }
            std::cout << std::endl;
        }
    }

    float get_edge_weight(Node parent, Node child){
        return parent.get_weight(child, seed);
    }
};

// /**
//  * A* map using pairs instead of objects as nodes.
// */
// class AStarMapParallel{
//     std::pair<int, int> start;
//     std::pair<int, int> end;
//     int size;
//     std::vector<Obstacle> obstacles;
//     std::vector<std::vector<char>> grid;

//     AStarMap(int s, std::vector<Obstacle> obstacleList)
//         : size(s), start(startPoint), end(endPoint), obstacles(obsList) {
//             std::vector<std::vector<char>> initGrid(size, std::vector<char>(size));
//         for(int i = 0; i < size; i++){
//             for(int j = 0; j < size; j++){
//                 Point p = Point(i, j);
//                 initGrid[i][j] = '.';
//                 for(Obstacle o : obstacleList){
//                     if(o.contains(p)){
//                         initGrid[i][j] = 'X';
//                     }
//                 }
//             }
//         }
//         std::uniform_int_distribution<> distrib(0, s-1);
//         Point startPoint = Point(distrib(gen), distrib(gen));
//         while(initGrid[startPoint.x][startPoint.y] == 'X'){
//             startPoint = Point(distrib(gen), distrib(gen));
//         }
//         initGrid[startPoint.x][startPoint.y] = 'S';
//         start = new Node(startPoint);

//         Point endPoint = Point(distrib(gen), distrib(gen));
//         while(initGrid[endX][endY] == 'X' || endX == startPoint.x && endY == startPoint.y){
//             endPoint = Point(distrib(gen), distrib(gen));
//         }
//         initGrid[endX][endY] = 'G';
//         goal = new Node(endPoint);
//         grid = initGrid;
//     }
// };

#endif