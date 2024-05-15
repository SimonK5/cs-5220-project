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
   /*UPCXX_SERIALIZED_FIELDS(x);
    UPCXX_SERIALIZED_FIELDS(y);
    UPCXX_SERIALIZED_FIELDS(parentX);
    UPCXX_SERIALIZED_FIELDS(parentY);
    UPCXX_SERIALIZED_FIELDS(cost_to_come);
    UPCXX_SERIALIZED_FIELDS(heuristic_cost); 
    */ 
    
    struct upcxx_serialization{
        //using Berkeley example for syntax
        template<typename Writer> 
        static void serialize(Writer& writer, Node const & object){
            writer.write(x); 
            writer.write(y); 
            writer.write(parentX);
            writer.write(parentY); 
            writer.write(cost_to_come); 
            writer.write(heuristic_cost); 
        }
        template<typename Reader, typename Storage> 
        static Node* deserialize(Reader& reader, Storage storage){
            int x = reader.template read<int>(); 
            int y = reader.template read<int>();
            int parentx = reader.template read<int>(); 
            int parenty = reader.template read<int>();
            Node *n = storage.construct(x,y, parentx, parenty); 
            n->cost_to_come = reader.template read<float> (); 
            n->heuristic_cost = reader.template read <float>(); 
            return n;
             //read in the fields we need to reconstruct node 
        }
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
    /*UPCXX_SERIALIZED_FIELDS(x1);
    UPCXX_SERIALIZED_FIELDS(y1);
    UPCXX_SERIALIZED_FIELDS(x2);
    UPCXX_SERIALIZED_FIELDS(y2); 
    */
    struct upcxx_serialization{
        //using Berkeley example for syntax
        template<typename Writer> 
        static void serialize(Writer& writer, Obstacle const & object){
            writer.write(x1); 
            writer.write(y1); 
            writer.write(x2);
            writer.write(y2); 
        }
        template<typename Reader, typename Storage> 
        static Obstacle* deserialize(Reader& reader, Storage storage){
            int x1 = reader.template read<int>(); 
            int y1 = reader.template read<int>();
            int x2 = reader.template read<int>(); 
            int y2 = reader.template read<int>();
            Obstacle *obs = storage.construct(x1,y1, x2,y2); 
            return obs;
        }
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
    AStarMap(){
    }
    AStarMap(int s) : size(s){
        std::vector<std::vector<char>> initGrid(size, std::vector<char>(size));
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
    struct upcxx_serialization{
        //using Berkeley example for syntax
        template<typename Writer> 
        static void serialize(Writer& writer, AStarMap const & object){
            writer.write(startX);
            writer.write(startY); 
            writer.write(endX); 
            writer.write(endY); 
            writer.write(size);  
            writer.write(seed);   
        }
        template<typename Reader, typename Storage> 
        static AStarMap* deserialize(Reader& reader, Storage storage){
            int startx = reader.template read<int>(); 
            int starty = reader.template read<int>(); 
            int endx = reader.template read<int>(); 
            int endy = reader.template read<int>(); 
            int size = reader.template read<int>(); 
            int seed = reader.template read<int>(); 
            AStarMap *a = storage.construct(size, std::vector<Obstacle>(), startx, starty, endx, endy); 
             a->seed = seed; 
             return a; 
             //read in the fields we need to reconstruct node 
        }
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