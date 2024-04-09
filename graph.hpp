#ifndef GRAPH
#define GRAPH
#include <vector>


class Point{
public:
    Point(int xCoord, int yCoord) : x(xCoord), y(yCoord) {}
    int x, y;
    float dist(Point other) const{
        return sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
    }
};

bool operator<(const Node& a, const Node& b) {
    return a.heuristic_cost < b.heuristic_cost;
}
bool operator==(const Node& a, const Node& b) {
    return a.pos.x == b.pos.x && a.pos.y == b.pos.y;
}

class Node{
public:
    Node(int x, int y) : pos(Point(x, y)), parent(nullptr), cost_to_come(0.0), heuristic_cost(0.0) {}
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

class Obstacle{
public:
    Point p1;
    Point p2;
};

class World{
public:
    Node *start;
    Node *end;
    int width;
    int height;
    std::vector<Obstacle> obstacles;
};

#endif