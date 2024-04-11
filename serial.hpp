#ifndef SERIAL
#define SERIAL

#include "graph.hpp"
#include <vector>
#include <queue>
#include <unordered_set>


void serial_astar(int grid_size){
    std::vector<Obstacle> obstacleList = {{{1, 1}, {5, 5}}, {{7, 7}, {9, 9}}};
    AStarMap map = AStarMap(grid_size, obstacleList);
    
    std::priority_queue<Node, std::vector<Node>, NodeCompare> open_queue;
    std::unordered_set<Node, NodeHash> open_set;
    std::unordered_set<Node, NodeHash> closed_set;

    open_set.emplace(map.start);
    open_queue.push(map.start);
    while(open_set.size() > 0){
        std::cout << open_set.size() << std::endl;
        Node cur = open_queue.top();
        open_queue.pop();
        open_set.erase(cur);

        if(closed_set.find(cur) != closed_set.end()){
            continue;
        }
        closed_set.emplace(cur);

        std::vector<std::vector<int>> dirn = cur.get_neighbor_directions();
        for(std::vector<int> d : dirn){
            Node n = Node(cur.pos.x + d[0], cur.pos.y + d[1]);
            auto found_node = closed_set.find(n);
            if(found_node != closed_set.end()){
                continue;
            }
            auto found_open = open_set.find(n);
            if(found_open != open_set.end()){
                n = *found_open;
                int new_cost_to_come = cur.cost_to_come + 1;
                int new_heuristic_cost = new_cost_to_come + n.heuristic(map.goal);
                
            }
            else{
                n.cost_to_come = cur.cost_to_come + 1;
                n.heuristic_cost = n.cost_to_come + n.heuristic(map.goal);
                open_queue.push(n);
                open_set.emplace(n);
            }
        }
    }
}

#endif