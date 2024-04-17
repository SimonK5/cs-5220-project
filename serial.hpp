#ifndef SERIAL
#define SERIAL

#include "graph.hpp"
#include <vector>
#include <queue>
#include <unordered_set>
#include <iostream>
#include "stdio.h"

int serial_astar(int grid_size, std::vector<Obstacle> obstacleList, Point startPoint, Point endPoint){
    AStarMap map = AStarMap(grid_size, obstacleList,startPoint, endPoint);
    
    std::priority_queue<Node*, std::vector<Node*>, NodeCompare> open_queue;
    std::unordered_set<Node*, NodeHash, NodeEqual> closed_set;

    open_queue.push(map.start);
    std::cout << map.start->pos.x << " " << map.start->pos.y << std::endl;
    std::cout << map.goal->pos.x << " " << map.goal->pos.y << std::endl;
    bool path_found = false;
    Node* end_node;
    while(open_queue.size() > 0){
        Node* cur = open_queue.top();

        open_queue.pop();
        if(closed_set.find(cur) != closed_set.end()){
            continue;
        }
        closed_set.emplace(cur);
        map.close_node(cur->pos.x, cur->pos.y);

        if(*cur == *(map.goal)){
            path_found = true;
            end_node = cur;
            break;
        }

        std::vector<std::vector<int>> dirn = cur->get_neighbor_directions();
        for(std::vector<int> d : dirn){
            Node *n = new Node(cur->pos.x + d[0], cur->pos.y + d[1]);
            if(closed_set.find(n) != closed_set.end() || !map.is_valid_node(*n)){
                continue;
            }
            n->cost_to_come = cur->cost_to_come + 1;
            n->heuristic_cost = n->cost_to_come + n->heuristic(*(map.goal));
            n->parent = cur;
            open_queue.push(n);
            map.open_node(n->pos.x, n->pos.y);
        }
    }

    if(path_found){
        Node* cur = end_node;
        while(cur != nullptr && cur->parent != nullptr){
            cur = cur->parent;
            map.add_to_path(cur->pos.x, cur->pos.y);
        }
    }

    for (auto it = closed_set.begin(); it != closed_set.end(); ++it) {
        delete *it;
    }
    closed_set.clear();

    while (!open_queue.empty()) {
        Node *n = open_queue.top();
        open_queue.pop();
        delete n;
    }

    map.render();
    return end_node->cost_to_come;
}

#endif