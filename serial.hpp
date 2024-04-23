#ifndef SERIAL
#define SERIAL

#include "graph.hpp"
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include "stdio.h"

int serial_astar(AStarMap &map){    
    std::priority_queue<Node, std::vector<Node>, NodeCompare> open_queue;
    std::unordered_set<Node, NodeHash, NodeEqual> closed_set;
    std::unordered_map<Node, Node, NodeHash, NodeEqual> node_to_parent;

    open_queue.push(*map.start);
    std::cout << map.start->x << " " << map.start->y << std::endl;
    std::cout << map.goal->x << " " << map.goal->y << std::endl;
    bool path_found = false;
    Node* end_node;
    while(open_queue.size() > 0){
        Node cur = open_queue.top();

        open_queue.pop();
        if(closed_set.find(cur) != closed_set.end()){
            continue;
        }
        closed_set.emplace(cur);
        map.close_node(cur.x, cur.y);

        if(cur == *(map.goal)){
            path_found = true;
            end_node = &cur;
            break;
        }

        std::vector<std::vector<int>> dirn = cur.get_neighbor_directions();
        for(std::vector<int> d : dirn){
            Node n = Node(cur.x + d[0], cur.y + d[1]);
            if(closed_set.find(n) != closed_set.end() || !map.is_valid_node(n)){
                continue;
            }
            n.cost_to_come = cur.cost_to_come + 1;
            n.heuristic_cost = n.cost_to_come + n.heuristic(*(map.goal));
            Node new_parent = Node(cur.x, cur.y);
            new_parent.cost_to_come = cur.cost_to_come;
            new_parent.heuristic_cost = cur.heuristic_cost;
            node_to_parent[n] = new_parent;
            open_queue.push(n);
            map.open_node(n.x, n.y);
        }
    }

    if(path_found){
        Node cur(end_node->x, end_node->y);

        while (node_to_parent.find(cur) != node_to_parent.end()) {
            Node parent = node_to_parent[cur];
            map.add_to_path(cur.x, cur.y);
            cur = parent;
        }
    }

    map.render();
    return end_node->cost_to_come;
}

#endif