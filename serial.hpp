#ifndef SERIAL
#define SERIAL

#include "graph.hpp"
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include "stdio.h"
#include <chrono>

int serial_astar(AStarMap &map){
    auto start_time = std::chrono::steady_clock::now();
    std::priority_queue<Node, std::vector<Node>, NodeCompare> open_queue;
    std::unordered_set<Node, NodeHash, NodeEqual> closed_set;
    std::unordered_map<Node, Node, NodeHash, NodeEqual> node_to_parent;

    open_queue.push(Node(map.startX, map.startY));
    std::cout << map.startX << " " << map.startY << std::endl;
    std::cout << map.endX << " " << map.endY << std::endl;
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
 
        if(cur == Node(map.endX, map.endY)){
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
            n.cost_to_come = cur.cost_to_come + map.get_edge_weight(cur, n);
            n.heuristic_cost = n.cost_to_come + n.heuristic(Node(map.endX, map.endY));
            Node new_parent = Node(cur.x, cur.y);
            new_parent.cost_to_come = cur.cost_to_come;
            new_parent.heuristic_cost = cur.heuristic_cost;
            node_to_parent[n] = new_parent;
            open_queue.push(n);
            map.open_node(n.x, n.y);
        }

        // map.render();
    }

    if(path_found){
        Node cur(end_node->x, end_node->y);

        while (node_to_parent.find(cur) != node_to_parent.end()) {
            Node parent = node_to_parent[cur];
            map.add_to_path(cur.x, cur.y);
            cur = parent;
        }
    }


    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    double seconds = diff.count();
    std::cout << "Time taken (s): " << seconds << std::endl;
    // map.render();
    return end_node->cost_to_come;
}

#endif