
#include "graph.hpp"
#include <vector>
#include <queue>
#include <unordered_set>
#include <iostream>
#include "stdio.h"
#include <upcxx>
using dist_queue = upcxx::dist_object<std::priority_queue<Node*, std::vector<Node*>, NodeCompare>>;  
//not quite the same reasoning as hashing
//(could use the same hashing methods)
//really looking for minimal queue
//this may be wasteful
int local_insert(dist_queue &lqueue, const Node* n, int num_passes, int size){
    if(num_passes>=upcxx::rank_n())lqueue.push(n); 
    if(lqueue.size()>=size){
        return upcxx::rpc((upcxx::rank_me()+1)%upcxx::rank_n(), local_insert(),lqueue, n, num_passes+1,lqueue.size()); 
    }
}
// trial impl, looks a lot like serial
int hpcxx_astar(int grid_size, std::vector<Obstacle> obstacleList, Point startPoint, Point endPoint){
     
    init(); 
    AStarMap map = AStarMap(grid_size, obstacleList,startPoint, endPoint);
    dist_queue local_queue std::priority_queue<Node*, std::vector<Node*>, NodeCompare>;
    
    //broadcast to all local pointers

    upcxx::global_ptr<std::unordered_set<Node*, NodeHash, NodeEqual>> closed_set 
    =upcxx::broadcast(upcxx::new_<std::unordered_set<Node*, NodeHash, NodeEqual>>(),0) ;

    if(upcxx::rank_me()==0)local_queue.push(map.start);

    std::cout << map.start->pos.x << " " << map.start->pos.y << std::endl;
    std::cout << map.goal->pos.x << " " << map.goal->pos.y << std::endl;
    bool path_found = upcxx::broadcast(false,0);
    Node* end_node;
    while(local_queue.size() > 0){
        Node* cur = local_queue.top();

        local_queue.pop();
        if(upcxx::rget(closed_set).find(cur) != upcxx::rget(closed_set).end()){
            continue;
        }
        upcxx::rput(closed_set, cur);
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
            //open_queue.push(n);
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

    while (!local_queue.empty()) {
        Node *n = local_queue.top();
        local_queue.pop();
        delete n;
    }

    map.render();
    return end_node->cost_to_come;
}
