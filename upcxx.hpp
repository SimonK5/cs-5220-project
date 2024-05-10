
#include "graph.hpp"
#include <vector>
#include <queue>
#include <unordered_set>
#include <iostream>
#include "stdio.h"
#include <upcxx/upcxx.hpp>

using dist_queue = upcxx::dist_object<std::priority_queue<Node, std::vector<Node>, NodeCompare>>;  
using dist_set = upcxx::dist_object<std::unordered_set<Node, NodeHash, NodeEqual>>;  
using dist_map = upcxx::dist_object<std::unordered_map<Node, Node, NodeHash, NodeEqual>>;
int local_insert(dist_queue &lqueue, int x, int y, int num_passes, int size){
     (*lqueue).push(Node(x,y)); 
     return num_passes; 
}
bool local_find(dist_set &closed,int x, int y){
	if(closed->find(Node(x,y))!=closed->end())return true;
       	return false; 
} 
void local_emplace(dist_set &closed, int x, int y){
	//std::unordered_set<Node, NodeHash, NodeEqual>* closed = closed_set.local(); 
	closed->emplace(Node(x, y));
}
// trial impl, looks a lot like serial
int upcxx_astar(int grid_size, std::vector<Obstacle> obstacleList){//, Point startPoint, Point endPoint){
    upcxx::init(); 
    AStarMap map = AStarMap(grid_size, obstacleList);
    dist_queue local_queue;
    
    //broadcast to all local pointers

    dist_set closed_set;
    dist_map node_to_parent;

    if(upcxx::rank_me()==get_proc(Node(map.startX, map.startY), upcxx::rank_n()))(*local_queue).push(Node(map.startX, map.startY));

    std::cout << map.startX << " " << map.startY << std::endl;
    std::cout << map.endX << " " << map.endY << std::endl;
    upcxx::global_ptr<bool>path_found = upcxx::broadcast(upcxx::new_<bool>(false),0).wait();
    Node end_node;
    while((*local_queue).size() > 0){
        Node cur = (*local_queue).top();

        (*local_queue).pop();
       if(upcxx::rpc(0, local_find,closed_set,cur.x, cur.y).wait()){
            continue;
        } 
       upcxx::rpc(0, local_emplace, closed_set,cur.x, cur.y).wait(); 
        map.close_node(cur.x, cur.y);

        if(cur ==Node(map.endX, map.endY)){
            upcxx::rput(true,path_found).wait();
            end_node = cur;
            break;
        }

        std::vector<std::vector<int>> dirn = cur.get_neighbor_directions();
        for(std::vector<int> d : dirn){
            Node n = Node(cur.x + d[0], cur.y + d[1]);
            if(upcxx::rpc(get_proc(n, upcxx::rank_n()), local_find,closed_set,cur.x, cur.y).wait()|| !map.is_valid_node(n)){
                continue;
            }
            n.cost_to_come = cur.cost_to_come + 1;
            n.heuristic_cost = n.cost_to_come + n.heuristic(Node(map.endX, map.endY));
            Node new_parent = Node(cur.x, cur.y);
            new_parent.cost_to_come = cur.cost_to_come;
            new_parent.heuristic_cost = cur.heuristic_cost;
            node_to_parent[n] = new_parent;
	       upcxx::rpc(get_proc(n, upcxx::rank_n()), local_insert,local_queue, n.x, n.y, 0,(*local_queue).size()).wait(); 
	        map.open_node(n.x, n.y);
        }
    }

    if(upcxx::rget(path_found).wait()){
        Node cur = end_node;
        while(node_to_parent.find(cur) != node_to_parent.end()){
            Node parent = node_to_parent[cur];
            map.add_to_path(cur.x, cur.y);
            cur = parent;
        }
    } 

    map.render();
    upcxx::finalize(); 
    return end_node.cost_to_come;
}
