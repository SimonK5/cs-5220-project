
#include "graph.hpp"
#include <vector>
#include <queue>
#include <unordered_set>
#include <iostream>
#include "stdio.h"
#include <upcxx/upcxx.hpp>

using dist_queue = upcxx::dist_object<std::priority_queue<Node*, std::vector<Node*>, NodeCompare>>;  
//not quite the same reasoning as hashing
//(could use the same hashing methods)
//really looking for minimal queue
//this may be wasteful
int local_insert(dist_queue &lqueue, Node* n, int num_passes, int size){
    if(num_passes>=upcxx::rank_n())(*lqueue).push(n); 
    if((*lqueue).size()>=size){
        return upcxx::rpc((upcxx::rank_me()+1)%upcxx::rank_n(), local_insert,lqueue, n, num_passes+1,(*lqueue).size()).wait(); 
    }
}
// trial impl, looks a lot like serial
int upcxx_astar(int grid_size, std::vector<Obstacle> obstacleList){//, Point startPoint, Point endPoint){
    upcxx::init(); 
    AStarMap map = AStarMap(grid_size, obstacleList);//,startPoint, endPoint);
    dist_queue local_queue;// =std::priority_queue<Node*, std::vector<Node*>, NodeCompare>;
    
    //broadcast to all local pointers

    upcxx::global_ptr<std::unordered_set<Node*, NodeHash, NodeEqual>> closed_set 
    =upcxx::broadcast(upcxx::new_<std::unordered_set<Node*, NodeHash, NodeEqual>>(),0).wait() ;
    std::unordered_map<Node, Node, NodeHash, NodeEqual> node_to_parent;

    if(upcxx::rank_me()==0)(*local_queue).push(new Node(map.startX, map.startY));

    std::cout << map.startX << " " << map.startY << std::endl;
    std::cout << map.endX << " " << map.endY << std::endl;
    upcxx::global_ptr<bool>path_found = upcxx::broadcast(upcxx::new_<bool>(false),0).wait();
    Node* end_node;
    while((*local_queue).size() > 0){
        Node* cur = (*local_queue).top();

        (*local_queue).pop();
        if(upcxx::rget(closed_set).wait().find(cur) != upcxx::rget(closed_set).wait().end()){
            continue;
        }
	std::unordered_set<Node*, NodeHash, NodeEqual> set = upcxx::rget(closed_set).wait();
	set.emplace(cur); 
	upcxx::rput(set, closed_set).wait(); 
        map.close_node(cur->x, cur->y);

        if(*cur ==Node(map.endX, map.endY)){
		    upcxx::rput(true,path_found).wait();
            end_node = cur;
            break;
        }

        std::vector<std::vector<int>> dirn = cur->get_neighbor_directions();
        for(std::vector<int> d : dirn){
            Node *n = new Node(cur->x + d[0], cur->y + d[1]);
            if(upcxx::rget(closed_set).wait().find(n) != upcxx::rget(closed_set).wait().end() || !map.is_valid_node(*n)){
                continue;
            }
            n->cost_to_come = cur->cost_to_come + 1;
            n->heuristic_cost = n->cost_to_come + n->heuristic(Node(map.endX, map.endY));
            Node new_parent = Node(cur->x, cur->y);
            new_parent.cost_to_come = cur->cost_to_come;
            new_parent.heuristic_cost = cur->heuristic_cost;
            node_to_parent[*n] = new_parent;
	        upcxx::rpc((upcxx::rank_me())%upcxx::rank_n(), local_insert,local_queue, n, 0,(*local_queue).size()).wait(); 
	        map.open_node(n->x, n->y);
        }
    }

    if(upcxx::rget(path_found).wait()){
        Node* cur = end_node;
        while(node_to_parent.find(*cur) != node_to_parent.end()){
            Node parent = node_to_parent[*cur];
            map.add_to_path(cur->x, cur->y);
            cur = &parent;
        }
    }

    for (auto it = upcxx::rget(closed_set).wait().begin(); it != upcxx::rget(closed_set).wait().end(); ++it) {
        delete *it;
    }
    /*std::unordered_set<Node*, NodeHash, NodeEqual> set = upcxx::rget(closed_set).wait();
    set.clear(); 
    upcxx::rput(set, closed_set).wait(); 
    while (!(*local_queue).empty()) {
        Node *n = (*local_queue).top();
        (*local_queue).pop();
        delete n;
    }*/

    map.render();
    upcxx::finalize(); 
    return end_node->cost_to_come;
}
