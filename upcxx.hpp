
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
using dist_amap = upcxx::dist_object<AStarMap>; 

int get_proc(Node n){
	NodeHash nh; 
	return nh(n)%upcxx::rank_n();
}
int local_insert(dist_queue &lqueue, int x, int y, int num_passes, int size){
     (*lqueue).push(Node(x,y)); 
     return num_passes; 
}
int local_insert(dist_queue &lqueue, Node n, int num_passes, int size){
     (*lqueue).push(n); 
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
int local_insert(dist_queue &lqueue, Node n, int num_passes, int size){
     (*lqueue).push(n); 
     return num_passes; 
}
bool local_find(dist_set &closed, Node n){
	if(closed->find(n)!=closed->end())return true;
       	return false; 
} 
void local_emplace(dist_set &closed, Node n){
	//std::unordered_set<Node, NodeHash, NodeEqual>* closed = closed_set.local(); 
	closed->emplace(n);
}
Node* local_get(dist_map &parent, int x, int y){
  return (*parent.local())[Node(x,y)]; 
}
Node* local_get(dist_map &parent,Node n){
  return (*parent.local())[n]; 
}
void local_put(dist_map &parent, int x, int y, Node n){
  (*parent.local())[Node(x,y)]=n;
}
void local_put(dist_map &parent, Node n, Node par){
  (*parent.local())[n]=par;
}
bool local_find(dist_map &parent, Node n){
	if(parent->find(n)!=parent->end())return true;
       	return false; 
} 
void atomic_add(upcxx::global_ptr<int> addr, int x){
	*(addr.local())+=x; 
}

int startX(dist_amap& map){
	return map->startX; 
}
int startY(dist_amap& map){
	return map->startY; 
}
int endX(dist_amap& map){
	return map->endX; 
}
int endY(dist_amap& map){
	return map->endY; 
}

void close_node(dist_amap& map, int x, int y){
	(*map).close_node(x, y); 
}
void add_to_path(dist_amap& map, int x, int y){
	(*map).add_to_path(x, y); 
}
void open_node(dist_amap& map, int x, int y){
	(*map).open_node(x,y); 
}
void open_node(upcxx::global_ptr<AStarMap> amap, int x, int y){
	amap.local().open_node(x,y); 
}
void close_node(upcxx::global_ptr<AStarMap> amap, int x, int y){
	amap.local().close_node(x,y); 
} 
 
void upcxx_astar(int size, std::vector<Obstacle> obstacleList){
    
    upcxx::init();  
    auto start_time = std::chrono::steady_clock::now();
    dist_queue queue = std::priority_queue<Node, std::vector<Node>, NodeCompare>();
    std::priority_queue<Node, std::vector<Node>, NodeCompare>() local_queue = (*queue); 
    dist_set closed_set = std::unordered_set<Node, NodeHash, NodeEqual>();
    dist_map node_to_parent = std::unordered_map<Node, Node, NodeHash, NodeEqual>();  
    upcxx::global_ptr<bool>path_found = upcxx::broadcast(upcxx::new_<bool>(false),0).wait();
    upcxx::global_ptr<int> count = upcxx::broadcast(upcxx::new_<int>(0),0).wait();  
    upcxx::global_ptr<AStarMap> amap = upcxx::broadcast(upcxx::new_ <AStarMap>(AStarMap(size, obstacleList)), 0).wait(); 
    AStarMap map = upcxx::rget(amap).wait();
    //reinstantiate grid
    map = AStarMap(size, obstacleList,map.startX, map.startY, map.endX, map.endY);

    if(upcxx::rank_me()==0)local_queue.push(Node(map.startX, map.startY));
    std::cout << map.startX << " " << map.startY << std::endl;
    std::cout << map.endX << " " << map.endY << std::endl;
    bool path_found = false;
    Node* end_node;
    upcxx::global_ptr<Node> end_node = upcxx::broadcast(upcxx::new_ <Node>(), 0).wait(); 
		bool init = true; 	
		while(upcxx::rget(count).wait()>0||init ){
		init = false; 
		upcxx::barrier(); 
		std::cout<<"proc  "<<upcxx::rank_me<<"  waiting "<<std::endl; 
		upcxx::barrier(); 
		while(local_queue.size() > 0){ 
        Node cur = local_queue.top();

        local_queue.pop();
        if(upcxx::rpc(get_proc(n), local_find,closed_set,cur).wait()){ 
            continue;
        }
        upcxx::rpc(get_proc(n), local_emplace, closed_set,cur).wait(); 
				//this should be replaced with a list per iteration and popped.
				upcxx::rpc(0, close_node, amap,cur.x, cur.y).wait();
        map.close_node(cur.x, cur.y);
        if(cur == Node(map.endX, map.endY)){
            upcxx::rput(path_found, true).wait(); 
            upcxx::rput(end_node, &cur).wait();  
            break;
        }

        std::vector<std::vector<int>> dirn = cur.get_neighbor_directions();
        for(std::vector<int> d : dirn){
            Node n = Node(cur.x + d[0], cur.y + d[1]);
            if(upcxx::rpc(get_proc(n), local_find,closed_set,cur).wait()||!map.is_valid_node(n)){ 
                continue;
            }
            n.cost_to_come = cur.cost_to_come + map.get_edge_weight(cur, n);
            n.heuristic_cost = n.cost_to_come + n.heuristic(Node(map.endX, map.endY));
            Node new_parent = Node(cur.x, cur.y);
            new_parent.cost_to_come = cur.cost_to_come;
            new_parent.heuristic_cost = cur.heuristic_cost; 
            upcxx::rpc(get_proc(n), local_put,node_to_parent,n, new_parent).wait();
            upcxx::rpc(get_proc(n), local_insert,local_queue,n, new_parent).wait(); 
            map.open_node(n.x, n.y);
						upcxx::rpc(0, open_node, amap,cur.x, cur.y).wait();
        }
				std::cout<<"proc "<<upcxx::rank_me()<<std::endl; 
        map.render();
    }
		//this might be 1) time-consuming and 2)messy
				upcxx::barrier(); 
				count = upcxx::broadcast(upcxx::new_<int>(0),0).wait();
				upcxx::barrier(); 
				if(local_queue.size()>0)rput(count, upcxx::rank_me()+1).wait(); 
				std::cout<<"proc  "<<upcxx::rank_me<<"  waiting end "<<std::endl; 
				upcxx::barrier(); 
				if(upcxx::rget(count).wait()<1)break; 
		}

    if(upcxx::rank_me()==0&&path_found){
        Node cur(end_node->x, end_node->y);
        while(upcxx::rpc(get_proc(n), local_find,node_to_parent,cur).wait()){
        //while (node_to_parent.find(cur) != node_to_parent.end()) {
            Node parent = upcxx::rpc(get_proc(n), local_get,node_to_parent,cur).wait(); //node_to_parent[cur];
            map.add_to_path(cur.x, cur.y);
            cur = parent;
        }
    }

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    double seconds = diff.count();
    std::cout << "Time taken (s): " << seconds << std::endl;
		if(upcxx::rank_me()==0)std::cout<<"cost  "<<end_node.cost_to_come<<std::endl; 
    if(upcxx::rank_me()==0)map.render();
    upcxx::finalize(); 
    return end_node.cost_to_come;
}
