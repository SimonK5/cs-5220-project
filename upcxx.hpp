
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
int local_insert(dist_queue &lqueue, int x, int y, int parentX, int parentY,   float cost_to_come, float heuristic_cost, int  num_passes, int size){
	Node n = Node(x, y); 
	n.parentX = parentX; 
	n.parentY = parentY; 
	n.cost_to_come = cost_to_come; 
	n.heuristic_cost = heuristic_cost; 
	std::cout<<"proc" <<upcxx::rank_me()<<"  queue size  "<<(*lqueue).size()<<std::endl;
	(*lqueue).push(Node(x,y)); 
	std::cout<<"proc" <<upcxx::rank_me()<<"  post size  "<<(*lqueue).size()<<std::endl;
	
     return num_passes; 
}
int add_parent(dist_map &lmap, int x, int y, int parentX, int parentY,   float cost_to_come, float heuristic_cost){
	Node n = Node(x, y); 
	Node parent = Node(parentX,parentY); 
	parent.cost_to_come = cost_to_come; 
	parent.heuristic_cost = heuristic_cost; 
      (*lmap)[n]=parent; 
     return 0; 
}
bool find_parent(dist_map &lmap, int x, int y){
	return ((*lmap).find(Node(x,y)) != (*lmap).end());
} 

int* parent_coord(dist_map &lmap, int x, int y){
		Node n= (*lmap)[Node(x, y)]; 
return new int[]{n.x, n.y, n.parentX, n.parentY};
}
float* parent_dist(dist_map  &lmap, int x, int y){
	Node n= (*lmap)[Node(x, y)]; 
	return new float[]{n.heuristic_cost, n.cost_to_come}; 
}

void parent_remove(dist_map &lmap, int x, int y){
	(*lmap).erase(Node(x, y)); 
}
bool local_find(dist_set &closed,int x, int y){
	if(closed->find(Node(x,y))!=closed->end())return true;
       	return false; 
} 
void local_emplace(dist_set &closed, int x, int y, int parentX, int parentY, float cost_to_come, float heuristic_cost){
	//std::unordered_set<Node, NodeHash, NodeEqual>* closed = closed_set.local(); 
	std::cout<<" emplace   "<<closed->size()<<std::endl; 
	Node n = Node(x, y); 
	n.parentX = parentX; 
	n.parentY = parentY; 
	n.cost_to_come = cost_to_come; 
	n.heuristic_cost = heuristic_cost; 
	closed->emplace(n);
	std::cout<<" emplace   "<<closed->size()<<std::endl; 
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

// trial impl, looks a lot like serial
void upcxx_astar(int grid_size, std::vector<Obstacle> obstacleList){//, Point startPoint, Point endPoint){
     upcxx::init(); 
     int startX = 1; 
     int startY=1; 
     int endX = 3; 
     int endY = 3; 
     dist_queue local_queue = dist_queue(std::priority_queue<Node, std::vector<Node>, NodeCompare>());
     dist_amap amap = dist_amap(AStarMap(grid_size, obstacleList,startX, startY, endX, endY)); 
     AStarMap map = *amap; 
     dist_set closed_set = dist_set(std::unordered_set<Node, NodeHash, NodeEqual>());
     dist_map node_to_parent = dist_map(std::unordered_map<Node, Node, NodeHash, NodeEqual>());
     (*local_queue).push(Node(map.startX, map.startY));
     std::cout <<"init   "<<(*local_queue).size()<<std::endl; 
     std::cout << map.startX << " " << map.startY << std::endl;
     std::cout << map.endX << " " << map.endY << std::endl;
     upcxx::global_ptr<bool>path_found = upcxx::broadcast(upcxx::new_<bool>(false),0).wait();
     Node end_node;
     upcxx::global_ptr<int> count = upcxx::broadcast(upcxx::new_<int>(0),0).wait();  
     int local = 0; 
     bool init = true; 
     std::cout << upcxx::rank_me() << " " << "loop"  << std::endl;
     upcxx::barrier(); 
    while((*local_queue).size() > 0||upcxx::rget(count).wait()>0||init){
	    upcxx::barrier();   

	std::cout << upcxx::rank_me() << " " << "in loop" << std::endl;
	init = false; //enter loop
	std::cout <<"proc  "<<upcxx::rank_me() << " outer loop   "<<(*local_queue).size()<<std::endl; 

	if((*local_queue).size()==0){local = 0; }
	else{
	local =1; 
//	upcxx::rpc(0,atomic_add, count, 1).wait(); 
//	std::cout<<upcxx::rank_me()<<" " <<"add"<<std::endl;
	init = false; 
        Node cur = (*local_queue).top();
 	std::cout << "proc" <<upcxx::rank_me() << " " << (*local_queue).size() << std::endl;
    

        (*local_queue).pop();
	std::cout <<"proc  "<<upcxx::rank_me() << "  pop   "<<(*local_queue).size()<<std::endl; 
	for(Node n:(*closed_set)){
		std::cout<<"   proc::"<< upcxx::rank_me()<< " Node closed    " <<n.x<<"  " <<n.y; 
	}
       if(upcxx::rpc(0, local_find,closed_set,cur.x, cur.y).wait()){
            continue;
        } 
       	upcxx::rpc(0, local_emplace, closed_set,cur.x, cur.y, cur.parentX, cur.parentY, cur.cost_to_come, cur.heuristic_cost).wait(); 

	std::cout<< "proc "<<upcxx::rank_me() <<"   emplace"<<std::endl; 
       map.close_node(cur.x, cur.y);
       upcxx::rpc(0, close_node, amap, cur.x, cur.y).wait();
       if(cur ==Node(map.endX, map.endY)){
	    std::cout<<"FOUND"<<map.endX<<","<<map.endY<<std::endl; 
            upcxx::rput(true,path_found).wait();
            end_node = cur;
            break;
        }
        std::vector<std::vector<int>> dirn = cur.get_neighbor_directions();

        for(std::vector<int> d : dirn){
        Node n = Node(cur.x + d[0], cur.y + d[1]);
       	std::cout<<"directions "<<d[0]<<","<<d[1]<<std::endl;
       	if(upcxx::rpc(0, local_find,closed_set,n.x, n.y).wait() || !map.is_valid_node(n)){
	//if( !map.is_valid_node(n)){
		continue;
        }
        n.cost_to_come = cur.cost_to_come + 1;
       	n.heuristic_cost = n.cost_to_come + n.heuristic(Node(map.endX, map.endY));
	std::cout<<"local insert"; 
	upcxx::rpc(0, local_insert,local_queue, n.x, n.y,n.parentX, n.parentY, n.cost_to_come, n.heuristic_cost, 0,(*local_queue).size()).wait(); 
 	upcxx::rpc(0, add_parent,node_to_parent, n.x, n.y,cur.x, cur.y, cur.cost_to_come, cur.heuristic_cost).wait(); 
       	map.open_node(n.x, n.y);
	std::cout<<"insert over"; 
       }
	std::cout<<"proc"<<upcxx::rank_me()<<"-4"<<std::endl; 
	}
	map.render(); 
	upcxx::barrier(); 
	upcxx::rpc(0, atomic_add, count, 0-local).wait(); 
	upcxx::barrier(); 
	if((*local_queue).size()>0)upcxx::rpc(0, atomic_add, count, 1).wait(); 
	upcxx::barrier(); 
	std::cout << "proc" <<upcxx::rank_me() << " -2 " <<upcxx::rget(count).wait() << std::endl;
     	std::cout << "proc" <<upcxx::rank_me() << " -1  " << upcxx::rget(count).wait() << std::endl;
          
}
  	std::cout << "proc" <<upcxx::rank_me() << " outloop  " << upcxx::rget(count).wait() << std::endl;
  
    if(upcxx::rget(path_found).wait()){
        Node cur = end_node;
	int x = 0; 
        while(upcxx::rpc(0, find_parent, node_to_parent, cur.x, cur.y).wait()){
		std::cout<< "iter "<< x<<"***";
	    if(cur.x==startX)break; 	
	    std::cout<<"curX"<<cur.x<<"curY"<<cur.y<<std::endl; 
	    int* coords = upcxx::rpc(0, parent_coord, node_to_parent,cur.x, cur.y).wait();
    	    float* dists =  upcxx::rpc(0, parent_dist, node_to_parent,cur.x, cur.y).wait();

	//	std::cout<<"AFTER SEGFAULT";
	    Node parent = Node(coords[0], coords[1]);
	    parent.parentX = coords[2]; 
	    parent.parentY = coords[3];
	    parent.heuristic_cost = dists[0]; 
	    parent.cost_to_come =dists[1]; 
            map.add_to_path(cur.x, cur.y);
       	    upcxx::rpc(0,add_to_path,amap, cur.x, cur.y).wait(); 
      	    upcxx::rpc(0,parent_remove, node_to_parent, cur.x, cur.y).wait();
	    cur = parent;
	   x++; 
	    
        }
    } 
   std::cout << "proc" <<upcxx::rank_me() << " end  " << upcxx::rget(count).wait() << std::endl;
 
    map.render();
    upcxx::finalize(); 
    printf("end  %d", end_node.cost_to_come); 
  //  return end_node.cost_to_come;
}
