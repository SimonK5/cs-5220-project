
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


// trial impl, looks a lot like serial
int upcxx_astar(int grid_size, std::vector<Obstacle> obstacleList){//, Point startPoint, Point endPoint){
    // AStarMap map =AStarMap(grid_size,obstacleList); 
     upcxx::init(); 
   // upcxx::broadcast(0, AStarMap(grid_size, obstacleList)).wait(); 
    dist_queue local_queue = std::priority_queue<Node, std::vector<Node>, NodeCompare>();
    //map.startX = upcxx::broadcast(
    //broadcast to all local pointers
  //  upcxx::global_ptr<AStarMap> gmap = upcxx::broadcast(upcxx::new_<AStarMap>(grid_size, obstacleList), 0).wait(); 
     dist_amap amap = dist_amap(AStarMap(grid_size, obstacleList)); 
     AStarMap map = *amap; 
     //if(upcxx::rank_me()==0)map = AStarMap(grid_size, obstacleList); 
     map.startX = upcxx::rpc(0, startX, amap).wait(); 
     map.startY = upcxx::rpc(0, startY, amap).wait(); 
     map.endX = upcxx::rpc(0, endX, amap).wait(); 
     map.endY = upcxx::rpc(0, endY, amap).wait(); 
     //upcxx::rget(gmap).wait(); 
    dist_set closed_set = dist_set({});
    dist_map node_to_parent = dist_map({});

    if(upcxx::rank_me()==map.get_proc(Node(map.startX, map.startY), upcxx::rank_n()))(*local_queue).push(Node(map.startX, map.startY));
    std::cout << map.startX << " " << map.startY << std::endl;
    std::cout << map.endX << " " << map.endY << std::endl;
 
     upcxx::global_ptr<bool>path_found = upcxx::broadcast(upcxx::new_<bool>(false),0).wait();
    Node end_node;
    upcxx::global_ptr<int> count = upcxx::broadcast(upcxx::new_<int>(0),0).wait();  
    bool init = true; 
     std::cout << upcxx::rank_me() << " " << "loop"  << std::endl;
     upcxx::barrier(); 
    while((*local_queue).size() > 0||upcxx::rget(count).wait()>0||init){
	    upcxx::barrier();   
	std::cout << upcxx::rank_me() << " " << "in loop" << std::endl;
	init = false; //enter loop

	if((*local_queue).size()>0){
	
	upcxx::rpc(0,atomic_add, count, 1).wait(); 
//	std::cout<<upcxx::rank_me()<<" " <<"add"<<std::endl;
	init = false; 
        Node cur = (*local_queue).top();
 	std::cout << "proc" <<upcxx::rank_me() << " " << (*local_queue).size() << std::endl;
    

        (*local_queue).pop();
	std::cout <<"proc  "<<upcxx::rank_me() << "  pop   "<<(*local_queue).size()<<std::endl; 
//	std::cout<< "proc  "<<upcxx::rank_me() << "  popA"<<std::endl; 
	closed_set;
	//std::cout<<"proc  "<<upcxx::rank_me() <<"  len  " <<closed_set.size()<<std::endl; 
//	std::cout<<" proc "<<upcxx::rank_me() << "pop End"<<std::endl;
       if(upcxx::rpc(0, local_find,closed_set,cur.x, cur.y).wait()){
            continue;
        } 
       upcxx::rpc(0, local_emplace, closed_set,cur.x, cur.y).wait(); 
        std::cout<< "proc "<<upcxx::rank_me() <<"   emplace"<<std::endl; 
       map.close_node(cur.x, cur.y);
 	// std::cout<< "proc "<<upcxx::rank_me() <<"   close   "<<std::endl; 
       if(cur ==Node(map.endX, map.endY)){
            upcxx::rput(true,path_found).wait();
            end_node = cur;
            break;
        }
	 //std::cout<< "proc "<<upcxx::rank_me() <<"   c   "<<std::endl; 
       
        std::vector<std::vector<int>> dirn = cur.get_neighbor_directions();

        for(std::vector<int> d : dirn){
     //      std::cout<< "proc "<<upcxx::rank_me() <<"   d   "<<std::endl; 
         Node n = Node(cur.x + d[0], cur.y + d[1]);
            if(upcxx::rpc(map.get_proc(n, upcxx::rank_n()), local_find,closed_set,cur.x, cur.y).wait()|| !map.is_valid_node(n)){
                continue;
            }
       //       std::cout<< "proc "<<upcxx::rank_me() <<"   e  "<<std::endl; 
         n.cost_to_come = cur.cost_to_come + 1;
  //             std::cout<< "proc "<<upcxx::rank_me() <<"   f  "<<std::endl; 
n.heuristic_cost = n.cost_to_come + n.heuristic(Node(map.endX, map.endY));
    //         std::cout<< "proc "<<upcxx::rank_me() <<"   g  "<<std::endl; 
Node new_parent = Node(cur.x, cur.y);
    // std::cout<< "proc "<<upcxx::rank_me() <<"   h  "<<std::endl; 
          new_parent.cost_to_come = cur.cost_to_come;
     // std::cout<< "proc "<<upcxx::rank_me() <<"   i  "<<std::endl; 
         new_parent.heuristic_cost = cur.heuristic_cost;
       //     std::cout<< "proc "<<upcxx::rank_me() <<"   j  "<<std::endl; 
   (*node_to_parent)[n] = new_parent;
      //  std::cout<< "proc "<<upcxx::rank_me() <<"  k   "<<std::endl; 
  
	       upcxx::rpc(map.get_proc(n, upcxx::rank_n()), local_insert,local_queue, n.x, n.y, 0,(*local_queue).size()).wait(); 
	 //     std::cout<< "proc "<<upcxx::rank_me() <<"   l  "<<std::endl; 
       map.open_node(n.x, n.y);
      //  std::cout<< "proc "<<upcxx::rank_me() <<"   m  "<<std::endl; 
  
        }
//	std::cout<<" proc"<<upcxx::rank_me()<< " -3 " <<std::endl; 
		upcxx::rpc(0, atomic_add, count, -1).wait(); 
		std::cout<<"proc"<<upcxx::rank_me()<<"-4"<<std::endl; 
}
upcxx::barrier(); 
	std::cout << "proc" <<upcxx::rank_me() << " -2 " <<upcxx::rget(count).wait() << std::endl;
    	std::cout << "proc" <<upcxx::rank_me() << " -1  " << upcxx::rget(count).wait() << std::endl;
    
}

    if(upcxx::rget(path_found).wait()){
        Node cur = end_node;
        while((*node_to_parent).find(cur) != (*node_to_parent).end()){
            Node parent = (*node_to_parent)[cur];
            map.add_to_path(cur.x, cur.y);
            cur = parent;
        }
    } 

    map.render();
    upcxx::finalize(); 
    return end_node.cost_to_come;
}
