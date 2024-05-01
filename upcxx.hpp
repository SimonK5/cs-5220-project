
#include "graph.hpp"
#include <vector>
#include <queue>
#include <unordered_set>
#include <iostream>
#include "stdio.h"
#include <upcxx/upcxx.hpp>

using dist_queue = upcxx::dist_object<std::priority_queue<Node, std::vector<Node>, NodeCompare>>;  
using dist_set = upcxx::dist_object<std::unordered_set<Node, NodeHash, NodeEqual>>;  
//AStarMap map = AStarMap(0, {});
using dist_amap = upcxx::dist_object<AStarMap>;
 
int local_insert(dist_queue &lqueue, int x, int y, int num_passes, int size){
    if(num_passes<upcxx::rank_n()&&(*lqueue).size()>=size){      
        return upcxx::rpc((upcxx::rank_me()+1)%upcxx::rank_n(), local_insert, lqueue, x, y, num_passes+1,(*lqueue).size()).wait(); 
    }
     (*lqueue).push(Node(x,y)); 
     return num_passes; 
}
bool local_find(dist_set &closed,int x, int y){
	//std::unordered_set<Node, NodeHash, NodeEqual>* closed = closed_set.local(); 
	if(closed->find(Node(x,y))!=closed->end())return true;
       	return false; 
} 
void local_emplace(dist_set &closed, int x, int y){
	//std::unordered_set<Node, NodeHash, NodeEqual>* closed = closed_set.local(); 
	closed->emplace(Node(x, y));
}
int get_map_startX(dist_amap &map ){
	return map->startX;
}
int get_map_startY(dist_amap &map) {return map->startY;}
int get_map_endX(dist_amap &map) {return map->endX;}
int get_map_endY(dist_amap &map ){return map->endY;}

void open_node(dist_amap &map, int X, int Y){
	map->open_node(X,Y); 
}
void close_node(dist_amap &map, int X, int Y){
	map->close_node(X,Y); 
}
void add_to_path(dist_amap &map, int X, int Y){
	map->add_to_path(X,Y); 
}

void stop_working(upcxx::global_ptr<int> count){
	int c = *count.local(); 
	c = c-1; 
}

void start_working(upcxx::global_ptr<int> count){
	int c = *count.local(); 
	c = c+1; 
}
// trial impl, looks a lot like serial
int upcxx_astar(int grid_size, std::vector<Obstacle> obstacleList){//, Point startPoint, Point endPoint){
  //  upcxx::init(); 
//,startPoint, endPoint);

  upcxx::init(); 
 
  AStarMap map = AStarMap(grid_size, obstacleList);
dist_amap lmap(map); 
  if(upcxx::rank_me()>0) {
	map.startX = upcxx::rpc(0,get_map_startX, lmap).wait(); 
	map.startY =upcxx::rpc(0, get_map_startY,lmap).wait();
	map.endX = upcxx::rpc(0, get_map_endX, lmap).wait(); 
	map.endY = upcxx::rpc(0, get_map_endY, lmap).wait(); 
	map = AStarMap(grid_size,obstacleList, map.startX, map.startY,map.endX, map.endY); 
  }
   dist_queue local_queue({});// =std::priority_queue<Node*, std::vector<Node*>, NodeCompare>;
    
    //broadcast to all local pointers
    dist_set closed_set({});

    std::unordered_map<Node, Node, NodeHash, NodeEqual> node_to_parent;
    if(upcxx::rank_me()==0)(*local_queue).push(Node(map.startX, map.startY));

    std::cout << map.startX << " " << map.startY << std::endl;
    std::cout << map.endX << " " << map.endY << std::endl;
   
    upcxx::global_ptr<bool>path_found = upcxx::broadcast(upcxx::new_<bool>(false),0).wait();
    Node end_node;
    upcxx::global_ptr<int> num_working = upcxx::broadcast(upcxx::new_<int>(1),0).wait(); 
    bool tracked = false; 
    if(upcxx::rank_me()==0)tracked=true; 
    while(true){
	    if((*local_queue).size()>0)std::cout<<upcxx::rank_me()<<"  "<<upcxx::rget(num_working).wait()<<"   " << (*local_queue).size()<<std::endl; 
	    upcxx::barrier(); 
	    if(upcxx::rget(num_working).wait()==0&&(*local_queue).size()==0)break;
	   	    else if(!tracked&&(*local_queue).size()>0){upcxx::rpc(0, start_working, num_working).wait();  tracked = true; }
		    upcxx::barrier(); 
 while( (*local_queue).size()> 0){
        Node cur = (*local_queue).top();

        (*local_queue).pop();
       if(upcxx::rpc(0, local_find,closed_set,cur.x, cur.y).wait()){
            continue;
        } 
       upcxx::rpc(0, local_emplace, closed_set,cur.x, cur.y).wait(); 
        map.close_node(cur.x, cur.y);
	upcxx::rpc(0, close_node,lmap, cur.x, cur.y).wait(); 
        if(cur ==Node(map.endX, map.endY)){
            upcxx::rput(true,path_found).wait();
            end_node = cur;
            break;
        }

        std::vector<std::vector<int>> dirn = cur.get_neighbor_directions();
        for(std::vector<int> d : dirn){
            Node n = Node(cur.x + d[0], cur.y + d[1]);
            if(upcxx::rpc(0, local_find,closed_set,cur.x, cur.y).wait()|| !map.is_valid_node(n)){
                continue;
            }
            n.cost_to_come = cur.cost_to_come + 1;
            n.heuristic_cost = n.cost_to_come + n.heuristic(Node(map.endX, map.endY));
            Node new_parent = Node(cur.x, cur.y);
            new_parent.cost_to_come = cur.cost_to_come;
            new_parent.heuristic_cost = cur.heuristic_cost;
            node_to_parent[n] = new_parent;
	       upcxx::rpc((upcxx::rank_me())%upcxx::rank_n(), local_insert,local_queue, n.x, n.y, 0,(*local_queue).size()).wait(); 
	       upcxx::rpc(0, open_node,lmap, n.x, n.y).wait();
		     map.open_node(n.x, n.y);
        }
    }
      
      upcxx::rpc(0, stop_working, num_working).wait(); 
    	tracked =false; 
    } 
upcxx::barrier(); 
    if(upcxx::rget(path_found).wait()){
        Node cur = end_node;
        while(node_to_parent.find(cur) != node_to_parent.end()){
            Node parent = node_to_parent[cur];
	    upcxx::rpc(0,add_to_path,lmap, cur.x,cur.y).wait(); 
	    map.add_to_path(cur.x, cur.y);
            cur = parent;
        }
    } 
   
    map.render();
   // upcxx::barrier(); 
std::cout<<end_node.cost_to_come<<"    " <<upcxx::rank_me()<<std::endl; 
upcxx::finalize(); 
       return end_node.cost_to_come;
}
