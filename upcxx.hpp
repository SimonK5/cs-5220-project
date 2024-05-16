
#include "graph.hpp"
#include <vector>
#include <queue>
#include <unordered_set>
#include <iostream>
#include "stdio.h"
#include <upcxx/upcxx.hpp>

const int options = 1; 
const int optimal =10; 
using dist_queue = upcxx::dist_object<std::priority_queue<Node, std::vector<Node>, NodeCompare>>;  
using dist_set = upcxx::dist_object<std::unordered_set<Node, NodeHash, NodeEqual>>;  
using dist_map = upcxx::dist_object<std::unordered_map<Node, Node, NodeHash, NodeEqual>>; 
using dist_amap = upcxx::dist_object<AStarMap>; 

int get_proc(Node n){
	if(options>=0){
	NodeHash nh; 
	return nh(n)%upcxx::rank_n();
	}
	return 0; //do this to debug
} 
int local_open(dist_queue &lqueue, Node n, int num_passes, int size){
	if(options==1){
		if(lqueue->size()<=size||lqueue->size()<=optimal||num_passes>=upcxx::rank_n()/2) { 
			lqueue->push(n);  
			return num_passes;
		}
		else{
			return -1; 
		}
	}else{
     (*lqueue).push(n); 
     return num_passes; 
	}
}
int local_open_f(dist_queue &lqueue, Node n){
     (*lqueue).push(n); 
     return 1;  
}
int local_insert1(dist_queue &lqueue, Node n, int num_passes, int size){
     (*lqueue).push(n); 
     return num_passes; 
}
bool local_find1(dist_set &closed,int x, int y){
	if(closed->find(Node(x,y))!=closed->end())return true;
       	return false; 
} 
void local_emplace1(dist_set &closed, int x, int y){
	//std::unordered_set<Node, NodeHash, NodeEqual>* closed = closed_set.local(); 
	closed->emplace(Node(x, y));
} 

bool newer_cost(dist_set &closed, Node n){
	if(closed->find(n)==closed->end()){
		closed->emplace(n); 
		return true; 
		}
	if((*(*closed).find(n)).cost_to_come<0){
		closed->emplace(n); 
		return true;
	}
	if((*(*closed).find(n)).cost_to_come<=n.cost_to_come){
		closed->emplace(n); 
		return true; 
	}
  return false; 
} 
int local_cost(dist_set &closed, Node n){
	if(closed->find(n)!=closed->end()){
	 return(*(*closed).find(n)).cost_to_come; 
	}
	return -1; 
}
bool local_find(dist_set &closed, Node n){
	if(closed->find(n)!=closed->end())return true;
       	return false; 
} 
 bool local_emplace(dist_set &closed, Node n){
	//std::unordered_set<Node, NodeHash, NodeEqual>* closed = closed_set.local(); 
		if(closed->find(n)!=closed->end())return false;
		closed->emplace(n);
    return true;
	
}
Node  local_get1(dist_map &parent, int x, int y){
  return (*parent)[Node(x,y)]; 
}
Node  local_get(dist_map &parent,Node n){
  return (*parent)[n]; 
}
void local_put1(dist_map &parent, int x, int y,  Node n){
  (*parent)[Node(x,y)]=n;
}
void local_put(dist_map &parent, Node n, Node par){
  (*parent)[n]=par;
}
void local_put_safe(dist_map &parent,dist_set &closed, Node n, Node par){
	if(closed->find(n)!=closed->end())return; 
  (*parent)[n]=par;
}
bool local_find2(dist_map &parent, Node n){
	if(parent->find(n)!=parent->end())return true;
       	return false; 
} 
void atomic_add(upcxx::global_ptr<int> addr, int x){
	*(addr.local())+=x; 
}

void set_node(upcxx::global_ptr<Node> addr, Node n){
	(*addr.local())=n; 
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
void open_node(upcxx::global_ptr<AStarMap> amap, int x, int y){
	amap.local()->open_node(x,y); 
}
void close_node(upcxx::global_ptr<AStarMap> amap, int x, int y){
	amap.local()->close_node(x,y); 
} 

AStarMap copy_map(upcxx::global_ptr<AStarMap> amap){
	return *(amap.local()); 
}

int upcxx_astar(int size, std::vector<Obstacle> obstacleList){
    auto start_time = std::chrono::steady_clock::now();
    dist_queue local_queue({}); 
    dist_set closed_set({});
    dist_map node_to_parent({});  
    upcxx::global_ptr<bool>path_found = upcxx::broadcast(upcxx::new_<bool>(false),0).wait();
    upcxx::global_ptr<int> count = upcxx::broadcast(upcxx::new_<int>(0),0).wait();  
		upcxx::global_ptr<int> path_len = upcxx::broadcast(upcxx::new_<int>(-1),0).wait();  
    upcxx::global_ptr<AStarMap> amap = upcxx::broadcast(upcxx::new_ <AStarMap>(AStarMap(size, obstacleList)), 0).wait(); 
    AStarMap map = upcxx::rpc(0, copy_map, amap).wait();
    //reinstantiate grid
		map = AStarMap(size, obstacleList, map.startX, map.startY, map.endX, map.endY);
    if(upcxx::rank_me()==0)local_queue->push(Node(map.startX, map.startY));
    upcxx::global_ptr<Node> end_node = upcxx::broadcast(upcxx::new_ <Node>(), 0).wait(); 
		bool init = true; 	
		while(upcxx::rget(count).wait()>0||init){
		int moved = 0; 
		init = false;  
		upcxx::barrier();   
		while(local_queue->size() > 0){   
        Node cur = local_queue->top();
				moved = 100;
        local_queue->pop();  
				upcxx::rpc(0, close_node, amap,cur.x, cur.y).wait();
        map.close_node(cur.x, cur.y);
        if(!upcxx::rpc(get_proc(cur), newer_cost, closed_set,cur).wait()){ 
            continue;
        }  
				bool loptimal = local_queue->size()<=optimal;  
		  	upcxx::rpc(0, close_node, amap,cur.x, cur.y).wait();
        map.close_node(cur.x, cur.y);
				int minLengthPath = upcxx::rget(path_len).wait(); 
        if(cur == Node(map.endX, map.endY)){
            upcxx::rput(true, path_found).wait(); 						
						if(cur.cost_to_come < minLengthPath || minLengthPath < 0){
                minLengthPath = cur.cost_to_come;
								upcxx::rput(minLengthPath, path_len).wait(); 
								upcxx::rpc(0, set_node, end_node,cur).wait();  
						} 
        }
				if(cur.heuristic_cost > minLengthPath && minLengthPath >= 0 ) continue;

        std::vector<std::vector<int>> dirn = cur.get_neighbor_directions();
				int i = 0; 
        for(std::vector<int> d : dirn){ 

            Node n = Node(cur.x + d[0], cur.y + d[1]);
						map.open_node(n.x, n.y);
						moved=100;  
						upcxx::rpc(0, open_node, amap,n.x, n.y).wait();
						int cost = upcxx::rpc(get_proc(n), local_cost,closed_set,n).wait();
            if((upcxx::rpc(get_proc(n), local_find,closed_set,n).wait()||!map.is_valid_node(n)))continue; 
						if((!cost>0 && cur.cost_to_come + map.get_edge_weight(cur, n) >= upcxx::rpc(get_proc(n), local_cost,closed_set,n).wait()))
						{continue;}
                   
            n.cost_to_come = cur.cost_to_come + map.get_edge_weight(cur, n);
            n.heuristic_cost = n.cost_to_come + n.heuristic(Node(map.endX, map.endY));
            Node new_parent = Node(cur.x, cur.y);
            new_parent.cost_to_come = cur.cost_to_come;
            new_parent.heuristic_cost = cur.heuristic_cost;  
						if(!upcxx::rpc(get_proc(n), local_find2,node_to_parent,n).wait()){ 
							upcxx::rpc(get_proc(n), local_put_safe,node_to_parent,closed_set,n, Node(cur.x, cur.y)).wait();
						}else{ 
							Node old_par =upcxx::rpc(get_proc(n), local_get, node_to_parent,n).wait(); 
							if(old_par.cost_to_come  >=new_parent.cost_to_come )  { 
							upcxx::rpc(get_proc(n), local_put_safe,node_to_parent,closed_set,n, Node(cur.x, cur.y)).wait();

							} 
						} 
						upcxx::rpc((get_proc(n))%upcxx::rank_n(), local_open_f, local_queue,n).wait();
						//upcxx::rpc((get_proc(n))%upcxx::rank_n(), local_open_f, local_queue,n).wait();
						//upcxx::rpc((upcxx::rank_me()+(++i))%upcxx::rank_n(), local_open_f, local_queue,n).wait();
						//upcxx::rpc((upcxx::rank_me()+2)%upcxx::rank_n(), local_open_f, local_queue,n).wait();
						// upcxx::rpc((upcxx::rank_me()+d[0]*d[0]+(upcxx::rank_n()/2)+3*d[1]*d[1])%upcxx::rank_n(), local_open_f, local_queue,n).wait();
						 //upcxx::rpc((upcxx::rank_me()+2)%upcxx::rank_n(), local_open_f, local_queue,n).wait();
						 //}
					/*	 if(loptimal){ 
							 local_queue->push(n); 
						 }else{ 
							upcxx::rpc((get_proc(n))%upcxx::rank_n(), local_open_f, local_queue,n).wait();
						  }
						 	/*int placed = -1; 
							int num = 0; 
							while(placed<0){
							num++; 
						  placed = upcxx::rpc((upcxx::rank_me()+num)%upcxx::rank_n(), local_open, local_queue,n,num,local_queue->size()).wait(); 
							//placed = upcxx::rpc(get_proc(cur), local_open, local_queue,n,num,local_queue->size()).wait(); 
							
						} upcxx::rpc((upcxx::rank_me()+num)%upcxx::rank_n(), local_open_f, local_queue,cur).wait();*/
						/*upcxx::rpc((upcxx::rank_me()+1)%upcxx::rank_n(), local_open_f, local_queue,n).wait();
						}*/
            map.open_node(n.x, n.y);
						moved=100;  
						upcxx::rpc(0, open_node, amap,n.x, n.y).wait();
        }  
				
    }
		   //this might be 1) time-consuming and 2)messy 
				if(upcxx::rank_me()==0)upcxx::rput(0, count).wait();  
				if(moved>0){
					upcxx::rput(upcxx::rank_me()+1, count).wait(); 
				}  
				upcxx::barrier(); 
				upcxx::barrier_async().wait();  
		}
		upcxx::barrier(); 
    if(upcxx::rank_me()==0&&path_found){ 
        Node cur(end_node.local()->x, end_node.local()->y);
        while(upcxx::rpc(get_proc(cur), local_find2,node_to_parent,cur).wait()){ 
            Node parent = upcxx::rpc(get_proc(cur), local_get,node_to_parent,cur).wait(); 
            map.add_to_path(cur.x, cur.y);
            cur = parent;
        }
    }
		upcxx::barrier(); 
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    double seconds = diff.count();
    //std::cout << upcxx::rank_me()<<" Time taken (s): " << seconds << std::endl;
		if(upcxx::rank_me()==0)std::cout<<"cost  "<<end_node.local()->cost_to_come<<std::endl; 
    if(upcxx::rank_me()==0)map.render();
    return end_node.local()->cost_to_come;
}
