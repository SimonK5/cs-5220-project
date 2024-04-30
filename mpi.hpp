#ifndef FINAL_MPI
#define FINAL_MPI

#include <mpi.h>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <limits>
#include <chrono>
#include "graph.hpp"

std::priority_queue<Node, std::vector<Node>, NodeCompare> open_queue;
std::unordered_set<Node, NodeHash, NodeEqual> closed_set;

std::unordered_map<Node, Node, NodeHash, NodeEqual> node_to_parent;
std::unordered_map<Node, int, NodeHash, NodeEqual> costs_to_come;
std::unordered_map<int, std::vector<Node>> to_send;
MPI_Datatype MPI_Vertex;

int my_rank;
int total_num_procs;
bool terminate = false;
bool pathFound = false;
std::vector<Node> path;
// The optimal path found by any processor so far.
float minLengthPath = -1;

void init(AStarMap map, int rank, int num_procs){
    int blocklengths[6] = {1, 1, 1, 1, 1, 1};
    MPI_Datatype types[6] = {MPI_INT, MPI_INT, MPI_INT, MPI_INT,
                             MPI_FLOAT,   MPI_FLOAT};
    MPI_Aint offsets[6];
    offsets[0] = offsetof(Node, x);
    offsets[1] = offsetof(Node, y);
    offsets[2] = offsetof(Node, parentX);
    offsets[3] = offsetof(Node, parentY);
    offsets[4] = offsetof(Node, cost_to_come);
    offsets[5] = offsetof(Node, heuristic_cost);
    MPI_Type_create_struct(6, blocklengths, offsets, types, &MPI_Vertex);
    MPI_Type_commit(&MPI_Vertex);

    my_rank = rank;
    total_num_procs = num_procs;
    int init_processor = map.get_proc(Node(map.startX, map.startY), num_procs);
    if(rank == init_processor){
        printf("%d starting %d, %d\n", rank, map.startX, map.startY);
        open_queue.push(Node(map.startX, map.startY));
    }
}

/**
 * Sends a list of nodes to a given processor.
*/
void send_msg(int other, const std::vector<Node>& nodes, MPI_Request &request){
    MPI_Isend(&nodes[0], nodes.size(), MPI_Vertex, other, 0, MPI_COMM_WORLD, &request);
}

void send_msg(int other, const std::vector<Node>& nodes){
    MPI_Request request;
    send_msg(other, nodes, request);
}

/**
 * Sends all of the queued nodes to send to their corresponding owner processors.
*/
void send_nodes(){
    int size = 0;

    MPI_Request* requests = new MPI_Request[to_send.size()];
    MPI_Status* statuses = new MPI_Status[to_send.size()];

    int i = 0;
    for (const auto &pair : to_send) {
        int proc = pair.first;
        const std::vector<Node> &node_list = pair.second;
        send_msg(proc, node_list, requests[i]);
        i += 1;
    }

    MPI_Waitall(to_send.size(), requests, statuses);
    delete[] requests;
    delete[] statuses;
}

/**
 * Returns true if a message has been received.
 * 
 * Side effects: New nodes are pushed to the open queue.
*/
bool receiveMessages(int rank, int num_procs, AStarMap &map){
    bool receivedMessage = false;

    MPI_Status status;
    int msg_waiting;
    MPI_Iprobe(MPI_ANY_SOURCE, 0, MPI_COMM_WORLD, &msg_waiting, &status);
    while(msg_waiting){
        receivedMessage = true;
        int size;
        MPI_Get_count(&status, MPI_Vertex, &size);
        std::vector<Node> nodes(size);
        MPI_Request request;
        MPI_Irecv(&nodes[0], size, MPI_Vertex, MPI_ANY_SOURCE, 0, MPI_COMM_WORLD, &request);
        for(Node n : nodes){
            if(closed_set.find(n) == closed_set.end()){
                costs_to_come[n] = n.cost_to_come;
                // printf("%d pushing %d, %d, %f, %f\n", rank, n.x, n.y, n.cost_to_come, n.heuristic_cost);
                open_queue.push(n);
            }
        }

        MPI_Iprobe(MPI_ANY_SOURCE, 0, MPI_COMM_WORLD, &msg_waiting, &status);
    }

    return receivedMessage;
}


/**
 * Performs one step of the A* algorithm.
*/
void step(AStarMap &map, int rank, int num_procs){
    bool receivedMessages = receiveMessages(rank, num_procs, map);

    int end_proc = map.get_proc(Node(map.endX, map.endY), num_procs);
    if(rank != end_proc){
        int msg_waiting;
        MPI_Status status;
        MPI_Iprobe(end_proc, 1, MPI_COMM_WORLD, &msg_waiting, &status);
        if(msg_waiting){
            int size;
            MPI_Get_count(&status, MPI_FLOAT, &size);
            if(size == 1){
                float receivedMinPath;
                MPI_Request request;
                MPI_Irecv(&receivedMinPath, 1, MPI_FLOAT, end_proc, 1, MPI_COMM_WORLD, &request);
                if(receivedMinPath < minLengthPath || minLengthPath == -1){
                    minLengthPath = receivedMinPath;
                    // std::cout << "min length path set to " << minLengthPath << std::endl;
                }
            }
        }
    }


    if(open_queue.size() == 0){
        MPI_Barrier(MPI_COMM_WORLD);
        receivedMessages = receiveMessages(rank, num_procs, map);
        // std::cout << rank << " received: " << receivedMessages << std::endl;

        bool anyReceivedMessages;
        MPI_Allreduce(&receivedMessages, &anyReceivedMessages, 1, MPI_C_BOOL, MPI_LOR, MPI_COMM_WORLD);
        // std::cout << rank << " any received: " << anyReceivedMessages << std::endl;

        if(!anyReceivedMessages){
            terminate = true;
            return;
        }
    }

    // TODO: If message queue is empty, select highest priority state from open set and expand it
    if(open_queue.size() > 0 && !receivedMessages){
        Node n = open_queue.top();
        // printf("n: %d, %d, %f, %f\n", n.x, n.y, n.cost_to_come, n.heuristic_cost);
        open_queue.pop();
        closed_set.emplace(Node(n));
        map.close_node(n.x, n.y);
        node_to_parent[n] = Node(n.parentX, n.parentY);

        if(n == Node(map.endX, map.endY)){
            printf("found end!\n");

            if(n.cost_to_come < minLengthPath || minLengthPath < 0){
                minLengthPath = n.cost_to_come;

                for(int i = 0; i < num_procs; i++){
                    if(i == rank) continue;

                    MPI_Request request;
                    MPI_Isend(&minLengthPath, 1, MPI_FLOAT, i, 1, MPI_COMM_WORLD, &request);
                }
            }
        }

        if(n.heuristic_cost > minLengthPath && minLengthPath != -1) return;

        to_send.clear();
        for(auto d : n.get_neighbor_directions()){
            Node neighbor(n.x + d[0], n.y + d[1], n.x, n.y);
            if(!map.is_valid_node(neighbor) || closed_set.find(neighbor) != closed_set.end()){
                    continue;
            }
            if(costs_to_come.find(neighbor) != costs_to_come.end() 
                && n.cost_to_come + map.get_edge_weight(n, neighbor) >= costs_to_come[neighbor]){
                    continue;
            }
            // map.open_node(neighbor.x, neighbor.y);
            neighbor.cost_to_come = n.cost_to_come + map.get_edge_weight(n, neighbor);
            neighbor.heuristic_cost = neighbor.cost_to_come + neighbor.heuristic(Node(map.endX, map.endY));
            neighbor.parentX = n.x;
            neighbor.parentY = n.y;

            costs_to_come[neighbor] = neighbor.cost_to_come;
            node_to_parent[neighbor] = Node(n.x, n.y);
            int other_proc_num = map.get_proc(neighbor, num_procs);
            auto to_send_list = to_send.find(other_proc_num);
            if(to_send_list == to_send.end()){
                std::vector<Node> new_list;
                new_list.push_back(neighbor);
                to_send[other_proc_num] = new_list;
                // printf("%d sending %d, %d\n", rank, neighbor.x, neighbor.y);
            }
            else{
                to_send[other_proc_num].push_back(neighbor);
                // printf("%d sending %d, %d\n", rank, neighbor.x, neighbor.y);
            }
        }

        send_nodes();
    }
}

/**
 * Determines whether another rank has sent this rank a partial path.
 * 
 * If a path is received, it sends the list to the rank corresponding to the
 * next node in the path.
*/
void backtrack(AStarMap map, int rank, int num_procs){

    bool indivPathFound = false;

    MPI_Status status;
    int incoming_msg = 0;
    MPI_Iprobe(MPI_ANY_SOURCE, 0, MPI_COMM_WORLD,
            &incoming_msg, &status);
    
    if(incoming_msg){
        int size;
        MPI_Get_count(&status, MPI_Vertex, &size);
        path = std::vector<Node>(size);

        MPI_Request request;
        MPI_Irecv(&path[0], size, MPI_Vertex, MPI_ANY_SOURCE, 0, MPI_COMM_WORLD, &request);
    }

    if(path.size() > 0){
        Node cur_node = path[path.size() - 1];
        if(cur_node == Node(map.startX, map.startY)){
            indivPathFound = true;
        }
        else{
            Node next_node = node_to_parent[cur_node];
            path.push_back(next_node);
            int next_proc = map.get_proc(next_node, num_procs);
            send_msg(next_proc, path);
        }
    }

    MPI_Allreduce(&indivPathFound, &pathFound, 1, MPI_C_BOOL, MPI_LOR, MPI_COMM_WORLD);
    if(!pathFound) path.clear();
}

void mpi_astar(int argc, char** argv){
    
    int num_procs, rank;
    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &num_procs);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

    int map_size = 500;

    std::vector<Obstacle> obstacleList = {Obstacle(0, 0, 3, 5)};
    // std::cout << "make map" << std::endl;
    AStarMap map = AStarMap(map_size, obstacleList, 20, 20, 400, 400);
    int params[4];
    if(rank == 0){
        // map.render();
        // map.print_assignments(num_procs);
        params[0] = map.startX;
        params[1] = map.startY;
        params[2] = map.endX;
        params[3] = map.endY;
    }

    // std::cout << "get/set params" << std::endl;
    MPI_Bcast(params, 4, MPI_INT, 0, MPI_COMM_WORLD);
    if(rank != 0){
        map = AStarMap(map_size, obstacleList, params[0], params[1], params[2], params[3]);
    }

    // if(rank == 0) std::cout << "init" << std::endl;
    init(map, rank, num_procs);

    auto start_time = std::chrono::steady_clock::now();
    if(rank == 0) std::cout << "start" << std::endl;
    while(!terminate){
        step(map, rank, num_procs);
    }

    // get rank of final processor
    int final_rank = map.get_proc(Node(map.endX, map.endY), num_procs);

    if(rank == final_rank){
        std::cout << "final rank is " << final_rank << std::endl;
        // map.print_assignments(num_procs);
        path.push_back(Node(map.endX, map.endY));
        if(node_to_parent.find(Node(map.endX, map.endY)) == node_to_parent.end()){
            std::cout << "final node has no parent" << std::endl;
        }
        std::cout << "looking for path" << std::endl;
        // map.render();
    }
    while(!pathFound){
        backtrack(map, rank, num_procs);
    }
    int initial_rank = map.get_proc(Node(map.startX, map.startY), num_procs);

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    double seconds = diff.count();
    if(rank == 0){
        std::cout << "Path is of length " << path.size() << std::endl;
        std::cout << "Time taken (s): " << seconds << std::endl;
    } 
    // if(rank == initial_rank){
    for(Node n : path){
        map.add_to_path(n.x, n.y);
    }
    // map.render();
    // std::cout << std::endl;
    // }
}

#endif