#ifndef FINAL_MPI
#define FINAL_MPI

#include <mpi.h>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
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
void send_msg(int other, const std::vector<Node>& nodes){
    MPI_Request request;
    MPI_Isend(&nodes[0], nodes.size(), MPI_Vertex, other, 0, MPI_COMM_WORLD, &request);
}

/**
 * Sends all of the queued nodes to send to their corresponding owner processors.
*/
void send_nodes(){
    for (const auto &pair : to_send) {
        int proc = pair.first;
        const std::vector<Node> &node_list = pair.second;
        if(node_list.size() > 0){
            // printf("sending list of size %d to %d\n", node_list.size(), proc);
            send_msg(proc, node_list);
        }
    }
}

/**
 * Obtains and returns a list of nodes sent from another rank to this rank.
*/
std::vector<Node> receive_nodes(int rank, int other){
    MPI_Status status;
    int flag;
    MPI_Iprobe(other, 0, MPI_COMM_WORLD, &flag, &status);
    int size;
    MPI_Get_count(&status, MPI_Vertex, &size);
    // printf("%d probing, got size %d with flag %d\n", rank, size, flag);
    
    std::vector<Node> buffer(0);
    if(!flag){
        return buffer;
    }
    buffer = std::vector<Node>(size);
    
    // printf("%d receiving\n", rank);
    MPI_Request request;
    MPI_Irecv(&buffer[0], size, MPI_Vertex, other, 0, MPI_COMM_WORLD, &request);
    for(Node n : buffer){
        // printf("%d received %d, %d from %d\n", rank, n.x, n.y, other);
    }
    return buffer;
}

/**
 * Returns true if a message has been received.
 * 
 * Side effects: New nodes are pushed to the open queue.
*/
bool receiveMessages(int rank, int num_procs){
    bool receivedMessage = false;
    for(int i = 0; i < num_procs; i++){
        std::vector<Node> nodes = receive_nodes(rank, i);
        if(nodes.size() > 0) receivedMessage = true;
        for(Node n : nodes){
            if(closed_set.find(n) == closed_set.end()){
                    costs_to_come[n] = n.cost_to_come;
                    // printf("%d pushing %d, %d, %f, %f\n", rank, n.x, n.y, n.cost_to_come, n.heuristic_cost);
                    open_queue.push(n);
            }
        }
    }

    return receivedMessage;
}


/**
 * Performs one step of the A* algorithm.
*/
void step(AStarMap &map, int rank, int num_procs){
    to_send.clear();

    // TODO: check if new states have been received in the message queue
    bool receivedMessages = receiveMessages(rank, num_procs);

    if(open_queue.size() == 0){
        // std::cout << rank << " reached barrier" << std::endl;
        MPI_Barrier(MPI_COMM_WORLD);
        receivedMessages = receiveMessages(rank, num_procs);
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
        }

        for(auto d : n.get_neighbor_directions()){
            Node neighbor(n.x + d[0], n.y + d[1], n.x, n.y);
            if(!map.is_valid_node(neighbor) || closed_set.find(neighbor) != closed_set.end()){
                    continue;
            }
            if(costs_to_come.find(neighbor) != costs_to_come.end() 
                && n.cost_to_come + 1 >= costs_to_come[neighbor]){
                    continue;
            }
            map.open_node(neighbor.x, neighbor.y);
            neighbor.cost_to_come = n.cost_to_come + 1;
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

    // TODO: Convergence check. Add barrier if size of openSet is 0. If no procs have new messages, terminate
}

/**
 * Determines whether another rank has sent this rank a partial path.
 * 
 * If a path is received, it sends the list to the rank corresponding to the
 * next node in the path.
*/
void backtrack(AStarMap map, int rank, int num_procs){
    for(int i = 0; i < num_procs; i++){
        std::vector<Node> nodes = receive_nodes(rank, i);
        if(nodes.size() > 0){
            path = nodes;
            break;
        }
    }

    bool indivPathFound = false;
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
    if(pathFound) return;

    path = std::vector<Node>();
}

void mpi_astar(int argc, char** argv){
    
    int num_procs, rank;
    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &num_procs);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

    std::vector<Obstacle> obstacleList = {Obstacle(0, 0, 3, 5)};
    AStarMap map = AStarMap(10, obstacleList);
    int params[4];
    if(rank == 0){
        // map.render()
        // map.print_assignments(num_procs);
        params[0] = map.startX;
        params[1] = map.startY;
        params[2] = map.endX;
        params[3] = map.endY;
    }

    MPI_Bcast(params, 4, MPI_INT, 0, MPI_COMM_WORLD);
    if(rank != 0){
        map = AStarMap(10, obstacleList, params[0], params[1], params[2], params[3]);
    }

    init(map, rank, num_procs);
    std::cout << "done initializing" << std::endl;
    
    int nsteps = 30;
    while(!terminate){
        // printf("\n%d step %d\n", rank, s);
        step(map, rank, num_procs);
    }

    // get rank of final processor
    int final_rank = map.get_proc(Node(map.endX, map.endY), num_procs);

    if(rank == final_rank){
        std::cout << "final rank is " << final_rank << std::endl;
        map.print_assignments(num_procs);
        path.push_back(Node(map.endX, map.endY));
    }
    while(!pathFound){
        backtrack(map, rank, num_procs);
    }
    int initial_rank = map.get_proc(Node(map.startX, map.startY), num_procs);
    if(rank == initial_rank){
        for(Node n : path){
            map.add_to_path(n.x, n.y);
        }
        map.render();

    }
}

#endif