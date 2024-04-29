#ifndef FINAL_MPI
#define FINAL_MPI

#include <mpi.h>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <limits>
#include "graph.hpp"
#include <chrono>

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
int step_num = 0;

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
        // printf("%d starting %d, %d\n", rank, map.startX, map.startY);
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
            // else{ // If the CTC of the received node is better than the original CTC, must replace in open set
            //     float other_ctc = costs_to_come[n];
            //     if(other_ctc > n.cost_to_come){
            //         closed_set.erase(n);
            //         costs_to_come[n] = n.cost_to_come;
            //         node_to_parent[n] = Node(n.parentX, n.parentY);
            //         open_queue.push(n);
            //     }
            // }
        }
    }

    return receivedMessage;
}


/**
 * Performs one step of the A* algorithm.
*/
void step(AStarMap &map, int rank, int num_procs){
    to_send.clear();

    // std::cout << "receiving msg\n";
    // TODO: check if new states have been received in the message queue
    bool receivedMessages = receiveMessages(rank, num_procs);

    
    
    if(open_queue.size() == 0){
        // std::cout << "queue size 0\n";

        MPI_Barrier(MPI_COMM_WORLD);
        
        // int end_proc = map.get_proc(Node(map.endX, map.endY), num_procs);
        // float pathBuffer;
        // if(rank == end_proc){
        //     pathBuffer = minLengthPath;
        // }
        // MPI_Request pathRequest;
        // MPI_Bcast(&pathBuffer, 1, MPI_FLOAT, end_proc, MPI_COMM_WORLD);
        // if(rank != end_proc && (pathBuffer < minLengthPath || minLengthPath == -1)){
        //     minLengthPath = pathBuffer;
        // }


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
        if(rank == 0) std::cout << "no msgs received\n";

        Node n = open_queue.top();
        if(rank == 0) std::cout << "n: " << n.x << " " << n.y << "\n";
        // printf("n: %d, %d, %f, %f\n", n.x, n.y, n.cost_to_come, n.heuristic_cost);
        open_queue.pop();
        closed_set.emplace(Node(n));
        map.close_node(n.x, n.y);
        node_to_parent[n] = Node(n.parentX, n.parentY);

        if(n == Node(map.endX, map.endY)){
            printf("found end!\n");
            printf("%d steps to converge\n", step_num);
            // terminate = true;
            if(n.cost_to_come < minLengthPath || minLengthPath < 0){
                minLengthPath = n.cost_to_come;
            }
            return;
        }

        if(n.heuristic_cost > minLengthPath && minLengthPath != -1) return;

        if(rank == 0) std::cout << "prcessing neighbor\n";
        for(auto d : n.get_neighbor_directions()){

            if(rank == 0) std::cout << "init neighbor\n";
            Node neighbor(n.x + d[0], n.y + d[1], n.x, n.y);
            if(rank == 0) std::cout << "checking valid\n";
            if(!map.is_valid_node(neighbor) || closed_set.find(neighbor) != closed_set.end()){
                    continue;
            }
            if(rank == 0) std::cout << "making/checking CTC\n";
            float new_CTC = n.cost_to_come + map.get_edge_weight(n, neighbor);
            if(costs_to_come.find(neighbor) != costs_to_come.end() 
                && new_CTC >= costs_to_come[neighbor]){
                    continue;
            }
            if(rank == 0) std::cout << "opening node\n";
            map.open_node(neighbor.x, neighbor.y);
            neighbor.cost_to_come = new_CTC;
            neighbor.heuristic_cost = neighbor.cost_to_come + neighbor.heuristic(Node(map.endX, map.endY));
            neighbor.parentX = n.x;
            neighbor.parentY = n.y;

            if(rank == 0) std::cout << "adding to maps\n";
            costs_to_come[neighbor] = neighbor.cost_to_come;
            node_to_parent[neighbor] = Node(n.x, n.y);
            int other_proc_num = map.get_proc(neighbor, num_procs);
            auto to_send_list = to_send.find(other_proc_num);
            if(to_send_list == to_send.end()){
                if(rank == 0) std::cout << "adding to to_send\n";
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
    step_num++;
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
        // std::cout << rank << " " << path.size() << std::endl;
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

    path.clear();
}

void mpi_astar(int argc, char** argv){
    
    int num_procs, rank;
    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &num_procs);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    auto start_time = std::chrono::steady_clock::now();


    int map_size = 1000;
    std::vector<Obstacle> obstacleList = {Obstacle(0, 0, 3, 5), Obstacle(12, 5, 16, 8), Obstacle(16, 17, 19, 19)};

    AStarMap map = AStarMap(map_size, obstacleList);
    if(rank == 0){
        std::cout << "creating map" << std::endl;
        map = AStarMap(map_size, obstacleList);
    }
    int params[4];
    if(rank == 0){
        std::cout << "sending map" << std::endl;
        // map.render();
        // map.print_assignments(num_procs);
        // printf("%d, %d, %d, %d", map.startX, map.startY, map.endX, map.endY);
        params[0] = map.startX;
        params[1] = map.startY;
        params[2] = map.endX;
        params[3] = map.endY;
    }

    MPI_Bcast(params, 4, MPI_INT, 0, MPI_COMM_WORLD);
    if(rank != 0){
        map = AStarMap(map_size, obstacleList, params[0], params[1], params[2], params[3]);
    }
    if(rank == 0) std::cout << "init" << std::endl;
    init(map, rank, num_procs);
    if(rank == 0) std::cout << "start A*" << std::endl;
    int nsteps = 1000000;
    int s = 0;
    while(!terminate){
        // printf("\n%d step %d\n", rank, s);
        // std::cout << s << std::endl;
        step(map, rank, num_procs);
        s+=1;
    }
    // for(int i = 0; i < nsteps; i++){
    //     step(map, rank, num_procs);
    //     if(terminate) break;
    // }
    // map.render();
    if(!terminate && rank == 0) printf("Not found :(\n");
    // printf("%d total steps\n", step_num);
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    double seconds = diff.count();
    if(rank == 0) std::cout << "Time taken (s): " << seconds << std::endl;
    // for(int i = 0; i < nsteps; i++){
    //     // std::cout << rank << " i: " << i << std::endl;
    //     step(map, rank, num_procs);
    // }

    // get rank of final processor
    int final_rank = map.get_proc(Node(map.endX, map.endY), num_procs);

    if(rank == final_rank){
        // std::cout << "final rank is " << final_rank << std::endl;
        // map.print_assignments(num_procs);
        path.push_back(Node(map.endX, map.endY));
        if(node_to_parent.find(Node(map.endX, map.endY)) == node_to_parent.end()){
            std::cout << "final node has no parent" << std::endl;
        }
        // std::cout << "looking for path" << std::endl;
    }
    // while(!pathFound){
    //     backtrack(map, rank, num_procs);
    // }
    // int initial_rank = map.get_proc(Node(map.startX, map.startY), num_procs);
    // if(rank == initial_rank){
    //     for(Node n : path){
    //         map.add_to_path(n.x, n.y);
    //     }
    // }

    
}

#endif