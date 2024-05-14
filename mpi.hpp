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

MPI_Datatype MPI_Control_Msg;

struct control_msg{
    int control_clock;
    int count;
    int invalid;
    int start;
};

int my_rank;
int total_num_procs;
bool terminate = false;
bool pathFound = false;
bool terminationCheckStarted = false;
std::vector<Node> path;
// The optimal path found by any processor so far.
float minLengthPath = -1;

// num messages sent - num messages received
int messageCount = 0;

// timestamp with respect to termination checks. Increments on a termination check
int p_clock = 0;

int tmax = 0;

#define BASIC_MESSAGE 0
#define PATH_LENGTH_MESSAGE 1
#define TERMINATION_MESSAGE 2
#define TIME_MESSAGE 3


void init(AStarMap map, int rank, int num_procs){
    // Ok, maybe having global state was a bad idea, but I don't feel like fixing it
    open_queue = std::priority_queue<Node, std::vector<Node>, NodeCompare>();
    closed_set = std::unordered_set<Node, NodeHash, NodeEqual>();
    node_to_parent = std::unordered_map<Node, Node, NodeHash, NodeEqual>();
    costs_to_come = std::unordered_map<Node, int, NodeHash, NodeEqual>();
    to_send = std::unordered_map<int, std::vector<Node>>();
    terminate = false;
    pathFound = false;
    terminationCheckStarted = false;
    path = std::vector<Node>();
    minLengthPath = -1;
    messageCount = 0;
    p_clock = 0;
    tmax = 0;

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

    int control_blocklengths[4] = {1, 1, 1, 1};
    MPI_Datatype control_types[4] = {MPI_INT, MPI_INT, MPI_INT, MPI_INT};

    MPI_Aint control_offsets[4];
    control_offsets[0] = offsetof(control_msg, control_clock);
    control_offsets[1] = offsetof(control_msg, count);
    control_offsets[2] = offsetof(control_msg, invalid);
    control_offsets[3] = offsetof(control_msg, start);

    MPI_Type_create_struct(4, control_blocklengths, control_offsets, control_types, &MPI_Control_Msg);
    MPI_Type_commit(&MPI_Control_Msg);


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
void send_msg(int other, const std::vector<Node>& nodes, MPI_Request &request){
    MPI_Isend(&nodes[0], nodes.size(), MPI_Vertex, other, BASIC_MESSAGE, MPI_COMM_WORLD, &request);
    messageCount += 1;
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

    MPI_Request* requests = new MPI_Request[to_send.size() * 2];
    MPI_Status* statuses = new MPI_Status[to_send.size() * 2];

    int i = 0;
    for (const auto &pair : to_send) {
        int proc = pair.first;
        const std::vector<Node> &node_list = pair.second;
        send_msg(proc, node_list, requests[i]);

        int clock_buf = p_clock;
        MPI_Isend(&clock_buf, 1, MPI_INT, proc, TIME_MESSAGE, MPI_COMM_WORLD, &requests[i + to_send.size()]);
        i += 1;
    }

    MPI_Waitall(to_send.size() * 2, requests, statuses);
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
        MPI_Irecv(&nodes[0], size, MPI_Vertex, MPI_ANY_SOURCE, BASIC_MESSAGE, MPI_COMM_WORLD, &request);
        messageCount -= 1;
        for(Node n : nodes){
            if(closed_set.find(n) == closed_set.end()){
                costs_to_come[n] = n.cost_to_come;
                // printf("%d pushing %d, %d, %f, %f\n", rank, n.x, n.y, n.cost_to_come, n.heuristic_cost);
                open_queue.push(n);
            }
        }        

        MPI_Iprobe(MPI_ANY_SOURCE, 0, MPI_COMM_WORLD, &msg_waiting, &status);
    }

    // get timestamps and set tmax to max timestamp
    int timestamp_waiting;
    MPI_Iprobe(MPI_ANY_SOURCE, TIME_MESSAGE, MPI_COMM_WORLD, &timestamp_waiting, &status);
    while(timestamp_waiting){
        MPI_Request request;
        int t;
        MPI_Irecv(&t, 1, MPI_INT, MPI_ANY_SOURCE, TIME_MESSAGE, MPI_COMM_WORLD, &request);
        tmax = std::max(tmax, t);
        MPI_Iprobe(MPI_ANY_SOURCE, TIME_MESSAGE, MPI_COMM_WORLD, &timestamp_waiting, &status);
    }

    return receivedMessage;
}

int rankMod(int a, int b) {
    return (a % b + b) % b;
}

void startTerminationCheck(int rank, int num_procs){
    p_clock += 1;
    control_msg msg;
    msg.control_clock = p_clock;
    msg.count = messageCount;
    msg.invalid = 0;
    msg.start = rank;
    int next_rank = (rank + 1) % num_procs;
    // std::cout << rank << " sending control message" << std::endl;

    MPI_Request request;
    int test = 1;
    MPI_Isend(&msg, 1, MPI_Control_Msg, next_rank, TERMINATION_MESSAGE, MPI_COMM_WORLD, &request);
    // std::cout << rank << " sent control message" << std::endl;
}

void terminationCheck(int rank, int num_procs){
    int next_rank = rankMod(rank + 1, num_procs);
    int prev_rank = rankMod(rank - 1, num_procs);

    // auto cur_time = std::chrono::steady_clock::now();
    // std::chrono::duration<double> diff = cur_time - start_time;
    // double seconds = diff.count();
    // if(seconds > 10){
    //     if(rank == 0) std::cout << "Timeout" << std::endl;
    //     terminate = true;
    //     return;
    // }


    MPI_Status status;
    int msg_waiting;
    MPI_Iprobe(MPI_ANY_SOURCE, TERMINATION_MESSAGE, MPI_COMM_WORLD, &msg_waiting, &status);
    if(msg_waiting){
        // std::cout << rank << " received control message" << std::endl;
        control_msg msg;
        MPI_Request request;
        MPI_Irecv(&msg, 1, MPI_Control_Msg, prev_rank, TERMINATION_MESSAGE, MPI_COMM_WORLD, &request);
        p_clock = std::max(p_clock, msg.control_clock);
        if(rank == msg.start){
            if(msg.count == 0 && !msg.invalid){
                terminate = true;
                msg.control_clock = -1;

                MPI_Isend(&msg, 1, MPI_Control_Msg, next_rank, TERMINATION_MESSAGE, MPI_COMM_WORLD, &request);
                return;
            }
            else{
                startTerminationCheck(rank, num_procs);
            }
        }
        else{
            if(msg.control_clock == -1){
                MPI_Isend(&msg, 1, MPI_Control_Msg, next_rank, TERMINATION_MESSAGE, MPI_COMM_WORLD, &request);
                terminate = true;
            }
            else{
                msg.count = msg.count + messageCount;
                msg.invalid = msg.invalid || tmax >= msg.control_clock || open_queue.size() > 0;
                MPI_Isend(&msg, 1, MPI_Control_Msg, next_rank, TERMINATION_MESSAGE, MPI_COMM_WORLD, &request);
            }
        }
    }
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
                MPI_Irecv(&receivedMinPath, 1, MPI_FLOAT, end_proc, PATH_LENGTH_MESSAGE, MPI_COMM_WORLD, &request);
                if(receivedMinPath < minLengthPath || minLengthPath == -1){
                    minLengthPath = receivedMinPath;
                    // std::cout << "min length path set to " << minLengthPath << std::endl;
                }
            }
        }
    }


    // if(open_queue.size() == 0){
    //     MPI_Barrier(MPI_COMM_WORLD);
    //     receivedMessages = receiveMessages(rank, num_procs, map);
    //     // std::cout << rank << " received: " << receivedMessages << std::endl;

    //     bool anyReceivedMessages;
    //     MPI_Allreduce(&receivedMessages, &anyReceivedMessages, 1, MPI_C_BOOL, MPI_LOR, MPI_COMM_WORLD);
    //     // std::cout << rank << " any received: " << anyReceivedMessages << std::endl;

    //     if(!anyReceivedMessages){
    //         terminate = true;
    //         return;
    //     }
    // }

    terminationCheck(rank, num_procs);

    // TODO: If message queue is empty, select highest priority state from open set and expand it
    if(open_queue.size() > 0 && !receivedMessages){
        Node n = open_queue.top();
        // printf("n: %d, %d, %f, %f\n", n.x, n.y, n.cost_to_come, n.heuristic_cost);
        open_queue.pop();
        closed_set.emplace(Node(n));
        map.close_node(n.x, n.y);
        node_to_parent[n] = Node(n.parentX, n.parentY);

        if(n == Node(map.endX, map.endY)){
            // printf("found end!\n");
            // terminate = true;
            if(!terminationCheckStarted){
                // std::cout << rank << " beginning termination check" << std::endl;
                terminationCheckStarted = true;
                startTerminationCheck(rank, num_procs);
            }

            if(n.cost_to_come < minLengthPath || minLengthPath < 0){
                minLengthPath = n.cost_to_come;

                for(int i = 0; i < num_procs; i++){
                    if(i == rank) continue;

                    MPI_Request request;
                    MPI_Isend(&minLengthPath, 1, MPI_FLOAT, i, PATH_LENGTH_MESSAGE, MPI_COMM_WORLD, &request);
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
        MPI_Irecv(&path[0], size, MPI_Vertex, MPI_ANY_SOURCE, BASIC_MESSAGE, MPI_COMM_WORLD, &request);
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

void run_mpi_astar(AStarMap map, int rank, int num_procs){
    init(map, rank, num_procs);

    // if(rank == 0) std::cout << "start" << std::endl;
    while(!terminate){
        step(map, rank, num_procs);
    }

    MPI_Barrier(MPI_COMM_WORLD);
    bool flushedMessages = receiveMessages(rank, num_procs, map);

    // get rank of final processor
    int final_rank = map.get_proc(Node(map.endX, map.endY), num_procs);

    if(rank == final_rank){
        // std::cout << "final rank is " << final_rank << std::endl;
        // map.print_assignments(num_procs);
        path.push_back(Node(map.endX, map.endY));
        if(node_to_parent.find(Node(map.endX, map.endY)) == node_to_parent.end()){
            std::cout << "final node has no paren for" << map.startX << " " << map.startY << " " << map.endX << " " << map.endY << std::endl;
        }
        // std::cout << "looking for path" << std::endl;
        // map.render();
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
        std::cout << std::endl;
    }
}

void mpi_astar(int argc, char** argv){
    
    int num_procs, rank;
    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &num_procs);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

    int map_size = 20;

    std::vector<Obstacle> obstacleList = {};
    // std::cout << "make map" << std::endl;
    AStarMap map = AStarMap(map_size, obstacleList, 5, 5, 15, 15);
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

    auto start_time = std::chrono::steady_clock::now();
    // if(rank == 0) std::cout << "init" << std::endl;
    run_mpi_astar(map, rank, num_procs);
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    double seconds = diff.count();
    if(rank == 0){
        // std::cout << "Path is of length " << path.size() << std::endl;
        // std::cout << "Time taken (s): " << seconds << std::endl;
    } 
}

void mpi_astar_metrics(int argc, char** argv, int map_size, int num_iter){
    int num_procs, rank;
    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &num_procs);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    if(rank == 0) std::cout << "Starting metrics" << std::endl;
    auto start_time = std::chrono::steady_clock::now();

    std::vector<Obstacle> obstacleList = {Obstacle(0, 0, 0, 0)};
    for(int i = 0; i < num_iter; i++){
        if(rank == 0) std::cout << i << std::endl;
        MPI_Barrier(MPI_COMM_WORLD);
        AStarMap map = AStarMap(map_size, obstacleList);
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
        run_mpi_astar(map, rank, num_procs);
    }

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    double seconds = diff.count();
    if(rank == 0){
        std::cout << "Avg time taken (s): " << seconds/(double)num_iter << std::endl;
    } 
        
}

#endif