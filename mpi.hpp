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
MPI_Datatype MPI_Vertex;

int my_rank;
int total_num_procs;

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

    int init_processor = map.get_proc(*map.start, num_procs);
    if(rank == init_processor){
        open_queue.push(*map.start);
    }
}

void send_node(Node n, int other){
    int buffer[4];
    buffer[0] = n.x;
    buffer[1] = n.y;
    buffer[2] = n.parentX;
    buffer[3] = n.parentY;
    buffer[4] = n.heuristic_cost;
    buffer[5] = n.cost_to_come;
    MPI_Request request;
    MPI_Isend(buffer, 4, MPI_INTEGER, other, 0, MPI_COMM_WORLD, &request);
}

void receive_buf(int other, int* buffer){
    MPI_Request request;
    MPI_Irecv(buffer, 4, MPI_INT, other, 0, MPI_COMM_WORLD, &request);
}

// Node* receive_node(int other, &int buffer){
//     MPI_Request request;
//     MPI_Irecv(&buffer, 4, MPI_INT, other, 0, MPI_COMM_WORLD, &request);
//     // if request is not fulfilled, return
//     Node n = new Node(buffer[0], buffer[1]);
//     if(buffer[2] != -1){
//         Node parent = new Node(buffer[2], buffer[3]);
//         n.parent = parent;
//     }

//     return n;
// }

void step(AStarMap map, int rank, int num_procs){
    // TODO: check if new states have been received in the message queue
    int buffer[num_procs][4];
    for(int i = 0; i < num_procs; i++){
        receive_buf(i, buffer[i]);
    }

    // TODO: If message queue is empty, select highest priority state from open set and expand it

    // TODO: Convergence check. Add barrier if size of openSet is 0. If no procs have new messages, terminate
}

#endif