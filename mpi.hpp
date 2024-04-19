#ifndef FINAL_MPI
#define FINAL_MPI

#include <mpi.h>
#include <vector>
#include <queue>
#include <unordered_set>
#include "graph.hpp"

std::priority_queue<Node*, std::vector<Node*>, NodeCompare> open_queue;
std::unordered_set<Node*, NodeHash, NodeEqual> closed_set;

int my_rank;
int total_num_procs;

void init(AStarMap map, int rank, int num_procs){
    my_rank = rank;
    total_num_procs = num_procs;

    int init_processor = map.get_proc(*map.start, num_procs);
    if(rank == init_processor){
        open_queue.push(map.start);
    }
}

void send_node(Node n, int other){
    int buffer[4];
    buffer[0] = n.pos.x;
    buffer[1] = n.pos.y;
    if(n.parent == nullptr){
        buffer[2] = -1;
        buffer[3] = -1;
    }
    else{
        buffer[2] = n.parent->pos.x;
        buffer[3] = n.parent->pos.y;
    }
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