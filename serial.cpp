#include <graph.hpp>
#include <vector>
#include <queue>
#include <unordered_set>


void serial_astar(Node start, Node end, int grid_size){
    std::priority_queue<Node> open_set;
    std::unordered_set<Node> closed_set;

    open_set.push(start);
    while(open_set.size() > 0){
        Node cur = open_set.top();
        open_set.pop();
        if(closed_set.find(cur) != closed_set.end()){
            continue;
        }
        closed_set.emplace(cur);

        std::vector<std::vector<int>> dirn = cur.get_neighbor_directions();
        for(std::vector<int> d : dirn){
            Node n = Node(cur.pos.x + d[0], cur.pos.y + d[1]);
            auto found_node = closed_set.find(n);
            if(found_node != closed_set.end()){
                continue;
            }
            n.cost_to_come = cur.cost_to_come + grid_size;
            n.heuristic_cost = n.cost_to_come + n.heuristic(end);
        }
    }
}