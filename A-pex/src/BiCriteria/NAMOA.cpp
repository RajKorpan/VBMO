 #include "NAMOA.h"
#include "Utils/MapQueue.h"


bool is_dominated_dr(NodePtr node, std::list<NodePtr>& list, EPS eps){
    for (auto n: list){
        if (is_dominated_dr(node, n, eps)){
            return true;
        }
    }
    return false;
}

void add_node_dr(NodePtr node, std::list<NodePtr>& list){
    for (auto it = list.begin(); it != list.end(); ){
        if (is_dominated_dr((*it), node)){
            it = list.erase(it);
        } else {
            it ++;
        }
    }
    list.push_back(node);
}

bool is_dominated_dr(NodePtr node, std::list<NodePtr>& list){
    for (auto n: list){
        if (is_dominated_dr(node, n)){
            return true;
        }
    }
    return false;
}


// void add_list(NodePtr node, std::list<NodePtr>& list){
    
// }

void NAMOAdr::operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit) {
    this->start_logging(source, target);
    auto start_time = std::clock();

    // std::list<NodePtr> solution_dr;

    NodePtr node;
    NodePtr next;

    NodeQueue open(this->adj_matrix.size()+1);

    // Saving all the unused NodePtrs in a vector improves performace for some reason
    // std::vector<NodePtr> closed;

    // Vector to hold mininum cost of 2nd criteria per node
    std::vector<std::list<NodePtr>> closed(this->adj_matrix.size()+1);

    // Init open heap
    // std::vector<NodePtr> open;
    // std::make_heap(open.begin(), open.end(), more_than);

    node = std::make_shared<Node>(source, std::vector<double>(adj_matrix.get_num_of_objectives(),0), heuristic(source));
    open.insert(node);

    while (open.empty() == false) {
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){
            this->end_logging(solutions, false);
            return;
        }
        // Pop min from queue and process
        node = open.pop();
        num_generation +=1;

        if (is_dominated_dr(node, closed[target], eps) ||
            is_dominated_dr(node, closed[node->id])
            ){
            continue;
        }
        add_node_dr(node, closed[node->id]);

        num_expansion += 1;

        if (node->id == target) {
            solutions.push_back(node);
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = adj_matrix[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            // std::vector<size_t> next_g = {node->g[0]+p_edge->cost[0], node->g[1]+p_edge->cost[1]};
            std::vector<double> next_g(node->g.size());
            for (size_t i = 0; i < next_g.size(); i++){
                next_g[i] = node->g[i] + p_edge->cost[i];
            }
            auto next_h = heuristic(next_id);
            next = std::make_shared<Node>(next_id, next_g, next_h, node);

            // Dominance check
            if (is_dominated_dr(next ,closed[next_id]) || is_dominated_dr(next, closed[target], eps))
                {
                    continue;
                }

            // If not dominated create node and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            open.insert(next);

        }
    }

    this->end_logging(solutions);
}
