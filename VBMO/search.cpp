#include <algorithm>
#include <cstdio>
#include <ctime>
#include <vector>
#include <unordered_set>
#include <functional>

#include "search.hpp"
#include "graph.cpp"
#include "utili.hpp"


// These will be the placeholder for the functor object that we for either for node ordering in the open set, and to calculate the heuristic
using heuristic = std::function<double (const NodePtr&, const NodePtr&)>;
using node_order = std::function<bool(const NodePtr&, const NodePtr&)>;
using NodePtr = std::shared_ptr<Node>;

//
// A* and weighted A* objects
//

ASTAR::ASTAR(const AdjMatrix &adj_matrix_, const std::vector<NodePtr> &node_list_): adj_matrix(adj_matrix_), node_list(node_list_) {}

// version that the paper uses. Not optimal?
NodePtr ASTAR::operator()(const size_t source, const size_t target, heuristic &h, node_order &order){

  std::unordered_set<size_t> closed;    // the CLOSED set, inclusion iff visited 

  std::vector<NodePtr> open;            // the OPEN set (priority queue)
  std::make_heap(open.begin(), open.end(), order);

  int b = adj_matrix.get_obj_count();   // the number of objectives present in the graph    
  
  NodePtr node = std::make_shared<Node>(node_list[source]->x,
                                        node_list[source]->y,
                                        node_list[source]->id,
                                        std::vector<double>(adj_matrix.get_obj_count(), 0), // the g-score
                                        h(node_list[source],node_list[target]));

  open.push_back(node); // add source node to open set
  std::push_heap(open.begin(), open.end(), order);


  while(open.empty() == false){       // MAIN LOOP

    // Retrieve the node with te lowest f-score according to the selected objective (paramater order) (called node hence)
    std::pop_heap(open.begin(), open.end(), order);
    node = open.back();
    open.pop_back();
    
    // Check if node has already been visited
    if(closed.find(node->id) != closed.end()){ // This is not how its typically done!
      continue;
    }
    // Else add it too closed list
    closed.insert(node->id);

    // Check if the node is our target
    if( node->id == target){
      return node;
    }

    // iterating over nodes neighbors
    const std::vector<Edge> &outgoing = adj_matrix[node->id];                 // Get all nodes adjacent to node
    for(auto p_edge = outgoing.begin(); p_edge != outgoing.end(); p_edge++){  // iterating over all neighbors
      
      size_t next_id = p_edge->target;
      std::vector<double> next_g = node->g;
      for(int i = 0; i < p_edge->cost.size(); i++){
        next_g[i] += p_edge->cost[i];
      }

      // std::cout << "edge cost: " << p_edge->cost;
      double next_h = h(node,node_list[next_id]);

      // ignore neighboring nodes if they were visted
      if(closed.find(next_id) != closed.end()){
        continue;
      }

      auto next = std::make_shared<Node>(node_list[next_id]->x,
                                          node_list[next_id]->y, 
                                          next_id, 
                                          next_g, 
                                          next_h, 
                                          node);


      open.push_back(next);
      std::push_heap(open.begin(), open.end(), order);
    }
  }       // END MAIN LOOP

  //only reached if there is no path
  return nullptr;
}


WEIGHTED_ASTAR::WEIGHTED_ASTAR(const AdjMatrix &adj_matrix_, const std::vector<NodePtr> &node_list_, const std::vector<double> &weight_set_)  
  : adj_matrix(adj_matrix_), node_list(node_list_), weight_set(weight_set_) {}

// WEIGHTED COMBINED VERSION
NodePtr WEIGHTED_ASTAR::operator()(const size_t source, const size_t target, heuristic &h, node_order &order){

  std::unordered_set<size_t> closed;    // the CLOSED set, incusuion -> visted 

  std::vector<NodePtr> open;            // the OPEN set (priority queue)
  std::make_heap(open.begin(), open.end(), order);

  int b = adj_matrix.get_obj_count();   // the number of objectives present in the graph    
  
  NodePtr node = std::make_shared<Node>(node_list[source]->x,
                                        node_list[source]->y,
                                        node_list[source]->id,
                                        std::vector<double>(adj_matrix.get_obj_count(), 0), // the g-score
                                        h(node_list[source],node_list[target]));


  open.push_back(node); // add source node
  std::push_heap(open.begin(), open.end(), order);


  while(open.empty() == false){       // MAIN LOOP

    // Retrive the node with te lowest f-sccore according to the selected objective (paramater order) (called node hence)
    std::pop_heap(open.begin(), open.end(), order);
    node = open.back();
    open.pop_back();
    
    // Check if node has already been visited
    if(closed.find(node->id) != closed.end()){ // This is not how its typically done!
      continue;
    }
    // Else add it too closed list
    closed.insert(node->id);

    // Check if the node is our target
    if( node->id == target){
      return node;
    }

    // iterating over nodes neighbors
    const std::vector<Edge> &outgoing = adj_matrix[node->id];                 // Get all nodes adjacent to node
    for(auto p_edge = outgoing.begin(); p_edge != outgoing.end(); p_edge++){  // iterating over all neighbors
      
      size_t next_id = p_edge->target;
      std::vector<double> next_g = node->g;
      for(int i = 0; i < p_edge->cost.size(); i++){
        next_g[i] += p_edge->cost[i];
      }

      // std::cout << "edge cost: " << p_edge->cost;
      double next_h = h(node,node_list[next_id]);

      // ignore neighboring nodes if they were visted
      if(closed.find(next_id) != closed.end()){
        continue;
      }

      auto next = std::make_shared<Node>(node_list[next_id]->x,
                                          node_list[next_id]->y, 
                                          next_id, 
                                          next_g, 
                                          next_h, // must pass a weight set for the weighted sum
                                          weight_set,
                                          node);


      open.push_back(next);
      std::push_heap(open.begin(), open.end(), order);
    }
  }      // END MAIN LOOP

  //only reached if there is no path
  return nullptr;
}

// WEIGHTED CONSCIOUS VERSION
NodePtr WEIGHTED_ASTAR::operator()(const size_t source, const size_t target, heuristic &h, node_order &order, const int focus){

  std::unordered_set<size_t> closed;    // the CLOSED set, incusuion -> visted 

  std::vector<NodePtr> open;            // the OPEN set (priority queue)
  std::make_heap(open.begin(), open.end(), order);

  int b = adj_matrix.get_obj_count();   // the number of objectives present in the graph    
  
  NodePtr node = std::make_shared<Node>(node_list[source]->x,
                                        node_list[source]->y,
                                        node_list[source]->id,
                                        std::vector<double>(adj_matrix.get_obj_count(), 0), // the g-score
                                        h(node_list[source],node_list[target]));


  open.push_back(node); // add source node
  std::push_heap(open.begin(), open.end(), order);


  while(open.empty() == false){       // MAIN LOOP

    // Retrive the node with te lowest f-sccore according to the selected objective (paramater order) (called node hence)
    std::pop_heap(open.begin(), open.end(), order);
    node = open.back();
    open.pop_back();
    
    // Check if node has already been visited
    if(closed.find(node->id) != closed.end()){ // This is not how its typically done!
      continue;
    }
    // Else add it too closed list
    closed.insert(node->id);

    // Check if the node is our target
    if( node->id == target){
      return node;
    }

    // iterating over nodes neighbors
    const std::vector<Edge> &outgoing = adj_matrix[node->id];                 // Get all nodes adjacent to node
    for(auto p_edge = outgoing.begin(); p_edge != outgoing.end(); p_edge++){  // iterating over all neighbors
      
      size_t next_id = p_edge->target;
      std::vector<double> next_g = node->g;
      for(int i = 0; i < p_edge->cost.size(); i++){
        next_g[i] += p_edge->cost[i];
      }

      // std::cout << "edge cost: " << p_edge->cost;
      double next_h = h(node,node_list[next_id]);

      // ignore neighboring nodes if they were visted
      if(closed.find(next_id) != closed.end()){
        continue;
      }

      auto next = std::make_shared<Node>(node_list[next_id]->x,
                                          node_list[next_id]->y, 
                                          next_id, 
                                          next_g, 
                                          next_h, // must pass a weight set for the weighted sum
                                          weight_set,
                                          focus,
                                          node);


      open.push_back(next);
      std::push_heap(open.begin(), open.end(), order);
    }
  }      // END MAIN LOOP

  //only reached if there is no path
  return nullptr;
}