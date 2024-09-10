#include <algorithm>
#include <unordered_map>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <ios>
#include <ostream>
#include <random>
#include <regex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <unordered_set>
#include <fstream>
#include <iostream>
#include <limits>
#include <functional>
#include <memory>
#include <cmath>
#include <chrono>

#include "graph.hpp"

// ASTAR

ASTAR::ASTAR(const AdjMatrix &adj_matrix_, std::vector<NodePtr> &node_list_):
  adj_matrix(adj_matrix_), node_list(node_list_) {}

NodePtr ASTAR::operator()(const size_t source, const size_t target, heuristic &h, node_order &order) const {
  std::unordered_set<size_t> closed;
  std::vector<NodePtr> open;

  std::make_heap(open.begin(), open.end(), order);
  int b = adj_matrix.get_obj_count();
  NodePtr node = std::make_shared<Node>(node_list[source]->x,
                                        node_list[source]->y,
                                        node_list[source]->id,
                                        std::vector<double>(adj_matrix.get_obj_count(), 0),
                                        h(node_list[source], node_list[target]));
  open.push_back(node);
  std::push_heap(open.begin(), open.end(), order);

  // BEGIN main loop
  while(!open.empty()){
    // retrieve the node with the lowest f-score for a specific objective function (as determined by the parameter order)
    std::pop_heap(open.begin(), open.end(), order);
    node = open.back();
    open.pop_back();

    // discard visted nodes?
    if(closed.find(node->id) != closed.end()){
       continue;
    } else {
      closed.insert(node->id);
    }

    if(node->id == target){ //motify to retrace path
      return node;
    }

    // iterate over nodes neighbors
    const std::vector<Edge> &outgoing = adj_matrix[node->id];
    for(auto p_edge = outgoing.begin(); p_edge != outgoing.end(); p_edge++){
      size_t next_id = p_edge->target;

      if(closed.find(next_id) != closed.end()){ // if next_id has not been visited (i.e. not in the closed list)
        continue;
      }

      std::vector<double> next_g = node->g;
      for(int i = 0; i < next_g.size(); i++){
        next_g[i] += p_edge->cost[i];
      }
      double next_h = h(node, node_list[next_id]);
      auto next = std::make_shared<Node>(node_list[next_id]->x,
                                         node_list[next_id]->y, 
                                         next_id, 
                                         next_g, 
                                         next_h, 
                                         node);
      //update parent
      node_list[next_id]->parent = node_list[node->id];

      open.push_back(next);
      std::push_heap(open.begin(), open.end(), order);
    }
  } // END main loop
  return nullptr;
}

NodePtr ASTAR::operator()(const size_t source, const size_t target, heuristic &h, node_order &order, std::vector<size_t> &trace_back) const {
  std::unordered_set<size_t> closed;
  std::vector<NodePtr> open;

  std::make_heap(open.begin(), open.end(), order);
  int b = adj_matrix.get_obj_count();
  NodePtr node = std::make_shared<Node>(node_list[source]->x,
                                        node_list[source]->y,
                                        node_list[source]->id,
                                        std::vector<double>(adj_matrix.get_obj_count(), 0),
                                        h(node_list[source], node_list[target]));
  open.push_back(node);
  std::push_heap(open.begin(), open.end(), order);

  // BEGIN main loop
  while(!open.empty()){
    // retrieve the node with the lowest f-score for a specific objective function (as determined by the parameter order)
    std::pop_heap(open.begin(), open.end(), order);
    node = open.back();
    open.pop_back();


    // discard visted nodes?
    if(closed.find(node->id) != closed.end()){
       continue;
    } 
    closed.insert(node->id);

    // std::cout << node->id << std::endl;


    if(node->id == target){ //motify to retrace path
      // trace back code
      // node_list[target]->parent = node;

      auto searchPtr = node_list[target];
      while(searchPtr != node_list[source]){
        trace_back.push_back(searchPtr->id);
        searchPtr = searchPtr->parent;
      }
      return node;
    }

    // iterate over nodes neighbors
    const std::vector<Edge> &outgoing = adj_matrix[node->id];
    for(auto p_edge = outgoing.begin(); p_edge != outgoing.end(); p_edge++){
      size_t next_id = p_edge->target;

      if(closed.find(next_id) != closed.end()){ // if next_id has not been visited (i.e. not in the closed list)
        continue;
      }

      std::vector<double> next_g = node->g;
      for(int i = 0; i < next_g.size(); i++){
        next_g[i] += p_edge->cost[i];
      }
      double next_h = h(node, node_list[next_id]);
      auto next = std::make_shared<Node>(node_list[next_id]->x,
                                         node_list[next_id]->y, 
                                         next_id, 
                                         next_g, 
                                         next_h, 
                                         node);
      //update parent
      node_list[next_id]->parent = node_list[node->id];

      open.push_back(next);
      std::push_heap(open.begin(), open.end(), order);
    }
  } // END main loop
  return nullptr;
}


//WEIGHTED ASTAR

WEIGHTED_ASTAR::WEIGHTED_ASTAR(const AdjMatrix &adj_matrix_, std::vector<NodePtr> &node_list_, std::vector<double> weight_set_)  
  : adj_matrix(adj_matrix_), node_list(node_list_), weight_set(weight_set_) {}

WEIGHTED_ASTAR::WEIGHTED_ASTAR(const AdjMatrix &adj_matrix_, std::vector<NodePtr> &node_list_)
  : adj_matrix(adj_matrix_), node_list(node_list_){
  weight_set = std::vector<double>(adj_matrix_.get_obj_count(), 1.0);
}  

void WEIGHTED_ASTAR::update_weight(const std::vector<double> new_weight_set){
  weight_set = new_weight_set;
}

// WEIGHTED
NodePtr WEIGHTED_ASTAR::operator()(const size_t source, const size_t target, heuristic &h, node_order &order) const {
  std::unordered_set<size_t> closed;
  std::vector<NodePtr> open;

  std::make_heap(open.begin(), open.end(), order);
  int b = adj_matrix.get_obj_count();
  NodePtr node = std::make_shared<Node>(node_list[source]->x,
                                        node_list[source]->y,
                                        node_list[source]->id,
                                        std::vector<double>(b, 0),
                                        h(node_list[source], node_list[target]));
  open.push_back(node);
  std::push_heap(open.begin(), open.end(), order);

  // BEGIN main loop
  while(!open.empty()){
    // retrieve the node with the lowest f-score for a specific objective function (as determined by the parameter order)
    std::pop_heap(open.begin(), open.end(), order);
    node = open.back();
    open.pop_back();


    // discard visted nodes?
    if(closed.find(node->id) != closed.end()){
       continue;
    } 
    closed.insert(node->id);

    // std::cout << node->id << std::endl;


    if(node->id == target){ //motify to retrace path
      // trace back code
      // std::cout << node->id << std::endl;
      // node_list[target]->parent = node;

      // auto searchPtr = node_list[target];
      // while(searchPtr != node_list[source]){
      //   trace_back.push_back(searchPtr->id);
      //   searchPtr = searchPtr->parent;
      // }
      return node;
    }

    // iterate over nodes neighbors
    const std::vector<Edge> &outgoing = adj_matrix[node->id];
    for(auto p_edge = outgoing.begin(); p_edge != outgoing.end(); p_edge++){
      size_t next_id = p_edge->target;

      if(closed.find(next_id) != closed.end()){ // if next_id has not been visited (i.e. not in the closed list)
        continue;
      }

      std::vector<double> next_g = node->g;
      for(int i = 0; i < next_g.size(); i++){
        next_g[i] += p_edge->cost[i];
      }
      double next_h = h(node, node_list[next_id]);
      auto next = std::make_shared<Node>(node_list[next_id]->x,
                                         node_list[next_id]->y, 
                                         next_id, 
                                         next_g, 
                                         next_h, 
                                         weight_set,
                                         node);
      //update parent
      node_list[next_id]->parent = node_list[node->id];

      open.push_back(next);
      std::push_heap(open.begin(), open.end(), order);
    }
  } // END main loop
  return nullptr;
}

//   std::unordered_set<size_t> closed;
//   std::vector<NodePtr> open;

//   std::make_heap(open.begin(), open.end(), order);
//   int b = adj_matrix.get_obj_count();
//   NodePtr node = std::make_shared<Node>(node_list[source]->x,
//                                         node_list[source]->y,
//                                         node_list[source]->id,
//                                         std::vector<double>(adj_matrix.get_obj_count(), 0),
//                                         h(node_list[source], node_list[target]));
//   open.push_back(node);
//   std::push_heap(open.begin(), open.end(), order);

//   // BEGIN main loop
//   while(!open.empty()){
//     // retrieve the node with the lowest f-score for a specific objective function (as determined by the parameter order)
//     std::pop_heap(open.begin(), open.end(), order);
//     node = open.back();
//     open.pop_back();


//     // discard visted nodes?
//     if(closed.find(node->id) != closed.end()){
//        continue;
//     } 
//     closed.insert(node->id);

//     // std::cout << node->id << std::endl;


//     if(node->id == target){ //motify to retrace path
//       // trace back code
//       // node_list[target]->parent = node;

//       auto searchPtr = node_list[target];
//     }

//     // iterate over nodes neighbors
//     const std::vector<Edge> &outgoing = adj_matrix[node->id];
//     for(auto p_edge = outgoing.begin(); p_edge != outgoing.end(); p_edge++){
//       size_t next_id = p_edge->target;

//       if(closed.find(next_id) != closed.end()){ // if next_id has not been visited (i.e. not in the closed list)
//         continue;
//       }

//       std::vector<double> next_g = node->g;
//       for(int i = 0; i < next_g.size(); i++){
//         next_g[i] += p_edge->cost[i];
//       }
//       double next_h = h(node, node_list[next_id]);
//       auto next = std::make_shared<Node>(node_list[next_id]->x,
//                                          node_list[next_id]->y, 
//                                          next_id, 
//                                          next_g, 
//                                          next_h,
//                                          weight_set,
//                                          node);
//       //update parent
//       node_list[next_id]->parent = node_list[node->id];

//       open.push_back(next);
//       std::push_heap(open.begin(), open.end(), order);
//     }
//   } // END main loop
//   return nullptr;
// }

// WEIGHTED CONCIOUS 
NodePtr WEIGHTED_ASTAR::operator()(const size_t source, const size_t target, heuristic &h, node_order &order, const int focus) const {
  std::unordered_set<size_t> closed;
  std::vector<NodePtr> open;

  std::make_heap(open.begin(), open.end(), order);
  int b = adj_matrix.get_obj_count();
  NodePtr node = std::make_shared<Node>(node_list[source]->x,
                                        node_list[source]->y,
                                        node_list[source]->id,
                                        std::vector<double>(adj_matrix.get_obj_count(), 0),
                                        h(node_list[source], node_list[target]));
  open.push_back(node);
  std::push_heap(open.begin(), open.end(), order);

  // BEGIN main loop
  while(!open.empty()){
    // retrieve the node with the lowest f-score for a specific objective function (as determined by the parameter order)
    std::pop_heap(open.begin(), open.end(), order);
    node = open.back();
    open.pop_back();


    // discard visted nodes?
    if(closed.find(node->id) != closed.end()){
       continue;
    } 
    closed.insert(node->id);

    // std::cout << node->id << std::endl;


    if(node->id == target){ //motify to retrace path
      // trace back code
      // node_list[target]->parent = node;

      auto searchPtr = node_list[target];
    }

    // iterate over nodes neighbors
    const std::vector<Edge> &outgoing = adj_matrix[node->id];
    for(auto p_edge = outgoing.begin(); p_edge != outgoing.end(); p_edge++){
      size_t next_id = p_edge->target;

      if(closed.find(next_id) != closed.end()){ // if next_id has not been visited (i.e. not in the closed list)
        continue;
      }

      std::vector<double> next_g = node->g;
      for(int i = 0; i < next_g.size(); i++){
        next_g[i] += p_edge->cost[i];
      }
      double next_h = h(node, node_list[next_id]);
      auto next = std::make_shared<Node>(node_list[next_id]->x,
                                         node_list[next_id]->y, 
                                         next_id, 
                                         next_g, 
                                         next_h,
                                         weight_set,
                                         focus,
                                         node);
      //update parent
      node_list[next_id]->parent = node_list[node->id];

      open.push_back(next);
      std::push_heap(open.begin(), open.end(), order);
    }
  } // END main loop
  return nullptr;
}


AdjMatrix::AdjMatrix(size_t graph_size_, std::vector<Edge> &edges, bool inverse)
  : matrix((graph_size_ + 1), std::vector<Edge>()), graph_size(graph_size_) { // + 1 because node count start at 1
    obj_count = edges[0].cost.size();
    for(auto iter = edges.begin(); iter != edges.end(); ++iter){ // Represented as an adjaceny list
      if(inverse) {
        this->add(iter->inverse());
      } else {
        this->add(*iter);
      }
    }      
  }

void AdjMatrix::add(Edge e){
  (this->matrix[e.source]).push_back(e);
}

size_t AdjMatrix::size() const {
  return this->graph_size;
}

size_t AdjMatrix::edge_size() const {
  size_t n = 0;
  for(auto &i: matrix){
    n += i.size();
  }    
  return n;
}

size_t AdjMatrix::get_obj_count() const{
   return obj_count;
}


void write_json_array(std::ostream &out_file, std::vector<double> &vec){
  if(vec.empty()){
    out_file << "[]";
  } else {
    out_file << "[" << vec[0];
    for(int i = 1; i < vec.size(); i++){
      out_file << ", " << vec[i];
    }
    out_file << "]";
  }
}

void write_edge(std::ostream &out_file, Edge &e){
  out_file  << "{" 
            << "\"source\": " << e.source 
            << "\"target\": " << e.target
            << "\"cost\": "; write_json_array(out_file, e.cost);
  out_file  << "},";
}

void AdjMatrix::jsonify(std::string FILENAME){
  std::ofstream out_file(FILENAME + ".json");
  out_file << "{\"node_cout\": " << graph_size << ", ";
  out_file << "\"graph\": [";

  std::vector<Edge> edges;
  for(auto &iter : matrix){
    edges.insert(iter.begin(), iter.end(), edges.end());
  }


  write_edge(out_file, edges[0]); out_file << ", ";
  for(int i = 1; i < edges.size(); i++){
    out_file << ", "; write_edge(out_file, edges[i]);
  }

  out_file << "]}";
}


