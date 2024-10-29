#ifndef GRAPH_SEARCH_
#define GRAPH_SEARCH_

#include <algorithm>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <ios>
#include <random>
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

#include "search.hpp"
#include "graph.cpp"
#include "utili.hpp"


// functor that allow comparison of two nodes based on a specific f-score (objective cost.)
struct more_than_specific{
  const size_t cost_idx; 

  more_than_specific(const size_t cost_idx_): cost_idx(cost_idx_) {};
  
  bool operator()(const NodePtr &a, const NodePtr &b) const {
    return (a->f[cost_idx] > b->f[cost_idx]);
  }
};

//functor for heuristic function, two "States." 0 for euclidean distance, 1 for Haversine distance
struct h_functor {
  size_t state;

  h_functor(size_t ver): state(ver) {}

  double operator()(const NodePtr &a, const NodePtr &b){

    if(state == 0){           // EUCLIDEAN DISTANCE
      return sqrt(pow((a.get()->x - b.get()->x), 2) + pow((a.get()->y - b.get()->y), 2) );
      
    } else if ( state == 1){  // HAVERSINE DISTANCE (using kilometers)

      double Lata = a.get()->x, //lat
             Latb = b.get()->x, 
             Lnga = a.get()->y, //lng
             Lngb = b.get()->y;

      double delta_lat = (Latb - Lata) * M_PI / 180.0;
      double delta_lng = (Lngb - Lnga) * M_PI / 180.0;

      double lat_1 = Lata * M_PI / 180.0;
      double lat_2 = Latb * M_PI / 180.0;

      double a = pow(sin(delta_lat / 2), 2) +
                 pow(sin(delta_lng / 2), 2) * 
                 cos(lat_1) * cos(lat_2);

      double rad = 6371;     // aprox radius of the earth in km 
      double c = 2 * asin(sqrt(a));
      return rad * c;
    } else {
      return -1;
      }
  }
};


using heuristic = std::function<double (const NodePtr&, const NodePtr&)>;
using node_order = std::function<bool(const NodePtr&, const NodePtr&)>;
using NodePtr = std::shared_ptr<Node>;

/**
 * A* Algorithm 
 */

class ASTAR{
  private:
  const AdjMatrix            &adj_matrix;
  const std::vector<NodePtr> &node_list;

  public:
  ASTAR(const AdjMatrix &adj_matrix_, const std::vector<NodePtr> &node_list_);

  NodePtr operator()(const size_t source, const size_t target, heuristic &h, node_order &order);

};

/**
 * Weighted A* Algorithm
 */


class WEIGHTED_ASTAR {
  private:
  const AdjMatrix             &adj_matrix;
  const std::vector<NodePtr>  &node_list;
  const std::vector<double>   &weight_set;

  public:

  WEIGHTED_ASTAR(const AdjMatrix &adj_matrix_, const std::vector<NodePtr> &node_list_, const std::vector<double> &weight_set_);
  // WEIGHTED COMBINED VERSION
  NodePtr operator()(const size_t source, const size_t target, heuristic &h, node_order &order);

  // WEIGHTED CONSCIOUS VERSION
  NodePtr operator()(const size_t source, const size_t target, heuristic &h, node_order &order, const int focus);
};

#endif