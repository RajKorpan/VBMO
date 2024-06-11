#ifndef GRAPH_
#define GRAPH_

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


/**
 * EDGE CLASS
*/

struct Edge {
  size_t               source,
                       target;
  std::vector<double>  cost;

  Edge(size_t source_, size_t target_, std::vector<double> cost_): source(source_), target(target_), cost(cost_) {}

  Edge inverse(){
    return Edge(this->target, this->source, this->cost);
  }
};

std::ostream& operator<<(std::ostream &stream, const Edge &edge);


/**
 *  Adjacency Matrix Class
 *  
*/
class AdjMatrix {
  private:
  std::vector<std::vector<Edge>>   matrix;      // [i] will return a reference to a vector of edges with source i. (thus checking for a particular entry is O(|V|)) not O(1) which is the deifntion of a Adj Matrix)
  size_t                           graph_size;  // the number of nodes (vertices) in the graph
  size_t                           obj_count = 0;

  public:

  AdjMatrix() = default;

  AdjMatrix(size_t graph_size_, std::vector<Edge> &edges, bool inverse);

  void add(Edge e);

  size_t size(void) const;

  size_t get_obj_count() const;

  const std::vector<Edge>& operator[](size_t vertex_id) const;
    
  friend std::ostream& operator<<(std::ostream &stream, const AdjMatrix &adj_matrix);
};


/**
 * NODE (VERTEX) CLASS
*/ 
using NodePtr = std::shared_ptr<Node>;

struct Node {
  size_t                id;
  std::vector<double>   g;  // the best current score from the source node for each objective cost ()
  
  //h originally had a vector of size_t, but for most applications, specifically graph searching, a spatial heuristic is sufficent
  double                h; // separate heuristic for the objectives? only need one?
  std::vector<double>   f;
  NodePtr               parent;

  // interpreted as lng, lat. Or as x,y coordinate in the case of 
  double                 x,y;


  Node(): id(-1), g({}), h(0), f({}), parent(nullptr), x(-1.0), y(-1.0) {}

  Node(double x_, double y_, size_t id_)
  : x(x_), y(y_), id(id_) {}

  // constructor used for adding to the open set in default A* open set
  Node(double x_, double y_, size_t id_, std::vector<double> g_, double h_, NodePtr parent_ = nullptr)
  : x(x_), y(y_), id(id_), g(g_), h(h_), f({}), parent(parent_){
    for(int i = 0; i < g.size(); i++){
      f.push_back(g[i] + h);
    }
  }

  // constructor used for weighted combined A* open set
  Node(double x_, double y_, size_t id_, std::vector<double> g_, double h_, const std::vector<double> &weight_set,  NodePtr parent_ )
  : x(x_), y(y_), id(id_), g(g_), h(h_), f({}), parent(parent_){
    double weighted_sum = 0;
    for(int i = 0; i < g.size(); i++){        // in the weighted combined, there is only one objective (the weighted sum)
      weighted_sum += (g[i] * weight_set[i]);
    }
    f.push_back(weighted_sum + h);
  }

  //constructor used for weighted conscious A* open set
  Node(double x_, double y_, size_t id_, std::vector<double> g_, double h_, const std::vector<double> &weight_set, const int focus, NodePtr parent_ )
  : x(x_), y(y_), id(id_), g(g_), h(h_), f({}), parent(parent_){
    double focused_sum= 0;
    for(int i = 0; i < g.size(); i++){        // in the weighted combined, there is only one objective (the weighted sum)
      if(i == focus){
        focused_sum += g[i];
      } else {
        focused_sum += g[i] * weight_set[i];
      }
    }
    f.push_back(focused_sum + h);
  }

  Node(const Node& rhs):
    x(rhs.x), y(rhs.y), id(rhs.y), h(rhs.h), f(rhs.f), parent(rhs.parent){}

  Node(Node&& rhs):
    x(rhs.x), y(rhs.y), id(rhs.y), h(rhs.h), f(rhs.f), parent(rhs.parent){}
  
  Node& operator=(const Node& rhs) {
    id = rhs.id;
    g = rhs.g;
    h = rhs.h;   
    f = rhs.f;
    parent= rhs.parent;
    x = rhs.x;
    y = rhs.y;

    return *this;
  }
  
  Node& operator=(Node&& rhs){
    id = rhs.id;
    g = rhs.g;
    h = rhs.h;   
    f = rhs.f;
    parent= rhs.parent;
    x = rhs.x;
    y = rhs.y;

    return *this;
  }

  ~Node(){}

  // RECALL:
  // each node is has three values
  // g: the current best found to the node from the start node
  // h: the heuristic for the node
  // f: the current "guess" distance from the node to the end node, calculated by g + h

  // comparator where we want to look at the heuristic with respect to a specific cost 
  //however, we will always the same heuristic for each objective

  friend std::ostream& operator<<(std::ostream &stream, const Node &node) {
    stream << std::fixed;
    stream << "{id: " << node.id << ", x: " << std::setprecision(4) << node.x << ", y: " << std::setprecision(4) << node.y <<  "}";

    return stream;
  }

  void display(){
    std::cout << "{" << std::endl;
    std::cout << "  " << *this << std::endl;
    std::cout 
              << "  f-score: " << this->f << '\n'
              << "  g-score: " << this->g << '\n'
              << "  h      : " << this->h << '\n'
              << "}" << std::endl;
  }
};

#endif
