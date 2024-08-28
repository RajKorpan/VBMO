#ifndef GRAPH_
#define GRAPH_

#include <cstdio>
#include <ctime>
#include <iomanip>
#include <ios>
#include <vector>
#include <iostream>
#include <functional>
#include <memory>
#include <cmath>

struct Node;

using NodePtr = std::shared_ptr<Node>;

struct Node{
  size_t                id;
  NodePtr               parent;
  std::vector<double>   g, // g-score, or best current path cost to this node
                        f; // the estimated cost for each objective for this node to the target
  double                h, // heuriistic for cost (in this implementation, no specific objective is specificed)
                        v, // velocity, needed for certain objectives
                        x,y; // lat-lng cordinates, or x,y cordinates

  // used for node list,
  Node(double x_, double y_, size_t id_)
  : x(x_), y(y_), id(id_) {
    // veloctiy
    double w = std::max(sin(x-1), cos(y-1));
    if(w > 0.9){
      v = 130;
    } else if(w < -0.4){
      v = 50;
    } else {
      v = 100;
    }
  }

  // defaulty A* constructor
  Node(double x_, double y_, size_t id_, std::vector<double> g_, double h_, NodePtr parent_ = nullptr)
  : x(x_), y(y_), id(id_), g(g_), h(h_), f({}), parent(parent_){
    for(int i = 0; i < g.size(); i++){
      f.push_back(g[i] + h);
    }
  }

  // weighted combined A* constructor
  Node(double x_, double y_, size_t id_, std::vector<double> g_, double h_, const std::vector<double> &weight_set,  NodePtr parent_ )
  : x(x_), y(y_), id(id_), g(g_), h(h_), f({}), parent(parent_){
    double weighted_sum = 0;
    for(int i = 0; i < g.size(); i++){        // in the weighted combined, there is only one objective (the weighted sum)
      weighted_sum += (g[i] * weight_set[i]);
    }
    f.push_back(weighted_sum + h);
  }

  // weighted conscious A* constructor
  Node(double x_, double y_, size_t id_, std::vector<double> g_, double h_, const std::vector<double> &weight_set, const int focus, NodePtr parent_ )
  : x(x_), y(y_), id(id_), g(g_), h(h_), f({}), parent(parent_){
    double focused_sum= 0;
    for(int i = 0; i < g.size(); i++){        // in the weighted combined, there is only one objective (the weighted sum)
      if(i == focus){                         // add the raw cost of the focused objective 
        focused_sum += g[i];
      } else {                                // add the weighted cost of the non-focused objectives
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

  friend std::ostream& operator<<(std::ostream &stream, const Node &node) {
    stream << std::fixed;
    stream << "{id: " << node.id << ", x: " << std::setprecision(4) << node.x << ", y: " << std::setprecision(4) << node.y <<  "}";

    return stream;
  }

  // void display(){
  //   std::cout << "{" << std::endl;
  //   std::cout << "  " << *this << std::endl;
  //   std::cout 
  //             << "  f-score: " << this->f << '\n'
  //             << "  g-score: " << this->g << '\n'
  //             << "  h      : " << this->h << '\n'
  //             << "}" << std::endl;
  // }
};

/**
 * Edge class
*/
struct Edge {
  size_t                source;
  size_t                target;
  std::vector<double>   cost;

  Edge(size_t source_, size_t target_, std::vector<double> cost_): source(source_), target(target_), cost(cost_) {}

  Edge inverse(){
    return Edge(this->target, this->source, this->cost);
  }
};

/**
 *  Adjacency Matrix Class
*/
class AdjMatrix {
  private:
  std::vector<std::vector<Edge>>   matrix;      // [i] will return a reference to a vector of edges with source i. (thus checking for a particular entry is O(|V|)) not O(1) which is the deifntion of a Adj Matrix)
  size_t                           graph_size;  // the number of nodes (vertices) in the graph
  size_t                           obj_count = 0;

  public:
  AdjMatrix() = default;

  AdjMatrix(size_t graph_size_, std::vector<Edge> &edges, bool inverse=false);

  void add(Edge e);

  size_t size() const;

  size_t edge_size() const;

  size_t get_obj_count() const;

  void jsonify(std::string FILENAME);

  const std::vector<Edge>& operator[](size_t vertex_id) const{
    return this->matrix.at(vertex_id);
  }
    
  friend std::ostream& operator<<(std::ostream &stream, const AdjMatrix &adj_matrix) {
      size_t  i = 0;
  
      stream << "{\n";
      for (auto vertex_iter = adj_matrix.matrix.begin(); vertex_iter != adj_matrix.matrix.end(); ++vertex_iter) {
          stream << "\t\"" << i++ << "\": [";
  
          std::vector<Edge> edges = *vertex_iter;
          for (auto edge_iter = edges.begin(); edge_iter != edges.end(); ++edge_iter) {
              stream << "\"" << edge_iter->source << "->" << edge_iter->target << "\", ";
          }
  
          stream << "],\n";
      }
      stream << "}";
      return stream;
  }  
};

//
// graph dependencies
//

using heuristic = std::function<double(const NodePtr&, const NodePtr&)>;
using node_order = std::function<bool(const NodePtr&, const NodePtr&)>;

// function used for ordering paths in an best first search open search by a single cost component
struct more_than_specific{
  const size_t cost_idx;

  more_than_specific(const size_t cost_idx_): cost_idx(cost_idx_){}

  bool operator()(const NodePtr &a, const NodePtr &b) const {
    return a->f[cost_idx] > b->f[cost_idx];
  }
};

struct h_functor {
  size_t state;

  h_functor(size_t ver): state(ver) {}

  double operator()(const NodePtr &a, const NodePtr &b){

    if(state == 0){           // EUCLIDEAN DISTANCE
      return sqrt(pow((a->x - b->x), 2) + pow((a->y - b->y), 2) );
      
    } else if ( state == 1){  // HAVERSINE DISTANCE (using kilometers)

      double Lata = a->x, //lat
             Latb = b->x, 
             Lnga = a->y, //lng
             Lngb = b->y;

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

//
// Graph Searching
//


class ASTAR{
  private:
  const AdjMatrix            &adj_matrix;
  std::vector<NodePtr>       &node_list;
  public:

  ASTAR(const AdjMatrix &adj_matrix_, std::vector<NodePtr> &node_list_);

  NodePtr operator()(const size_t source, const size_t target, heuristic &h, node_order &order) const;
  
  NodePtr operator()(const size_t source, const size_t target, heuristic &h, node_order &order, std::vector<size_t> &trace_back) const;
};


class WEIGHTED_ASTAR {
  private:
    const AdjMatrix       &adj_matrix;
    std::vector<NodePtr>  &node_list;
    std::vector<double>   weight_set;

  public:

  void update_weight(const std::vector<double> new_weight_set);
  
  WEIGHTED_ASTAR(const AdjMatrix &adj_matrix_, std::vector<NodePtr> &node_list_, std::vector<double> weight_set_);

  WEIGHTED_ASTAR(const AdjMatrix &adj_matrix_, std::vector<NodePtr> &node_list_);

  NodePtr operator()(const size_t source, const size_t target, heuristic &h, node_order &order) const;

  NodePtr operator()(const size_t source, const size_t target, heuristic &h, node_order &order, const int focus) const;
};

#endif
