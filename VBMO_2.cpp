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



#include "voting.hpp"
#include "utili.hpp"

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

// overide for edge oprator that prints it in JSON format
std::ostream& operator<<(std::ostream &stream, const Edge &edge) {
  stream
     << "{"
     <<  "\"edge_source\": " << edge.source << ", "
     <<  "\"edge_target\": " << edge.target << ", "
     <<  "\"edge_cost\": ";

  stream << "{" << edge.cost[0];
  for(int i = 1; i < edge.cost.size(); i++){
    stream << ", " << edge.cost[i];   
  }
  stream << "}";

  return stream;
}

// struct Edge {
//   size_t                source;
//   size_t                target;
//   std::vector<size_t>   cost;

//   Edge(size_t source_, size_t target_, std::vector<size_t> cost_): source(source_), target(target_), cost(cost_) {}

//   Edge inverse(){
//     return Edge(this->target, this->source, this->cost);
//   }
// };

// // overide for edge oprator that prints it in JSON format
// std::ostream& operator<<(std::ostream &stream, const Edge &edge) {
//   stream
//      << "{"
//      <<  "\"edge_source\": " << edge.source << ", "
//      <<  "\"edge_target\": " << edge.target << ", "
//      <<  "\"edge_cost\": ";

//   stream << "{" << edge.cost[0];
//   for(int i = 1; i < edge.cost.size(); i++){
//     stream << ", " << edge.cost[i];   
//   }
//   stream << "}";

//   return stream;
// }



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

  AdjMatrix(size_t graph_size_, std::vector<Edge> &edges, bool inverse=false)
  : matrix((graph_size_ + 1), std::vector<Edge>()), graph_size(graph_size_) { // graph_size + 1 is because vertices id's start at 1.
    obj_count = edges[0].cost.size();

    for(auto iter = edges.begin(); iter != edges.end(); ++iter){ // turn the edge list into an adacency list
      if(inverse) {
        this->add(iter->inverse());
      } else {
        this->add(*iter);
      }
    }      
  }

  void add(Edge e){
    (this->matrix[e.source]).push_back(e);
  }

  size_t size(void) const {
    return this->graph_size;
  }
  size_t get_obj_count() const{
     return obj_count;
  }
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

/**
 * Node (vertex) Class
*/
struct Node;

using NodePtr = std::shared_ptr<Node>;

// each node as a respective f, g, and h score for each objectice?
struct Node {
  size_t                id;
  std::vector<double>   g;  // the best current score from the source node for each objective cost ()


  
  //h originally had a vector of size_t, but for most applications, specifically graph searching, a spatial heuristic is sufficent
  double                h; // seperate heuristic for the objectives? only need one?
  std::vector<double>   f;
  NodePtr               parent;

  // interpreted as lng, lat. Or as x,y cordinate in the case of 
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
  // f: the current "guess" distance from the node to the end node, calcuated by g + h

  //comparitor where we want to look at the hueristic with respect to a specific cost 
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

struct pair_hash{
  template<class t1, class t2>
  size_t operator()(const std::pair<t1, t2>& p) const {
    auto h1 = std::hash<t1>{}(p.first);
    auto h2 = std::hash<t2>{}(p.second);
    if(h1 != h2){
      return h1 ^ h2;
    } else {
      return h1;
    }
  }
};


/**
 * functor for selecting which f-score to use for the open set for decomposing the multiobjective search problem  
*/
struct more_than_specific{
  const size_t cost_idx; 

  more_than_specific(const size_t cost_idx_): cost_idx(cost_idx_) {};
  
  bool operator()(const NodePtr &a, const NodePtr &b) const {
    return (a->f[cost_idx] > b->f[cost_idx]);
  }
};

// state can take 2 takes {0,1} for euclidean, or haversine distance.
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

// These will be the placeholder for the fuctor object that we for either for node ordersing in the open set, and to calculate the heuristic
using heuristic = std::function<double (const NodePtr&, const NodePtr&)>;
using node_order = std::function<bool(const NodePtr&, const NodePtr&)>;

//
// A* and weighted A* objects
//

class ASTAR{
  private:
  const AdjMatrix            &adj_matrix;
  const std::vector<NodePtr> &node_list;

  public:
  ASTAR(const AdjMatrix &adj_matrix_, const std::vector<NodePtr> &node_list_): adj_matrix(adj_matrix_), node_list(node_list_) {}

  // version that the paper uses. Not optimal?
  NodePtr operator()(const size_t source, const size_t target, heuristic &h, node_order &order){

    std::unordered_set<size_t> closed;    // the CLOSED set, incusuion <-> visted 

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
                                           next_h, 
                                           node);


        open.push_back(next);
        std::push_heap(open.begin(), open.end(), order);
      }
    }       // END MAIN LOOP

    //only reached if there is no path
    return nullptr;
  }

};

class WEIGHTED_ASTAR {
  private:
  const AdjMatrix             &adj_matrix;
  const std::vector<NodePtr>  &node_list;
  const std::vector<double>   &weight_set;

  public:

  WEIGHTED_ASTAR(const AdjMatrix &adj_matrix_, const std::vector<NodePtr> &node_list_, const std::vector<double> &weight_set_)  
  : adj_matrix(adj_matrix_), node_list(node_list_), weight_set(weight_set_) {}

  // WEIGHTED COMBINED VERSION
  NodePtr operator()(const size_t source, const size_t target, heuristic &h, node_order &order){

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
  NodePtr operator()(const size_t source, const size_t target, heuristic &h, node_order &order, const int focus){

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

};

/**
 * VBEA DRIVER 
 * @param: adj_matrix, the graph,
 * @param: node_list, a method for getting information about the vertext that is needed for the heuristic,
 * @param: h, hueristic functor,
 * @param: k, the number of candidates for the next generation
 * @param: source, targert, the start and end vertex (node) id of the graph search,
 * @param: voting_method, the voting method that is used for solution evaluation.
 */ 

// FIX Record the front after intial population, gen1, and gen2 along with winner
std::vector<std::vector<double>> VBEA(const AdjMatrix &adj_matrix, const std::vector<NodePtr> &node_list, heuristic &h, const size_t source, const size_t target, const voting_method vote_scheme){

  const int k = adj_matrix.get_obj_count();
  std::cout << "start: \n";
  std::cout << *node_list[source].get() << std::endl;
  std::cout << "end: \n";
  std::cout << *node_list[target].get() << std::endl;
  std::cout << "----------------------" << std::endl;

  // A* Object
  ASTAR A(adj_matrix, node_list);

  // INITIAL POPULATION (gen0)
  std::vector<std::vector<double>> results;
  int b = adj_matrix.get_obj_count();
  for(int i = 0; i < b; i++){
    std::cout << "runing A* on objective " << i << "... \n";
    auto Y = more_than_specific(i);
    node_order order(Y);
    results.push_back(A(source, target, h, order)->g);
  }

  // VOTING INITIAL POPULATION
  auto norm_results = normalize_matrix(results);
  auto d_scores = d_score(norm_results);
  std::cout << "Initial population: " << std::endl;
  for(int i = 0; i < norm_results.size(); i++){
      std::cout << "P_" << i << " " << norm_results[i] << " | " << results[i] << " d: " << d_scores[i] << std::endl;
  }
  std::cout << "Initial Front Sparcity: " << sparcity_metric(results) << std::endl;

  // DISPLAYING VOTING RESULTS
  std::cout << "Voting:" << std::endl;
  auto vote_results = vote(results, vote_scheme); // solutions index is displayed in order of "fitness" according to voting method.
  std::cout << vote_results << std::endl;

  // CREATING WEGIHT SET OF TOP k CANDAIATES
  std::vector<std::vector<double>> weight_sets;
  for(int i = 0; i < k && i < norm_results.size(); i++){
        weight_sets.push_back(complement_weight_set(norm_results[vote_results[i]]));
  }

 // FIRST CHILD POPULATION (Gen1)
  for(int i = 0; i < k && i < weight_sets.size(); i++){
    std::cout << "running weighted A* on weight set " << i << "..." << std::endl;
    WEIGHTED_ASTAR WA(adj_matrix, node_list, weight_sets[i]);
    auto Y = more_than_specific(0);        // The weighted combined f-score will be the only f-score, so use the first one
    node_order order(Y);
    // results.push_back(WA(source, target, h, order)->g);
    results.push_back(WA(source, target, h, order, i)->g);
  }

  // REMOVING DOMINATED SOLUTIONS FROM PARENT AND CHILD POPULATION
  std::cout << "fltering dominated solutions..." << std::endl;
  results = non_dominated_filter(results);  

  // NORMALIZING GEN1
  norm_results = normalize_matrix(results);
  d_scores = d_score(norm_results);

  // DISPALYING GEN1
  for(int i = 0; i < norm_results.size(); i++){
      std::cout << "P_" << i << " " << norm_results[i] << " | " << results[i] << " d: " << d_scores[i] << std::endl;
  }
  std::cout << "GEN1 Sparcity: " << sparcity_metric(results) << std::endl;


  // VOTING FOR SELECTION FOR 2nd GENERATION
  std::cout << "Voting round 2:" << std::endl;
  vote_results = vote(norm_results, vote_scheme);
  std::cout << vote_results << std::endl;
  
  std::vector<std::vector<double>> gen2, norm_gen2;  
  std::vector<double> gen2_d_scores;

  // CREATINGE GEN2

  // PICKING TOP k from gen1
  for(int i = 0; i < k && i < results.size(); i++){
    gen2.push_back(results[vote_results[i]]);
    norm_gen2.push_back(norm_results[vote_results[i]]);
  }

  weight_sets.clear();
  for(int i = 0; i < gen2.size(); i++){
    weight_sets.push_back(complement_weight_set(norm_gen2[i]));
  }  

  std::cout << "Generating Gen2 childen..." << std::endl;

  for(int i = 0; i < k && i < weight_sets.size(); i++){
    std::cout << "running weighted A* on weight set " << i  << "..." << std::endl;
    WEIGHTED_ASTAR WA(adj_matrix, node_list, weight_sets[i]);
    auto Y = more_than_specific(0);        // The weighted combined f-score will be the only f-score, so use the first one
    node_order order(Y);
    gen2.push_back(WA(source, target, h, order)->g);
  }

  // renormalize data
  norm_gen2 = normalize_matrix(gen2);
  gen2_d_scores = d_score(norm_gen2);

  
  for(int i = 0; i < gen2.size(); i++){
    std::cout << "P_" << i << " " << norm_gen2[i] << " | " << gen2[i] << " d: " << gen2_d_scores[i] << std::endl;
  }
  std::cout << "GEN2 Sparcity: " << sparcity_metric(gen2) << std::endl;

  vote_results = vote(norm_gen2, vote_scheme);

  std::cout << "GEN2 voting results: " << std::endl;
  std::cout << vote_results << std::endl;


  return gen2;

}




//
// I/O FUNCTIONS
//

void split_string(std::string string, std::string delimiter, std::vector<std::string> &results)
{
    size_t first_delimiter;

    while ((first_delimiter = string.find_first_of(delimiter)) != string.npos) {
        if (first_delimiter > 0) {
            results.push_back(string.substr(0, first_delimiter));
        }
        string = string.substr(first_delimiter + 1);
    }

    if (string.length() > 0) {
        results.push_back(string);
    }
}


bool load_gr_files(std::vector<std::string> gr_files, std::vector<Edge> &edges_out, size_t &graph_size){
  size_t          max_node_num = 0;
  for (auto gr_file: gr_files){
    std::ifstream file(gr_file.c_str());

    // std::cout << gr_file << std::endl;
    // continue;
    
    if (file.is_open() == false){
      std::cerr << "cannot open the gr file " << gr_file << std::endl;
      return false;
    }

    std::string line;
    int idx_edge = 0;
    while (file.eof() == false) {
        std::getline(file, line);

        if (line == "") {
            break;
        }

        std::vector<std::string> decomposed_line;
        split_string(line, " ", decomposed_line);

        std::string type = decomposed_line[0];
        if ((std::strcmp(type.c_str(),"c") == 0) || (std::strcmp(type.c_str(),"p") == 0)) {
            continue; //comment or problem lines, not part of the graph
        }

        if (std::strcmp(type.c_str(),"a") == 0) { //arc
          if (idx_edge < (int)edges_out.size()){
            if (
                (stoul(decomposed_line[1]) != edges_out[idx_edge].source) ||
                (stoul(decomposed_line[2]) != edges_out[idx_edge].target)) {
              // arc_sign src dest should be same in both files
              std::cerr << "file inconsistency" << std::endl;
              return false;
            }
            edges_out[idx_edge].cost.push_back(std::stoul(decomposed_line[3]));
          }else{
            Edge e(std::stoul(decomposed_line[1]),    //source
                   std::stoul(decomposed_line[2]),    //target
                   {std::stod(decomposed_line[3])});
            edges_out.push_back(e);
            max_node_num = std::max({max_node_num, e.source, e.target}); //update max node count
          }
        }
        idx_edge ++;
    }
    file.close();
  }
  graph_size = max_node_num;
  return true;
}



void getNodes_DOT(const std::string DOT_FILE, std::vector<NodePtr> &out_nodes){
  std::fstream ifs(DOT_FILE);
  out_nodes.clear();
  out_nodes.push_back(nullptr);     // id starts at 1;
  std::cout << "Node File: "<< DOT_FILE << std::endl;
  
  std::string x;
  double a, b;
  size_t id;

  for(int i = 0; i < 4; i++){
    ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n' );
  }
  std::getline(ifs, x);  
  // std::cout << x << std::endl;

  int size = stoi(x.substr(12));
  // out_nodes.resize(size);

  ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n' );
  ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n' );

  while(ifs >> x){
    if(x == "v"){
      ifs >> id >> a >> b;
      out_nodes.push_back(std::make_shared<Node>(b / 1000000, a / 1000000, id));      
    } else {
      std::cout << "inconsistancy in file, aborting. \n";
      break;
    }
  }

  ifs.close();
}

//
// Requires some motification to be usbale for the A*pex and other similar algorithms (or make them compatable with doubles?)
// 

/**
 * 
*/
void getNodes_ASCII(const std::string ASCII_FILE, std::vector<NodePtr> &out_nodes, AdjMatrix &adj_matrix){
  
  std::fstream ifs(ASCII_FILE);
  std::string temp;
  int height;
  int width;

  //get height and width of the graph
  std::getline(ifs, temp);
  std::getline(ifs, temp);
  height = stoi(temp.substr(7));
  std::getline(ifs, temp);
  width = stoi(temp.substr(6));
  std::getline(ifs, temp);

  // std::cout << "h: " << height << " w: " << width << std::endl;


  size_t id = 0; // we can start at index 0 for ASCII since id's are not predetermined
  double x, y;
  
  std::vector<std::vector<char>> map(height, std::vector<char>(width));

  // Converting the text file into a matrix (map)
  // need a map from x,y cordinates to the id for constructing edges
  std::unordered_map<std::pair<double,double>, size_t, pair_hash> cordinateIDmap;
  for(int i = 0; i < height; i++){
    std::getline(ifs, temp);
    for(int j = 0; j < width; j++){
      map[i][j] = temp[j];
      if(map[i][j] == '.'){
        out_nodes.push_back(std::make_shared<Node>(i, j, id));     // create the nodes with the x,y cordinate of the grid, and with a unique id provided by the counter
        cordinateIDmap[std::make_pair(i,j)] = id;
        id++;
      }
    }
  }
  // std::cout << out_nodes.size() << " " << id << std::endl;


  // // cross refercing count, the id - 1 (the last recordede ID used) should be the same as the size of both out_nodes and cordinateMap
  // std::cout << "last id:   " << (id - 1) << std::endl
  //           << "edge list: " << out_nodes.size() << std::endl
  //           << "cord map:  " << cordinateMap.size() << std::endl;

  /* Second pass: constructing the adjaceny matrix adding the following objective
  *  - Euclidean distance 
  *  - Uniform (1)
  *  - Random [0,20]
  *  - Danger (1.5, 10)
  *  - Safety (def)
  */

  std::vector<Edge> Edges;

  std:std::mt19937 RNG(96534);

  //adding edge cost to all eight neighbors
  for(double i = 0; i < height; i++){
    for(double  j = 0; j < width; j++){
      //check all 8 neighbors checking the neighbor
      if(map[i][j] == '.'){
        //add all 8 neighbors 
        if(i-1 >= 0 && map[i-1][j] == '.'){                      // up 
          
          Edges.push_back(Edge(cordinateIDmap[{i,j}], 
                               cordinateIDmap[{i-1,j}], {1, 1, double(RNG()%20)}));
        }
        if(i-1 >= 0 && j + 1 < width && map[i-1][j+1] == '.'){   // up right
          Edges.push_back(Edge(cordinateIDmap[{i,j}], 
                               cordinateIDmap[{i-1,j+1}], {1, 1.5, double(RNG()%20)}));
        }
        if(j+1 < width && map[i][j+1] == '.'){                   // right
          Edges.push_back(Edge(cordinateIDmap[{i,j}], 
                               cordinateIDmap[{i,j+1}], {1, 1, double(RNG()%20)}));
        }
        if(i+1 < height && j+1 < width && map[i+1][j+1] == '.'){ // down right
          Edges.push_back(Edge(cordinateIDmap[{i,j}], 
                               cordinateIDmap[{i+1,j+1}], {1, 1.5, double(RNG()%20)}));
        }
        if(i+1 < height && map[i+1][j] == '.'){                  // down
          Edges.push_back(Edge(cordinateIDmap[{i,j}], 
                               cordinateIDmap[{i+1,j}], {1, 1, double(RNG()%20)}));
        }
        if(i+1 < height && j-1 >= 0 && map[i+1][j-1] == '.'){    // down left
          Edges.push_back(Edge(cordinateIDmap[{i,j}], 
                               cordinateIDmap[{i+1,j-1}], {1, 1.5, double(RNG()%20)}));
        }
        if(j-1 >= 0 && map[i][j-1] == '.'){                      // left
          Edges.push_back(Edge(cordinateIDmap[{i,j}], 
                               cordinateIDmap[{i,j-1}], {1, 1, double(RNG()%20)}));
        }
        if(i-1 >= 0 && j-1 >= 0 && map[i-1][j-1] == '.'){        // up left
          Edges.push_back(Edge(cordinateIDmap[{i,j}], 
                               cordinateIDmap[{i-1,j-1}], {1, 1.5, double(RNG()%20)}));
        }
      } else {
        continue;
      }
    }
  }


  // Similar to adj list, but the node id (index) will correspond to a list of all edges to go into that id(index)
  std::vector<std::vector<Edge>> tempMap(id, std::vector<Edge> {});  

  for(auto iter = Edges.begin(); iter != Edges.end(); iter++){
    tempMap[iter->target].push_back(*iter);
  }

  // randomly select 10% of edges and make all incoming edges expensive
  for(auto iter = tempMap.begin(); iter != tempMap.end(); iter++){
    double n;
    if(RNG() % 10 == 0){
      n = 10;
    } else {
      n = 1.5;
    }   
    for(auto jter = iter->begin(); jter != iter->end(); jter++){
      jter->cost.push_back(n);
    }
  }

  // return to edge list
  Edges.clear();
  for(auto iter = tempMap.begin(); iter != tempMap.end(); iter++){
    Edges.insert(Edges.end(), iter->begin(), iter->end());
  }

  // another adjaceny matrix but for incoming edges
  tempMap.clear();
  tempMap = std::vector<std::vector<Edge>>(id, std::vector<Edge> {});  
  for(auto iter = Edges.begin(); iter != Edges.end(); iter++){
    tempMap[iter->source].push_back(*iter);
  }

  // adding safett objective
  for(int i = 0; i < tempMap.size(); i++){
    for(int j = 0; j < tempMap[i].size(); j++){
      tempMap[i][j].cost.push_back( (10 - tempMap[i].size() + 10 - tempMap[j].size()) / 2.0);
    }
  }

  // Convert back to vector of Edges
  Edges.clear();
  for(auto iter = tempMap.begin(); iter != tempMap.end(); iter++){
    Edges.insert(Edges.end(), iter->begin(), iter->end());
  }

  // for(auto i = Edges.begin(); i != Edges.end(); i++){
  //   std::cout << *i << std::endl;
  // }
  
  // std::cout << out_nodes.size() << std::endl;

  adj_matrix = AdjMatrix(id, Edges);

}

bool ASCII_MAP_RUNNER(const std::string MAP_FILE, voting_method voting_scheme, const int k){
    std::vector<NodePtr> node_list;
    AdjMatrix adj_matrix;
    std::mt19937 rng(std::clock());

    std::cout << "constructing graph for " << MAP_FILE << std::endl;
    getNodes_ASCII(MAP_FILE, node_list, adj_matrix);

    // size_t source = rng() % adj_matrix.size(), target = rng() % adj_matrix.size(); // randomly select 
    size_t source = 64, target = 14;
  
    // std::cout << "node count: " << node_list.size() << " adj_matrox size: " << adj_matrix.size() << std::endl;
    auto X = h_functor(0);     // set heuristic function to euclidean
    heuristic h(X);

    VBEA(adj_matrix, node_list, h, source, target, voting_scheme);
    return true;
}

// map_instnaces maps to files 
void ASCII_MAP_INSTANCE_MAKER(const std::string MAP_FILE, std::unordered_map<std::string, std::vector<std::vector<size_t>>> &map_instances){
  std::vector<NodePtr> node_list;
  AdjMatrix adj_matrix;
  std::mt19937 rng(std::clock());

  getNodes_ASCII(MAP_FILE, node_list, adj_matrix);

  ASTAR A(adj_matrix, node_list);
  auto X = h_functor(0);
  heuristic h(X);

  auto Y = more_than_specific(0);
  node_order order(Y);
  int count = 0;

  // return;

  while(count < 40){
    size_t source = rng() % adj_matrix.size(), target = rng() % adj_matrix.size(); // randomly select start and end location
    if(A(source, target, h, order) == nullptr){ // if no path connecting the two exists
      continue; // redo
    } else{
      count++;  // increment count & record
      map_instances[MAP_FILE].push_back({source, target});
    }
  }

}

// driver code 
void make_ASCII_instances(const std::string dir, std::unordered_map<std::string, std::vector<std::vector<size_t>>> &map_instances){

  for(auto &file_iter: std::filesystem::directory_iterator(dir)){
    std::string MAP_FILE = file_iter.path().relative_path().string();
    std::cout << MAP_FILE << std::endl;
    map_instances[MAP_FILE];
    ASCII_MAP_INSTANCE_MAKER(MAP_FILE, map_instances);
    // break;
  }

}


/**
 * @param: dir is the directory name of the files we wish to go over.
*/
void ASCII_DIR_RUNNER(const std::string dir, voting_method voting_scheme, const int k){
  for(auto &file_iter: std::filesystem::directory_iterator(dir)){
    std::string MAP_FILE = file_iter.path().relative_path().string();
    // std::cout << MAP_FILE << std::endl;
    ASCII_MAP_RUNNER(MAP_FILE, voting_scheme, k);
    break;
  
  }
}




void DOT_RUNNER(voting_method voting_scheme, const int k){
  // I/O
  std::string place;
  std::cout << "BAY, COL, FLA, NY: "; std::cin >> place;
  std::string dir = "USA-road/USA-road-" + place;
  std::ifstream ifs;
  std::string x; 

  
  std::vector<std::string> files;
  for(auto file_iter : std::filesystem::directory_iterator(dir)){
      files.push_back(file_iter.path());
  }

  //getting the file that has the nodes (the one that ends with the .co exstention)
  std::string node_file;
  for(auto file = files.begin(); file != files.end(); file++){
    if(file->substr(file->size()-2) == "co"){
      node_file = *file;
      files.erase(file);
      break;
    }
  }

  for(auto &f: files){
    std::cout << f << std::endl;
  }
  std::cout << node_file << std::endl;

  std::vector<Edge> edges;
  size_t graph_size;

  std::cout << "generating graph..." << std::endl;
  if(!load_gr_files(files, edges, graph_size )){
    std::cout << "Error reading .gr files" << std::endl;
  }

  // NODE LIST
  std::vector<NodePtr> node_list;
  std::cout << "creating node list..." << std::endl;
  getNodes_DOT(node_file, node_list);

  // VERIFYING
  // std::cout << graph << std::endl;
  std::cout << "graph size:     " <<  graph_size << " , edge size: " << edges.size() << std::endl;
  std::cout << "node list size: " << node_list.size()-1 << std::endl;   // account for the 0th index which has no node associated with it as node id start at 1

  AdjMatrix adj_matrix(graph_size, edges);

  // h_functor is set to 1 for Haversine distance (km), use 0 for euclidean distance
  auto X = h_functor(1);
  heuristic h(X);

  std::mt19937 rng(std::clock());
  // more_than_specific is used to determin which f-score is used for the open set,  
  size_t source = 20002, target = 164983;
  // size_t source = rng() % adj_matrix.size(), target = rng() % adj_matrix.size();
    
  VBEA(adj_matrix, node_list, h, source, target, voting_scheme);
}


// FIX need a method for reading the text file, and all of the set start and end locations
std::unordered_map<std::string, std::vector<std::vector<size_t>>> get_ASCII_instances(const std::string FILE){
  std::unordered_map<std::string, std::vector<std::vector<size_t>>> instances;

  std::ifstream ifs(FILE);

  std::string map;
  size_t source, target;

  while(ifs >> map){
    for(int i = 0; i < 40; i++){            
      ifs >> source >> target;
      instances[map].push_back({source, target});
    }
  }


  return instances;
}


// instances are in seperate files
void get_DOT_instances(const std::string FILE){
  
}
// main for dao maps, change dir to the name of 
// int main(){
//   std::string dir = "dao-map";
//   ASCII_DIR_RUNNER(dir, voting_method::borda, 4);

//   // DOT_RUNNER(voting_method::borda,3);
// }

//testing reading instances

struct record{
  std::string                      voting_method;      // range, borda, concorcet, etc..
  std::string                      generations;        // 1-2
  std::string                      file_name;
  std::string                      child_method = "weighted_conscious";       //either weighted combined or conscious
  std::vector<std::vector<double>> front;
  record(const std::string file_name_, const enum::voting_method vote_method, const std::vector<std::vector<double>> &front_)
  :file_name(file_name_), front(front_){
    if(vote_method == voting_method::borda){
        voting_method = "borda";
    } else if(vote_method == voting_method::combined_approval){
        voting_method = "combined_approval";
    } else if(vote_method == voting_method::range){
        voting_method = "range";
    } else {
      // voting_method::condornet
        voting_method = "condornet";
    }
  }
};

void write_array(std::ostream &out_file, const std::vector<double> &vec){
  
}

void write_record(std::ostream &out_file, const record &r){
  out_file << "\t\"" << r.file_name << "\"";
  
            
}



// all the records will be avereged in python, this is just to write it in json format

void write_all_records(const std::vector<record> &rec, std::string file_name){
  std::ofstream out_file(file_name + ".json");

  out_file << "{" << std::endl;

  for(auto r : rec){
    write_record(out_file, r);
  }

  out_file << "{";

  out_file.close();
  
}





std::vector<record> ASCII_instance_runner(const enum::voting_method vote_method, const std::unordered_map<std::string, std::vector<std::vector<size_t>>> &instances){

  std::vector<record> LOG;
  
  for(auto &inst : instances){ // for each map 
    //create a the map once
    std::vector<NodePtr> node_list;
    AdjMatrix adj_matrix;

    auto x = h_functor(0);
    heuristic h = x;
    getNodes_ASCII("dao-map/" + inst.first, node_list, adj_matrix);
    for(auto param : inst.second){ // for each valid source and target position (there will be 40)
      // get source and target
      size_t source = param[0],
             target = param[1];
      // run VBEA with param 
      // Independent variable to incorperate
      // 1. combined or conscious
      // 2. number of generations (1, 2)
      auto front = VBEA(adj_matrix, node_list, h, source, target, vote_method);
      LOG.push_back({inst.first, vote_method, front});
    }
    break;
  }

  return LOG;
}

  
int main(){
  auto instances = get_ASCII_instances("./DAO-instances_40.txt");
  ASCII_instance_runner(voting_method::borda, instances);

}


// // main for dao maps, change dir to the name of 
// int main(){
//   std::string dir = "dao-map";
//   std::unordered_map<std::string, std::vector<std::vector<size_t>>> map_instances;
//   make_ASCII_instances(dir, map_instances);

//   // for(auto &iter : map_instances){
//   //   std::cout << iter.first << std::endl;
//   //   for(auto &jter : iter.second){
//   //     std::cout << "  " << jter[0] << " " << jter[1] << std::endl;
//   //   }
//   //   std::cout << "----------" << std::endl;
//   // }


//   std::ofstream out_file("./DAO-instances_40.txt");
  
//   for(auto &iter : map_instances){
//     out_file << iter.first << '\n';
//     for(auto &jter : iter.second){
//       out_file << jter[0] << " " << jter[1] << '\n';
//     }
//   }

//   out_file.close();

//   return 0;
// }
