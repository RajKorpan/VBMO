#include <algorithm>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <ios>
#include <random>
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



#include "voting.hpp"
#include "utili.hpp"
// #include "logger.hpp"

/**
 * Edge class
*/
struct Edge {
  size_t                source;
  size_t                target;
  std::vector<double>   cost;
  double                velocity;

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

  size_t edge_size() const {
    size_t n = 0;
    for(auto &i: matrix){
      n += i.size();
    }    

    return n;
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

  void add_velocity(){
    for(int x = 0; x < matrix.size(); x++){
      for(int y = 0; y < matrix[x].size(); y++){
        matrix[x][y].velocity = std::max(sin(x-1), cos(y-1));
      }
    }
  }
};

/**
 * Node (vertex) Class
*/
struct Node;

using NodePtr = std::shared_ptr<Node>;

// each node hold the f, g, and heuristic cost
struct Node {
  size_t                id;
  std::vector<double>   g;  // the best current score from the source node for each objective cost ()
  
  //h originally had a vector of size_t, but for most applications, specifically graph searching, a spatial heuristic is admissable &|| consistant?
  double                h; // seperate heuristic for the objectives? only need one?
  std::vector<double>   f; // the estimated distnace from this node to the target
  NodePtr               parent; 

  // interpreted as lng, lat. Or as x,y cordinate in the case of 
  double                 x,y;


  // default constructor
  Node(): id(-1), g({}), h(0), f({}), parent(nullptr), x(-1.0), y(-1.0) {}

  // used for node list,
  Node(double x_, double y_, size_t id_)
  : x(x_), y(y_), id(id_) {}

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


/**
 * functor for selecting which f-score to use for the open set for decomposing the multiobjective search problem  
*/
struct more_than_specific{
  const size_t cost_idx; // the index that will be used for comparison in the open set

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

// Note that this version does not create a copy of the graph with the weighted update, rather an implicit update (the wighted sum is calculated only when they are need, i.e. when adding to the open set).
class WEIGHTED_ASTAR {
  private:
  const AdjMatrix             &adj_matrix;
  const std::vector<NodePtr>  &node_list;
  std::vector<double>   &weight_set;

  public:

  WEIGHTED_ASTAR(const AdjMatrix &adj_matrix_, const std::vector<NodePtr> &node_list_, std::vector<double> &weight_set_)  
  : adj_matrix(adj_matrix_), node_list(node_list_), weight_set(weight_set_) {}



  void update_weight_set(std::vector<double> &weight_set_){
    weight_set = weight_set_;
  }
  // WEIGHTED COMBINED VERSION
  NodePtr operator()(const size_t source, const size_t target, heuristic &h, node_order &order) const {

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
  NodePtr operator()(const size_t source, const size_t target, heuristic &h, node_order &order, const int focus) const {

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
  // std::cout << "Node File: "<< DOT_FILE << std::endl;
  
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
 * @param: out_nodes is the number of vertieces in the graph:
 * @param: adj_matrix is the 
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
  *  - Uniform (1)
  *  - Euclidean distance 
  *  - Random [1,20]
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
                               cordinateIDmap[{i-1,j}], {1, 1, 1+ double(RNG()%20)}));
        }
        if(i-1 >= 0 && j + 1 < width && map[i-1][j+1] == '.'){   // up right
          Edges.push_back(Edge(cordinateIDmap[{i,j}], 
                               cordinateIDmap[{i-1,j+1}], {1, 1.414, 1+ double(RNG()%20)}));
        }
        if(j+1 < width && map[i][j+1] == '.'){                   // right
          Edges.push_back(Edge(cordinateIDmap[{i,j}], 
                               cordinateIDmap[{i,j+1}], {1, 1, 1+ double(RNG()%20)}));
        }
        if(i+1 < height && j+1 < width && map[i+1][j+1] == '.'){ // down right
          Edges.push_back(Edge(cordinateIDmap[{i,j}], 
                               cordinateIDmap[{i+1,j+1}], {1, 1.414, 1+ double(RNG()%20)}));
        }
        if(i+1 < height && map[i+1][j] == '.'){                  // down
          Edges.push_back(Edge(cordinateIDmap[{i,j}], 
                               cordinateIDmap[{i+1,j}], {1, 1, 1+ double(RNG()%20)}));
        }
        if(i+1 < height && j-1 >= 0 && map[i+1][j-1] == '.'){    // down left
          Edges.push_back(Edge(cordinateIDmap[{i,j}], 
                               cordinateIDmap[{i+1,j-1}], {1, 1.414, 1+ double(RNG()%20)}));
        }
        if(j-1 >= 0 && map[i][j-1] == '.'){                      // left
          Edges.push_back(Edge(cordinateIDmap[{i,j}], 
                               cordinateIDmap[{i,j-1}], {1, 1, 1+ double(RNG()%20)}));
        }
        if(i-1 >= 0 && j-1 >= 0 && map[i-1][j-1] == '.'){        // up left
          Edges.push_back(Edge(cordinateIDmap[{i,j}], 
                               cordinateIDmap[{i-1,j-1}], {1, 1.414, 1+ double(RNG()%20)}));
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

  // adding safety objective
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



void RAND_ASCII_RUNNER(const std::string MAP_FILE, const std::string vote_scheme){
  std::vector<NodePtr> node_list;
  AdjMatrix adj_matrix;

  getNodes_ASCII(MAP_FILE, node_list, adj_matrix);

  auto X = h_functor(0);
  heuristic h = X;


  std::mt19937 rng(std::clock());

  size_t source = rng() % node_list.size(), target = rng() % node_list.size();

  // auto results = VBEA(adj_matrix, node_list, h, source, target, vote_scheme);
}

void ASCII_RUNNER(const std::string MAP_FILE, const size_t source, const size_t target, const std::string vote_scheme){
  std::vector<NodePtr> node_list;
  AdjMatrix adj_matrix;

  getNodes_ASCII(MAP_FILE, node_list, adj_matrix);

  auto X = h_functor(0);
  heuristic h = X;

  // auto results = VBEA(adj_matrix, node_list, h, source, target, vote_scheme);
}



void DOT_RUNNER(const std::string voting_scheme){
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
  // more_than_specific is used to determine which f-score is used for the open set,  
  size_t source = 20002, target = 164983;
  // size_t source = rng() % adj_matrix.size(), target = rng() % adj_matrix.size();
    
  // VBEA(adj_matrix, node_list, h, source, target, voting_scheme);
}


/**
 * FORMING INSTANCES 
*/

typedef std::unordered_map<std::string, std::vector<std::vector<size_t>>> inst;

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

// finds 40 source and target nodes that are reachable
void ASCII_MAP_INSTANCE_MAKER(const std::string MAP_FILE, std::unordered_map<std::string, std::vector<std::vector<size_t>>> &map_instances, const int num){
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

  while(count < num){
    size_t source = rng() % adj_matrix.size(), target = rng() % adj_matrix.size(); // randomly select start and end location
    if(A(source, target, h, order) == nullptr){ // if no path connecting the two exists
      continue; // redo
    } else{
      count++;  // increment count & record
      map_instances[MAP_FILE].push_back({source, target});
    }
  }

}

// Dir driver code for all files in a dir
void make_ASCII_instances(const std::string dir, std::unordered_map<std::string, std::vector<std::vector<size_t>>> &map_instances, const int num){

  for(auto &file_iter: std::filesystem::directory_iterator(dir)){
    std::string MAP_FILE = file_iter.path().relative_path().string();
    std::cout << MAP_FILE << std::endl;
    map_instances[MAP_FILE];
    ASCII_MAP_INSTANCE_MAKER(MAP_FILE, map_instances, num);
    // break;
  }

}

/**
 * @param mao
*/
void road_map_instance_maker(const std::string MAP_DIR, const int inst_count, const std::string FILE_NAME){
  std::vector<NodePtr> node_list;
  std::vector<Edge> edges;
  size_t graph_size;

  std::uniform_int_distribution<size_t> rng;
  std::default_random_engine generator;

  std::vector<std::string> files;
  for(auto file_iter : std::filesystem::directory_iterator(MAP_DIR)){
      files.push_back(file_iter.path());
  }

  //finding the file that has the nodes (the one that ends with the .co exstention) from the other files (.gr)
  std::string node_file;
  for(auto file = files.begin(); file != files.end(); file++){
    if(file->substr(file->size()-2) == "co"){
      node_file = *file;
      files.erase(file); //remove from other files
      break;
    }
  }
  
  std::cout << "Constructing Graph..." << std::endl;
  load_gr_files(files, edges, graph_size);
  getNodes_DOT(node_file, node_list);
  
  AdjMatrix adj_matrix(graph_size, edges);

  // h_functor is set to 1 for Haversine distance (km), use 0 for euclidean distance
  auto X = h_functor(1);
  heuristic h(X);

  ASTAR A(adj_matrix, node_list);

  auto Y = more_than_specific(0);
  node_order order(Y);

  std::vector<std::vector<size_t>> instances;

  for(int i = 0; i < inst_count; i++){
    size_t s = rng(generator) % adj_matrix.size(), t = rng(generator) % adj_matrix.size();

    if(A(s, t, h, order) == nullptr){
      i--;
      continue;
    } else {
      std::cout << "s: " << s << ", " << "t: " << t << " (" << i << "/" << inst_count << ")" << std::endl;
      std::vector<size_t> temp = {s, t};
      instances.push_back(temp);
    }
  }

  // Savin
  std::cout << "writting " << instances.size() << " instnaces" << std::endl;
  std::ofstream out_file(FILE_NAME + ".txt");

  out_file << "source target" << std::endl;
  for(auto &inst : instances){
    out_file << "v " << inst[0]<< " " << inst[1] << std::endl;
  }
  
  std::cout << "DONE" << std::endl;

  out_file.close();
}




/**
 * LOGGER
 */
      // struct::log a(map_name, source, target, vote_method);
      // VBEA(adj_matrix, node_list, h, source, target, vote_method, a);


struct log{
  std::string     voting_method,      // range, borda, concorcet, etc..
                  file_name,
                  child_method = "weighted_combined",       //either weighted combined or conscious
                  source,
                  target;

  std::vector<std::vector<std::vector<double>>> fronts, 
                                                norm_fronts;

  std::vector<std::vector<double>>              raw_d_scores,
                                                norm_d_scores,
                                                winner,
                                                norm_winner; // keeps track of the winning candidate for each generation

  std::vector<int>                              run_times;
  std::vector<double>                           winner_raw_d_score,
                                                winner_norm_d_score;

  // all other data members are written during run time
  log(const std::string file_name_, const size_t source_, const size_t target_, const std::string voting_method_)
  :file_name(file_name_), fronts({}), source(std::to_string(source_)), voting_method(voting_method_), target(std::to_string(target_)) {}
};

void write_array(std::ostream &out_file, const std::vector<double> &vec){

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


/**
 * VBEA DRIVER 
 * @param: adj_matrix, the graph,
 * @param: node_list, a method for getting information about the vertext that is needed for the heuristic,
 * @param: h, hueristic functor,
 * @param: k, the number of candidates for the next generation
 * @param: source, targert, the start and end vertex (node) id of the graph search,
 * @param: voting_method, the voting method that is used for solution evaluation.
 */ 

// void VBEA(const AdjMatrix &adj_matrix, const std::vector<NodePtr> &node_list, heuristic &h, const size_t source, const size_t target, const std::string vote_scheme, struct::log &LOG){

//   std::vector<std::vector<std::vector<double>>> FRONTS; // will hold snapshots of the fronts of gen0, 1, and 2 (after removing duplicates and filtering non-dominated solutions)
//   const int k = adj_matrix.get_obj_count();
//   // Start and end location
//   std::cout << "start: \n";
//   std::cout << *node_list[source].get() << std::endl;
//   std::cout << "end: \n";
//   std::cout << *node_list[target].get() << std::endl;
//   std::cout << "----------------------" << std::endl;


//   auto start_t = std::chrono::high_resolution_clock::now();


//   // A* Object
//   ASTAR A(adj_matrix, node_list);

//   // FORMING INITIAL POPULATION (gen0)
//   std::vector<std::vector<double>> results;
//   int b = adj_matrix.get_obj_count();
//   for(int i = 0; i < b; i++){
//     // std::cout << "runing A* on objective " << i << "... \n"; // VISUAL
//     auto Y = more_than_specific(i);
//     node_order order(Y);
//     results.push_back(A(source, target, h, order)->g);
//   }

//   // NORMALIZING GEN0
//   results = remove_duplicate(results);  // Optional, solutions 0 and 1 often are the same!
//   auto norm_results = normalize_matrix(results);
//   auto d_scores = d_score(norm_results);

//   // VOTING RESULTS
//   auto vote_results = vote(norm_results, d_scores,  vote_scheme); // solutions index is displayed in order of "fitness" according to voting method.
//   // std::cout << vote_results << std::endl; // VISUAL

//   int t0 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();
//   FRONTS.push_back(results);

  
//   // SHOW GEN0
//   std::cout << "-----gen0-----" << std::endl;
//   for(int i = 0; i < norm_results.size(); i++){
//       std::cout << "P_" << i << " " << norm_results[i] << " | " << results[i] << " d: " << d_scores[i] << std::endl;
//   }

//   start_t = std::chrono::high_resolution_clock::now();

//   // CREATING WEGIHT SET OF TOP k CANDAIATES
//   std::vector<std::vector<double>> weight_sets;
//   for(int i = 0; i < k && i < norm_results.size(); i++){
//         weight_sets.push_back(complement_weight_set(norm_results[vote_results[i]]));
//   }

//  // FIRST CHILD POPULATION (Gen1)
//   for(int i = 0; i < k && i < weight_sets.size(); i++){
//     // std::cout << "running weighted A* on weight set " << i << "..." << std::endl; // VISUAL
//     WEIGHTED_ASTAR WA(adj_matrix, node_list, weight_sets[i]);
//     auto Y = more_than_specific(0);        // The weighted combined f-score will be the only f-score, so use the first one
//     node_order order(Y);
//     // results.push_back(WA(source, target, h, order)->g); // weighted combined
//     results.push_back(WA(source, target, h, order, i)->g); // weighted conscious, the 
//   }

//   // remove duplicates and dominated solutions
//   results = remove_duplicate(results);
//   results = non_dominated_filter(results);  

//   // NORMALIZING GEN1
//   norm_results = normalize_matrix(results);
//   d_scores = d_score(norm_results);

//   vote_results = vote(norm_results, d_scores, vote_scheme);

 
//   int t1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();
//   FRONTS.push_back(results);
//  // DISPALYING GEN1
//   std::cout << "-----gen1-----" << std::endl;
//   for(int i = 0; i < norm_results.size(); i++){
//       std::cout << "P_" << i << " " << norm_results[i] << " | " << results[i] << " d: " << d_scores[i] << std::endl;
//   }
//   std::cout << vote_results << std::endl; // VISUAL

//   std::vector<std::vector<double>> gen2, norm_gen2;  
//   std::vector<double> gen2_d_scores;

//   start_t = std::chrono::high_resolution_clock::now();

//   // SELECTING TOP K=5 paths form prev generation with respect to voting 
//   for(int i = 0; i < k && i < results.size(); i++){
//     gen2.push_back(results[vote_results[i]]);
//     norm_gen2.push_back(norm_results[vote_results[i]]);
//   }

//   // WEIGHT SET
//   weight_sets.clear();
//   for(int i = 0; i < gen2.size(); i++){
//     weight_sets.push_back(complement_weight_set(norm_gen2[i]));
//   }  

//   // CREATING GEN2
//   for(int i = 0; i < k && i < weight_sets.size(); i++){
//     // std::cout << "running weighted A* on weight set " << i  << "..." << std::endl; // VISUAL 
//     WEIGHTED_ASTAR WA(adj_matrix, node_list, weight_sets[i]);
//     auto Y = more_than_specific(0);        // The weighted combined f-score will be the only f-score
//     node_order order(Y);
//     gen2.push_back(WA(source, target, h, order)->g);
//   }

  
//   gen2 = remove_duplicate(gen2);
//   gen2 = non_dominated_filter(gen2);  

//   // Normalize data
//   norm_gen2 = normalize_matrix(gen2);
//   gen2_d_scores = d_score(norm_gen2);


//   vote_results = vote(norm_gen2, d_scores, vote_scheme);
//   int t2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();
//   FRONTS.push_back(gen2);

//   // DISPLAY GEN2  
//   std::cout << "-----gen2-----" << std::endl;
//   for(int i = 0; i < gen2.size(); i++){
//     std::cout << "P_" << i << " " << norm_gen2[i] << " | " << gen2[i] << " d: " << gen2_d_scores[i] << std::endl;
//   }
//   std::cout << vote_results << std::endl; // VISUAL


// }


void write_matrix(std::ostream &out_file, const std::vector<std::vector<double>> &matrix){
  if(matrix.empty()){
    out_file << "[[]]";
  } else {
    out_file << "["; write_array(out_file, matrix[0]);
    for(int i = 1; i < matrix.size(); i++){
      out_file << ", "; write_array(out_file, matrix[i]);
    }

    out_file << "]";
  }
}

int max_i(const std::vector<double> &vec){
  int j;
  for(int i = 0; i < vec.size(); i++){
    if(vec[i] > vec[j]){
      j = i;
    }
  }

  return j;
}

/**
 * VBEA DRIVER 
 * @param: adj_matrix, the graph,
 * @param: node_list, a method for getting information about the vertext that is needed for the heuristic,
 * @param: h, hueristic functor,
 * @param: k, the number of candidates for the next generation. 1 <= k <= j (number of objectives in the enviornment)
 * @param: T, the number of generations. T >= 0
 * @param: source, targert, the start and end vertex (node) id of the graph search,
 * @param: voting_method, the voting method that is used for solution evaluation.
 */ 

void VBEA(const AdjMatrix &adj_matrix, const std::vector<NodePtr> &node_list, heuristic &h, const size_t source, const size_t target, const std::string vote_scheme, const int T, int K,  struct::log &LOG){

  // if(K > adj_matrix.get_obj_count()){
  //   std::cout << "WARNING: " << K << ">" << "j" << std::endl << "setting k = j" << std::endl;
  //   K = adj_matrix.get_obj_count();
  // }

  K = 100; // will stop before if there are not enougt weight sets
 
  // std::cout << "start: \n";
  // std::cout << *node_list[source].get() << std::endl;
  // std::cout << "end: \n";
  // std::cout << *node_list[target].get() << std::endl;
  // std::cout << "----------------------" << std::endl;

  auto start_t = std::chrono::high_resolution_clock::now();
  // auto start_t = std::clock();

  // A* Object
  ASTAR A(adj_matrix, node_list);

  // FORMING INITIAL POPULATION (gen0)
  std::vector<std::vector<double>> results; // will hold the combined child and parent population
  int b = adj_matrix.get_obj_count();
  for(int i = 0; i < b; i++){
    auto Y = more_than_specific(i);
    node_order order(Y);
    results.push_back(A(source, target, h, order)->g);
  }

  // NORMALIZING AND CLEAN GEN0
  results = remove_duplicate(results);  // Optional, solutions for the decomposied problems for obkectives 0 and 1 often are the same, especially for instances where distance is nearby
  auto norm_results = normalize_matrix(results);
  auto d_scores = d_score(norm_results);

  // VOTING 
  auto vote_results = vote(norm_results, d_scores, vote_scheme); // solutions index is displayed in order of "fitness" according to voting method.

  // Logging

  LOG.run_times.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count());

  // LOG.run_times.push_back(std::clock() - start_t);
  // std::cout << ((double) LOG.run_times.back() / CLOCKS_PER_SEC ) * 1000 << std::endl;

  LOG.fronts.push_back(results);
  LOG.norm_fronts.push_back(norm_results);
  
  auto raw_d_score = d_score(results);
  LOG.raw_d_scores.push_back(raw_d_score);
  LOG.norm_d_scores.push_back(d_scores);
  
  LOG.winner.push_back(results[vote_results[0]]);
  LOG.norm_winner.push_back(norm_results[vote_results[0]]);
  LOG.winner_raw_d_score.push_back(raw_d_score[vote_results[0]]);
  LOG.winner_norm_d_score.push_back(d_scores[vote_results[0]]);

  // Displaying gen 0
  std::cout << "-----gen0-----" << std::endl;
  for(int i = 0; i < norm_results.size(); i++){
      std::cout << "P_" << i << " " << norm_results[i] << " | " << results[i] << " d: " << d_scores[i] << std::endl;
  }
  std::cout << "voting results:" << std::endl;
  std::cout << vote_results << std::endl; // VISUAL

  // What is not local to the loop
  // - results
  // - norm_results
  // - d_scores
  // - vote_reults
  //

  // main loop
  for(int t = 0; t < T; t++){
    start_t = std::chrono::high_resolution_clock::now();
    // start_t = std::clock();

    // Creating the weight set form the top K solution from the previous generation 
    std::vector<std::vector<double>> weight_sets;
    for(int i = 0; i < K && i < norm_results.size(); i++){
      weight_sets.push_back(complement_weight_set(norm_results[vote_results[i]]));  // pick the top K candidates normalized cost to create weight sets
    }
    std::vector<std::vector<double>> temp;           // Save only up too the top K result from the previous generation
    for(int i = 0; i < K && i < results.size(); i++){
      temp.push_back(results[vote_results[i]]);
    }
    std::swap(temp,results);


    // creating the child population
    for(int i = 0; i < K && i < weight_sets.size(); i++){
      // the focus is which objective has the highest weighted, but what if there are multiple max's?
      WEIGHTED_ASTAR WA(adj_matrix, node_list, weight_sets[i]);
      auto Y = more_than_specific(0); // the new objective will be the only objective for the search.
      node_order order(Y);

      // select the focus to be the lowest number:
      int j = max_i(weight_sets[i]);
      // results.push_back(WA(source, target, h, order, j)->g);    // conscious
      results.push_back(WA(source, target, h, order)->g);    // combined
    }

    // remove duplicates and dominates dolutions from the front
    results = remove_duplicate(results);
    results = non_dominated_filter(results);

    // voting
    norm_results = normalize_matrix(results);
    d_scores = d_score(norm_results);
    vote_results = vote(norm_results, d_scores, vote_scheme);

    // logging generation
    LOG.run_times.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count());
    // LOG.run_times.push_back(std::clock() - start_t);
    // std::cout << ((double) LOG.run_times.back() / CLOCKS_PER_SEC ) * 1000 << std::endl;


    LOG.fronts.push_back(results);
    LOG.norm_fronts.push_back(norm_results);

    auto raw_d_score = d_score(results);
    LOG.raw_d_scores.push_back(raw_d_score);
    LOG.norm_d_scores.push_back(d_scores);

    LOG.winner.push_back(results[vote_results[0]]);
    LOG.norm_winner.push_back(norm_results[vote_results[0]]);
    LOG.winner_raw_d_score.push_back(raw_d_score[vote_results[0]]);
    LOG.winner_norm_d_score.push_back(d_scores[vote_results[0]]);

    // SHOW GEN i
    std::cout << "-----gen" << t + 1 << "-----" << std::endl;
    for(int i = 0; i < norm_results.size(); i++){
        std::cout << "P_" << i << " " << norm_results[i] << " | " << results[i] << " d: " << d_scores[i] << std::endl;
    }
    std::cout << "winner:" << std::endl;
    std::cout << results[vote_results[0]] << " d: " << d_scores[vote_results[0]] << " r_d: " << path_d_score(results[vote_results[0]]) << std::endl;
    
  } // end main loop;
  return ;
}


void write_record(std::ostream &out_file, const struct::log &r) {
  // std::cout << "  "; std::cout << "{"; std::cout << std::endl;
  
  // std::cout << "  "; std::cout << "\"map-id\": " << "\"" << r.file_name << "\""; std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"source\": " << r.source; std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"target\": " << r.target; std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"voting-mechanism\": " << "\"" << r.voting_method << "\""; std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"child-generation-method\": " << "\"" << r.child_method << "\""; std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;
  out_file << "{";
  
  out_file << "\"map-id\": " << "\"" << r.file_name << "\"";
  out_file << ", ";

  out_file << "\"source\": " << r.source;
  out_file << ", ";

  out_file << "\"target\": " << r.target;
  out_file << ", ";

  out_file << "\"voting-mechanism\": " << "\"" << r.voting_method << "\"";
  out_file << ", ";

  out_file << "\"child-generation-method\": " << "\"" << r.child_method << "\"";
  out_file << ", ";

// FRONTS
  
  // std::cout << "  "; std::cout << "\"gen0-front\": " << r.fronts[0]; std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"gen0-norm-front\": " << r.norm_fronts[0]; std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"gen0-d-score\": " << r.all_d_scores[0]; std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"gen0-norm-d-score\": " << r.all_norm_d_scores[0]; std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"gen0-sparsity\": " << sparsity_metric(r.fronts[0]); std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"gen0-winner\": " << r.winners[0]; std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"gen0-time\": " << r.run_times[0]; std::cout << std::endl;


  out_file << "\"gen0-front\": "; write_matrix(out_file, r.fronts[0]);
  out_file << ", ";

  out_file << "\"gen0-norm-front\": "; write_matrix(out_file, r.norm_fronts[0]);
  out_file << ", ";

  out_file << "\"gen0-raw-d-score\": "; write_array(out_file, r.raw_d_scores[0]);
  out_file << ", ";

  out_file << "\"gen0-norm-d-score\": "; write_array(out_file, r.norm_d_scores[0]);
  out_file << ", ";

  out_file << "\"gen0-sparsity\": " << sparsity_metric(r.fronts[0]);
  out_file << ", ";

  out_file << "\"gen0-raw-winner\": "; write_array(out_file, r.winner[0]);
  out_file << ", ";

  out_file << "\"gen0-norm-winner\": "; write_array(out_file, r.norm_winner[0]);
  out_file << ", ";

  out_file << "\"gen0-winner-naw-d-score\": " <<  r.winner_raw_d_score[0];
  out_file << ", ";
  
  out_file << "\"gen0-winner-norm-d-score\": " << r.winner_norm_d_score[0];
  out_file << ", ";

  out_file << "\"gen0-time\": " << r.run_times[0];


  // for(int i = 1; i < r.fronts.size(); i++){
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;
  
  // std::cout << "  "; std::cout << "\"gen" << i << "-front\": " << r.fronts[i]; std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"gen" << i << "-norm-front\": " << r.norm_fronts[i]; std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"gen" << i << "-d-score\": " << r.all_d_scores[i]; std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"gen" << i << "-norm-d-score\": " << r.all_norm_d_scores[i]; std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"gen" << i << "-sparsity\": " << sparsity_metric(r.fronts[i]); std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"gen" << i << "-winner\": " << r.winners[i]; std::cout << std::endl;
  // std::cout << "  "; std::cout << ", "; std::cout << std::endl;

  // std::cout << "  "; std::cout << "\"gen" << i << "-time\": " << r.run_times[i]; std::cout << std::endl;

  // }
  for(int i = 1; i < r.fronts.size(); i++){
    out_file << ", ";
  
    out_file << "\"gen" << i << "-front\": "; write_matrix(out_file, r.fronts[i]);
    out_file << ", ";

    out_file << "\"gen" << i << "-norm-front\": "; write_matrix(out_file, r.norm_fronts[i]);
    out_file << ", ";

    out_file << "\"gen" << i << "-raw-d-score\": "; write_array(out_file, r.raw_d_scores[0]);
    out_file << ", ";

    out_file << "\"gen" << i << "-norm-d-score\": "; write_array(out_file, r.norm_d_scores[i]);
    out_file << ", ";

    out_file << "\"gen" << i << "-sparsity\": " << sparsity_metric(r.fronts[i]);
    out_file << ", ";

    out_file << "\"gen" << i << "-raw-winner\": "; write_array(out_file, r.winner[i]);
    out_file << ", ";

    out_file << "\"gen" << i << "-norm-winner\": "; write_array(out_file, r.norm_winner[i]);
    out_file << ", ";

    out_file << "\"gen" << i << "-winner-raw-d-score\": " << r.winner_raw_d_score[i];
    out_file << ", ";
  
    out_file << "\"gen" << i << "-winner-norm-d-score\": " << r.winner_norm_d_score[i];
    out_file << ", ";

    out_file << "\"gen" << i << "-time\": " << r.run_times[i];

  }


  out_file << "}";
            
}


// all the records will be averaged in python, this is just to write it in json format
void write_all_records(const std::vector<struct::log> &logs, std::string file_name){
  std::ofstream out_file(file_name + ".json");

  std::cout << "writting data..." << std::endl;

  out_file << "{\"data\": [";

  write_record(out_file, logs[0]);
  for(int i = 1; i < logs.size(); i++){
    out_file << ", ";
    write_record(out_file, logs[i]);
  }

  out_file << "]}";

  out_file.close();

  std::cout << "DONE" << std::endl;
}

/**
 * End Logger
 */

// reading the instances txt file.
std::unordered_map<std::string, std::vector<std::vector<size_t>>> read_ASCII_instances(const std::string INSTANCE_FILE, const int inst_count){
  std::unordered_map<std::string, std::vector<std::vector<size_t>>> instances;

  std::ifstream ifs(INSTANCE_FILE);

  std::string x;
  size_t s, t;
  
  
  while(ifs >> x){
    // std::cout << x << std::endl;
    instances[x];
    for(int i = 0; i < inst_count; i++){
      ifs >> s >> t;
      // std::cout << "  " << s << " " << t << std::endl;
      instances[x].push_back({s,t});
    }
  }

  ifs.close();


  

   return instances;
}


std::vector<struct::log> ASCII_instance_runner(const std::string vote_method, const std::unordered_map<std::string, std::vector<std::vector<size_t>>> &instances){

  std::vector<struct::log> logs;
  
  int n = 1;   // map counter
  int E = 0;
  for(auto &inst : instances){ // for each map 
    std::string map_name = inst.first;
    std::vector<NodePtr> node_list;
    AdjMatrix adj_matrix;

    std::cout << map_name << " (" << n << "/156)" << std::endl;

    auto x = h_functor(0);
    heuristic h = x;
    getNodes_ASCII(map_name, node_list, adj_matrix);
    int m = 1;  // instances counter
    for(auto param : inst.second){ // for each valid source and target position (there will be 40)
      // get source and target
      size_t source = param[0],
             target = param[1];

      std::cout << "  " << source << " " << target << " (" << m << "/15)" << " E = " << E << std::endl;
      
      struct::log a(map_name, source, target, vote_method);
      VBEA(adj_matrix, node_list, h, source, target, vote_method, 5, adj_matrix.get_obj_count(), a);
      logs.push_back(a);
      m++;
      // break;
    }
    n++;
    // break;
  }
  std::cout << "Done!" << std::endl;

  return logs;
}

// for creating the .gr file that contains the thrid objective for all the other files
// suppose objective cost 1 and two are already there
void add_third_objective(const std::string MAP, std::vector<Edge> &edge_list, const std::vector<std::vector<size_t>> &instances){
  std::default_random_engine generator(1395);
  std::uniform_real_distribution<double> urng(0.3,0.4); // uniform random generator between 0.3 and 0.4

  for(auto &e: edge_list){
    e.cost.push_back( urng(generator)*(e.cost[0] + e.cost[1]));
  }

  // save to a text file
  // std::cout << "enter map name (BAY, FLA, NY, etc): ";
  // std::string MAP;
  // std::cin >> MAP;
  std::ofstream out_file("USA-road-3." + MAP + ".gr");

  out_file << "source target" << std::endl;
  for(auto &inst : instances){
    out_file << "v " << inst[0] << " " << inst[1] << std::endl;
  }
}


/**
 * add the function for up too five objectives for Apex and VBEA forthe Road maps intnace
*/

std::vector<std::vector<size_t>> load_road_instances(const std::string MAP){
  std::vector<std::vector<size_t>> instances;
  // std::ifstream ifs(MAP + "-20.txt");
  std::ifstream ifs(MAP + "-25.txt");
  std::string x;
  size_t s, t;

  getline(ifs, x);

  while(ifs >> x){
    if(x == "v"){
      ifs >> s >> t;
      std::cout << s << " " << t << std::endl;
      instances.push_back({s, t});
    } else{
      break;
    }
  }

  ifs.close();
  return instances;
}

std::vector<struct::log> road_instances_runner(const std::string MAP, const std::string voting_method){
  std::vector<struct::log> LOGS;

  // fetching instances
  std::cout << "Reading Instances... " << std::endl;
  std::vector<std::vector<size_t>> instances = load_road_instances(MAP);
  std::cout << "Read " << instances.size() << " instances." << std::endl;

  // return {{}};

  //contruct graph
  std::vector<NodePtr> node_list;
  std::vector<Edge> edges;
  size_t graph_size;
  std::string MAP_DIR = "USA-road/USA-road-" + MAP;


  std::vector<std::string> files;
  for(auto file_iter : std::filesystem::directory_iterator(MAP_DIR)){
      files.push_back(file_iter.path());
  }
  std::string node_file;
  for(auto file = files.begin(); file != files.end(); file++){
    if(file->substr(file->size()-2) == "co"){
      node_file = *file;
      files.erase(file); //remove from other files
      break;
    }
  }
  
  std::cout << "Constructing Graph...";
  load_gr_files(files, edges, graph_size);
  getNodes_DOT(node_file, node_list);
  //check if third objective file exist:
  bool has_third = false;
  for(auto &f : files){
    if(f == "USA-road-3." + MAP + ".gr"){
      has_third = true;
      break;
    }
  }
  if(!has_third){ // add if it doesnt
    add_third_objective(MAP, edges, instances);
  }

  AdjMatrix adj_matrix(graph_size, edges);
  std::cout << " Finished" << std::endl << "Graph Size: " << adj_matrix.size() << ", objective count: " << adj_matrix.get_obj_count() << std::endl;
  
  heuristic h = h_functor(1);
  int n = 1;
  for(auto &inst: instances){
    std::cout << "  " << inst[0] << " " << inst[1] << " (" << n << "/" << instances.size() << ")" << std::endl;
    struct::log a(MAP, inst[0], inst[1], voting_method);
    VBEA(adj_matrix, node_list, h, inst[0], inst[1], voting_method, 5, adj_matrix.get_obj_count(), a);
    LOGS.push_back(a);
    // break;
    n++;
  }


  return LOGS;
}


// making instances
// int main(){
//   std::unordered_map<std::string, std::vector<std::vector<size_t>>> map_instances;
//   make_ASCII_instances("dao-map", map_instances, 15);

//   std::ofstream out_file("dao-uniform-15.txt");
  
//   for(auto &iter : map_instances){
//     out_file << iter.first << '\n';
//     for(auto &jter : iter.second){
//       out_file << jter[0] << " " << jter[1] << '\n';
//     }
//   }

//   out_file.close();

//   return 0;
// }


// ROAD MAP main
int main(){
  std::string MAP = "NY";
  std::string VOTING = "combined_approval";
  auto LOGS = road_instances_runner(MAP, VOTING);
  write_all_records(LOGS,  "uncapped_" + MAP + "_25_" + VOTING + "combined");

  return 0;
}



// ASCCI INSTANCE MAIN

void map_data_write(std::ostream &out_file, const std::string map_name, const size_t node_size, const size_t edge_size){
  out_file << " {";

  out_file << "\"map-name\": " << "\"" << map_name << "\"";
  out_file << ", ";

  out_file << "\"nodes\": " << node_size;
  out_file << ", ";

  out_file << "\"edges\": " << edge_size;

  out_file << "}";

}
// main for getitng the map size
// int main(){
//   // 0: vertext count,
//   // 1: edge count
//   std::unordered_map<std::string, std::vector<size_t>> map_sizes;
//   std::cout << "Constructing graphs..." << std::endl;
//   for(auto &file_iter: std::filesystem::directory_iterator("dao-map")){
//     std::vector<NodePtr> node_list;
//     AdjMatrix adj_matrix;
//     std::string MAP_FILE = file_iter.path().relative_path().string();
//     std::cout << MAP_FILE<< "...";
//     getNodes_ASCII(MAP_FILE, node_list, adj_matrix);
//     map_sizes[MAP_FILE] = {adj_matrix.size(), adj_matrix.edge_size()};
//     std::cout << "Done" << std::endl;
//   }

//   //write to json;

//   std::ofstream out_file("dao-map-info.json");

//   out_file << "{ \"data\": [";

//   for(auto i = map_sizes.begin(); i != map_sizes.end(); i++){
//     std::string MAP_NAME = i->first.substr(8);
//     std::cout << "map: " << MAP_NAME << "V: " << i->second[0] << ", E: " << i->second[1] << std::endl;
//     if(i == map_sizes.begin()){
//       map_data_write(out_file, i->first, i->second[0], i->second[1]);
//     } else {
//       out_file << ", ";
//       map_data_write(out_file, i->first, i->second[0], i->second[1]);
//     }
//   }


//   out_file << "]}";
// }



/**
 * MAIN'S 
*/  

// main for dao maps, change dir to the name of 
// int main(){
//   std::string dir = "dao-map";
//   std::string voting_method = "borda";

//   auto instances = get_ASCII_instances("dao-uniform-15.txt");
//   auto LOGS = ASCII_instance_runner(voting_method, instances);
//   write_all_records(LOGS, "uncap_borda_total_combined");
//   return 0;
// }

//testing reading instances

  
// int main(){
//   auto instances = get_ASCII_instances("./DAO-instances_40.txt");
//   ASCII_instance_runner(voting_method::borda, instances);

// }

