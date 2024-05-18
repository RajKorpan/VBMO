#include <algorithm>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <ios>
#include <random>
#include <vector>
#include <unordered_set>
#include <fstream>
#include <iostream>
#include <limits>
#include <functional>
#include <memory>
#include <cmath>

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

// print vectors of printable things
std::ostream& operator<<(std::ostream &stream, const std::vector<double> &str_vec){
    if(str_vec.size() > 0) {
        std::cout << "{" << str_vec[0];
        for(int i = 1; i < str_vec.size(); i++) {
            std::cout << ", " << std::fixed << str_vec[i];
        }
        std::cout << "}";
    } 
    else { // the vecetor is empty
        std::cout << "{}";
    }

  return stream; 
}

std::ostream& operator<<(std::ostream &stream, const std::vector<size_t> &str_vec){
    if(str_vec.size() > 0) {
        std::cout << "{" << str_vec[0];
        for(int i = 1; i < str_vec.size(); i++) {
            std::cout << ", " << std::fixed << str_vec[i];
        }
        std::cout << "}" << std::endl;
    } 
    else { // the vecetor is empty
        std::cout << "{}" << std::endl;
    }

  return stream; 
}


/**
 *  Adjacency Matrix Class
 *  
*/
class AdjMatrix {
  private:
  std::vector<std::vector<Edge>>   matrix;      // [i] will return a reference to a vector of edges with source i. (thus checking for a particular entry is O(|V|)) not O(1) which is the deifntion of a Adj Matrix)
  size_t                           graph_size;
  size_t                           obj_count = 0;

  public:

  AdjMatrix() = default;

  AdjMatrix(size_t graph_size_, std::vector<Edge> &edges, bool inverse=false)
  : matrix((graph_size_ + 1), std::vector<Edge>()), graph_size(graph_size_) { // graph_size + 1 is because vertices id's start at 1.
    obj_count = edges[0].cost.size();

    for(auto iter = edges.begin(); iter != edges.end(); ++iter){
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

  Node(double x_, double y_, size_t id_, std::vector<double> g_ = {}, double h_ = 0, NodePtr parent_ = nullptr)
  : x(x_), y(y_), id(id_), g(g_), h(h_), f({}), parent(parent_){
    for(int i = 0; i < g.size(); i++){
      f.push_back(g[i] + h);
    }
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
    stream << "{id: " << node.id << ", x:" << std::setprecision(4) << node.x << ", y: " << std::setprecision(4) << node.y <<  "}";

    return stream;
  }

  void display(){
    std::cout << *this << std::endl;
    std::cout 
    << "f-score: " << this->f << '\n'
    << "g-score: " << this->g << '\n'
    << "h      : " << this->h << '\n';
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
struct more_than_specific{
  size_t cost_idx; 

  more_than_specific(size_t cost_idx_): cost_idx(cost_idx_) {};
  
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
//
// weight method
//

// These will be the placeholder for the fuctor object that we for either for node ordersing in the open set, and to calculate the heuristic
using heuristic = std::function<double (const NodePtr&, const NodePtr&)>;
using node_order = std::function<bool(const NodePtr&, const NodePtr&)>;

//
// A*
//

class ASTAR{
  private:
  const AdjMatrix &adj_matrix;
  const std::vector<NodePtr> &node_list;

  public:
  ASTAR(const AdjMatrix &adj_matrix_, const std::vector<NodePtr> &node_list_): adj_matrix(adj_matrix_), node_list(node_list_) {}


  // version that the paper uses. Not optimal?
  NodePtr operator()(size_t source, size_t target, heuristic &h, node_order &order){

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
                                           next_h, 
                                           node);


        open.push_back(next);
        std::push_heap(open.begin(), open.end(), order);
      }
    }
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
                   // {std::stoul(decomposed_line[3])}); //objective cost !!!! FIX !!!!
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


std::ostream& operator<<(std::ostream &stream, const std::vector<std::string> &str_vec){
    if(str_vec.size() > 0) {
        std::cout << "{" << str_vec[0];
        for(int i = 1; i < str_vec.size(); i++) {
            std::cout << ", " << std::fixed << str_vec[i];
        }
        std::cout << "}" << std::endl;
    } 
    else { // the vecetor is empty
        std::cout << "{}" << std::endl;
    }

  return stream; 
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
      std::cout << x << std::endl;
      break;
    }
  }

  ifs.close();
}

// Requires some motification to be usbale for the A*pex and other similar algorithms (or make them compatable with doubles?)
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

  size_t id = 1;
  double x, y;
  
  std::vector<std::vector<char>> map(height, std::vector<char>(width));

  std::unordered_map<std::pair<int,int>, NodePtr, pair_hash> cordinateMap;

  for(int i = 0; i < height; i++){
    std::getline(ifs, temp);
    for(int j = 0; j < width; j++){
      map[i][j] = temp[j];
      if(map[i][j] == '.'){
        out_nodes.push_back(std::make_shared<Node>(i, j, id));     // create the nodes with the x,y cordinate of the grid, and with a unique id provided by the counter
        cordinateMap[{i,j}] = out_nodes.back();                    // save the cordinate for the second pass where we need to find the id of the map from its coridnates in the graph
        id++;
      }

    }
  }

  // cross refercing count, the id - 1 (the last recordede ID used) should be the same as the size of both out_nodes and cordinateMap
  std::cout << "id:        " << (id - 1) << std::endl
            << "edge list: " << out_nodes.size() << std::endl
            << "cord map:  " << cordinateMap.size() << std::endl;

  /* Second pass: constructing the adjaceny matrix adding the following objective
  *  - Euclidean distance 
  *  - Uniform (1)
  *  - Random [0,20]
  *  - Safety ()
  */

  std::vector<Edge> Edges;

  std:std::mt19937 RNG(67507);
  
  for(size_t i = 0; i < height; i++){
    for(size_t j = 0; j < width; j++){
      //check all 8 neighbors checking the neighbor
      if(map[i][j] == '.'){
        //add all 8 neighbors 
        // if(i-1 >= 0 && map[i-1][j] == '.'){                      // up 
        //   Edges.push_back(Edge(i, j, {1, 1, double(RNG()%20)}));
        // }
        // if(i-1 >= 0 && j + 1 < width && map[i-1][j+1] == '.'){   // up right
        //   Edges.push_back(Edge(i, j, {1, 1.5, double(RNG()%20)}));
        // }
        // if(j+1 < width && map[i][j+1] == '.'){                   // right
        //   Edges.push_back(Edge(i, j, {1, 1, double(RNG()%20)}));
        // }
        // if(i+1 < height && j+1 < width && map[i+1][j+1] == '.'){ // down right
        //   Edges.push_back(Edge(i, j, {1, 1.5, double(RNG()%20)}));
        // }
        // if(i+1 < height && map[i+1][j] == '.'){                  // down
        //   Edges.push_back(Edge(i, j, {1, 1, double(RNG()%20)}));
        // }
        // if(i+1 < height && j-1 >= 0 && map[i+1][j-1] == '.'){    // down left
        //   Edges.push_back(Edge(i, j, {1, 1.5, double(RNG()%20)}));
        // }
        // if(j-1 >= 0 && map[i][j-1] == '.'){                      // left
        //   Edges.push_back(Edge(i, j, {1, 1, double(RNG()%20)}));
        // }
        // if(i-1 >= 0 && j-1 >= 0 && map[i-1][j-1] == '.'){        // up left
        //   Edges.push_back(Edge(i, j, {1, 1.5, double(RNG()%20)}));
        // }
      } else {
        continue;
      }
    }
  }


  // Randomly assined %10 of nodes to be dangerous (and safety objective is defined how?)
  
}



// int main(){
//   std::string MAP = "dao-map/arena.map";
//   std::vector<NodePtr> node_list;
//   AdjMatrix adj_matrix;

//   getNodes_ASCII(MAP,node_list, adj_matrix);
// }


int main(){
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
 
  std::reverse(files.begin(), files.end());  // for some reason the files are put in reverse order
  std::string node_file = files.front();     // get node file for node list
  files.erase(files.begin());                // remove vertex list (so obly .gr files)o

  for(auto &f: files){
    std::cout << f << std::endl;
  }
  
  std::vector<Edge> edges;
  size_t graph_size;


  std::cout << "generating graph..." << std::endl;
  if(load_gr_files(files, edges, graph_size )){
    for(auto& e: edges){
      if(e.cost.size() != 3){
        std::cout << e << std::endl;      
      }
    }
  } else {
    std::cout << "Error reading .gr files" << std::endl;
  }

  // NODE LIST
  std::vector<NodePtr> node_list;
  std::cout << "creating node list..." << std::endl;
  getNodes_DOT(node_file, node_list);

  // VERIFYING
  // std::cout << graph << std::endl;
  std::cout << "graph size:     " <<  graph_size << " , edge size: " << edges.size() << std::endl;
  std::cout << "node list size: " << node_list.size() << std::endl;



  AdjMatrix adj_matrix(graph_size, edges);

  ASTAR A(adj_matrix, node_list);

  // Uisng points source: 20002, target: 164983. to compare with A*pex

  // change the f-score that A* uses for sorting of the openSet. 


  // h_functor is set to 1 for Haversine distance (km), use 0 for euclidean distance
  auto X = h_functor(1);
  heuristic h(X);

  // more_than_specific is used to determin which f-score is used for the open set,  
    
  std::cout << "start: \n";
  std::cout << *node_list[20002].get() << std::endl;
  std::cout << "end: \n";
  std::cout << *node_list[163983].get() << std::endl;
  std::cout << "----------------------" << std::endl;

  std::vector<NodePtr> results;
  int b = adj_matrix.get_obj_count();
  for(int i = 0; i < b; i++){
    std::cout << "runing A* on objective " << i << "... \n";
    auto Y = more_than_specific(i);
    node_order order(Y);
    results.push_back(A(20002, 163983, h, order));
    std::cout << results.back()->g << std::endl;
  }

  

  return 0;
}

