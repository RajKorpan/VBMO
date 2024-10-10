#include <algorithm>
#include <cstdio>
#include <ctime>
#include <ios>
#include <string>
#include <unordered_map>
#include <vector>
#include <random>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>

#include "IOutili.hpp"
#include "graph.hpp"
#include "utili.hpp"

//
// reading instances
//


// TODO: add text for visualization

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

bool load_gr_files(std::vector<std::string> gr_files, std::vector<Edge> &edges_out, size_t &graph_size_out){
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
  graph_size_out = max_node_num;
  return true;
}

void get_nodes_ROAD(const std::string ASCII_FILE, std::vector<NodePtr> &out_nodes){
  // std::cout << ASCII_FILE << std::endl;
  std::fstream ifs(ASCII_FILE);
  out_nodes.clear();
  out_nodes.push_back(nullptr);     // id starts at 1;
  
  std::string x;
  size_t id = 1;

  for(int i = 0; i < 5; i++){       // ignore header 
    std::getline(ifs, x);
  }
  int node_count = stoi(x.substr(12));  
  std::cout << node_count << std::endl;

  std::getline(ifs, x);
  std::getline(ifs, x);

  double a, b;
  while(ifs >> x){
    if(x == "v"){
      ifs >> id >> a >> b; // file is formated lng lat
      out_nodes.push_back(std::make_shared<Node>(b / 1000000, a / 1000000, id));      // but stored lat lngW
      id++;
    } else {
      std::cout << "inconsistancy in file, aborting. \n";
      break;
    }
  }

  ifs.close();
}

void get_nodes_ASCII(const std::string ASCII_FILE, std::vector<NodePtr> &out_nodes, AdjMatrix &adj_matrix){
  
  std::fstream ifs(ASCII_FILE);
  std::string temp;
  int height;
  int width;

  //get height and width of the graph
  std::getline(ifs, temp);
  // std::cout << temp << std::endl;
  std::getline(ifs, temp);
  height = stoi(temp.substr(7));
  std::getline(ifs, temp);
  width = stoi(temp.substr(6));
  std::getline(ifs, temp);


  // std::cout << "h: " << height << ", w: " << width << std::endl;

  size_t id = 0; // id will be incremented before assigment (i.e. their enumeraton will start at one)
  double x, y;

  std::vector<std::vector<char>> map(height, std::vector<char>(width));

  // Text file into matrix

  std::unordered_map<std::pair<double,double>, size_t, pair_hash> cordinateIDmap;
  std::unordered_map<size_t, NodePtr> idNodeMap;

  for(int i = 0; i < height; i++){
    std::getline(ifs, temp);
    for(int j = 0; j < width; j++){
      map[i][j] = temp[j];
      if(map[i][j] == '.'){
        id++;
        out_nodes.push_back(std::make_shared<Node>(i, j, id));     // create the nodes with the x,y cordinate of the grid, and with a unique id provided by the counter
        cordinateIDmap[std::make_pair(i,j)] = id;
        idNodeMap[id] = out_nodes.back();
      }
    }
  }

  // Contructing adjacencny matrix with the following objectives
  // - Uniform (1)
  // - Euclidean Distance (1, 1.414)
  // - Random interger within [1,20]
  // - Danger (1.5, 20)
  // - Safety (see paper for definitiion)
  
  std::vector<Edge> Edges;

  std:std::mt19937 RNG(96534);

  //adding edge cost to all eight neighbors
  for(double i = 0; i < height; i++){
    for(double  j = 0; j < width; j++){
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
  ifs.close();


  // temporary adjacency list needed for adding the other objectives
  std::vector<std::vector<Edge>> tempMap(id, std::vector<Edge> {});  


  // DANGER OBJECTIVE
  // group nodes by which have a shared incoming edge (y in x->y)
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
  // Restore 
  Edges.clear();
  for(auto iter = tempMap.begin(); iter != tempMap.end(); iter++){
    Edges.insert(Edges.end(), iter->begin(), iter->end());
  }

  // SAFTELY OBJECTIVE
  // group by outgoing edges (x in x->y)
  tempMap.clear();
  tempMap = std::vector<std::vector<Edge>>(id, std::vector<Edge> {});  
  for(auto iter = Edges.begin(); iter != Edges.end(); iter++){
    tempMap[iter->source].push_back(*iter);
  } 
  for(int i = 0; i < tempMap.size(); i++){
    for(int j = 0; j < tempMap[i].size(); j++){     
      tempMap[i][j].cost.push_back( (10 - tempMap[i].size() + 10 - tempMap[j].size()) / 2.0);
    }
  }
  // Restore 
  Edges.clear();
  for(auto iter = tempMap.begin(); iter != tempMap.end(); iter++){
    Edges.insert(Edges.end(), iter->begin(), iter->end());
  }

  // DELAY 
  for(auto iter = Edges.begin(); iter != Edges.end(); iter++){
    size_t s_v = idNodeMap[iter->source]->v, t_v = idNodeMap[iter->target]->v;
    double D;
    if(s_v != t_v){
      D = 2;
    } else if(s_v == 50 && t_v == 50){
      D = 3;
    } else if(s_v == 100 && t_v == 100){
      D = 1;
    } else {
      D = 0.25;
    }

    iter->cost.emplace_back(D);
  }

  adj_matrix = AdjMatrix(id, Edges);

  // displaying all edges and nodes
  // std::cout << adj_matrix << std::endl;
  // for(auto &iter : out_nodes){
  //   std::cout << *iter << std::endl;
  // }

  // std::cout << id << std::endl;
  // std::cout << out_nodes.size() << std::endl;
}


//
// << overloads
//

std::ostream& operator<<(std::ostream &stream, const std::vector<double> &vec){
    if(vec.size() > 0) {
        std::cout << "{" << vec[0];
        for(int i = 1; i < vec.size(); i++) {
            std::cout << ", " << std::fixed << vec[i];
        }
        std::cout << "}";
    } 
    else { // the vecetor is empty
        std::cout << "{}";
    }

  return stream;
}



std::ostream& operator<<(std::ostream &stream, const std::vector<std::vector<double>> &vec){
    if(vec.size() > 0) {
        std::cout << "{" << vec[0];
        for(int i = 1; i < vec.size(); i++) {
            std::cout << ", " << vec[i];
        }
        std::cout << "}";
    } 
    else { // the vecetor is empty
        std::cout << "{}";
    }

  return stream;
}

std::ostream& operator<<(std::ostream &stream, const std::vector<size_t> &vec){
    if(vec.size() > 0) {
        std::cout << "{" << vec[0];
        for(int i = 1; i < vec.size(); i++) {
            std::cout << ", " << vec[i];
        }
        std::cout << "}";
    } 
    else { // the vecetor is empty
        std::cout << "{}";
    }

  return stream;
}

std::ostream& operator<<(std::ostream &stream, const std::vector<int> &vec){
    if(vec.size() > 0) {
        std::cout << "{" << vec[0];
        for(int i = 1; i < vec.size(); i++) {
            std::cout << ", " << vec[i];
        }
        std::cout << "}";
    } 
    else { // the vecetor is empty
        std::cout << "{}";
    }

  return stream;  
}

std::ostream& operator<<(std::ostream &stream, const std::vector<std::string> &vec){
    if(vec.size() > 0) {
        std::cout << "{" << vec[0];
        for(int i = 1; i < vec.size(); i++) {
            std::cout << ", " << vec[i];
        }
        std::cout << "}";
    } 
    else { // the vecetor is empty
        std::cout << "{}";
    }

  return stream;
}


std::ostream& operator<<(std::ostream &stream, const Edge &edge) {
    // Printed in JSON format
    stream
        << "{"
        <<  "\"edge_source\": " << edge.source << ", "
        <<  "\"edge_target\": " << edge.target << ", "
        <<  "\"edge_cost\": ";

    for (auto c: edge.cost){
      stream << c << ", ";
    }
    stream << "}";

    return stream;
}// 
// output 
// 


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

void write_record(std::ostream &out_file, const struct::log &r) {
  out_file << "{";
  
  out_file << "\"source\": " << r.source;
  out_file << ", ";

  out_file << "\"target\": " << r.target;
  out_file << ", ";

// FRONTS

  out_file << "\"gen0-front\": "; write_matrix(out_file, r.fronts[0]);
  out_file << ", ";

  out_file << "\"gen0-norm-front\": "; write_matrix(out_file, r.norm_fronts[0]);
  out_file << ", ";

  out_file << "\"gen0-raw-d-score\": "; write_array(out_file, r.raw_d_scores[0]);
  out_file << ", ";

  out_file << "\"gen0-norm-d-score\": "; write_array(out_file, r.norm_d_scores[0]);
  out_file << ", ";

  out_file << "\"gen0-sparsity\": " << sparsity(r.fronts[0]);
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

  for(int i = 1; i < r.fronts.size(); i++){
    out_file << ", ";
  
    out_file << "\"gen" << i << "-front\": "; write_matrix(out_file, r.fronts[i]);
    out_file << ", ";

    out_file << "\"gen" << i << "-norm-front\": "; write_matrix(out_file, r.norm_fronts[i]);
    out_file << ", ";

    out_file << "\"gen" << i << "-raw-d-score\": "; write_array(out_file, r.raw_d_scores[i]);
    out_file << ", ";

    out_file << "\"gen" << i << "-norm-d-score\": "; write_array(out_file, r.norm_d_scores[i]);
    out_file << ", ";

    out_file << "\"gen" << i << "-sparsity\": " << sparsity(r.fronts[i]);
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

void write_all_records_alt(const std::vector<struct::log> &logs, std::string file_name, const size_t T, const size_t K, const std::string voting_method, const std::string child_method){
  std::ofstream out_file(file_name + "-" + voting_method + "-" + child_method + "-" + std::to_string(K) +  ".json");

  std::cout << "writting data...";

  out_file << "{\"data\": [";

  write_record(out_file, logs[0]);
  for(int i = 1; i < logs.size(); i++){
    out_file << ", ";
    write_record(out_file, logs[i]);
  }
  out_file  << "], " 

            << "\"T\": " << T << ", "
            << "\"K\": " << K << ", "
            << "\"voting_method\": " << "\"" <<  voting_method << "\"" << "}";

}


// for creating the .gr file that contains the thrid objective for all the other files
// suppose objective cost 1 and two are already there
void add_third_objective(const std::string MAP, std::vector<Edge> &edge_list,  std::vector<NodePtr> &node_list, bool save){ 
  std::default_random_engine generator(1395);
  std::uniform_real_distribution<double> urng(0.3,0.4); // uniform random generator between 0.3 and 0.4

  for(auto &e: edge_list){
    e.cost.push_back( urng(generator)*(e.cost[0] + e.cost[1])); // the function provided in a-pexs original paper
  }

  if(save){
    std::ofstream out_file("USA-road-3." + MAP + ".gr");

    out_file << "c 9th DIMACS Implementation Challenge: Shortest Paths" << std::endl
             << "c https://ojs.aaai.org/index.php/ICAPS/article/view/19825" << std::endl
             << "c TIGER/Line graph USA-road-3." + MAP << std::endl 
             << "c" << std::endl
             << "p aux sp"  << node_list.size() - 1  << " " << edge_list.size() << std::endl
             << "c graph contains " << node_list.size() - 1 << " nodes and " << edge_list.size() << " arcs" << std::endl
             << "c" << std::endl;
    for(auto &e: edge_list){
      out_file << "a " << e.source << " " << e.target << " " << e.cost.back() << std::endl;
    }
    out_file.close();
  }
}

// creating a .gr file for the delay objective
void add_delay_objective(const std::string MAP, std::vector<Edge> &edge_list, std::vector<NodePtr> &node_list, bool save){     
  // std::vector<double> map(4,0.0);
  for(auto &e : edge_list){
    size_t s_v = node_list[e.source]->v, t_v = node_list[e.target]->v;
    double D;
    if(s_v != t_v){
      D = 2;
      // map[1]++;
    } else if(s_v == 50 && t_v == 50){
      D = 3;
      // map[2]++;
    } else if(s_v == 100 && t_v == 100){
      D = 1;
      // map[0]++;
    } else {
      D = 0.25;
      // map[3]++; 
    }

    e.cost.emplace_back(D);
  }
  if(save){
    std::ofstream out_file("USA-road-DELAY." + MAP + ".gr");

    out_file << "c 9th DIMACS Implementation Challenge: Shortest Paths" << std::endl
             << "c https://ci.ovgu.de/Publications/Scalable+Many_Objective+Pathfinding+Benchmark+Suite-p-910.html" << std::endl
             << "c TIGER/Line graph USA-road-DELAY." + MAP << std::endl 
             << "c" << std::endl
             << "p sp "  << node_list.size() - 1  << " " << edge_list.size() << std::endl
             << "c graph contains " << node_list.size() - 1 << " nodes and " << edge_list.size() << " arcs" << std::endl
             << "c" << std::endl;
    for(auto &e: edge_list){
      out_file << "a " << e.source << " " << e.target << " " <<  e.cost.back() << std::endl;
    }
    // std::cout << "D = " << std::endl 
    //           << ".25 " << map[3] << std::endl
    //           << "1.0 " << map[0] << std::endl
    //           << "2.0 " << map[1] << std::endl
    //           << "3.0 " << map[2] << std::endl;
  }
}


std::unordered_map<std::string, std::vector<std::vector<size_t>>>  load_asci_queries(const std::string asci_query_file){
    std::ifstream ifs(asci_query_file);
    std::unordered_map<std::string, std::vector<std::vector<size_t>>> queries;

    std::string x;
    size_t s, t;

    while(ifs >> x){
    // std::cout << x << std::endl;
    queries[x];
    // std::cout << x.substr(18) << std::endl;
    for(int i = 0; i < 5; i++){
      ifs >> s >> t;
      // std::cout << "  " << s << " " << t << std::endl;
      std::vector<size_t> temp = {s, t};
      queries[x].push_back(temp);
    }
    }

    ifs.close();
    std::cout << "loaded " << queries.size() << " queries" << std::endl;

    return queries;
}
// bool load_ascii_file(std::string ascii_file, std::vector<Edge> &edges, std::vector<NodePtr> &out_nodes,  size_t &graph_size){
//     std::ifstream ifs(ascii_file);

//     std::string temp;
//     int height, width;

//     //removing header 
//     std::getline(ifs, temp);
//     std::getline(ifs, temp);
//     height = stoi(temp.substr(7));
//     std::getline(ifs, temp);
//     width = stoi(temp.substr(6));
//     std::getline(ifs, temp);

//     size_t id = 0; // increments before assignment.
    

//     //allocate just enough space for the ascii map
//     std::vector<std::vector<char>> map(height, std::vector<char>(width));
//     // need a way of converting xy coordinates into the id for creating the edges later on

//     struct pair_hash{
//         size_t operator()(const std::pair<size_t,size_t> &p) const {
//             auto h1 = std::hash<size_t>{}(p.first),
//                  h2 = std::hash<size_t>{}(p.second);
//             if(h1 != h2){
//                 return h1 ^ h2;
//             } else {
//                 return h1;
//             }
//         }
//     };

//     std::unordered_map<std::pair<size_t,size_t>, size_t, pair_hash> coordinateIDmap;
//     edges.clear();

//     // matching xy coordinates to id's 
//     for(int i = 0; i < height; i++){
//         std::getline(ifs, temp);
//         // std::cout << temp << std::endl; // DEBUGGING
//         for(int j = 0; j < width; j++){
//             map[i][j] = temp[j];
//             if(map[i][j] == '.') { // "."s on the ascii map is a position 
//                 id++;
//                 coordinateIDmap[{i,j}] = id;
//                 out_nodes.push_back(std::make_shared<Node>(i,j,id));
//                 // std::cout << i << " " << j << " " << id << std::endl;
//             }
//         }
//     }

//     ifs.close();
//     graph_size = id;

//     // adding euclidean distance, uniform, and random objective costs

//     std::mt19937 RNG(96534);

//     // !!! size_t does not go into the negatives, 0-1 does not result in a negative number, but the larget v
//     for(int i = 0; i < height; i++){
//         for(int j = 0; j < width; j++){
//             if(map[i][j] == '.'){
          
//                 if(i-1 >= 0 && map[i-1][j] == '.'){                      // up 
//                 // std::cout << "  " << i-1 << " " << j << std::endl;
//                 edges.push_back(Edge(coordinateIDmap[{i,j}], 
//                                     coordinateIDmap[{i-1,j}], {1, 1, 1 + double(RNG()%20)}));
//                 }
//                 if(i-1 >= 0 && j + 1 < width && map[i-1][j+1] == '.'){   // up right
//                 // std::cout << "  " << i-1 << " " << j+1 << std::endl;
//                 edges.push_back(Edge(coordinateIDmap[{i,j}], 
//                                     coordinateIDmap[{i-1,j+1}], {1, 1.414, 1 + double(RNG()%20)}));
//                 }
//                 if(j+1 < width && map[i][j+1] == '.'){                   // right
//                 // std::cout << "  " << i << " " << j+1 << std::endl;
//                 edges.push_back(Edge(coordinateIDmap[{i,j}], 
//                                     coordinateIDmap[{i,j+1}], {1, 1, 1 + double(RNG()%20)}));
//                 }
//                 if(i+1 < height && j+1 < width && map[i+1][j+1] == '.'){ // down right
//                 // std::cout << "  " << i+1 << " " << j+1 << std::endl;
//                 edges.push_back(Edge(coordinateIDmap[{i,j}], 
//                                     coordinateIDmap[{i+1,j+1}], {1, 1.414, 1 + double(RNG()%20)}));
//                 }
//                 if(i+1 < height && map[i+1][j] == '.'){                  // down
//                 // std::cout << "  " << i+1 << " " << j << std::endl;
//                 edges.push_back(Edge(coordinateIDmap[{i,j}], 
//                                     coordinateIDmap[{i+1,j}], {1, 1, 1 + double(RNG()%20)}));
//                 }
//                 if(i+1 < height && j-1 >= 0 && map[i+1][j-1] == '.'){    // down left
//                 // std::cout << "  " << i+1 << " " << j-1 << std::endl;
//                 edges.push_back(Edge(coordinateIDmap[{i,j}], 
//                                     coordinateIDmap[{i+1,j-1}], {1, 1.414, 1 + double(RNG()%20)}));
//                 }
//                 if(j-1 >= 0 && map[i][j-1] == '.'){                      // left
//                 // std::cout << "  " << i << " " << j-1 << std::endl;
//                 edges.push_back(Edge(coordinateIDmap[{i,j}], 
//                                     coordinateIDmap[{i,j-1}], {1, 1, 1 + double(RNG()%20)}));
//                 }
//                 if(i-1 >= 0 && j-1 >= 0 && map[i-1][j-1] == '.'){        // up lef
//                 // std::cout << "  " << i-1 << " " << j-1 << std::endl;
//                 edges.push_back(Edge(coordinateIDmap[{i,j}], 
//                                     coordinateIDmap[{i-1,j-1}], {1, 1.414, 1 + double(RNG()%20)}));
//                 }
//             }
//         }
//     }

//     // std::cout << "added first three objectives" << std::endl;

//     //leave in for 3-objective, comment out to add last 2 objectives
//     // return true;

//     // adding random dangerous objective
//     std::vector<std::vector<Edge>> tempMap(id, std::vector<Edge> {});

//     // create list of all edges that have the same target
//     for(auto iter = edges.begin(); iter != edges.end(); iter++){
//         tempMap[iter->target].push_back(*iter);
//     }

//     //iterating over id's adjacency list
//     for(auto iter = tempMap.begin(); iter != tempMap.end(); iter++){
//         double n;
//         if(RNG() % 10 == 0){    //randomly make 10% of all incoming edges expensive 
//             n = 10;
//         } else {
//             n = 1.5;
//         }

//         for(auto jter = iter->begin(); jter != iter->end(); jter++){
//             jter->cost.push_back(n);
//         }
//     }

//     //return to edge list
//     edges.clear();
//     for(auto iter =  tempMap.begin(); iter != tempMap.end(); iter++){
//         edges.insert(edges.end(), iter->begin(), iter->end());
//     }

//     // reverse adjacency list
//     tempMap.clear();
//     tempMap = std::vector<std::vector<Edge>>(id, std::vector<Edge> {});
//     for(auto iter = edges.begin(); iter != edges.end(); iter++){
//         tempMap[iter->source].push_back(*iter);
//     }

//     //adding safety objective
//     for(int i = 0; i < tempMap.size(); i++){
//         for(int j = 0; j < tempMap[i].size(); j++){
//             tempMap[i][j].cost.push_back( (10 - tempMap[i].size() + 10 - tempMap[j].size()) / 2.0);
//         }
//     }

//     // revert to edges;
//     edges.clear();
//     for(auto iter =  tempMap.begin(); iter != tempMap.end(); iter++){
//         edges.insert(edges.end(), iter->begin(), iter->end());
//     }

//     // std::cout << coordinateIDmap.size() << " " << id << " " << graph_size << std::endl;
//     return true;
// }

bool load_ascii_file(std::string ascii_file, std::vector<Edge> &out_edges, std::vector<NodePtr> &out_nodes, size_t &graph_size){
    std::ifstream ifs(ascii_file);

    std::string temp;
    int height, width;

    //removing header 
    std::getline(ifs, temp);
    std::getline(ifs, temp);
    height = stoi(temp.substr(7));
    std::getline(ifs, temp);
    width = stoi(temp.substr(6));
    std::getline(ifs, temp);

    size_t id = 0;
    

    //allocate just enough space for the ascii map
    std::vector<std::vector<char>> map(height, std::vector<char>(width));
    // need a way of converting xy coordinates into the id for creating the out_edges later on

    struct pair_hash{
        size_t operator()(const std::pair<size_t,size_t> &p) const {
            auto h1 = std::hash<size_t>{}(p.first),
                 h2 = std::hash<size_t>{}(p.second);
            if(h1 != h2){
                return h1 ^ h2;
            } else {
                return h1;
            }
        }
    };

    std::unordered_map<std::pair<size_t,size_t>, size_t, pair_hash> coordinateIDmap;
    out_edges.clear();
    out_nodes.clear();

    // matching xy coordinates to id's 
    for(int i = 0; i < height; i++){
        std::getline(ifs, temp);
        // std::cout << temp << std::endl; // DEBUGGING
        for(int j = 0; j < width; j++){
            map[i][j] = temp[j];
            if(map[i][j] == '.') { // "."s on the ascii map is a position 
                out_nodes.push_back(std::make_shared<Node>(i, j, id));
                coordinateIDmap[{i,j}] = id;
                id++;
                // std::cout << i << " " << j << " " << id << std::endl;
            }
        }
    }

    ifs.close();
    graph_size = id;

    // adding euclidean distance, uniform, and random objective costs

    std::mt19937 RNG(96534);

    // !!! size_t does not go into the negatives, 0-1 does not result in a negative number, but the larget v
    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            if(map[i][j] == '.'){
          
                if(i-1 >= 0 && map[i-1][j] == '.'){                      // up 
                // std::cout << "  " << i-1 << " " << j << std::endl;
                out_edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i-1,j}], {1, 1, 1 + double(RNG()%20)}));
                }
                if(i-1 >= 0 && j + 1 < width && map[i-1][j+1] == '.'){   // up right
                // std::cout << "  " << i-1 << " " << j+1 << std::endl;
                out_edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i-1,j+1}], {1, 1.414, 1 + double(RNG()%20)}));
                }
                if(j+1 < width && map[i][j+1] == '.'){                   // right
                // std::cout << "  " << i << " " << j+1 << std::endl;
                out_edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i,j+1}], {1, 1, 1 + double(RNG()%20)}));
                }
                if(i+1 < height && j+1 < width && map[i+1][j+1] == '.'){ // down right
                // std::cout << "  " << i+1 << " " << j+1 << std::endl;
                out_edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i+1,j+1}], {1, 1.414, 1 + double(RNG()%20)}));
                }
                if(i+1 < height && map[i+1][j] == '.'){                  // down
                // std::cout << "  " << i+1 << " " << j << std::endl;
                out_edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i+1,j}], {1, 1, 1 + double(RNG()%20)}));
                }
                if(i+1 < height && j-1 >= 0 && map[i+1][j-1] == '.'){    // down left
                // std::cout << "  " << i+1 << " " << j-1 << std::endl;
                out_edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i+1,j-1}], {1, 1.414, 1 + double(RNG()%20)}));
                }
                if(j-1 >= 0 && map[i][j-1] == '.'){                      // left
                // std::cout << "  " << i << " " << j-1 << std::endl;
                out_edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i,j-1}], {1, 1, 1 + double(RNG()%20)}));
                }
                if(i-1 >= 0 && j-1 >= 0 && map[i-1][j-1] == '.'){        // up lef
                // std::cout << "  " << i-1 << " " << j-1 << std::endl;
                out_edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i-1,j-1}], {1, 1.414, 1 + double(RNG()%20)}));
                }
            }
        }
    }

    // std::cout << "added first three objectives" << std::endl;

    //leave in for 3-objective, comment out to add last 2 objectives
    // return true;

    // adding random dangerous objective
    std::vector<std::vector<Edge>> tempMap(id, std::vector<Edge> {});

    // create list of all out_edges that have the same target
    for(auto iter = out_edges.begin(); iter != out_edges.end(); iter++){
        tempMap[iter->target].push_back(*iter);
    }

    //iterating over id's adjacency list
    for(auto iter = tempMap.begin(); iter != tempMap.end(); iter++){
        double n;
        if(RNG() % 10 == 0){    //randomly make 10% of all incoming out_edges expensive 
            n = 10;
        } else {
            n = 1.5;
        }

        for(auto jter = iter->begin(); jter != iter->end(); jter++){
            jter->cost.push_back(n);
        }
    }

    //return to edge list
    out_edges.clear();
    for(auto iter =  tempMap.begin(); iter != tempMap.end(); iter++){
        out_edges.insert(out_edges.end(), iter->begin(), iter->end());
    }

    // reverse adjacency list
    tempMap.clear();
    tempMap = std::vector<std::vector<Edge>>(id, std::vector<Edge> {});
    for(auto iter = out_edges.begin(); iter != out_edges.end(); iter++){
        tempMap[iter->source].push_back(*iter);
    }

    //adding safety objective
    for(int i = 0; i < tempMap.size(); i++){
        for(int j = 0; j < tempMap[i].size(); j++){
            tempMap[i][j].cost.push_back( (10 - tempMap[i].size() + 10 - tempMap[j].size()) / 2.0);
        }
    }

    // revert to out_edges;
    out_edges.clear();
    for(auto iter =  tempMap.begin(); iter != tempMap.end(); iter++){
        out_edges.insert(out_edges.end(), iter->begin(), iter->end());
    }

    // std::cout << coordinateIDmap.size() << " " << id << " " << graph_size << std::endl;


    return true;
}

void load_road_instances(const std::string query_file, std::vector<std::vector<size_t>> &queries_out){
  std::ifstream ifs(query_file);

  std::ifstream   file(query_file.c_str());

  if (file.is_open() == false) {
    return ;
  }

  std::string line;
  while (file.eof() == false) {
      std::getline(file, line);

      if (line == "") {
          break;
      } else if (line[0] == '#') {
          continue; // Commented out queries
      }

      std::vector<std::string> decomposed_line;
      split_string(line, ",", decomposed_line);

      std::vector<size_t> query = {std::stoul(decomposed_line[0]), std::stoul(decomposed_line[1])};
      queries_out.push_back(query);
  }
}



void write_logs(std::string MAP_NAME, const std::string VOTE_METHOD, const std::string CHILD_METHOD, const std::vector<struct::log> &LOGS){

    std::ofstream out_file(".json");

    std::cout << "writting data...";

    out_file << "{"
             << "\"map\": " << "\"" << VOTE_METHOD << "\"" <<  ", "
             << "\"merge strategy\": " << "\"" << CHILD_METHOD << "\"" << ", ";
            
        out_file << "\"data\": [";
    write_record(out_file, LOGS[0]);
    for(int i = 1; i < LOGS.size(); i++){
        out_file << ", ";
        write_record(out_file, LOGS[i]);
    }
    out_file << "]";

    out_file << "}";
  
}


// WRITTING FOR DAO
