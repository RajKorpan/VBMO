#include <chrono>
#include <cstddef>
#include <random>
#include <string>
#include <filesystem>
#include <vector>
#include <iostream>
#include <fstream>

#include "inc/graph.hpp"
#include "inc/IOutili.hpp"
#include "inc/utili.hpp"
#include "inc/v.hpp"

// namespace po = boost::program_options;


// K: size of the child population, 
// T: number of generations, limit 

// WEIGHTED COMBINED VERSION
void VBEA_COMBINED(struct::log &LOG, const AdjMatrix &adj_matrix, std::vector<NodePtr> node_list, heuristic &h,  const size_t source, const size_t target, const std::string voting_method, const size_t K, const size_t T = 0, const bool visualize = false){
  auto start_t = std::chrono::high_resolution_clock::now();
  int b = adj_matrix.get_obj_count();
  
// BEGIN GEN 0
  ASTAR A(adj_matrix, node_list);
  std::vector<std::vector<double>> front;
  std::vector<std::vector<double>> normalized_front;
  std::vector<double> d_score;
  std::vector<int> vote_results;

  for(int i = 0; i < b; i++){
    std::cout << i << std::endl;
    std::cout << "===========" << std::endl;
    node_order order = more_than_specific(i);
    front.push_back(A(source, target, h, order)->g);
  }
  // if K > adj_matrix.size(), then create random weight sets to fill up the population
  if(K > b){
    std::uniform_real_distribution<double> rng(0.0, 1.0);
    std::mt19937 generator(std::clock());
    WEIGHTED_ASTAR WA(adj_matrix, node_list);
    node_order order = more_than_specific(0);
    for(size_t i = b; i < K; i++){
      std::vector<double> rand_weight(b, rng(generator));
      WA.update_weight(rand_weight);
      std::cout << i << std::endl;
      std::cout << "===========" << std::endl;
      front.push_back(WA(source, target, h, order)->g);
    }
  }

  // clearning front
  std::cout << "?" << std::endl;
  // remove_duplicates(front);
  // std::cout << "rd" << std::endl;
  normalized_front = normalize_matrix(front);
  std::cout << "nm" << std::endl;
  d_score = calculate_d_score(normalized_front);
  std::cout << "ds" << std::endl;
  vote_results = vote(voting_method, normalized_front, d_score);
  std::cout << "v" << std::endl;


  if(visualize){
    std::cout << "-----gen0-----" << std::endl;
    for(int i = 0; i < front.size(); i++){
        std::cout << "P_" << i << " " << front[i] << " | " << normalized_front[i] << " d: " << d_score[i] << std::endl;
    }
    std::cout << "voting results:" << std::endl;
    std::cout << vote_results << std::endl;
  }
// END GEN 0 
 

  
// SAVING
  auto raw_d_score = calculate_d_score(front);
  LOG.run_times.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count());
  LOG.fronts.push_back(front);
  LOG.norm_fronts.push_back(normalized_front);
  LOG.raw_d_scores.push_back(raw_d_score);
  LOG.norm_d_scores.push_back(d_score);
  LOG.winner.push_back(front[vote_results[0]]);
  LOG.norm_winner.push_back(normalized_front[vote_results[0]]);
  LOG.winner_norm_d_score.push_back(d_score[vote_results[0]]);
  LOG.winner_raw_d_score.push_back(raw_d_score[vote_results[0]]);
  std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();

  return;

  
  for(int t = 0; t < T; t++){
    // create weight sets and culling previous population
    std::vector<std::vector<double>> temp;
    std::vector<std::vector<double>> weight_sets;
    for(int i = 0; i < K && i < front.size(); i++){ // save only the top K solutions from the previous generation
      temp.push_back(front[vote_results[i]]);
      weight_sets.push_back(complement_weight_set(normalized_front[vote_results[i]]));
    }
    std::swap(temp,front);

    while(weight_sets.size() < K){    // fill the weight_sets with random weight sets if there are not enough
      std::uniform_real_distribution<double> rng(0.0, 1.0);
      std::mt19937 generator(std::clock());

      weight_sets.push_back(std::vector<double>(b, rng(generator)));
    }

    //creating child population
    WEIGHTED_ASTAR WA(adj_matrix, node_list);
    auto order = node_order(more_than_specific(0));
    for(int i = 0; i < K; i++){
      WA.update_weight(weight_sets[i]);
      front.push_back(WA(source, target, h, order)->g);
    }

    //cleaning
    remove_duplicates(front);
    front = non_dominated_filter(front);
    normalized_front = normalize_matrix(front);
    d_score = calculate_d_score(normalized_front);

    //SAVING

    if(visualize){
      std::cout << "-----gen" << t + 1 << "-----" << std::endl;
      for(int i = 0; i < front.size(); i++){
          std::cout << "P_" << i << " " << front[i] << " | " << normalized_front[i] << " d: " << d_score[i] << std::endl;
      }
      std::cout << "voting results:" << std::endl;
      std::cout << vote_results << std::endl;
    }

  }

  return ;
}

void VBEA_CONSCIOUS(struct::log &LOG, const AdjMatrix &adj_matrix, std::vector<NodePtr> node_list, heuristic &h,  const size_t source, const size_t target, const std::string voting_method, const size_t K, const size_t T = 0, const bool visualize = false){
  auto start_t = std::chrono::high_resolution_clock::now();
  int b = adj_matrix.get_obj_count();
  
// BEGIN GEN 0
  ASTAR A(adj_matrix, node_list);
  std::vector<std::vector<double>> front;
  std::vector<std::vector<double>> normalized_front;
  std::vector<double> d_score;
  std::vector<int> vote_results;

  // the initial decomompositon
  for(int i = 0; i < b; i++){
    node_order order = more_than_specific(i);
    front.push_back(A(source, target, h, order)->g);
  }
  // if K > adj_matrix.size(), then create random weight sets to fill up the population
  if(K > b){
    std::uniform_real_distribution<double> rng(0.0, 1.0);
    std::mt19937 generator(std::clock());

    WEIGHTED_ASTAR WA(adj_matrix, node_list);
    node_order order = more_than_specific(0);
    for(size_t i = b; i < K; i++){
      std::vector<double> rand_weight(b, rng(generator));
      WA.update_weight(rand_weight);
      front.push_back(WA(source, target, h, order)->g);
    }
    
  }
    
  remove_duplicates(front);
  normalized_front = normalize_matrix(front);
  d_score = calculate_d_score(normalized_front);

  vote_results = vote(voting_method, normalized_front, d_score);

  if(visualize){
    std::cout << "-----gen0-----" << std::endl;
    for(int i = 0; i < front.size(); i++){
        std::cout << "P_" << i << " " << front[i] << " | " << normalized_front[i] << " d: " << d_score[i] << std::endl;
    }
    std::cout << "voting results:" << std::endl;
    std::cout << vote_results << std::endl;
  }
// END GEN 0 
 
// SAVING
  
  for(int t = 0; t < T; t++){
    start_t = std::chrono::high_resolution_clock::now();
    // create weight sets;
    std::vector<std::vector<double>> weight_sets;
    for(int i = 0; i < K && i < vote_results.size(); i++){ 
        weight_sets.push_back(complement_weight_set(normalized_front[vote_results[i]]));
    }
    while(weight_sets.size() < K){    // fill the weight_sets with random weight sets if there are not enough
      std::uniform_real_distribution<double> rng(0.0, 1.0);
      std::mt19937 generator(std::clock());

      weight_sets.push_back(std::vector<double>(b, rng(generator)));
    }

    //creating child population
    WEIGHTED_ASTAR WA(adj_matrix, node_list);
    auto order = node_order(more_than_specific(0));
    int f; // focus
    for(int i = 0; i < K; i++){
      //select the focus to be the lowest objective
    
      WA.update_weight(weight_sets[i]);
      front.push_back(WA(source, target, h, order)->g);
    }

    //cleanring
    remove_duplicates(front);
    front = non_dominated_filter(front);
    normalized_front = normalize_matrix(front);
    d_score = calculate_d_score(normalized_front);

    //culling?

    //SAVING
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();


    if(visualize){
      std::cout << "-----gen" << t + 1 << "-----" << std::endl;
      for(int i = 0; i < front.size(); i++){
          std::cout << "P_" << i << " " << front[i] << " | " << normalized_front[i] << " d: " << d_score[i] << std::endl;
      }
      std::cout << "voting results:" << std::endl;
      std::cout << vote_results << std::endl;
    }

  }

  return ;

};


// trace back testing 
// int main(){

//   std::string MAP = "../dao-map/den001d.map";
//   // std::cout << MAP << std::endl;
//   std::string MAP_NAME = MAP.substr(MAP.size()-11,7);
//   std::cout << MAP_NAME << std::endl;
//   AdjMatrix adj_matrix;
//   std::vector<NodePtr> node_list;
//   std::vector<size_t> trace;
//   std::mt19937 rng(std::clock());

//   heuristic h = h_functor(0);
//   node_order order = more_than_specific(0);
//   std::cout << "contructing map for " << MAP_NAME << "...";
//   get_nodes_ASCII(MAP, node_list, adj_matrix);
//   ASTAR A(adj_matrix, node_list);
//   std::cout << "done" << std::endl << "graph size: " << adj_matrix.size() << std::endl;

//   size_t source = rng() % adj_matrix.size();
//   size_t target = rng() % adj_matrix.size();
//   std::cout << "running A from " << source << " to " << target << "...";
  
//   if(A(source, target, h, order, trace) == nullptr){
//     std::cout << "no path found" << std::endl;
//     return 0;
//   };
//   std::cout << "done" << std::endl;

//   std::cout << "trace len: " << trace.size() << std::endl;
//   // for(auto &i : trace){ std::cout << node_list[i]->x << " " << node_list[i]->y << std::endl;}
//   std::cout << std::endl;

//   std::cout << "writing to file...";
//   std::ofstream out_file(MAP_NAME + ".txt");
//   for(auto &i : trace){ out_file << node_list[i]->x << " " << node_list[i]->y << std::endl;}
//   std::cout << "done" << std::endl;

//   out_file.close();
//   return 0;
// }


int main(int argc, char **arg){

  // PARAMETERS
  std::string VOTING_METHOD = "borda";
  std::string MAP = "../dao-map/den001d.map";
  std::string HEURISTIC = "euclidean";
  // std::string HEURISTIC = "haversine";
  // std::string MAP = "../USA-ROAD/USA=ROAD-BAY";
  std::string CHILD_METHOD = "combined";
  size_t  K = 10,
          T = 5,
          source,
          target;
  bool visualize = true;
  // END PARAMETERS

  AdjMatrix adj_matrix;
  std::vector<NodePtr> node_list;
  // std::cout << MAP.substr(MAP.length()-3) << std::endl;
  std::cout << "Constructing Graph...";

  if(MAP.substr(MAP.length()-3, 3) == "map"){                 // ASCII MAP CONSTRUCTOR
    get_nodes_ASCII(MAP, node_list, adj_matrix);
    return 0;

  } else{                                      // ROAD MAP CONSTRUCTOR
    std::vector<std::string> obj_files;
    std::vector<Edge> edge_list;
    size_t graph_size;
    std::string map_name = MAP.substr(-3);
    if(map_name[0] == '-'){ map_name = map_name.substr(-2);}
    // get all files from the folder
    for(auto file_iter : std::__fs::filesystem::directory_iterator(MAP)){obj_files.push_back(file_iter.path());}
    std::string NODE_FILE; // isolating the node file
    for(auto file = obj_files.begin(); file != obj_files.end(); file++){
      if(file->substr(file->size()-2) == "co"){
        NODE_FILE = *file;
        obj_files.erase(file); //remove from other files
        break;
      }
    }
  
    load_gr_files(obj_files, edge_list, graph_size);
    get_nodes_ROAD(NODE_FILE, node_list);

    //add thrid and delay objective if they don't exist
    bool has_third = false, has_delay = false;
    for(auto &f: obj_files){
      if(f == "USA-road-3." + map_name + ".gr"){ has_third = true;}
      if(f == "USA-road-DELAY."+ map_name + ".gr"){ has_delay = true;}
    }
    if(!has_third){add_third_objective(map_name, edge_list, node_list);}
    if(!has_delay){add_delay_objective(map_name, edge_list, node_list);} 
 
    adj_matrix = AdjMatrix(graph_size, edge_list);
 
  }

  std::cout << " Finished" << std::endl 
            << " Graph Size: " << adj_matrix.size() << ", objective count: " << adj_matrix.get_obj_count() << std::endl;


  struct::log LOG(MAP, source, target, VOTING_METHOD, CHILD_METHOD);
  std::mt19937 rng(std::clock());
  // source = rng() % adj_matrix.size();
  // target = rng() % adj_matrix.size();
  source = rng() % adj_matrix.size();
  target = rng() % adj_matrix.size();

  size_t s_x = node_list[source]->x,
         s_y = node_list[source]->y,
         t_x = node_list[target]->x,
         t_y = node_list[target]->y;
  heuristic h;
  if(HEURISTIC == "euclidean"){
    h = h_functor(0);
  } else if (HEURISTIC == "haversine"){
    h = h_functor(1);
  }

  std::fstream ifs(MAP);
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

  size_t id = 1;
  double x, y;

  std::vector<std::vector<char>> map(height, std::vector<char>(width));

  // Text file into matrix
  for(int i = 0; i < height; i++){
    std::getline(ifs, temp);
    for(int j = 0; j < width; j++){
      map[i][j] = temp[j];
      if(map[i][j] == '.'){
        if(i == s_x && j == s_y){
          map[i][j] = '!';
        }
        if(i == t_x && j == t_y){
          map[i][j] = '?';
        }
      }
      std::cout << map[i][j];
    }
    std::cout << std::endl;
  }

  if(CHILD_METHOD == "conscious"){
    VBEA_CONSCIOUS(LOG, adj_matrix, node_list, h, source, target, VOTING_METHOD, K, T, visualize);
  } else { // combined
    VBEA_COMBINED(LOG, adj_matrix, node_list, h, source, target, VOTING_METHOD, K, T, visualize);
  }


  // writingn
  std::ofstream OUT_FILE("testing.json");

  write_record(OUT_FILE, LOG);

  return 0;
}
