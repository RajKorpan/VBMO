#include <algorithm>
#include <chrono>
#include <climits>
#include <cstddef>
#include <random>
#include <string>
#include <filesystem>
#include <vector>
#include <iostream>
#include <stack>

#include "inc/IOutili.hpp"
#include "inc/graph.hpp"
#include "inc/utili.hpp"
#include "inc/v.hpp"


// move to voting?
std::vector<double> random_weight(const size_t &b){
  std::vector<double> new_weight;
  std::uniform_real_distribution<double> rng(0.0, 1.0);
  std::mt19937 generator(std::clock());
  for(int i = 0; i < b; i++){
    new_weight.push_back(rng(generator));
  }

  return new_weight;
}


std::vector<double> UNCAPPED_VBEA_COMBINED(struct::log &LOG, const AdjMatrix &adj_matrix, std::vector<NodePtr> node_list, heuristic &h,  const size_t source, const size_t target, const std::string voting_method, const size_t cutoff_time, const size_t T = 0, const bool visualize = false){
  auto start_t = std::chrono::high_resolution_clock::now();
  int end_t;
  int total_t = 0;
  int b = adj_matrix.get_obj_count();

  // BEGIN GEN 0
  ASTAR A(adj_matrix, node_list);
  std::vector<std::vector<double>> front;
  std::vector<std::vector<double>> normalized_front;
  std::vector<double> d_score,
                      norm_d_score;
  std::vector<int> vote_results;

  // creating initial population
  for(int i = 0; i < b; i++){
    node_order order = more_than_specific(i);
    front.push_back(A(source, target, h, order)->g);
  }
  remove_duplicates(front);
  normalized_front = normalize_matrix(front);
  d_score = calculate_d_score(normalized_front);
  vote_results = vote(voting_method, normalized_front, d_score);

  end_t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();
  total_t += end_t;
  
  if(visualize){
    std::cout << "-----gen0-----" << std::endl;
    for(int i = 0; i < front.size(); i++){
        std::cout << "P_" << i << " " << front[i] << " | " << normalized_front[i] << " d: " << d_score[i] << std::endl;
    }
    std::cout << "voting results:" << std::endl;
    std::cout << vote_results << std::endl;
    std::cout << "time: " << end_t <<  " ms " << std::endl;
    std::cout << "current run time: " << total_t << " ms" << std::endl;
  }
  // END GEN 0
  
  // WRITE GEN 0
  if(total_t >= cutoff_time){
    std::cout << "TIME LIMIT REACHED" << std::endl;
    return {}; // failed to make initial generation

  } else {  
    LOG.fronts.push_back(front);
    LOG.norm_fronts.push_back(normalized_front);
    LOG.raw_d_scores.push_back(calculate_d_score(front));
    LOG.norm_d_scores.push_back(d_score);
    LOG.winner.push_back(front[vote_results[0]]);
    LOG.norm_winner.push_back(normalized_front[vote_results[0]]);
    LOG.winner_raw_d_score.push_back(LOG.raw_d_scores.back()[vote_results[0]]);
    LOG.winner_norm_d_score.push_back(d_score[vote_results[0]]);
    LOG.run_times.push_back(total_t);
  }

  // BEGIN GEN i
  for(int t = 1; t <= T; t++){
    start_t = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<double>> weight_sets;
    for(int i = 0; i < normalized_front.size(); i++){ // save only the top K solutions from the previous generation
      weight_sets.push_back(complement_weight_set(normalized_front[i]));
    }

    //creating child population for gen i
    WEIGHTED_ASTAR WA(adj_matrix, node_list);
    auto order = node_order(more_than_specific(0));
    for(int i = 0; i < weight_sets.size(); i++){
      WA.update_weight(weight_sets[i]);
      front.push_back(WA(source, target, h, order)->g);
    }
    remove_duplicates(front);
    front = non_dominated_filter(front);
    normalized_front = normalize_matrix(front);
    d_score = calculate_d_score(normalized_front);
    vote_results = vote(voting_method, normalized_front, d_score);

    end_t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();
    total_t += end_t;

    if(visualize){
      std::cout << "-----gen" << t << "-----" << std::endl;
      for(int i = 0; i < front.size(); i++){
          std::cout << "P_" << i << " " << front[i] << " | " << normalized_front[i] << " d: " << d_score[i] << std::endl;
      }
      std::cout << "voting results:" << std::endl;
      std::cout << vote_results << std::endl;
      std::cout << "time: " << end_t << " ms" << std::endl;
      std::cout << "current run time: " << total_t << " ms" << std::endl;

    }
    if(total_t >= cutoff_time){
      std::cout << "TIME LIMIT REACHED" << std::endl;
      // FILL LOG with sentinal values
      for(int i = LOG.fronts.size() ; i < T; i++){
     }

      return {}; // the best solutions from the previous generation.
    } else {  
      LOG.fronts.push_back(front);
      LOG.norm_fronts.push_back(normalized_front);
      LOG.raw_d_scores.push_back(calculate_d_score(front));
      LOG.norm_d_scores.push_back(d_score);
      LOG.winner.push_back(front[vote_results[0]]);
      LOG.norm_winner.push_back(normalized_front[vote_results[0]]);
      LOG.winner_raw_d_score.push_back(LOG.raw_d_scores.back()[vote_results[0]]);
      LOG.winner_norm_d_score.push_back(d_score[vote_results[0]]);
      LOG.run_times.push_back(total_t);
    }
  }
}

// Move to ulti
size_t select_focus(const std::vector<double> &vec){
  size_t minIndex = 0;
  double minVal = vec[0];
  bool multiMin = false;
  std::stack<size_t> stack;

  for(int i = 1; i < vec.size(); i++){

    if(multiMin){
      if(vec[i] < minVal){
        if(vec[i] == minVal){
          stack.push(i);
        } else{
          stack = std::stack<size_t>();
          minIndex = i;
          minVal = vec[i];
          multiMin = false;
        }
      }
    } else {
      
      if(vec[i] > minVal){
        if(vec[i] == minVal){
          multiMin = true;
          stack.push(minIndex);
          stack.push(i);
        }
      }
    }   
  }

  if(multiMin){ //randomly select amonst
     std::vector<size_t> list;
     while(!stack.empty()){
       list.push_back(stack.top());
       stack.pop();
     }
     std::mt19937 rng(std::clock());
     minIndex = (rng() & list.size());
  }

  return minIndex;

}


void UNCAPPED_VBEA_CONSCIOUS(struct::log &LOG, const AdjMatrix &adj_matrix, std::vector<NodePtr> node_list, heuristic &h,  const size_t source, const size_t target, const std::string voting_method, const size_t cutoff_time, const size_t T = 0, const bool visualize = false){
  auto start_t = std::chrono::high_resolution_clock::now();
  int end_t;
  int total_t = 0;
  int b = adj_matrix.get_obj_count();

  // BEGIN GEN 0
  ASTAR A(adj_matrix, node_list);
  std::vector<std::vector<double>> front;
  std::vector<std::vector<double>> normalized_front;
  std::vector<double> d_score,
                      norm_d_score;
  std::vector<int> vote_results;

  for(int i = 0; i < b; i++){
    node_order order = more_than_specific(i);
    front.push_back(A(source, target, h, order)->g);
  }
  remove_duplicates(front);
  normalized_front = normalize_matrix(front);
  d_score = calculate_d_score(normalized_front);
  vote_results = vote(voting_method, normalized_front, d_score);

  end_t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();
  total_t += end_t;
  
  if(visualize){
    std::cout << "-----gen0-----" << std::endl;
    for(int i = 0; i < front.size(); i++){
        std::cout << "P_" << i << " " << front[i] << " | " << normalized_front[i] << " d: " << d_score[i] << std::endl;
    }
    std::cout << "voting results:" << std::endl;
    std::cout << vote_results << std::endl;
    std::cout << "time: " << end_t <<  " ms " << std::endl;
    std::cout << "current run time: " << total_t << " ms" << std::endl;
  }
  // END GEN 0
  
  // WRITE GEN 0
  if(total_t >= cutoff_time){
    std::cout << "TIME LIMIT REACHED" << std::endl;
    return ;

  } else {  
    LOG.fronts.push_back(front);
    LOG.norm_fronts.push_back(normalized_front);
    LOG.raw_d_scores.push_back(calculate_d_score(front));
    LOG.norm_d_scores.push_back(d_score);
    LOG.winner.push_back(front[vote_results[0]]);
    LOG.norm_winner.push_back(normalized_front[vote_results[0]]);
    LOG.winner_raw_d_score.push_back(LOG.raw_d_scores.back()[vote_results[0]]);
    LOG.winner_norm_d_score.push_back(d_score[vote_results[0]]);
    LOG.run_times.push_back(total_t);

  }

  // BEGIN GEN i
  for(int t = 1; t <= T; t++){
    start_t = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<double>> weight_sets;
    for(int i = 0; i < normalized_front.size(); i++){ // save only the top K solutions from the previous generation
      weight_sets.push_back(complement_weight_set(normalized_front[i]));
    }

    //creating child population for gen i
    WEIGHTED_ASTAR WA(adj_matrix, node_list);
    auto order = node_order(more_than_specific(0));
    int f;
    for(int i = 0; i < weight_sets.size(); i++){
      WA.update_weight(weight_sets[i]);
      f = select_focus(weight_sets[i]);
      front.push_back(WA(source, target, h, order, f)->g);
    }
    remove_duplicates(front);
    front = non_dominated_filter(front);
    normalized_front = normalize_matrix(front);
    d_score = calculate_d_score(normalized_front);
    vote_results = vote(voting_method, normalized_front, d_score);

    end_t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();
    total_t += end_t;

    if(visualize){
      std::cout << "-----gen" << t << "-----" << std::endl;
      for(int i = 0; i < front.size(); i++){
          std::cout << "P_" << i << " " << front[i] << " | " << normalized_front[i] << " d: " << d_score[i] << std::endl;
      }
      std::cout << "voting results:" << std::endl;
      std::cout << vote_results << std::endl;
      std::cout << "time: " << end_t << " ms" << std::endl;
      std::cout << "current run time: " << total_t << " ms" << std::endl;

    }
    if(total_t >= cutoff_time){
      std::cout << "TIME LIMIT REACHED" << std::endl;
      // FILL LOG with sentinal values

      return ;
    } else {  
      LOG.fronts.push_back(front);
      LOG.norm_fronts.push_back(normalized_front);
      LOG.raw_d_scores.push_back(calculate_d_score(front));
      LOG.norm_d_scores.push_back(d_score);
      LOG.winner.push_back(front[vote_results[0]]);
      LOG.norm_winner.push_back(normalized_front[vote_results[0]]);
      LOG.winner_raw_d_score.push_back(LOG.raw_d_scores.back()[vote_results[0]]);
      LOG.winner_norm_d_score.push_back(d_score[vote_results[0]]);
      LOG.run_times.push_back(total_t);

    }
  }
}


void VBEA_COMBINED(struct::log &LOG, const AdjMatrix &adj_matrix, std::vector<NodePtr> node_list, heuristic &h,  const size_t source, const size_t target, const std::string voting_method, const size_t cutoff_time, const size_t K, const size_t T = 0, const bool visualize = false){
  auto start_t = std::chrono::high_resolution_clock::now();
  int end_t;
  int total_t = 0;
  int b = adj_matrix.get_obj_count();

  // BEGIN GEN 0
  ASTAR A(adj_matrix, node_list);
  WEIGHTED_ASTAR WA(adj_matrix, node_list);
  std::vector<std::vector<double>> front;
  std::vector<std::vector<double>> normalized_front;
  std::vector<double> d_score,
                      norm_d_score;
  std::vector<int> vote_results;

  // creating objective optimized solutions
  for(int i = 0; i < b; i++){
    node_order order = node_order(more_than_specific(i));
    front.push_back(A(source, target, h, order)->g);
  }

  // filling the remaining with soltions created through searching with random weight set
  auto order = node_order(more_than_specific(0));
  std::vector<double> rand_weight(b);
  while(front.size() < K){
    rand_weight = random_weight(b);
    WA.update_weight(rand_weight);
    front.push_back(WA(source, target, h, order)->g);
  }  
  remove_duplicates(front);
  front = non_dominated_filter(front);
  normalized_front = normalize_matrix(front);
  d_score = calculate_d_score(normalized_front);
  vote_results = vote(voting_method, normalized_front, d_score);

  end_t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();
  total_t += end_t;
  
  if(visualize){
    std::cout << "-----gen0-----" << std::endl;
    for(int i = 0; i < front.size(); i++){
        std::cout << "P_" << i << " " << front[i] << " | " << normalized_front[i] << " d: " << d_score[i] << std::endl;
    }
    std::cout << "voting results:" << std::endl;
    std::cout << vote_results << std::endl;
    std::cout << "time: " << end_t <<  " ms " << std::endl;
    std::cout << "current run time: " << total_t << " ms" << std::endl;
  }
  // END GEN 0
  
  // WRITE GEN 0
  if(total_t >= cutoff_time){
    std::cout << "TIME LIMIT REACHED" << std::endl;
    for(int i = LOG.fronts.size() ; i <= T; i++){
    }
    return ;

  } else {  
    LOG.fronts.push_back(front);
    LOG.norm_fronts.push_back(normalized_front);
    LOG.raw_d_scores.push_back(calculate_d_score(front));
    LOG.norm_d_scores.push_back(d_score);
    LOG.winner.push_back(front[vote_results[0]]);
    LOG.norm_winner.push_back(normalized_front[vote_results[0]]);
    LOG.winner_raw_d_score.push_back(LOG.raw_d_scores.back()[vote_results[0]]);
    LOG.winner_norm_d_score.push_back(d_score[vote_results[0]]);
    LOG.run_times.push_back(total_t);
  }

  // BEGIN GEN i
  for(int t = 1; t <= T; t++){
    start_t = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<double>> temp; // order the solutions on their voting rank
    std::vector<std::vector<double>> weight_sets;
    for(int i = 0; i < K && i < front.size(); i++){ // save only the top K solutions from the previous generation
      temp.push_back(front[vote_results[i]]); 
      weight_sets.push_back(complement_weight_set(normalized_front[vote_results[i]]));
    }
    
    std::swap(temp,front); // front now only contains at most the top K candidates

    // creating child population of gen i
    WEIGHTED_ASTAR WA(adj_matrix, node_list);
    auto order = node_order(more_than_specific(0));
    int i;
    for(i = 0; i < K && i < weight_sets.size(); i++){
      WA.update_weight(weight_sets[i]);
      front.push_back(WA(source, target, h, order)->g);
    }
    std::vector<double> rand_weight;
    while(i < K){
      rand_weight = random_weight(b);
      WA.update_weight(rand_weight);
      front.push_back(WA(source, target, h, order)->g);
      i++;
    }
    remove_duplicates(front);
    front = non_dominated_filter(front);
    normalized_front = normalize_matrix(front);
    d_score = calculate_d_score(normalized_front);
    vote_results = vote(voting_method, normalized_front, d_score);

    end_t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();
    total_t += end_t;

    if(visualize){
      std::cout << "-----gen" << t << "-----" << std::endl;
      for(int i = 0; i < front.size(); i++){
          std::cout << "P_" << i << " " << front[i] << " | " << normalized_front[i] << " d: " << d_score[i] << std::endl;
      }
      std::cout << "voting results:" << std::endl;
      std::cout << vote_results << std::endl;
      std::cout << "time: " << end_t << " ms" << std::endl;
      std::cout << "current run time: " << total_t << " ms" << std::endl;

    }
    if(total_t >= cutoff_time){
      std::cout << "TIME LIMIT REACHED" << std::endl;
      // FILL LOG with sentinal values
      for(int i = LOG.fronts.size() ; i < T; i++){
      }

      return ;
    } else {  
      LOG.fronts.push_back(front);
      LOG.norm_fronts.push_back(normalized_front);
      LOG.raw_d_scores.push_back(calculate_d_score(front));
      LOG.norm_d_scores.push_back(d_score);
      LOG.winner.push_back(front[vote_results[0]]);
      LOG.norm_winner.push_back(normalized_front[vote_results[0]]);
      LOG.winner_raw_d_score.push_back(LOG.raw_d_scores.back()[vote_results[0]]);
      LOG.winner_norm_d_score.push_back(d_score[vote_results[0]]);
      LOG.run_times.push_back(total_t);
    }
  }
  
}



void VBEA_CONSCIOUS(struct::log &LOG, const AdjMatrix &adj_matrix, std::vector<NodePtr> node_list, heuristic &h,  const size_t source, const size_t target, const std::string voting_method, const size_t cutoff_time,  const size_t K, const size_t T = 0, const bool visualize = false){
  auto start_t = std::chrono::high_resolution_clock::now();
  int end_t;
  int total_t = 0;
  int b = adj_matrix.get_obj_count();
  // BEGIN GEN 0
  ASTAR A(adj_matrix, node_list);
  WEIGHTED_ASTAR WA(adj_matrix, node_list);
  std::vector<std::vector<double>> front;
  std::vector<std::vector<double>> normalized_front;
  std::vector<double> d_score,
                      norm_d_score;
  std::vector<int> vote_results;

  for(int i = 0; i < b; i++){
    std::cout << i << std::endl;
    node_order order = node_order(more_than_specific(i));
    front.push_back(A(source, target, h, order)->g);
    std::cout << "===========" << std::endl;
  }

  auto order = node_order(more_than_specific(0));
  std::vector<double> rand_weight(b);
  while(front.size() < K){
    std::cout << front.size() << std::endl;
    rand_weight = random_weight(b);
    int f = select_focus(rand_weight);
    WA.update_weight(rand_weight);
    front.push_back(WA(source, target, h, order, f)->g);
    std::cout << "===========" << std::endl;
  }
  
  remove_duplicates(front);
  front = non_dominated_filter(front);
  normalized_front = normalize_matrix(front);
  d_score = calculate_d_score(normalized_front);
  vote_results = vote(voting_method, normalized_front, d_score);

  end_t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();
  total_t += end_t;
  
  if(visualize){
    std::cout << "-----gen0-----" << std::endl;
    for(int i = 0; i < front.size(); i++){
        std::cout << "P_" << i << " " << front[i] << " | " << normalized_front[i] << " d: " << d_score[i] << std::endl;
    }
    std::cout << "voting results:" << std::endl;
    std::cout << vote_results << std::endl;
    std::cout << "time: " << end_t <<  " ms " << std::endl;
    std::cout << "current run time: " << total_t << " ms" << std::endl;
  }
  // END GEN 0
  
  // WRITE GEN 0
  if(total_t >= cutoff_time){
    std::cout << "TIME LIMIT REACHED" << std::endl;
    for(int i = LOG.fronts.size() ; i <= T; i++){
        
    }

    return ;

  } else {  
    LOG.fronts.push_back(front);
    LOG.norm_fronts.push_back(normalized_front);
    LOG.raw_d_scores.push_back(calculate_d_score(front));
    LOG.norm_d_scores.push_back(d_score);
    LOG.winner.push_back(front[vote_results[0]]);
    LOG.norm_winner.push_back(normalized_front[vote_results[0]]);
    LOG.winner_raw_d_score.push_back(LOG.raw_d_scores.back()[vote_results[0]]);
    LOG.winner_norm_d_score.push_back(d_score[vote_results[0]]);
    LOG.run_times.push_back(total_t);

  }

  // BEGIN GEN i
  for(int t = 1; t <= T; t++){
    // std::cout << "-----" << t << "------" << std::endl;
    start_t = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<double>> temp; // order the solutions on their voting rank
    std::vector<std::vector<double>> weight_sets;
    for(int i = 0; i < K && i < front.size(); i++){ // save only the top K solutions from the previous generation
      temp.push_back(front[vote_results[i]]); 
      weight_sets.push_back(complement_weight_set(normalized_front[vote_results[i]]));
    }
    
    std::swap(temp,front); // front now only contains at most the top K candidates

    WEIGHTED_ASTAR WA(adj_matrix, node_list);
    auto order = node_order(more_than_specific(0));
    int i = 0;
    for(i = 0; i < K && i < weight_sets.size(); i++){
      std::cout << i << std::endl;
      WA.update_weight(weight_sets[i]);
      int f = select_focus(weight_sets[i]);
      front.push_back(WA(source, target, h, order)->g);
      std::cout << "===========" << std::endl;
    }
    std::vector<double> rand_weight;
    while(i < K){
      std::cout << i << std::endl;
      rand_weight = random_weight(b);
      WA.update_weight(rand_weight);
      int f = select_focus(rand_weight);
      front.push_back(WA(source, target, h, order, f)->g);
      std::cout << "===========" << std::endl;
      i++;
    }

    remove_duplicates(front);
    front = non_dominated_filter(front);
    normalized_front = normalize_matrix(front);
    d_score = calculate_d_score(normalized_front);
    vote_results = vote(voting_method, normalized_front, d_score);

    end_t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();
    total_t += end_t;

    if(visualize){
      std::cout << "-----gen" << t << "-----" << std::endl;
      for(int i = 0; i < front.size(); i++){
          std::cout << "P_" << i << " " << front[i] << " | " << normalized_front[i] << " d: " << d_score[i] << std::endl;
      }
      std::cout << "voting results:" << std::endl;
      std::cout << vote_results << std::endl;
      std::cout << "time: " << end_t << " ms" << std::endl;
      std::cout << "current run time: " << total_t << " ms" << std::endl;

    }
    if(total_t >= cutoff_time){
      std::cout << "TIME LIMIT REACHED" << std::endl;
      // FILL LOG with sentinal values
      for(int i = LOG.fronts.size() ; i < T; i++){
      }

      return ;
    } else {  
      LOG.fronts.push_back(front);
      LOG.norm_fronts.push_back(normalized_front);
      LOG.raw_d_scores.push_back(calculate_d_score(front));
      LOG.norm_d_scores.push_back(d_score);
      LOG.winner.push_back(front[vote_results[0]]);
      LOG.norm_winner.push_back(normalized_front[vote_results[0]]);
      LOG.winner_raw_d_score.push_back(LOG.raw_d_scores.back()[vote_results[0]]);
      LOG.winner_norm_d_score.push_back(d_score[vote_results[0]]);
      LOG.run_times.push_back(total_t);
    }
  }
  
}

void road_instnaces_runner(const std::string query_file, size_t graph_size, std::vector<Edge> &edge_list, std::vector<NodePtr> &node_list, const size_t K, const size_t T, const std::string voting_method, const std::string child_method, const size_t cutoff_time, std::vector<struct::log> &LOGS){

  std::vector<std::vector<size_t>> queries;
  load_road_instances(query_file, queries);
  std::cout << "loaded instances" << std::endl;

  AdjMatrix adj_matrix(graph_size, edge_list);
  heuristic h = h_functor(1);
  int n = queries.size();
  int i = 0;
  for(auto &q : queries){
    size_t source = q[0],
           target = q[1];
    std::cout << "=====" << std::endl;
    std::cout << "(" << i << "/" << n << ")" << std::endl;
    std::cout << "=====" << std::endl;
    i++;
    struct::log a(source, target, "test"); 

    if(child_method == "combined"){
      // UNCAPPED_VBEA_COMBINED(a, adj_matrix, node_list, h, source, target, voting_method, cutoff_time, K, T, true);
      VBEA_COMBINED(a, adj_matrix, node_list, h, source, target, voting_method, cutoff_time, K, T, true);
    } else {
      // UNCAPPED_VBEA_CONSCIOUS(a, adj_matrix, node_list, h, source, target, voting_method, cutoff_time, K, T, true);
      VBEA_CONSCIOUS(a, adj_matrix, node_list, h, source, target, voting_method, cutoff_time, K, T, true);
    }

    // VBEA_CONSCIOUS(a, adj_matrix, node_list, h, source, target, voting_method, cutoff_time, K, T, true);

    LOGS.push_back(a);
  }  
}



// remove?
void inst_maker(){
  const int N = 5;

  std::default_random_engine generator(1395);
  std::uniform_int_distribution<size_t> urng(0, INT_MAX); // uniform random generator between 0.3 and 0.4
  std::string DIR = "dao-map";
  std::vector<std::string> files;
  heuristic h = h_functor(0);
  node_order order = more_than_specific(0);
  for(auto file_iter : std::__fs::filesystem::directory_iterator(DIR)){files.push_back(file_iter.path());}

  std::unordered_map<std::string, std::vector<std::vector<size_t>>> instances;
  size_t source, target;

  for(auto &f: files){
    std::cout << f << std::endl;
    instances[f];
    std::vector<Edge> edge_list;
    std::vector<NodePtr> node_list;
    size_t graph_size;
    load_ascii_file(f, edge_list, node_list, graph_size);
    AdjMatrix adj_matrix(graph_size, edge_list);
    ASTAR A(adj_matrix, node_list);
    for(int i = 0; i < N; i++){
      source = urng(generator) % adj_matrix.size();
      target = urng(generator) % adj_matrix.size();

      if(A(source, target, h, order) ==  nullptr){
        i--;
      } else {
        std::cout << "  " << source << " " << target << std::endl;
        instances[f].push_back({source, target});
      }
    }
  }


  std::ofstream out_file("dao-uniform-5.txt");

  for(auto &i: instances){
    out_file << i.first << std::endl;
    for(auto &j: i.second){
      out_file << j[0] << " " << j[1] << std::endl;
    }
  }

  out_file.close();
}

NodePtr SOLVER(struct::log &LOG, const AdjMatrix &adj_matrix, const size_t source, const size_t target, const std::vector<NodePtr> &node_list, heuristic h, const std::string VOTING_METHOD, const std::string CHILD_METHOD, const size_t CUTOFF_TIME, const size_t K, const size_t T = 0, const bool visualize = true){

  if(K == INT_MAX){ // UNCAPPED
    if(CHILD_METHOD == "combined"){
      UNCAPPED_VBEA_COMBINED(LOG, adj_matrix, node_list, h, source, target, VOTING_METHOD, CUTOFF_TIME, T, visualize); // WHY IS THERE A
    } else if (CHILD_METHOD == "conscious"){
      UNCAPPED_VBEA_CONSCIOUS(LOG, adj_matrix, node_list, h, source, target, VOTING_METHOD, CUTOFF_TIME, T, visualize);

    }
  } else {         // CAPPED
    if(CHILD_METHOD == "combined"){
      VBEA_COMBINED(LOG, adj_matrix, node_list, h, source, target, VOTING_METHOD, CUTOFF_TIME, K, T, visualize);
    } else if(CHILD_METHOD == "consious"){
      VBEA_CONSCIOUS(LOG, adj_matrix, node_list, h, source, target, VOTING_METHOD, CUTOFF_TIME, K, T, visualize);
    }
    
  }


  return nullptr;
}

void ascii_wrapper(const std::string FILE_NAME, const size_t source, const size_t target, const std::string VOTING_METHOD, const std::string CHILD_METHOD, const size_t CUTOFF_TIME, const size_t K, const size_t T = 0, const bool visualize = true){
  std::vector<NodePtr> node_list;
  std::vector<Edge> edge_list;
  size_t graph_size;
  std::cout << "Constructing Graph..." << std::endl;

  load_ascii_file(FILE_NAME, edge_list, node_list, graph_size);
  AdjMatrix adj_matrix(graph_size, edge_list);
  std::cout << "done." << std::endl;

  heuristic h = h_functor(0);

  std::vector<struct::log> LOGS;
  struct::log LOG(source, target, FILE_NAME);

  SOLVER(LOG, adj_matrix, source, target, node_list, h, VOTING_METHOD, CHILD_METHOD, CUTOFF_TIME, K, T, visualize);
  LOGS.push_back(LOG);

  write_all_records_alt(LOGS, FILE_NAME, T, K, VOTING_METHOD, CHILD_METHOD);
}


void road_wrapper(const std::string FILE_NAME, const size_t source, const size_t target, const std::string VOTING_METHOD, const std::string CHILD_METHOD, const size_t CUTOFF_TIME, const size_t K, const size_t T = 0, const bool visualize = true){
  std::vector<NodePtr> node_list;
  std::vector<Edge> edge_list;
  size_t graph_size;

  std::vector<struct::log> LOGS;

  std::cout << "Constructing Graph..." << std::endl;

  std::vector<std::string> obj_files;

  for(auto file_iter : std::__fs::filesystem::directory_iterator(FILE_NAME)){obj_files.push_back(file_iter.path());}
  std::string NODE_FILE; // isolating the node file
  for(auto file = obj_files.begin(); file != obj_files.end(); file++){
    if(file->substr(file->size()-2) == "co"){
      NODE_FILE = *file;
      obj_files.erase(file); //removing node file from the objective files
      break;
    }
  }

  std::cout << "Node file:" << std::endl;
  std::cout << NODE_FILE << std::endl;
  std::cout << "Obj file(s):" << std::endl;
  for(auto &f: obj_files){
    std::cout << f << std::endl;
  }

  load_gr_files(obj_files, edge_list, graph_size);
  get_nodes_ROAD(NODE_FILE, node_list);
  AdjMatrix adj_matrix(graph_size, edge_list);

  std::cout << "done." << std::endl;

  heuristic h = h_functor(1);

  struct::log LOG(source, target, FILE_NAME);

  SOLVER(LOG, adj_matrix, source, target, node_list, h, VOTING_METHOD, CHILD_METHOD, CUTOFF_TIME, K, T, visualize);

  LOGS.push_back(LOG);
}


// make it read from them command line
int main(int argc, char **arg){

  // PARAM (make program options using boost)
  std::string VOTING_METHOD = "range";
  // std::string MAP = "resources/dao-map/den001d.map";
  std::string MAP = "resources/USA-ROAD/USA-ROAD-COL";
  std::string CHILD_METHOD = "combined";
  size_t  K = 20,
          T = 5,
          source = 113,
          target = 227;
  bool visualize = true;
  size_t CUTOFF_TIME = INT_MAX; // in milliseconds
  // END PRAM

  // THINGS THAT SHOULD BE CHECKED BEFORE RURNNING
  // T is non-negnative
  // K > # of objectives
  // source and target are legitimiate 
  // voting & child method is valid

  if(MAP.substr(MAP.size()-3) == "map"){
    ascii_wrapper(MAP, source, target, VOTING_METHOD, CHILD_METHOD, CUTOFF_TIME, K, T, visualize);
  } else if(MAP.substr(10,3) == "USA"){
    road_wrapper(MAP, source, target, VOTING_METHOD, CHILD_METHOD, CUTOFF_TIME, K, T, visualize);
  }
  
  return 0;
}
