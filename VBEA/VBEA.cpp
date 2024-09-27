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


void UNCAPPED_VBEA_COMBINED(struct::log &LOG, const AdjMatrix &adj_matrix, std::vector<NodePtr> node_list, heuristic &h,  const size_t source, const size_t target, const std::string voting_method, const size_t cutoff_time, const size_t K, const size_t T = 0, const bool visualize = false){
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
    std::cout << i << std::endl;
    node_order order = more_than_specific(i);
    front.push_back(A(source, target, h, order)->g);
    std::cout << "===========" << std::endl;

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
    for(int i = LOG.fronts.size() ; i <= T; i++){
      LOG.fronts.push_back({});
      LOG.norm_fronts.push_back({});

      LOG.norm_d_scores.push_back({-1});
      LOG.raw_d_scores.push_back({-1});

      LOG.winner.push_back({-1});
      LOG.norm_winner.push_back({-1});
      LOG.winner_norm_d_score.push_back(-1);
      LOG.winner_raw_d_score.push_back(i);

      LOG.run_times.push_back(-1);

    }
    return ;

  } else {  
    LOG.fronts.push_back(front);
    LOG.norm_fronts.push_back(normalized_front);

    LOG.norm_d_scores.push_back(d_score);
    LOG.raw_d_scores.push_back(calculate_d_score(front));

    LOG.winner.push_back(front[vote_results[0]]);
    LOG.norm_winner.push_back(normalized_front[vote_results[0]]);
    LOG.winner_norm_d_score.push_back(d_score[vote_results[0]]);
    LOG.winner_raw_d_score.push_back(LOG.raw_d_scores.back()[vote_results[0]]);

    LOG.run_times.push_back(end_t);
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
    std::swap(temp,front); // why?
    //front now is now ordered according to its rank 

    // std::cout << "f: " << front.size() << std::endl
    //           << "w: " << weight_sets.size() << std::endl;
    //creating child population
    WEIGHTED_ASTAR WA(adj_matrix, node_list);
    auto order = node_order(more_than_specific(0));
    for(int i = 0; i < K && i < weight_sets.size(); i++){
      // std::cout << i << std::endl;
      WA.update_weight(weight_sets[i]);
      front.push_back(WA(source, target, h, order)->g);
      // std::cout << "===========" << std::endl;

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
        LOG.run_times.push_back(-1);
        LOG.fronts.push_back({});
        LOG.norm_fronts.push_back({});
        LOG.norm_d_scores.push_back({-1});
        LOG.raw_d_scores.push_back({-1});
        LOG.winner.push_back({});
        LOG.winner_norm_d_score.push_back({});
        LOG.winner_raw_d_score.push_back({});
      }

      return ;
    } else {  
      LOG.run_times.push_back(end_t);
      LOG.fronts.push_back(front);
      LOG.norm_fronts.push_back(normalized_front);
      LOG.norm_d_scores.push_back(d_score);
      LOG.winner.push_back(front[vote_results[0]]);
      LOG.norm_winner.push_back(normalized_front[vote_results[0]]);
      LOG.winner_norm_d_score.push_back(d_score[vote_results[0]]);
    }
  }
}


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

void UNCAPPED_VBEA_CONSCIOUS(struct::log &LOG, const AdjMatrix &adj_matrix, std::vector<NodePtr> node_list, heuristic &h,  const size_t source, const size_t target, const std::string voting_method, const size_t cutoff_time, const size_t K, const size_t T = 0, const bool visualize = false){
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
    std::cout << i << std::endl;
    node_order order = more_than_specific(i);
    front.push_back(A(source, target, h, order)->g);
    std::cout << "===========" << std::endl;

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
    for(int i = LOG.fronts.size() ; i <= T; i++){
      LOG.fronts.push_back({});
      LOG.norm_fronts.push_back({});

      LOG.norm_d_scores.push_back({-1});
      LOG.raw_d_scores.push_back({-1});

      LOG.winner.push_back({-1});
      LOG.norm_winner.push_back({-1});
      LOG.winner_norm_d_score.push_back(-1);
      LOG.winner_raw_d_score.push_back(i);

      LOG.run_times.push_back(-1);

    }
    return ;

  } else {  
    LOG.fronts.push_back(front);
    LOG.norm_fronts.push_back(normalized_front);

    LOG.norm_d_scores.push_back(d_score);
    LOG.raw_d_scores.push_back(calculate_d_score(front));

    LOG.winner.push_back(front[vote_results[0]]);
    LOG.norm_winner.push_back(normalized_front[vote_results[0]]);
    LOG.winner_norm_d_score.push_back(d_score[vote_results[0]]);
    LOG.winner_raw_d_score.push_back(LOG.raw_d_scores.back()[vote_results[0]]);

    LOG.run_times.push_back(end_t);
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

    // std::cout << "f: " << front.size() << std::endl
    //           << "w: " << weight_sets.size() << std::endl;
    //creating child population

    WEIGHTED_ASTAR WA(adj_matrix, node_list);
    auto order = node_order(more_than_specific(0));
    for(int i = 0; i < K && i < weight_sets.size(); i++){
      std::cout << i << std::endl;
      WA.update_weight(weight_sets[i]);
      int f = select_focus(weight_sets[i]);
      front.push_back(WA(source, target, h, order)->g);
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
        LOG.run_times.push_back(-1);
        LOG.fronts.push_back({});
        LOG.norm_fronts.push_back({});
        LOG.norm_d_scores.push_back({-1});
        LOG.raw_d_scores.push_back({-1});
        LOG.winner.push_back({});
        LOG.winner_norm_d_score.push_back({});
        LOG.winner_raw_d_score.push_back({});
      }

      return ;
    } else {  
      LOG.run_times.push_back(end_t);
      LOG.fronts.push_back(front);
      LOG.norm_fronts.push_back(normalized_front);
      LOG.norm_d_scores.push_back(d_score);
      LOG.winner.push_back(front[vote_results[0]]);
      LOG.norm_winner.push_back(normalized_front[vote_results[0]]);
      LOG.winner_norm_d_score.push_back(d_score[vote_results[0]]);
    }
  }
}


// void UNCAPPED_VBEA_CONCIOUS(struct::log &LOG, const AdjMatrix &adj_matrix, std::vector<NodePtr> node_list, heuristic &h,  const size_t source, const size_t target, const std::string voting_method, const size_t focus, const size_t cutoff_time, const size_t K, const size_t T = 0, const bool visualize = false);
// K: size of the child population, 
// T: number of generations, limit 

void VBEA_COMBINED(struct::log &LOG, const AdjMatrix &adj_matrix, std::vector<NodePtr> node_list, heuristic &h,  const size_t source, const size_t target, const std::string voting_method, const size_t cutoff_time, const size_t K, const size_t T = 0, const bool visualize = false){
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
    std::cout << i << std::endl;
    std::cout << "===========" << std::endl;
    node_order order = more_than_specific(i);
    front.push_back(A(source, target, h, order)->g);
  }
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
    for(int i = LOG.fronts.size() ; i <= T; i++){
      LOG.fronts.push_back({});
      LOG.norm_fronts.push_back({});

      LOG.norm_d_scores.push_back({-1});
      LOG.raw_d_scores.push_back({-1});

      LOG.winner.push_back({-1});
      LOG.norm_winner.push_back({-1});
      LOG.winner_norm_d_score.push_back(-1);
      LOG.winner_raw_d_score.push_back(i);

      LOG.run_times.push_back(-1);

    }
    return ;

  } else {  
    LOG.fronts.push_back(front);
    LOG.norm_fronts.push_back(normalized_front);

    LOG.norm_d_scores.push_back(d_score);
    LOG.raw_d_scores.push_back(calculate_d_score(front));

    LOG.winner.push_back(front[vote_results[0]]);
    LOG.norm_winner.push_back(normalized_front[vote_results[0]]);
    LOG.winner_norm_d_score.push_back(d_score[vote_results[0]]);
    LOG.winner_raw_d_score.push_back(LOG.raw_d_scores.back()[vote_results[0]]);

    LOG.run_times.push_back(end_t);
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
    while(weight_sets.size() < K){    // fill the weight_sets with random weight sets if there are not enough
      std::uniform_real_distribution<double> rng(0.0, 1.0);
      std::mt19937 generator(std::clock());

      weight_sets.push_back(std::vector<double>(b, rng(generator)));
    }

    std::swap(temp,front); // why?
    //front now is now ordered according to its rank 

    // std::cout << "f: " << front.size() << std::endl
    //           << "w: " << weight_sets.size() << std::endl;
    //creating child population
    WEIGHTED_ASTAR WA(adj_matrix, node_list);
    auto order = node_order(more_than_specific(0));
    for(int i = 0; i < K && i < weight_sets.size(); i++){
      std::cout << i << std::endl;
      WA.update_weight(weight_sets[i]);
      front.push_back(WA(source, target, h, order)->g);
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
        LOG.run_times.push_back(-1);
        LOG.fronts.push_back({});
        LOG.norm_fronts.push_back({});
        LOG.norm_d_scores.push_back({-1});
        LOG.raw_d_scores.push_back({-1});
        LOG.winner.push_back({});
        LOG.winner_norm_d_score.push_back({});
        LOG.winner_raw_d_score.push_back({});
      }

      return ;
    } else {  
      LOG.run_times.push_back(end_t);
      LOG.fronts.push_back(front);
      LOG.norm_fronts.push_back(normalized_front);
      LOG.norm_d_scores.push_back(d_score);
      LOG.winner.push_back(front[vote_results[0]]);
      LOG.norm_winner.push_back(normalized_front[vote_results[0]]);
      LOG.winner_norm_d_score.push_back(d_score[vote_results[0]]);
    }
  }
}

// WEIGHTED COMBINED VERSION
void VBEA_COMBINED(struct::log &LOG, const AdjMatrix &adj_matrix, std::vector<NodePtr> node_list, heuristic &h,  const size_t source, const size_t target, const std::string voting_method, const size_t K, const size_t T = 0, const bool visualize = false){
  std::cout << source << " " << target << std::endl;
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
    vote_results = vote(voting_method, normalized_front, d_score);

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

}

void VBEA_CONSCIOUS(struct::log &LOG, const AdjMatrix &adj_matrix, std::vector<NodePtr> node_list, heuristic &h,  const size_t source, const size_t target, const std::string voting_method, const size_t K, const size_t T = 0, const bool visualize = false){
  auto start_t = std::chrono::high_resolution_clock::now();
  std::uniform_real_distribution<double> rng(0.0, 1.0);
  std::mt19937 generator(std::clock());
  WEIGHTED_ASTAR WA(adj_matrix, node_list);



  int b = adj_matrix.get_obj_count();
  
// BEGIN GEN 0
  ASTAR A(adj_matrix, node_list);
  std::vector<std::vector<double>> front;
  std::vector<std::vector<double>> normalized_front;
  std::vector<double> d_score;
  std::vector<int> vote_results;

  // the initial decomompositon
  for(int i = 0; i < b; i++){
    std::cout << i << std::endl;
    node_order order = more_than_specific(i);
    front.push_back(A(source, target, h, order)->g);
    std::cout << "=====" << std::endl;
  }
  // if K > adj_matrix.size(), then create random weight sets to fill up the population
  if(K > b){
    node_order order = more_than_specific(0);
    for(size_t i = b; i < K; i++){
      std::cout << i << std::endl;
      std::vector<double> rand_weight(b, rng(generator));
      WA.update_weight(rand_weight);
      int f = select_focus(rand_weight);
      front.push_back(WA(source, target, h, order, f)->g);
      std::cout << "=====" << std::endl;
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
    if(weight_sets.size() < K){    // fill the weight_sets with random weight sets if there are not enough
      node_order order = more_than_specific(0);
      for(int i = weight_sets.size(); i < K; i++){
        std::cout << i << std::endl;
        std::vector<double> rand_weight(b, rng(generator));
        WA.update_weight(rand_weight);
        int f = select_focus(rand_weight);
        front.push_back(WA(source, target, h, order, f)->g);
        std::cout << "=====" << std::endl;
      }
    }

    //creating child population
    WEIGHTED_ASTAR WA(adj_matrix, node_list);
    auto order = node_order(more_than_specific(0));
    int f; // focus
    for(int i = 0; i < K; i++){
      std::cout << i << std::endl;
      WA.update_weight(weight_sets[i]);
      int f = select_focus(weight_sets[i]);
      front.push_back(WA(source, target, h, order)->g);
      std::cout << "===========" << std::endl;

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


void road_instnaces_runner(const std::string query_file, size_t graph_size, std::vector<Edge> &edge_list, std::vector<NodePtr> &node_list, const size_t K, const size_t T, const std::string voting_method, const size_t cutoff_time, std::vector<struct::log> &LOGS){

  std::vector<std::vector<size_t>> queries;
  load_road_instances(query_file, queries);

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
    struct::log a(source, target); 
    UNCAPPED_VBEA_CONSCIOUS(a, adj_matrix, node_list, h, source, target, voting_method, cutoff_time, K, T, true);
    LOGS.push_back(a);
  }  
}

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

// int main(){

//   // PARAMETERS
//   std::string VOTING_METHOD = "borda";
//   // std::string MAP = "dao-map/den001d.map";
//   // std::string HEURISTIC = "euclidean";

//   std::string HEURISTIC = "haversine";
//   std::string MAP = "USA-road/USA-road-BAY";
//   std::string CHILD_METHOD = "combined";
//   size_t  K = 10,
//           T = 5,
//           source,
//           target;
//   bool visualize = true;
//   // END PARAMETERS
//   // OPTIONAL
//   size_t cutoffTime = 300; // in seconds
//   // END OPTIONAL

//   std::vector<std::string> obj_files;
//   std::vector<Edge> edge_list;
//   size_t graph_size;
//   std::string map_name = MAP.substr(MAP.length() - 3,3);
//   std::cout << map_name << std::endl;
//   if(map_name[0] == '-'){ map_name = MAP.substr(MAP.length() - 2, 2); std::cout << map_name << std::endl;} // for NY

//   // load all the files form the DAO folder to seperate the objective folders from node folder.
//     for(auto file_iter : std::__fs::filesystem::directory_iterator(MAP)){obj_files.push_back(file_iter.path());}
//     std::string NODE_FILE; // isolating the node file
//     for(auto file = obj_files.begin(); file != obj_files.end(); file++){
//       if(file->substr(file->size()-2) == "co"){
//         NODE_FILE = *file;
//         obj_files.erase(file); //removing node file from the objective files
//         break;
//       }
//   }
//   std::cout << NODE_FILE << std::endl;
//   for(auto &iter: obj_files){
//     std::cout << iter << std::endl;
//   }

//   std::vector<NodePtr> node_list;
//   AdjMatrix adj_matrix;
//   load_gr_files(obj_files, edge_list, graph_size);
//   get_nodes_ROAD(NODE_FILE, node_list);

//   std::cout << "Graph size: " << graph_size << std::endl;

//   // add thrid and delay objective if they don't exist
//    bool has_third = false, has_delay = false;
//    for(auto &f: obj_files){
//      if(f == "USA-road/USA-road-" + map_name + "/USA-road-3." + map_name + ".gr"){ has_third = true;}
//      if(f == "USA-road/USA-road-" + map_name + "/USA-road-DELAY."+ map_name + ".gr"){ has_delay = true;}
//    }
//    if(!has_third){add_third_objective(map_name, edge_list, node_list);}
//    if(!has_delay){add_delay_objective(map_name, edge_list, node_list);} 

//   adj_matrix = AdjMatrix(graph_size, edge_list);
//   std::cout << "Objectives: " << adj_matrix.get_obj_count() << std::endl;

//   std::mt19937 rng(std::clock());
//   source = rng() % adj_matrix.size() - 1;
//   target = rng() % adj_matrix.size() - 1;

//   struct::log LOG();

//   return 0;
  
// }


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

void wrapper(){
  std::string VOTING_METHOD;
  std::string HEURISTIC = "haversine";
  std::string CHILD_METHOD = "combined";
  std::vector<std::string> ALL_VOTING_METHODS = {"condorent", "combined_approval"};
  size_t  K = INT_MAX,
          T = 5,
          source,
          target;
  bool visualize = true;
  // END PARAMETERS
  // OPTIONAL
  size_t cutoff_time = 300000; // in milliseconds
  // END OPTIONAL

  std::vector<std::string> suffixs = {"BAY", "FLA", "COL", "NY"};
  std::string prefix = "USA-ROAD/USA-ROAD-";


  for(auto &v: ALL_VOTING_METHODS){

    VOTING_METHOD = v;
    for(auto &s : suffixs){ // for loop for mao
    
      std::vector<std::string> obj_files;
      std::vector<Edge> edge_list;
      std::vector<NodePtr> node_list;
      size_t graph_size;
      std::string DIR = prefix + s;

      for(auto file_iter : std::__fs::filesystem::directory_iterator(DIR)){obj_files.push_back(file_iter.path());}
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
      std::cout << "Obj files:" << std::endl;
      for(auto &f: obj_files){
        std::cout << f << std::endl;
      }

      std::cout << "child method: " << CHILD_METHOD << std::endl;

    
      load_gr_files(obj_files, edge_list, graph_size);
      get_nodes_ROAD(NODE_FILE, node_list);

      //add thrid and delay objective if they don't exist
       bool has_third = false, has_delay = false;
       for(auto &f: obj_files){
         if(f == "USA-ROAD/USA-ROAD-" + s + "/USA-road-3." + s + ".gr"){ has_third = true;}
         if(f == "USA-ROAD/USA-ROAD-" + s + "/USA-road-DELAY."+ s + ".gr"){ has_delay = true;}
       }
       if(!has_third){add_third_objective(s, edge_list, node_list);}
       if(!has_delay){add_delay_objective(s, edge_list, node_list);} 
     

        std::string query_file = "USA-ROAD/instances/" + s + "_instances.txt";

        std::vector<struct::log> LOGS;

        road_instnaces_runner(query_file, graph_size, edge_list, node_list, K, T, VOTING_METHOD, cutoff_time, LOGS);

        write_all_records_alt(LOGS, s, T, K, VOTING_METHOD, CHILD_METHOD);


      } // end map for loop
  }
  
}


void ascii_wrapper(){
  
  std::string VOTING_METHOD = "";
  std::string CHILD_METHOD = "conscious";
  std::vector<std::string> V_METHODS = {"range", "borda", "condorcet", "combined_approval"};
  heuristic h = h_functor(0);
  size_t  K = INT_MAX,
          T = 5,
          source,
          target;
  // END PARAMETERS
  // OPTIONAL
  size_t cutoff_time = 180000; // in milliseconds

  std::string instance_file = "dao-uniform-5.txt";
  std::unordered_map<std::string, std::vector<std::vector<size_t>>> query = load_asci_queries(instance_file);
  // for(auto &inst: query){
  //   std::cout << inst.first << std::endl;
  //   std::vector<std::vector<size_t>> source_target = inst.second;
  //   for(auto &i: source_target){
  //     source = i[0];
  //     target = i[1];
  //     std::cout << source << " " << target << std::endl;
  //   }
  // }
  

  for(auto &v: V_METHODS){
    VOTING_METHOD = v;
    std::cout << v << std::endl;
    std::vector<struct::log> LOGS;

    for(auto &inst: query){
      std::string map_file = inst.first; 
      std::vector<std::vector<size_t>> source_target = inst.second;
      size_t source, target;
      std::vector<NodePtr> node_list;
      std::vector<Edge> edge_list;
      size_t graph_size;
      load_ascii_file(map_file, edge_list, node_list, graph_size);
      AdjMatrix adj_matrix(graph_size, edge_list);
    
      for(auto &i: source_target){
        source = i[0];
        target = i[1];
        struct::log a(source, target);      

        if(CHILD_METHOD == "combined"){
          UNCAPPED_VBEA_COMBINED(a, adj_matrix, node_list, h, source, target, VOTING_METHOD, cutoff_time, K, T, true);
        } else{
          UNCAPPED_VBEA_CONSCIOUS(a, adj_matrix, node_list, h, source, target, VOTING_METHOD, cutoff_time, K, T, true);
        }

        LOGS.push_back(a);
      }

      write_all_records_alt(LOGS, "", T, K, VOTING_METHOD, CHILD_METHOD);
    }
  }
}

int main(int argc, char **arg){


  // inst_maker();
  // return 0;

  ascii_wrapper();
  return 0;
  // PARAMETERS
  std::string VOTING_METHOD = "range";
  // std::string MAP = "dao-map/den001d.map";
  // std::string HEURISTIC = "euclidean";
  std::string HEURISTIC = "haversine";
  std::string MAP = "USA-ROAD/USA-ROAD-BAY";
  std::string CHILD_METHOD = "conscious";
  size_t  K = 10,
          T = 5,
          source,
          target;
  bool visualize = true;
  // END PARAMETERS
  // OPTIONAL
  size_t cutoff_time = 300000; // in milliseconds
  // END OPTIONAL

  AdjMatrix adj_matrix;
  std::vector<NodePtr> node_list;
  std::cout << "Constructing Graph..." << std::endl;

  if(MAP.substr(MAP.length()-3, 3) == "map"){                 // ASCII MAP CONSTRUCTOR
    get_nodes_ASCII(MAP, node_list, adj_matrix);

  } else{                                      // ROAD MAP CONSTRUCTOR

    std::vector<std::string> obj_files;
    std::vector<Edge> edge_list;
    size_t graph_size;
    std::string map_name = MAP.substr(MAP.size()-3);
    if(map_name[0] == '-'){ map_name = map_name.substr(1);}
    // load all the files form the DAO folder to seperate the objective folders from node folder.
    for(auto file_iter : std::__fs::filesystem::directory_iterator(MAP)){obj_files.push_back(file_iter.path());}
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
    std::cout << "Obj files:" << std::endl;
    for(auto &f: obj_files){
      std::cout << f << std::endl;
    }
  
    load_gr_files(obj_files, edge_list, graph_size);
    get_nodes_ROAD(NODE_FILE, node_list);
    // std::cout << edge_list.front().cost.size() << std::endl;
    // for(auto &i : edge_list.front().cost){
    //   std::cout << i << ", ";
    // }
    // std::cout << std::endl;    


    //add thrid and delay objective if they don't exist
   bool has_third = false, has_delay = false;
   for(auto &f: obj_files){
     if(f == "USA-ROAD/USA-ROAD-" + map_name + "/USA-road-3." + map_name + ".gr"){ has_third = true;}
     if(f == "USA-ROAD/USA-ROAD-" + map_name + "/USA-road-DELAY."+ map_name + ".gr"){ has_delay = true;}
   }
   if(!has_third){add_third_objective(map_name, edge_list, node_list);}
   if(!has_delay){add_delay_objective(map_name, edge_list, node_list);} 
    // adj_matrix = AdjMatrix(graph_size, edge_list);
    std::string query_file = "USA-ROAD/instances/BAY_instances.txt";

    std::vector<struct::log> LOGS;

    road_instnaces_runner(query_file, graph_size, edge_list, node_list, K, T, VOTING_METHOD, cutoff_time, LOGS);

    write_all_records_alt(LOGS, map_name, T, K, VOTING_METHOD, CHILD_METHOD);

  }

  // std::mt19937 rng(std::clock());
  // source = rng() % adj_matrix.size() - 1;
  // target = rng() % adj_matrix.size() - 1;

  // heuristic h;
  // if(HEURISTIC == "euclidean"){
  //   h = h_functor(0);
  // } else if (HEURISTIC == "haversine"){
  //   h = h_functor(1);
  // }

// // MAP DISPLAY (REMOVE FOR FINAL)

//   std::fstream ifs(MAP);
//   std::string temp;
//   int height;
//   int width;

//   //get height and width of the graph
//   std::getline(ifs, temp);
//   // std::cout << temp << std::endl;
//   std::getline(ifs, temp);
//   height = stoi(temp.substr(7));
//   std::getline(ifs, temp);
//   width = stoi(temp.substr(6));
//   std::getline(ifs, temp);

//   size_t id = 1;
//   double x, y;

//   std::vector<std::vector<char>> map(height, std::vector<char>(width));

//   // Text file into matrix
//   for(int i = 0; i < height; i++){
//     std::getline(ifs, temp);
//     for(int j = 0; j < width; j++){
//       map[i][j] = temp[j];
//       if(map[i][j] == '.'){
//         if(i == s_x && j == s_y){ // source
//           map[i][j] = '!';
//         }
//         if(i == t_x && j == t_y){ // target
//           map[i][j] = '?';
//         }
//       }
//       std::cout << map[i][j];
//     }
//     std::cout << std::endl;
//   }
//   ifs.close();
// // END OF MAP DISPLAY

// return 0;

  // if(CHILD_METHOD == "conscious"){
  //   // VBEA_CONSCIOUS(LOG, adj_matrix, node_list, h, source, target, VOTING_METHOD, K, T, visualize);
  // } else { // combined
  //   // VBEA_COMBINED(LOG, adj_matrix, node_list, h, source, target, VOTING_METHOD, K, T, visualize);
  //   UNCAPPED_VBEA_COMBINED(LOG, adj_matrix, node_list, h , source, target, VOTING_METHOD, cutoffTime, K, T, true);

  // }



  return 0;
}
