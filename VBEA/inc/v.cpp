#include <algorithm>
#include <climits>
#include <cstdlib>
#include <numeric>
#include <vector>           // vectors
#include <iostream>         // input and output functionality
#include <cmath>            // sqrt(), abs(), and other basic math functions
#include <queue>            // using priority_queue as the openSet for A*

#include "v.hpp"
#include "utili.hpp"

std::vector<double> calculate_d_score(const std::vector<std::vector<double> > &normalized_path_cost){
    std::vector<double> scores;

    for(int i = 0; i < normalized_path_cost.size(); i++){
        double score = 0.0;
        for(int j = 0; j < normalized_path_cost.size(); j++){
            score += std::pow(normalized_path_cost[i][j], 2);
        }
        scores.emplace_back(std::sqrt(score));
    }

    return scores;
}

/**
 * Voting methods
*/

// lower is better
std::vector<double> combined_approval(const std::vector<std::vector<double> > normalized_path_costs){
    std::vector<double> combined_sums;
    for(size_t i = 0; i < normalized_path_costs.size(); i++){
        int running_sum = 0;
        for(size_t j = 0; j < normalized_path_costs[0].size(); j++){
            if(normalized_path_costs[i][j] == 1.0)       {running_sum++;}
            else if(normalized_path_costs[i][j] == 0.0)  {running_sum--;} 
            else                                         {/* do nothing*/}
        }
       combined_sums.push_back(running_sum);
    }

    return combined_sums;
}

// lower score better
std::vector<double> borda(const std::vector<std::vector<double> > &normalized_path_costs){
    std::vector<double> results(normalized_path_costs.size());
    double rank;
    double prev;


    // will order them as <objective score, solution id>
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int> >, std::greater<std::pair<double, int> > > pq;

    // std::greater<double, int> will rank thing off the first value in the pair,
    for(int j = 0; j < normalized_path_costs[0].size(); j++){
        // for each objective, push all solutions scores into a priority queue to rank then by that 
        for(int i = 0; i < normalized_path_costs.size(); i++){
            pq.push(std::make_pair(normalized_path_costs[i][j], i));
        }

        // go through the pq to assing score
        rank = 0;
        prev = -1;

        while(!pq.empty()){
            auto top = pq.top();
            if(top.first == prev){  // if the next key-value in the pq have the same key, they get the same rank
                results[top.second] += rank;
            } else {
                rank++;
                results[top.second] += rank;
                prev = top.first;
            }
            pq.pop();
        }
    }

    return results;
}

// lower is better
std::vector<double> range(const std::vector<std::vector<double> > &normalized_path_costs){
    std::vector<double> solution_sums;

    for(int i = 0; i < normalized_path_costs.size(); i++){
        solution_sums.emplace_back(std::accumulate(normalized_path_costs[i].begin(), normalized_path_costs[i].end(),0));
    }

    return solution_sums;
}

// higher is better
std::vector<double> condorcet(const std::vector<std::vector<double> > &normalized_path_costs){
    std::vector<double> results(normalized_path_costs.size());

    for(int j = 0; j < normalized_path_costs[0].size(); j++){
        for(int i = 0; i < normalized_path_costs.size(); i++){
            for(int k = i; k < normalized_path_costs.size(); k++){
                if(k != i){
                    if(normalized_path_costs[i][j] < normalized_path_costs[k][j]){
                        results[i]++;
                    } else if(normalized_path_costs[i][j] > normalized_path_costs[k][j]){
                        results[k]++;
                    } else{/* its a draw, no nothing */}
                }
            }
        }
    }

    return results;
}


// d_score is a secrete return value
std::vector<int> vote(const std::string voting_method, const std::vector<std::vector<double> > &path_costs, const std::vector<double> &d_scores){
  std::vector<double> vote_result;

  std::vector<std::vector<double> > norm_path_costs = normalize_matrix(path_costs);
  std::vector<int> order;

  if(voting_method == "borda"){
      vote_result = borda(norm_path_costs);
      order = rank_accending(vote_result, d_scores);
  } else if (voting_method == "range") {
      vote_result = range(norm_path_costs);
      order = rank_decending(vote_result, d_scores);
  } else if(voting_method == "condorcet"){
      vote_result = condorcet(norm_path_costs);
      order = rank_decending(vote_result, d_scores);
  } else if(voting_method == "combined_approval") {
      vote_result = combined_approval(norm_path_costs);
      order = rank_decending(vote_result, d_scores);
  } else {
    // should never be reached!!
    exit(-1);
  }



  return order;
}


