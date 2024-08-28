#include <algorithm>
#include <codecvt>
#include <cstddef>
#include <limits>
#include <tuple>
#include <utility>
#include <random>
#include <vector>
#include <cmath>
#include <iostream>

#include "utili.hpp"

std::vector<std::vector<double> > normalize_matrix(std::vector<std::vector<double> > path_costs){
    double min, max;
    std::vector<std::vector<double> > normalized_matrix(path_costs.size(), std::vector<double>(path_costs[0].size(), 0));

    for(int j = 0; j < path_costs[0].size(); j++){
        min = INT_MAX;
        max = -1;
        //find min and max of the column (multiple max and mins are allowed)
        for(int i = 0; i < path_costs.size(); i++){
            if(path_costs[i][j] > max) {
                max = path_costs[i][j];
            } 
            if(path_costs[i][j] < min) {
                min = path_costs[i][j];
            }
        }

        for(int i = 0; i < path_costs.size(); i++){
            if(max == min){                                                       // remove NaN
                normalized_matrix[i][j] = 0.0;
            } else {
                normalized_matrix[i][j] = (path_costs[i][j] - min) / (max - min); // normalize using:  val - min / max - min
            }
        }
    }
    
    return normalized_matrix;
}

/**
 * @return an index of the smallest element 
 * @return -1 if there are multiple min elements
 * @return -2 if container is empty 
*/
int min_index(const std::vector<int> &list){
  int min = std::numeric_limits<int>::max(),
      minIndex = 0;
  bool multipleMin = false;

  for(int i = 0; i < list.size(); i++){
    if(list[i] < min){
      min = list[i];
      minIndex = i;
      multipleMin = false;
    } else if(list[i] == min){
      multipleMin = true;
    }
  }

  if(multipleMin){
    return -1;
  } else {
    return minIndex;
  }
}


int max_index(const std::vector<int> &list){
  int max = std::numeric_limits<int>::min(),
      maxIndex = 0;
  bool multipleMax = false;

  for(int i = 0; i < list.size(); i++){
    if(list[i] > max){
      max = list[i];
      maxIndex = i;
      multipleMax = false;
    } else if(list[i] == max){
      multipleMax = true;
    }
  }

  if(multipleMax){
    return -1;
  } else {
    return maxIndex;
  }

}



// paths will be represneted as a tripplet [voting score, d-score, id]
std::vector<int> rank_accending(const std::vector<double> &path_scores, const std::vector<double> &path_d_scores){
    std::vector<std::vector<double> > temp;
    rank_increase order;

    for(int i = 0; i < path_scores.size(); i++){
        temp.push_back({path_scores[i], path_d_scores[i], (double)i});
    }

    std::sort(temp.begin(), temp.end(), order);

    std::vector<int> results;
    for(auto iter: temp){
        results.push_back(iter.back());
    }
    
    return results;
}

// paths will be represneted as a tripplet [voting score, d-score, id]
std::vector<int> rank_decending(const std::vector<double> &path_scores, const std::vector<double> &path_d_scores){
    std::vector<std::vector<double> > temp;
    rank_decrease order;

    for(int i = 0; i < path_scores.size(); i++){
        temp.push_back({path_scores[i], path_d_scores[i], (double)i});
    }

    std::sort(temp.begin(), temp.end(), order);

    std::vector<int> results;
    for(auto iter: temp){
        results.push_back(iter.back());
    }
  
    return results;
}

    
/**
 * used at run time
*/ 

// arguments must be normalized between [0,1]
std::vector<double> complement_weight_set(const std::vector<double> &normalized_path_cost){
  std::vector<double> weight_set(normalized_path_cost.size());
  for(int i = 0; i < normalized_path_cost.size(); i++){
    weight_set[i] = (1 - normalized_path_cost[i]);
  }
  return weight_set;
}


// cuases issuesQ
void remove_duplicates(std::vector<std::vector<double> > &matrix){
  for(auto iter = matrix.begin(); iter != matrix.end(); iter++){
    for(auto jter = iter; jter != matrix.end(); jter++){
      if(*iter == *jter){
        jter = matrix.erase(jter);
      }
    }
  }
}


// return true if A dominates B, lower is better
bool dominates(const std::vector<double> &A, std::vector<double> &B){
  bool greater = false;
  for(int i = 0; i < A.size(); i++){
    if(A[i] < B[i]){    // for A to dominate B, it must, for the same objective, have a cost that is strictly lowerd
      greater = true;
    }
    if(A[i] > B[i]){    // A cannot dominate B if B, for the same objective, has a lower objective cost.
      return false;
    }
  }

  return greater;
}

bool weak_dominates(const std::vector<double> &A, const std::vector<double> &B){
  if(A.size() != B.size()){
    std::cout << "ERROR, domimnace check of vectors of different length." << std::endl;
    exit(-1);
  }
  for(int i = 0; i < A.size(); i++){
    if(A[i] > B[i]){
      return false;
    }
  }
  return true;
}

// Credits: Code for dominination filtering obtained from 
// https://github.com/jMetal/jMetalCpp/blob/master/src/qualityIndicator/Hypervolume.cpp

std::vector<std::vector<double> > non_dominated_filter(std::vector<std::vector<double> > front){
  int i = 0, 
      j = 0,
      n = front.size();

  while(i < n){
    j = i + 1;
    while(j < n) {
      if(dominates(front[i], front[j])){
        n--;
        
        front[j].swap(front[n]);
      } else if(dominates(front[j], front[i])){

        n--;        
        front[i].swap(front[n]);
        i--;
        break;
      } else {
        j++;
      }
    } // end while
    i++;
  }   // end while

  // return vector of options that are not dominated, the first n (when it finishes)
  std::vector<std::vector<double> > non_dominated(front.begin(), front.begin() + n);

  return non_dominated;
}



/** 
 * Evaluations 
*/

double sparsity(const std::vector<std::vector<double> > &front){
  int m = front[0].size(),
      n = front.size();

  double sparsity_score = 0.0;

  for(int j = 0; j < m; j++){
    std::vector<double> p_j; // gather and sort all the cost components of objective j
    for(int i = 0; i < n; i++){
      p_j.push_back(front[i][j]);
    }

    std::sort(p_j.begin(), p_j.end());

    for(auto k = 0; k < n-1; k++){
      sparsity_score += pow(p_j[k] - p_j[k+1], 2);
    }
  }

  sparsity_score = ((1.0) / (n-1)) * sparsity_score;

  if(isnan(sparsity_score)){
    return 0;
  } else {
    return sparsity_score;
  }
}

