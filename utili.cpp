#include "utili.hpp"
#include <algorithm>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

/**
 * IO
*/

void load_DOA_instnaces(const std::string FILE, std::vector<std::pair<double, double>> &out_inst){
  std::ifstream ifs(FILE);
  ifs.ignore(std::numeric_limits<int>::max(), '\n');


  std::string temp;
  double s, t;
  while(ifs >> temp){
    if(temp == "v"){
      ifs >> s >> t;
      out_inst.push_back({s, t});
    }
  }
}

/**
 * << overloads
 */

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

void display(const std::vector<double> &vec){
  std::cout << "{";
  std::cout << vec[0];
  for(int i = 1; i < vec.size(); i++){
    std::cout << ", " << vec[i];
  }
  std::cout << "}";
}

// Weight Selection Method

std::vector<double> complement_weight_set(const std::vector<double>& normalized_path_cost){
    // using complement for now, new version of weight sets are planned 
    std::vector<double> weight_set(normalized_path_cost.size());
    for(int i = 0; i < normalized_path_cost.size(); i++){
        weight_set[i] = (1 - normalized_path_cost[i]);
    }

    return weight_set;
}

double sparcity_metric(const std::vector<std::vector<double>> &front_approximation){
  int m = front_approximation[0].size();      // m = number of objectives in the environment
  int n = front_approximation.size();         // n = number of solutions
  double sparcity = 0;

  for(int j = 0; j < m; j++){ // get the cost of a objective j for all paths
    std::vector<double> p_j;
    for(int i = 0; i < n; i++){
      p_j.push_back(front_approximation[i][j]);
    }

    std::sort(p_j.begin(), p_j.end()); //sparcity metric requires that we sort the objectve costs

    for(auto i = 0; i < n-1; i++){     // squared difference of the sorted objective cost
      sparcity += pow(p_j[i] - p_j[i+1], 2);
    }
  }


  sparcity = (1.0 / (n - 1)) * sparcity;  
  return sparcity;
  
}

// Credits: Code for dominination filtering obtained from 
// https://github.com/jMetal/jMetalCpp/blob/master/src/qualityIndicator/Hypervolume.cpp


// checks if point_1 dominates point_2
bool dominate_check(const std::vector<double> &point_1, const std::vector<double> &point_2){
  bool greaterCheck = false;
  for(int j = 0; j < point_1.size(); j++){
    if(point_1[j] <= point_2[j]){
      if(point_1[j] < point_2[j]){ // in order for point 1 to dominate point 2, it must be have an objective cost that is strictly greater.
        greaterCheck = true;
      }
      continue;
    }
    if(point_1[j] > point_2[j]){       // point 1 cannot dominate a path that it not have atleast an equal objective cost of.
      return false;
      break;
    }

  }   

  return greaterCheck;
}

bool weak_dominate_check(const std::vector<double> &point_1, const std::vector<double> &point_2){
  for(int i = 0; i < point_1.size(); i++){
    if(point_1[i] > point_2[i]){
      return false;
    }
  }

  return true;
}


void remove_dulplicates(std::vector<std::vector<double>> front){
    for(auto iter = front.begin(); iter != front.end(); iter++){
      for(auto jter = iter; jter != front.end(); jter++){
        if(*iter == *jter){
          jter = front.erase(jter);
        }
      }      
    }
}


double objMin(std::vector<std::vector<double>> front, int obj){
  int b = front[0].size(),
      n = front.size();
      
  // if(n < 1){      //debug
  //   std::cout << "ERROR"
  //   exit(-1)
  // }

  double minVal = front[0][obj];
  for(int i = 1; i < n; i++){
    if(front[i][obj] < minVal){
      minVal = front[i][obj];
    }
  }

  return minVal;
}
// a pre-requsite for calculating hypervolume metric

std::vector<std::vector<double>> reduce_non_dominated(std::vector<std::vector<double>> front, const int obj, const double threshold){
  int n = front.size();
  for(int i = 0; i < n; i++){
    if(front[i][obj] <= threshold){
      n--;
      std::swap(front[i], front[n]);
    }
  }

  return std::vector<std::vector<double>>(front.begin(), front.begin() + n);
}

std::vector<std::vector<double>> non_dominated_filter(std::vector<std::vector<double>> front){
  int i = 0, 
      j = 0,
      n = front.size();

  while(i < n){
    j = i + 1;
    while(j < n) {
      if(dominate_check(front[i], front[j])){
        n--;
        display(front[i]);
        std::cout << " <  ";
        display(front[j]);
        std::cout << std::endl;
        
        front[j].swap(front[n]);
      } else if(dominate_check(front[j], front[i])){
        display(front[j]);
        std::cout << " < ";
        display(front[i]);
        std::cout << std::endl;

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
  std::vector<std::vector<double>> non_dominated(front.begin(), front.begin() + n);

  return non_dominated;
}

// hypervolume calculator
// uses default refernce point of {0, 0, ..., 0}
// pre-suppose that all solutions have been filtered for non dominance.
double hypervolume_metric(const std::vector<double> &path_cost){
  double HV_metric;
  std::vector<double> temp;

  for(int i = 0; i < path_cost.size(); i++){
    temp.push_back(0 - path_cost[i]); // reference point is 1 since we normalized the data
  }

  double sum = 0;
  
  temp.push_back(0.0);
  std::sort(temp.begin(), temp.end());


  for(int i = 0; i < temp.size(); i++){
    sum += temp[i] * pow(-1.0, i+1);
  }

  std::sort(temp.begin(), temp.end());

  HV_metric = sum / std::tgamma(temp.size() + 1);
  
  return HV_metric; 
}


struct HypterVolumeCalculator{

  bool WeakDominate(const std::vector<double> &p1, const std::vector<double> &p2){
    for(int i = 0; i < p1.size(); i++){
      if(p1[i] > p2[i]){
        return false;
      }
    }
    return true;
  }

  void hvRecursive(const int dimIndex, const int length){
     
  }

  double Calculate(const std::vector<std::vector<double>> &front){
    auto FRONT = front;

    int n = front.size();       // the number of solutions
    int b = front[0].size();    // the dimensinos

    std::vector<std::vector<double>> relevantPoints;

    for(int i = 0; i < FRONT.size(); i++){
      for(int j = 0; j < FRONT[i].size(); j++){
        FRONT[i][j] = -FRONT[i][j];
      }

    }
    
    
    double hvol = 0.0;

    return 1.0;

  }

  
    
};