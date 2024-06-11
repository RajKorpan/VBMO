#ifndef UTILI_
#define UTILI_
#include <algorithm>
#include <utility>
#include <vector>
#include <cmath>
#include <iostream>

std::ostream& operator<<(std::ostream &stream, const std::vector<double> &vec);

std::ostream& operator<<(std::ostream &stream, const std::vector<std::vector<double>> &vec);

std::ostream& operator<<(std::ostream &stream, const std::vector<size_t> &vec);

std::ostream& operator<<(std::ostream &stream, const std::vector<int> &vec);

std::ostream& operator<<(std::ostream &stream, const std::vector<std::string> &vec);

void display(const std::vector<double> &vec);

std::vector<double> complement_weight_set(const std::vector<double>& normalized_path_cost);

// std::vector<double> complement_weight_set(const std::vector<double>& normalized_path_cost);
double sparsity_metric(const std::vector<std::vector<double>> &front_approximation);

// checks if a dominates b 
bool dominate_check(const std::vector<double> &point_1, const std::vector<double> &point_2);

double objMin(std::vector<std::vector<double>> front, int obj);

std::vector<std::vector<double>> reduce_non_dominated(std::vector<std::vector<double>> front, const int obj, const double threshold);


std::vector<std::vector<double>> remove_duplicate(const std::vector<std::vector<double>> front);

// a pre-requsite for calculating hypervolume metric
std::vector<std::vector<double>> non_dominated_filter(std::vector<std::vector<double>> front);



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

// hypervolume calculator
// uses default refernce point of {0, 0, ..., 0}

#endif
