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
double sparcity_metric(const std::vector<std::vector<double>> &front_approximation);

// checks if a dominates b 
bool dominate_check(const std::vector<double> &point_1, const std::vector<double> &point_2);

double objMin(std::vector<std::vector<double>> front, int obj);

std::vector<std::vector<double>> reduce_non_dominated(std::vector<std::vector<double>> front, const int obj, const double threshold);

// a pre-requsite for calculating hypervolume metric
std::vector<std::vector<double>> non_dominated_filter(std::vector<std::vector<double>> front);

// hypervolume calculator
// uses default refernce point of {0, 0, ..., 0}
double hypervolume_metric(const std::vector<double> &path_cost);

#endif
