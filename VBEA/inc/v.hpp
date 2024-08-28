#ifndef VOTING_
#define VOTING_

#include <vector>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

std::vector<double> calculate_d_score(const std::vector<std::vector<double> > &normalized_path_cost);

std::vector<double> combined_approval(const std::vector<std::vector<double> > normalized_path_costs);

std::vector<double> borda(const std::vector<std::vector<double> > &normalized_path_cost);

std::vector<double> range(const std::vector<std::vector<double> > &normalized_path_cost);

std::vector<double> condorcet(const std::vector<std::vector<double> > &normalized_path_cost);

std::vector<int> vote(const std::string voting_method, const std::vector<std::vector<double> > &path_costs, const std::vector<double> &d_scores);

#endif
