#ifndef UTILI_
#define UTILI_

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



std::vector<std::vector<double> > normalize_matrix(std::vector<std::vector<double> > path_costs);

int min_index(const std::vector<int> &list);

int max_index(const std::vector<int> &list);

std::vector<int> rank_accending(const std::vector<double> &path_scores, const std::vector<double> &path_d_scores);

std::vector<int> rank_decending(const std::vector<double> &path_scores, const std::vector<double> &path_d_scores);

std::vector<double> complement_weight_set(const std::vector<double> &normalized_path_cost);

void remove_duplicates(std::vector<std::vector<double> > &matrix);

bool dominates(const std::vector<double> &A, std::vector<double> &B);

bool weak_dominates(const std::vector<double> &A, const std::vector<double> &B);

std::vector<std::vector<double> > non_dominated_filter(std::vector<std::vector<double> > front);

double sparsity(const std::vector<std::vector<double> > &front);

struct rank_decrease{
    const bool operator()(const std::vector<double>&a, std::vector<double> &b) const {
        if(a[0] == b[0]){
            return a[1] < b[1];
        } else {
            return a[0] < b[0];
        }
    }
};

struct rank_increase{
    const bool operator()(const std::vector<double> &a, std::vector<double> &b) const {
        if(a[0] == b[0]){
            return a[1] < b[1];
        } else {
            return a[0] < b[0];
        }
    }
};

#endif
