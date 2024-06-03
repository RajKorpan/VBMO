#ifndef VOTING_METHOD_
#define VOTING_METHOD_
#include <vector>
#include <iostream>
#include <cmath>
#include <queue>

/**
 * ===========================================
 * HELPERS FUNCTIONS
 * ===========================================
*/

double add_scores(std::vector<double> normalized_path_cost);

int findMin(std::vector<double> list);

int findMax(std::vector<double> list);

/**
 * @brief Normalize paths with respect to the objectives across the path (i.e. normalize objectives across  paths) 
 * @param path_costs is a n x n matrix.
 * @return a n x n matrix that is normalized.
 */
std::vector<std::vector<double>> normalize_matrix(std::vector<std::vector<double>> path_costs);

/** 
 * ===========================================
 * START OF VOTING METHODS
 * @paragraph for the sake of readablility, let n be the number of objectives 0, 1, ..., n-1 thar are found in multi objecive graph. 
 * @param normalized_path_costs a n x n matrix (where n is the number of obejectives) contaning the objective cost (as doubles) of the n single objective optimized paths created by VMBO_A_STAR
 * @pre there are no negative objective costs
 * @return a vector containing the results of the voting scheme such that vector[i] is the score of the path i as determined by the said voting scheme. (a higher or low score is not necssarily good or bad, it depends on the voting schemme used, e.g. in borda higher is better, but in combined_approaval, lower is better).
 * ===========================================
*/  

std::vector<double> range_voting(std::vector<std::vector<double>> normalized_path_costs);

std::vector<double> condorcet_voting(std::vector<std::vector<double>> normalized_path_costs);

std::vector<double> borda_voting(std::vector<std::vector<double>> normalized_path_costs);

std::vector<double> combined_aproval_voting(std::vector<std::vector<double>> normalized_path_costs);


/** 
 * @details find the euclidean distance in n-dimension space for a normalized path for tie resolution
*/
double path_d_score(const std::vector<double>& normalized_path_cost);

/**
 * @details When a tie is detected in a voting scheme, we pick the candidate that is closest to the Pareto space origin ({0, 0, 0, ..., 0}).
 * @return a n sized vector where vector[i] is the euclidean distance in Pareto space between path i and the origin.
*/
std::vector<double> d_score(std::vector<std::vector<double>> normalized_path_costs);

/**
 * @param voting_method determins which voting methods, options are:
 *                                      "range",
 *                                      "combined_approval",
 *                                      "borda", and, 
 *                                      "condorent".
 * @param normalized_path_score is a n x n matrix containing the normalized path scores produced by the n single objective optimized A* shortest paths.
 * @details If a tie was archieved in the voting stage, compares the paths "d_scores" (see d_score for details)
 * @return the index of the path that wins
*/

enum voting_method {range, combined_approval, borda, condornet};

std::vector<int> vote(std::vector<std::vector<double>>& normalized_path_score, voting_method scheme);
#endif
