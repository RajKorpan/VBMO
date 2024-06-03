#include "voting.hpp"
#include <algorithm>
#include <vector>           // vectors
#include <iostream>         // input and output functionality
#include <cmath>            // sqrt(), abs(), and other basic math functions
#include <queue>            // using priority_queue as the openSet for A*



/**
 * ===========================================
 * HELPERS FUNCTIONS
 * ===========================================
*/

/**
 * @brief Normalize paths with respect to the objectives across the path (i.e. normalize objectives across  paths) 
 * @param path_costs is a n x n matrix.
 * @return a n x n matrix that is normalized.
 */

 // FIX: what if max == min, then lots of NaN
std::vector<std::vector<double>> normalize_matrix(std::vector<std::vector<double>> path_costs){
    double min, max;
    std::vector<std::vector<double>> normalized_matrix(path_costs.size(), std::vector<double>(path_costs[0].size(), 0));

    for(int j = 0; j < path_costs[0].size(); j++){
        //find min and max value of an objective cross all paths
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
 * @details a method for summing a vector of doubles
*/
double add_scores(std::vector<double> normalized_path_cost) {
    double total = 0;

    for(auto &iter: normalized_path_cost){
        total += iter;
    }

    return total;
}

/**
 * @details a method for finding the index of the smallest element in a vector, or signal that there are multiple smallest elements.
 * @return an index [0,n-1] of the smallest value should it be unique. 
 * @return -1 if there are multiple elements that are the min.
 * @return -2 if container is empty.
*/
int findMin(std::vector<double> list) {
    if(list.empty()) {
        return -2;
    }

    double min = std::numeric_limits<double>::max(); //why is this needed?
    double minIndex;
    bool doubleMin = false;

    for(int i = 0; i < list.size(); i++) {
        if(list[i] < min) {
            min = list[i];
            minIndex = i;
            doubleMin = false;
        } else if (min == list[i]) {
            doubleMin = true;
        }
    }

    if(doubleMin) {
        return -1;
    } else {
        return minIndex;
    }
}




/**
 * @details a method for finding the index of the largest element in a vector, or signal that there are multiple largest elements.
 * @return an index [0,n-1] of the largest element if it is unique. 
 * @return -1 if there are multiple elements that are the max.
 * @return -2 if container is empty.
*/
int findMax(std::vector<double> list) {
    if(list.empty()) {
        return -2;
    }
    double max = -1; //objective cost cannot be negative.
    double maxIndex = std::numeric_limits<double>::min();
    bool doubleMax = false;

    for(int i = 0; i < list.size(); i++) {
        if(list[i] > max) {
            max = list[i];
            maxIndex = i; 
        } else if (max == list[i]) {
            doubleMax = true;
        }
    }

    if(doubleMax) {
        return -1;
    } else {
        return maxIndex;
    }
}


// paths will be represneted as a tripplet [voting score, d-score, id]
struct rank_decrease {
    bool operator()(const std::vector<double>&a, std::vector<double> &b) const {
        if(a[0] == b[0]){
            return a[1] < b[1];
        } else {
            return a[0] < b[0];
        }
    }
};

struct rank_increase {
    bool operator()(const std::vector<double> &a, std::vector<double> &b) const {
        if(a[0] == b[0]){
            return a[1] < b[1];
        } else {
            return a[0] < b[0];
        }
    }
};

// paths will be represneted as a tripplet [voting score, d-score, id]
std::vector<int> rank_increasing(const std::vector<double> &path_scores, const std::vector<double> &path_d_scores){
    std::vector<std::vector<double>> temp;
    rank_increase order;

    for(int i = 0; i < path_scores.size(); i++){
        temp.push_back({path_scores[i], path_d_scores[i], (double)i});
    }

    std::sort(temp.begin(), temp.end(), order);

    std::vector<int> results;
    for(auto iter: temp){
        results.push_back(iter.back());
    }


    // std::cout << "score | d     | id  \n";
    // for(auto &iter: temp){
    //     std::cout << iter[0] << "| " << iter[1] << "| " << (int)iter[2] << std::endl;
    // }
    
    return results;
}

// paths will be represneted as a tripplet [voting score, d-score, id]
std::vector<int> rank_decreasing(const std::vector<double> &path_scores, const std::vector<double> &path_d_scores){
    std::vector<std::vector<double>> temp;
    rank_decrease order;

    for(int i = 0; i < path_scores.size(); i++){
        temp.push_back({path_scores[i], path_d_scores[i], (double)i});
    }

    std::sort(temp.begin(), temp.end(), order);

    std::vector<int> results;
    for(auto iter: temp){
        results.push_back(iter.back());
    }


    // std::cout << "score | d     | id  \n";
    // for(auto &iter: temp){
    //     std::cout << iter[0] << "| " << iter[1] << "| " << (int)iter[2] << std::endl;
    // }
    
    return results;
}

    



/** 
 * ===========================================
 * START OF VOTING METHODS
 * @paragraph for the sake of readablility, let n be the number of objectives 0, 1, ..., n-1 thar are found in multi objecive graph. 
 * @param normalized_path_costs a n x n matrix (where n is the number of obejectives) contaning the objective cost (as doubles) of the n single objective optimized paths created by VMBO_A_STAR
 * @pre there are no negative objective costs
 * @return a vector containing the results of the voting scheme such that vector[i] is the score of the path i as determined by the said voting scheme. (a higher or low score is not necssarily good or bad, it depends on the voting schemme used, e.g. in borda higher is better, but in combined_approaval, lower is better).
 * ===========================================
*/  

/**
 * @return a 1 x n vector where vector[i] contains the sum of all of the (normalized) objective cost of path i.
*/
std::vector<double> range_voting(std::vector<std::vector<double>> normalized_path_costs) {
    std::vector<double> sum_of_paths = {};
    for(int i = 0; i < normalized_path_costs.size(); i++) {
        sum_of_paths.emplace_back(add_scores(normalized_path_costs[i]));
    }
    return sum_of_paths;
}

/**
 * @details 
 * @return a n sized vector where vector[i] is the number of times path i won the comparison to another paths objective  
*/
std::vector<double> condorcet_voting(std::vector<std::vector<double>> normalized_path_costs) {
    //vector that stores the results, initially al are set by zero with this construction.
    std::vector<double> condorcet_scores(normalized_path_costs.size());;

    for(int i = 0; i < normalized_path_costs[0].size(); i++) {
    //            std::cout << "Objective " << i << std:: endl;         //debug
        for(int j = 0; j < normalized_path_costs.size(); j++) {

            for(int k = j; k < normalized_path_costs.size(); k++) {
                if(k != j) { // 
                    std::cout << normalized_path_costs[j][i] << " " << normalized_path_costs[k][i] << std::endl;      //debug
                    if(normalized_path_costs[j][i] < normalized_path_costs[k][i]) {
                        condorcet_scores[j]++;
                    } else if (normalized_path_costs[j][i] > normalized_path_costs[k][i]) {
                        condorcet_scores[k]++;
                    } else {/*is a draw, do nothing*/}
                    std::cout << "-----" << std::endl;        //debug
                }
            }
        }
    }

    return condorcet_scores;
}

/**
 * @details Borda voting involves assigning rank to each candidate based on their score such that the candidate with the smallest score is assigned rank 0, the runner up is given rank 1, etc.. This implementation gives candidates with equal score equal rank. We run n votes, one for each objective.
 * @return a 1 x n vector where vector[i] is the sum of the rans of path i.
*/
std::vector<double> borda_voting(std::vector<std::vector<double>> normalized_path_costs) {
    std::vector<double> borda_results(normalized_path_costs.size());

    double rank;
    double prev;
    //                           <key,value>                                         comparitor for pairs using the first value as the key for comparison
    std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, std::greater<std::pair<double,int>> > pq;

    for(int i = 0; i < normalized_path_costs[0].size(); i++) {
        //for each objective
        //Push all the paths into a priority queue where their key is a their objective cost 
        for(int j = 0; j < normalized_path_costs.size(); j++) {
            pq.push(std::make_pair(normalized_path_costs[j][i],j));
        }
        rank = 0;
        prev = -1; // to be overwritten as we cannot have an objective cost cost less then 0
        while(!pq.empty()){
            auto top = pq.top();
            if(top.first > prev) {
                rank++;
                borda_results[top.second] += rank;
                prev = top.first;                   //set the prev to be the previosuly
            } else {
                borda_results[top.second] += rank;
            }
        //              <Objective cost>      <path number>      <rank assigned>
            // std::cout << top.first << " " << top.second << " " << rank << std::endl;     //debug
            pq.pop();
        }
        // std::cout << "----" << std::endl;     //debug
    }

    return borda_results;
}

/**
 * @details combined approval invloves comparing every candidate with each other, giving the "greater" (the one with the lower objective cost) a point, the losser receiving nothing. Repeart for each objective.
 * @return a 1 x n vector where vector[i] is the total number of points path i won via the n trials.
*/
std::vector<double> combined_aproval_voting(std::vector<std::vector<double>> normalized_path_costs) {
    // result vector will have the total sum 
    std::vector<double> combined_approval_sum = {};
    for(int i = 0; i < normalized_path_costs.size(); i++) {
        double path_combined_score = 0;
        for(int j = 0; j < normalized_path_costs[i].size(); j++) {
            if(normalized_path_costs[i][j] == 0)        {path_combined_score++;}
            else if(normalized_path_costs[i][j] == 1)   {path_combined_score--;}
            else                                        {/*do nothing*/}
        }
        // add the combined approval sum to the result vector 
        combined_approval_sum.emplace_back(path_combined_score);
    }
    
    return combined_approval_sum;
}

/** 
 * @details find the euclidean distance in n-dimension space for a normalized path 
 * 
*/
double path_d_score(const std::vector<double>& normalized_path_cost){
    double d_score;
    for(int i = 0; i < normalized_path_cost.size(); i++){
        d_score += std::pow(normalized_path_cost[i], 2);
    }
    d_score = std::sqrt(d_score);

    return d_score;
}

/**
 * @details When a tie is detected in a voting scheme, we pick the candidate that is closest to the Pareto space origin ({0, 0, 0, ..., 0}).
 * @return a n sized vector where vector[i] is the euclidean distance in Pareto space between path i and the origin.
*/
std::vector<double> d_score(std::vector<std::vector<double>> normalized_path_costs) {
    std::vector<double> d_score;
    for(int i = 0; i < normalized_path_costs.size(); i++) {
        d_score.emplace_back(path_d_score(normalized_path_costs[i]));
    }

    return d_score;
}

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

// updated to account for d-score tie breaking 
std::vector<int> vote(std::vector<std::vector<double>>& path_scores, voting_method scheme){
    std::vector<double> voting_results;    // ith paths score w.r.s.t the voting method
    std::vector<int> rank;                 // index corresponds to values ranking, v[i] = j means path j is in ith place

    std::vector<std::vector<double>> norm_path_scores = normalize_matrix(path_scores);
    std::vector<double> d_scores = d_score(path_scores); // using d-score of the raw cost as it yiled more precision
    std::vector<int> order;

    switch(scheme){
        case range:
            voting_results = range_voting(norm_path_scores);
            order = rank_increasing(voting_results, d_scores);
            break;
            
        case borda:
            voting_results = borda_voting(norm_path_scores);
            order = rank_increasing(voting_results, d_scores);
            break;

        case combined_approval:
            voting_results = combined_aproval_voting(norm_path_scores);
            order = rank_decreasing(voting_results, d_scores);
            break;

        case condornet:
           voting_results = condorcet_voting(norm_path_scores);
            order = rank_decreasing(voting_results, d_scores);
            break;
    }

    return order;
}

