#include <cstdio>
#include <vector>           // vectors
#include <string>           // getline() and reading files
#include <iostream>         // input and output functionality
#include <fstream>          // saving maps, paths, and other data to external files 
#include <unordered_map>    // for lookup tabled 
#include <cmath>            // sqrt(), abs(), sin() and other basic math functions
#include <queue>            // using priority_queue as the openSet for A*
#include <algorithm>        // ???
#include <list>             // shortest path are put into lists for their quick emplace_front() feature
#include <chrono>           // for timing and random number generator seed
#include <random>           // using the mersenne_twister_engine for random numbers
#include <iterator>         // ostream iterator 
#include <filesystem>       // for iterating through files
#include <functional>       // for hash
#include <numbers>          // for pi and other mathmatical constants
#include <math.h>
/**
 * ===========================================
 * HELPER FUNCTIOIONS
 * ===========================================
*/



/**
 * @brief Normalize paths with respect to the objectives across the path (i.e. normalize objectives across  paths) 
 * @param path_costs is a n x n matrix.
 * @return a n x n matrix that is normalized.
 */
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
            normalized_matrix[i][j] = (path_costs[i][j] - min) / (max - min);
        }
    }
    
    return normalized_matrix;
}


double sparcity_metric(const std::vector<std::vector<double>>&  normalized_path_costs){ 
        int m = normalized_path_costs[0].size(); // the number of objectives
        int n = normalized_path_costs.size(); //the size of the pareto oproximation (number of paths we have)
        double sum = 0;
        for(int j = 0; j < m; j++){
            for(int i = 0; i < n - 1; i++){
                  sum += pow(normalized_path_costs[i][j] - normalized_path_costs[i + 1][j], 2);
            }
        }

        sum *= 1.0/(n-1);

        return sum;
}
/**
 * @details a method for displaying a one-dimensional vector
*/
void display_vector(std::vector<double> normalized_path_cost) {
    if(normalized_path_cost.size() > 0) {
        std::cout << "{" << normalized_path_cost[0];
        for(int i = 1; i < normalized_path_cost.size(); i++) {
            std::cout << ", " << std::fixed << normalized_path_cost[i];
        }
        std::cout << "}" << std::endl;
    } 
    else { // the vecetor is empty
        std::cout << "{}" << std::endl;
    }
}

void display_vector(std::vector<int> normalized_path_cost) {
    if(normalized_path_cost.size() > 0) {
        std::cout << "{" << normalized_path_cost[0];
        for(int i = 1; i < normalized_path_cost.size(); i++) {
            std::cout << ", " << std::fixed << normalized_path_cost[i];
        }
        std::cout << "}" << std::endl;
    } 
    else { // the vecetor is empty
        std::cout << "{}" << std::endl;
    }
}


/**
 * @details is a n x n matrix of the (what should be normalized) path objective costs
*/
void display_matrix(std::vector<std::vector<double>> normalized_path_costs) {
    if(normalized_path_costs.size() == 0){
        std::cout << "{}" << std::endl;
        return;
    }
    for(int i = 0; i < normalized_path_costs.size(); i++) {
        std::cout << "P_" << i << ": "; display_vector(normalized_path_costs[i]);
    }
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
int VOTE(std::vector<std::vector<double>>& normalized_path_score, const std::string voting_method) {
    std::vector<double> results;
    int winner; 

    if(voting_method == "range") {
        results = range_voting(normalized_path_score);
        winner = findMin(results);
        //lowest wins
    } else if(voting_method == "combined_approval")  {
        results = combined_aproval_voting(normalized_path_score);
        winner = findMax(results);
        //highest win
    } else if(voting_method == "borda") {
        results = borda_voting(normalized_path_score);
        winner = findMin(results);
        //lowest win
    } else if(voting_method == "condorcet") {
        results = condorcet_voting(normalized_path_score);
        winner = findMax(results);        //highest win
    } else {
        std::cout << "Invalid voting method" << std::endl;
        return {};
    }

    // display_vector(results);
    if(winner == -1) {
        std::cout << "Tie detected" << std::endl;
        std::vector<double> d_scores = d_score(normalized_path_score);;

        // display_vector(d_scores);
        winner = findMin(d_scores);
        if(winner == -1){
            std::cout << "TIE in d_score!" << std::endl;
            return 0;
        }
    }
    std::cout << "====================" << std::endl;
    std::cout << "Winner is P_" << winner << ": "; display_vector(normalized_path_score[winner]);
    std::cout << "====================" << std::endl;

    return winner;
}

std::vector<int> rank_decreasing_order(std::vector<double> path_scores){
    std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, std::less<std::pair<double,int>>> temp;
    for(int i = 0; i < path_scores.size(); i++){
        temp.push({path_scores[i], i});
    }
    std::vector<int> order;
    while(!temp.empty()){
        order.push_back(temp.top().second);
        temp.pop();
    }

    return order;
}

std::vector<int> rank_increasing_order(std::vector<double> path_scores){
    std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, std::greater<std::pair<double,int>>> temp;
    for(int i = 0; i < path_scores.size(); i++){
        temp.push({path_scores[i], i});
    }
    std::vector<int> order;
    while(!temp.empty()){
        order.push_back(temp.top().second);
        temp.pop();
    }

    return order;
}

std::vector<int> vote(std::vector<std::vector<double>>& normalized_path_score, std::string voting_method){
    std::vector<double> results;
    std::vector<int> order;

    if(voting_method == "range") {
        results = range_voting(normalized_path_score);
        order = rank_increasing_order(results);
        //lowest wins
    } else if(voting_method == "combined_approval")  {
        results = combined_aproval_voting(normalized_path_score);
        order = rank_decreasing_order(results);
        //highest win
    } else if(voting_method == "borda") {
        results = borda_voting(normalized_path_score);
        order = rank_increasing_order(results);
        //lowest win
    } else if(voting_method == "condorcet") {
        results = condorcet_voting(normalized_path_score);
        order = rank_decreasing_order(results);
        // highest win
    } else {
        std::cout << "Invalid voting method" << std::endl;
        order = {};
    }
    // all voting schemes return a verctor where v[i] is the score of the path i is the cost of path i. 

    //return a vector of ints where the value at v[i] is the path in the i+1th place 
    // e.g. the value at index 0 is the winner, index 1 has the 2nd place, etc.
    return order;
}


/**
 * IDEAS
 * 1. SEE WHAT A*PEX DID TO ADD OBJECTIVE
 * 2. IN NOT DONE BEFORE. ADD A RISK FACTOR, WHICH IS A MEASURE OF THE DENSITY OF THE POPULATION?
 * 3. CHECK WHAT KORPAN DID
*/

class node{
    private:
    int id;
    double latitude, longitude , weight;

    public:

        node(){
            latitude = 0;
            longitude = 0;
            id = -1;
        }
    
        node(double a, double b, int num){
            latitude = a;
            longitude = b;
            id = num;
        }

        node(const node& rhs){
            latitude = rhs.latitude;
            longitude = rhs.longitude;
            id = rhs.id;
            weight = rhs.id;
        }

        node(node&& rhs){
            latitude = rhs.latitude;
            longitude = rhs.longitude;
            id = rhs.id;
            weight = rhs.id;
        }

        node& operator=(const node& rhs) {
            latitude = rhs.latitude;
            longitude = rhs.longitude;
            id = rhs.id;
            weight = rhs.id;

            return *this;
        }
        
        node& operator=(node&& rhs) {
            latitude = rhs.latitude;
            longitude = rhs.longitude;
            id = rhs.id;
            weight = rhs.id;

            return *this; 
        }

        bool operator==(const node& rhs) const {
            return id == rhs.id;
        }

        void setWeight(double w){
            weight = w;
        }

        double getWeight() const {
            return weight;
        }
        
        double lat() const {
            return latitude;
        }

        double lng() const {
            return longitude;
        }

        void display() const {
            std::cout << std::fixed << "(" << latitude << ", " << longitude << ", " << id << ")";
        }
};

struct NodeHash{
    size_t operator()(const node& a) const {
        std::hash<double> double_hash;
        return double_hash(a.lat() * a.lng());
    }
};

struct PQNodeHash{
    size_t operator()(const node& a, const node& b) const {
        return a.getWeight() > b.getWeight();
    }
};

/**
 * TYPEDEF for adjacency matrices and priority queues for A* 
*/
typedef std::unordered_map<node, std::unordered_map<node, double, NodeHash>, NodeHash>              SO_adjacency_matrix;
typedef std::priority_queue<node, std::vector<node>, PQNodeHash>                                    node_priority_queue; 
typedef std::unordered_map<node,std::unordered_map<node, std::vector<double>, NodeHash>, NodeHash>  MO_adjacency_matrix;   

struct HEURISTIC {
    double operator()(const node& origin, const node& target){
        //haversine distance formula is used instead of euclidean distance        
        double dLat = (target.lat() - origin.lat()) * M_PI / 180.0;
        double dLon = (target.lng() - origin.lng()) * M_PI / 180.0;

        double lata = origin.lat() * M_PI  / 180;
        double latb = target.lat() * M_PI  / 180;

        double a = pow(sin(dLat / 2), 2) + pow (sin(dLon / 2), 2) + cos(origin.lat()) * cos(target.lat());

        double c = 2 * asin(sqrt(a));

        return 6371 * c;
    }
};

//The only difference is that one must use the haversine distance formuka.
/**
 * A* METHOD THAT GENERATES THE SHORTEST PATH WITH RESPECT TO ONLY ONE OBJECTIVE OF A MULTI OBJECTIVE ADJACENCY MATRIX 
 * @param origin, target are the start and end point for the search algorithm.
 * @param graph is a n objective adjacency matrix.
 * @param h is the hueristic functor that can be set in the driver code to use one of the supported heuristic functions (see HEURISTIC for more details).
 * @param objective is the objective {0, 1, ..., n-1} that A* will find the shortest path with respect to.
 * @return a vector of all of the objective cost 
 * @return {} if no path from the origin and target exist.
*/
std::vector<double> SINGLE_OBJECTIVE_A_STAR(node origin, node target, const MO_adjacency_matrix& graph, HEURISTIC& h, const int objective){
    auto startTime = std::chrono::high_resolution_clock::now();

    //BOOK KEEPING 
    node_priority_queue                         openSet;
    origin.setWeight(h(origin, target));
    openSet.push(origin);

    std::unordered_map<node, node, NodeHash>    descendentList;     // maintain a list of immedaiate descendent that follow the shortest path.
    std::unordered_map<node, double, NodeHash>  scoreList;          // maintaned the shortest distance found to a node from the origin of the search.
    scoreList[origin] = 0;
    std::unordered_map<node, bool, NodeHash>    visitList;          // keeps track of nodes that we have "explpred" i.e. added its neighbors to the openSet.

    //SEARCHING
    while(!openSet.empty()){
        //since this implementation uses lazy deletion, we must discard nodoes that we have already explored.
        while(visitList.find(openSet.top()) != visitList.end()){
            openSet.pop();
        }

        node top = openSet.top();
        if(top == target){      // target is reached, begin tracing steps back towards the origin.
            int n = graph.begin()->second.begin()->second.size();   // find the number of objectives in the graph.
            std::vector<double> path_costs(n);
            node prev;

            while(!(top == origin)){   //begin back tracking the descendentList until we reach the origin.
                prev = descendentList[top];
                for(int i = 0; i < n; i++){
                    path_costs[i] += graph.at(prev).at(top)[i];
                }
                top = prev;
            } 
            
            return path_costs;
        } else {
            openSet.pop();
            //iterate throgh all of tops neighbors and calculate their tenative score: the distance to get to top and then to its neighbor
            for(auto& iter: graph.at(top)){
                node neighbor = iter.first;
                double tenativeScore = scoreList[top] + iter.second[objective];
                // if this neighbor has never been seen, or if this path is cheaper then a previously found path
                if(scoreList.find(neighbor) == scoreList.end() || tenativeScore < scoreList[neighbor]){
                    descendentList[neighbor] = top;         // update/set the decedant of neighbor
                    scoreList[neighbor] = tenativeScore;    // update/set the estimate score of neigbor
                    neighbor.setWeight(tenativeScore + h(neighbor, target));    //update/set the nodes weight in the openSet
                    openSet.push(neighbor);                                     //note we are using lazy deletion so the updated/better score will appear first in the open set)

                }
            }
        }
    }

    //return empty set if no path exist
    return {};
}

/**
 * ===========================================
 * VBMO OPPONENT, A NAIVE APPROACH VIA SUMMING THE OBJECIVES INTO A SINGLE OBJECTIVE
 * @param origin, target are the start and end point for the search algorithm.
 * @param graph is a n objective adjacency matrix.
 * @param h is the heuristic functor that can be set in the driver code to select a supported heuristic function (see HUERISTIC for more details).
 * @details the first step is to iteratore through the entire adjacency matrix and sum all the objective cost into one objective. Then run A* with the sole objective
 * @return a vector with the shorest path with respect to the combined objective.
 * @return {} if no path from the orgigin and target exist.
 * ===========================================
*/
std::vector<double> COMBINED_MO_A_STAR(node origin, node target, const MO_adjacency_matrix graph, HEURISTIC& h, int& duration){
    //sum the objective costs into one objective
    auto startTime = std::chrono::high_resolution_clock::now();

    MO_adjacency_matrix single_MO_matrix;
    for(auto& iter: graph){                             
        for(auto& jter: iter.second){
            single_MO_matrix[iter.first][jter.first].emplace_back(std::accumulate(jter.second.begin(), jter.second.end(),0));
        }
    }

    std::vector<double> result = SINGLE_OBJECTIVE_A_STAR(origin, target, single_MO_matrix, h, 0); 

    duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count();

    return result;
}

/**
 * ===========================================
 * VBMO A* METHOD
 * @param origin, target 
 * @param graph
 * @param h, 
 * @param duration
 * @param visualization 
 * @details "breaking up" the multi objective graph is implemented by conducting the A* search with respect to a single objective, i.e. SINGLE_OBJECTIVE_A_STAR() 
 * @return an n x n vector containing the objective cost of all singly optimized A* paths (path i is optimized for objective i).
 * @return {{}, {}, ..., {}} if no path connecting the origin and target exist.
 * ===========================================
*/
std::vector<std::vector<double>> VBMO_A_STAR(const node origin, const node target, const MO_adjacency_matrix& graph, HEURISTIC& h, int& duration) {
    auto startTime = std::chrono::high_resolution_clock::now();
    int n = graph.begin()->second.begin()->second.size();   //fetching the number of objectives
    std::vector<std::vector<double>> path_costs;
    
    for(int i = 0; i < n; i++){
        path_costs.push_back(SINGLE_OBJECTIVE_A_STAR(origin, target, graph, h, i));
    }

    duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count();
    return path_costs;
}

/**
 * WEIGHT SET GENERATORS
 * GTO = Greater then one.
*/
std::vector<double> complement_weight_set(const std::vector<double>& normalized_path_cost){
    // using complement for now, new version of weight sets are planned 
    std::vector<double> weight_set(normalized_path_cost.size());
    for(int i = 0; i < normalized_path_cost.size(); i++){
        weight_set[i] = (1 - normalized_path_cost[i]);
    }

    return weight_set;
}

/**
 * Select the smalest and largest objective cost and seek to "Compress" them by increasing the weight for path of the objective that has a weaker score to boast it, and a wieght less then one.
 * By how much? 1.5? 1.3? Complement?
 * Better way to deal with multiple mins/maxes?
 */
std::vector<double> increase_target_weight_set(const std::vector<double>& normalized_path_cost){
    std::vector<double> weight_set(normalized_path_cost.size(),1);
    double max = 0;
    double min = 0;
    for(int i = 0; i < normalized_path_cost.size(); i++){
        if(normalized_path_cost[i] < normalized_path_cost[min]) {
            min = i;
        } else if(normalized_path_cost[i] > normalized_path_cost[max]){
            max = i;
        }
    }

    weight_set[min] = .75;
    weight_set[max] = 1.25;

    return weight_set;
}

//Randomize GTO weighted
std::vector<double> randomized_weight_set(const std::vector<double>& normalized_path_cost){
    auto a = std::chrono::high_resolution_clock::now();
    int b = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - a).count();

    std::mt19937 RNG(b);

    std::vector<double> weight_set(normalized_path_cost.size());
    for(int i = 0; i < normalized_path_cost.size(); i++){
        weight_set[i] = 1 + (float)(rand())/ (float)(RAND_MAX);
    }

    return weight_set;
}

/**
 * ITERATIVE SELF-REINFORCING 
 */

/**
 * WEIGHTED A* METHOD
 * @param origin, target 
 * @param graph
 * @param weight_set is a weight set by which nodes weight are motified to induce certain search behavior
 * @param h 
 * @param duration
 * @details "breaking up" the multo objective graph is implemented by conducting the A* search with respect to a single objective, i.e. SINGLE_OBJECTIVE_A_STAR() 
 * @return an n x n vector containing the objective cost of all singly optimized A* paths (path i is optimized for objective i).
 * @return {{}, {}, ..., {}} if no path connecting the origin and target exist.
*/
std::vector<double> WEIGHTED_SINGLE_OBJECTIVE_A_STAR(node origin, node target, const MO_adjacency_matrix& graph, const std::vector<double> weight_set , const int objective, HEURISTIC& h){
    auto startTime = std::chrono::high_resolution_clock::now();

    //BOOK KEEPING 
    node_priority_queue                         openSet;
    origin.setWeight(h(origin, target));
    openSet.push(origin);

    std::unordered_map<node, node, NodeHash>    descendentList;     // maintain a list of immedaiate descendent that follow the shortest path.
    std::unordered_map<node, double, NodeHash>  scoreList;          // maintaned the shortest distance found to a node from the origin of the search.
    scoreList[origin] = 0;
    std::unordered_map<node, bool, NodeHash>    visitList;          // keeps track of nodes that we have "explpred" i.e. added its neighbors to the openSet.

    //SEARCHING
    while(!openSet.empty()){
        //since this implementation uses lazy deletion, we must discard nodoes that we have already explored.
        while(visitList.find(openSet.top()) != visitList.end()){
            openSet.pop();
        }

        node top = openSet.top();
        if(top == target){      // target is reached, begin tracing steps back towards the origin.
            int n = graph.begin()->second.begin()->second.size();   // find the number of objectives in the graph.
            std::vector<double> path_costs(n);
            node prev;
            std::list<node> shortest_path;
            shortest_path.emplace_front(top);                                           //comment in for path visualization

            while(!(top == origin)){   //begin back tracking the descendentList until we reach the origin.
                prev = descendentList[top];
                for(int i = 0; i < n; i++){
                    path_costs[i] += graph.at(prev).at(top)[i];
                }
                top = prev;
                shortest_path.emplace_front(top);                                       //comment in for path visualization
            } 
            
            return path_costs;
        } else {
            openSet.pop();
            //iterate throgh all of tops neighbors and calculate their tenative score: the distance to get to top and then to its neighbor
            for(auto& iter: graph.at(top)){
                node neighbor = iter.first;
                double tenativeScore = scoreList[top] + iter.second[objective];
                // if this neighbor has never been seen, or if this path is cheaper then a previously found path
                if(scoreList.find(neighbor) == scoreList.end() || tenativeScore < scoreList[neighbor]){
                    descendentList[neighbor] = top;         // update/set the decedant of neighbor
                    scoreList[neighbor] = tenativeScore;    // update/set the estimate score of neigbor
                    neighbor.setWeight(tenativeScore + (h(neighbor, target) * weight_set[objective]));    //update/set the nodes weight in the openSet (A weight of >1 would disenstivise exploration, a weight of <1 would speed up searchy)
                    openSet.push(neighbor);                                     //note we are using lazy deletion so the updated/better score will appear first in the open set)
                }
            }
        }
    }

    //return empty set if no path exist
    return {};
}

std::vector<std::vector<double>> WEIGHTED_VBMO_A_STAR(node origin, node target, const MO_adjacency_matrix graph, const std::vector<double>& weight_set, HEURISTIC& h, int& duration){
    auto startTime = std::chrono::high_resolution_clock::now();
    int n = graph.begin()->second.begin()->second.size();
    std::vector<std::vector<double>> path_costs;

    for(int i = 0; i < n; i++){
        path_costs.push_back(WEIGHTED_SINGLE_OBJECTIVE_A_STAR(origin, target, graph, weight_set, i, h));
    }

    duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count();
    return path_costs;
}



std::vector<double> IMPLICIT_WEIGHTED_COMBINED(node origin, node target, const MO_adjacency_matrix& graph, const std::vector<double> weight_set, HEURISTIC h, int& duration){
    auto startTime = std::chrono::high_resolution_clock::now();

    //BOOK KEEPING
    node_priority_queue                     openSet;
    origin.setWeight(h(origin, target));
    openSet.push(origin);

    std::unordered_map<node, node, NodeHash>    descendentList;     // maintain a list of immedaiate descendent that follow the shortest path.
    std::unordered_map<node, double, NodeHash>  scoreList;          // maintaned the shortest distance found to a node from the origin of the search.
    scoreList[origin] = 0;
    std::unordered_map<node, bool, NodeHash>    visitList;          // keeps track of nodes that we have "explpred" i.e. added its neighbors to the openSet.

    while(!openSet.empty()){
        while(visitList.find(openSet.top()) != visitList.end()){
            openSet.pop();
        }
        node top = openSet.top();

        if(top == target){      // target is reached, begin tracing steps back towards the origin.
            int n = graph.begin()->second.begin()->second.size();   // find the number of objectives in the graph.
            std::vector<double> path_costs(n);
            node prev;

            while(!(top == origin)){   //begin back tracking the descendentList until we reach the origin.
                prev = descendentList[top];
                for(int i = 0; i < n; i++){                         //add the the objective costs (using the graphs metrics)
                    path_costs[i] += graph.at(prev).at(top)[i];
                }
                top = prev;
            } 
            
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count();
            return path_costs;       
        } else {
            openSet.pop();
            for(auto &iter: graph.at(top)){
                node neighbor = iter.first;
                double tenative_score = scoreList[top];
                for(int i = 0; i < iter.second.size(); i++){
                    tenative_score += weight_set[i] * iter.second[i];
                } 

                if(scoreList.find(neighbor) == scoreList.end() || tenative_score < scoreList[neighbor]){ // IF the neighbor has never been discovered, or if a shorter path to the neighbor node has been found
                    descendentList[neighbor] = top;                                 // update/set the decedant of neighbor
                    scoreList[neighbor] = tenative_score;                           // update/set the estimate score of neigbor
                    neighbor.setWeight(tenative_score + h(neighbor, target));       // update/set the nodes weight in the openSet
                    openSet.push(neighbor);                                         // note we are using lazy deletion so the updated/better score will appear first in the open set)
                
                }
            }
        }
    }

    return {};
}

/**
 * @param focus is the objective that we focus on but don't
 */
std::vector<double> CONSCIOUS_SINGLE_OBJECTIVE_A_STAR(node origin, node target, const MO_adjacency_matrix& graph, const std::vector<double>& weight_set, const int focus, HEURISTIC h, int& duration){
    auto startTime = std::chrono::high_resolution_clock::now();

    //BOOK KEEPING 
    node_priority_queue                         openSet;
    origin.setWeight(h(origin, target));
    openSet.push(origin);

    std::unordered_map<node, node, NodeHash>    descendentList;     // maintain a list of immedaiate descendent that follow the shortest path.
    std::unordered_map<node, double, NodeHash>  scoreList;          // maintaned the shortest distance found to a node from the origin of the search.
    scoreList[origin] = 0;
    std::unordered_map<node, bool, NodeHash>    visitList;          // keeps track of nodes that we have "explpred" i.e. added its neighbors to the openSet.

    //SEARCHING
    while(!openSet.empty()){
        //since this implementation uses lazy deletion, we must discard nodoes that we have already explored.
        while(visitList.find(openSet.top()) != visitList.end()){
            openSet.pop();
        }

        node top = openSet.top();
        if(top == target){      // target is reached, begin tracing steps back towards the origin.
            int n = graph.begin()->second.begin()->second.size();   // find the number of objectives in the graph.
            std::vector<double> path_costs(n);
            node prev;

            while(!(top == origin)){   //begin back tracking the descendentList until we reach the origin.
                prev = descendentList[top];

                for(int i = 0; i < n; i++){                         //add the the objective costs (using the graphs metrics)
                    path_costs[i] += graph.at(prev).at(top)[i];
                }

                top = prev;
            } 
            
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count();
            return path_costs;
        } else {
            openSet.pop();
            //iterate throgh all of tops neighbors and calculate their tenative score: the distance to get to top and then to its neighbor
            for(auto& iter: graph.at(top)){
                node neighbor = iter.first; 

                double tenativeScore = scoreList[top] + iter.second[focus];

                std::vector<double> neighborScores = iter.second;
                for(int i = 0; i < neighborScores.size(); i++){
                    if(i != focus){
                        tenativeScore += weight_set[i]*neighborScores[i];       //tenative score is the distance to the 
                    }
                }

                // if this neighbor has never been seen, or if this path is cheaper then a previously found path
                if(scoreList.find(neighbor) == scoreList.end() || tenativeScore < scoreList[neighbor]){

                    descendentList[neighbor] = top;                             // update/set the decedant of neighbor
                    scoreList[neighbor] = tenativeScore;                        // update/set the estimate score of neigbor
                    neighbor.setWeight(tenativeScore + h(neighbor, target));    //update/set the nodes weight in the openSet
                    openSet.push(neighbor);                                     //note we are using lazy deletion so the updated/better score will appear first in the open set)

                }
            }
        }
    }

    //return empty set if no path exist
    return {};
}


void VBMO_2(const node& origin, const node& target, MO_adjacency_matrix& graph, HEURISTIC h, std::string voteScheme, int& duration){
    int time0, time1, time2, timeN;

    std::vector<std::vector<double>> path_costs = VBMO_A_STAR(origin, target, graph, h, time0);
    std::vector<std::vector<double>> norm_path_costs = normalize_matrix(path_costs);

    if(path_costs[0].size() == 0){
        std::cout << "NO PATH FOUND" << std::endl;
        return;
    }
    
    display_matrix(path_costs);
    std::cout << "===============" << std::endl;
    display_matrix(norm_path_costs);

    
    std::vector<int> path_ranking = vote(path_costs, voteScheme);
    display_vector(path_ranking);

    std::cout << "===============" << std::endl;    

    for(int i = 0; i < 2; i++){
        std::cout << "-------" << i << "--------" << std::endl;    
        
        std::vector<double> norm_selected_path = norm_path_costs[path_ranking[i]];
        std::vector<double> selected_path = path_costs[path_ranking[i]];    // NOT NEEDED
        std::vector<double> weight_set = complement_weight_set(norm_selected_path);
        
        std::cout << "Weight set             : "; display_vector(weight_set);
        
        
        std::vector<double> conscious_path, combined_weighted;

        int focus = 0;
        for(int i = 0; i < selected_path.size(); i++){
            if(selected_path[i] < selected_path[i]){
                focus = i;
            }
        }

        
        conscious_path = CONSCIOUS_SINGLE_OBJECTIVE_A_STAR(origin, target, graph, weight_set, focus, h, time1);
        combined_weighted = IMPLICIT_WEIGHTED_COMBINED(origin, target, graph, weight_set, h, time2);

        
        std::cout << i <<  "th place candidate    : "; display_vector(selected_path);
        std::cout << "conscious path         : ";      display_vector(conscious_path);
        std::cout << "combined weighted      : ";      display_vector(combined_weighted);

        std::cout << std::endl;
        std::cout << "d-score" << std::endl;
        std::cout << i <<  "th place candidate    : " <<  path_d_score(selected_path) << std::endl;
        std::cout << "conscious path         : " <<       path_d_score(conscious_path) << std::endl;
        std::cout << "combined weighted      : " <<       path_d_score(combined_weighted) << std::endl;
             
        std::cout << std::endl;
        std::cout << "VBMO time              : " << time0 << " ms" << std::endl;
        std::cout << "conscious time         : " << time1 << " ms" <<  std::endl;
        std::cout << "combined weighted time : " << time2 << " ms" <<  std::endl;
        std::cout << "TOTAL                  : " << time0 + time1 + time2 << " ms " << std::endl;
        
    }
}


//method for reading in DOT file and forming the graph looking at the .co, distance, and time file
MO_adjacency_matrix CONSTRUCT_GRAPH(std::string place) {
    std::string dir = "USA-road/USA-road-" + place;
    std::ifstream ifs;
    std::string x; 
    auto begin = std::chrono::high_resolution_clock::now(); 

    std::vector<std::string> files;
    for(auto file_iter : std::filesystem::directory_iterator(dir)){
        files.push_back(file_iter.path());
    }

    std::reverse(files.begin(), files.end());
    for(auto file : files){
        std::cout << file << std::endl;
    }





    return {{}};

    

    ifs.open(file_iter->path(), std::ifstream::in);
    for(int i = 0; i < 4; i++){
        ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    //get node and edge count;
    std::getline(ifs, x);
    int numNodes = stoi(x.substr(12));  //get number of nodes in the environment

    // std::cout << numNodes << std::endl;
    ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    std::vector<node> nodeList;

    int count = 0;
    double id, lat, lng; 
    while(ifs >> x){
        if(x == "v"){
            ifs >> id >> lat >> lng; 
            nodeList.push_back(node(lat, lng, id));
            count++;
        }
    }
    std::cout << nodeList.size() << " vertices created." << std::endl;

    std::cout << "Finished reading vertex file." << std::endl;
    ifs.close();

//
// DISTANCE
//

    file_iter++;
    std::cout << file_iter->path() << std::endl;

    ifs.open(file_iter->path(), std::ifstream::in);
    for(int i = 0; i < 4; i++){                                         //ignore header
        // ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        getline(ifs, x);
        std::cout << x << std::endl;
    }

    // recieve vertex count V, and edge count E;
    int V, E;
    ifs >> x >> x >> V >> E;
    std::cout << "Graph contain " << V << " vertices, and "<< E << " edges" <<  std::endl;   
    
    ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    // ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    MO_adjacency_matrix adjMatrix;
    int start, end;
    double obj;
    count = 0;
    while(ifs >> x){
        if(x == "a") {
            ifs >> start >> end >> obj;
            std::cout << start << " " << end << std::endl;
            adjMatrix[nodeList[start-1]][nodeList[end-1]] = {obj};
            count++;   
        }
    }

    int doubleCheck = 0;
    for(auto& iter: adjMatrix){
        doubleCheck += iter.second.size();
    }

    std::cout << "Finish reading distance file." << std::endl;
    std::cout << "expected: " << E << std::endl;
    std::cout << "actual:   " << count << std::endl; 
    std::cout << "in graph: " << doubleCheck << std::endl;
    ifs.close();

//
// TIME 
//

    file_iter++;
    std::cout << file_iter->path() << std::endl;


    ifs.open(file_iter->path(), std::ifstream::in);
    for(int i = 0; i < 7; i++){
        ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    count = 0;
    while(ifs >> x){
        if(x == "a"){
            ifs >> start >> end >> obj;
            adjMatrix[nodeList[start-1]][nodeList[end-1]].push_back(obj);
            count++;    
        }
    }
    
    doubleCheck = 0;
    for(auto& iter: adjMatrix){
        doubleCheck += iter.second.size();
    }

    std::cout << "Finish reading time file." << std::endl;
    std::cout << "expected: " << E << std::endl;
    std::cout << "in graph: " << doubleCheck << std::endl;
    ifs.close();
    
    
    //adding uniform and random cost
    std::mt19937 RNG(std::chrono::high_resolution_clock::duration(std::chrono::high_resolution_clock::now() - begin).count()); 

    for(auto& iter: adjMatrix){
        for(auto& jter: iter.second){
            jter.second.emplace_back(1000);
            jter.second.emplace_back(RNG() % 37000);
        }
    }
    // every edge should have exactly four objectives: distance, time, uniform, and random (in that order)
    
    for(auto& iter: adjMatrix){
        for(auto& jter: iter.second){
            if(jter.second.size() != 4) {
                display_vector(jter.second);
                iter.first.display(); jter.first.display();
                return {};
            }
        }
    }

    return adjMatrix;
}    

int main(){
    auto begin = std::chrono::high_resolution_clock::now();

    MO_adjacency_matrix graph = CONSTRUCT_GRAPH("BAY");
    
    
    // int m = graph.size();
    // int a(rand() % m), b(rand() % m);

    // node origin = std::next(graph.begin(), a)->first;
    // node target = std::next(graph.begin(), b)->first;

    // origin.display(); std::cout << " to "; target.display();

    // HEURISTIC h;
    // int duration;


    return 0;


}
