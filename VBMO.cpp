/**
 * VBMO and VBMO* implementation in C++
 * CREDITS: Raj Korpan, Daniel Merino
*/
#include <numeric>
#include <vector>           // vectors
#include <string>           // getline() and reading files
#include <iostream>         // input and output functionality
#include <fstream>          // saving maps, paths, and other data to external files 
#include <unordered_map>    // for lookup tabled 
#include <cmath>            // sqrt(), abs(), and other basic math functions
#include <queue>            // using priority_queue as the openSet for A*
#include <list>             // shortest path are put into lists for their quick emplace_front() feature
#include <chrono>           // for timing and random number generator seed
#include <random>           // using the mersenne_twister_engine for random numbers
#include <iterator>         // ostream iterator 
#include <filesystem>       // for filesystem iterators

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

/**
 * @brief metric for evaluating a pareto-front approxination
 */
// Does it need to be normalized?
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

// same as above but does not create a new line
void alt_display_vector(std::vector<int> vec){
     if(vec.size() == 0) {
        std::cout << "{}" << std::endl;
    } else {
        std::cout << "{" << vec[0];
        for(int i = 1; i < vec.size(); i++){
            std::cout << ", " << vec[i];
        }
    }
    std::cout << "}"; // not end;
}

void alt_display_vector(std::vector<double> vec){
     if(vec.size() == 0) {
        std::cout << "{}" << std::endl;
    } else {
        std::cout << "{" << vec[0];
        for(int i = 1; i < vec.size(); i++){
            std::cout << ", " << vec[i];
        }
    }
    std::cout << "}"; // not end;
}


/**
 * @details Used for printng the paths
*/
void display_matrix(std::vector<std::vector<double>> matrix) {
    if(matrix.size() == 0){
        std::cout << "{}" << std::endl;
        return;
    }
    for(int i = 0; i < matrix.size(); i++) {
            if(i >= 10) {
                std::cout << "P_" << i << ": ";   
            } else {
                std::cout << "P_" << i << " : ";
            }
         display_vector(matrix[i]);
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

// display vector and their D-score
void alt_display_matrix(std::vector<std::vector<double>> matrix){
    if(matrix.size() == 0){
        std::cout << "{{}}" << std::endl;
        return;
    } else {
        for(int i = 0; i < matrix.size(); i++){
            if(i >= 10) {
                std::cout << "P_" << i << ": ";   
            } else {
                std::cout << "P_" << i << " : ";
            }
            alt_display_vector(matrix[i]); std::cout << " " << path_d_score(matrix[i]) << std::endl;
        }
    }
    
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
// int VOTE(std::vector<std::vector<double>>& normalized_path_score, const std::string voting_method) {
//     std::vector<double> results;
//     int winner; 

//     if(voting_method == "range") {
//         results = range_voting(normalized_path_score);
//         winner = findMin(results);
//         //lowest wins
//     } else if(voting_method == "combined_approval")  {
//         results = combined_aproval_voting(normalized_path_score);
//         winner = findMax(results);
//         //highest win
//     } else if(voting_method == "borda") {
//         results = borda_voting(normalized_path_score);
//         winner = findMin(results);
//         //lowest win
//     } else if(voting_method == "condorcet") {
//         results = condorcet_voting(normalized_path_score);
//         winner = findMax(results);        //highest win
//     } else {
//         std::cout << "Invalid voting method" << std::endl;
//         return {};
//     }

//     // display_vector(results);
//     if(winner == -1) {
//         std::cout << "Tie detected" << std::endl;
//         std::vector<double> d_scores = d_score(normalized_path_score);;

//         // display_vector(d_scores);
//         winner = findMin(d_scores);
//         if(winner == -1){
//             std::cout << "TIE in d_score!" << std::endl;
//             return 0;
//         }
//     }
//     std::cout << "====================" << std::endl;
//     std::cout << "Winner is P_" << winner << ": "; display_vector(normalized_path_score[winner]);
//     std::cout << "====================" << std::endl;

//     return winner;
// }

// return a vector of ints where the value of v[i] is the ranked value of path_scores[i] decreasing
// std::vector<int> rank_decreasing_order(std::vector<double> path_scores){
//     std::priority_queue<int, std::vector<int>, std::less<int>> temp;
//     for(int i : path_scores){
//         temp.push(i);
//     }
//     std::vector<int> order;
//     while(!temp.empty()){
//         order.push_back(temp.top());
//         temp.pop();
//     }

//     return order;
// }


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


// std::vector<int> rank_increasing_order(std::vector<double> path_scores){
//     std::priority_queue<int, std::vector<int>, std::greater<int>> temp;
//     for(int i : path_scores){
//         temp.push(i);
//     }
//     std::vector<int> order;
//     while(!temp.empty()){
//         order.push_back(temp.top());
//         temp.pop();
//     }

//     return order;
    
// }


/**
 * @detail: 
 *
 */
struct path_greater{
    bool operator()(const std::pair<double, int> a, std::pair<double, int> b, std::vector<std::vector<double>> path_costs) const {
        if(a.first == b.first){
            return path_d_score(path_costs[a.second]) < path_d_score(path_costs[b.second]);
        } else{
            return a < b;
        }
    }
};

//
std::vector<int> rank_increasing_order(std::vector<double> path_scores){
    path_greater COMP;
    std::priority_queue<std::pair<std::pair<double,int>, int>, std::vector<std::pair<double,int>>, std::less<std::pair<double,int>>> temp;
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
    // dealing with tied, put the one with the lower d-score first

    //return a vector of ints where the value at v[i] is the path in the i+1th place 
    // e.g. the value at index 0 is the winner, index 1 has the 2nd place, etc.
    return order;
}

/**
 * ===========================================
 * NODE OBJECTECT (VERTEX) FOR GRAPHS THAT USE X, Y CORDINATES
 * @details the nodes themselves contain the key by which the openSet in A* ranks the nodes.
 * ===========================================
*/
class node {
    private:
    int x,y;
    double weight;

    public:

    node(){
        x = 0;
        y = 0;
    };

    node(int a, int b): x(a), y(b) {};

    node(const node& rhs) {
        x = rhs.x;
        y = rhs.y;
        weight = rhs.weight;
    };

    node(const node&& rhs) {
        x = rhs.x;
        y = rhs.y;
        weight = rhs.weight;
    };

    node& operator=(const node& rhs) {
        x = rhs.x;
        y = rhs.y;
        weight = rhs.weight;

        return *this;
    };

    node& operator=(node&& rhs) {
        x = rhs.x;
        y = rhs.y;
        weight = rhs.weight;

        return *this;
    };

    int getX() const {
        return x;
    };

    int getY() const {
        return y;
    };

    bool operator==(const node& rhs) const {
        return (x == rhs.x) && (y == rhs.y);
    };

    bool operator!=(const node& rhs) const {
        return (x != rhs.x) || (y != rhs.y);
    };

    void setWeight(double W){
        weight = W;
    };

    double getWeight() const {
        return weight;
    };

    void display() const {
        std::cout << "(" << x << ", " << y << ")";
    };

};

/**
 * @details a heuristic fuctor that has an internal state that determins which determins which heuristic to use,
 *          supported heuristisc are: Euclidean, Manhattan, and Octagonal.
*/
class HEURISTIC {
    private:
        int state = 0;

    public:
    HEURISTIC(int a){
        state = a;
    }

    HEURISTIC() {
        state = 0;
    }

    void change(int a){
        state = a;
        //invalid hueristic?
    }

    double operator()(const node& origin, const node& target) {
        //EUCLIDEAN 
        if(state == 0){
            return sqrt( pow((origin.getX() - target.getX()), 2) + pow((origin.getY() - target.getY()), 2) );
        } else if(state == 1){
            return abs(origin.getX() - target.getX()) + abs(origin.getY() - target.getY());
        } else {
            return -1;
        }
    }
};



// Use this for when the order of nodes does not mater (e.g. unordered_map & look up tables)
struct NodeHash {
    size_t operator()(const node& key) const {
        return key.getX() ^ key.getY();
    }
};

// Use this when the order of the nodes based of their weight (distance from parent node and )
struct PQNodeHash{
    size_t operator()(const node& a, const node& b){
        return a.getWeight() > b.getWeight();
    }
};

/**
 * TYPEDEF for adjacency matrices and priority queues for A* 
*/
typedef std::unordered_map<node, std::unordered_map<node, double, NodeHash>, NodeHash>              SO_adjacency_matrix;
typedef std::priority_queue<node, std::vector<node>, PQNodeHash>                                    node_priority_queue; 
typedef std::unordered_map<node,std::unordered_map<node, std::vector<double>, NodeHash>, NodeHash>  MO_adjacency_matrix;     

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
 * VBMO OPPONENT, A NAIVE APPROACH VIA SUMMING THE OBJECIVES INTO A SINGLE OBJECTIVE
 * @param origin, target are the start and end point for the search algorithm.
 * @param graph is a n objective adjacency matrix.
 * @param h is the heuristic functor that can be set in the driver code to select a supported heuristic function (see HUERISTIC for more details).
 * @details the first step is to iteratore through the entire adjacency matrix and sum all the objective cost into one objective. Then run A* with the sole objective
 * @return a vector with the shorest path with respect to the combined objective.
 * @return {} if no path from the orgigin and target exist.
*/
std::vector<double> COMBINED_MO_A_STAR(node origin, node target, const MO_adjacency_matrix graph, HEURISTIC& h, int& duration){
    //sum the objective costs into one objective
    auto startTime = std::chrono::high_resolution_clock::now();

    MO_adjacency_matrix copy_graph = graph;

    for(auto& iter : copy_graph){
        for(auto& jter : iter.second){
            jter.second.emplace_back(std::accumulate(jter.second.begin(), jter.second.end(), 0));
        }
    }
    
    int n = copy_graph.begin()->second.begin()->second.size();   
    std::vector<double> result = SINGLE_OBJECTIVE_A_STAR(origin, target, copy_graph, h, n-1);

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
 * ===========================================
 * TURNING DAO MAP INTO MATRIX
 * @param DAO map is the file name of a map from the DAO director (or any map file that follows its ASCI encoding)
 * @details Reads the inpute file and constructs a multi-objectie graph with the following objectives:
 *              1. Euclidian distance,
 *              2. Uniform distance (1.5),
 *              3. Random (range between 0 - 20),
 *              4. Safety (average of the nodes degree, nodes with fewer edges (i.e. those near walls) are higher), and 
 *              5. Danger (nodes at randomy are selected to have expensive incoming and outcoming edges, non selected nodes have a uniform cost of 1.5).
 * @return a 5 objective adjacecny matrix based off the DAOmap.
 * ===========================================
*/
MO_adjacency_matrix DAO_MAP_TO_MO_ADJ_MATRIX(std::string DAOmap){
    auto begin = std::chrono::high_resolution_clock::now();

    std::ifstream ifs;
    DAOmap = "dao-map/" + DAOmap;
    ifs.open(DAOmap, std::ifstream::in);

    std::string x;
    std::getline(ifs,x);// remove "type ..."

    std::getline(ifs,x);    //getting the height
    int height = stoi(x.substr(7));

    std::getline(ifs,x);    //getting the width 
    int width = stoi(x.substr(6));

    std::getline(ifs,x); // remove "map"

    /**
     * TURNING THE MAP INTO GRID
    */

    // proble in loop or 
    std::vector<std::vector<char>> map (height, std::vector<char> (width));
    for(int i = 0; i < height; i++){
        std::getline(ifs,x);
        for(int j = 0;j < width; j++){
            // std::cout << i << " " << j << std::endl;
            map[i][j] = x[j];
        }
    }

    MO_adjacency_matrix AdjMatrix;  
    height = map.size();
    width = map[0].size();

    //random engine for the random objective uses the current time as a seed.
    std::mt19937 RNG(std::chrono::high_resolution_clock::duration(std::chrono::high_resolution_clock::now() - begin).count()); 

    //constructing the adjacecny matrix for the grid
    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            auto begin = std::chrono::high_resolution_clock::now();

            if(map[i][j] == '.'){
                // create node and add its to the matrix 
                node cur(i,j);
                AdjMatrix[cur];
                if(i+1 < height && map[i+1][j] == '.'){
                    AdjMatrix[cur][node(i+1,j)] = {1, 1.5, double(RNG() % 20)};
                }
                //down
                if(i-1 >= 0 && map[i-1][j] == '.') {
                    AdjMatrix[cur][node(i-1,j)] = {1, 1.5, double(RNG() % 20)};
                }
                //right `
                if(j+1 < width && map[i][j+1] == '.'){
                    AdjMatrix[cur][node(i,j+1)] = {1, 1.5, double(RNG() % 20)};
                }
                //left
                if(j-1 >= 0 && map[i][j-1] == '.'){
                    AdjMatrix[cur][node(i,j-1)] = {1, 1.5, double(RNG() % 20)};
                }
                //down right
                if(i+1 < height && j+1 < width && map[i+1][j+1] == '.'){
                    AdjMatrix[cur][node(i+1,j+1)] = {1.413, 1.5, double(RNG() % 20)};
                }
                //up right
                if(i-1 >= 0 && j+1 < width && map[i-1][j+1] == '.'){
                    AdjMatrix[cur][node(i-1,j+1)] = {1.413, 1.5, double(RNG() % 20)};
                }
                //down left
                if(i+1 < height && j-1 >= 0 && map[i+1][j-1] == '.') {
                    AdjMatrix[cur][node(i+1,j-1)] = {1.413, 1.5, double(RNG() % 20)};
                }
                //up left
                if(i-1 >= 0 && j-1 >= 0 && map[i-1][j-1] == '.') {
                    AdjMatrix[cur][node(i-1,j-1)] = {1.413, 1.5, double(RNG() % 20)};
                }
            }
        }
    }

    //adding safety objective and randonmy pick dangerous nodes
    std::vector<node> dangerousNodes;
    for(auto& iter: AdjMatrix){
        if(rand() % 10 == 0) {  // 10% chance of a node being dangerous, all edges into a dangerous node are expensive 
            dangerousNodes.emplace_back(iter.first);
        }
        for(auto& jter: iter.second){
            jter.second.emplace_back( (10 - AdjMatrix.at(iter.first).size()  + 10 - AdjMatrix.at(jter.first).size() ) / 2 );   //safety objective 
            jter.second.emplace_back(1.5);  //if the node is selected to be dangerous, then this vaue will be changed to 10; in the next pass        
        }
    }

    //make dangerous nodes expensive 
    for(auto& iter: dangerousNodes){
        for(auto& jter: AdjMatrix.at(iter)){    
            jter.second.back() = 10;                        //make incoming edge expensive 
            *AdjMatrix.at(jter.first).at(iter).end() = 10;  //make outgoing edge expensive
        }
    }

    return AdjMatrix;
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

//add duration to subtract from algorithms...
std::list<node> trace_shortest_path(node origin, node target, const std::unordered_map<node, node, NodeHash>& descendentList){
    std::list<node> shortest_path;
    shortest_path.push_front(target);
    node temp = target;
    while(descendentList.find(temp) != descendentList.end()){   //while there are still decedants i.e. we are not at origin
        temp = descendentList.at(temp);
        shortest_path.push_front(temp);
    }
    //fix offset since the origin has no starting
    shortest_path.push_front(origin);

    return shortest_path;

}

std::vector<double> difference(std::vector<double>& before, std::vector<double>& after){
    std::vector<double> diff(before.size());
    for(int i = 0; i < before.size(); i++){
        diff[i] = before[i] - after[i];
    }
    return diff;
}

/**
 * Format will be the same as the order of the paramaters 
 */
void SAVE(node origin, node target, std::vector<double> VMBOresult, int VBMOduration){

    std::ofstream results_file("./VBMO_RESULTS.txt");

    
    results_file.close();
}

/**
 * @brief updates the graph with the weighted cost of the objective of those edgeb
 * @param graph is the graph taht we wish to add
 * @param winner is the winning path from VBMO 
 */
void COMBINED_WEIGHTED_UPDATE(MO_adjacency_matrix& graph, const std::vector<double> weight_set){
    for(auto& iter: graph){
        for(auto& jter: iter.second){
            double weighted_sum = 0;
            for(int i = 0; i < jter.second.size(); i++){
                weighted_sum += weight_set[i]*jter.second[i];
            }
            jter.second.push_back(weighted_sum);
        }
    }
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
        display_vector(norm_selected_path);
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
        std::cout << "d-score (smaller is better)" << std::endl;
        std::cout << i <<  "th place candidate    : " <<  path_d_score(selected_path) << std::endl;
        std::cout << "conscious path         : " <<       path_d_score(conscious_path) << std::endl;
        std::cout << "combined weighted      : " <<       path_d_score(combined_weighted) << std::endl;
             
        std::cout << std::endl;
        std::cout << "VBMO time              : " << time0 << " ms" << std::endl;
        std::cout << "conscious time         : " << time1 << " ms" <<  std::endl;
        std::cout << "combined weighted time : " << time2 << " ms" <<  std::endl;
        std::cout << "TOTAL                  : " << time0 + time1 + time2 << " ms " << std::endl;

    }

    //RUNING ADVERSITY
    std::cout << "------naive-----" << std::endl;
    int timec;
    std::vector<double> naive_path_cost = COMBINED_MO_A_STAR(origin, target, graph, h, timec);   

    naive_path_cost.pop_back();
    std::cout << "naive combined         : "; display_vector(naive_path_cost);
    std::cout << "naive combined time    : " << timec << std::endl;
    std::cout << "naive d-score          : " << path_d_score(naive_path_cost) << std::endl;
}

/**
 *  SINGLE ITERATION 
 */
// void VBMO(const node& origin, const node& target, MO_adjacency_matrix& graph, HEURISTIC& h, std::string voteScheme, int& duration) {
//     int time0 = 0, time1 = 0, time2 = 0;
//     std::vector<std::vector<double>> results; // will contain the winning VBMO 
    
//     std::vector<std::vector<double>> raw_path_costs = VBMO_A_STAR(origin, target, graph, h, time0);
//     std::vector<std::vector<double>> norm_path_costs = normalize_matrix(raw_path_costs);

//     std::cout << "RAW PATH COSTS:" << std::endl;
//     display_matrix(raw_path_costs);
//     std::cout << "NORMALIZED COSTS:" << std::endl;
//     display_matrix(norm_path_costs);
//     std::cout << "VBMO time: " << time0 << " ms" << std::endl;
//     std::cout << "VBMO SPARCITY: " << sparcity_metric(raw_path_costs) << std::endl;
    
//     int winner = VOTE(norm_path_costs, voteScheme);
//     std::vector<double> norm_winning_path = norm_path_costs[winner];
//     std::vector<double> raw_winning_path = raw_path_costs[winner];
//     results.push_back(raw_winning_path);

//     // GENERATING WEIGHT SET
//     std::vector<double> weight_set = complement_weight_set(norm_winning_path);
//     std::cout << "WEIGHT SET GENERATED" << std::endl;
//     display_vector(weight_set);  

//     // Which objective to we choose to be conscious of in the first place? 
//     int focus = 0;
//     for(int i = 0; i < norm_winning_path.size(); i++) {
//         if(norm_winning_path[i] < norm_winning_path[focus]){
//             focus = i;
//         }
//     }

//     //CONSCIOUS RUN
//     std::vector<double> conscious_path_cost = CONSCIOUS_SINGLE_OBJECTIVE_A_STAR(origin, target, graph, weight_set, 1, h, time1);
//     std::cout << "=========================" << std::endl;   
//     std::cout << "CONSCIOUS PATH:" << std::endl;
//     std::cout << "time: " << time1 << " ms" << std::endl;

//     //add the conscious path to the pool of possible answers
  
//     results.push_back(conscious_path_cost);
//     display_vector(conscious_path_cost); 

//     //COMBINED WEIGHTED OBJECTIVE RUN
//     auto combinedStart = std::chrono::high_resolution_clock::now(); //timing should also include the time it takes to update the graph.
//     COMBINED_WEIGHTED_UPDATE(graph, weight_set);
//     std::cout << "=========================" << std::endl;
//     std::cout << "COMBINED WEGHTED PATH:" << std::endl;
//     //run A* but on the last objective
//     std::vector<double> combined_path_cost = SINGLE_OBJECTIVE_A_STAR(origin, target, graph, h, graph.begin()->second.begin()->second.size()-1);
//     display_vector(combined_path_cost);   
//     combined_path_cost.pop_back();  //remove the combined weighted objective
//     time2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - combinedStart).count();
//     std::cout << "time: " << time2 << std::endl;

//     //add combined weighted path to pool of possible answers
//     results.push_back(combined_path_cost);     
//     display_vector(combined_path_cost);

//     std::vector<std::vector<double>> norm_results = normalize_matrix(results);

//     std::cout << "=========================" << std::endl;
//     std::cout << "original:  "; display_vector(results[0]);
//     std::cout << "conscious: "; display_vector(results[1]);
//     std::cout << "combined:  "; display_vector(results[2]);
    
//     std::cout << "=========================" << std::endl;
//     std::cout << "normalized original  : "; display_vector(norm_results[0]);
//     std::cout << "normalized conscious : "; display_vector(norm_results[1]);
//     std::cout << "normalized combined  : "; display_vector(norm_results[2]);
    
//     std::vector<double> results_d_score = d_score(results);
//     std::cout << "=========================" << std::endl; 
//     std::cout << "original d-score:   " << results_d_score[0] << std::endl;
//     std::cout << "conscious d-score:  " << results_d_score[1] << std::endl;
//     std::cout << "combined d-score:   " << results_d_score[2] << std::endl;

//     std::cout << "Total Time: " << time1 + time2 << std::endl;
//     std::cout << "New Sparcity metric: " << sparcity_metric(results) << std::endl;
// }



bool VBMO_MAX(const node& origin, const node& target, MO_adjacency_matrix& graph, HEURISTIC h, std::string voteScheme, int& duration){
    int time0, time1, time2, timeN;

    std::vector<std::vector<double>> path_costs = VBMO_A_STAR(origin, target, graph, h, time0);
    std::vector<std::vector<double>> norm_path_costs = normalize_matrix(path_costs);
    if(path_costs[0].size() == 0){
        // std::cout << "NO PATH FOUND" << std::endl;
        return false;
    }
    // display_matrix(path_costs);
    // std::cout << "===============" << std::endl;
    // display_matrix(norm_path_costs);
    // will hold all generated paths in the following order:
    // 1, originakl
    // 2. conscious 
    // 3. weighted combined
    std::vector<std::vector<double>> FRONT;
    for(int i = 0; i < path_costs.size(); i++){
        // std::cout << "P_" << i << " : "; display_vector(path_costs[i]);
        // std::cout << "norm: "; display_vector(norm_path_costs[i]);
        std::vector<double> weight_set = complement_weight_set(norm_path_costs[i]);
        // std::cout << "w   : "; display_vector(weight_set);
        int focus = 0; 
        for(int j = 0; j < path_costs[0].size(); j++){
            if(path_costs[i][j] < path_costs[i][focus]){
                focus = j;
            }
        }
        std::vector<double> conscious_path_cost = CONSCIOUS_SINGLE_OBJECTIVE_A_STAR(origin, target, graph, weight_set, focus, h, time0);
         
        std::vector<double> weighted_combined_cost = IMPLICIT_WEIGHTED_COMBINED(origin, target, graph, weight_set, h, time1);

        // std::cout << "c   : "; display_vector(conscious_path_cost);
        // std::cout << "wc  : "; display_vector(weighted_combined_cost);

        // std::cout << "=============" << std::endl;

        FRONT.push_back(path_costs[i]);
        FRONT.push_back(conscious_path_cost);
        FRONT.push_back(weighted_combined_cost);
        // timeN += time0 + time1 + time2;
    }

    alt_display_matrix(FRONT);
    // std::cout << timeN << " ms or " << timeN / 1000 << " seconds" <<  std::endl;

    std::vector<std::vector<double>> NORM_FRONT = normalize_matrix(FRONT);

    std::cout << "---------------" << std::endl;
    display_matrix(NORM_FRONT);

    std::cout << "---------------" << std::endl;
    std::vector<int> voting_results = vote(NORM_FRONT, voteScheme);
    for(int i = 0; i < 3; i++){
        std::cout << i + 1 << "th place : "; alt_display_vector(FRONT[voting_results[i]]); std::cout << ", d-score: " << path_d_score(FRONT[voting_results[i]]) << std::endl;
    }

    return true;
}



void VBMO_RANDOM_POINT_RUNNER_MULTI(std::string file_name, int trials){
    auto begin = std::chrono::high_resolution_clock::now();
    
    std::vector<std::string> RESULTS;
    MO_adjacency_matrix graph = DAO_MAP_TO_MO_ADJ_MATRIX(file_name);

    for(int i = 0; i < trials; i++){
        std::mt19937 (std::chrono::high_resolution_clock::duration(std::chrono::high_resolution_clock::now() - begin).count()); 
                
        int m = graph.size();
        int a(rand() % m), b(rand() % m);

        node origin = std::next(graph.begin(), a)->first;
        node target = std::next(graph.begin(), b)->first;

        HEURISTIC h;
        int duration;
        
        // VBMO_1(origin, target, graph, h, "borda", duration);
        std::cout << "trial " << i << std::endl;
        origin.display(); std::cout << " to "; target.display(); std::cout << std::endl;
        
        if(!VBMO_MAX(origin, target, graph, h, "borda", duration)){
            i--; // redoo
            continue;
        }

        // node origin = graph.begin()->first;
        // node target = graph.begin()->first;

    
    }
}

void VBMORUNNER(){
    //iterators through all the files 
    std::string dir = "dao-map";
    std::ifstream ifs;
    std::string x;

    //while not at the end of the directory
    for(auto& file_iter: std::filesystem::directory_iterator(dir)){
        std::string file_name = file_iter.path().filename().string();
        std::cout << "============================================================" << std::endl;
        std::cout << file_name << std::endl;
        // VBMO_RANDOM_POINT_RUNNER(file_name);
        VBMO_RANDOM_POINT_RUNNER_MULTI(file_name, 1);
    }
}


int main() {
    VBMORUNNER();
    return 0;
}
