/**
 * VBMO IMPLEMENTAION IN C++ 
 * CREDITS: Raj Korpan, Daniel Merino
*/
#include <vector>           // vectors
#include <string>           // getline() and reading files
#include <iostream>         // input and output functionality
#include <fstream>          // saving maps, paths, and other data to external files 
#include <unordered_map>    // for lookup tabled 
#include <cmath>            // sqrt(), abs(), and other basic math functions
#include <queue>            // using priority_queue as the openSet for A*
#include <algorithm>        // ???
#include <list>             // shortest path are put into lists for their quick emplace_front() feature
#include <chrono>           // for timing and random number generator seed
#include <random>           // using the mersenne_twister_engine for random numbers
#include <iterator>         // ostream iterator 

/**
 * ===========================================
 * HELPER FUNCTIOIONS
 * ===========================================
*/

/**
 * @details an inplace method of normalzing a one-dimensional vector
 * @param path_cost is a 1-d vector
*/
void normalize_vector(std::vector<std::vector<double>>& path_costs){
    for(int i = 0; i < path_costs.size(); i++){
        double magnitude;
        for(int j = 0; j < path_costs[0].size(); j++){
            magnitude += path_costs[i][j] * path_costs[i][j];
        }
        magnitude = sqrt(magnitude);

        for(int k = 0; k < path_costs[0].size(); k++){
            path_costs[i][k] /= magnitude;
        }
    }
}

/**
 * @details a method for displaying a one-dimensional vector
*/
void display_vector(std::vector<double> normalized_path_cost) {
    if(normalized_path_cost.size() > 0) {
        std::cout << "{" << normalized_path_cost[0];
        for(int i = 1; i < normalized_path_cost.size(); i++) {
            std::cout << ", " << normalized_path_cost[i];
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

    double min = std::numeric_limits<double>::max(); //
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
            std::cout << top.first << " " << top.second << " " << rank << std::endl;
            pq.pop();
        }
        std::cout << "----" << std::endl;     //debug
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
 * @details When a tie is detected in a voting scheme, we pick the candidate that is closest to the Pareto space origin ({0, 0, 0, ..., 0}).
 * @return a n sized vector where vector[i] is the euclidean distance in Pareto space between path i and the origin.
*/
std::vector<double> d_score(std::vector<std::vector<double>> normalized_path_costs) {
    std::vector<double> d_score = {};
    for(int i =0; i < normalized_path_costs.size(); i++) {
        double path_squared_sum = 0;
        for(int j = 0; j < normalized_path_costs[i].size(); j++){
            path_squared_sum += std::pow(normalized_path_costs[i][j],2);
        }
        d_score.emplace_back(std::sqrt(path_squared_sum));
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
std::vector<double> VOTE(std::vector<std::vector<double>>& normalized_path_score, const std::string voting_method) {
    std::vector<double> results;
    std::vector<double> d_scores;
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
        winner = findMax(results);
        //highest win
    } else {
        std::cout << "Invalid voting method" << std::endl;
        return {};
    }

    // display_vector(results);
    if(winner == -1) {
        std::cout << "Tie detected" << std::endl;
        d_scores = d_score(normalized_path_score);
        // display_vector(d_scores);
        winner = findMin(d_scores);
    }
    std::cout << "===============" << std::endl;
    std::cout << "Winner is P_" << winner << ": "; display_vector(normalized_path_score[winner]);

    return normalized_path_score[winner];
}

/**
 * @details For testing and troubleshooting for vistualizing all the current voting methods.
*/
void test_voting(std::vector<std::vector<double>> normalized_path_scores) {
    int winner;

    std::vector<double> range_results = range_voting(normalized_path_scores);
    std::cout << "range voting results:" << std::endl;
    display_vector(range_results);
    winner = findMin(range_results);
    std::cout << "Winner is path " << winner << std::endl;


    std::cout << "-----" << std::endl;


    std::cout << "combined voting result:" << std::endl;
    std::vector<double> combined_results = combined_aproval_voting(normalized_path_scores);
    display_vector(combined_results);
    winner = findMin(combined_results);
    std::cout << "Winner is path " << winner << std::endl;


    std::cout << "-----" << std::endl;


    std::cout << "condorcet voting results:" << std::endl;
    std::vector<double> condorcet_results = condorcet_voting(normalized_path_scores);
    display_vector(condorcet_results);
    winner = findMax(condorcet_results);
    std::cout << "Winner is path " << winner << std::endl;



    std::cout << "-----" << std::endl;


    std::cout << "borda voting results:" << std::endl;
    std::vector<double> borda_results = borda_voting(normalized_path_scores);
    display_vector(borda_results);
    winner = findMax(borda_results);
    std::cout << "Winner is path " << winner << std::endl;


    std::cout << "-----" << std::endl; 

    std::cout << "d scores:" << std::endl;
    std::vector<double> d_scores = d_score(normalized_path_scores);
    display_vector(d_scores);

    std::cout << "-----" << std::endl;
    
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

    bool operator<(const node& rhs) {
        return x ^ y < rhs.x ^ y;
    }


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
        if(state = 0){
            return sqrt( pow((origin.getX() - target.getX()), 2) + pow((origin.getY() - target.getY()), 2) );
        } else if(state = 1){
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

// std::list<node> TESTING_A_STAR(const node& start, const node& target, const SO_adjacency_matrx& graph, int& duration, double& score){

//     auto starTime = std::chrono::steady_clock::now();

//     /**
//      * BOOK KEEPING 
//     */
//     node_priority_queue openSet;
//     node start_copy = start;
//     start_copy.setWeight(euclidean_distance(start, target));
//     openSet.push(start_copy); //initially the openSet has the start node.


//     std::unordered_map<node, node, NodeHash> descendentList; //track the immediate predecesor of each node explored.

//     std::unordered_map<node, double, NodeHash> scoreList; //tracks the best found distance to each node explored
//     scoreList[start_copy] = 0;

//     std::unordered_map<node,bool, NodeHash> visitList; // marks all nodes whos children have been added to the openSet I don't relly care for the bool, it is just a look up table.

//     /**
//      * GRAPH SEARCHING SECTION
//     */
//     while(!openSet.empty()){

//         /**
//          * This version uses lazy deletion, so there is a chance that nodes can be in the openSet multiple times, so we must remove those that already have been explored (by explore we mean that we've added their children to the openSet)
//         */
//         while(visitList.find(openSet.top()) != visitList.end()){  //Since we are using lazy deletion, we ignore nodes that we have been explored / visisted / had their childrens added
//             openSet.pop();
//         }

//         node top = openSet.top();   //get the node with the shortest weight fron the openSet

//         if(top == target){
//             auto finishTime = std::chrono::steady_clock::now();
//             duration = std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - starTime).count();
//             //add top to the search region and backtrack via the desecneant list to find the path from the target back to the start.
//             std::list<node> path;   // using list since we are constructing the path backwards, we are always inserting in the front.
//             path.emplace_front(top);   
//             while(descendentList.find(top) != descendentList.end()){    //while there are still decendents (i.e. not reached the start, <- use that definition, its much more clearn)
//                 top = descendentList.at(top);
//                 path.emplace_front(top);
//             }
//             return path;

//         } else {
//             openSet.pop();
//             //iterate through all the neighbors of top, calculate their their tentative score which is the best distance to get to top and then the distance to get to the neighbor.
//             for(auto iter = graph.at(top).begin(); iter != graph.at(top).end(); iter++){
//                 node copy = iter->first;
//                 double tenativeScore = scoreList[top] + graph.at(top).at(copy); // the tenatives score is the distance from the start to node we popped, and its distance from this node to one of its neighbors
//                 if(scoreList.find(copy) == scoreList.end() || tenativeScore < scoreList.at(copy)){    //if this node has not been explored, or if it had, and the tenative distance is shorted then the previously discovered distance
//                     descendentList[copy] = top;     //update its decendent
//                     scoreList[copy] = tenativeScore;    //update its score
//                     //where a weight set would be applied.
//                     copy.setWeight(tenativeScore + euclidean_distance(copy, target));   //give this node its estimated score..
//                     openSet.push(copy);
//                 } else {
//                     //ignore
//                 }
//             }
//         }
//     }
//     //END OF MAIN LOOP

//     /**
//      * THIS SECTION IS ONLY REACHED IF THE openSet IS EMPTIED AND THE TARGET NODE WAS NEVER REACHED...
//     */

//    return {}; //return an empty list.

// }

/**
 * ===========================================
 * A* METHOD THAT GENERATES THE SHORTEST PATH WITH RESPECT TO ONLY ONE OBJECTIVE OF A MULTI OBJECTIVE ADJACENCY MATRIX 
 * @param origin, target are the start and end point for the search algorithm.
 * @param graph is a n objective adjacency matrix.
 * @param h is the hueristic functor that can be set in the driver code to use one of the supported heuristic functions (see HEURISTIC for more details).
 * @param objective is the objective {0, 1, ..., n-1} that A* will find the shortest path with respect to.
 * @return a vector of all of the objective cost 
 * @return {} if no path from the origin and target exist.
 * ===========================================
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
 * @details "breaking up" the multo objective graph is implemented by conducting the A* search with respect to a single objective, i.e. SINGLE_OBJECTIVE_A_STAR() 
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
 * TURNING DOA MAP INTO MATRIX
 * @param DOAmap is the file name of a map from the DOA director (or any map file that follows its ASCI encoding)
 * @details Reads the inpute file and constructs a multi-objectie graph with the following objectives:
 *              1. Euclidian distance,
 *              2. Uniform distance (1.5),
 *              3. Ranodmm (range between 0 - 20),
 *              4. Safety (average of the nodes degree, nodes with fewer edges (i.e. those near walls) are higher), and 
 *              5. Danger (nodes at randomy are selected to have expensive incoming and outcoming edges, non selected nodes have a uniform cost of 1.5).
 * @return a 5 objective adjacecny matrix based off the DOAmap.
 * ===========================================
*/
MO_adjacency_matrix DOA_MAP_TO_MO_ADJ_MATRIX(std::string DOAmap){
    auto begin = std::chrono::high_resolution_clock::now();

    std::ifstream ifs;
    DOAmap = "dao-map/" + DOAmap+ ".map";
    ifs.open(DOAmap, std::ifstream::in);

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
    std::vector<std::vector<char>> map (height, std::vector<char> (width));
    for(int i = 0; i < height; i++){
        std::getline(ifs,x);
        for(int j = 0; j < x.size(); j++){
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
 * 
*/
void VBMO(const node& origin, const node& target, const MO_adjacency_matrix& graph, int& duration) {

}

std::vector<double> generate_weight_set(const std::vector<double>& normalized_path_cost){
    // using complement for now, new version of weight sets are planned 
    std::vector<double> weight_set(normalized_path_cost.size());
    for(int i = 0; i < normalized_path_cost.size(); i++){
        weight_set[i] = 1 - normalized_path_cost[i];
    }

    return weight_set;
}

std::vector<double> randomized_weight_set(const std::vector<double>& normalized_path_cost){
    //in progress
}

std::vector<double> WEIGHTED_VBMO_A_STAR(){
    //in progress
}

int main() {
    //example
    auto begin = std::chrono::high_resolution_clock::now();

    MO_adjacency_matrix graph = DOA_MAP_TO_MO_ADJ_MATRIX("brc202d");

    std::mt19937 rand(std::chrono::high_resolution_clock::duration(std::chrono::high_resolution_clock::now() - begin).count()); 

    int m = graph.size();
    int a(rand() % m), b(rand() % m);

    node origin = std::next(graph.begin(), a)->first;
    node target = std::next(graph.begin(), b)->first;

    HEURISTIC h;

    int duration;
    std::vector<std::vector<double>> test = VBMO_A_STAR(origin, target, graph, h, duration);

    normalize_vector(test);
    display_matrix(test);

    std::vector<double> winner = VOTE(test, "range");
    std::cout << "time: " << duration << "ms" << std::endl;

    std::vector<double> weight_set = generate_weight_set(winner);

    std::cout << "WEIGHT SET GENERATED:" << std::endl;
    display_vector(weight_set);

    std::cout << "===============" << std::endl;

    std::vector<double> combined_results = COMBINED_MO_A_STAR(origin, target, graph, h, duration);

    display_vector(combined_results);

    std::cout << "time: " << duration << "ms" << std::endl;

    return 0;
}
