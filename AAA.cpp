#include <iostream>
#include <unordered_map>
#include <queue>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <list>
#include <functional>
#include <chrono>


// NEW INCLUDES
#include <random>
#include <iterator>



// typedef std::unordered_map<node,std::unordered_map<node,std::vector<double>>> MO_adjacency_matrix;



/**
 * THIS METHOD IS FOR A A GRAPH WHERE NODES ARE REPRESENTED AS A (X,Y) CORDINATE PAIR...
*/



class node {
    private:
    int x,y;
    double weight;

    public:

    node(){
        x = 0;
        y = 0;
    }

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

    double hask(){
        return x ^ y;
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
    
    bool operator!=(const node& rhs) const {
        return x != rhs.x || y != rhs.y;
    }

    // double hash() const {
    //     return x ^ y;
    // };

    void setWeight(double W){
        weight = W;
    };

    double getWeight() const {
        return weight;
    };

    double euclidean_distance(const node& rhs) const {
        return sqrt( ( (this->x - rhs.x)^ 2) + ( (this->y - rhs.y)^ 2) );
    };

    void display() const {
        std::cout << "(" << x << ", " << y << ")";
    };

};

double euclidean_distance(const node& origin, const node& target) {
    return sqrt( pow((origin.getX() - target.getX()), 2) + pow((origin.getY() - target.getY()), 2) );
}


// Use this for when the order of nodes does not mater (e.g. unordered_map for look up tables)
struct NodeHash {

    size_t operator()(const node& key) const {
        return key.getX() ^ key.getY();
    }
};


// Use this when the order of the nodes based of their weight (distance from parent node and )
// struct PQNodeHash{
//     size_t operator()(const node& key) const {
//         return key.getWeight();
//     }
// };

struct PQNodeHash{
    size_t operator()(const node& a, const node& b){
        return a.getWeight() > b.getWeight();
    }
};

/**
 * TYPEDEF
*/
typedef std::unordered_map<node,std::unordered_map<node, std::vector<double>, NodeHash>, NodeHash> MO_adjacency_matrix;     //
typedef std::unordered_map<node, std::unordered_map<node, double, NodeHash>, NodeHash> SO_adjacency_matrix;                  //
typedef std::priority_queue<node, std::vector<node>, PQNodeHash> node_priority_queue;                                        //

/**
 * GENERATING A MO GRAPH (DISTANCE, UNIFORM, RANDOM, SAFETY)
 * NEED TO ADD ADD RANDOM based "DANGEROUS" 
 * MAYBE ADD PEPPER OF "DANGEROUS AREA"
 *  SAFETY = the average of the max degree minus the degree of each node on the edge's ends
*/

MO_adjacency_matrix DOA_MO_adj_matrix_generator(const std::vector<std::vector<char>>& map) {
    auto begin = std::chrono::high_resolution_clock::now();
    MO_adjacency_matrix AdjMatrix;  
    int height = map.size();
    int width = map[0].size();

    //random engine for 
    std::mt19937 RNG(std::chrono::high_resolution_clock::duration(std::chrono::high_resolution_clock::now() - begin).count()); 
    int temp;


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

    //addition "random" based objectives:

    //why not move? BC compiler optimizes this...?
    return AdjMatrix;
}

std::vector<std::vector<char>> DOA_map(std::string DOAmap){
    std::ifstream ifs;
    DOAmap = "dao-map/" + DOAmap+ ".map";
    ifs.open(DOAmap, std::ifstream::in);
    //ifs.open("testmap.map", std::ifstream::in);

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

    return map;
}

// double sum_path(std::list<node> path, const MO_adjacency_matrix& graph, int obejective){
//     int sum;
//     for(auto &i: path){
//         sum+= graph
//     }
// }

void display_vector(std::vector<double> vec) {
    if(vec.size() > 0) {
        std::cout << "{" << vec[0];
        for(int i = 1; i < vec.size(); i++) {
            std::cout << ", " << vec[i];
        }
        std::cout << "}" << std::endl;
    } 
    else { // the vecetor is empty
        std::cout << "{}" << std::endl;
    }
}

/**
 * FENCE POST ERROR DOES NOT INCLUDE THE DISTANCE NOR EDGE CONNECTING THE SHORTEST PATH TO THE TARGET
*/
std::list<node> SELECT_OBJECTIVE_A_STAR(node start, node target, const MO_adjacency_matrix& graph, const int objective, int& duration, double& cost) {
    auto startTime = std::chrono::steady_clock::now();

    node_priority_queue openSet;
    start.setWeight(euclidean_distance(start, target));
    openSet.push(start);

    std::unordered_map<node, node, NodeHash> descendentList;    //maintains the immediate decendent that provides the shortest path

    std::unordered_map<node, double, NodeHash> scoreList;   //maintains the best discoved distance discovered for each node
    scoreList[start] = 0;

    std::unordered_map<node, bool, NodeHash> visitList;     //nodes are added to this list once their neighbors have been added to the openSet

    while(!openSet.empty()){

        while(visitList.find(openSet.top()) != visitList.end()){
            openSet.pop();
        }
        node top = openSet.top();
        if(top == target){
            auto finishTime = std::chrono::steady_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(finishTime - startTime).count();
            double path_cost;
            std::list<node> path;
            path.emplace_front(top);
            // path_cost += graph.at(top).at(target)[objective];

            while(descendentList.find(top) != descendentList.end()){
                top = descendentList[top];
                path_cost += graph.at(top).at(path.front())[objective];
                path.emplace_front(top);
            }

            cost = path_cost; 

            return path;

        } else {
            openSet.pop();
            // iterate through all the neighbors of the top
            for(auto &iter: graph.at(top)){
                node copy = iter.first;
                double tentativeScore = scoreList[top] + graph.at(top).at(copy)[objective];
                // if the neightbor has not been seen (and thus does not have a score) or this path to the neighbor is better the one previous best, add/update the score and its decendent
                if(scoreList.find(copy) == scoreList.end() || tentativeScore < scoreList[copy]){ 
                    // if(tentativeScore < scoreList[copy]){
                    //     //do smt
                    // }
                    descendentList[copy] = top;
                    scoreList[copy] = tentativeScore;
                    copy.setWeight(tentativeScore + euclidean_distance(copy, target));
                    openSet.push(copy);
                }
            }
        }
    }   

    //This section is reach only if the empty set is empied and the target was never found
    return {};
}

std::vector<double> SINGLE_OBJECTIVE_A_STAR(node start, node target, const MO_adjacency_matrix& graph, const int objective, int& duration) {
    auto startTime = std::chrono::steady_clock::now();

    node_priority_queue openSet;
    start.setWeight(euclidean_distance(start, target));
    openSet.push(start);

    std::unordered_map<node, node, NodeHash> descendentList;    //maintains the immediate decendent that provides the shortest path

    std::unordered_map<node, double, NodeHash> scoreList;   //maintains the best discoved distance discovered for each node
    scoreList[start] = 0;

    std::unordered_map<node, bool, NodeHash> visitList;     //nodes are added to this list once their neighbors have been added to the openSet

    while(!openSet.empty()){

        while(visitList.find(openSet.top()) != visitList.end()){
            openSet.pop();
        }
        node top = openSet.top();
        if(top == target){
            //Calcaulate all the objective cost.
            int n = graph.at(top).at(descendentList[top]).size(); //fetching number of objectives

            std::vector<double> path_cost(n);

            node temp;
            temp = top;
            // path_cost += graph.at(top).at(target)[objective];

            while(descendentList.find(top) != descendentList.end()){
                top = descendentList[top];
                // path_cost += graph.at(top).at(temp)[objective];
                for(int i = 0; i < n; i++){
                    path_cost[i] += graph.at(top).at(temp)[i];
                }
                temp = top;
            }


            auto finishTime = std::chrono::steady_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(finishTime - startTime).count();
            return path_cost;
        } else {
            openSet.pop();
            // iterate through all the neighbors of the top
            for(auto &iter: graph.at(top)){
                node copy = iter.first;
                double tentativeScore = scoreList[top] + graph.at(top).at(copy)[objective];
                // if the neightbor has not been seen (and thus does not have a score) or this path to the neighbor is better the one previous best, add/update the score and its decendent
                if(scoreList.find(copy) == scoreList.end() || tentativeScore < scoreList[copy]){ 
                    // if(tentativeScore < scoreList[copy]){
                    //     //do smt
                    // }
                    descendentList[copy] = top;
                    scoreList[copy] = tentativeScore;
                    copy.setWeight(tentativeScore + euclidean_distance(copy, target));
                    openSet.push(copy);
                }
            }
        }
    }   

    //This section is reach only if the empty set is empied and the target was never found
    return {};
}

std::list<node> NAIVE_MO_A_STAR(node start, node target, const MO_adjacency_matrix& MO_adj_matrix, int& duration, double& cost){

    MO_adjacency_matrix single_MO_matrix;
    for(auto iter = MO_adj_matrix.begin(); iter != MO_adj_matrix.end(); iter++){
        for(auto jter = iter->second.begin(); jter != iter->second.end(); jter++){
            double sum = 0;
            for(auto kter = jter->second.begin(); kter != jter->second.end(); kter++){
                sum += *kter;
            }
            single_MO_matrix[iter->first][jter->first].emplace_back(sum);

        }
    }

    //calls the Select A* method 
    std::list<node> path = SELECT_OBJECTIVE_A_STAR(start, target, single_MO_matrix, 0, duration, cost);

    return path ;
}


void save_map(std::vector<std::vector<char>>& map) {
    std::ofstream results_file("./shortest_path.text");

    std::ostream_iterator<char> out_iterator(results_file);
    for(auto iter = map.begin(); iter != map.end(); iter++){
        std::copy(iter->begin(), iter->end(), out_iterator);
        results_file << "\n";
    }
}

std::vector<double> normalize_vector(std::vector<double>& vec){
    int magnitude;
    //magnitude formula
    for(int i = 0; i < vec.size(); i++){
        magnitude += vec[i] * vec[i];
    }

    magnitude = sqrt(magnitude);

    std::vector<double> normalized_vec(vec.size());

    for(int i = 0; i < vec.size(); i++){
        normalized_vec[i] = vec[i] / magnitude;
    }

    return normalized_vec;
}

std::vector<double> generate_weight_set(std::vector<double>& normalized_vector){
    return {};
}

/**
 * UPDATE TO ALSO
*/
std::vector<std::vector<double>> VBMO(node start, node target, const MO_adjacency_matrix& MO_adjacency_matrix, int& duration) {
    //get the number of objectives
    int b = MO_adjacency_matrix.begin()->second.begin()->second.size();

    std::vector<std::vector<double>> objective_costs(b);
    int d;

    for(int i = 0; i < b; i++){
        objective_costs[i] = SINGLE_OBJECTIVE_A_STAR(start, target, MO_adjacency_matrix, i, d);
    }

    return objective_costs;
}




int main(){
    auto begin = std::chrono::high_resolution_clock::now();

    //using dao-map\lak302d.map
    //
    std::vector<std::vector<char>> map = DOA_map("den203d");

    MO_adjacency_matrix graph = DOA_MO_adj_matrix_generator(map);

    std::mt19937 rand(std::chrono::high_resolution_clock::duration(std::chrono::high_resolution_clock::now() - begin).count()); 

    //random selection;
    int n = graph.size();
    int s(rand() % n), t(rand() % n);   // numbers that will determin which two nodes are the start and end node (from the list of nodes )

    //choosing random keys from the adjaceny matrix to ensure nodes exist
    node start = std::next(graph.begin(), s)->first;  
    node target = std::next(graph.begin(), t)->first;
    int duration;
    double path_cost;


    std::vector<std::vector<double>> test = VBMO(start, target, graph, duration);


    for(int i = 0; i < test.size(); i++){
        display_vector(test[i]);
    }



    // std::list<node> path = SELECT_OBJECTIVE_A_STAR(start, target, graph, 2, duration, path_cost);

    // std::list<node> path = NAIVE_MO_A_STAR(start, target, graph, duration, path_cost);


/**
     * VERIFIYING THE ASCII MAP WITH SHORTEST PATH HIGHLIGHTED 
     * @param: path, is the shortest path returned by A* 
     * @param: map, is the orignal ASCI map
    */
    // for(auto iter = path.begin(); iter != path.end(); iter++){
    //     map[iter->getX()][iter->getY()] = '$';
    // }

    // save_map(map);
    // for(int i = 0; i < height; i++) {
    //     for(int j = 0; j < width; j++) {
    //         std::cout << map[i][j];
    //     }
    //     std::cout << std::endl;
    // }


    // path_cost = SINGLE_OBJECTIVE_A_STAR(start, target, graph, 0, duration);
    // std::cout << "start point: "; start.display();  std::cout << " end point: "; target.display(); std::cout << std::endl;
    // // std::cout << "actual start: "; path.front().display(); std::cout << " end:"; path.back().display(); std::cout << std::endl;
    // std::cout << "Time: " << duration << "us" <<  std::endl;
    // std::cout << "Path Cost: " << path_cost << std::endl;

    // iter = NEW.begin();
    // for(int i = 0; i < 3; i++){
    //     iter->first.display(); std::cout << " "; iter->second.begin()->first.display(); std::cout << " costs: ";
    //     display_vector(iter->second.beg`=~in()->second);
    //     iter++;
    // }



    return 0;
}


/**
 * Add some number of random start and end nodes for the Road map..
 * WHEN USING CORDIANTE DATA< MAKE SURE TO USE HAVERSINE DISTANCE 
 * TRY AS MANY EVIRONMENT AS POSSIBLE
 * SAVE THAT CSV file for visualization in python
*/