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
#include <random>
#include <iterator>

/**
 * 1. REMOVE DISJOINTED GRAPHS?
 * 2. ADD UNIFORM AND RANDOM[2,20] COST
 * 3. SAFTEY COST?
 * 4. NODE LIST (for easy)
*/



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
struct PQNodeHash{
    size_t operator()(const node& a, const node& b){
        return a.getWeight() > b.getWeight();
    }
};




/**
 * TYPEDEF
*/
typedef std::unordered_map<node, std::unordered_map<node,double, NodeHash> , NodeHash> SO_adjacency_matrx;
typedef std::priority_queue<node, std::vector<node>, PQNodeHash> nodePriority_queue; 


/**
 * A* FUNCTION FOR SINGLE OBJECTIVE
 * PASS THE HEURISTIC AS A FUNCTOR FOR TWO NODES? FOR FUTURE VERSIONS
*/

std::list<node> SO_A_STAR(const node& start, const node& target, const SO_adjacency_matrx& graph, int& duration){
    auto startTime = std::chrono::steady_clock::now();

    /**
     * BOOK KEEPING 
    */
    nodePriority_queue openSet;
    node start_copy = start;
    start_copy.setWeight(euclidean_distance(start, target));
    openSet.push(start_copy); //initially the openSet has the start node.


    std::unordered_map<node, node, NodeHash> descendentList; //track the immediate predecesor of each node explored.

    std::unordered_map<node, double, NodeHash> scoreList; //tracks the best found distance to each node explored
    scoreList[start_copy] = 0;

    std::unordered_map<node,bool, NodeHash> visitList; // marks all nodes whos children have been added to the openSet I don't relly care for the bool, it is just a look up table.

    /**
     * GRAPH SEARCHING SECTION
    */
    while(!openSet.empty()){

        /**
         * This version uses lazy deletion, so there is a chance that nodes can be in the openSet multiple times, so we must remove those that already have been explored (by explore we mean that we've added their children to the openSet)
        */
        while(visitList.find(openSet.top()) != visitList.end()){  //Since we are using lazy deletion, we ignore nodes that we have been explored / visisted / had their childrens added
            openSet.pop();
        }

        node top = openSet.top();   //get the node with the shortest weight fron the openSet

        if(top == target){
            auto finishTime = std::chrono::steady_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(finishTime - startTime).count();

            //add top to the search region and backtrack via the desecneant list to find the path from the target back to the start.
            std::list<node> path;   // using list since we are constructing the path backwards, we are always inserting in the front.
            path.emplace_front(top);   
            while(descendentList.find(top) != descendentList.end()){    //while there are still decendents (i.e. not reached the start, <- use that definition, its much more clearn)
                top = descendentList.at(top);
                path.emplace_front(top);
            }
            return path;

        } else {
            openSet.pop();
            //iterate through all the neighbors of top, calculate their their tentative score which is the best distance to get to top and then the distance to get to the neighbor.
            for(auto iter = graph.at(top).begin(); iter != graph.at(top).end(); iter++){
                node copy = iter->first;
                double tenativeScore = scoreList[top] + graph.at(top).at(copy); // the tenatives score is the distance from the start to node we popped, and its distance from this node to one of its neighbors
                if(scoreList.find(copy) == scoreList.end() || tenativeScore < scoreList.at(copy)){    //if this node has not been explored, or if it had, and the tenative distance is shorted then the previously discovered distance
                    descendentList[copy] = top;     //update its decendent
                    scoreList[copy] = tenativeScore;    //update its score
                    //where a weight set would be applied.
                    copy.setWeight(tenativeScore + euclidean_distance(copy, target));   //give this node its estimated score..
                    openSet.push(copy);
                } else {
                    //ignore
                }
            }
        }
    }
    //END OF MAIN LOOP

    /**
     * THIS SECTION IS ONLY REACHED IF THE openSet IS EMPTIED AND THE TARGET NODE WAS NEVER REACHED...
    */

   return {}; //return an empty list.

}

////////////////////////////////////////////////////////////////////////

/**
 * SAVING RESULTS
*/

void save_map(std::vector<std::vector<char>>& map) {
    std::ofstream results_file("./shortest_path.text");

    std::ostream_iterator<char> out_iterator(results_file);
    for(auto iter = map.begin(); iter != map.end(); iter++){
        std::copy(iter->begin(), iter->end(), out_iterator);
        results_file << "\n";
    }
}
//make a void version that you pass the open list to add the edges too.


int main() {
    std::chrono::high_resolution_clock();
    auto begin = std::chrono::high_resolution_clock::now();

    /**
     * FETCHING THE MAP FILE
    */
    std::ifstream ifs;
    ifs.open("dao-map/brc101d.map", std::ifstream::in);
    //ifs.open("testmap.map", std::ifstream::in);

    std::string x;
    std::getline(ifs,x);// remove "type ..."

    std::getline(ifs,x);    //getting the height
    int height = stoi(x.substr(7));

    std::getline(ifs,x);    //getting the width 
    int width = stoi(x.substr(6));

    std::getline(ifs,x); // remove "map"


    /**
     * TURNING THE TEXT FILE INTO A GRID/MATRIX
    */
    std::vector<std::vector<char>> map (height, std::vector<char> (width));
    for(int i = 0; i < height; i++){
        std::getline(ifs,x);
        for(int j = 0; j < x.size(); j++){
            map[i][j] = x[j];
        }
    }

    /**
     * VERIFIYING THE MAP 
    */
    // for(int i = 0; i < height; i++) {
    //     for(int j = 0; j < width; j++) {
    //         std::cout << map[i][j];
    //     }
    //     std::cout << std::endl;
    // }

    SO_adjacency_matrx AdjMatrix;  

    bool first = true;

    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){

            if(map[i][j] == '.'){

                // create node and add its to the matrix 
                node cur(i,j);
                
                AdjMatrix[cur];
                //up
                if(i+1 < height && map[i+1][j] == '.'){
                    AdjMatrix[cur][node(i+1,j)] = 1.0;
                }
                //down
                if(i-1 >= 0 && map[i-1][j] == '.') {
                    AdjMatrix[cur][node(i-1,j)] = 1;
                }

                //right `
                if(j+1 < width && map[i][j+1] == '.'){
                    AdjMatrix[cur][node(i,j+1)] = 1;
                }
                //left
                if(j-1 >= 0 && map[i][j-1] == '.'){
                    AdjMatrix[cur][node(i,j-1)] = 1;
                }

                //down right
                if(i+1 < height && j+1 < width && map[i+1][j+1] == '.'){
                    AdjMatrix[cur][node(i+1,j+1)] = 1.414;
                }
                //up right
                if(i-1 >= 0 && j+1 < width && map[i-1][j+1] == '.'){
                    AdjMatrix[cur][node(i-1,j+1)] = 1.414;
                }
                //down left
                if(i+1 < height && j-1 >= 0 && map[i+1][j-1] == '.') {
                    AdjMatrix[cur][node(i+1,j-1)] = 1.414;
                }
                //up left
                if(i-1 >= 0 && j-1 >= 0 && map[i-1][j-1] == '.') {
                    AdjMatrix[cur][node(i-1,j-1)] = 1.414;
                }
            }
        }
    }

    
    int duration;
    // Random number machine with the time beign the random seed.
    std::mt19937 rand(std::chrono::high_resolution_clock::duration(std::chrono::high_resolution_clock::now() - begin).count()); 

    //random selection;
    int n = AdjMatrix.size();
    int s(rand() % n), t(rand() % n);   // numbers that will determin which two nodes are the start and end node (from the list of nodes )

    //choosing random keys from the adjaceny matrix to ensure nodes exist
    node start = std::next(AdjMatrix.begin(), s)->first;
    node target = std::next(AdjMatrix.begin(), t)->first;



    // manual selection
    // for a DOA map, the x cordinate is: <file line number> - 5;
    //                    y cordinate is: zero indexed.               

    //random 

    auto path = SO_A_STAR(start, target, AdjMatrix, duration);

    /**
     * VERIFIYING THE ASCII MAP WITH SHORTEST PATH HIGHLIGHTED 
     * @param: path, is the shortest path returned by A* 
     * @param: map, is the orignal ASCI map
    */
    for(auto iter = path.begin(); iter != path.end(); iter++){
        map[iter->getX()][iter->getY()] = '$';
    }

    save_map(map);
    // for(int i = 0; i < height; i++) {
    //     for(int j = 0; j < width; j++) {
    //         std::cout << map[i][j];
    //     }
    //     std::cout << std::endl;
    // }


    std::cout << "start point: "; start.display();  std::cout << " end point: "; target.display(); std::cout << std::endl;
    std::cout << "Time: " << duration << "us" <<  std::endl;
 
    
    
    return 0;
}