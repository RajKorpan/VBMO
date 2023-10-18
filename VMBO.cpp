// This is an implemenation of the VMBO algorithm
#include <vector>
#include <iostream>
#include <cmath>

/**
 *          TO DO 
 * 0. Change return type to be tuples to contain the best path, the best score, etc.
 * 1. Optimize voting methods?
 * 2. Create new voting methods
 * 3. Ensemble/tornements method
 * 4. find or craate A* algorith
 * 5. create combinedatory A* method
 * 6. commit to git?
*/



/**
*           VOTING METHODS
* @param normalized_path_costs is a n x n matrix (where n is the number of obejective) contaning the objective cost of the n paths created by VMBO 
* @return a 1 x n matrix containing the resu such that [0] is the score of the path 1 depending on the voting scheme up til [n-1] that has the sum of paths n.
*/

/*
    std::vector<std::vector<float>> normalize_scores(std::vector<std::vector<float>> path_costs) {}
*/   
    void display_vector(std::vector<float> normalized_path_cost) {
        std::cout << "{" << normalized_path_cost[0];
        for(int i = 1; i < normalized_path_cost.size(); i++) {
            std::cout << ", " << normalized_path_cost[i];
        }
        std::cout << "}" << std::endl;
    }

    void display_all_path_objective_costs(std::vector<std::vector<float>> normalized_path_costs) {
           std::cout << "-------" << std::endl;
        for(int i = 0; i < normalized_path_costs.size(); i++) {
            std::cout << "P_" << i << ": "; display_vector(normalized_path_costs[i]); std::cout << std::endl;
            std::cout << "-------" << std::endl;
        }
    }

    float add_scores(std::vector<float> normalized_path_cost) {
        float total = 0;
        std::vector<float>::iterator front;
        for(front = normalized_path_cost.begin(); front != normalized_path_cost.end(); front++){
            total += *front;
        }
        return total;
    }

    std::vector<float> range_voting(std::vector<std::vector<float>> normalized_path_costs) {
        std::vector<float> sum_of_paths = {};
        for(int i = 0; i < normalized_path_costs.size(); i++) {
            sum_of_paths.emplace_back(add_scores(normalized_path_costs[i]));
        }
        return sum_of_paths;
    }

    // a rank function that works on columns
//    std::vector<float> borda_voting(std::vector<std::vector<float>> normalized_path_costs) {
//        int max_rank =  normalized_path_costs.size();
//    }

    std::vector<float> combined_aproval_voting(std::vector<std::vector<float>> normalized_path_costs) {
        // result vector will have the total sum 
        std::vector<float> combined_approval_sum = {};
        for(int i = 0; i < normalized_path_costs.size(); i++) {
            float path_combined_score = 0;
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

    std::vector<float> d_score(std::vector<std::vector<float>> normalized_path_costs) {
        std::vector<float> d_score = {};
        for(int i =0; i < normalized_path_costs.size(); i++) {
            float path_squared_sum = 0;
            for(int j = 0; j < normalized_path_costs[i].size(); j++){
                path_squared_sum += std::pow(normalized_path_costs[i][j],2);
            }
            d_score.emplace_back(std::sqrt(path_squared_sum));
        }

        return d_score;
    }

/*
    std::vector<float> condorent_voting(std::vector<std::vector<float>> normalized_path_costs) {

    }
*/


    void test_voting(std::vector<std::vector<float>> normalized_path_scores) {
        std::vector<float> test_set_sum = range_voting(normalized_path_scores);
        std::cout << "range voting results:" << std::endl;
        display_vector(test_set_sum);


        std::cout << "-----" << std::endl;


        std::cout << "combined voting result" << std::endl;;
        std::vector<float> test_combined_sum = combined_aproval_voting(normalized_path_scores);
        display_vector(test_combined_sum);


        std::cout << "-----" << std::endl;


    }

    int main() {
        std::vector<std::vector<float>> test_set = {{0.12, 0.89, 0.56}, {0.69, 0.02, 0.92}, {1, 0.48, 0.17}};
        test_voting(test_set);
        std::vector<float> test_set_d_scores = d_score(test_set);
        display_vector(test_set_d_scores);
        return 0;
    }

