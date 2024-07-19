#include <iostream>
#include <set>
#include <ostream>
#include <string>
#include "Utils/Definitions.h"

bool is_bounded(NodePtr apex, NodePtr node){
  for (int i = 0; i < apex->f.size(); i ++ ){
    if (node->f[i] > apex->f[i]){
      return false;
    }
  }
  return true;
}


AdjacencyMatrix::AdjacencyMatrix(size_t graph_size, std::vector<Edge> &edges, bool inverse)
    : matrix((graph_size+1), std::vector<Edge>()), graph_size(graph_size) {

  num_of_objectives = edges[0].cost.size();

    for (auto iter = edges.begin(); iter != edges.end(); ++iter) {
        if (inverse) {
            this->add(iter->inverse());
        } else {
            this->add((*iter));
        }
    }
}



size_t AdjacencyMatrix::get_num_of_objectives() const {
  return num_of_objectives;
}

void AdjacencyMatrix::add(Edge edge) {
    (this->matrix[edge.source]).push_back(edge);
}


size_t AdjacencyMatrix::size() const {return this->graph_size;}


const std::vector<Edge>& AdjacencyMatrix::operator[](size_t vertex_id) const {
    return this->matrix.at(vertex_id);
}


std::ostream& operator<<(std::ostream &stream, const AdjacencyMatrix &adj_matrix) {
    size_t  i = 0;

    stream << "{\n";
    for (auto vertex_iter = adj_matrix.matrix.begin(); vertex_iter != adj_matrix.matrix.end(); ++vertex_iter) {
        stream << "\t\"" << i++ << "\": [";

        std::vector<Edge> edges = *vertex_iter;
        for (auto edge_iter = edges.begin(); edge_iter != edges.end(); ++edge_iter) {
            stream << "\"" << edge_iter->source << "->" << edge_iter->target << "\", ";
        }

        stream << "],\n";
    }
    stream << "}";
    return stream;
}

std::ostream& operator<<(std::ostream &stream, const Edge &edge) {
    // Printed in JSON format
    stream
        << "{"
        <<  "\"edge_source\": " << edge.source << ", "
        <<  "\"edge_target\": " << edge.target << ", "
        <<  "\"edge_cost\": ";

    for (auto c: edge.cost){
      stream << c << ", ";
    }
    stream << "}";

    return stream;
}


bool Node::more_than_specific_heurisitic_cost::operator()(const NodePtr &a, const NodePtr &b) const {
    return (a->h[cost_idx] > b->h[cost_idx]);
}

bool Node::more_than_combined_heurisitic::operator()(const NodePtr &a, const NodePtr &b) const {
  return (a->g[0] + this->factor * a->g[1] > b->g[0] + this->factor * b->g[1]);
}


bool Node::more_than_full_cost::operator()(const NodePtr &a, const NodePtr &b) const {
  for (int i = 0; i + 1 < a->f.size(); i++){
    if (a->f[i] != b->f[i]) {
      return (a->f[i] > b->f[i]);
    }
  }
  return (a->f.back() > b->f.back());
}

bool Node::more_than_lex::operator()(const NodePtr &a, const NodePtr &b) const {
  if (order == Node::LEX_ORDER::LEX0){
    if (a->f[0] != b->f[0]) {
      return (a->f[0] > b->f[0]);
    } else {
      return (a->f[1] > b->f[1]);
    }
  }else{
      if (a->f[1] != b->f[1]) {
        return (a->f[1] > b->f[1]);
    } else {
        return (a->f[0] > b->f[0]);
    }
  }
}


std::ostream& operator <<(std::ostream &stream, const std::vector<size_t> &vec){
  stream << "[";
  for (size_t i = 0 ;  i < vec.size(); i ++){
    stream << vec[i];
    if (i + 1 <vec.size()){
      stream << ", ";
    }
  }
  stream << "]";
  return stream;
}

std::ostream& operator <<(std::ostream &stream, const std::vector<double> &vec){
    stream << "[";
    for (size_t i = 0 ;  i < vec.size(); i ++){
        stream << vec[i];
        if (i + 1 <vec.size()){
            stream << ", ";
        }
    }
    stream << "]";
    return stream;
}

std::ostream& operator<<(std::ostream &stream, const Node &node) {
    // Printed in JSON format
    std::string parent_id = node.parent == nullptr ? "-1" : std::to_string(node.parent->id);
    stream
        << "{"
        <<      "\"id\": " << node.id << ", "
        <<      "\"parent\": " << parent_id << ", "
        <<      "\"cost_until_now\": " << node.g << ", "
        <<      "\"heuristic_cost\": " << node.h << ", "
        <<      "\"full_cost\": " << node.f
        << "}";
    return stream;
}

bool PathPair::update_nodes_by_merge_if_bounded(const PathPairPtr &other, const Pair<double> eps) {
    // Returns true on sucessful merge and false if it failure
    if (this->id != other->id) {
        return false;
    }

    NodePtr new_top_left;
    NodePtr new_bottom_right;

    // Pick node with min cost1 (min cost2 if equal)
    if ((this->top_left->f[0] < other->top_left->f[0]) ||
        ((this->top_left->f[0] == other->top_left->f[0]) && (this->top_left->f[1] < other->top_left->f[1]))) {
        new_top_left = this->top_left;
    } else {
        new_top_left = other->top_left;
    }

    // Pick node with min cost2 (min cost1 if equal)
    if ((this->bottom_right->f[1] < other->bottom_right->f[1]) ||
        ((this->bottom_right->f[1] == other->bottom_right->f[1]) && (this->bottom_right->f[0] < other->bottom_right->f[0]))) {
        new_bottom_right = this->bottom_right;
    } else {
        new_bottom_right = other->bottom_right;
    }

    // Check if path pair is bounded after merge - if not the merge is illegal
    if ((((1+eps[0])*new_top_left->g[0]) < new_bottom_right->g[0]) ||
        (((1+eps[1])*new_bottom_right->g[1]) < new_top_left->g[1])) {
        return false;
    }

    this->top_left = new_top_left;
    this->bottom_right = new_bottom_right;
    return true;
}


bool PathPair::update_nodes_by_merge_if_bounded_keep_track(const PathPairPtr &other, const Pair<double> eps, std::list<NodePtr> & pruned_list) {
    // Returns true on sucessful merge and false if it failure
    if (this->id != other->id) {
        return false;
    }

    NodePtr new_top_left;
    NodePtr new_bottom_right;
    NodePtr pruned_top_left;
    NodePtr pruned_bottom_right;

    // Pick node with min cost1 (min cost2 if equal)
    if ((this->top_left->f[0] < other->top_left->f[0]) ||
        ((this->top_left->f[0] == other->top_left->f[0]) && (this->top_left->f[1] < other->top_left->f[1]))) {
        new_top_left = this->top_left;
        pruned_top_left = other->top_left;
    } else {
        new_top_left = other->top_left;
        pruned_top_left = this->top_left;
    }

    // Pick node with min cost2 (min cost1 if equal)
    if ((this->bottom_right->f[1] < other->bottom_right->f[1]) ||
        ((this->bottom_right->f[1] == other->bottom_right->f[1]) && (this->bottom_right->f[0] < other->bottom_right->f[0]))) {
        new_bottom_right = this->bottom_right;
        pruned_bottom_right = other->bottom_right;
    } else {
        new_bottom_right = other->bottom_right;
        pruned_bottom_right = this->bottom_right;
    }

    // Check if path pair is bounded after merge - if not the merge is illegal
    if ((((1+eps[0])*new_top_left->g[0]) < new_bottom_right->g[0]) ||
        (((1+eps[1])*new_bottom_right->g[1]) < new_top_left->g[1])) {
        return false;
    }

    this->top_left = new_top_left;
    this->bottom_right = new_bottom_right;

    // move_pruned_nodes(this, other.get());
    if (!is_bounded(new_top_left, pruned_top_left) && !is_bounded(new_bottom_right, pruned_top_left)){
      pruned_list.push_back(pruned_top_left);
    }

    if (!is_bounded(new_top_left, pruned_bottom_right) && !is_bounded(new_bottom_right, pruned_bottom_right)){
      pruned_list.push_back(pruned_bottom_right);
    }

    return true;
}

bool PathPair::if_merge_bounded(const PathPairPtr &other, const Pair<double> eps)  const {
    // Returns true on sucessful merge and false if it failure
    if (this->id != other->id) {
        return false;
    }

    NodePtr new_top_left;
    NodePtr new_bottom_right;

    // Pick node with min cost1 (min cost2 if equal)
    if ((this->top_left->f[0] < other->top_left->f[0]) ||
        ((this->top_left->f[0] == other->top_left->f[0]) && (this->top_left->f[1] < other->top_left->f[1]))) {
        new_top_left = this->top_left;
    } else {
        new_top_left = other->top_left;
    }

    // Pick node with min cost2 (min cost1 if equal)
    if ((this->bottom_right->f[1] < other->bottom_right->f[1]) ||
        ((this->bottom_right->f[1] == other->bottom_right->f[1]) && (this->bottom_right->f[0] < other->bottom_right->f[0]))) {
        new_bottom_right = this->bottom_right;
    } else {
        new_bottom_right = other->bottom_right;
    }

    // Check if path pair is bounded after merge - if not the merge is illegal
    if ((((1+eps[0])*new_top_left->g[0]) < new_bottom_right->g[0]) ||
        (((1+eps[1])*new_bottom_right->g[1]) < new_top_left->g[1])) {
        return false;
    }

    return true;
}


bool PathPair::update_nodes_by_merge_if_bounded2(const PathPairPtr &other, const Pair<double> eps) {
    // Returns true on sucessful merge and false if it failure
    if (this->id != other->id) {
        return false;
    }

    NodePtr new_top_left;
    NodePtr new_bottom_right;

    // Pick node with min cost1 (min cost2 if equal)
    if ((this->top_left->f[0] < other->top_left->f[0]) ||
        ((this->top_left->f[0] == other->top_left->f[0]) && (this->top_left->f[1] < other->top_left->f[1]))) {
        new_top_left = this->top_left;
    } else {
        new_top_left = other->top_left;
    }

    // Pick node with min cost2 (min cost1 if equal)
    if ((this->bottom_right->f[1] < other->bottom_right->f[1]) ||
        ((this->bottom_right->f[1] == other->bottom_right->f[1]) && (this->bottom_right->f[0] < other->bottom_right->f[0]))) {
        new_bottom_right = this->bottom_right;
    } else {
        new_bottom_right = other->bottom_right;
    }

    // Check if path pair is bounded after merge - if not the merge is illegal
    if ((1+eps[0])*new_top_left->f[0] < new_bottom_right->f[0]){
        return false;
    }

    this->top_left = new_top_left;
    this->bottom_right = new_bottom_right;
    return true;
}


bool PathPair::more_than_full_cost::operator()(const PathPairPtr &a, const PathPairPtr &b) const {
    if (a->top_left->f[0] != b->top_left->f[0]) {
        return (a->top_left->f[0] > b->top_left->f[0]);
    } else {
        return (a->bottom_right->f[1] > b->bottom_right->f[1]);
    }
}

std::ostream& operator<<(std::ostream &stream, const PathPair &pp) {
    // Printed in JSON format
    stream << "{" << pp.top_left << ", " << pp.bottom_right << "}";
    return stream;
}

Interval::Interval(const NodePtr top_left, const NodePtr bottom_right, std::shared_ptr<std::list<NodePtr>> to_expand): top_left(top_left), bottom_right(bottom_right), to_expand(to_expand){
  eps = 0;
  //this->to_expand.reserve(to_expand.size());
  for (auto& node: *to_expand){
    eps = std::max(eps, std::min( ((double)top_left->f[1]) / node->f[1] - 1, ((double)bottom_right->f[0])/node->f[0] - 1  ));
  }
}

std::ostream& operator<<(std::ostream& os, const Interval& interval){
  os << "Top left: " << *interval.top_left  << ", Bottom right: " << *interval.bottom_right << ", #nodes: " << interval.to_expand->size();
  return os;
}

double sparsity_metric(const std::vector<std::vector<double>> &front_approximation){
  int m = front_approximation[0].size();      // m = number of objectives in the environment
  int n = front_approximation.size();         // n = number of solutions
  double sparsity = 0;

  for(int j = 0; j < m; j++){ // get the cost of a objective j for all paths
    std::vector<double> p_j;
    for(int i = 0; i < n; i++){
      p_j.push_back(front_approximation[i][j]);
    }

    std::sort(p_j.begin(), p_j.end()); //sparsity metric requires that we sort the objectve costs

    for(auto i = 0; i < n-1; i++){     // squared difference of the sorted objective cost
      sparsity += pow(p_j[i] - p_j[i+1], 2);
    }
  }


  sparsity = (1.0 / (n - 1)) * sparsity;  
  if(std::isnan(sparsity)){
    return 0;
  } else {
    return sparsity;
  }
  
}


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
