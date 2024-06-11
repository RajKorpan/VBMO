#include <algorithm>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <ios>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>
#include <unordered_set>
#include <fstream>
#include <iostream>
#include <limits>
#include <functional>
#include <memory>
#include <cmath>

#include "graph.hpp"
#include "utili.hpp"



/**
 * Edge (in .hpp)
*/

// override for edge operator that prints it in JSON format
std::ostream& operator<<(std::ostream &stream, const Edge &edge) {
  stream
     << "{"
     <<  "\"edge_source\": " << edge.source << ", "
     <<  "\"edge_target\": " << edge.target << ", "
     <<  "\"edge_cost\": ";

  stream << "{" << edge.cost[0];
  for(int i = 1; i < edge.cost.size(); i++){
    stream << ", " << edge.cost[i];   
  }
  stream << "}";

  return stream;
}

/**
 *  Node (in .hpp)
 */



/**
 *  Adjacency Matrix 
*/

AdjMatrix::AdjMatrix(size_t graph_size_, std::vector<Edge> &edges, bool inverse=false)
  : matrix((graph_size_ + 1), std::vector<Edge>()), graph_size(graph_size_) { // graph_size + 1 is because vertices id's start at 1.
    obj_count = edges[0].cost.size();

    for(auto iter = edges.begin(); iter != edges.end(); ++iter){ // turn the edge list into an adacency list
      if(inverse) {
        this->add(iter->inverse());
      } else {
        this->add(*iter);
      }
    }      
  }

void AdjMatrix::add(Edge e){
  (this->matrix[e.source]).push_back(e);
}

size_t AdjMatrix::size(void) const {
  return this->graph_size;
}

size_t AdjMatrix::get_obj_count() const{
    return obj_count;
}

const std::vector<Edge>& AdjMatrix::operator[](size_t vertex_id) const{
  return this->matrix.at(vertex_id);
}

std::ostream& operator<<(std::ostream &stream, const AdjMatrix &adj_matrix){

}


  
 
