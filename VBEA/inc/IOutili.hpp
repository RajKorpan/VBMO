#ifndef IO_UTILI_
#define IO_UTILI_

#include <algorithm>
#include <cstdio>
#include <ctime>
#include <ios>
#include <string>
#include <vector>
#include <random>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>

#include "graph.hpp"

//
// mis
//

struct pair_hash{
  template<class t1, class t2>
  const size_t operator()(const std::pair<t1, t2>& p) const {
    auto h1 = std::hash<t1>{}(p.first);
    auto h2 = std::hash<t2>{}(p.second);
    if(h1 != h2){
      return h1 ^ h2;
    } else {
      return h1;
    }
  }
};

std::ostream& operator<<(std::ostream &stream, const std::vector<double> &vec);

std::ostream& operator<<(std::ostream &stream, const std::vector<std::vector<double>> &vec);

std::ostream& operator<<(std::ostream &stream, const std::vector<size_t> &vec);

std::ostream& operator<<(std::ostream &stream, const std::vector<int> &vec);

std::ostream& operator<<(std::ostream &stream, const std::vector<std::string> &vec);

//
// IO functions for USA-road
//

void split_string(std::string string, std::string delimiter, std::vector<std::string> &results);

bool load_gr_files(std::vector<std::string> gr_files, std::vector<Edge> &edges_out, size_t &graph_size_out);

void get_nodes_ROAD(const std::string ASCII_FILE, std::vector<NodePtr> &out_nodes);

//
// IO function for ASCII maps
//

void get_nodes_ASCII(const std::string ASCII_FILE, std::vector<NodePtr> &out_nodes, AdjMatrix &adj_matrix);

//
// output
//
struct log{
  std::string     voting_method,      // range, borda, concorcet, etc..
                  file_name,          // the map file name
                  child_method,       // either weighted combined or conscious
                  source,
                  target;

  std::vector<std::vector<std::vector<double>>> fronts, 
                                                norm_fronts;

  std::vector<std::vector<double>>              raw_d_scores,
                                                norm_d_scores,
                                                winner,
                                                norm_winner; // keeps track generation

  std::vector<int>                              run_times;
  std::vector<double>                           winner_raw_d_score,
                                                winner_norm_d_score;
  std::vector<std::vector<size_t>>              trace; // series of nodes that was used in tracing the path

  // all other data members are written during run time
  log(const std::string file_name_, const size_t source_, const size_t target_, const std::string voting_method_, const std::string child_method_)
  :file_name(file_name_), fronts({}), source(std::to_string(source_)), voting_method(voting_method_), target(std::to_string(target_)), child_method(child_method_) {}
};

void write_array(std::ostream &out_file, const std::vector<double> &vec);

void write_matrix(std::ostream &out_file, const std::vector<std::vector<double>> &matrix);

void write_record(std::ostream &out_file, const struct::log &log);

void write_all_records(const std::vector<struct::log> &logs, std::string file_name);

void load_road_instances();

void load_ascii_instances();

void road_instnaces_runner();

void ascii_instances_runner();

void add_third_objective(const std::string MAP, std::vector<Edge> &edge_list,  std::vector<NodePtr> &node_list);

void add_delay_objective(const std::string MAP, std::vector<Edge> &edge_list, std::vector<NodePtr> &node_list);
#endif


