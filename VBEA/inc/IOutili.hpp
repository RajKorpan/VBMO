#ifndef IO_UTILI_
#define IO_UTILI_

#include <algorithm>
#include <cstdio>
#include <ctime>
#include <ios>
#include <unordered_map>
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

std::ostream& operator<<(std::ostream &stream, const Edge &edge);


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
                  child_method,       // either weighted combined or conscious
                  map_name,           // only for DAO?
                  source,
                  target;

  std::vector<std::vector<std::vector<double>>> fronts, 
                                                norm_fronts;

  std::vector<std::vector<double>>              raw_d_scores,
                                                norm_d_scores,

                                                winner,
                                                norm_winner; // keeps track generation
  std::vector<double>                           winner_raw_d_score,
                                                winner_norm_d_score;

  std::vector<int>                              run_times;

  // all other data members are written during run time
  log(const size_t source_, const size_t target_, const std::string map_name_)
  :source(std::to_string(source_)), target(std::to_string(target_)), map_name(map_name_) {}
};

void write_array(std::ostream &out_file, const std::vector<double> &vec);

void write_matrix(std::ostream &out_file, const std::vector<std::vector<double>> &matrix);

void write_record(std::ostream &out_file, const struct::log &log);

void write_all_records(const std::vector<struct::log> &logs, std::string file_name);

void write_all_records_alt(const std::vector<struct::log> &logs, std::string file_name, const size_t T, const size_t K, const std::string voting_method, const std::string child_method);

void load_road_instances(const std::string query_file, std::vector<std::vector<size_t>> &queries_out);

bool load_ascii_file(std::string ascii_file, std::vector<Edge> &out_edges, std::vector<NodePtr> &out_nodes, size_t &graph_size);

std::unordered_map<std::string, std::vector<std::vector<size_t>>>  load_asci_queries(const std::string asci_query_file);

void add_third_objective(const std::string MAP, std::vector<Edge> &edge_list,  std::vector<NodePtr> &node_list, bool save = false);

void add_delay_objective(const std::string MAP, std::vector<Edge> &edge_list,  std::vector<NodePtr> &node_list, bool save = false);

void write_ascii_inst(std::ostream &out_file, const std::vector<struct::log> &LOG, const std::string &MAP_NAME);
#endif


