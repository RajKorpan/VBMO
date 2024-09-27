#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <time.h>
#include <random>
#include <iterator>
#include <ostream>
#include <fstream>
#include <filesystem>
#include <fstream>
#include <chrono>

#include "ShortestPathHeuristic.h"
#include "Utils/Definitions.h"
#include "Utils/IOUtils.h"
#include "Utils/Logger.h"
#include "BOAStar.h"
#include "PPA.h"
#include "SingleCriteria.h"
#include "ApexSearch.h"
#include "NAMOA.h"

#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>
#include <unordered_map>

struct log{
    size_t          source,
                    target;

    int             time;   
    double          sparsity;
    
    std::vector<std::vector<double>>    front,
                                        norm_front;

    std::vector<double>                 d_score, 
                                        norm_d_score;

    log() = default;

    log(size_t source_, size_t target_)
    : source(source_), target(target_) {}

};


void write_array(std::ostream &out_file, const std::vector<double> &vec){
   if(vec.empty()){
     out_file << "[]";
   } else {
   
     out_file << "[" << vec[0];

     for(int i = 1; i < vec.size(); i++){
       out_file << ", " << vec[i];
     }

     out_file << "]";
   }
   //  std::cout << vec.size() << std::endl;
   // if(vec.empty()){
   //   std::cout << "[]";
   // } else {
   
   //   std::cout << "[" << vec[0];

   //   for(int i = 1; i < vec.size(); i++){
   //     std::cout << ", " << vec[i];
   //   }

   //   std::cout << "]";
   // }

}

void write_matrix(std::ostream &out_file, const std::vector<std::vector<double>> &matrix){
  if(matrix.empty()){
    out_file << "[[]]";
  } else {
    out_file << "["; write_array(out_file, matrix[0]);
    for(int i = 1; i < matrix.size(); i++){
      out_file << ", "; write_array(out_file, matrix[i]);
    }

    out_file << "]";
  }
//
  // std::cout << matrix.size() << std::endl;
  // if(matrix.empty()){
  //   std::cout <<  "[[]]";
  // } else {
  //   std::cout <<  "["; write_array(out_file, matrix[0]);
  //   for(int i = 1; i < matrix.size(); i++){
  //     std::cout <<  ", "; write_array(out_file, matrix[i]);
  //   }

  //   std::cout <<  "]";
  // }

}

void write_record(std::ostream &out_file, const struct::log &LOG){
    out_file    << "{"
                << "\"source\": " << LOG.source << ", "
                << "\"target\": " << LOG.target << ", "
                << "\"front\":"; write_matrix(out_file, LOG.front); 
    out_file    << ", ";
    out_file    << "\"d-score\": "; write_array(out_file, LOG.d_score); 
    out_file    << ", ";
    out_file    << "\"norm front\": "; write_matrix(out_file, LOG.norm_front); 
    out_file    << ", ";
    out_file    << "\"norm d-score\": "; write_array(out_file, LOG.norm_d_score);
    out_file    << ", ";
    out_file    << "\"sparsity\": " << LOG.sparsity << ", "
                << "\"time\": " << LOG.time;
    

    out_file << "}";
//
    // std::cout <<  "{"
    //             << "\"source\": " << LOG.source << ", "
    //             << "\"target\": " << LOG.target << ", "
    //             << "\"front\":"; write_matrix(out_file, LOG.front); 
    // std::cout <<  ", ";
    // std::cout <<  "\"d-score\": "; write_array(out_file, LOG.d_score); 
    // std::cout <<  ", ";
    // std::cout <<  "\"norm front\": "; write_matrix(out_file, LOG.norm_front); 
    // std::cout <<  ", ";
    // std::cout <<  "\"norm d-score\": "; write_array(out_file, LOG.norm_d_score);
    // std::cout <<  ", ";
    // std::cout <<  "\"sparsity\": " << LOG.sparsity << ", "
    //             << "\"time\": " << LOG.time;
    

    // out_file << "}";

}

void write_ascii_log(std::ostream &out_file, const struct::log &LOG, const std::string MAP_NAME){
    
    out_file    << "{"
                << "\"map-name\": " << "\"" << MAP_NAME << "\"" << ", "
                << "\"source\": " << LOG.source << ", "
                << "\"target\": " << LOG.target << ", "
                << "\"front\":"; write_matrix(out_file, LOG.front); 
    out_file    << ", ";
    out_file    << "\"d-score\": "; write_array(out_file, LOG.d_score); 
    out_file    << ", ";
    out_file    << "\"norm front\": "; write_matrix(out_file, LOG.norm_front); 
    out_file    << ", ";
    out_file    << "\"norm d-score\": "; write_array(out_file, LOG.norm_d_score);
    out_file    << ", ";
    out_file    << "\"sparsity\": " << LOG.sparsity << ", "
                << "\"time\": " << LOG.time;
        out_file << "}";

}

void write_ascii_inst(std::ostream &out_file, const std::vector<struct::log> &LOG, const std::string &MAP_NAME){

    write_ascii_log(out_file, LOG[0], MAP_NAME);
    for(int i = 1; i < LOG.size(); i++){
        out_file << ", ";
        write_ascii_log(out_file, LOG[i], MAP_NAME);
    }
    
}



void write_logs(std::string MAP_NAME, const std::string MERGE_METHOD, const double EPSILON, const std::vector<struct::log> &LOGS){

    std::ofstream out_file(std::to_string(EPSILON) + "-" + MAP_NAME + "-" + MERGE_METHOD + ".json");

    std::cout << "writting data...";

    out_file << "{"
             << "\"map\": " << "\"" << MAP_NAME << "\"" <<  ", "
             << "\"merge strategy\": " << "\"" << MERGE_METHOD << "\"" << ", "
             << "\"eps\": " << EPSILON << ", ";
    
    out_file << "\"data\": [";
    write_record(out_file, LOGS[0]);
    for(int i = 1; i < LOGS.size(); i++){
        out_file << ", ";
        write_record(out_file, LOGS[i]);
    }
    out_file << "]";

    out_file << "}";
  
}

// ASCII
void write_logs(const std::string MERGE_METHOD, const double EPSILON, const std::vector<std::vector<struct::log>> &LOGS, const std::vector<std::string> &MAP_NAMES){
    
    std::ofstream out_file(std::to_string(EPSILON) + "-" + MERGE_METHOD + ".json");

    std::cout << "writting data...";


    out_file << "{"
             << "\"merge strategy\": " << "\"" << MERGE_METHOD << "\"" << ", "
             << "\"eps\": " <<  EPSILON << ", "
             << "\"data\": [";
    write_ascii_inst(out_file, LOGS[0], MAP_NAMES[0]);
    for(int i = 0; i < MAP_NAMES.size(); i++){
        out_file << ", ";
        write_ascii_inst(out_file, LOGS[i], MAP_NAMES[i]);
    }

    out_file << "]}";

    std::cout << " Done!" << std::endl;
  
}




// END NEW

using namespace std;

const std::string resource_path = "resources/";
const std::string output_path = "output/";
const MergeStrategy DEFAULT_MERGE_STRATEGY = MergeStrategy::SMALLER_G2;
std::string alg_variant = "";

void single_run_map(size_t graph_size, AdjacencyMatrix& graph, AdjacencyMatrix&inv_graph, size_t source, size_t target, std::ofstream& output, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, unsigned int time_limit) {
    std::cout << "Start Computing Heuristic" << std::endl;
    ShortestPathHeuristic sp_heuristic(target, graph_size, inv_graph);
    // sp_heuristic.set_all_to_zero();
    std::cout << "Finish Computing Heuristic\n" << std::endl;

    using std::placeholders::_1;
    Heuristic heuristic = std::bind( &ShortestPathHeuristic::operator(), sp_heuristic, _1);

    SolutionSet solutions;
    int num_exp, num_gen;
    auto runtime = std::clock();
    
    std::unique_ptr<AbstractSolver> solver;
    if (algorithm == "PPA"){
        Pair<double> eps_pair({eps, eps});
        solver = std::make_unique<PPA>(graph, eps_pair, logger);
    }else if (algorithm == "BOA"){
        Pair<double> eps_pair({eps, eps});
        solver = std::make_unique<BOAStar>(graph, eps_pair, logger);
    }else if (algorithm == "NAMOAdr"){
        EPS eps_vec (graph.get_num_of_objectives(), eps);
        solver = std::make_unique<NAMOAdr>(graph, eps_vec, logger);
        // ((ApexSearch*)solver.get())->set_merge_strategy(ms);
    }else if (algorithm == "Apex"){
        EPS eps_vec (graph.get_num_of_objectives(), eps);
        solver = std::make_unique<ApexSearch>(graph, eps_vec, logger);
        ((ApexSearch*)solver.get())->set_merge_strategy(ms);
    }else{
        std::cerr << "unknown solver name" << std::endl;
        exit(-1);
    }

    auto start =std::clock();
    (*solver)(source, target, heuristic, solutions, time_limit);
    runtime = std::clock() - start;

    std::vector<std::vector<double>> front;

    std::cout << "Node expansion: " << solver->get_num_expansion() << std::endl;
    std::cout << "Runtime: " <<  ((double) runtime) / CLOCKS_PER_SEC<< std::endl;
    num_exp = solver->get_num_expansion();
    num_gen = solver->get_num_generation();
    for (auto sol: solutions){
        front.push_back(sol->g);
        std::cout << *sol << std::endl;
    }


    output << algorithm << "-" << alg_variant << " (" << eps << ")" << "\t"
           << source << "\t" << target << "\t"
           << num_gen << "\t"
           << num_exp << "\t"
           << solutions.size() << "\t"
           << (double) runtime / CLOCKS_PER_SEC
           << std::endl;

    std::cout << "-----End Single Example-----" << std::endl;
}


void single_run_map(size_t graph_size, std::vector<Edge> & edges, size_t source, size_t target, std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, int time_limit) {

    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);
    std::ofstream stats;
    stats.open(output_path + output_file, std::fstream::app);

    single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit);
 }

void my_single_run_map(size_t graph_size, AdjacencyMatrix& graph, AdjacencyMatrix&inv_graph, size_t source, size_t target, std::string algorithm, MergeStrategy ms, LoggerPtr logger, struct::log &mylog, double eps, unsigned int time_limit){
    std::cout << "Start Computing Heuristic" << std::endl;
    ShortestPathHeuristic sp_heuristic(target, graph_size, inv_graph);
    // sp_heuristic.set_all_to_zero();
    std::cout << "Finish Computing Heuristic\n" << std::endl;

    using std::placeholders::_1;
    Heuristic heuristic = std::bind( &ShortestPathHeuristic::operator(), sp_heuristic, _1);

    SolutionSet solutions;
    int runtime = 0;
    auto start_t = std::chrono::high_resolution_clock::now();
    
    std::unique_ptr<AbstractSolver> solver;
    if (algorithm == "PPA"){
        Pair<double> eps_pair({eps, eps});
        solver = std::make_unique<PPA>(graph, eps_pair, logger);
    }else if (algorithm == "BOA"){
        Pair<double> eps_pair({eps, eps});
        solver = std::make_unique<BOAStar>(graph, eps_pair, logger);
    }else if (algorithm == "NAMOAdr"){
        EPS eps_vec (graph.get_num_of_objectives(), eps);
        solver = std::make_unique<NAMOAdr>(graph, eps_vec, logger);
    }else if (algorithm == "Apex"){
        EPS eps_vec (graph.get_num_of_objectives(), eps);
        solver = std::make_unique<ApexSearch>(graph, eps_vec, logger);
        ((ApexSearch*)solver.get())->set_merge_strategy(ms);
    }else{
        std::cerr << "unknown solver name" << std::endl;
        exit(-1);
    }

    
    (*solver)(source, target, heuristic, solutions, time_limit);

    runtime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();
    std::vector<std::vector<double>> front;

    std::cout << "Node expansion: " << solver->get_num_expansion() << std::endl;
    std::cout << "Runtime: " <<  runtime << " ms"<< std::endl;
    if(solutions.empty()){
        std::cout << "NO SOLUTION FOUND" << std::endl;

    } else{ 
    
        for (auto sol: solutions){
            front.push_back(sol->g);
            mylog.front.push_back(sol->g);
            std::cout << *sol << std::endl;
        }
        mylog.norm_front = normalize_matrix(front);
        mylog.time = runtime;
        mylog.d_score = d_score(front);
        mylog.norm_d_score = d_score(mylog.norm_front);
        mylog.sparsity = sparsity_metric(front);
    }

}


void my_single_run_map(size_t graph_size, std::vector<Edge> & edges, size_t source, size_t target, std::string algorithm, MergeStrategy ms, LoggerPtr logger, struct::log &mylog, double eps, int time_limit){

    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);
    my_single_run_map(graph_size, graph, inv_graph, source, target, algorithm, ms, logger, mylog, eps, time_limit);

}

void run_query(size_t graph_size, std::vector<Edge> & edges, std::string query_file, std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, int time_limit) {
    std::ofstream stats;
    stats.open(output_path + output_file, std::fstream::app);


    std::vector<std::pair<size_t, size_t>> queries;
    if (load_queries(query_file, queries) == false) {
        std::cout << "Failed to load queries file" << std::endl;
        return;
    }

    // Build graphs
    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);
    std::cout << "graph size: " << graph.get_num_of_objectives() << std::endl;

    size_t query_count = 0;
    for (auto iter = queries.begin(); iter != queries.end(); ++iter) {

        query_count++;
        std::cout << "Started Query: " << query_count << "/" << queries.size() << std::endl;
        size_t source = iter->first;
        size_t target = iter->second;

        single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit);
    }

}
// motify to pass my logger
void my_run_query(size_t graph_size, std::vector<Edge> & edges, std::string query_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, std::vector<struct::log> &mylogs, double eps, int time_limit){

    std::vector<std::pair<size_t, size_t>> queries;
    if (load_queries(query_file, queries) == false) {
        std::cout << "Failed to load queries file" << std::endl;
        return;
    }

    // Build graphs
    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);
    std::cout << "objectives: " << graph.get_num_of_objectives() << std::endl;


    size_t query_count = 0;
    for (auto iter = queries.begin(); iter != queries.end(); ++iter) {

        query_count++;
        std::cout << "Started Query: " << query_count << "/" << queries.size() << std::endl;
        size_t source = iter->first;
        size_t target = iter->second;

        struct::log a(source, target);
        a.source = source;
        a.target = target;
        my_single_run_map(graph_size, graph, inv_graph, source, target, algorithm, ms, logger, a, eps, time_limit);
        mylogs.push_back(a);
    }

    std::cout << "querey done" << std::endl;
}

// QUERY FOR ASCII MAPS
bool load_asci_queries(const std::string asci_query_file, std::unordered_map<std::string, std::vector<std::vector<size_t>>> &queries){
    std::ifstream ifs(asci_query_file);

    std::string x;
    size_t s, t;



    while(ifs >> x){
    // std::cout << x << std::endl;
    queries[x];
    // std::cout << x.substr(18) << std::endl;
    for(int i = 0; i < 5; i++){
      ifs >> s >> t;
      // std::cout << "  " << s << " " << t << std::endl;
      std::vector<size_t> temp = {s, t};
      queries[x].push_back(temp);
    }
    }

    ifs.close();
    std::cout << "loaded " << queries.size() << " queries" << std::endl;

    return true;
}


void my_run_query_ascii(std::string asci_query_file, std::string algorithm, MergeStrategy ms, const boost::program_options::variables_map &vm,  LoggerPtr logger, double eps, int time_limit){
    std::unordered_map<std::string, std::vector<std::vector<size_t>>> queries;
    size_t graph_size;
    std::vector<Edge> edges;

    if(load_asci_queries(asci_query_file, queries) == false){
        std::cout << "Failed to load queries file" << std::endl;
        return;
    }
    
    std::vector<std::vector<struct::log>> INST_LOGS;
    std::vector<std::string> MAP_NAMES;
    int m = 1;
    for(auto map : queries){    

        std::vector<struct::log> LOGS;
        //build graph
        load_ascii_file(map.first, edges, graph_size);

        AdjacencyMatrix graph(graph_size, edges);
        AdjacencyMatrix inv_graph(graph_size, edges, true);
        

        int n = 1;
        for(auto &inst : map.second){
            size_t source = inst[0],
                   target = inst[1];
            
            std::cout << "------" << std::endl;
            std::cout << map.first.substr(18) << std::endl;
            std::cout << "(" << m << "/156)" << std::endl;
            std::cout << source << " " << target << std::endl;
            std::cout << "(" << n << "/5)" << std::endl;
            std::cout << "------" << std::endl;

            struct::log a(source, target);
            a.source = source;
            a.target = target;
            my_single_run_map(graph_size, graph, inv_graph, source, target, algorithm, ms, logger, a, eps, time_limit);
            LOGS.push_back(a);
            n++;
        }
        std::cout << "======" << std::endl;
        m++;
        INST_LOGS.push_back(LOGS);
        MAP_NAMES.push_back(map.first.substr(18));
    }
    write_logs(vm["merge"].as<string>(), vm["eps"].as<double>(), INST_LOGS, MAP_NAMES);
}

int main(int argc, char** argv){
    namespace po = boost::program_options;

    std::vector<string> objective_files;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("start,s", po::value<int>()->default_value(-1), "start location")
        ("goal,g", po::value<int>()->default_value(-1), "goal location")
        ("query,q", po::value<std::string>()->default_value(""), "number of agents")
        ("map,m",po::value< std::vector<string> >(&objective_files)->multitoken(), "files for edge weight") // order does not matter (dont put cer)
        ("eps,e", po::value<double>()->default_value(0), "approximation factor")
        ("merge,x", po::value<std::string>()->default_value("RANDOM"), "strategy for merging apex node pair: SMALLER_G2, RANDOM or MORE_SLACK")
        ("algorithm,a", po::value<std::string>()->default_value("Apex"), "solvers (BOA, PPA or Apex search)")
        ("cutoffTime,t", po::value<int>()->default_value(180), "cutoff time (seconds)")
        ("output,o", po::value<std::string>(), "Name of the output file")
        ("logging_file", po::value<std::string>()->default_value(""), "logging file" )
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);
    // srand((int)time(0));

    // splitting information flow for ASCII and ROAD maps 
    std::cout << "---" << std::endl;
    for(auto &file: objective_files){
        std::cout << file << std::endl;
    }
    std::cout << "---" << std::endl;


    std::vector<struct::log> LOGS;

    std::vector<Edge> edges;
    size_t graph_size;
    LoggerPtr logger = nullptr;

    MergeStrategy ms = DEFAULT_MERGE_STRATEGY; // overwitten later
    alg_variant = vm["merge"].as<std::string>();


    if (vm["merge"].as<std::string>() != "" && vm["algorithm"].as<std::string>()!= "Apex"){
        alg_variant = "";
        std::cout << "WARNING: merge strategy with non-apex search" << std::endl;
    }else if(vm["merge"].as<std::string>() == "SMALLER_G2"){
        ms = MergeStrategy::SMALLER_G2;
    }else if(vm["merge"].as<std::string>() == "SMALLER_G2_FIRST"){
        ms = MergeStrategy::SMALLER_G2_FIRST;
    }else if(vm["merge"].as<std::string>() == "RANDOM"){
        ms = MergeStrategy::RANDOM;
    }else if(vm["merge"].as<std::string>() == "MORE_SLACK"){
        ms = MergeStrategy::MORE_SLACK;
    }else if(vm["merge"].as<std::string>() == "REVERSE_LEX"){
        ms = MergeStrategy::REVERSE_LEX;
    }else{
        std::cerr << "unknown merge strategy" << std::endl;
    }

    std::string MAP_NAME;
    if(objective_files.front().back() == 'r') {                            // ROAD
        MAP_NAME = objective_files.front();
        MAP_NAME = MAP_NAME.substr(MAP_NAME.size() - 6, 3);
        // std::cout << MAP_NAME << std::endl;
        if( load_gr_files(objective_files, edges, graph_size) == false){
            std::cout << "failed to load .gr files." << std::endl;
            return -1;
        }
        std::cout << "Graph size: " << graph_size << std::endl;
        // for(auto &e: edges){
        //     std::cout << e << std::endl;
        // }
    
        if (vm["query"].as<std::string>() != ""){
            my_run_query(graph_size, edges, vm["query"].as<std::string>(), vm["algorithm"].as<std::string>(), ms, logger, LOGS,  vm["eps"].as<double>(), vm["cutoffTime"].as<int>());
            // std::cout << vm["merge"].as<std::string>() << " " << vm["eps"].as<double>() << std::endl;
            write_logs(MAP_NAME, vm["merge"].as<std::string>(), vm["eps"].as<double>(), LOGS);

        } else{
            struct::log a;
            my_single_run_map(graph_size, edges, vm["start"].as<int>(), vm["end"].as<int>(), vm["algorithm"].as<string>(), ms, logger, a, vm["eps"].as<double>(), vm["cutoffTime"].as<int>());
        }           
    } else if(objective_files.front() == "ASCII") {                    // DAO
        // only supports queuries... for now...
        my_run_query_ascii(vm["query"].as<std::string>(), vm["algorithm"].as<std::string>(), ms, vm, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>());
    }


    return 0;
}
