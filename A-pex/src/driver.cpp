#include <iostream>
#include <memory>
#include <time.h>
#include <random>
#include <iterator>
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

// reading instances
std::unordered_map<std::string, std::vector<std::vector<size_t>>> read_ASCII_instances(const std::string INSTANCE_FILE){
    std::unordered_map<std::string, std::vector<std::vector<size_t>>> instances;

    std::ifstream ifs(INSTANCE_FILE);

    std::string x;
    size_t s, t;

    while(ifs >> x){
    // std::cout << x << std::endl;
    instances[x];
    for(int i = 0; i < 40; i++){
            ifs >> s >> t;
            // std::cout << "  " << s << " " << t << std::endl;
            instances[x].push_back({s,t});
        }
    }

    ifs.close();  

    return instances;
}




struct log{
    std::string     algorithm,
                    merge_strategy,
                    file_name,
                    source,
                    target;
    int             time;   

  std::vector<std::vector<double>> front;

  log() = default;

  log(const std::string file_name_, const size_t source_, const size_t target_, const std::string alg_, const std::string merge_strategy_,  const std::vector<std::vector<double>> &front_)
  :file_name(file_name_), front(front_), source(std::to_string(source_)), target(std::to_string(target_)), algorithm(alg_), merge_strategy(merge_strategy_) {}
};

void write_array(std::ostream &out_file, const std::vector<double> &vec){
   out_file << "[" << vec[0];

   for(int i = 1; i < vec.size(); i++){
     out_file << ", " << vec[i];
   }

   out_file << "]";
}

void write_matrix(std::ostream &out_file, const std::vector<std::vector<double>> &matrix){
  out_file << "["; write_array(out_file, matrix[0]);
  for(int i = 0; i < matrix.size(); i++){
    out_file << ", "; write_array(out_file, matrix[i]);
  }

  out_file << "]";
}

void write_record(std::ostream &out_file, const struct::log &r){
    out_file << "{";

    out_file << "\"map-id\": " << "\"" << r.file_name << "\"";
    out_file << ", ";

    out_file << "\"source\": " << r.source;
    out_file << ", ";

    out_file << "\"target\": " << r.target;
    out_file << ", ";

    out_file << "\"algorithm\": " << r.algorithm;
    out_file << ", ";

    out_file << "\"front\": "; write_matrix(out_file ,r.front);
    out_file << ", ";

    out_file << "\"sparsity\": " << sparsity_metric(r.front);
    out_file << ", ";

    out_file << "\"time\": " << r.time;

    out_file << "}";
}

// all the records will be averaged in python, this is just to write it in json format
void write_all_records(const std::vector<struct::log> &rec, std::string file_name){
  std::ofstream out_file(file_name + ".json");

  out_file << "{\"data\": [";

  write_record(out_file, rec[0]);
  for(int i = 1; i < rec.size(); i++){
    out_file << ", ";
    write_record(out_file, rec[i]);
  }

  out_file << "]}";

  out_file.close();
}

using namespace std;

const std::string resource_path = "resources/";
const std::string output_path = "output/";
const MergeStrategy DEFAULT_MERGE_STRATEGY = MergeStrategy::SMALLER_G2;
std::string alg_variant = "";


struct::log logged_single_run_map(size_t graph_size, AdjacencyMatrix& graph, AdjacencyMatrix&inv_graph, size_t source, size_t target, std::ofstream& output, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, unsigned int time_limit) {
    // Compute heuristic
    struct::log LOG;
    LOG.algorithm = algorithm;

    LOG.source = source;
    LOG.target = target;

    //{SMALLER_G2, RANDOM, MORE_SLACK, SMALLER_G2_FIRST, REVERSE_LEX};
    switch (ms)
    {
    case SMALLER_G2:
        LOG.merge_strategy = "SMALLER_G2";
        break;
    
    case SMALLER_G2_FIRST:
        LOG.merge_strategy = "SMALLER_G2_FIRST";
        break;
    
    case RANDOM:
        LOG.merge_strategy = "RANDOM";
        break;

    case MORE_SLACK:
        LOG.merge_strategy = "MORE_SLACK";
        break;

    case REVERSE_LEX:
        LOG.merge_strategy = "REVERSE_LEX";
        break;
    default:
        //error
        LOG.merge_strategy = "ERROR";
        break;
    }

    auto start_t = std::chrono::high_resolution_clock::now();

    std::cout << "Start Computing Heuristic" << std::endl;
    ShortestPathHeuristic sp_heuristic(target, graph_size, inv_graph);
    // sp_heuristic.set_all_to_zero();
    std::cout << "Finish Computing Heuristic\n" << std::endl;
    std::cout << "Running " << algorithm << "..." << std::endl;

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

    int time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();
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

    LOG.front = front;
    LOG.time = time;

    return LOG;

    output << algorithm << "-" << alg_variant << " (" << eps << ")" << "\t"
           << source << "\t" << target << "\t"
           << num_gen << "\t"
           << num_exp << "\t"
           << solutions.size() << "\t"
           << (double) runtime / CLOCKS_PER_SEC
           << std::endl;
    
}

struct::log logged_single_run_map(size_t graph_size, std::vector<Edge> & edges, size_t source, size_t target, std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, int time_limit) {

    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);
    std::ofstream stats;
    stats.open(output_path + output_file, std::fstream::app);

    return logged_single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit);
 }

void single_run_map(size_t graph_size, AdjacencyMatrix& graph, AdjacencyMatrix&inv_graph, size_t source, size_t target, std::ofstream& output, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, unsigned int time_limit) {
    // Compute heuristic
    struct::log LOG;
    LOG.algorithm = algorithm;

    LOG.source = source;
    LOG.target = target;

    //{SMALLER_G2, RANDOM, MORE_SLACK, SMALLER_G2_FIRST, REVERSE_LEX};
    switch (ms)
    {
    case SMALLER_G2:
        LOG.merge_strategy = "SMALLER_G2";
        break;
    
    case SMALLER_G2_FIRST:
        LOG.merge_strategy = "SMALLER_G2_FIRST";
        break;
    
    case RANDOM:
        LOG.merge_strategy = "RANDOM";
        break;

    case MORE_SLACK:
        LOG.merge_strategy = "MORE_SLACK";
        break;

    case REVERSE_LEX:
        LOG.merge_strategy = "REVERSE_LEX";
        break;
    default:
        //error
        LOG.merge_strategy = "ERROR";
        break;
    }

    auto start_t = std::chrono::high_resolution_clock::now();

    std::cout << "Start Computing Heuristic" << std::endl;
    ShortestPathHeuristic sp_heuristic(target, graph_size, inv_graph);
    // sp_heuristic.set_all_to_zero();
    std::cout << "Finish Computing Heuristic\n" << std::endl;

    std::cout << "Running " << algorithm << std::endl;

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

    int time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_t).count();
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

    LOG.front = front;
    LOG.time = time;

    output << algorithm << "-" << alg_variant << " (" << eps << ")" << "\t"
           << source << "\t" << target << "\t"
           << num_gen << "\t"
           << num_exp << "\t"
           << solutions.size() << "\t"
           << (double) runtime / CLOCKS_PER_SEC
           << std::endl;
    
}

void single_run_map(size_t graph_size, std::vector<Edge> & edges, size_t source, size_t target, std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, int time_limit) {

    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);
    std::ofstream stats;
    stats.open(output_path + output_file, std::fstream::app);

    single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit);
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

    size_t query_count = 0;
    for (auto iter = queries.begin(); iter != queries.end(); ++iter) {

        query_count++;
        std::cout << "Started Query: " << query_count << "/" << queries.size() << std::endl;
        size_t source = iter->first;
        size_t target = iter->second;

        single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit);
    }

}

void display(const std::vector<size_t> &vec){
    if(vec.empty()){
        std::cout << "{}";
    } else{
        std::cout << "{" << vec[0];
        for(int i = 1; i < vec.size(); i++){
            std::cout << ", " << vec[i];
        }

        std::cout << "}";
    }
}

// int main(int argc, char** argv){
//     namespace po = boost::program_options;

//     std::vector<string> objective_files;

//     // Declare the supported options.
//     po::options_description desc("Allowed options");
//     desc.add_options()
//         ("help", "produce help message")
//         ("start,s", po::value<int>()->default_value(-1), "start location")
//         ("goal,g", po::value<int>()->default_value(-1), "goal location")
//         ("query,q", po::value<std::string>()->default_value(""), "number of agents")
//         ("map,m",po::value< std::vector<string> >(&objective_files)->multitoken(), "files for edge weight")
//         ("eps,e", po::value<double>()->default_value(0), "approximation factor")
//         ("merge", po::value<std::string>()->default_value(""), "strategy for merging apex node pair: SMALLER_G2, RANDOM or MORE_SLACK")
//         ("algorithm,a", po::value<std::string>()->default_value("Apex"), "solvers (BOA, PPA or Apex search)")
//         ("cutoffTime,t", po::value<int>()->default_value(300), "cutoff time (seconds)")
//         ("output,o", po::value<std::string>()->required(), "Name of the output file")
//         ("logging_file", po::value<std::string>()->default_value(""), "logging file" )
//         ;

//     po::variables_map vm;
//     po::store(po::parse_command_line(argc, argv, desc), vm);

//     if (vm.count("help")) {
//         std::cout << desc << std::endl;
//         return 1;
//     }

//     po::notify(vm);
//     srand((int)time(0));

//     if (vm["query"].as<std::string>() != ""){
//         if (vm["start"].as<int>() != -1 || vm["goal"].as<int>() != -1){
//             std::cerr << "query file and start/goal cannot be given at the same time !" << std::endl;
//             return -1;
//         }
//     }
    
//     LoggerPtr logger = nullptr;

//     if (vm["logging_file"].as<std::string>() != ""){
//         logger = new Logger(vm["logging_file"].as<std::string>());
//     }

//     // Load files
//     size_t graph_size;
//     std::vector<Edge> edges;

//     for (auto file:objective_files){
//         std::cout << file << std::endl;
//     }


//     if (load_gr_files(objective_files, edges, graph_size) == false) {
//         std::cout << "Failed to load gr files" << std::endl;
//         return -1;
//     }

//     std::cout << "Graph Size: " << graph_size << std::endl;

//     // Build graphs
//     MergeStrategy ms = DEFAULT_MERGE_STRATEGY;
//     alg_variant = vm["merge"].as<std::string>();

//     if (vm["merge"].as<std::string>() != "" && vm["algorithm"].as<std::string>()!= "Apex"){
//         alg_variant = "";
//         std::cout << "WARNING: merge strategy with non-apex search" << std::endl;
//     }else if(vm["merge"].as<std::string>() == "SMALLER_G2"){
//         ms = MergeStrategy::SMALLER_G2;
//     }else if(vm["merge"].as<std::string>() == "SMALLER_G2_FIRST"){
//         ms = MergeStrategy::SMALLER_G2_FIRST;
//     }else if(vm["merge"].as<std::string>() == "RANDOM"){
//         ms = MergeStrategy::RANDOM;
//     }else if(vm["merge"].as<std::string>() == "MORE_SLACK"){
//         ms = MergeStrategy::MORE_SLACK;
//     }else if(vm["merge"].as<std::string>() == "REVERSE_LEX"){
//         ms = MergeStrategy::REVERSE_LEX;
//     }else{
//         std::cerr << "unknown merge strategy" << std::endl;
//     }


//     if (vm["query"].as<std::string>() != ""){
//         run_query(graph_size, edges, vm["query"].as<std::string>(), vm["output"].as<std::string>(), vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>());
//     } else{
//         single_run_map(graph_size, edges, vm["start"].as<int>(), vm["goal"].as<int>(), vm["output"].as<std::string>(), vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>());
//     }

//     delete(logger);

//     return 0;
// }


std::vector<struct::log> instance_runner(const boost::program_options::variables_map &vm, std::unordered_map<std::string, std::vector<std::vector<size_t>>> &instances){

    std::vector<struct::log> logs;
    
    size_t graph_size;
    std::vector<Edge> edges;

    for(auto &inst : instances){
        std::string map_file = inst.first;
        load_ascii_file(map_file, edges, graph_size);
    }

}

int main(int argc, char** argv){

    auto instances = read_ASCII_instances("resources/instances/DAO-instances.txt");

    //USER INTERFACE STUFF
    namespace po = boost::program_options;

    std::string objective_files;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("start,s", po::value<int>()->default_value(-1), "start location")
        ("goal,g", po::value<int>()->default_value(-1), "goal location")
        ("query,q", po::value<std::string>()->default_value(""), "number of agents")
        ("map,m",po::value<std::string>()->default_value(""), "files for edge weight")
        ("eps,e", po::value<double>()->default_value(0), "approximation factor")
        ("merge", po::value<std::string>()->default_value("MORE_SLACK"), "strategy for merging apex node pair: SMALLER_G2, RANDOM or MORE_SLACK")
        ("algorithm,a", po::value<std::string>()->default_value("Apex"), "solvers (BOA, PPA or Apex search)")
        ("cutoffTime,t", po::value<int>()->default_value(300), "cutoff time (seconds)")
        ("output,o", po::value<std::string>()->required()->default_value("output.txt"), "Name of the output file")
        ("logging_file", po::value<std::string>()->default_value(""), "logging file" )
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);
    std::string file = vm["map"].as<std::string>();
    std::cout << file << std::endl;

    srand((int)time(0));
    

    if (vm["query"].as<std::string>() != ""){
        if (vm["start"].as<int>() != -1 || vm["goal"].as<int>() != -1){
            std::cerr << "query file and start/goal cannot be given at the same time !" << std::endl;
            return -1;
        }
    }
    
    LoggerPtr logger = nullptr;

    if (vm["logging_file"].as<std::string>() != ""){
        logger = new Logger(vm["logging_file"].as<std::string>());
    }

    // LOAD FILES
    size_t graph_size;
    std::vector<Edge> edges;

    // USE FOR ASCII 
    if(!(load_ascii_file(file, edges, graph_size))){
        std::cout << "Failed to load ASCII file" << std::endl;
        return -1;
    } else {
        std::cout << "Formed graph!" << std::endl;
    }

    // USE FOR DOA
    // if (load_gr_files(objective_files, edges, graph_size) == false) {
    //     std::cout << "Failed to load gr files" << std::endl;
    //     return -1;
    // }

    std::cout << "running  on " << file << std::endl; 
    std::cout << "Graph Size: " << graph_size << std::endl;

    // Build graphs
    MergeStrategy ms = DEFAULT_MERGE_STRATEGY;
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


    mt19937 rng(31848);

    size_t s = rng() % (graph_size - 1), g = rng() % (graph_size - 1);
    std::cout << "start: " << s << std::endl << "goal: " << g << std::endl;

    auto LOG = logged_single_run_map(graph_size, edges, s, g, vm["output"].as<std::string>(), vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>());

    // if (vm["query"].as<std::string>() != ""){
    //     run_query(graph_size, edges, vm["query"].as<std::string>(), vm["output"].as<std::string>(), vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>());
    // } else{
    //     // single_run_map(graph_size, edges, vm["start"].as<int>(), vm["goal"].as<int>(), vm["output"].as<std::string>(), vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>());
    //     single_run_map(graph_size, edges, s, g, vm["output"].as<std::string>(), vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>());

    // }

    delete(logger);

    return 0;
}
