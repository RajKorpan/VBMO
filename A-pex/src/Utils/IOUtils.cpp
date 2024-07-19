#include <string>
#include <vector>
#include <fstream>
#include <unordered_map>
#include <algorithm>
#include <random>
//#include "../../inc/Utils/Definitions.h"
#include "Utils/IOUtils.h"

void split_string(std::string string, std::string delimiter, std::vector<std::string> &results)
{
    size_t first_delimiter;

    while ((first_delimiter = string.find_first_of(delimiter)) != string.npos) {
        if (first_delimiter > 0) {
            results.push_back(string.substr(0, first_delimiter));
        }
        string = string.substr(first_delimiter + 1);
    }

    if (string.length() > 0) {
        results.push_back(string);
    }
}

bool load_gr_files(std::vector<std::string> gr_files, std::vector<Edge> &edges_out, size_t &graph_size){
  size_t          max_node_num = 0;
  for (auto gr_file: gr_files){
    std::ifstream file(gr_file.c_str());
    
    if (file.is_open() == false){
      std::cerr << "cannot open the gr file " << gr_file << std::endl;
      return false;
    }

    std::string line;
    int idx_edge = 0;
    while (file.eof() == false) {
        std::getline(file, line);

        if (line == "") {
            break;
        }

        std::vector<std::string> decomposed_line;
        split_string(line, " ", decomposed_line);

        std::string type = decomposed_line[0];
        if ((std::strcmp(type.c_str(),"c") == 0) || (std::strcmp(type.c_str(),"p") == 0)) {
            continue; //comment or problem lines, not part of the graph
        }

        if (std::strcmp(type.c_str(),"a") == 0) { //arc
          if (idx_edge < (int)edges_out.size() - 1){
            if (
                (stoul(decomposed_line[1]) != edges_out[idx_edge].source) ||
                (stoul(decomposed_line[2]) != edges_out[idx_edge].target)) {
              // arc_sign src dest should be same in both files
              std::cerr << "file inconsistency" << std::endl;
              return false;
            }
            edges_out[idx_edge].cost.push_back(std::stoul(decomposed_line[3]));
          }else{
            Edge e(std::stod(decomposed_line[1]),
                   std::stod(decomposed_line[2]),
                   {std::stod(decomposed_line[3])});
            edges_out.push_back(e);
            max_node_num = std::max({max_node_num, e.source, e.target});
          }
        }
        idx_edge ++;
    }
    file.close();
  }
  graph_size = max_node_num;
  return true;
}

bool load_gr_files(std::string gr_file1, std::string gr_file2, std::vector<Edge> &edges_out, size_t &graph_size) {
    size_t          max_node_num = 0;
    std::ifstream   file1(gr_file1.c_str());
    std::ifstream   file2(gr_file2.c_str());

    if ((file1.is_open() == false) || (file2.is_open() == false)) {
        return false;
    }

    std::string line1, line2;
    while ((file1.eof() == false) && (file2.eof() == false)) {
        std::getline(file1, line1);
        std::getline(file2, line2);

        if ((line1 == "") || (line2 == "")) {
            break;
        }

        std::vector<std::string> decomposed_line1, decomposed_line2;
        split_string(line1, " ", decomposed_line1);
        split_string(line2, " ", decomposed_line2);

        std::string type = decomposed_line1[0];
        if ((std::strcmp(type.c_str(),"c") == 0) || (std::strcmp(type.c_str(),"p") == 0)) {
            continue; //comment or problem lines, not part of the graph
        }

        if ((decomposed_line1[0] != decomposed_line2[0]) ||
            (decomposed_line1[1] != decomposed_line2[1]) ||
            (decomposed_line1[2] != decomposed_line2[2])) {
            // arc_sign src dest should be same in both files
            return false;
        }

        if (std::strcmp(type.c_str(),"a") == 0) { //arc
            Edge e(std::stoul(decomposed_line1[1]),
                   std::stoul(decomposed_line1[2]),
                   {std::stod(decomposed_line1[3]), std::stod(decomposed_line2[3])});
            edges_out.push_back(e);
            max_node_num = std::max({max_node_num, e.source, e.target});
        }
    }
    graph_size = max_node_num;
    return true;
}

bool load_txt_file(std::string txt_file, std::vector<Edge> &edges_out, size_t &graph_size) {
    bool            first_line = true;
    size_t          max_node_num = 0;
    std::ifstream   file(txt_file.c_str());

    if (file.is_open() == false) {
        return false;
    }

    std::string line;
    while (file.eof() == false) {
        std::getline(file, line);

        if (line == "") {
            break;
        }

        std::vector<std::string> decomposed_line;
        split_string(line, " ", decomposed_line);

        if (first_line) {
            first_line = false;
            continue;
        }
        Edge e(std::stoul(decomposed_line[0]), //source
               std::stoul(decomposed_line[1]), //target
               {std::stod(decomposed_line[2]), std::stod(decomposed_line[3])}); //objective cost
        edges_out.push_back(e);
        max_node_num = std::max({max_node_num, e.source, e.target});
    }
    graph_size = max_node_num;
    return true;
}


bool load_queries(std::string query_file, std::vector<std::pair<size_t, size_t>> &queries_out) {
    std::ifstream   file(query_file.c_str());

    if (file.is_open() == false) {
        return false;
    }

    std::string line;
    while (file.eof() == false) {
        std::getline(file, line);

        if (line == "") {
            break;
        } else if (line[0] == '#') {
            continue; // Commented out queries
        }

        std::vector<std::string> decomposed_line;
        split_string(line, ",", decomposed_line);

        std::pair<size_t, size_t> query = {std::stoul(decomposed_line[0]), std::stoul(decomposed_line[1])};
        queries_out.push_back(query);
    }
    return true;
}

bool load_ascii_file(std::string ascii_file, std::vector<Edge> &edges, size_t &graph_size){
    std::ifstream ifs(ascii_file);

    std::string temp;
    int height, width;

    //removing header 
    std::getline(ifs, temp);
    std::getline(ifs, temp);
    height = stoi(temp.substr(7));
    std::getline(ifs, temp);
    width = stoi(temp.substr(6));
    std::getline(ifs, temp);

    size_t id = 0;
    

    //allocate just enough space for the ascii map
    std::vector<std::vector<char>> map(height, std::vector<char>(width));
    // need a way of converting xy coordinates into the id for creating the edges later on

    struct pair_hash{
        size_t operator()(const std::pair<size_t,size_t> &p) const {
            auto h1 = std::hash<size_t>{}(p.first),
                 h2 = std::hash<size_t>{}(p.second);
            if(h1 != h2){
                return h1 ^ h2;
            } else {
                return h1;
            }
        }
    };

    std::unordered_map<std::pair<size_t,size_t>, size_t, pair_hash> coordinateIDmap;
    edges.clear();

    // matching xy coordinates to id's 
    for(int i = 0; i < height; i++){
        std::getline(ifs, temp);
        // std::cout << temp << std::endl; // DEBUGGING
        for(int j = 0; j < width; j++){
            map[i][j] = temp[j];
            if(map[i][j] == '.') { // "."s on the ascii map is a position 
                coordinateIDmap[{i,j}] = id;
                id++;
                // std::cout << i << " " << j << " " << id << std::endl;
            }
        }
    }

    ifs.close();
    graph_size = id;

    // adding euclidean distance, uniform, and random objective costs

    std::mt19937 RNG(96534);

    // !!! size_t does not go into the negatives, 0-1 does not result in a negative number, but the larget v
    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            if(map[i][j] == '.'){
          
                if(i-1 >= 0 && map[i-1][j] == '.'){                      // up 
                // std::cout << "  " << i-1 << " " << j << std::endl;
                edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i-1,j}], {1, 1, 1 + double(RNG()%20)}));
                }
                if(i-1 >= 0 && j + 1 < width && map[i-1][j+1] == '.'){   // up right
                // std::cout << "  " << i-1 << " " << j+1 << std::endl;
                edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i-1,j+1}], {1, 1.414, 1 + double(RNG()%20)}));
                }
                if(j+1 < width && map[i][j+1] == '.'){                   // right
                // std::cout << "  " << i << " " << j+1 << std::endl;
                edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i,j+1}], {1, 1, 1 + double(RNG()%20)}));
                }
                if(i+1 < height && j+1 < width && map[i+1][j+1] == '.'){ // down right
                // std::cout << "  " << i+1 << " " << j+1 << std::endl;
                edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i+1,j+1}], {1, 1.414, 1 + double(RNG()%20)}));
                }
                if(i+1 < height && map[i+1][j] == '.'){                  // down
                // std::cout << "  " << i+1 << " " << j << std::endl;
                edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i+1,j}], {1, 1, 1 + double(RNG()%20)}));
                }
                if(i+1 < height && j-1 >= 0 && map[i+1][j-1] == '.'){    // down left
                // std::cout << "  " << i+1 << " " << j-1 << std::endl;
                edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i+1,j-1}], {1, 1.414, 1 + double(RNG()%20)}));
                }
                if(j-1 >= 0 && map[i][j-1] == '.'){                      // left
                // std::cout << "  " << i << " " << j-1 << std::endl;
                edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i,j-1}], {1, 1, 1 + double(RNG()%20)}));
                }
                if(i-1 >= 0 && j-1 >= 0 && map[i-1][j-1] == '.'){        // up lef
                // std::cout << "  " << i-1 << " " << j-1 << std::endl;
                edges.push_back(Edge(coordinateIDmap[{i,j}], 
                                    coordinateIDmap[{i-1,j-1}], {1, 1.414, 1 + double(RNG()%20)}));
                }
            }
        }
    }

    // std::cout << "added first three objectives" << std::endl;

    //leave in for 3-objective, comment out to add last 2 objectives
    // return true;

    // adding random dangerous objective
    std::vector<std::vector<Edge>> tempMap(id, std::vector<Edge> {});

    // create list of all edges that have the same target
    for(auto iter = edges.begin(); iter != edges.end(); iter++){
        tempMap[iter->target].push_back(*iter);
    }

    //iterating over id's adjacency list
    for(auto iter = tempMap.begin(); iter != tempMap.end(); iter++){
        double n;
        if(RNG() % 10 == 0){    //randomly make 10% of all incoming edges expensive 
            n = 10;
        } else {
            n = 1.5;
        }

        for(auto jter = iter->begin(); jter != iter->end(); jter++){
            jter->cost.push_back(n);
        }
    }

    //return to edge list
    edges.clear();
    for(auto iter =  tempMap.begin(); iter != tempMap.end(); iter++){
        edges.insert(edges.end(), iter->begin(), iter->end());
    }

    // reverse adjacency list
    tempMap.clear();
    tempMap = std::vector<std::vector<Edge>>(id, std::vector<Edge> {});
    for(auto iter = edges.begin(); iter != edges.end(); iter++){
        tempMap[iter->source].push_back(*iter);
    }

    //adding safety objective
    for(int i = 0; i < tempMap.size(); i++){
        for(int j = 0; j < tempMap[i].size(); j++){
            tempMap[i][j].cost.push_back( (10 - tempMap[i].size() + 10 - tempMap[j].size()) / 2.0);
        }
    }

    // revert to edges;
    edges.clear();
    for(auto iter =  tempMap.begin(); iter != tempMap.end(); iter++){
        edges.insert(edges.end(), iter->begin(), iter->end());
    }

    // std::cout << coordinateIDmap.size() << " " << id << " " << graph_size << std::endl;


    return true;
}
