#include <iostream>
#include <vector>
#include <ostream>
#include <fstream>

int main(){
  std::vector<int> V;

  std::ofstream out_file("seg.json");


  out_file << "{\"data\": [";

  out_file << V[10000];

  out_file << "]}";


  return 0;
}
