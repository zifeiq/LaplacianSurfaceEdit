// basic file operations
#include <iostream>
#include <fstream>
using namespace std;

int main () {
  ofstream myfile;
  myfile.open ("example.txt");
  int c = 1;
  // myfile << "Writing this to a file.\n";
  myfile << c;
  c++;
  myfile << c;
  myfile.close();
  return 0;
}