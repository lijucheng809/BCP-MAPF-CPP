

#ifndef MAPF_BCP_MAP_MAP_H_
#define MAPF_BCP_MAP_MAP_H_
#include <vector>
#include <string>

//#include "../STA_STAR/Include.h"
#include "../Include/Include.h"

class Map {
 public:
  Map(std::string file,int num_points):file(file),num_points(num_points){}
  ~Map() = default;
  Map(Map&) = default;
  std::vector<Point>& getmap();
  int getMapsize() const {return mapset.size();}

 private:
  int num_points;
  std::string file;
  void readfile();
  std::vector<Point>mapset;

};

#endif//MAPF_BCP_MAP_MAP_H_
