
#include "Map.h"

#include <iostream>
#include <fstream>

void Map::readfile() {

  std::ifstream f(file);
  if(!f) std::cout<<"not open"<<std::endl;
  for(int i=0;i<num_points;i++){
    std::vector<double>arr(14,0.0);
    for(int j=0;j<14;j++){
      f>>arr[j];
    }
    Point point(arr[0],arr[1],i,false);

    if(int(arr[2])!=0 || arr[3]!=0 || arr[4]!=0){
      auto *point_kid = new Point(arr[3], arr[4], arr[2], false);
      if(arr[3]>arr[0]) {
        point.RIGHT = point_kid;
      }
      else if(arr[3]<arr[0]){
        point.LEFT = point_kid;
      }
      else if(arr[4]>arr[1]){
        point.UP = point_kid;
      }
      else if(arr[4]<arr[1]){
        point.DOWN = point_kid;
      }
    }

    if(int(arr[5])!=0 || arr[6]!=0 || arr[7]!=0){
      auto *point_kid = new Point(arr[6], arr[7], arr[5], false);
      if(arr[6]>arr[0]) {
        point.RIGHT = point_kid;
      }
      else if(arr[6]<arr[0]){
        point.LEFT = point_kid;
      }
      else if(arr[7]>arr[1]){
        point.UP = point_kid;
      }
      else if(arr[7]<arr[1]){
        point.DOWN = point_kid;
      }
    }

    if(int(arr[8])!=0 || arr[9]!=0 || arr[10]!=0){
      auto *point_kid = new Point(arr[9], arr[10], arr[8], false);
      if(arr[9]>arr[0]) {
        point.RIGHT = point_kid;
      }
      else if(arr[9]<arr[0]){
        point.LEFT = point_kid;
      }
      else if(arr[10]>arr[1]){
        point.UP = point_kid;
      }
      else if(arr[10]<arr[1]){
        point.DOWN = point_kid;
      }
    }
    if(int(arr[11])!=0 || arr[12]!=0 || arr[13]!=0){
      auto *point_kid = new Point(arr[12], arr[13], arr[11], false);
      if(arr[12]>arr[0]) {
        point.RIGHT = point_kid;
      }
      else if(arr[12]<arr[0]){
        point.LEFT = point_kid;
      }
      else if(arr[13]>arr[1]){
        point.UP = point_kid;
      }
      else if(arr[13]<arr[1]){
        point.DOWN = point_kid;
      }
    }
    mapset.emplace_back(point);
  }
  f.close();
}

std::vector<Point>& Map::getmap() {

  readfile();

  return mapset;
}
