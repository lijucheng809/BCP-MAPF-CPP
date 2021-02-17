

#ifndef MAPF_BCP_STA_STAR_STA_STAR_H_
#define MAPF_BCP_STA_STAR_STA_STAR_H_

#include <utility>
#include <vector>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <string>

#include "../Include/Include.h"
#include "../Map/Map.h"

class STA_STAR {
 public:
  STA_STAR() = default;
  explicit STA_STAR(const std::vector<Point>& mapSet,int num_agvs):mapSet(mapSet),num_agvs(num_agvs){
    nt_id = 0 ;};
  ~STA_STAR() = default;

  virtual std::pair<std::vector<std::vector<NodeTime>>,std::vector<double>>& solve(const std::vector<std::pair<Point,Point>>&agvs_od_point );

 protected:
  virtual void get_Neighborlabel(const label& lb,int goal_id);
  void search(int start_id,int goal_id);
  void get_Kidlable(const Point& point,const label& lb,int goal_id,int time,Direction d);
  virtual double huristics(int start,int goal);
  virtual void finalProcess(label&lb_top);
  virtual void getTurnaroundLabel(int time,
                            int turnaroundSecond,
                            std::shared_ptr<label>& lb_parent_sp,
                            Direction d,
                            double h);
  //label* turnaround(const label& lb,const label& lb_kid,int time);

 private:
  int num_agvs;
  int nt_id;

  //double huristics(int start,int goal);

  std::vector<std::vector<NodeTime>> path_soultionSet;

  std::unordered_map<int,int>goalntMap;     //key->gola_node_id value->nt_time
  std::vector<label>labelpool;
  std::vector<double>path_costSet;
  std::pair<std::vector<std::vector<NodeTime>>,std::vector<double>>output;
  std::vector<Point>mapSet;
  std::unordered_set<NodeTime,NodeTimeHasher>closedTable;
  std::priority_queue<label,std::vector<label>,compare>openTable;
  std::unordered_set<NodeTime,NodeTimeHasher> reseverdVertexTable;
  std::unordered_set<Edge,EdgeHasher>reserverdEdgeTable;

  inline static Edge getEdge(const NodeTime& n1,const NodeTime& n2) {
    Edge edge;
    edge.l1 = std::make_shared<NodeTime>(n1);
    edge.l2 = std::make_shared<NodeTime>(n2);
    return edge;
    }

  inline bool notinReservedEdgeTable(const Point& pt_first, const Point& pt_second,int time){
    NodeTime nt_first(time,pt_first.ID,1);
    NodeTime nt_second(time+1,pt_second.ID,2);
    auto edge = getEdge(nt_first,nt_second);
    if(reserverdEdgeTable.find(edge) != reserverdEdgeTable.end())
      return false;
    return true;
  }

  inline bool next_nt_not_in_resveredVertexTable(Point& pt,int time){
    nt_id += 1;
    NodeTime nt(time+1,pt.ID,nt_id);
    if(reseverdVertexTable.find(nt) != reseverdVertexTable.end())
      return false;
    return true;
  }
  //double huristics(int start,int goal);
  //bool nodetime_in_closedTable(NodeTime nt) {return closedTable.find(nt) != closedTable.end();}

};

#endif//MAPF_BCP_STA_STAR_STA_STAR_H_
