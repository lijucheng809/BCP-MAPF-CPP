

#ifndef MAPF_BCP_PRICER_PRICERPROBLEM_H_
#define MAPF_BCP_PRICER_PRICERPROBLEM_H_

#include <utility>

#include "../STA_STAR/STA_STAR.h"
#include "../Include/Include.h"

class Pricerproblem : public STA_STAR{
 public:
  Pricerproblem(const std::vector<Point>&mapSet,
                int start_id,
                int goal_id,
                const std::pair<double,std::pair<std::unordered_map<NodeTime,double,NodeTimeHasher>,std::unordered_map<Edge,double,EdgeHasher>> >& dualSet)
      :STA_STAR(),
        mapSet(mapSet),
        start_id(start_id),
        goal_id(goal_id),
        agv_route_dual(dualSet.first),
        vertexPenalty(dualSet.second.first),
        edgePenalty(dualSet.second.second)
  {
    nt_id = 0;
  }

  double solve();
  inline std::pair<std::vector<NodeTime>,double> getRoute() const {return output;}
  inline bool isReducedcostNegtive(){
    //std::cout<<"the reduced cost is "<<reduced_cost<<std::endl;
    if(reduced_cost<0)
      return true;
    return false;
  }

 protected:
  void get_Neighborlabel(const label& lb,int goal) override;
  void get_Kidlable(const Point& point,
                    const label& lb,
                    int time,
                    Direction d) ;
  void finalProcess(label&lb_top) override;
  void getTurnaroundLabel(int time,
                          int turnaroundSecond,
                          std::shared_ptr<label>& lb_parent_sp,
                          Direction d,
                          double h) override;
  double huristics(int start,int goal) override;
  void search(int start_id,int goal_id);

 private:
  int start_id;
  int goal_id;
  double reduced_cost;
  int nt_id ;

  std::unordered_map<NodeTime,double,NodeTimeHasher>vertexPenalty;
  std::unordered_map<Edge,double,EdgeHasher>edgePenalty;
  double agv_route_dual;

  std::vector<Point>mapSet;
  std::unordered_set<NodeTime,NodeTimeHasher>closedTable;
  std::priority_queue<label,std::vector<label>,compare>openTable;
  std::pair<std::vector<NodeTime>,double>output;

  inline static Edge getEdge(const NodeTime& n1,const NodeTime& n2) {
    Edge edge;
    edge.l1 = std::make_shared<NodeTime>(n1);
    edge.l2 = std::make_shared<NodeTime>(n2);
    return edge;
  }
};

#endif//MAPF_BCP_PRICER_PRICERPROBLEM_H_
