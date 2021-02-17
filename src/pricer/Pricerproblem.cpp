
#include "Pricerproblem.h"
void Pricerproblem::getTurnaroundLabel(int time,
                                       int turnaroundSecond,
                                       std::shared_ptr<label> &lb_parent_sp,
                                       Direction d,
                                       double h) {

  int temp_time = time;
  Point pt = mapSet[lb_parent_sp->nt.node_id];
  for(int i=0;i<turnaroundSecond;i++){
    nt_id += 1;
    temp_time += 1;
    NodeTime nt(temp_time,pt.ID,nt_id);

    auto edge = getEdge(lb_parent_sp->nt,nt);

    double g = temp_time;
    if(vertexPenalty.find(nt)!=vertexPenalty.end()){
      g += vertexPenalty[nt];
    }
    if(edgePenalty.find(edge)!=edgePenalty.end())
      g += edgePenalty[edge];
    std::shared_ptr<label>lb_kid_sp = std::make_shared<label>(nt, g,h);

    lb_kid_sp->real_distance = temp_time;
    lb_kid_sp->d = d;
    lb_kid_sp->parent = lb_parent_sp;
    lb_parent_sp = lb_kid_sp;
  }

}

double Pricerproblem::huristics(int start, int goal) {
  return fabs(mapSet[start].x - mapSet[goal].x) + fabs(mapSet[start].y - mapSet[goal].y);
}

void Pricerproblem::get_Kidlable(const Point &point,
                                 const label &lb,
                                 int time,
                                 Direction d) {
  double h = huristics(point.ID,goal_id);
  Turnaround isturnaround = false;
  std::shared_ptr<label>lb_parent_sp = std::make_shared<label>(lb);
  if(lb.d == -1 || lb.d == d){
    nt_id += 1;
    time += 1;
  }else if(std::abs(lb.d-d ) == 1){
    isturnaround = true;

    getTurnaroundLabel(time,4,lb_parent_sp,d,h);

    nt_id += 1;
    time = time + 5;
  }else{
    isturnaround = true;

    getTurnaroundLabel(time,2,lb_parent_sp,d,h);

    nt_id += 1;
    time = time + 3;
  }

  NodeTime nt(time,point.ID,nt_id);
  auto edge = getEdge(lb_parent_sp->nt,nt);

  double g = time;
  if(vertexPenalty.find(nt)!=vertexPenalty.end()){
    g += vertexPenalty[nt];
  }
  if(edgePenalty.find(edge)!=edgePenalty.end())
    g += edgePenalty[edge];

  label lb_kid(nt,g,h); //记得写好turnaround后减1

  lb_kid.real_distance = time;
  lb_kid.d = d;
  std::shared_ptr<label>lb_kid_sp = std::make_shared<label>(lb_kid);  //sp --> smart pointer
  lb_kid_sp->tr = isturnaround;
  lb_kid_sp->parent = lb_parent_sp;
  openTable.push(*lb_kid_sp);

}

void Pricerproblem::get_Neighborlabel(const label &lb,int goal ) {
  Point pt = mapSet[lb.nt.node_id];
  int now_time = lb.nt.time ;

  if(pt.UP &&closedTable.find(lb.nt) == closedTable.end()){
    get_Kidlable(*pt.UP,lb,now_time,1);
  }
  if(pt.DOWN &&closedTable.find(lb.nt) == closedTable.end()){
    get_Kidlable(*pt.DOWN,lb,now_time,2);
  }
  if(pt.LEFT &&closedTable.find(lb.nt) == closedTable.end()){
    get_Kidlable(*pt.LEFT,lb,now_time,4);
  }
  if(pt.RIGHT &&closedTable.find(lb.nt) == closedTable.end()){
    get_Kidlable(*pt.RIGHT,lb,now_time,5);
  }
  /*wait*/
  if(lb.parent){
    get_Kidlable(pt,lb, now_time,lb.parent->d);
  }else{
    get_Kidlable(pt,lb, now_time,-1);
  }
}

void Pricerproblem::finalProcess(label &lb_top) {

  output.second = lb_top.real_distance;
  reduced_cost = lb_top.g - agv_route_dual;
/*  for(const auto& item:vertexPenalty){
    std::cout<<"vextex penalty nodeid is "<<item.first.node_id<<" penalty is "<<item.first.time<<std::endl;
  }
  std::cout<<" lb.top.g is "<<lb_top.g<<" dual is "<<agv_route_dual<<std::endl;*/

  while(lb_top.parent != nullptr){
    NodeTime nt1 = lb_top.nt;
    output.first.push_back(nt1);
    lb_top = *(lb_top.parent);
  }
  NodeTime nt1 = lb_top.nt;
  output.first.push_back(nt1);
  std::reverse(output.first.begin(),output.first.end());

}

void Pricerproblem::search(int start_id, int goal_id) {

  NodeTime node_time(0,start_id,nt_id);
  label lb(node_time,0,huristics(start_id,goal_id));
  std::shared_ptr<label>top_lab_sp = std::make_shared<label>(lb);
  openTable.push(*top_lab_sp);

  while(!openTable.empty()){
    auto lb_top = openTable.top();
    openTable.pop();

    if(lb_top.nt.node_id == goal_id){
      finalProcess(lb_top);
      break;
    }

    get_Neighborlabel(lb_top,goal_id);

    if(!lb_top.parent || lb_top.parent->tr == lb_top.tr){
      closedTable.insert(lb_top.nt);
    }else if(abs(lb_top.parent->d - lb_top.d)==1){
      auto parent_temp = lb_top;
      for(int i=0;i<4;i++){
        auto nt_temp = parent_temp.parent->nt;
        closedTable.insert(nt_temp);
        parent_temp = *(parent_temp.parent);
      }
    }else {
      auto parent_temp = lb_top;
      for (int i = 0; i < 2; i++) {
        auto nt_temp = parent_temp.parent->nt;
        closedTable.insert(nt_temp);
        parent_temp = *(parent_temp.parent);
      }
    }
  }
}
double Pricerproblem::solve() {

  search(start_id,goal_id);

  return reduced_cost;
}
