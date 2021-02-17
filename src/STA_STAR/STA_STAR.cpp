
#include "STA_STAR.h"

double STA_STAR::huristics(int start, int goal) {
  return fabs(mapSet[start].x - mapSet[goal].x) + fabs(mapSet[start].y - mapSet[goal].y);
}

void STA_STAR::getTurnaroundLabel(int time,
                                    int turnaroundSecond,
                                    std::shared_ptr<label>& lb_parent_sp,
                                    Direction d,
                                    double h) {
  int g = time;
  Point pt = mapSet[lb_parent_sp->nt.node_id];
  for(int i=0;i<turnaroundSecond;i++){
    nt_id += 1;
    g += 1;

    NodeTime nt(g,pt.ID,nt_id);

    std::shared_ptr<label>lb_kid_sp = std::make_shared<label>(nt, g,h);
    lb_kid_sp->d = d;
    lb_kid_sp->parent = lb_parent_sp;
    lb_parent_sp = lb_kid_sp;
  }
}

void STA_STAR::get_Kidlable(const Point& point,
                            const label& lb,
                            int goal_id,
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
  label lb_kid(nt,time,h);
  lb_kid.d = d;
  std::shared_ptr<label>lb_kid_sp = std::make_shared<label>(lb_kid);  //sp --> smart pointer
  lb_kid_sp->tr = isturnaround;
  lb_kid_sp->parent = lb_parent_sp;
  openTable.push(*lb_kid_sp);
}

void STA_STAR::get_Neighborlabel(const label& lb,int goal_id) {

  Point pt = mapSet[lb.nt.node_id];
  int now_time = lb.nt.time ;

  if(pt.UP &&
      next_nt_not_in_resveredVertexTable(*pt.UP,now_time)&&
      reseverdVertexTable.find(lb.nt) == reseverdVertexTable.end()&&
      closedTable.find(lb.nt) == closedTable.end()&&
      (goalntMap.find(pt.UP->ID)==goalntMap.end()||goalntMap[pt.UP->ID]>now_time)&&
      notinReservedEdgeTable(pt,*pt.UP,now_time)){

    get_Kidlable(*pt.UP,lb,goal_id, now_time,1);

  }
  if(pt.DOWN &&
      next_nt_not_in_resveredVertexTable(*pt.DOWN,now_time)&&
      reseverdVertexTable.find(lb.nt) == reseverdVertexTable.end()&&
      closedTable.find(lb.nt) == closedTable.end()&&
      (goalntMap.find(pt.DOWN->ID)==goalntMap.end()||goalntMap[pt.DOWN->ID]>now_time)&&
      notinReservedEdgeTable(pt,*pt.DOWN,now_time)){

    get_Kidlable(*pt.DOWN,lb,goal_id, now_time,2);

  }
  if(pt.LEFT &&
      next_nt_not_in_resveredVertexTable(*pt.LEFT,now_time)&&
      reseverdVertexTable.find(lb.nt) == reseverdVertexTable.end()&&
      closedTable.find(lb.nt) == closedTable.end()&&
      (goalntMap.find(pt.LEFT->ID)==goalntMap.end()||goalntMap[pt.LEFT->ID]>now_time)&&
      notinReservedEdgeTable(pt,*pt.LEFT,now_time)){

    get_Kidlable(*pt.LEFT,lb,goal_id, now_time,4);

  }
  if(pt.RIGHT &&
      next_nt_not_in_resveredVertexTable(*pt.RIGHT,now_time)&&
      reseverdVertexTable.find(lb.nt) == reseverdVertexTable.end()&&
      closedTable.find(lb.nt) == closedTable.end()&&
      (goalntMap.find(pt.RIGHT->ID)==goalntMap.end()||goalntMap[pt.RIGHT->ID]>now_time)&&
      notinReservedEdgeTable(pt,*pt.RIGHT,now_time)){

    get_Kidlable(*pt.RIGHT,lb,goal_id, now_time,5);

  }

  /*wait*/
  if(next_nt_not_in_resveredVertexTable(pt,now_time)) {
    if (lb.parent) {
      get_Kidlable(pt, lb, goal_id, now_time, lb.parent->d);
    } else {
      get_Kidlable(pt, lb, goal_id, now_time, -1);
    }
  }
}

void STA_STAR::finalProcess(label &lb_top) {
  path_costSet.push_back(lb_top.g);
  goalntMap[lb_top.nt.node_id] = lb_top.nt.time;

  std::vector<NodeTime>temp;
  while(lb_top.parent != nullptr){
    NodeTime nt1 = lb_top.nt;
    temp.push_back(nt1);
    lb_top = *(lb_top.parent);
  }
  NodeTime nt1 = lb_top.nt;
  temp.push_back(nt1);
  path_soultionSet.push_back(temp);
  std::reverse(path_soultionSet[path_soultionSet.size()-1].begin(),
               path_soultionSet[path_soultionSet.size()-1].end());
}

void STA_STAR::search(int start_id, int goal_id) {

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
    //closedTable.insert(lb_top.nt);
  }
}

std::pair<std::vector<std::vector<NodeTime>>,std::vector<double>>& STA_STAR::solve(const std::vector<std::pair<Point, Point>> &agvs_od_point) {
  for(int i=0;i<agvs_od_point.size();i++){
    //std::cout<<agvs_od_point[i].first.ID<<" "<<agvs_od_point[i].second.ID<<std::endl;
    search(agvs_od_point[i].first.ID,agvs_od_point[i].second.ID);
    //std::cout<<agvs_od_point[i].first.ID<<" "<<agvs_od_point[i].second.ID<<std::endl;

    std::unordered_set<NodeTime,NodeTimeHasher>().swap(closedTable);  //clear closedTable memory
    std::priority_queue<label,std::vector<label>,compare>().swap(openTable);

    //std::vector<label>().swap(labelpool);
    /*给某个agv规划好路径后，需要将途经的时空点锁定*/
    for(auto & nt : path_soultionSet[i]){
      reseverdVertexTable.insert(nt);
    }
    /*锁定逆向边*/
    for(int j=path_soultionSet[i].size()-1;j>0;j--){

      NodeTime nt1(path_soultionSet[i][j].time-1,
                   path_soultionSet[i][j].node_id,
                   path_soultionSet[i][j].nt_id);

      NodeTime nt2(path_soultionSet[i][j-1].time+1,
                 path_soultionSet[i][j-1].node_id,
                   path_soultionSet[i][j-1].nt_id);

      auto edge = getEdge(nt1,nt2);
      reserverdEdgeTable.insert(edge);
    }
}
  for(int i=0;i<path_soultionSet[0].size();i++){
    std::cout<<"(node:"<<path_soultionSet[0][i].node_id<<" time:"<<path_soultionSet[0][i].time<<")->";
  }

  std::cout<<std::endl;
  std::cout<<path_costSet[0]<<std::endl;

  for(int i=0;i<path_soultionSet[1].size();i++){
    std::cout<<"(node:"<<path_soultionSet[1][i].node_id<<" time:"<<path_soultionSet[1][i].time<<")->";
  }

  std::cout<<std::endl;
  std::cout<<path_costSet[1]<<std::endl;
  output.first  = path_soultionSet;
  output.second = path_costSet;

  return output;
}
