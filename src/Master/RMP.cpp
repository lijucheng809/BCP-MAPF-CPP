
#include "RMP.h"

RMP::RMP(const std::pair<std::vector<std::vector<NodeTime>>,
    std::vector<double>> &initSloution,
         const int n_agvs)
    :n_agvs(n_agvs){

  routeSet      = NumVarMatrix(env,n_agvs);
  model         = IloModel(env,"CGcplex");
  constraints_  = IloRangeArray(env);
  objective_    = IloObjective(env);

  for(int i=0;i<n_agvs;i++){
    agvs_routeSet.push_back({initSloution.first[i]});
    agvs_routeSetCost.push_back({initSloution.second[i]});
    for(int j=0;j<initSloution.first[i].size()-1;j++){
      auto edge= getEdge(initSloution.first[i][j],initSloution.first[i][j+1]);
      std::pair<int,int>index(i,0);                                 //first->agvindex,second->agvrouteindex
      edgeMap[edge].push_back(index);
      nodetimeMap[initSloution.first[i][j+1]].push_back(index);
    }
  }
}

void RMP::addbranchConstraints() {
    auto branch_LHS_RHS = branch_strategy->getBranchConsLHS_RHS();
    int RHS = branch_LHS_RHS.second;
    std::vector<std::pair<int,int>>LHS = branch_LHS_RHS.first;
    constraints_.add(IloRange(env,0,RHS,"brcCons"));
    IloExpr branch_constraints(env);
    for(const auto& item:LHS){
        branch_constraints += routeSet[item.first][item.second];
    }
    constraints_[constraints_.getSize() - 1].setExpr(branch_constraints);
}

void RMP::buildModel() {
  std::cout<<"the model in building "<<std::endl;

  /*clear*/
  agvs_dual.clear();
  edgePenalty.clear();
  vertexPenalty.clear();
  vertexConflits_map.clear();
  edgeConflicts_map.clear();

  model.remove(constraints_);
  model.remove(objective_);
  objective_.removeFromAll();
  constraints_.clear();

  /*object function*/
  IloExpr obj(env);
  for(int i=0;i<n_agvs;i++){
    int n = agvs_routeSetCost[i].size();
    routeSet[i] = IloNumVarArray(env,n);
    for(int j=0;j<n;j++){
      routeSet[i][j] = IloNumVar(env,0,IloInfinity,ILOFLOAT);
      obj += agvs_routeSetCost[i][j]*routeSet[i][j];
    }
  }
  objective_.setExpr(IloMinimize(env,obj));

  /*set_partition constraints*/
  for(int i=0;i<n_agvs;i++){
    IloExpr set_partition_ctrs(env);
    const char* name = ("agv_"+to_string(i)).c_str();
    constraints_.add(IloRange(env,1,IloInfinity,name));
    int n = agvs_routeSetCost[i].size();
    for(int j=0;j<n;j++){
      set_partition_ctrs += routeSet[i][j];
    }
    constraints_[i].setExpr(set_partition_ctrs);
  }

  /*vertex conflicts constraints*/
  for(const auto& item:nodetimeMap){
    if(item.second.size() >1) {
      vertexConflits_map.insert(std::make_pair(item.first,constraints_.getSize()));
      IloExpr vertex_conflits_constraints(env);
      const char *name = ("node_" + to_string(item.first.node_id) + " time_" + to_string(item.first.time)).c_str();
      constraints_.add(IloRange(env, 0, 1, name));
      for (const auto &i : item.second) {
        vertex_conflits_constraints += routeSet[i.first][i.second];
      }
      constraints_[constraints_.getSize() - 1].setExpr(vertex_conflits_constraints);
    }
  }

  /*edge conflicts constraints*/
  std::unordered_set<Edge,EdgeHasher>tempEdgePool;
  for(const auto& item:edgeMap){
    if(tempEdgePool.find(item.first)==tempEdgePool.end()){
      tempEdgePool.insert(item.first);

      Edge reverseEdge;
      reverseEdge.l1 = std::make_shared<NodeTime>(item.first.l1->time,
                                           item.first.l2->node_id,
                                           item.first.l1->nt_id+1);
      reverseEdge.l2 = std::make_shared<NodeTime>(item.first.l2->time,
                                           item.first.l1->node_id,
                                           item.first.l2->nt_id+1);


      if(edgeMap.find(reverseEdge)!=edgeMap.end() && tempEdgePool.find(reverseEdge) == tempEdgePool.end()){
        edgeConflicts_map.insert(std::make_pair(item.first,constraints_.getSize()-1));
        edgeConflicts_map.insert(std::make_pair(reverseEdge,constraints_.getSize()-1));

        IloExpr edge_conflits_constraints(env);
        const char* name = ("edge_"+to_string(item.first.l1->node_id)+"_"+to_string(item.first.l2->node_id)).c_str();

        constraints_.add(IloRange(env,0,1,name));
        for(const auto& i:item.second){
          edge_conflits_constraints += routeSet[i.first][i.second];
        }

        tempEdgePool.insert(reverseEdge);
        for(const auto& j:edgeMap[reverseEdge]){
          edge_conflits_constraints += routeSet[j.first][j.second];
        }
        constraints_[constraints_.getSize()-1].setExpr(edge_conflits_constraints);
      }
    }else{
      continue;
    }
  }
  if(branch_strategy)
      addbranchConstraints();

  model.add(constraints_);
  model.add(objective_);
  cplex = IloCplex(model);
  cplex.exportModel("CG.lp");
  cplex.setOut(env.getNullStream());

}

void RMP::addColumns(int agv_id, const std::vector<NodeTime>& route,const double cost) {

  agvs_routeSet[agv_id].push_back(route);
  agvs_routeSetCost[agv_id].push_back(cost);

  for(int j=0;j<route.size()-1;j++){
    auto edge= getEdge(route[j],route[j+1]);
    std::pair<int,int>index(agv_id,agvs_routeSet[agv_id].size()-1);
    edgeMap[edge].push_back(index);

    nodetimeMap[route[j+1]].push_back(index);
  }
}

bool RMP::solve() {
  if(cplex.solve()){

    return true;
  }
  return false;
}

bool RMP::isOptimal() {

  if(cplex.getStatus() == IloAlgorithm::Optimal)
    return true;
  return false;

}

Dual_EdgeVertex_pair& RMP::getDual() {
  for(int i=0;i<n_agvs;i++){
    agvs_dual.push_back(cplex.getDual(constraints_[i]));
  }

  for(const auto & item:vertexConflits_map){
    if(cplex.getDual(constraints_[item.second])<0){
      vertexPenalty[item.first] = - cplex.getDual(constraints_[item.second]);
    }
  }

  for(const auto& item:edgeConflicts_map){
    if(cplex.getDual(constraints_[item.second]) < 0 ) {
      edgePenalty[item.first] = -cplex.getDual(constraints_[item.second]);
    }
  }

  std::pair<std::unordered_map<NodeTime,double,NodeTimeHasher>,std::unordered_map<Edge,double,EdgeHasher>>temp;
  temp.first  = vertexPenalty;
  temp.second = edgePenalty;

  dualSet.first   = agvs_dual;
  dualSet.second  = temp;

  return dualSet;
}

std::vector<std::vector<float>> RMP::getVars() {
  std::vector<std::vector<float>>varSet;
  for(int i=0;i<n_agvs;i++){
    int col = routeSet[i].getSize();
    std::vector<float>temp;
    temp.reserve(col);
    for (int j = 0; j < col; ++j) {
      temp.push_back(cplex.getValue(routeSet[i][j]));
    }
    varSet.push_back(temp);
  }
  return varSet;
}
