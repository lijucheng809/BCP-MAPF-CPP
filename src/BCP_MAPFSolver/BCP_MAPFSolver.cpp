
#include "BCP_MAPFSolver.h"

BCP_MAPFSolver::BCP_MAPFSolver(const std::vector<Point> &mapSet,
                               const std::vector<std::pair<Point, Point>> &agvs_od,
                               int n_agvs)
    :n_agvs(n_agvs),
     CG_(mapSet,agvs_od,n_agvs){}

bool BCP_MAPFSolver::isIntegerSolution(std::vector<std::vector<float>> vars) {
  for(int i=0;i<vars.size();i++){
    for(int j=0;j<vars[i].size();j++){
      if(vars[i][j]>0 && vars[i][j]<1){
        return false;
      }
    }
  }
  return true;
}

int BCP_MAPFSolver::SolutionCost(ColumnGeneration &currentModel) {
    auto vars = currentModel.getVars();
    auto agvRouteCost = currentModel.solution().second;
    int totalCost = 0;
    for(int i=0;i<vars.size();i++){
        for(int j=0;j<vars[i].size();j++){
            if(vars[i][j]<=1.0 && vars[i][j]>0){
                totalCost += agvRouteCost[i][j];
            }
        }
    }
    return totalCost;
}

void BCP_MAPFSolver::pushChildnode2FractionalSolutionSet(const std::shared_ptr<ColumnGeneration>& currentModel) {
    if(currentModel->CGsolve()){
        auto vars = currentModel->getVars();
        auto totalCost = SolutionCost(*currentModel);
        if(isIntegerSolution(vars)){
            if(totalCost < up_bound_cost){
                opt_model = currentModel;
                up_bound_cost = totalCost;
            }
        }else{
            if(totalCost < up_bound_cost)
                fractionalSolutionSet.push(currentModel);
        }
    }
}
void BCP_MAPFSolver::branch_on_path_length(ColumnGeneration &currentModel,
                                           const std::pair<int, std::vector<int>> &agvId_agvRouteId_pair,
                                           const std::vector<std::vector<double>>& routSetCost) {
  int agvid = agvId_agvRouteId_pair.first;
  std::vector<int>agvRoutId = agvId_agvRouteId_pair.second;
  int min_agvRouteCost = INT_MAX;
  for(int i=0;i<agvRoutId.size();i++){
    if(min_agvRouteCost > routSetCost[agvid][agvRoutId[i]])
      min_agvRouteCost = routSetCost[agvid][agvRoutId[i]];
  }
  /*left child node */
  std::shared_ptr<ColumnGeneration>leftChildModel = std::make_shared<ColumnGeneration>(currentModel);
  leftChildModel->restrictedMasterProblem->branch_strategy = std::make_shared<BranchOnLength>(currentModel.solution(),min_agvRouteCost,true,agvid);
  leftChildModel->restrictedMasterProblem->branch_strategy->creatBranchConsLHS_RHS();
  pushChildnode2FractionalSolutionSet(leftChildModel);
  /*if(leftChildModel->CGsolve()){
      auto vars = leftChildModel->getVars();
      auto totalCost = SolutionCost(*leftChildModel);
      if(isIntegerSolution(vars)){
          if(totalCost < up_bound_cost){
              opt_model = leftChildModel;
              up_bound_cost = totalCost;
          }
      }else{
          if(totalCost < up_bound_cost)
            fractionalSolutionSet.push(leftChildModel);
      }
  }*/

  /*right child node*/
  std::shared_ptr<ColumnGeneration>rightChildModel = std::make_shared<ColumnGeneration>(currentModel);
  rightChildModel->restrictedMasterProblem->branch_strategy = std::make_shared<BranchOnLength>(currentModel.solution(),min_agvRouteCost,false,agvid);
  rightChildModel->restrictedMasterProblem->branch_strategy->creatBranchConsLHS_RHS();
  pushChildnode2FractionalSolutionSet(rightChildModel);
}
void BCP_MAPFSolver::branch_on_vertices(ColumnGeneration &currentModel,
                                        const std::vector<std::vector<double>>& routSetCost) {
    auto nodetimeMap = currentModel.restrictedMasterProblem->getNodeTimeMap();
    auto vars = currentModel.restrictedMasterProblem->getVars();
    int min_time = INT_MAX;
    NodeTime *branch_nt;
    for(const auto& ntmap:nodetimeMap){
        if(ntmap.first.time<min_time) {
            for (const auto &agv_route_pair :ntmap.second) {
                if(vars[agv_route_pair.first][agv_route_pair.second]>0 &&
                   vars[agv_route_pair.first][agv_route_pair.second]<1){
                    branch_nt = new NodeTime(ntmap.first);
                    break;
                }
            }
        }
    }
    int agv_id;
    int min_routeCost = INT_MAX;
    if(branch_nt){
        for(const auto&agv_route_pair :nodetimeMap.at(*branch_nt)){
            if(routSetCost[agv_route_pair.first][agv_route_pair.second]<min_routeCost){
                agv_id = agv_route_pair.first;
            }
        }
    }
    /*left child node */
    std::shared_ptr<ColumnGeneration>leftChildModel = std::make_shared<ColumnGeneration>(currentModel);
    leftChildModel->restrictedMasterProblem->branch_strategy = std::make_shared<BranchOnVertex>(currentModel.solution(),*branch_nt,true,agv_id);
    leftChildModel->restrictedMasterProblem->branch_strategy->creatBranchConsLHS_RHS();
    pushChildnode2FractionalSolutionSet(leftChildModel);
    /*right child node */
    std::shared_ptr<ColumnGeneration>rightChildModel = std::make_shared<ColumnGeneration>(currentModel);
    rightChildModel->restrictedMasterProblem->branch_strategy = std::make_shared<BranchOnVertex>(currentModel.solution(),*branch_nt,false,agv_id);
    rightChildModel->restrictedMasterProblem->branch_strategy->creatBranchConsLHS_RHS();
    pushChildnode2FractionalSolutionSet(rightChildModel);

}

void BCP_MAPFSolver::branch(ColumnGeneration &currentModel,const std::vector<std::vector<float>>& vars){
  auto route_cost_pair = currentModel.solution();
  auto routeSet = route_cost_pair.first;
  auto routeSet_Cost = route_cost_pair.second;
  std::pair<int,std::vector<int>>agvId_agvRouteId_pair;
  bool findFractionalSolution = false;
  bool isbranch_on_vertices = true;
  for(int i=0;i<n_agvs;i++){
    for(int j=0;j<vars[i].size();j++){
      if(vars[i][j]>0 && vars[i][j]<1){
        findFractionalSolution = true;
        agvId_agvRouteId_pair.first = i;
        if(!agvId_agvRouteId_pair.second.empty() &&
            routeSet_Cost[i][agvId_agvRouteId_pair.second.size()-1] != routeSet_Cost[i][j]){
          isbranch_on_vertices = false;
        }
        agvId_agvRouteId_pair.second.push_back(j);
      }
    }
    if(findFractionalSolution) break;
  }
  if(findFractionalSolution && isbranch_on_vertices){
    branch_on_vertices(currentModel,routeSet_Cost);
  }else if(findFractionalSolution && !isbranch_on_vertices){
    branch_on_path_length(currentModel,agvId_agvRouteId_pair,routeSet_Cost);
  }
}

void BCP_MAPFSolver::solve() {
  CG_.getInitSolution();
  CG_.CGsolve();
  auto vars = CG_.getVars();
  if(isIntegerSolution(vars)){
    opt_model = std::make_shared<ColumnGeneration>(CG_);
  }else{
    std::shared_ptr<ColumnGeneration>fractionnode = std::make_shared<ColumnGeneration>(CG_);
    fractionalSolutionSet.push(fractionnode);
  }
  while(!fractionalSolutionSet.empty()){

    auto current_model = fractionalSolutionSet.front();
    branch(*current_model,vars);
    fractionalSolutionSet.pop();
  }

}
