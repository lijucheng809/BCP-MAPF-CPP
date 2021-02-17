

#include "ColumnGeneration.h"

void ColumnGeneration::getInitSolution() {
  getInitSoltion = std::make_shared<STA_STAR>(mapSet,n_agvs);
  initSolution = getInitSoltion->solve(agvs_od);
  restrictedMasterProblem = std::make_shared<RMP>(initSolution,n_agvs);
  for(const auto& route:initSolution.first){
    routeSet_dict.insert(route);
  }
}

bool ColumnGeneration::CGsolve() {
  while(true){
    restrictedMasterProblem->buildModel();
    if(!restrictedMasterProblem->solve())
        return false;
    auto dualSet = restrictedMasterProblem->getDual();
    bool allnegative = true;
    for(int i=0;i<n_agvs;i++){
      std::pair<double,
          std::pair<std::unordered_map<NodeTime,double,NodeTimeHasher>,std::unordered_map<Edge,double,EdgeHasher>> >agv_dualSet;
      agv_dualSet.first = dualSet.first[i];
      agv_dualSet.second = dualSet.second;

      Pricerproblem pricerproblem(mapSet,agvs_od[i].first.ID,
                                  agvs_od[i].second.ID,
                                  agv_dualSet);
      pricerproblem.solve();
      auto agv_route_cost_pair = pricerproblem.getRoute();
      if(pricerproblem.isReducedcostNegtive() && (!routeSet_dict.search(agv_route_cost_pair.first))){
        allnegative = false;
        addColumns(i,agv_route_cost_pair.first,agv_route_cost_pair.second);
        routeSet_dict.insert(agv_route_cost_pair.first);
        if(restrictedMasterProblem->branch_strategy && restrictedMasterProblem->branch_strategy->getFractionAgvId() == i){
            auto routeSet = restrictedMasterProblem->outputSolution().first;
            auto latest_agv_route_index = routeSet[i].size()-1;
            restrictedMasterProblem->branch_strategy->newRoutePush2BranchConsLHS(agv_route_cost_pair,latest_agv_route_index);
        }
      }
    }
    if(allnegative)
      break;
  }
  return true;
}
