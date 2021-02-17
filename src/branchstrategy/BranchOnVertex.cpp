

#include "BranchOnVertex.h"

void BranchOnVertex::creatBranchConsLHS_RHS() {
    if(choosenode)
        LHS_RHS.second = 1;
    else
        LHS_RHS.second = 0;
    for(int i=0;i<routeCostPair.first[fraction_agvId].size();i++){
        for(int j=0;j<routeCostPair.first[fraction_agvId][i].size();j++){
            if(routeCostPair.first[fraction_agvId][i][j]==branchnt){
                LHS_RHS.first.emplace_back(fraction_agvId,i);
                break;
            }
        }
    }
}
void BranchOnVertex::newRoutePush2BranchConsLHS(const std::pair<std::vector<NodeTime>, double> &pricerRoute,
                                                int routeId) {
    for(const auto&nt:pricerRoute.first){
        if(nt==branchnt){
            LHS_RHS.first.emplace_back(fraction_agvId,routeId);
            break;
        }
    }
}
