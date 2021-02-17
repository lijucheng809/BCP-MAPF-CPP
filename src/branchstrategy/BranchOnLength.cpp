

#include "BranchOnLength.h"
void BranchOnLength::creatBranchConsLHS_RHS() {
    if(choosenode)
        LHS_RHS.second = 1;
    else
        LHS_RHS.second = 0;
    for(int i=0;i<routeCostPair.second[fraction_agvId].size();i++){
        if(routeCostPair.second[fraction_agvId][i]<=min_agvRouteCost && choosenode)
            LHS_RHS.first.emplace_back(fraction_agvId,i);
        else if(routeCostPair.second[fraction_agvId][i]>min_agvRouteCost && (!choosenode))
            LHS_RHS.first.emplace_back(fraction_agvId,i);
    }
}
void BranchOnLength::newRoutePush2BranchConsLHS(const std::pair<std::vector<NodeTime>,double> &pricerRoute,
                                                int routeId) {
    if(pricerRoute.second<=min_agvRouteCost && choosenode)
        LHS_RHS.first.emplace_back(fraction_agvId,routeId);
    else if(pricerRoute.second>min_agvRouteCost && (!choosenode))
        LHS_RHS.first.emplace_back(fraction_agvId,routeId);
}
