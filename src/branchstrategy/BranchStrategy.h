

#ifndef MAPF_BCP_BRANCHSTRATEGY_H
#define MAPF_BCP_BRANCHSTRATEGY_H
#include "../Include/Include.h"

class BranchStrategy {
public:
    virtual branch_LHS_RHS getBranchConsLHS_RHS() = 0;
    virtual void creatBranchConsLHS_RHS() = 0;
    virtual void newRoutePush2BranchConsLHS(const std::pair<std::vector<NodeTime>,double>& pricerRoute,int routeId) = 0;
    virtual int getFractionAgvId() = 0;
    virtual ~BranchStrategy(){};
};

#endif //MAPF_BCP_BRANCHSTRATEGY_H
