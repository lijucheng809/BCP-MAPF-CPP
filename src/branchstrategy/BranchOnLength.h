

#ifndef MAPF_BCP_BRANCHONLENGTH_H
#define MAPF_BCP_BRANCHONLENGTH_H
#include "BranchStrategy.h"
#include "../Include/Include.h"

class BranchOnLength : public BranchStrategy{
public:
    BranchOnLength(const Route_Cost_pair& rcp,
                   int min_agvRouteCost,
                   bool choosenode,
                   int frac_agvId)
    :routeCostPair(rcp),
    min_agvRouteCost(min_agvRouteCost),
    choosenode(choosenode),
    fraction_agvId(frac_agvId){};
    inline branch_LHS_RHS getBranchConsLHS_RHS() override{
        return LHS_RHS;
    }
    void newRoutePush2BranchConsLHS(const std::pair<std::vector<NodeTime>,double> &pricerRoute,int routeId) override;
    void creatBranchConsLHS_RHS() override;
    inline int getFractionAgvId() override{return fraction_agvId;}
    ~BranchOnLength() override = default;
private:
    Route_Cost_pair routeCostPair;
    branch_LHS_RHS LHS_RHS;
    int min_agvRouteCost;
    bool choosenode;
    int fraction_agvId;
};


#endif //MAPF_BCP_BRANCHONLENGTH_H
