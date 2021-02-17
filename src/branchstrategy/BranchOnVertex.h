
#ifndef MAPF_BCP_BRANCHONVERTEX_H
#define MAPF_BCP_BRANCHONVERTEX_H
#include "BranchStrategy.h"
#include "../Include/Include.h"

class BranchOnVertex : public BranchStrategy{
private:
    Route_Cost_pair routeCostPair;
    branch_LHS_RHS LHS_RHS;
    NodeTime branchnt;
    bool choosenode;
    int fraction_agvId;
public:
    BranchOnVertex(const Route_Cost_pair& rcp,
                   NodeTime branchnt,
                   bool choosenode,
                   int frac_agvId):
                   routeCostPair(rcp),
                   branchnt(branchnt),
                   choosenode(choosenode),
                   fraction_agvId(frac_agvId){}
    void creatBranchConsLHS_RHS() override;
    inline branch_LHS_RHS getBranchConsLHS_RHS() override {
        return LHS_RHS;
    }
    void newRoutePush2BranchConsLHS(const std::pair<std::vector<NodeTime>, double> &pricerRoute, int routeId) override;
    inline int getFractionAgvId() override{return fraction_agvId;}
    ~BranchOnVertex() override = default;
};


#endif //MAPF_BCP_BRANCHONVERTEX_H
