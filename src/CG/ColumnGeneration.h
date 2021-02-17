

#ifndef MAPF_BCP_CG_COLUMNGENERATION_H_
#define MAPF_BCP_CG_COLUMNGENERATION_H_


#include "../Include/Include.h"
#include "../STA_STAR/STA_STAR.h"
#include "../Master/RMP.h"
#include "../pricer/Pricerproblem.h"
#include "../Include/Trie.h"
#include "../branchstrategy/BranchStrategy.h"

class ColumnGeneration {
 private:

  int n_agvs;

  std::shared_ptr<STA_STAR>getInitSoltion;
  //std::shared_ptr<RMP>restrictedMasterProblem;

  std::pair<std::vector<std::vector<NodeTime>>,std::vector<double>>initSolution;
  std::vector<Point> mapSet;
  std::vector<std::pair<Point, Point>>agvs_od;

  Trie routeSet_dict;

 public:

  ColumnGeneration(const std::vector<Point>& mapSet,
                   const std::vector<std::pair<Point, Point>>& agvs_od,
                   int n_agvs)

       :mapSet(mapSet),
        agvs_od(agvs_od),
        n_agvs(n_agvs),
        routeSet_dict(){}

  std::shared_ptr<RMP>restrictedMasterProblem;

  void getInitSolution();
  bool CGsolve();

  std::vector<std::vector<float>> getVars(){
    auto varSet = restrictedMasterProblem->getVars();
    for(int i=0;i<varSet.size();i++){
      for(int j=0;j<varSet[i].size();j++){
        std::cout<<"agv "<<i<<" route "<<j<<" is:"<<varSet[i][j]<<std::endl;
      }
    }
    return varSet;
  }

  inline Route_Cost_pair solution() const{
    return restrictedMasterProblem->outputSolution();
  }

 private:

  void buildModel(){
    restrictedMasterProblem->buildModel();
  }

  void addColumns(int agv_id,const std::vector<NodeTime>& route,double cost){
    restrictedMasterProblem->addColumns(agv_id,route,cost);
  }
};

#endif//MAPF_BCP_CG_COLUMNGENERATION_H_
