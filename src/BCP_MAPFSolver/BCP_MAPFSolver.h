
#ifndef MAPF_BCP_BCP_MAPFSOLVER_BCP_MAPFSOLVER_H_
#define MAPF_BCP_BCP_MAPFSOLVER_BCP_MAPFSOLVER_H_

#include "../Include/Include.h"
#include "../CG/ColumnGeneration.h"
#include "../branchstrategy/BranchStrategy.h"
#include "../branchstrategy/BranchOnLength.h"
#include "../branchstrategy/BranchOnVertex.h"

class BCP_MAPFSolver {
 private:
  std::queue<std::shared_ptr<ColumnGeneration>> fractionalSolutionSet;
  ColumnGeneration CG_;
  std::vector<Point>mapSet;
  std::vector<std::pair<Point, Point>>agvs_od;
  int up_bound_cost{INT_MAX};
  std::shared_ptr<ColumnGeneration>opt_model;
  int n_agvs;

 public:
  BCP_MAPFSolver(const std::vector<Point>& mapSet,
                 const std::vector<std::pair<Point, Point>>& agvs_od,
                 int n_agvs);
  ~BCP_MAPFSolver() = default;
  void solve();

 private:
  static bool isIntegerSolution(std::vector<std::vector<float>>vars);
  void branch_on_path_length(ColumnGeneration &currentModel,
                             const std::pair<int,std::vector<int>>& agvId_agvRouteId_pair,
                             const std::vector<std::vector<double>>& routSetCost);
  void branch_on_vertices(ColumnGeneration &currentModel,
                          const std::vector<std::vector<double>>& routSetCost);
  void branch(ColumnGeneration& currentModel,const std::vector<std::vector<float>>& vars);
  static int SolutionCost(ColumnGeneration &currentModel);
  void pushChildnode2FractionalSolutionSet(const std::shared_ptr<ColumnGeneration>&currentModel);
  
};


#endif//MAPF_BCP_BCP_MAPFSOLVER_BCP_MAPFSOLVER_H_
