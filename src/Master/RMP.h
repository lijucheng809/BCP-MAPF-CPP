
#ifndef MAPF_BCP_MASTER_RMP_H_
#define MAPF_BCP_MASTER_RMP_H_

//#include<ilcplex/ilocplex.h>
#include "../Include//Include.h"
#include "../branchstrategy/BranchStrategy.h"

ILOSTLBEGIN

class RMP {
 private:
  int n_agvs;

  std::vector<std::vector<std::vector<NodeTime>>>agvs_routeSet;
  std::vector<std::vector<double>> agvs_routeSetCost;
  std::unordered_map<Edge,std::vector<std::pair<int,int>>,EdgeHasher>edgeMap;
  std::unordered_map<NodeTime,std::vector<std::pair<int,int>>,NodeTimeHasher>nodetimeMap;

  std::unordered_map<NodeTime,int,NodeTimeHasher>vertexConflits_map;     //key->nodetime,value->index of constriants
  std::unordered_map<Edge,int,EdgeHasher>edgeConflicts_map;              //key->Edge ,value ->index of onstraints

  std::unordered_map<NodeTime,double,NodeTimeHasher>vertexPenalty;
  std::unordered_map<Edge,double,EdgeHasher>edgePenalty;
  std::vector<double>agvs_dual;
  Dual_EdgeVertex_pair dualSet;

  IloEnv env;
  IloRangeArray constraints_;
  NumVarMatrix routeSet;
  IloModel model;
  IloCplex cplex;
  IloObjective objective_;

  IloNumVarArray vars;

 public:
  RMP(const std::pair<std::vector<std::vector<NodeTime>>,
      std::vector<double>>&initSloution,
      int n_agvs);
  ~RMP(){ env.end();}
  void buildModel();
  void addColumns(int agv_id,const std::vector<NodeTime>& route,double cost);
  bool solve();
  bool isOptimal();

  std::shared_ptr<BranchStrategy>branch_strategy;
  std::vector<std::vector<float>>getVars();

  inline Route_Cost_pair outputSolution() const {
      Route_Cost_pair res;
      res.first = agvs_routeSet;
      res.second = agvs_routeSetCost;
      return res;
  };
  inline std::unordered_map<NodeTime,std::vector<std::pair<int,int>>,NodeTimeHasher>getNodeTimeMap() const{
      return nodetimeMap;}
  Dual_EdgeVertex_pair& getDual();

 private:
  inline static Edge getEdge(const NodeTime& n1,const NodeTime& n2){
    Edge edge;
    edge.l1 = std::make_shared<NodeTime>(n1);
    edge.l2 = std::make_shared<NodeTime>(n2);
    return edge;
  }
  void addbranchConstraints();

 public:
  /*void cplextest(){
    int c[3] =  {1,2,3};
    int w1[3] = {-1,1,1};
    int w2[3] = {1,-3,1};
    vars = IloNumVarArray(env,3);
    vars[0] = IloNumVar(env,0,40,ILOFLOAT,"vars_1");
    vars[1] = IloNumVar(env,0,IloInfinity,ILOFLOAT,"vars_2");
    vars[2] = IloNumVar(env,0,IloInfinity,ILOFLOAT,"vars_3");
    IloRangeArray constrains(env);
    model->add(constrains);

    IloExpr obj(env);
    for(int i=0;i<3;i++){
      obj +=c[i]*vars[i];
    }
    model->add(IloMaximize(env,obj));

    //IloRangeArray constrains(env);

    constrains.add(IloRange(env,0,20,"con1"));
    //model->add(constrains);
    IloExpr s1(env);
    for(int i=0;i<3;i++){
      s1 += w1[i]*vars[i];
    }
    constrains[0].setExpr(s1);

    constrains.add(IloRange(env,0,30,"con2"));
    IloExpr s2(env);
    for(int i=0;i<3;i++){
      s2 += w2[i]*vars[i];
    }
    constrains[1].setExpr(s2);
    //model->add(s2<=30);
    model->add(constrains);
    cplex->solve();
    std::cout<<cplex->getStatus()<<std::endl;
    std::cout<<cplex->getDual(constrains[0]);
    cplex->exportModel("model.lp");
};*/
/*  void cplextest(){
    vars = IloNumVarArray(env,3);
    vars[0] = IloNumVar(env,0,IloInfinity,ILOFLOAT);
    vars[1] = IloNumVar(env,0,IloInfinity,ILOFLOAT);
    vars[2] = IloNumVar(env,0,IloInfinity,ILOFLOAT);

    IloExpr obj(env);
    model->add(IloMinimize(env,obj+13*vars[0]+7*vars[1]+13*vars[2]));

    IloRangeArray constrains(env);

    constrains.add(IloRange(env,1,IloInfinity,"con1"));
    IloExpr s1(env);
    s1 += vars[1] + vars[2];
    constrains[0].setExpr(s1);

    constrains.add(IloRange(env,1,IloInfinity,"con2"));
    IloExpr s3(env);
    s3 += vars[0];
    constrains[1].setExpr(s3);

    constrains.add(IloRange(env,0,1,"con3"));
    IloExpr s4(env);
    s4 += vars[0] + vars[1];
    constrains[2].setExpr(s4);

    constrains.add(IloRange(env,0,1,"con4"));
    IloExpr s5(env);
    s5 += vars[1] ;
    constrains[3].setExpr(s5);

    constrains.add(IloRange(env,0,1,"con5"));
    IloExpr s6(env);
    s6 += vars[2] ;
    constrains[4].setExpr(s6);

    constrains.add(IloRange(env,0,1,"con6"));
    IloExpr s7(env);
    s7 += vars[0] + vars[1];
    constrains[5].setExpr(s7);

    model->add(constrains);
    cplex->exportModel("test.lp");
    cplex->solve();

    std::cout<<"obj is "<<cplex->getObjValue()<<std::endl;
    std::cout<<"dual0 is "<<cplex->getDual(constrains[0])<<std::endl;
    std::cout<<"dual1 is "<<cplex->getDual(constrains[1])<<std::endl;
    std::cout<<"dual2 is "<<cplex->getDual(constrains[2])<<std::endl;
    std::cout<<"dual3 is "<<cplex->getDual(constrains[3])<<std::endl;
    std::cout<<"dual4 is "<<cplex->getDual(constrains[4])<<std::endl;
    std::cout<<"dual5 is "<<cplex->getDual(constrains[5])<<std::endl;
  }*/
};

#endif//MAPF_BCP_MASTER_RMP_H_
