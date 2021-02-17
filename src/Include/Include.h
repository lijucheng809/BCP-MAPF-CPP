

#ifndef MAPF_BCP_INCLUDE_INCLUDE_H_
#define MAPF_BCP_INCLUDE_INCLUDE_H_
#include <boost/thread/thread.hpp>
#include<ilcplex/ilocplex.h>

#include<vector>
#include <unordered_set>
#include <unordered_map>

typedef int Direction;
typedef bool Turnaround;
typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<NumVarMatrix> NumVar3Matrix;

struct NodeTime{
  int time;
  int node_id;
  int nt_id;
  NodeTime(int time,int node_id,int nt_id):time(time),node_id(node_id),nt_id(nt_id){}
  bool operator==(const NodeTime& nt)const{
    //return time == nt.time && node_id == nt.node_id && nt_id == nt.nt_id;
    return time == nt.time && node_id == nt.node_id;
  }
  bool operator<(const NodeTime& nt) const{
    if(node_id < nt.node_id) return true;
    if(node_id > nt.node_id) return false;
    return time < nt.time;
    //return nt_id <= nt.nt_id;
  }
};

struct NodeTimeHasher{
  std::size_t operator()(const NodeTime& nt)const{
    std::size_t seed = 0;
    boost::hash_combine(seed,boost::hash_value(nt.node_id));
    boost::hash_combine(seed,boost::hash_value(nt.time));
    return seed;
  }
};

struct Point{
  double x;
  double y;
  int ID;
  bool obstacle;
  struct {
    Point* UP = nullptr;
    Point* DOWN= nullptr;
    Point* LEFT = nullptr;
    Point* RIGHT = nullptr;
  };
  Point(double x,double y,int id,bool obstacle):x(x),y(y),ID(id),obstacle(obstacle){}
  bool operator==(const Point& p) const{
    return x == p.x && y == p.y && ID == p.ID;
  }
};

struct PointHasher{
  std::size_t operator()(const Point& pt)const{
    std::size_t seed = 0;
    boost::hash_combine(seed,boost::hash_value(pt.x));
    boost::hash_combine(seed,boost::hash_value(pt.y));
    boost::hash_combine(seed,boost::hash_value(pt.ID));
    return seed;
  }
};

struct Edge{
  std::shared_ptr<NodeTime>l1;
  std::shared_ptr<NodeTime>l2;

  bool operator==(const Edge& edge) const {
    return l1->node_id == edge.l1->node_id &&
           l2->node_id == edge.l2->node_id &&
           l1->time == edge.l1->time &&
           l2->time == edge.l2->time;
  }

  bool operator<(const Edge& edge)const{
    return edge.l1->nt_id <= l1->nt_id ;
  }
};

struct EdgeHasher{
  std::size_t operator()(const Edge& edge)const{
    std::size_t seed = 0;
    boost::hash_combine(seed,boost::hash_value(edge.l1->time));
    boost::hash_combine(seed,boost::hash_value(edge.l2->time));
    boost::hash_combine(seed,boost::hash_value(edge.l1->node_id));
    boost::hash_combine(seed,boost::hash_value(edge.l2->node_id));
    return seed;
  }
};

struct label{
  double g;
  double h;
  double real_distance = 0;
  Direction d = -1;      //初始值为-1,1->up 2->down 4->left 5->right
  Turnaround tr = false; //转向
  NodeTime nt;
  std::shared_ptr<label>parent;
  label(const NodeTime& nt,double g,double h):nt(nt),g(g),h(h){}
};

struct compare{
  bool operator()(const label& a,const label& b){
    //return(a.h+a.g >= b.h+b.g);
    return(a.h+a.g > b.h+b.g||
        (a.h+a.g == b.h+b.g && a.g < b.g)||
        (a.h+a.g == b.h+b.g && a.h > b.h));
  }
};

struct TrieNode{

  bool isRoute;
  TrieNode* next[1000000];
  TrieNode():isRoute(false){
    std::memset(next,NULL,sizeof(next));
  }

};

using Route_Cost_pair = std::pair<std::vector<std::vector<std::vector<NodeTime>>>,std::vector<std::vector<double>>>;
using Dual_EdgeVertex_pair = std::pair<std::vector<double>,
                          std::pair<std::unordered_map<NodeTime,double,NodeTimeHasher>,
                                    std::unordered_map<Edge,double,EdgeHasher>>>;
using branch_LHS_RHS = std::pair<std::vector<std::pair<int,int>>,int>;

#endif//MAPF_BCP_INCLUDE_INCLUDE_H_
