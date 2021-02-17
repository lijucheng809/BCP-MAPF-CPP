#include <unordered_set>
#include <set>
#include <unordered_map>
#include <map>

#include "Include/Include.h"
#include "Map/Map.h"

#include "Master/RMP.h"
#include "pricer/Pricerproblem.h"
#include "CG/ColumnGeneration.h"
#include "Include/Trie.h"
#include "BCP_MAPFSolver/BCP_MAPFSolver.h"


int main() {

    std::string file = "/Users/lijucheng/Desktop/MAPF_BCP/Map/res.txt";
    Map map(file,445);
    std::vector<Point>mapset =map.getmap();

    /*测试集：110-113,107-120*/
    STA_STAR sta_star(mapset,2);
    std::pair<Point,Point>agv(mapset[49],mapset[164]);
    std::vector<std::pair<Point, Point>>agvs;
    std::pair<Point,Point>agv1(mapset[189],mapset[74]);
    agvs.push_back(agv);
    agvs.push_back(agv1);

    BCP_MAPFSolver bcpsolver(mapset,agvs,2);
    bcpsolver.solve();

    return 0;

}
