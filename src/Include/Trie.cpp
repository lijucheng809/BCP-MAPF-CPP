

#include "Trie.h"

int Trie::Max = 444;

void Trie::insert(const std::vector<NodeTime>& route) {
  TrieNode* nt = root;
  for(auto i : route){
    int index = i.node_id + Max*i.time;
    //int index = (i.node_id+1) * (i.time+1);
    if(nt->next[index] == nullptr){
      auto temp = new TrieNode();
      nt->next[index] = temp;
    }
    nt =  nt->next[index];
  }
  nt->isRoute = true;
}

bool Trie::search(const std::vector<NodeTime>& route) {
  TrieNode* nt = root;
  for(int i=0;i<route.size()&&nt;i++)
  {
    int index = route[i].node_id + Max*route[i].time;
    //int index = (route[i].time+1) * (route[i].node_id+1);
    nt = nt->next[index];
  }
  return (nt != nullptr && nt->isRoute);
}
