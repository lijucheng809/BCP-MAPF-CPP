

#ifndef MAPF_BCP_INCLUDE_TRIE_H_
#define MAPF_BCP_INCLUDE_TRIE_H_
#include "Include.h"

class Trie {
 private:
  TrieNode* root;

 public:
  static int Max;
  Trie(){root = new TrieNode();}
  void insert(const std::vector<NodeTime>& route);
  bool search(const std::vector<NodeTime>& route);
};

#endif//MAPF_BCP_INCLUDE_TRIE_H_
