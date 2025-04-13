#ifndef BCLIB_H
#define BCLIB_H

#include <vector>
#include <boost/dynamic_bitset.hpp>
#include "../pacman_graph.hpp"
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <stack>
#include <utility>
#include <queue>
class BCPGraph {
    private:
    PacmanGraph pg;
    boost::dynamic_bitset<> zero_node;
    std::stack<boost::dynamic_bitset<>> edge_stack;
    boost::unordered::unordered_map<boost::dynamic_bitset<>, bool> articulation_points;
    boost::unordered::unordered_map<boost::dynamic_bitset<>, boost::dynamic_bitset<>> parent;
    boost::unordered::unordered_map<boost::dynamic_bitset<>, int> low;
    boost::unordered::unordered_map<boost::dynamic_bitset<>, int> discover_time;
    int tarjan_time;
    // One node is a single bit, so a biconnected component is multiple bits and can also be represented in a single bitset.
    std::vector<boost::dynamic_bitset<>> biconnected_components;
    void tarjan(boost::dynamic_bitset<> node);
    
    std::vector<boost::dynamic_bitset<>> get_neighbors(boost::dynamic_bitset<> state); 
    public:
    BCPGraph(PacmanGraph &pg);
    std::vector<std::string> optimal_path_calc();
    PacmanGraph get_pacgraph();
    std::vector<boost::dynamic_bitset<>> get_components(boost::dynamic_bitset<> node);
    boost::unordered::unordered_map<boost::dynamic_bitset<>, bool> articulation_table();

    void treeify();
    class TreeNode {
        private:
        PacmanGraph pg;
        std::vector<TreeNode> children;
        public:
        std::vector<std::pair<boost::dynamic_bitset<>, std::vector<std::string>>> compute_children();
        TreeNode(BCPGraph b, boost::dynamic_bitset<> node);
        TreeNode(BCPGraph b, boost::dynamic_bitset<> node, std::vector<boost::dynamic_bitset<>>* visited_components);
    };

    TreeNode* t;
};

#endif