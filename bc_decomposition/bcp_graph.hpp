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
    Graph g;
    std::pair<int, int> zero_node;
    std::stack<std::pair<std::pair<int, int>, std::pair<int, int>>> edge_stack;
    boost::unordered::unordered_map<std::pair<int, int>, bool> articulation_points;
    boost::unordered::unordered_map<std::pair<int, int>, std::pair<int, int>> parent;
    boost::unordered::unordered_map<std::pair<int, int>, int> low;
    boost::unordered::unordered_map<std::pair<int, int>, int> discover_time;
    int tarjan_time;
    // One node is a single bit, so a biconnected component is multiple bits and can also be represented in a single bitset.
    std::vector<std::vector<std::pair<int, int>>> biconnected_components;
    void tarjan(std::pair<int, int> node);
    
    std::vector<std::pair<int, int>> get_neighbors(std::pair<int, int> state); 
    public:
    BCPGraph(Graph &g);
    std::vector<std::string> optimal_path_calc();
    Graph get_graph();
    std::vector<std::vector<std::pair<int, int>>> get_components(std::pair<int, int> node);
    boost::unordered::unordered_map<std::pair<int, int>, bool> articulation_table();

    void treeify();
    class TreeNode {
        private:
        PacmanGraph pg;
        std::vector<TreeNode> children;
        boost::unordered::unordered_map<boost::dynamic_bitset<>, boost::dynamic_bitset<>> ap_to_foreign_component_map;
        public:
        void compute_children(BCPGraph* b, std::vector<boost::dynamic_bitset<>>* visited_components);
        TreeNode(BCPGraph* b, boost::dynamic_bitset<> node, boost::dynamic_bitset<> curr_component, std::vector<boost::dynamic_bitset<>>* visited_components);
    };

    TreeNode* t;
};

#endif