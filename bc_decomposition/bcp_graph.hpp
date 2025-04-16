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
    std::pair<std::pair<int, int>, std::vector<std::pair<int, int>>> get_pac_start();
    std::vector<std::vector<std::pair<int, int>>> get_components(std::pair<int, int> node);
    boost::unordered::unordered_map<std::pair<int, int>, bool> articulation_table();

    void treeify();
    class TreeNode {
        private:
        PacmanGraph pg;
        std::vector<TreeNode> children;
        boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> ap_to_foreign_component_map;
        bool food;
        std::vector<std::pair<int, int>> parent_component;
        public:
        bool food_in_component();
        void compute_children(BCPGraph b, std::vector<std::pair<int, int>> curr_component, std::vector<std::vector<std::pair<int, int>>>* visited_components, std::vector<std::pair<std::pair<int, int>, std::vector<std::pair<int, int>>>> child_components, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes);
        TreeNode(BCPGraph b, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes, std::pair<int, int> start_node, std::vector<std::pair<int, int>> curr_component, std::vector<std::vector<std::pair<int, int>>>* visited_components, std::vector<std::pair<int, int>> parent);
        std::vector<std::string> optimal_path_calc();
    };

    TreeNode* t;
};

#endif