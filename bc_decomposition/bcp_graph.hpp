#ifndef BCLIB_H
#define BCLIB_H

#include <vector>
#include <boost/dynamic_bitset.hpp>
#include "../a*.hpp"
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <stack>
#include <utility>
#include <queue>
class BCPGraph {
    private:
    Graph g;
    boost::unordered::unordered_map<std::vector<std::pair<int, int>>, std::pair<std::vector<std::string>, std::vector<std::string>>> child_paths;
    std::pair<int, int> zero_node;
    std::stack<std::pair<std::pair<int, int>, std::pair<int, int>>> edge_stack;
    boost::unordered::unordered_map<std::pair<int, int>, bool> articulation_points;
    boost::unordered::unordered_map<std::pair<int, int>, std::pair<int, int>> parent;
    boost::unordered::unordered_map<std::pair<int, int>, int> low;
    boost::unordered::unordered_map<std::pair<int, int>, int> discover_time;
    int tarjan_time;
    std::vector<std::vector<std::pair<int, int>>> biconnected_components;
    void tarjan(std::pair<int, int> node);
    boost::unordered::unordered_map<std::pair<std::pair<int, int>, std::pair<int, int>>, int> path_memo;
    std::vector<std::pair<int, int>> get_neighbors(std::pair<int, int> state); 
    public:
    BCPGraph(Graph &g);
    void init_path_memo();
    std::vector<std::string> optimal_path_calc();
    Graph get_graph();
    std::pair<std::pair<int, int>, std::vector<std::pair<int, int>>> get_pac_start();
    std::vector<std::vector<std::pair<int, int>>> get_components(std::pair<int, int> node);
    boost::unordered::unordered_map<std::pair<int, int>, bool> articulation_table();
    std::vector<std::string> biconnected_graph_optimal_path(std::vector<std::pair<int, int>> curr_component, std::vector<std::vector<std::pair<int, int>>> visited_components, std::pair<int, int> start_node, bool last_component, std::vector<std::vector<std::pair<int, int>>> peer_components);
    void treeify();
    std::vector<std::string> total;
    class TreeNode {
        private:
        std::vector<TreeNode*> children;
        // (a_point, treenode connection node), value 
        boost::unordered::unordered_map<std::pair<int, int>, TreeNode*> external_node_to_child_map;
        boost::unordered::unordered_set<std::pair<int, int>> external_nodes;
        bool food;
        TreeNode* parent_tree;
        std::vector<std::pair<int, int>> parent_component;
        std::vector<std::pair<int, int>> treenode_component;
        boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> component_nodes;
        std::vector<std::string> optimal_path_mid;
        int optimal_cost_mid;
        std::vector<std::string> optimal_path_last;
        int optimal_cost_last;
        std::pair<int, int> pac_start;
        public:
        PacmanGraph pg;
        const PacmanGraph pg_init(BCPGraph b, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes, std::pair<int, int> start_node, std::vector<std::pair<int, int>> curr_component);
        bool food_in_component();
        void compute_children(BCPGraph b, std::vector<std::pair<int, int>> curr_component, std::vector<std::vector<std::pair<int, int>>>* visited_components, std::vector<std::pair<std::pair<int, int>, std::vector<std::pair<int, int>>>> child_components, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes);
        TreeNode(BCPGraph b, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes, std::pair<int, int> start_node, std::vector<std::pair<int, int>> curr_component, std::vector<std::vector<std::pair<int, int>>>* visited_components, std::vector<std::pair<int, int>> parent, TreeNode* parent_treenode);
        void optimal_path_calc(bool last, BCPGraph* b);
        std::pair<int, std::vector<std::string>> optimal_cost_and_path(bool last);
        std::vector<std::pair<int, int>> curr_component();
        TreeNode* get_child(std::vector<std::pair<int, int>> component);
    };

    TreeNode* t;
};

#endif