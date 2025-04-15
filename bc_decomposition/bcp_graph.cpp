#include "bcp_graph.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>

std::vector<std::pair<int, int>> BCPGraph::get_neighbors(std::pair<int, int> state) {
    auto nodes = g.get_nodes();
    return nodes[state];
}

void BCPGraph::tarjan(std::pair<int, int> node) {
    discover_time[node] = low[node] = tarjan_time;
    tarjan_time += 1;
    int children = 0;
    for (auto n : get_neighbors(node)) {
        if (discover_time[n] == -1) {
            children += 1;
            parent[n] = node;
            edge_stack.push({node, n});
            tarjan(n);
            low[node] = std::min(low[node], low[n]);
            if ((parent[node] == zero_node && children > 1) || (parent[node] != zero_node && low[n] >= discover_time[node])) {
                articulation_points[node] = true;
                boost::unordered::unordered_set<std::pair<int, int>> component;
                // std::cout << "edge stack size: " << edge_stack.size() << '\n';
                std::pair<std::pair<int, int>, std::pair<int, int>> e = {{-1, -1}, {-1, -1}};
                while (e != (std::pair<std::pair<int, int>, std::pair<int, int>> ){node, n}) {
                    // // std::cout << edge_stack.top() << "\n";
                    component.insert(edge_stack.top().first);
                    component.insert(edge_stack.top().second);
                    e = edge_stack.top();
                    edge_stack.pop();
                }
                // // std::cout << "component: " << component << '\n';
                std::vector<std::pair<int, int>> cv;
                for (auto k : component) {
                    cv.push_back(k);
                }
                biconnected_components.push_back(cv);
            }
        }
        else if (n != parent[node]) {
            low[node] = std::min(low[node], low[n]);
        }
    }
}

BCPGraph::BCPGraph(Graph &gr) {
    g = gr;
    tarjan_time = 0;
    // std::cout << "BCP Constructor\n";
    for (auto n : g.get_nodes()) {
        auto node = n.first;
        low[node] = -1;
        discover_time[node] = -1;
        articulation_points[node] = false;
        parent[node] = zero_node;
    }
    // std::cout << "Begin Tarjan\n";
    for (auto n : g.get_nodes()) {
        auto node = n.first;
        if (discover_time[node] == -1) {
            tarjan(node);
        }
    }
    // std::cout << "End Tarjan\n";
    if (!(edge_stack.size() == 0)) {
        // std::cout << "Last Component\n";
        boost::unordered::unordered_set<std::pair<int, int>> last_component;
        // std::cout << zero_node << '\n';
        while (!(edge_stack.size() == 0)) {
            last_component.insert(edge_stack.top().first);
            last_component.insert(edge_stack.top().second);
            edge_stack.pop();
        }
         std::vector<std::pair<int, int>> cv;
            for (auto k : last_component) {
                cv.push_back(k);
            }
        biconnected_components.push_back(cv);
    }
    // std::cout << biconnected_components.size() << "\n";
    // for (auto k : biconnected_components) {
    //     for (auto v : k) {
    //         std::cout << v.first << ", " << v.second << "; ";
    //     }
    //     std::cout << "\n";
    // }
}

boost::unordered::unordered_map<std::pair<int, int>, bool> BCPGraph::articulation_table() {
    return articulation_points;
}

Graph BCPGraph::get_graph() {
    return g;
}

std::vector<std::vector<std::pair<int, int>>> BCPGraph::get_components(std::pair<int, int> node) {
    std::vector<std::vector<std::pair<int, int>>> components;
    for (auto c : biconnected_components) {
        if (std::find(c.begin(), c.end(), node) != c.end()) {
            components.push_back(c);
        }
    }
    return components;
}

bool node_within_component(boost::dynamic_bitset<> n, boost::dynamic_bitset<> c) {
    return (n & c) != boost::dynamic_bitset<>(n.size(), 0);
}

BCPGraph::TreeNode::TreeNode(BCPGraph* b, boost::dynamic_bitset<> start_node, boost::dynamic_bitset<> curr_component, std::vector<boost::dynamic_bitset<>>* visited_components) {
   
}

void BCPGraph::TreeNode::compute_children(BCPGraph* b, std::vector<boost::dynamic_bitset<>>* visited_components) {
    
}

void BCPGraph::treeify() {
    
}