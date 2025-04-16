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

/**
 * 
 */
BCPGraph::TreeNode::TreeNode(BCPGraph b, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes, std::pair<int, int> start_node, std::vector<std::pair<int, int>> curr_component, std::vector<std::vector<std::pair<int, int>>>* visited_components) {
    boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> component_nodes;
    // TODO: Include one node from each foreign component that's adjacent to the current component to represent TreeNode connections.
    for (auto kv : nodes) {
        if (std::find(curr_component.begin(), curr_component.end(), kv.first) != curr_component.end()) {
            for (auto n : kv.second) {
                if (std::find(curr_component.begin(), curr_component.end(), n) != curr_component.end()) {
                    component_nodes[kv.first].push_back(n);
                }
            }
            if (b.articulation_table()[kv.first]) {
                for (auto n : b.get_neighbors(kv.first)) {
                    if (std::find(curr_component.begin(), curr_component.end(), n) == curr_component.end()) {
                        component_nodes[kv.first].push_back(n);
                        component_nodes[n].push_back(kv.first);
                    }
                }
            }
        }
    }
    pg = PacmanGraph(b.get_graph(), component_nodes, start_node);
    // TODO: Edit pg food to accurately reflect foreign component food presence
    if (pg.get_food() != boost::dynamic_bitset<>(pg.get_food().size(), 0)) {
        food = true;
    }
    // TODO: Edit pg path_memo to accurately reflect component path length -- best done in path computation
    std::vector<std::pair<std::pair<int, int>, std::vector<std::pair<int, int>>>> child_components;
    for (auto n : curr_component) {
        if (b.articulation_table()[n]) {
            for (auto c : b.get_components(n)) {
                if (std::find(visited_components->begin(), visited_components->end(), c) != visited_components->end()) {
                    visited_components->push_back(c);
                    child_components.push_back({n, c});
                }
            }
        }
    }
    compute_children(b, visited_components, child_components, nodes);
}

void BCPGraph::TreeNode::compute_children(BCPGraph b, std::vector<std::vector<std::pair<int, int>>>* visited_components, std::vector<std::pair<std::pair<int, int>, std::vector<std::pair<int, int>>>> child_components, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes) {
    for (auto c : child_components) {
        children.push_back(TreeNode(b, nodes, c.first, c.second, visited_components));
    }
}

std::pair<std::pair<int, int>, std::vector<std::pair<int, int>>> BCPGraph::get_pac_start() {
    auto matrix = g.matrix();
    for (auto i = 0; i < (int)matrix.size(); i++) {
        for (auto j = 0; j < (int)matrix[0].size(); j++) {
            if (matrix[i][j] == 3) {
                return {{i, j}, get_components({i, j})[0]};
            }
        }
    }
    return {{-1, -1}, {}};
}

void BCPGraph::treeify() {
    auto start = get_pac_start();
    t = new TreeNode(*this, g.get_nodes(), start.first, start.second, new std::vector<std::vector<std::pair<int, int>>>());
}

bool BCPGraph::TreeNode::food_in_component() {
    return food;
}