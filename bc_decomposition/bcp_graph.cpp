#include "bcp_graph.hpp"
#include <iostream>
#include <algorithm>

std::vector<boost::dynamic_bitset<>> BCPGraph::get_neighbors(boost::dynamic_bitset<> state) {
    return pg.get_neighbors(state);
}

void BCPGraph::tarjan(boost::dynamic_bitset<> node) {
    discover_time[node] = low[node] = tarjan_time;
    tarjan_time += 1;
    int children = 0;
    for (auto n : get_neighbors(node)) {
        if (discover_time[n] == -1) {
            children += 1;
            parent[n] = node;
            edge_stack.push(node | n);
            // std::cout << "Recurse Tarjan\n";
            tarjan(n);
            low[node] = std::min(low[node], low[n]);
            if ((parent[node] == zero_node && children > 1) || (parent[node] != zero_node && low[n] >= discover_time[node])) {
                articulation_points[node] = true;
                boost::dynamic_bitset<> component = boost::dynamic_bitset<>(pg.num_nodes(), 0);
                // std::cout << "edge stack size: " << edge_stack.size() << '\n';
                boost::dynamic_bitset<> e = zero_node;
                while (e != (n | node)) {
                    // std::cout << edge_stack.top() << "\n";
                    component |= edge_stack.top();
                    e = edge_stack.top();
                    edge_stack.pop();
                }
                // std::cout << "component: " << component << '\n';
                biconnected_components.push_back(component);
            }
        }
        else if (n != parent[node]) {
            low[node] = std::min(low[node], low[n]);
        }
    }
}

BCPGraph::BCPGraph(PacmanGraph &g) : pg(g) {
    zero_node = boost::dynamic_bitset<>(pg.num_nodes(), 0);
    int bit = 0;
    tarjan_time = 0;
    std::cout << "BCP Constructor\n";
    while (bit < pg.num_nodes()) {
        boost::dynamic_bitset<> node = boost::dynamic_bitset(pg.num_nodes(), 1) << bit;
        low[node] = -1;
        discover_time[node] = -1;
        articulation_points[node] = false;
        parent[node] = zero_node;
        bit += 1;
    }
    std::cout << "Begin Tarjan\n";
    bit = 0;
    while (bit < pg.num_nodes()) {
        boost::dynamic_bitset<> node = boost::dynamic_bitset(pg.num_nodes(), 1) << bit;
        if (discover_time[node] == -1) {
            std::cout << "bit: " << bit << '\n';
            tarjan(node);
        }
        bit += 1;
    }
    if (!(edge_stack.size() == 0)) {
        boost::dynamic_bitset<> last_component = zero_node;
        std::cout << zero_node << '\n';
        while (!(edge_stack.size() == 0)) {
            last_component |= edge_stack.top();
            edge_stack.pop();
        }
        biconnected_components.push_back(last_component);
    }
    std::cout << biconnected_components.size() << '\n';
    std::cout << "time: " << tarjan_time << '\n';
    for (auto b : biconnected_components) {
        std::cout << b << "\n";
    }
    std::cout << "Remaining edges: " << edge_stack.size() << '\n';
    std::cout << "End Constructor\n";
}