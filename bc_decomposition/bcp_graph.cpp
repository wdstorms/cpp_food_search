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

const PacmanGraph BCPGraph::TreeNode::pg_init(BCPGraph b, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes, std::pair<int, int> start_node, std::vector<std::pair<int, int>> curr_component) {
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
                        external_node_to_foreign_component_map[{kv.first, n}] = b.get_components(n)[0];
                        external_nodes.insert(kv.first);
                        component_nodes[kv.first].push_back(n);
                        component_nodes[n].push_back(kv.first);
                    }
                }
            }
        }
    }
    Graph g(b.get_graph());
    return PacmanGraph(g, component_nodes, start_node);
}

/**
 * 
 */
BCPGraph::TreeNode::TreeNode(BCPGraph b, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes, std::pair<int, int> start_node, std::vector<std::pair<int, int>> curr_component, std::vector<std::vector<std::pair<int, int>>>* visited_components, std::vector<std::pair<int, int>> parent) : pg(pg_init(b, nodes, start_node, curr_component)) {
    pac_start = start_node;
    parent_component = parent;
    treenode_component = curr_component;
    optimal_cost_last = -1;
    optimal_cost_mid = -1;
    // TODO: Edit pg food to accurately reflect foreign component food presence
    // TODO: Edit pg path_memo to accurately reflect component path length -- best done in path computation
    // Moves out from foreign components will cost 0, but moves into foreign components will cost the exact amount to process them.
    for (auto kv : component_nodes) {
        if (!b.articulation_table()[kv.first] && b.get_components(kv.first)[0] != curr_component) {
            pg.insert_to_path_memo(pg.bit_encode(kv.first), pg.bit_encode(kv.second[0]), 0);
        }
    }

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
    compute_children(b, curr_component, visited_components, child_components, nodes);
    food = false;
    if (pg.get_food() != boost::dynamic_bitset<>(pg.get_food().size(), 0)) {
        food = true;
    }
    for (auto c : children) {
        food |= c.food_in_component();
    }
}

void BCPGraph::TreeNode::compute_children(BCPGraph b, std::vector<std::pair<int, int>> curr_component, std::vector<std::vector<std::pair<int, int>>>* visited_components, std::vector<std::pair<std::pair<int, int>, std::vector<std::pair<int, int>>>> child_components, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes) {
    for (auto c : child_components) {
        children.push_back(TreeNode(b, nodes, c.first, c.second, visited_components, curr_component));
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
    auto v = new std::vector<std::vector<std::pair<int, int>>>();
    t = new TreeNode(*this, g.get_nodes(), start.first, start.second, v, {});
    delete v;
}

bool BCPGraph::TreeNode::food_in_component() {
    return food;
}

std::vector<std::string> BCPGraph::optimal_path_calc() {
    t->optimal_path_calc(true);
    return t->optimal_cost_and_path(true).second;
}

BCPGraph::TreeNode BCPGraph::TreeNode::get_child(std::vector<std::pair<int, int>> component) {
    for (auto c : children) {
        if (component == c.curr_component()) {
            return c;
        }
    }
    return children[0];
}

void BCPGraph::TreeNode::optimal_path_calc(bool last) {
    /**
     * To get the optimal path in the current node, we need to follow these steps:
     * 1) Get optimal paths within all child nodes -- once for when the child is last and once for not
     * 2) Edit path_memo in pg to reflect the exact cost of going to child components
     * 3) Perform A* search on pg twice -- once for when the current component is last and once when it's not 
     * 4) Edit the resultant path to replace child component placeholders with actual child paths
     * 
     * Determining when a component is "last":
     * "Last" means that when all food has been consumed in the component (and its children),
     * Pacman will not have to double back to the parent component to address the parent's other child nodes.
     * Because the root has no parent, the root is always "Last".
     * A component's "Last" path is used if, during an A* search in the parent, the component's connecting node
     * is the last node visited with no outstanding food within the component or other children of the parent.
     */

    if ((last && optimal_cost_last != -1) || (!last && optimal_cost_mid != -1)) {
        return;
    }
    for (auto c : children) {
        c.optimal_path_calc(last);
        c.optimal_path_calc(!last);
        for (auto kv : external_node_to_foreign_component_map) {
            if (kv.second == c.curr_component()) {
                auto s = boost::dynamic_bitset<>(pg.bit_encode(kv.first.first).size(), 1) << (pg.bit_encode(kv.first.first).size() - 1);
                pg.insert_to_path_memo(pg.bit_encode(kv.first.first), pg.bit_encode(kv.first.second), c.optimal_cost_and_path(false).first);
                pg.insert_to_path_memo(pg.bit_encode(kv.first.first), pg.bit_encode(kv.first.second) | s, c.optimal_cost_and_path(false).first);
                break;
            }
        }
    }
    // One last edit to food needs to be done.
    // If this component is last, the node connecting to the parent should not have food.
    // Otherwise, it should have food.
    bool found_parent = false;
    for (auto kv : external_node_to_foreign_component_map) {
        if (kv.second  == parent_component) {
            found_parent = true;
            if (last) {
                    pg.insert_to_path_memo(pg.bit_encode(kv.first.second), pg.bit_encode(kv.first.first), 99999);
                    pg.set_food_bit(pg.bit_encode(kv.first.second), 1);
            }
            else {
                pg.insert_to_path_memo(pg.bit_encode(kv.first.second), pg.bit_encode(kv.first.first), 99999);
                pg.set_food_bit(pg.bit_encode(kv.first.second), 0);
            }
        }
    }
    // assert(found_parent);
    std::cout << "Food: " << pg.get_food() <<  "\n";
    auto pl = astar(pg);
    // std::cout << "Component astar done\n";
    // for (auto i : pl) {
	// 	std::cout << i + ", ";
	// }
	// std::cout << '\n';
	// std::cout << pl.size() << '\n';
    // Edit path to include child paths.
    // Follow the path from the start position
    // When an external node is reached, remove placeholder and insert  child path
    std::pair<int, int> node = pac_start;
    std::pair<int, int> prev;

    boost::unordered::unordered_map<std::string, std::pair<int, int>> dirs;
    dirs["East"] = {0, 1};
    dirs["West"] = {0, -1};
    dirs["South"] = {1, 0};
    dirs["North"] = {-1, 0};
    int i = 0;
    std::vector<std::string> total_path;
    bool skip = false;
    for (auto d : pl) {
        if (skip) {
            skip = false;
            continue;
        }
        prev = node;
        node = {node.first + dirs[d].first, node.second + dirs[d].second};
        if (external_nodes.contains(node)) {
            for (auto cd : get_child(external_node_to_foreign_component_map[{prev, node}]).optimal_cost_and_path(i == (int)pl.size() - 1).second) {
                total_path.push_back(cd);
            }
        }
        else {
            total_path.push_back(d);
        }
        i++;
    }

    if (last) {
        optimal_path_last = total_path;
        optimal_cost_last = total_path.size();
    }
    else {
        optimal_path_mid = total_path;
        optimal_cost_mid = total_path.size();
    }
}

std::pair<int, std::vector<std::string>> BCPGraph::TreeNode::optimal_cost_and_path(bool last) {
    std::pair<int, std::vector<std::string>> l = {optimal_cost_last, optimal_path_last};
    std::pair<int, std::vector<std::string>> m = {optimal_cost_mid, optimal_path_mid};
    return last ? l : m;
}

std::vector<std::pair<int, int>> BCPGraph::TreeNode::curr_component() {
    return treenode_component;
}