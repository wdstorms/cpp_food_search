#include "bcp_graph.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>

std::vector<std::pair<int, int>> BCPGraph::get_neighbors(std::pair<int, int> state) {
    auto nodes = g.get_nodes();
    return nodes[state];
}

void BCPGraph::init_path_memo() {
    for (auto n : g.get_nodes()) {
        int depth = 0;
        auto visited = boost::unordered::unordered_set<std::pair<int ,int>>();
        auto q = std::queue<std::pair<int ,int>>();
        q.push(n.first);
        while (!(q.size() == 0)) {
            int level_size = q.size();
            while (level_size != 0 && !(q.size() == 0)) {
                auto state = q.front();
                q.pop();
                if (visited.contains(state)) {
                    continue;
                }
                visited.insert(state);
                if (depth > 0) {
                    path_memo[{n.first, state}] = depth;
                }
                for (auto node : get_neighbors(state)) {
                    if (!visited.contains(node)) {
                        q.push(node);
                    }
                }
                level_size -= 1;
            }
            depth += 1;
        }
    }
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
                // // std::cout << "edge stack size: " << edge_stack.size() << '\n';
                std::pair<std::pair<int, int>, std::pair<int, int>> e = {{-1, -1}, {-1, -1}};
                while (e != (std::pair<std::pair<int, int>, std::pair<int, int>> ){node, n}) {
                    // // // std::cout << edge_stack.top() << "\n";
                    component.insert(edge_stack.top().first);
                    component.insert(edge_stack.top().second);
                    e = edge_stack.top();
                    edge_stack.pop();
                }
                // // // std::cout << "component: " << component << '\n';
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
    // // std::cout << "BCP Constructor\n";
    for (auto n : g.get_nodes()) {
        auto node = n.first;
        low[node] = -1;
        discover_time[node] = -1;
        articulation_points[node] = false;
        parent[node] = zero_node;
    }
    // // std::cout << "Begin Tarjan\n";
    for (auto n : g.get_nodes()) {
        auto node = n.first;
        if (discover_time[node] == -1) {
            tarjan(node);
        }
    }
    // // std::cout << "End Tarjan\n";
    if (!(edge_stack.size() == 0)) {
        // // std::cout << "Last Component\n";
        boost::unordered::unordered_set<std::pair<int, int>> last_component;
        // // std::cout << zero_node << '\n';
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
    // // std::cout << biconnected_components.size() << "\n";
    // for (auto k : biconnected_components) {
    //     for (auto v : k) {
    //         // std::cout << v.first << ", " << v.second << "; ";
    //     }
    //     // std::cout << "\n";
    // }
    std::vector<std::vector<std::pair<int, int>>> v;
    std::vector<std::vector<std::pair<int, int>>> p;
    total = biconnected_graph_optimal_path(get_pac_start().second, v, get_pac_start().first, true, p);
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

// const PacmanGraph BCPGraph::TreeNode::pg_init(BCPGraph b, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes, std::pair<int, int> start_node, std::vector<std::pair<int, int>> curr_component) {
//     // TODO: Include one node from each foreign component that's adjacent to the current component to represent TreeNode connections.
//     for (auto kv : nodes) {
//         if (std::find(curr_component.begin(), curr_component.end(), kv.first) != curr_component.end()) {
//             for (auto n : kv.second) {
//                 if (std::find(curr_component.begin(), curr_component.end(), n) != curr_component.end()) {
//                     component_nodes[kv.first].push_back(n);
//                 }
//             }
//             if (b.articulation_table()[kv.first]) {
//                 for (auto n : b.get_neighbors(kv.first)) {
//                     if (std::find(curr_component.begin(), curr_component.end(), n) == curr_component.end()) {
//                         external_nodes.insert(n);
//                         component_nodes[kv.first].push_back(n);
//                         component_nodes[n].push_back(kv.first);
//                     }
//                 }
//             }
//         }
//     }
//     Graph g(b.get_graph());
//     return PacmanGraph(g, component_nodes, start_node);
// }

/**
 * 
 */
// BCPGraph::TreeNode::TreeNode(BCPGraph b, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes, std::pair<int, int> start_node, std::vector<std::pair<int, int>> curr_component, std::vector<std::vector<std::pair<int, int>>>* visited_components, std::vector<std::pair<int, int>> parent, TreeNode* parent_treenode) : pg(pg_init(b, nodes, start_node, curr_component)) {
//     pac_start = start_node;
//     parent_component = parent;
//     parent_tree =  parent_treenode;
//     treenode_component = curr_component;
//     optimal_cost_last = -1;
//     optimal_cost_mid = -1;
//     // TODO: Edit pg food to accurately reflect foreign component food presence
//     // TODO: Edit pg path_memo to accurately reflect component path length -- best done in path computation
//     // Moves out from foreign components will cost 0, but moves into foreign components will cost the exact amount to process them.
//     for (auto kv : component_nodes) {
//         if (!b.articulation_table()[kv.first] && b.get_components(kv.first)[0] != curr_component) {
//             pg.insert_to_path_memo(pg.bit_encode(kv.first), pg.bit_encode(kv.second[0]), 0);
//         }
//     }
//     std::vector<std::pair<std::pair<int, int>, std::vector<std::pair<int, int>>>> child_components;
//     for (auto n : curr_component) {
//         if (b.articulation_table()[n]) {
//             for (auto c : b.get_components(n)) {
//                 if (std::find(visited_components->begin(), visited_components->end(), c) == visited_components->end()) {
//                     visited_components->push_back(c);
//                     child_components.push_back({n, c});
//                 }
//             }
//         }
//     }
//     compute_children(b, curr_component, visited_components, child_components, nodes);
//     // // std::cout << "Child Vector Size: " << child_components.size() << "\n";
//     // // std::cout << "Number of Child TreeNodes: " << children.size() << "\n";
// }

// void BCPGraph::TreeNode::compute_children(BCPGraph b, std::vector<std::pair<int, int>> curr_component, std::vector<std::vector<std::pair<int, int>>>* visited_components, std::vector<std::pair<std::pair<int, int>, std::vector<std::pair<int, int>>>> child_components, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes) {
//     for (auto c : child_components) {
//         auto t = new TreeNode(b, nodes, c.first, c.second, visited_components, curr_component, this);
//         auto ap = c.first;
//         for (auto n : b.get_neighbors(ap)) {
//             if (std::find(c.second.begin(), c.second.end(), n) != c.second.end()) {
//                 external_node_to_child_map[n] = t;
//             }
//         }
//         children.push_back(t);
//     }
// }

std::vector<std::pair<int, int>> longest_component(std::vector<std::vector<std::pair<int, int>>> components) {
    std::vector<std::pair<int, int>> component;
    for (auto c : components) {
        if (c.size() > component.size()) {
            component = c;
        }
    }
    return component;
}

std::pair<std::pair<int, int>, std::vector<std::pair<int, int>>> BCPGraph::get_pac_start() {
    auto matrix = g.matrix();
    for (auto i = 0; i < (int)matrix.size(); i++) {
        for (auto j = 0; j < (int)matrix[0].size(); j++) {
            if (matrix[i][j] == 3) {
                return {{i, j}, longest_component(get_components({i, j}))};
            }
        }
    }
    return {{-1, -1}, {}};
}

// void BCPGraph::treeify() {
//     auto start = get_pac_start();
//     auto v = new std::vector<std::vector<std::pair<int, int>>>();
//     v->push_back(start.second);
//     t = new TreeNode(*this, g.get_nodes(), start.first, start.second, v, {}, nullptr);
//     delete v;
// }

bool BCPGraph::TreeNode::food_in_component() {
    return food;
}

// std::vector<std::string> BCPGraph::optimal_path_calc() {
//     t->optimal_path_calc(true, this);
//     // std::cout << "Optimal Path Calc Done\n";;
//     return t->optimal_cost_and_path(true).second;
// }

// BCPGraph::TreeNode* BCPGraph::TreeNode::get_child(std::vector<std::pair<int, int>> component) {
//     for (auto c : children) {
//         if (component == c->curr_component()) {
//             return c;
//         }
//     }
//     return parent_tree;
// }

// void BCPGraph::TreeNode::optimal_path_calc(bool last, BCPGraph* b) {
//     /**
//      * To get the optimal path in the current node, we need to follow these steps:
//      * 1) Get optimal paths within all child nodes -- once for when the child is last and once for not
//      * 2) Edit path_memo in pg to reflect the exact cost of going to child components
//      * 3) Perform A* search on pg twice -- once for when the current component is last and once when it's not 
//      * 4) Edit the resultant path to replace child component placeholders with actual child paths
//      * 
//      * Determining when a component is "last":
//      * "Last" means that when all food has been consumed in the component (and its children),
//      * Pacman will not have to double back to the parent component to address the parent's other child nodes.
//      * Because the root has no parent, the root is always "Last".
//      * A component's "Last" path is used if, during an A* search in the parent, the component's connecting node
//      * is the last node visited with no outstanding food within the component or other children of the parent.
//      */

//     if ((last && optimal_cost_last != -1) || (!last && optimal_cost_mid != -1)) {
//         return;
//     }

//     for (auto c : children) {
//         c->optimal_cost_and_path(true);
//         c->optimal_cost_and_path(false);
//     }
//     if (!last) {
//         pg.set_food_bit(boost::dynamic_bitset<>(pg.num_nodes(), 1) << (pg.num_nodes() - 1), 1);
//     }
//     for (auto n : pg.nodes) {
//         if (external_nodes.contains(pg.unencode(n.first))) {
//             if (std::find(parent_component.begin(), parent_component.end(), pg.unencode(n.first)) == parent_component.end()) {
//                 auto child = external_node_to_child_map[pg.unencode(n.first)];
//                 if (child->food_in_component()) {
//                     pg.set_food_bit(n.first, 1);
//                 }
//                 pg.insert_to_path_memo(pg.get_neighbors(n.first)[0], n.first, child->optimal_cost_and_path(false).first);
//                 pg.insert_to_path_memo(n.first, pg.get_neighbors(n.first)[0], 0);
//                 pg.insert_to_path_memo(pg.get_neighbors(n.first)[0], (n.first & (boost::dynamic_bitset<>(pg.num_nodes(), 1) << (pg.num_nodes() - 1))), child->optimal_cost_and_path(true).first);
//                 pg.insert_to_path_memo((n.first & (boost::dynamic_bitset<>(pg.num_nodes(), 1) << (pg.num_nodes() - 1))), pg.get_neighbors(n.first)[0], 0);
//             }
//             else {
//                 pg.insert_to_path_memo(pg.get_neighbors(n.first)[0], n.first, 99999);
//                 pg.insert_to_path_memo(n.first, pg.get_neighbors(n.first)[0], 0);
//                 pg.insert_to_path_memo(pg.get_neighbors(n.first)[0], (n.first & (boost::dynamic_bitset<>(pg.num_nodes(), 1) << (pg.num_nodes() - 1))), 0);
//                 pg.insert_to_path_memo((n.first & (boost::dynamic_bitset<>(pg.num_nodes(), 1) << (pg.num_nodes() - 1))), pg.get_neighbors(n.first)[0], 0);
//             }
//         }
//     }

//     auto p = astar(pg);

//     std::vector<std::string> total_path;
//     auto s = pg.unencode(pg.get_pac_start());
//     int i = 0;
//     boost::unordered::unordered_map<std::string, std::pair<int, int>> dirs;
//     dirs["East"] = {0, 1};
//     dirs["West"] = {0, -1};
//     dirs["South"] = {1, 0};
//     dirs["North"] = {-1, 0};
//     bool skip = false;
//     for (auto d : p) {
//         s = {s.first + dirs[d].first, s.second + dirs[d].second};
//         if (skip) {
//             skip = false;
//         }
//         if (!external_nodes.contains(s)) {
//             total_path.push_back(d);
//         }
//         else {
//             std::vector<std::string> subpath;
//             if (i == (int)p.size() - 1) {
//                 subpath = external_node_to_child_map[s]->optimal_cost_and_path(true).second;
//             }
//             else {
//                 subpath = external_node_to_child_map[s]->optimal_cost_and_path(false).second;
//             }
//             s = {s.first - dirs[d].first, s.second - dirs[d].second};
//             for (auto sd : subpath) {
//                 total_path.push_back(sd);
//             }
//         }
//     }

//     if (last)  {
//         optimal_path_last = total_path;
//         optimal_cost_last = total_path.size();
//     }
//     else {
//         optimal_path_mid = total_path;
//         optimal_cost_mid = total_path.size();
//     }
// }

std::pair<int, std::vector<std::string>> BCPGraph::TreeNode::optimal_cost_and_path(bool last) {
    std::pair<int, std::vector<std::string>> l = {optimal_cost_last, optimal_path_last};
    std::pair<int, std::vector<std::string>> m = {optimal_cost_mid, optimal_path_mid};
    return last ? l : m;
}

std::vector<std::pair<int, int>> BCPGraph::TreeNode::curr_component() {
    return treenode_component;
}

std::vector<std::string> BCPGraph::biconnected_graph_optimal_path(std::vector<std::pair<int, int>> curr_component, std::vector<std::vector<std::pair<int, int>>> visited_components, std::pair<int, int> start_node, bool last_component, std::vector<std::vector<std::pair<int, int>>> peer_components) {
    if (child_paths.contains(curr_component)) {
        if (last_component) {
            return child_paths[curr_component].second;
        }
        else {
            return child_paths[curr_component].first;
        }
    }
    auto vis = visited_components;
    vis.push_back(curr_component);
    if (last_component) {
        // std::cout << "Last\n";
    }
    else {
        // std::cout << "Intermediate\n";
    }

    std::vector<std::pair<std::pair<int, int>, std::vector<std::pair<int, int>>>> child_components; 
    for (auto kv : g.get_nodes()) {
        auto n = kv.first;
        if (articulation_table()[n]) {
            if (std::find(curr_component.begin(), curr_component.end(), n) != curr_component.end() ) {
                for (auto c : get_components(n)) {
                    if (std::find(vis.begin(), vis.end(), c) == vis.end()) {
                        child_components.push_back({n, c});
                    }
                }
            }
        }
    }
    // std::cout << "Child Component Count: " << child_components.size() << "\n";
    for (auto c : child_components) {
        auto a_point = c.first;
        auto child_component = c.second;
        if (!child_paths.contains(child_component)) {
            std::vector<std::vector<std::pair<int, int>>> peers;
            for (auto child : child_components) {
                if (child.second != child_component) {
                    peers.push_back(child.second);
                }
            }
            auto last_component_path = biconnected_graph_optimal_path(child_component, vis, a_point, true, peers);
            auto intermediate_component_path = biconnected_graph_optimal_path(child_component, vis, a_point, false, peers);
            child_paths[child_component] = {intermediate_component_path, last_component_path};
        }
    }
    std::vector<std::pair<int, int>> food;
    for (auto n : curr_component) {
        if (g.matrix()[n.first][n.second] == 2) {
            food.push_back(n);
        }
    }
    std::sort(food.begin(), food.end());
    std::vector<std::pair<int, int>> parent_component;
    if (visited_components.size() > 0) {
        parent_component = visited_components[visited_components.size() - 1];
    }

    // {a_point, intermediate length, last length}
    boost::unordered::unordered_map<std::pair<std::pair<int ,int>, std::vector<std::pair<int, int>>>, std::pair<std::pair<std::pair<int, int>, std::pair<int, int>>, std::pair<int, int>>> foreign_food_map;

    for (auto n : curr_component) {
        if (articulation_table()[n]) {
            for (auto c : get_components(n)) {
                if (c != curr_component) {
                    std::pair<int, int> min_tile = {-1, -1};
                    std::pair<int, int> max_tile;
                    boost::unordered::unordered_set<std::pair<int, int>> visited;
                    std::queue<std::pair<int, int>> q;
                    q.push(n);
                    while (!(q.size() == 0)) {
                        auto state = q.front();
                        q.pop();
                        if (visited.contains(state)) {
                            continue;
                        }
                        visited.insert(state);
                        if (g.matrix()[state.first][state.second] == 2 && state != start_node) {
                            max_tile = state;
                            if (min_tile == (std::pair<int, int>){-1, -1}) {
                                min_tile = state;
                            }
                        }
                        for (auto neighbor : get_neighbors(state)) {
                            if (std::find(curr_component.begin(), curr_component.end(), neighbor) == curr_component.end() && neighbor != n) {
                                q.push(neighbor);
                            }
                        }
                    }
                    if (min_tile != (std::pair<int, int>){-1, -1}) {
                        if (child_paths.contains(c) && std::find(peer_components.begin(), peer_components.end(), c) == peer_components.end()) {
                            foreign_food_map[{n, c}] = {{min_tile, max_tile}, {child_paths[c].first.size(), child_paths[c].second.size()}};
                        }
                        else if (!last_component && std::find(peer_components.begin(), peer_components.end(), c) == peer_components.end()) {
                            foreign_food_map[{n, c}] = {{min_tile, max_tile}, {99999, 0}};
                        }
                    }
                }
            }
        }
    }

    PacmanGraph* pg = new PacmanGraph(food, start_node, curr_component, foreign_food_map, articulation_points, path_memo);
    // std::cout << "Begin A*\n";
    auto start = std::chrono::high_resolution_clock::now();
    auto path = astar(*pg);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> dur = end - start;

    delete pg;
    // exit(0);
    boost::unordered::unordered_map<std::pair<int ,int>, std::vector<std::vector<std::pair<int ,int>>>> foreign_components;
    boost::unordered::unordered_map<std::pair<std::pair<int, int>, std::string>, std::vector<std::pair<int, int>>> s_to_component;
    for (auto ac : foreign_food_map) {
        auto a = ac.first.first;
        auto c = ac.first.second;
        foreign_components[a].push_back(c);
    }
    for (auto ac : foreign_components) {
        int i = 0;
        for (auto c : ac.second) {
            s_to_component[{ac.first, std::to_string(i)}] = c;
            i++;
        }
    }

    // std::cout << "A* path: ";
    for (auto p : path) {
        // std::cout << p << ", ";
    }
    // std::cout << "\n";

    // std::cout << "\nForeign Component Conversion\n";
    for (auto c : s_to_component) {
        // std::cout << c.first.first.first << " " << c.first.first.second << ": " << c.first.second << "\n";
    }

    // std::cout << "Invalid Switch Pushback\n";
    std::vector<int> invalid_switches;
    auto s = start_node;
    int removal_index = 0;
    for (auto d : path) {
        if (!undirs.contains(d)) {
            // s = {s.first + undirs.find(d)->second.first, s.second + undirs.find(d)->second.second};
            if (std::find(visited_components.begin(), visited_components.end(), s_to_component[{s, d}]) != visited_components.end()) {
                invalid_switches.push_back(removal_index);
            }
        }
        else {
            s = {s.first + undirs.find(d)->second.first, s.second + undirs.find(d)->second.second};
        }
        removal_index += 1;
    }
    // std::cout << "Invalid Switch Removal\n";
    for (auto sw : invalid_switches) {
        path.erase(path.begin() + sw);
    }

    std::vector<std::string> total_path;
    s = start_node;
    int i = 0;

    for (auto d : path) {
        // std::cout << "d: " << d << "\n";
        if (undirs.contains(d)) {
            // std::cout << "Normal Direction\n";
            s = {s.first + undirs.find(d)->second.first, s.second + undirs.find(d)->second.second};
            total_path.push_back(d);
        }
        else {
            // std::cout << "Component Switch\n";
            // std::cout << d << "\n";
            // std::cout << s.first << " " << s.second << ": " << d << "\n";
            if (child_paths.contains(s_to_component[{s, d}])) {
                // std::cout << "Subpath obtained\n";
                auto subpath = child_paths[s_to_component[{s, d}]].first;
                if (i == (int)path.size() - 1) {
                    subpath = child_paths[s_to_component[{s, d}]].second;
                }
                for (auto sd : subpath) {
                    total_path.push_back(sd);
                }
            }
            else {
                // std::cout << "Subpath for component " << d << " not found in child_paths\n";
                exit(0);
            }
        }
        i++;
    }

    // std::cout << "Total path: ";
    for (auto p : total_path) {
        // std::cout << p << ", ";
    }
    // std::cout << "\n";

    // std::cout << "\n";
    // std::cout << "Component ";

    // for (auto n : curr_component) {
    //     std::cout << n.first << " " << n.second << ", ";
    // }
    // std::cout << "Done in " << dur.count() << " seconds.\n";
    return total_path;
}