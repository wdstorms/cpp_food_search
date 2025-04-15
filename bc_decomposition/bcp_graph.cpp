#include "bcp_graph.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>

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
            // // std::cout << "Recurse Tarjan\n";
            tarjan(n);
            low[node] = std::min(low[node], low[n]);
            if ((parent[node] == zero_node && children > 1) || (parent[node] != zero_node && low[n] >= discover_time[node])) {
                articulation_points[node] = true;
                boost::dynamic_bitset<> component = boost::dynamic_bitset<>(pg.num_nodes(), 0);
                // // std::cout << "edge stack size: " << edge_stack.size() << '\n';
                boost::dynamic_bitset<> e = zero_node;
                while (e != (n | node)) {
                    // // std::cout << edge_stack.top() << "\n";
                    component |= edge_stack.top();
                    e = edge_stack.top();
                    edge_stack.pop();
                }
                // // std::cout << "component: " << component << '\n';
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
    // std::cout << "BCP Constructor\n";
    while (bit < pg.num_nodes()) {
        boost::dynamic_bitset<> node = boost::dynamic_bitset(pg.num_nodes(), 1) << bit;
        low[node] = -1;
        discover_time[node] = -1;
        articulation_points[node] = false;
        parent[node] = zero_node;
        bit += 1;
    }
    // std::cout << "Begin Tarjan\n";
    bit = 0;
    while (bit < pg.num_nodes()) {
        boost::dynamic_bitset<> node = boost::dynamic_bitset(pg.num_nodes(), 1) << bit;
        if (discover_time[node] == -1) {
            // std::cout << "bit: " << bit << '\n';
            tarjan(node);
        }
        bit += 1;
    }
    if (!(edge_stack.size() == 0)) {
        boost::dynamic_bitset<> last_component = zero_node;
        // std::cout << zero_node << '\n';
        while (!(edge_stack.size() == 0)) {
            last_component |= edge_stack.top();
            edge_stack.pop();
        }
        biconnected_components.push_back(last_component);
    }
    // std::cout << biconnected_components.size() << '\n';
    // std::cout << "time: " << tarjan_time << '\n';
    for (auto b : biconnected_components) {
        // std::cout << b << "\n";
    }
    // std::cout << "Remaining edges: " << edge_stack.size() << '\n';
    // std::cout << "End Constructor\n";
}

boost::unordered::unordered_map<boost::dynamic_bitset<>, bool> BCPGraph::articulation_table() {
    return articulation_points;
}

PacmanGraph BCPGraph::get_pacgraph() {
    return pg;
}

std::vector<boost::dynamic_bitset<>> BCPGraph::get_components(boost::dynamic_bitset<> node) {
    std::vector<boost::dynamic_bitset<>> components;
    for (auto c : biconnected_components) {
        if ((c & node) != boost::dynamic_bitset<>(pg.num_nodes(), 0)) {
            components.push_back(c);
        }
    }
    return components;
}

bool node_within_component(boost::dynamic_bitset<> n, boost::dynamic_bitset<> c) {
    return (n & c) != boost::dynamic_bitset<>(n.size(), 0);
}

/**
 * Process for constructing the biconnected component tree:
 * 1) Determine the component this node encapsulates.
 * 2) Revise node mapping -- remove unnecessary nodes, and add connections to other TreeNodes.
 * 3) Create new path memo, adding TreeNode connections as well.
 * 4) Create a new food bitset which includes neighboring components.
 * 5) Create new PacmanGraph.
 */
BCPGraph::TreeNode::TreeNode(BCPGraph* b, boost::dynamic_bitset<> start_node, boost::dynamic_bitset<> curr_component, std::vector<boost::dynamic_bitset<>>* visited_components) {
    // Determine biconnected component for this treenode to encapsulate.
    auto p = b->get_pacgraph();
    // std::cout << curr_component << "\n";
    // std::cout << "Step 1 done\n";

    // Revise node mapping -- remove unnecessary nodes, and add connections to other TreeNodes.
    auto start = std::chrono::high_resolution_clock::now();
    int total_bits = p.num_nodes();
    int bit = 0;
    boost::dynamic_bitset<> bitset = boost::dynamic_bitset<>(p.num_nodes(), 1);
    boost::unordered::unordered_map<boost::dynamic_bitset<>, bool> components_table;
    boost::unordered::unordered_map<boost::dynamic_bitset<>, boost::dynamic_bitset<>> components_map;
    while (bit < p.num_nodes()) {
        components_table[bitset] = false;
        if (node_within_component(bitset, curr_component) && b->articulation_table()[bitset]) {
            for (auto c : b->get_components(bitset)) {
                if (std::find(visited_components->begin(), visited_components->end(), c) == visited_components->end()) {
                    components_table[c] = true;
                    components_map[c] = boost::dynamic_bitset<>(total_bits + 1, 1) << total_bits;
                    total_bits += 1;
                }
                else {
                    components_table[c] = false;
                }
            }
        }
        bit++;
        bitset <<= 1;
    }

    for (auto kv : components_map) {
        kv.second.resize(total_bits);
    }

    boost::unordered::unordered_map<boost::dynamic_bitset<>, std::vector<boost::dynamic_bitset<>>> new_nodes;
    bit = 0;
    bitset = boost::dynamic_bitset<>(total_bits, 1);
    curr_component.resize(total_bits);
    while (bit < total_bits) {
        if (node_within_component(bitset, curr_component)) {
            for (auto n : p.get_neighbors(bitset)) {
                if (node_within_component(n, curr_component)) {
                    new_nodes[bitset].push_back(n);
                }
            }
            if (b->articulation_table()[bitset]) {
                for (auto c : b->get_components(bitset)) {
                    if (components_table[c]) {
                        c.resize(total_bits);
                        new_nodes[components_map[c]].push_back(bitset);
                        new_nodes[bitset].push_back(components_map[c]);
                    }
                }
            }
        }
        bit++;
        bitset <<= 1;
    }

	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> dur = end - start;
	std::cout << "Node Mapping done in: " << dur.count() << " seconds.\n";
    // std::cout << "Step 2 done\n";
    // Create new path memo, adding TreeNode connections as well.

	start = std::chrono::high_resolution_clock::now();
    boost::unordered::unordered_map<std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, int> path_memo;
    for (auto kv : new_nodes) {
        auto n = kv.first;
        if (components_table[n]) {
            path_memo[{components_map[n], new_nodes[n].front()}] = 99999;
            continue;
        }
        int depth = 0;
        auto visited = boost::unordered::unordered_set<boost::dynamic_bitset<>>();
        auto q = std::queue<boost::dynamic_bitset<>>();
        q.push(n);
        while (!(q.size() == 0)) {
            int level_size = q.size();
            while (level_size != 0 && !(q.size() == 0)) {
                auto state = q.front();
                q.pop();
                if (visited.contains(state)) {
                    continue;
                }
                visited.insert(state);
                if (!components_table[state]) {
                    path_memo[{n, state}] = depth;
                }
                else {
                    path_memo[{n, components_map[state]}] = 99999;
                }
                for (auto node : new_nodes[state]) {
                    if (!visited.contains(node)) {
                        q.push(node);
                    }
                }
                level_size -= 1;
            }
            depth += 1;
        }
    }

	end = std::chrono::high_resolution_clock::now();
	dur = end - start;
	std::cout << "Path Memo done in: " << dur.count() << " seconds.\n";
    // std::cout << "Step 3 done\n";
    auto new_food = p.get_food();
    start = std::chrono::high_resolution_clock::now();
    new_food.resize(total_bits, true);
	end = std::chrono::high_resolution_clock::now();
	dur = end - start;
	std::cout << "New Food done in: " << dur.count() << " seconds.\n";
    // std::cout << "Step 4 done\n";
    // std::cout << new_food << "\n";

    // Revise bit encodings for the given component (including connections to adjacent components).
    // int bit = 0;
    // boost::dynamic_bitset<> bitset = boost::dynamic_bitset<>(p.num_nodes(), 1);
    // int curr_component_num_nodes = 0;
    // int nodes_within_component_count = 0;
    // while (bit < p.num_nodes()) {
    //     if ((bitset & curr_component) != boost::dynamic_bitset<>(p.num_nodes(), 0)) {
    //         curr_component_num_nodes += 1;
    //         nodes_within_component_count += 1;
    //         if (b->articulation_table()[bitset]) {
    //             for (auto c : b->get_components(bitset)) {
    //                 // // std::cout << c << "\n";
    //                 if (std::find(visited_components->begin(), visited_components->end(), c) == visited_components->end()) {
    //                     // // std::cout << "Foreign Component\n";
    //                     curr_component_num_nodes += 1;
    //                 }
    //             }
    //         }
    //     } 
    //     bit += 1;
    //     bitset <<= 1;
    // }
    // boost::unordered::unordered_map<boost::dynamic_bitset<>, boost::dynamic_bitset<>> new_encodings;
    // boost::unordered::unordered_map<boost::dynamic_bitset<>, std::vector<boost::dynamic_bitset<>>> new_nodes;
    // bit = 0;
    // bitset = boost::dynamic_bitset<>(p.num_nodes(), 1);
    // int new_bit = 0;
    // int component_bit = nodes_within_component_count;
    // // // std::cout << curr_component_num_nodes << "\n";
    // bitset = boost::dynamic_bitset<>(p.num_nodes(), 1);
    // while (bit < p.num_nodes()) {
    //     if ((bitset & curr_component) != boost::dynamic_bitset<>(p.num_nodes(), 0)) {
    //         new_encodings[bitset] = boost::dynamic_bitset<>(curr_component_num_nodes, 1) << new_bit;
    //         new_bit += 1;
    //         if (b->articulation_table()[bitset]) {
    //             for (auto c : b->get_components(bitset)) {
    //                 if (std::find(visited_components->begin(), visited_components->end(), c) == visited_components->end()) {
    //                     new_encodings[c] = boost::dynamic_bitset<>(curr_component_num_nodes, 1) << component_bit;
    //                     component_bit += 1;
    //                     // Getting ahead of step 3 by adding treenode connections to adjacency list
    //                     new_nodes[new_encodings[bitset]].push_back(new_encodings[c]);
    //                     new_nodes[new_encodings[c]].push_back(new_encodings[bitset]);
    //                 }
    //             }
    //         }
    //     } 
    //     bit += 1;
    //     bitset <<= 1;
    // }
    // // // std::cout << "Step 2 done\n";
    // for (auto kv : new_encodings) {
    //     // std::cout << kv.first << " : " << kv.second << "\n";
    // }
    // // std::cout << "\n";
    // for (auto kv : new_nodes) {
    //     // std::cout << kv.first << ": ";
    //     for (auto n : kv.second) {
    //         // std::cout << n << " ";
    //     }
    //     // std::cout << "\n";
    // }
    // // std::cout << "\n";
    // // Create a new adjacency mapping for the new encodings.
    // for (auto kv : new_encodings) {
    //     for (auto n : p.get_neighbors(kv.first)) {
    //         if ((n & curr_component) != boost::dynamic_bitset<>(p.num_nodes(), 0)) {
    //             // // std::cout << "_____\n";
    //             // // std::cout << kv.first << " " << kv.second << "\n";
    //             // // std::cout << new_encodings[kv.first] << " " << new_encodings[n] << "\n";
    //             if (new_encodings.count(n)) {
    //             new_nodes[kv.second].push_back(new_encodings[n]);
    //             }
    //             else {
    //             std::cerr << "Error: Neighbor encoding not found for " << n << std::endl;
    
    //             }
    //         }
    //     }
    // }
    // // std::cout << "Step 3 done\n";
    // for (auto kv : new_nodes) {
    //     // std::cout << kv.first << ": ";
    //     for (auto n : kv.second) {
    //         // std::cout << n << " ";
    //     }
    //     // std::cout << "\n";
    // }

    // // Create a new path memo for the given component.
    // boost::unordered::unordered_map<std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, int> path_memo;
    // for (auto kv : new_nodes) {
    //     auto n = kv.first;
    //     if ((n >= boost::dynamic_bitset<>(curr_component_num_nodes, 1) << nodes_within_component_count)) {
    //         path_memo[{n, new_nodes[n].front()}] = 99999;
    //         continue;
    //     }
    //     int depth = 0;
    //     auto visited = boost::unordered::unordered_set<boost::dynamic_bitset<>>();
    //     auto q = std::queue<boost::dynamic_bitset<>>();
    //     q.push(n);
    //     while (!(q.size() == 0)) {
    //         int level_size = q.size();
    //         while (level_size != 0 && !(q.size() == 0)) {
    //             auto state = q.front();
    //             q.pop();
    //             if (visited.contains(state)) {
    //                 continue;
    //             }
    //             visited.insert(state);
    //             if ((state < boost::dynamic_bitset<>(curr_component_num_nodes, 1) << nodes_within_component_count)) {
    //                 path_memo[{n, state}] = depth;
    //             }
    //             else {
    //                 path_memo[{n, state}] = 99999;
    //             }
    //             for (auto node : new_nodes[state]) {
    //                 if (!visited.contains(node)) {
    //                     q.push(node);
    //                 }
    //             }
    //             level_size -= 1;
    //         }
    //         depth += 1;
    //     }
    // }
    // // std::cout << "Step 4 done\n";
    // // Determine a new food bitset for the component.
    // auto old_food = p.get_food();
    // boost::dynamic_bitset<> new_food = boost::dynamic_bitset<>(curr_component_num_nodes, 0);
    // bit = 0;
    // bitset = boost::dynamic_bitset(p.num_nodes(), 1);
    // while (bit < p.num_nodes()) {
    //     if ((bitset & curr_component) != boost::dynamic_bitset<>(p.num_nodes(), 0)) {
    //         if ((bitset & old_food) != boost::dynamic_bitset<>(p.num_nodes(), 0)) {
    //             new_food |= new_encodings[bitset];
    //         }
    //     }
    //     bit += 1;
    //     bitset <<= 1;
    // }
    // // std::cout << "Step 5 done\n";
    // // std::cout << old_food << " " << new_food << "\n";
    // Construct a new PacmanGraph using the new adjacency map, path memo, food, and converted start position.
    pg = PacmanGraph(start_node, new_food, path_memo, new_nodes);
    // std::cout << "Step 5 done\n";
    // std::cout << "Step 6 done\n";
    // Find all articulation points for which there is at least one component not represented and make TreeNodes for them (child TreeNodes).
    compute_children(b, visited_components);
    // std::cout << "Step 6 done\n";
    // std::cout << "Step 7 done\n";
}

void BCPGraph::TreeNode::compute_children(BCPGraph* b, std::vector<boost::dynamic_bitset<>>* visited_components) {
    std::vector<std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>> child_components;
    int bit = 0;
    auto bitset = boost::dynamic_bitset<>(b->get_pacgraph().num_nodes(), 1);
    while (bit < b->get_pacgraph().num_nodes()) {
        if (b->articulation_table()[bitset]) {
            for (auto c : b->get_components(bitset)) {
            if (std::find(visited_components->begin(), visited_components->end(), c) == visited_components->end()) {
                child_components.push_back({bitset, c}); // New start position, component
                visited_components->push_back(c);
            }
            }
        }
        bit += 1;
        bitset <<= 1;
    }
    std::vector<std::chrono::duration<double>> children_time;
    auto start = std::chrono::high_resolution_clock::now();
    for (auto sc : child_components) {
        auto s = sc.first;
        auto c = sc.second;
        auto single_child_start = std::chrono::high_resolution_clock::now();
        children.push_back(TreeNode(b, s, c, visited_components));
        auto single_child_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> single_dur = single_child_end - single_child_start;
        children_time.push_back(single_dur);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> dur = end - start;
    std::cout << "Children for component done in: " << dur.count() << " seconds.\n";
    for (auto t : children_time) {
        std::cout << t.count() << " ";
    }
    std::cout << "\n";
}

void BCPGraph::treeify() {
    auto visited = new std::vector<boost::dynamic_bitset<>>();
    PacmanGraph p = get_pacgraph();
    auto components = get_components(p.get_pac_start());
    boost::dynamic_bitset curr_component;
    for (auto c : components) {
            curr_component = c;
            visited->push_back(c);
            break;
    }
    // // std::cout << curr_component << "\n";
    // // std::cout << "Step 1 done\n";
    t = new TreeNode(this, pg.get_pac_start(), curr_component, visited);
    delete visited;
}