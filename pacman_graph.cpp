#include "pacman_graph.hpp"
#include <boost/unordered_set.hpp>
#include <iostream>
#include <functional>
#include <queue>
#include <cassert>

void PacmanGraph::init_path_memo() {
    for (auto n : curr_component) {
        int depth = 0;
        auto visited = boost::unordered::unordered_set<std::pair<int ,int>>();
        auto q = std::queue<std::pair<int ,int>>();
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
                if (depth > 0) {
                    path_memo[{n, state}] = depth;
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

PacmanGraph::PacmanGraph(Graph &g) {
    std::vector<std::vector<int>> matrix = g.matrix();
    for (auto n : g.get_nodes()) {
        curr_component.push_back(n.first);
    }
    for (auto n : curr_component) {
        if (g.matrix()[n.first][n.second] == 2) {
            food.push_back(n);
        }
        else if (g.matrix()[n.first][n.second] == 3) {
            pac_start = n;
        }
    }
    std::sort(food.begin(), food.end());
    init_path_memo();
}

// const std::vector<std::vector<int>> &PacmanGraph::matrix() {
//     return g.matrix();
// }

const std::pair<int ,int> PacmanGraph::get_pac_start() {
    return pac_start;
}

const std::vector<std::pair<int, int>> PacmanGraph::get_food() {
    return food;
}

const boost::unordered::unordered_set<std::vector<std::pair<int ,int>>> PacmanGraph::get_foreign_components() {
    boost::unordered::unordered_set<std::vector<std::pair<int ,int>>> s;
    for (auto kv : foreign_components) {
        auto cs = kv.second;
        for (auto c : cs) {
            s.insert(c);
        }
    }
    return s;
}

// boost::dynamic_bitset<> PacmanGraph::bit_encode(std::pair<int, int> coord) {
//     assert(bitmap.contains(coord));
//     return bitmap[coord];
// }

// std::pair<int, int> PacmanGraph::unencode(boost::dynamic_bitset<> bit) {
//     assert(unmap.contains(bit));
//     return unmap[bit];
// }

int PacmanGraph::num_nodes() const {
    return node_count;
};

boost::unordered::unordered_map<std::pair<std::pair<int, int>, std::pair<int, int>>, int> PacmanGraph::memo() {
    return path_memo;
}

// void PacmanGraph::insert_to_path_memo(boost::dynamic_bitset<> src, boost::dynamic_bitset<> dst, int v) {
//     path_memo[{src, dst}] = v;
// }

std::vector<std::tuple<int, std::tuple<std::pair<int, int>, std::vector<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int>, std::vector<std::string>, int>> PacmanGraph::get_successors(std::tuple<std::pair<int, int>, std::vector<std::pair<int ,int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int> state, std::vector<std::string> actions, int cost, std::function<int(std::pair<int ,int>, std::vector<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int, int>>>, PacmanGraph*)> food_heuristic) {
    // return get_successors_(state, actions, cost, food_heuristic);
    std::vector<std::tuple<int, std::tuple<std::pair<int, int>, std::vector<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int>, std::vector<std::string>, int>> successors;
    // exit(0);
    int x = std::get<0>(state).first;
    int y = std::get<0>(state).second;
    // std::cout << "get_successors call\n";
    for (auto directions : dirs) {
        auto d = directions.first;
        int nextx = x + d.first;
        int nexty = y + d.second;
        std::pair<int, int> next_coord = {nextx, nexty};
        if (std::find(curr_component.begin(), curr_component.end(), next_coord) != curr_component.end()) {
            // std::cout << "Normal Direction Successor\n";
            // std::cout << nextx << " " << nexty << "\n"; 
            // std::cout << dirs.find(d)->second << "\n";
            auto next_food = std::get<1>(state);
            auto next_foreign_food = std::get<2>(state);
            // std::find(next_food.begin(), next_food.end(), {nextx, nexty}) != next_food.end()
            if (std::find(next_food.begin(), next_food.end(), (std::pair<int, int>){nextx, nexty}) != next_food.end()) {
                next_food.erase(std::find(next_food.begin(), next_food.end(), (std::pair<int, int>){nextx, nexty}));
            }
            auto a = actions;
            a.push_back(dirs.find(d)->second);
            successors.push_back({cost - 1 - food_heuristic({nextx, nexty}, next_food, next_foreign_food, this), {{nextx, nexty}, next_food, next_foreign_food, -1}, a, cost - 1});
        }
    }
    if (a_points[{x, y}]) {
        // std::cout << "Articulation Point Found\n";
        if (foreign_components.contains({x, y})) {
            // std::cout << "Foreign Component Found\n";
            for (int i = 0; i < (int)foreign_components[{x, y}].size(); i++) {
                if (i != std::get<3>(state)) {
                    // std::cout << "Component Switch\n";
                    auto next_foreign_food = std::get<2>(state);
                    if (next_foreign_food.contains(foreign_components[{x, y}][i])) {
                        auto next_food = std::get<1>(state);
                        int min_component_cost = foreign_food[{{x, y}, foreign_components[{x, y}][i]}].second.first;
                        if (next_food.size() == 0 && next_foreign_food.size() == 1) {
                            min_component_cost = foreign_food[{{x, y}, foreign_components[{x, y}][i]}].second.second;
                        }
                        next_foreign_food.erase(foreign_components[{x, y}][i]);
                        auto a = actions;
                        a.push_back(std::to_string(i));
                        successors.push_back({cost - min_component_cost - food_heuristic({x, y}, next_food, next_foreign_food, this), {{x, y}, next_food, next_foreign_food, i}, a, cost - min_component_cost});
                    }
                }
            }
        }
    }

    return successors;
    
}

std::vector<std::pair<int, int>> PacmanGraph::get_neighbors(std::pair<int, int> state) {
    std::vector<std::pair<int, int>> neighbors;
    for (auto d : dirs) {
        std::pair<int, int> n = {state.first + d.first.first, state.second + d.first.second};
        if (std::find(curr_component.begin(), curr_component.end(), n) != curr_component.end()) {
            neighbors.push_back(n);
        }
    }
    return neighbors;
}

// PacmanGraph::PacmanGraph(Graph &g, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> graph_nodes, std::pair<int, int> start_node) : g(g) {
//     dirs[{0, 1}] = "East";
//     dirs[{0, -1}] = "West";
//     dirs[{1, 0}] = "South";
//     dirs[{-1, 0}] = "North";
//     std::vector<std::vector<int>> matrix = g.matrix();
//     int n = graph_nodes.size() + 1;
//     node_count = n;
//     food = boost::dynamic_bitset<>(n, 0);
//     int bit = 0;
//     for (auto node : graph_nodes) {
//         int i = node.first.first;
//         int j = node.first.second;
//         // // std::cout << i << " " << j << "\n";
//         if (matrix[i][j] > 0) {
//             bitmap[{i, j}] = boost::dynamic_bitset<>(n, 1) << bit;
//             unmap[boost::dynamic_bitset<>(n, 1) << bit] = {i, j};
//             if (matrix[i][j] == 2 && (std::pair<int, int>){i, j} != start_node) {
//                 food ^= bitmap[{i, j}];
//             }
//             bit += 1;
//         }
//     }
//     for (auto n : graph_nodes) {
//         // // std::cout << bit_encode(n.first) << "\n";
//         boost::dynamic_bitset<> b = bit_encode(n.first);
//         nodes[b] = std::vector<boost::dynamic_bitset<>>();
//         for (std::pair<int, int> v : n.second) {
//             nodes[b].push_back(bit_encode(v));
//         }
//     }
//     pac_start = bit_encode(start_node);
//     init_path_memo();
//     get_successors_ = [this](std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>> state, std::vector<std::string> actions, int cost, std::function<int(boost::dynamic_bitset<>, boost::dynamic_bitset<>)> food_heuristic) -> std::vector<std::tuple<int, std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, std::vector<std::string>, int>> {
//         std::vector<std::tuple<int, std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, std::vector<std::string>, int>> successors;
//         for (auto n : nodes[state.first]) {
//             auto n_coord = unmap[n];
//             auto coord = unmap[state.first];
//             std::pair<int, int> d = std::pair<int, int>(n_coord.first - coord.first, n_coord.second - coord.second);
//             std::vector<std::string> a = actions;
//             a.push_back(dirs[d]);
//             successors.push_back({cost - 1 - food_heuristic(n, state.second & (~n)), {n, state.second & (~n)}, a, cost - 1});
//         }
//         return successors;
//     };
// }

// void PacmanGraph::set_food_bit(boost::dynamic_bitset<> node, int bit) {
//     if (bit == 0) {
//         food &= (~node);
//     }
//     else {
//         food |= node;
//     }
// }

PacmanGraph::PacmanGraph(std::vector<std::pair<int, int>> food_list, std::pair<int, int> start_node, std::vector<std::pair<int, int>> component, boost::unordered::unordered_map<std::pair<std::pair<int ,int>, std::vector<std::pair<int, int>>>, std::pair<std::pair<std::pair<int, int>, std::pair<int, int>>, std::pair<int, int>>> foreign_food_map, boost::unordered::unordered_map<std::pair<int, int>, bool> articulation_points, boost::unordered::unordered_map<std::pair<std::pair<int, int>, std::pair<int, int>>, int> pm) {
    // foreign food map bits denote component switch
    curr_component = component;
    // std::cout << "Component size: " << component.size() << "\n";
    node_count = component.size() + foreign_food_map.size() + 1;
    // foreign_food_mask = boost::dynamic_bitset<>(node_count, 0);
    // food = boost::dynamic_bitset<>(node_count, 0);
    // int bit = 0;
    // for (auto n : component) {
    //     bitmap[n] = boost::dynamic_bitset<>(node_count, 1) << bit;
    //     unmap[bitmap[n]] = n;
    //     bit++;
    // }
    // for (auto n : component) { 
    //     for (auto d : dirs) {
    //         std::pair<int, int> neighbor = {n.first + d.first.first, n.second + d.first.second};
    //         if (std::find(component.begin(), component.end(), neighbor) != component.end()) {
    //             nodes[bitmap[n]].push_back(bitmap[neighbor]);
    //         }
    //     }
    // }
    // // std::cout << "Nodes:\n";
    // for (auto kv : nodes) {
    //     // std::cout << kv.first << ": ";
    //     for (auto n : kv.second) {
    //         // std::cout << n << " ";
    //     }
    //     // std::cout << "\n";
    // }
    path_memo = pm;
    // for (auto n : foreign_food_map) {
    //     nodes[bitmap[n.first.first]].push_back(boost::dynamic_bitset<>(node_count, 1) << bit);
    //     nodes[boost::dynamic_bitset<>(node_count, 1) << bit].push_back(bitmap[n.first.first]);
    //     nodes[(boost::dynamic_bitset<>(node_count, 1) << bit) | (boost::dynamic_bitset<>(node_count, 1) << (node_count - 1))].push_back(bitmap[n.first.first]);
    //     food |= boost::dynamic_bitset<>(node_count, 1) << bit;
    //     foreign_food_mask |= boost::dynamic_bitset<>(node_count, 1) << bit;
    //     unmap_foreign_component[boost::dynamic_bitset<>(node_count, 1) << bit] = n.first.second;
    //     // path_memo[{bitmap[n.first.first], boost::dynamic_bitset<>(node_count, 1) << bit}] = n.second.first;
    //     // path_memo[{boost::dynamic_bitset<>(node_count, 1) << bit, bitmap[n.first.first]}] = 0;
    //     // path_memo[{bitmap[n.first.first], (boost::dynamic_bitset<>(node_count, 1) << bit) | (boost::dynamic_bitset<>(node_count, 1) << (node_count - 1))}] = n.second.second;
    //     // path_memo[{(boost::dynamic_bitset<>(node_count, 1) << bit) | (boost::dynamic_bitset<>(node_count, 1) << (node_count - 1)), bitmap[n.first.first]}] = 0;
    //     bit++;
    // }
    a_points = articulation_points;
    // std::cout << "Foreign Food Init\n";
    foreign_food = foreign_food_map;
    // std::cout << foreign_food.size() << "\n";
    // std::cout << "Foreign Food Done\n";
    // for (auto n : food_list) {
    //     assert(food.size() == bitmap[n].size());
    //     food |= bitmap[n];
    // }
    food = food_list;
    // std::cout << "Food Size: " << food.size() << "\n";
    // food |= boost::dynamic_bitset<>(node_count, !last) << node_count - 1;
    pac_start = start_node;
    // food &= ~pac_start;
    if (std::find(food.begin(), food.end(), pac_start) != food.end()) {
        food.erase(std::find(food.begin(), food.end(), pac_start));
    }
    for (auto ac : foreign_food_map) {
        auto a = ac.first.first;
        auto c = ac.first.second;
        foreign_components[a].push_back(c);
    }
    // get_successors_ = [this](std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>> state, std::vector<std::string> actions, int cost, std::function<int(boost::dynamic_bitset<>, boost::dynamic_bitset<>)> food_heuristic) -> std::vector<std::tuple<int, std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, std::vector<std::string>, int>> {
    //     std::vector<std::tuple<int, std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, std::vector<std::string>, int>> successors;
    //     // exit(0);
    //     // std::cout << "get_successors call\n";
    //     // std::cout << state.first << "\n";
    //     // std::cout << state.second << "\n";
    //     // std::cout << cost << "\n";
    //     if ((state.first & foreign_food_mask) == boost::dynamic_bitset(node_count, 0)) {
    //         // std::cout << "No Switchback\n";
    //         // std::cout << nodes[state.first].size() << "\n";
    //         for (auto n : nodes[state.first]) {
    //             // std::cout << n << "\n";
    //             if ((n & foreign_food_mask) == boost::dynamic_bitset(node_count, 0)) {
    //                 // std::cout << "Normal Node\n";
    //                 auto n_coord = unmap[n];
    //                 auto coord = unmap[state.first];
    //                 std::pair<int, int> d = std::pair<int, int>(n_coord.first - coord.first, n_coord.second - coord.second);
    //                 std::vector<std::string> a = actions;
    //                 a.push_back(dirs[d]);
    //                 assert(state.second.size() == n.size());
    //                 successors.push_back({cost - 1 /*- food_heuristic(n, state.second & (~n))*/, {n, state.second & (~n)}, a, cost - 1});
    //             }
    //             else {
    //                 // std::cout << "Component Switch\n";
    //                 std::vector<std::string> a = actions;
    //                 std::string comp_switch;
    //                 comp_switch += "{";
    //                 for (auto h : unmap_foreign_component[n]) {
    //                     comp_switch += h.first;
    //                     comp_switch += " ";
    //                     comp_switch += h.second;
    //                     comp_switch += ", ";
    //                 }
    //                 comp_switch += "}";
    //                 a.push_back(comp_switch);
    //                 assert(state.second.size() == n.size());
    //                 // int p = 1;
    //                 int p = foreign_food[{unencode(state.first), unmap_foreign_component[n]}].first;
    //                 if ((state.second & n) == boost::dynamic_bitset(node_count, 1) << (node_count-1) || (state.second & n) == boost::dynamic_bitset(node_count, 0)) {
    //                     n |= boost::dynamic_bitset(node_count, 1) << (node_count-1);
    //                     p = foreign_food[{unencode(state.first), unmap_foreign_component[n]}].second;
    //                 }
    //                 // std::cout << n << "\n";
    //                 // // std::cout << path_memo[{state.first, n}] << "\n";
    //                 successors.push_back({cost - p, {n, state.second & ~n}, a, cost - p});
    //             }
    //         }
    //     }
    //     else {
    //         // std::cout << "Switchback\n";
    //         std::vector<std::string> a = actions;
    //         std::string comp_switch;
    //         comp_switch += "{";
    //         for (auto h : curr_component) {
    //             comp_switch += h.first;
    //             comp_switch += " ";
    //             comp_switch += h.second;
    //             comp_switch += ", ";
    //         }
    //         comp_switch += "}";
    //         // std::cout << "Component String Done\n";
    //         // std::cout << state.first << "\n";
    //         a.push_back(comp_switch);
    //         assert(state.second.size() == nodes[state.first][0].size());
    //         // std::cout << nodes[state.first].size() << "\n";
    //         auto n = nodes[state.first][0];
    //         int p = 0;//path_memo[{state.first, n}];
    //         // std::cout << p << "\n";
    //         successors.push_back({cost - p, {n, state.second & ~n}, a, cost - p});
    //     }
    // // std::cout << "Successors: " << successors.size() << "\n";
    // for (auto s : successors) {
    //     // std::cout << std::get<0>(s) << "\n";
    // }
    //     return successors;
    // };
}

boost::unordered::unordered_map<std::pair<std::pair<int ,int>, std::vector<std::pair<int, int>>>, std::pair<std::pair<std::pair<int, int>, std::pair<int, int>>, std::pair<int, int>>>* PacmanGraph::get_foreign_food_map() {
    return &foreign_food;
}

PacmanGraph::PacmanGraph() {
}