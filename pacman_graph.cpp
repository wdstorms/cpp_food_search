#include "pacman_graph.hpp"
#include <boost/unordered_set.hpp>
#include <iostream>
#include <functional>
#include <queue>

void PacmanGraph::init_path_memo() {
    for (auto kv : nodes) {
        auto n = kv.first;
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
                path_memo[{n, state}] = depth;
                for (auto node : nodes[state]) {
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

PacmanGraph::PacmanGraph(Graph &g) : g(g){
    dirs[{0, 1}] = "East";
    dirs[{0, -1}] = "West";
    dirs[{1, 0}] = "South";
    dirs[{-1, 0}] = "North";
    std::vector<std::vector<int>> matrix = g.matrix();
    int n = g.num_nodes();
    food = boost::dynamic_bitset<>(n, 0);
    int bit = 0;
    for (int i = 0; i < (int)matrix.size(); i++) {
        for (int j = 0; j < (int)matrix[0].size(); j++) {
            if (matrix[i][j] > 0) {
                bitmap[{i, j}] = boost::dynamic_bitset<>(n, 1) << bit;
                unmap[boost::dynamic_bitset<>(n, 1) << bit] = {i, j};
                if (matrix[i][j] == 2) {
                    food ^= bitmap[{i, j}];
                }
                if (matrix[i][j] == 3) {
                    pac_start = bitmap[{i, j}];
                }
                bit += 1;
            }
        }
    }
    for (auto n : g.get_nodes()) {
        boost::dynamic_bitset<> b = bit_encode(n.first);
        nodes[b] = std::vector<boost::dynamic_bitset<>>();
        for (std::pair<int, int> v : n.second) {
            nodes[b].push_back(bit_encode(v));
        }
    }
    init_path_memo();
}

const std::vector<std::vector<int>> &PacmanGraph::matrix() {
    return g.matrix();
}

const boost::dynamic_bitset<> PacmanGraph::get_pac_start() {
    return pac_start;
}

const boost::dynamic_bitset<> PacmanGraph::get_food() {
    return food;
}

boost::dynamic_bitset<> PacmanGraph::bit_encode(std::pair<int, int> coord) {
    assert(bitmap.contains(coord));
    return bitmap[coord];
}

std::pair<int, int> PacmanGraph::unencode(boost::dynamic_bitset<> bit) {
    assert(unmap.contains(bit));
    return unmap[bit];
}

int PacmanGraph::num_nodes() const {
    return g.num_nodes();
};

boost::unordered::unordered_map<std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, int> PacmanGraph::memo() {
    return path_memo;
}

void PacmanGraph::insert_to_path_memo(boost::dynamic_bitset<> src, boost::dynamic_bitset<> dst, int v) {
    path_memo[{src, dst}] = v;
}

std::vector<std::tuple<int, std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, std::vector<std::string>, int>> PacmanGraph::get_successors(std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>> state, std::vector<std::string> actions, int cost, std::function<int(boost::dynamic_bitset<>, boost::dynamic_bitset<>)> food_heuristic) {
    std::vector<std::tuple<int, std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, std::vector<std::string>, int>> successors;
    for (auto n : nodes[state.first]) {
        auto n_coord = unmap[n];
        auto coord = unmap[state.first];
        std::pair<int, int> d = std::pair<int, int>(n_coord.first - coord.first, n_coord.second - coord.second);
        std::vector<std::string> a = actions;
        a.push_back(dirs[d]);
        successors.push_back({cost - 1 - food_heuristic(n, state.second & (~n)), {n, state.second & (~n)}, a, cost - 1});
    }
    return successors;
}

std::vector<boost::dynamic_bitset<>> PacmanGraph::get_neighbors(boost::dynamic_bitset<> state) {
    return nodes[state];
}

PacmanGraph::PacmanGraph(Graph &g, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> graph_nodes) {
    dirs[{0, 1}] = "East";
    dirs[{0, -1}] = "West";
    dirs[{1, 0}] = "South";
    dirs[{-1, 0}] = "North";
    std::vector<std::vector<int>> matrix = g.matrix();
    int n = nodes.size();
    food = boost::dynamic_bitset<>(n, 0);
    int bit = 0;
    for (auto node : graph_nodes) {
        int i = node.first.first;
        int j = node.first.second;
        if (matrix[i][j] > 0) {
            bitmap[{i, j}] = boost::dynamic_bitset<>(n, 1) << bit;
            unmap[boost::dynamic_bitset<>(n, 1) << bit] = {i, j};
            if (matrix[i][j] == 2) {
                food ^= bitmap[{i, j}];
            }
            if (matrix[i][j] == 3) {
                pac_start = bitmap[{i, j}];
            }
            bit += 1;
        }
    }
    for (auto n : graph_nodes) {
        boost::dynamic_bitset<> b = bit_encode(n.first);
        nodes[b] = std::vector<boost::dynamic_bitset<>>();
        for (std::pair<int, int> v : n.second) {
            nodes[b].push_back(bit_encode(v));
        }
    }
    init_path_memo();
}

PacmanGraph::PacmanGraph() {
}