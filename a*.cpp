#include <vector>
#include "a*.hpp"
#include <boost/unordered_set.hpp>
#include <utility>
#include <boost/dynamic_bitset.hpp>
#include <boost/heap/priority_queue.hpp>
#include <string>
#include <iostream>
using namespace boost;

std::vector<std::string> astar(PacmanGraph pg) {
    auto visited = unordered::unordered_set<std::pair<dynamic_bitset<>, dynamic_bitset<>>>();
    heap::priority_queue<std::tuple<int, std::pair<dynamic_bitset<>, dynamic_bitset<>>, std::vector<std::string>>> heap;
    std::tuple<int, std::pair<dynamic_bitset<>, dynamic_bitset<>>, std::vector<std::string>> start;
    start = std::tuple<int, std::pair<dynamic_bitset<>, dynamic_bitset<>>, std::vector<std::string>>(0, {pg.get_pac_start(), pg.get_food()}, std::vector<std::string>());
    heap.push(start);
    while (!heap.empty()) {
        auto [cost, state, path] = heap.top();
        heap.pop();
        const auto [curr_coord, curr_food] = state;
        if (curr_food == dynamic_bitset<>(pg.num_nodes(), 0)) {
            return path;
        }
        if (visited.contains(state)) {
            continue;
        }
        visited.insert(state);
        for (auto n : pg.get_successors(state, path, cost)) {
            heap.push(n);
        }
    }

    return std::vector<std::string>();
}

int food_search_heuristic(PacmanGraph pg) {
    return 0;
}