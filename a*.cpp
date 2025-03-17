#include <vector>
#include "a*.hpp"
#include <boost/unordered_set.hpp>
#include <utility>
#include <boost/dynamic_bitset.hpp>
#include <boost/heap/priority_queue.hpp>
#include <string>
#include <iostream>
#include <algorithm>
using namespace boost;

static boost::unordered::unordered_map<std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, int> memo_in_use;
int food_search_heuristic(boost::dynamic_bitset<> position, boost::dynamic_bitset<> food) {
    if (food == dynamic_bitset(food.size(), 0)) {
        return 0;
    }
    int min_distance = 99999;
    int max_distance = -1;
    dynamic_bitset<> min_tile;
    dynamic_bitset<> max_tile;
    auto food_length = food.size();
    dynamic_bitset<> tile = dynamic_bitset<>(food_length, 1);
    while  (food_length > 0) {
        if ((food & tile) == dynamic_bitset<>(food.size(), 0)) {
            food_length -= 1;
            tile <<= 1;
            continue;
        }
        int m = memo_in_use[{position, tile}];
        if (min_distance > m) {
            min_distance = m;
            min_tile = tile;
        }
        if (max_distance < m) {
            max_distance = m;
            max_tile = tile;
        }
        food_length -= 1;
        tile <<= 1;
    }
    return std::max(max_distance, memo_in_use[{min_tile, max_tile}]);
    // return 0;
}

std::vector<std::string> astar(PacmanGraph pg) {
    auto visited = unordered::unordered_set<std::pair<dynamic_bitset<>, dynamic_bitset<>>>();
    heap::priority_queue<std::tuple<int, std::pair<dynamic_bitset<>, dynamic_bitset<>>, std::vector<std::string>, int>> heap;
    std::tuple<int, std::pair<dynamic_bitset<>, dynamic_bitset<>>, std::vector<std::string>, int> start;
    start = std::tuple<int, std::pair<dynamic_bitset<>, dynamic_bitset<>>, std::vector<std::string>, int>(0, {pg.get_pac_start(), pg.get_food()}, std::vector<std::string>(), 0);
    heap.push(start);
    memo_in_use = pg.memo();
    while (!heap.empty()) {
        auto [heur, state, path, cost] = heap.top();
        heap.pop();
        const auto [curr_coord, curr_food] = state;
        if (curr_food == dynamic_bitset<>(pg.num_nodes(), 0)) {
            return path;
        }
        if (visited.contains(state)) {
            continue;
        }
        visited.insert(state);
        for (auto n : pg.get_successors(state, path, cost, food_search_heuristic)) {
            heap.push(n);
        }
    }

    return std::vector<std::string>();
}
