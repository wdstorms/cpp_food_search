#include <vector>
#include "a*.hpp"
#include <boost/unordered_set.hpp>
#include <utility>
#include <boost/dynamic_bitset.hpp>
#include <boost/heap/priority_queue.hpp>
#include <queue>
#include <string>
#include <iostream>
#include <algorithm>
#include <cassert>
using namespace boost;


struct CustomComparator {
    bool operator()(const std::tuple<int, std::tuple<std::pair<int, int>, boost::unordered::unordered_set<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int>, std::vector<std::string>, int>& lhs, const std::tuple<int, std::tuple<std::pair<int, int>, boost::unordered::unordered_set<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int>, std::vector<std::string>, int>& rhs) const {
        return std::get<0>(lhs) < std::get<0>(rhs);
    }
};

static boost::unordered::unordered_map<std::pair<std::pair<int, int>, std::pair<int, int>>, int> memo_in_use;


using namespace boost;
int food_search_heuristic(std::pair<int, int> position, boost::unordered::unordered_set<std::pair<int, int>> food) {
    if (food.size() == 0) {
        return 0;
    }
    int min_distance = 99999;
    int max_distance = -1;
    std::pair<int ,int> min_tile;
    std::pair<int ,int> max_tile;
    for (auto tile : food) {
        int m = memo_in_use[{position, tile}];
        if (min_distance > m) {
            min_distance = m;
            min_tile = tile;
        }
        if (max_distance < m) {
            max_distance = m;
            max_tile = tile;
        }
    }
    return std::max(max_distance, memo_in_use[{min_tile, max_tile}]);
    // return 0;
}

std::vector<std::string> astar(PacmanGraph pg) {
    auto visited = unordered::unordered_set<std::tuple<std::pair<int, int>, boost::unordered::unordered_set<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int>>();
    heap::priority_queue<std::tuple<int, std::tuple<std::pair<int, int>, boost::unordered::unordered_set<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int>, std::vector<std::string>, int>, boost::heap::compare<CustomComparator>> heap;
    std::tuple<int, std::tuple<std::pair<int, int>, boost::unordered::unordered_set<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int>, std::vector<std::string>, int> start;
    start = std::tuple<int, std::tuple<std::pair<int, int>, boost::unordered::unordered_set<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int>, std::vector<std::string>, int>(0, {pg.get_pac_start(), pg.get_food(), pg.get_foreign_components(), -1}, std::vector<std::string>(), 0);
    // // std::cout << pg.get_pac_start() << "\n";
    heap.push(start);
    memo_in_use = pg.memo();
    // // std::cout << "Memo in use:\n";
    // for (auto kv : memo_in_use) {
    //     // std::cout << kv.first.first  << " " << kv.first.second << " " << kv.second << "\n";
    // }
    // std::cout << "\n";
    while (!heap.empty()) {
        auto [heur, state, path, cost] = heap.top();
        heap.pop();
        const auto [curr_coord, curr_food, curr_foreign_food, component] = state;
        // assert((int)curr_food.size() == pg.num_nodes());
        if (curr_food.size() == 0 && curr_foreign_food.size() == 0) {
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
    assert(false);
    return std::vector<std::string>();
}
