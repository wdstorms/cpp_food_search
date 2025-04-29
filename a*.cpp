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
    bool operator()(const std::tuple<int, std::tuple<std::pair<int, int>, std::vector<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int>, std::vector<std::string>, int>& lhs, const std::tuple<int, std::tuple<std::pair<int, int>, std::vector<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int>, std::vector<std::string>, int>& rhs) const {
        if (std::get<0>(lhs) != std::get<0>(rhs)) {
            return std::get<0>(lhs) < std::get<0>(rhs);
        }
        else if (std::get<0>(std::get<1>(lhs)).first != std::get<0>(std::get<1>(rhs)).first) {
            return std::get<0>(std::get<1>(lhs)).first > std::get<0>(std::get<1>(rhs)).first;
        }
        return std::get<0>(std::get<1>(lhs)).second > std::get<0>(std::get<1>(rhs)).second;
    }
};

static boost::unordered::unordered_map<std::pair<std::pair<int, int>, std::pair<int, int>>, int> memo_in_use;


using namespace boost;
void print_tuple (std::pair<int, int> n) {
    std::cout << "(" << n.first << ", " << n.second << ")";
}
int food_search_heuristic(std::pair<int, int> position, std::vector<std::pair<int, int>> food, boost::unordered::unordered_set<std::vector<std::pair<int, int>>> foreign_food, PacmanGraph* pg) {
    if (food.size() == 0 && foreign_food.size() == 0) {
        return 0;
    }
    // print_tuple(position);
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
    // std::cout << " max_tile: ";
    // print_tuple(max_tile);
    // std::cout << " " << max_distance;
    // std::cout << " min_tile: ";
    // print_tuple(min_tile);
    // std::cout << " " << min_distance;
    // std::cout << "\n";
    auto fmap = pg->get_foreign_food_map();
    for (auto kv : *fmap) {
        if (foreign_food.contains(kv.first.second)) {
            auto min_tile_check = kv.second.first.first;
            auto max_tile_check = kv.second.first.second;
            if (min_distance > memo_in_use[{min_tile_check, position}]) {
                min_distance = memo_in_use[{min_tile_check, position}];
                min_tile = min_tile_check;
            }
            if (max_distance < memo_in_use[{max_tile_check, position}]) {
                max_distance = memo_in_use[{max_tile_check, position}];
                max_tile = max_tile_check;
            }
        }
    }
    return std::max(max_distance, memo_in_use[{min_tile, max_tile}]);
    // return 0;
}

std::vector<std::string> astar(PacmanGraph pg) {
    auto visited = unordered::unordered_set<std::tuple<std::pair<int, int>, std::vector<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int>>();
    heap::priority_queue<std::tuple<int, std::tuple<std::pair<int, int>, std::vector<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int>, std::vector<std::string>, int>, boost::heap::compare<CustomComparator>> heap;
    std::tuple<int, std::tuple<std::pair<int, int>, std::vector<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int>, std::vector<std::string>, int> start;
    start = std::tuple<int, std::tuple<std::pair<int, int>, std::vector<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int>, std::vector<std::string>, int>(0, {pg.get_pac_start(), pg.get_food(), pg.get_foreign_components(), -1}, std::vector<std::string>(), 0);
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
        auto successors = pg.get_successors(state, path, cost, food_search_heuristic);
        // std::cout << "(" << curr_coord.first << ", " << curr_coord.second << "), " << successors.size() << ": ";
        // for (auto n : successors) {
        //     std::cout << "(" << std::get<0>(std::get<1>(n)).first << ", " << std::get<0>(std::get<1>(n)).second << "), " << food_search_heuristic(std::get<0>(std::get<1>(n)), std::get<1>(std::get<1>(n)), std::get<2>(std::get<1>(n)), &pg) << ", ";
        // }
        // std::cout << "\n";
        std::cout << successors.size() << "\n";
        for (auto n : successors) {
            heap.push(n);
        }
    }
    assert(false);
    return std::vector<std::string>();
}
