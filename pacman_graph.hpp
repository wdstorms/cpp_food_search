#include "graph.hpp" 
#include <boost/dynamic_bitset.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <vector>
#include <utility>
#include <algorithm>
#include "dirs.hpp"
class PacmanGraph {
    private:
    // Graph g;
    boost::unordered::unordered_map<std::pair<std::pair<int ,int>, std::vector<std::pair<int, int>>>, std::pair<int, int>> foreign_food;
    // boost::unordered::unordered_map<std::pair<int, int>, boost::dynamic_bitset<>> bitmap;
    // boost::unordered::unordered_map<boost::dynamic_bitset<>, std::pair<int, int>> unmap;
    // boost::unordered::unordered_map<boost::dynamic_bitset<>, std::vector<std::pair<int, int>>> unmap_foreign_component;
    boost::unordered::unordered_set<std::pair<int, int>> food;
    std::vector<std::vector<std::pair<int, int>>> foreign_food_list;
    std::vector<std::pair<int, int>> curr_component;
    boost::unordered::unordered_map<std::pair<int ,int>, std::vector<std::vector<std::pair<int ,int>>>> foreign_components;
    boost::unordered::unordered_map<std::pair<int, int>, bool> a_points;
    int node_count;
    std::pair<int, int> pac_start;
    boost::unordered::unordered_map<std::pair<std::pair<int, int>, std::pair<int, int>>, int> path_memo;
    std::function<std::vector<std::tuple<int, std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, std::vector<std::string>, int>>(std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>> state, std::vector<std::string> actions, int cost, std::function<int(boost::dynamic_bitset<>, boost::dynamic_bitset<>)> food_heuristic)> get_successors_; 
    public:
    PacmanGraph();
    PacmanGraph(Graph &g);
    PacmanGraph(Graph &g, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes, std::pair<int, int> start_node);
    PacmanGraph(boost::unordered::unordered_set<std::pair<int, int>> food_list, std::pair<int, int> start_node, std::vector<std::pair<int, int>> component, boost::unordered::unordered_map<std::pair<std::pair<int ,int>, std::vector<std::pair<int, int>>>, std::pair<int, int>> foreign_food_map, boost::unordered::unordered_map<std::pair<int, int>, bool> articulation_points);
    int num_nodes() const;
    // const std::vector<std::vector<int>> &matrix();
    const std::pair<int, int> get_pac_start();
    const boost::unordered::unordered_set<std::pair<int, int>> get_food();
    const boost::unordered::unordered_set<std::vector<std::pair<int ,int>>> get_foreign_components();
    // std::pair<int, int> unencode(boost::dynamic_bitset<> bit);
    // boost::dynamic_bitset<> bit_encode(std::pair<int, int> coord);
    std::vector<std::tuple<int, std::tuple<std::pair<int, int>, boost::unordered::unordered_set<std::pair<int, int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int>, std::vector<std::string>, int>> get_successors(std::tuple<std::pair<int, int>, boost::unordered::unordered_set<std::pair<int ,int>>, boost::unordered::unordered_set<std::vector<std::pair<int ,int>>>, int> state, std::vector<std::string> actions, int cost, std::function<int(std::pair<int ,int>, boost::unordered::unordered_set<std::pair<int, int>>)> food_heuristic); 
    std::vector<std::pair<int ,int>> get_neighbors(std::pair<int, int> state); 
    void init_path_memo();
    // void insert_to_path_memo(boost::dynamic_bitset<> src, boost::dynamic_bitset<> dst, int v);
    boost::unordered::unordered_map<std::pair<std::pair<int, int>, std::pair<int, int>>, int> memo();
    // void set_food_bit(boost::dynamic_bitset<> node, int bit);
};