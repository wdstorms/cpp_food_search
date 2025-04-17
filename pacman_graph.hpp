#include "graph.hpp" 
#include <boost/dynamic_bitset.hpp>
#include <boost/unordered_map.hpp>
#include <vector>
#include <utility>
#include <algorithm>
class PacmanGraph {
    private:
    Graph g;
    boost::unordered::unordered_map<std::pair<int, int>, boost::dynamic_bitset<>> bitmap;
    boost::unordered::unordered_map<boost::dynamic_bitset<>, std::pair<int, int>> unmap;
    boost::dynamic_bitset<> food;
    int node_count;
    boost::dynamic_bitset<> pac_start;
    
    boost::unordered::unordered_map<std::pair<int, int>, std::string> dirs;
    boost::unordered::unordered_map<std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, int> path_memo;
    std::function<std::vector<std::tuple<int, std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, std::vector<std::string>, int>>(std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>> state, std::vector<std::string> actions, int cost, std::function<int(boost::dynamic_bitset<>, boost::dynamic_bitset<>)> food_heuristic)> get_successors_; 
    public:
    boost::unordered::unordered_map<boost::dynamic_bitset<>, std::vector<boost::dynamic_bitset<>>> nodes;
    PacmanGraph();
    PacmanGraph(Graph &g);
    PacmanGraph(Graph &g, boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes, std::pair<int, int> start_node);
    int num_nodes() const;
    const std::vector<std::vector<int>> &matrix();
    const boost::dynamic_bitset<> get_pac_start();
    const boost::dynamic_bitset<> get_food();
    std::pair<int, int> unencode(boost::dynamic_bitset<> bit);
    boost::dynamic_bitset<> bit_encode(std::pair<int, int> coord);
    std::vector<std::tuple<int, std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, std::vector<std::string>, int>> get_successors(std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>> state, std::vector<std::string> actions, int cost, std::function<int(boost::dynamic_bitset<>, boost::dynamic_bitset<>)> food_heuristic); 
    std::vector<boost::dynamic_bitset<>> get_neighbors(boost::dynamic_bitset<> state); 
    void init_path_memo();
    void insert_to_path_memo(boost::dynamic_bitset<> src, boost::dynamic_bitset<> dst, int v);
    boost::unordered::unordered_map<std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, int> memo();
    void set_food_bit(boost::dynamic_bitset<> node, int bit);
};