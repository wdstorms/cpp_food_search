#include "graph.hpp" 
#include <boost/dynamic_bitset.hpp>
#include <boost/unordered_map.hpp>
#include <vector>
#include <utility>
class PacmanGraph {
    private:
    Graph g;
    boost::unordered::unordered_map<std::pair<int, int>, boost::dynamic_bitset<>> bitmap;
    boost::unordered::unordered_map<boost::dynamic_bitset<>, std::pair<int, int>> unmap;
    boost::dynamic_bitset<> food;
    boost::dynamic_bitset<> pac_start;
    std::vector<std::vector<int>> walls;
    boost::unordered::unordered_map<boost::dynamic_bitset<>, std::vector<boost::dynamic_bitset<>>> nodes;
    boost::unordered::unordered_map<std::pair<int, int>, std::string> dirs;
    public:
    PacmanGraph(Graph &g);
    int num_nodes() const;
    const std::vector<std::vector<int>> &matrix();
    const std::vector<std::vector<int>> &get_walls();
    const boost::dynamic_bitset<> get_pac_start();
    const boost::dynamic_bitset<> get_food();
    std::pair<int, int> unencode(boost::dynamic_bitset<> bit);
    boost::dynamic_bitset<> bit_encode(std::pair<int, int> coord);
    std::vector<std::tuple<int, std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>>, std::vector<std::string>>> get_successors(std::pair<boost::dynamic_bitset<>, boost::dynamic_bitset<>> state, std::vector<std::string> actions, int cost); 
};