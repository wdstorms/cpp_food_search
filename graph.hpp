#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <utility>
#include <boost/unordered_map.hpp>

class Graph
{
private:
	int number_nodes;
	boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> nodes;
    std::vector<std::vector<int>> m;
public:
	Graph(const std::vector<std::vector<int>> &matrix);
	int num_nodes() const;
	void addNode(int x, int y);
    const std::vector<std::vector<int>> &matrix();
	const boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> &get_nodes();
};

#endif