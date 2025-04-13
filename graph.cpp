#include "graph.hpp"
#include <iostream>

int count_nodes(std::vector<std::vector<int>> g)
{
	int num_nodes = 0;
	for (int i = 0; i < (int)g.size(); ++i)
	{
		for (int j = 0; j < (int)g[0].size(); ++j)
		{
			if (g[i][j] > 0)
			{
				num_nodes++;
			}
		}
	}
	return num_nodes;
}

Graph::Graph(const std::vector<std::vector<int>> &matrix) : m(matrix)
{
	number_nodes = count_nodes(matrix);
	for (int i = 0; i < (int)matrix.size(); ++i)
	{
		for (int j = 0; j < (int)matrix[i].size(); ++j)
		{
			if (matrix[i][j] >= 1)
			{
				addNode(i, j);
			}
		}
	}
}

int Graph::num_nodes() const
{
	return number_nodes;
}

void Graph::addNode(int x, int y)
{
	nodes[{x, y}] = std::vector<std::pair<int, int>>();
	std::vector<std::pair<int, int>> dirs = {{0, 1}, {1, 0}, {-1, 0}, {0, -1}};
	for (std::pair<int, int> d : dirs) {
		if (x + d.first >= 0 && x + d.first < (int)m.size() && y + d.second >= 0 && y + d.second < (int)m[0].size() && m[x+d.first][y+d.second] != 0) 
		{
			nodes[{x, y}].push_back({x + d.first, y + d.second});
		}
	}
}

const std::vector<std::vector<int>> &Graph::matrix() {
    return m;
}

const boost::unordered::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>> &Graph::get_nodes() {
	return nodes;
}

Graph::Graph() {
	
}

