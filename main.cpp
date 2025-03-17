#include "graph.hpp"
#include "a*.hpp"
#include <iostream>
#include <vector>
#include <functional>
#include <chrono>

int main()
{
	std::vector<std::vector<int>> minimalSearch = {
		{2, 0, 0},
		{3, 1, 2},
		{0, 2, 0}};

	std::vector<std::vector<int>> tinySearch = {
		{2,2,1,1,1,2,2},
		{0,0,0,2,0,0,1},
		{1,1,1,3,1,1,1},
		{2,0,0,1,0,0,2},
		{2,0,2,1,1,1,2}
	};

	std::vector<std::vector<int>> trickySearch = {
		{2,1,1,1,1,1,1,1,1,1,1,1,2,2,0,1,1,1},
		{2,0,0,2,0,0,2,0,0,2,0,0,2,0,0,1,0,1},
		{1,1,1,1,1,1,1,1,3,1,1,1,1,1,1,1,0,1},
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
		{2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1}
	};

	std::vector<std::vector<int>> smallSearch = {
		{2,1,1,1,1,1,1,1,1,1,1,1,2,2,2,3,1,2},
		{2,0,0,2,0,0,2,0,0,2,0,0,2,0,0,1,0,2},
		{1,0,0,1,0,2,2,2,2,2,1,1,1,1,1,1,0,2},
	};

	Graph graph(smallSearch);

    PacmanGraph pg(graph);
	std::function<std::vector<std::string>(PacmanGraph)> solve = astar;
	auto start = std::chrono::high_resolution_clock::now();
	auto path = solve(pg);
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> dur = end - start;
	std::cout << "Path found in: " << dur.count() << " seconds.\n";
	for (auto i : path) {
		std::cout << i + ", ";
	}
	std::cout << '\n';
	std::cout << path.size() << '\n';
	return 0;
}