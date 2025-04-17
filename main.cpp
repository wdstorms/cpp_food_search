#include "graph.hpp"
#include "bc_decomposition/bcp_graph.hpp"
#include <iostream>
#include <vector>
#include <functional>
#include <chrono>

int main()
{
	std::vector<std::vector<int>> minimalSearch = {
		{2, 0, 0},
		{3, 0, 0},
		{0, 0, 0}};

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

	Graph graph(minimalSearch);
	auto start = std::chrono::high_resolution_clock::now();
    PacmanGraph pg(graph);
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> dur = end - start;
	std::cout << "PacmanGraph done in: " << dur.count() << " seconds.\n";
	start = std::chrono::high_resolution_clock::now();
	BCPGraph bg(graph);
	bg.treeify();
	end = std::chrono::high_resolution_clock::now();
	dur = end - start;
	std::cout << "Precomputation done in: " << dur.count() << " seconds.\n";

    // for (auto p : bg.t->pg.nodes)  {
    //     std::cout <<  p.first << " " ;
    // }
	// std::cout << bg.t->pg.nodes.size() << "\n";
	// exit(0);
	bg.optimal_path_calc();
	delete bg.t;
	std::function<std::vector<std::string>(PacmanGraph)> solve = astar;
	start = std::chrono::high_resolution_clock::now();
	auto path = solve(pg);
	end = std::chrono::high_resolution_clock::now();
	dur = end - start;
	std::cout << "Path found in: " << dur.count() << " seconds.\n";
	for (auto i : path) {
		std::cout << i + ", ";
	}
	std::cout << '\n';
	std::cout << path.size() << '\n';


	std::cout << "\n";
	for (auto i : bg.t->optimal_cost_and_path(true).second) {
		std::cout << i + ", ";
	}
	std::cout << '\n';
	std::cout << bg.t->optimal_cost_and_path(true).first << '\n';
	return 0;
}