#include <iostream>
#include <algorithm>
#include "Grid.h"
#include "Tile.h"

void print_dijkstra_algorithm(const GRID& grid, const TILE& start)
{
	auto costs = grid.dijkstra_algorithm(start);

	for (unsigned row = 0; row < grid.get_rows(); row++)
	{
		for (unsigned column = 0; column < grid.get_columns(); column++)
		{
			const TILE* tile = &grid[row][column];
			std::cout << costs[tile] << " ";
		}

		std::cout << std::endl;
	}
}

void print_path_bfs(const GRID& grid, const TILE& start, const TILE& end)
{
	const auto shortest_path = grid.find_path_bfs(start, end);

	std::cout << std::endl << "BFS path" << std::endl;

	std::for_each(shortest_path.cbegin(), shortest_path.cend(), [](const TILE* tile)
	{
		std::cout << tile->to_string() << " ";
	});

	std::cout << std::endl;
}

void print_path_dijkstra(const GRID& grid, const TILE& start, const TILE& end)
{
	const auto cheapest_path = grid.find_path_ucs(start, end);

	std::cout << std::endl << "Dijkstra path" << std::endl;

	std::for_each(cheapest_path.cbegin(), cheapest_path.cend(), [](const TILE* tile)
	{
		std::cout << tile->to_string() << " ";
	});

	std::cout << std::endl;
}

void print_path_astar(const GRID& grid, const TILE& start, const TILE& end)
{
	const auto cheapest_path = grid.find_path_astar(start, end);

	std::cout << std::endl << "A* path" << std::endl;

	std::for_each(cheapest_path.cbegin(), cheapest_path.cend(), [](const TILE* tile)
	{
		std::cout << tile->to_string() << " ";
	});
	
	std::cout << std::endl;
}

int main()
{
	const GRID grid = GRID(5, 5);

	std::cout << grid.to_string() << std::endl;

	print_dijkstra_algorithm(grid, grid[2][2]);

	const TILE& start = grid[0][0];
	const TILE& end = grid[3][3];

	print_path_bfs(grid, start, end);
	print_path_dijkstra(grid, start, end);
	print_path_astar(grid, start, end);

	return 0;
}

