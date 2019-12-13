#include <iostream>
#include <algorithm>
#include "Grid.h"
#include "Tile.h"

using namespace std;

void PrintDijkstraAlgorithm(const Grid& grid)
{
	auto costs = grid.dijkstraAlgorithm(grid[2][2]);

	for (unsigned row = 0; row < grid.getRows(); row++)
	{
		for (unsigned column = 0; column < grid.getColumns(); column++)
		{
			cout << costs[&grid[row][column]] << " ";
		}

		cout << endl;
	}
}

void PrintPathBFS(const Grid& grid, const Tile& start, const Tile& end)
{
	auto shortestPath = grid.findPathBFS(start, end);

	cout << endl << "BFS path" << endl;
	for_each(shortestPath.cbegin(), shortestPath.cend(), [](const Tile* tile)
	{
		cout << tile->toString() << " ";
	});

	cout << endl;
}

void PrintPathDijkstra(const Grid& grid, const Tile& start, const Tile& end)
{
	auto cheapestPath = grid.findPathUCS(start, end);

	cout << endl << "Dijkstra path" << endl;
	for_each(cheapestPath.cbegin(), cheapestPath.cend(), [](const Tile* tile)
	{
		cout << tile->toString() << " ";
	});

	cout << endl;
}

void PrintPathAStar(const Grid& grid, const Tile& start, const Tile& end)
{
	auto cheapestPath = grid.findPathAStar(start, end);

	cout << endl << "A* path" << endl;
	for_each(cheapestPath.cbegin(), cheapestPath.cend(), [](const Tile* tile)
	{
		cout << tile->toString() << " ";
	});
	
	cout << endl;
}

int main()
{
	Grid grid = Grid(5, 5);

	Tile& start = grid[0][0];
	Tile& end = grid[3][3];

	cout << grid.toString() << endl;

	PrintDijkstraAlgorithm(grid);
	PrintPathBFS(grid, start, end);
	PrintPathDijkstra(grid, start, end);
	PrintPathAStar(grid, start, end);

	return 0;
}

