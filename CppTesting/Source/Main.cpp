#include <iostream>
#include <algorithm>
#include "Grid.h"
#include "Tile.h"

using namespace std;

void PrintDijkstraAlgorithm(Grid& grid)
{
	auto costs = grid.dijkstraAlgorithm(grid[2][2]);

	for (int row = 0; row < grid.getRows(); row++)
	{
		for (int column = 0; column < grid.getColumns(); column++)
		{
			cout << costs[&grid[row][column]] << " ";
		}

		cout << endl;
	}
}

void PrintPathBFS(Grid& grid, Tile& start, Tile& end)
{
	auto shortestPath = grid.findPathBFS(start, end);

	cout << endl << "BFS path" << endl;
	for_each(shortestPath.cbegin(), shortestPath.cend(), [](const Tile* tile)
	{
		cout << tile->toString() << " ";
	});

	cout << endl;
}

void PrintPathDijkstra(Grid& grid, Tile& start, Tile& end)
{
	auto cheapestPath = grid.findPathUCS(start, end);

	cout << endl << "Dijkstra path" << endl;
	for_each(cheapestPath.cbegin(), cheapestPath.cend(), [](const Tile* tile)
	{
		cout << tile->toString() << " ";
	});

	cout << endl;
}

void PrintPathAStar(Grid& grid, Tile& start, Tile& end)
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

//priority_queue <Tile*> test1;
//
//auto tileComparer = [](Tile* left, Tile* right) { return *left > *right; };
//priority_queue <Tile*, vector<Tile*>, decltype(tileComparer)> test2(tileComparer);

