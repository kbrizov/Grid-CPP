#include <iostream>
#include "Grid.h"
#include "Tile.h"
#include <set>
#include <vector>
#include <unordered_set>
#include <queue>
#include <algorithm>

using namespace std;

int main()
{
	Grid grid = Grid(5, 5);
	cout << grid.toString() << endl;

	auto costs = grid.dijkstraAlgorithm(grid[2][2]);

	for (int row = 0; row < grid.getRows(); row++)
	{
		for (int column = 0; column < grid.getColumns(); column++)
		{
			cout << costs[&grid[row][column]] << " ";
		}

		cout << endl;
	}

	cout << endl << "Cheapest path" << endl;

	auto cheapestPath = grid.findPathDijkstra(grid[0][0], grid[4][4]);

	for_each(cheapestPath.cbegin(), cheapestPath.cend(), [](const Tile* tile)
	{
		cout << tile->toString() << " ";
	});

	cout << endl << "Shortest path" << endl;

	auto shortestPath = grid.findPathBFS(grid[0][0], grid[4][4]);

	for_each(shortestPath.cbegin(), shortestPath.cend(), [](const Tile* tile)
	{
		cout << tile->toString() << " ";
	});

	return 0;
}


//priority_queue <Tile*> test1;
//
//auto tileComparer = [](Tile* left, Tile* right) { return *left > *right; };
//priority_queue <Tile*, vector<Tile*>, decltype(tileComparer)> test2(tileComparer);

