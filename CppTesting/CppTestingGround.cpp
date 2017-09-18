#include <iostream>
#include "Grid.h"
#include "Tile.h"
#include <set>
#include <unordered_set>
#include <queue>
#include <algorithm>

using namespace std;

int main()
{
	Grid grid = Grid(10, 10);

	cout << grid.toString() << endl;
	cout << endl;

	Tile& start = grid[0][0];
	Tile& end = grid[0][5];

	auto& path = grid.findPathUsingBFS(start, end);

	for_each(path.cbegin(), path.cend(), [](const Tile* tile)
	{
		cout << tile->toString() << " ";
	});

	return 0;
}


