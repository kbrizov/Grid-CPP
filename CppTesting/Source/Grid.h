#pragma once

#include "Tile.h"
#include <vector>
#include <string>
#include <unordered_map>

using namespace std;

class Grid
{
public:

	Grid(int rows, int columns);

	vector<Tile>& operator[](const int index);

	int getRows() const;

	int getColumns() const;

	/** Finds the shortest path from Start to End if it exists. Uses Breath first search. */
	vector<const Tile*> findPathBFS(const Tile& start, const Tile& end) const;

	/** Finds a path from Start to End if it exists. Uses Depth first search. */
	vector<const Tile*> findPathDFS(const Tile& start, const Tile& end) const;

	/** Finds the cheapest path from Start to End if it exists. Uses Uniform Cost Search (Special case for Dijkstra's algorithm).*/
	vector<const Tile*> findPathUCS(const Tile& start, const Tile& end) const;

	/** Finds the cheapest path from Start to End if it exists. Uses A*. */
	vector<const Tile*> findPathAStar(const Tile& start, const Tile& end) const;

	unordered_map<const Tile*, float> dijkstraAlgorithm(const Tile& start);

	string toString() const;

private:

	void setRows(int rows);

	void setColumns(int columns);

	vector<const Tile*> getPathTo(const Tile& end, const unordered_map<const Tile*, const Tile*>& visited) const;

	vector<const Tile*> getTileNeighbors(const Tile& tile) const;

	vector<const Tile*> getTileNeighbors(int row, int column) const;

	float getManhattanDistance(const Tile& a, const Tile& b) const;

	unordered_map<const Tile*, float> getInitialCosts() const;

	bool canGetTile(int row, int column) const;

	bool isRowInRange(int row) const;

	bool isColumnInRange(int columnt) const;

private:

	int rows;
	int columns;
	vector<vector<Tile>> grid;
};



