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

	vector<const Tile*> findPathUsingBFS(const Tile& start, const Tile& end);

	vector<const Tile*> findPathUsingDFS(const Tile& start, const Tile& end);

	string toString() const;

private:

	void setRows(int rows);

	void setColumns(int columns);

	vector<const Tile*> getPathTo(const Tile& end, const unordered_map<const Tile*, const Tile*>& visited) const;

	vector<const Tile*> getTileNeighbors(const Tile& tile) const;

	vector<const Tile*> getTileNeighbors(int row, int column) const;

	bool canGetTile(int row, int column) const;

private:

	int rows;
	int columns;
	vector<vector<Tile>> grid;
};



