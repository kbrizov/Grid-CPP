#pragma once

#include <cassert>
#include <vector>
#include <string>
#include <unordered_map>
#include "Tile.h"

class Grid
{
public:

	Grid(unsigned rows, unsigned columns);

	unsigned getRows() const;

	unsigned getColumns() const;

	std::vector<Tile>& operator[](unsigned index);

	const std::vector<Tile>& operator[](unsigned index) const;

	/** Finds the shortest path from Start to End if it exists. Uses Breath first search. */
	std::vector<const Tile*> findPathBFS(const Tile& start, const Tile& end) const;

	/** Finds a path from Start to End if it exists. Uses Depth first search. */
	std::vector<const Tile*> findPathDFS(const Tile& start, const Tile& end) const;

	/** Finds the cheapest path from Start to End if it exists. Uses Uniform Cost Search (Special case for Dijkstra's algorithm).*/
	std::vector<const Tile*> findPathUCS(const Tile& start, const Tile& end) const;

	/** Finds the cheapest path from Start to End if it exists. Uses A*. */
	std::vector<const Tile*> findPathAStar(const Tile& start, const Tile& end) const;

	std::unordered_map<const Tile*, float> dijkstraAlgorithm(const Tile& start) const;

	std::string toString() const;

private:

	void setRows(unsigned rows);

	void setColumns(unsigned columns);

	std::vector<const Tile*> getPathTo(const Tile& end, const std::unordered_map<const Tile*, const Tile*>& visited) const;

	std::vector<const Tile*> getTileNeighbors(const Tile& tile) const;

	std::vector<const Tile*> getTileNeighbors(unsigned row, unsigned column) const;

	float getManhattanDistance(const Tile& a, const Tile& b) const;

	std::unordered_map<const Tile*, float> getInitialCosts() const;

	bool canGetTile(unsigned row, unsigned column) const;

	bool isRowInRange(unsigned row) const;

	bool isColumnInRange(unsigned column) const;

private:

	unsigned m_rows;
	unsigned m_columns;
	std::vector<std::vector<Tile>> m_grid;
};

inline unsigned Grid::getRows() const
{
	return m_rows;
}

inline void Grid::setRows(unsigned rows)
{
	assert(0 < rows);
	m_rows = rows;
}

inline unsigned Grid::getColumns() const
{
	return m_columns;
}

inline void Grid::setColumns(unsigned columns)
{
	assert(0 < columns);
	m_columns = columns;
}

inline bool Grid::canGetTile(unsigned row, unsigned column) const
{
	bool isInRange = isRowInRange(row) && isColumnInRange(column);

	return isInRange;
}

inline bool Grid::isRowInRange(unsigned row) const
{
	bool isInRange = (0 <= row && row < m_rows);

	return isInRange;
}

inline bool Grid::isColumnInRange(unsigned column) const
{
	bool isInRange = (0 <= column && column < m_columns);

	return isInRange;
}
