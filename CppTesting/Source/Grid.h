#pragma once

#include <cassert>
#include <vector>
#include <string>
#include <unordered_map>
#include "Tile.h"

class GRID
{
public:

	GRID(unsigned rows, unsigned columns);

	unsigned get_rows() const;

	unsigned get_columns() const;

	std::vector<TILE>& operator[](unsigned index);

	const std::vector<TILE>& operator[](unsigned index) const;

	/** Finds the shortest path from Start to End if it exists. Uses Breath first search. */
	std::vector<const TILE*> find_path_bfs(const TILE& start, const TILE& end) const;

	/** Finds a path from Start to End if it exists. Uses Depth first search. */
	std::vector<const TILE*> find_path_dfs(const TILE& start, const TILE& end) const;

	/** Finds the cheapest path from Start to End if it exists. Uses Uniform Cost Search (Special case for Dijkstra's algorithm).*/
	std::vector<const TILE*> find_path_ucs(const TILE& start, const TILE& end) const;

	/** Finds the cheapest path from Start to End if it exists. Uses A*. */
	std::vector<const TILE*> find_path_astar(const TILE& start, const TILE& end) const;

	std::unordered_map<const TILE*, float> dijkstra_algorithm(const TILE& start) const;

	std::string to_string() const;

private:

	void set_rows(unsigned rows);

	void set_columns(unsigned columns);

	std::vector<const TILE*> get_path_to(const TILE& end, const std::unordered_map<const TILE*, const TILE*>& visited) const;

	std::vector<const TILE*> get_tile_neighbors(const TILE& tile, bool should_include_diagonals = false) const;

	std::vector<const TILE*> get_tile_neighbors(unsigned row, unsigned column, bool should_include_diagonals = false) const;

	float get_manhattan_distance(const TILE& a, const TILE& b) const;

	std::unordered_map<const TILE*, float> get_initial_costs() const;

	bool can_get_tile(unsigned row, unsigned column) const;

	bool is_row_in_range(unsigned row) const;

	bool is_column_in_range(unsigned column) const;

private:

	unsigned m_rows;
	unsigned m_columns;
	std::vector<std::vector<TILE>> m_grid;
};

inline unsigned GRID::get_rows() const
{
	return m_rows;
}

inline void GRID::set_rows(unsigned rows)
{
	assert(0 < rows);
	m_rows = rows;
}

inline unsigned GRID::get_columns() const
{
	return m_columns;
}

inline void GRID::set_columns(unsigned columns)
{
	assert(0 < columns);
	m_columns = columns;
}

inline bool GRID::can_get_tile(unsigned row, unsigned column) const
{
	const bool is_in_range = is_row_in_range(row) && is_column_in_range(column);

	return is_in_range;
}

inline bool GRID::is_row_in_range(unsigned row) const
{
	const bool is_in_range = (0 <= row && row < m_rows);

	return is_in_range;
}

inline bool GRID::is_column_in_range(unsigned column) const
{
	const bool is_in_range = (0 <= column && column < m_columns);

	return is_in_range;
}
