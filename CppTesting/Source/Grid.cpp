#include "Grid.h"
#include <cassert>
#include <queue>
#include <stack>
#include <unordered_set>
#include <algorithm>
#include <utility>

GRID::GRID(unsigned rows, unsigned columns)
{
	this->set_rows(rows);
	this->set_columns(columns);

	m_grid = std::vector<std::vector<TILE>>(rows);

	for (unsigned row = 0; row < rows; row++)
	{
		m_grid[row] = std::vector<TILE>(columns);

		for (unsigned column = 0; column < columns; column++)
		{
			m_grid[row][column] = TILE(row, column);
		}
	}
}

std::vector<TILE>& GRID::operator[](unsigned index)
{
	// A trick I learned from Scott Meyers. The idea is to avoid code duplication.
	// 1. Treat *this as a const in order to call the const version of operator[].
	// 2. Cast away the constness of the result with const_cast.
	return const_cast<std::vector<TILE>&>(static_cast<const GRID&>(*this)[index]);
}

const std::vector<TILE>& GRID::operator[](unsigned index) const
{
	return m_grid[index];
}

std::vector<const TILE*> GRID::find_path_bfs(const TILE& start, const TILE& end) const
{
	// BFS start.
	std::queue<const TILE*> frontier;
	frontier.push(&start);

	std::unordered_map<const TILE*, const TILE*> visited;
	visited.insert(std::make_pair(&start, nullptr));

	while (!frontier.empty())
	{
		const TILE* current = frontier.front();
		frontier.pop();

		if (*current == end)
		{
			break;
		}

		std::vector<const TILE*> neighbors = this->get_tile_neighbors(*current);

		for (const auto& tile : neighbors)
		{
			const bool is_contained = visited.count(tile) > 0;

			if (!is_contained)
			{
				frontier.push(tile);
				visited.insert(std::make_pair(tile, current));
			}
		}
	} // BFS end.

	std::vector<const TILE*> path = this->get_path_to(end, visited);

	return path;
}

std::vector<const TILE*> GRID::find_path_dfs(const TILE& start, const TILE& end) const
{
	// DFS start.
	std::stack<const TILE*> frontier;
	frontier.push(&start);

	std::unordered_map<const TILE*, const TILE*> visited;
	visited.insert(std::make_pair(&start, nullptr));

	while (!frontier.empty())
	{
		const TILE* current = frontier.top();
		frontier.pop();

		if (*current == end)
		{
			break;
		}

		std::vector<const TILE*> neighbors = this->get_tile_neighbors(*current);

		for (const auto& tile : neighbors)
		{
			const bool is_contained = visited.count(tile) > 0;

			if (!is_contained)
			{
				frontier.push(tile);
				visited.insert(std::make_pair(tile, current));
			}
		}
	} // DFS end.

	std::vector<const TILE*> path = this->get_path_to(end, visited);

	return path;
}

std::vector<const TILE*> GRID::find_path_ucs(const TILE& start, const TILE& end) const
{
	// UCS start.
	auto costs = this->get_initial_costs();
	costs[&start] = 0.0f;

	auto heuristic_comparer = [&](const TILE* lhs, const TILE* rhs)
	{
		const float lhs_priority = costs[lhs];
		const float rhs_priority = costs[rhs];

		return lhs_priority > rhs_priority;
	};

	auto frontier = std::priority_queue<const TILE*, std::vector<const TILE*>, decltype(heuristic_comparer)>(heuristic_comparer);
	frontier.push(&start);

	auto visited = std::unordered_map<const TILE*, const TILE*>();
	visited.insert(std::make_pair(&start, nullptr));

	while (!frontier.empty())
	{
		const TILE* current = frontier.top();
		frontier.pop();

		if (*current == end)
		{
			break;
		}

		std::vector<const TILE*> neighbors = this->get_tile_neighbors(*current);

		for (const auto& tile : neighbors)
		{
			const float current_cost = costs[tile];
			const float new_cost = costs[current] + tile->get_weight();

			const bool is_visited = visited.count(tile) > 0;

			if (new_cost < current_cost)
			{
				costs[tile] = new_cost;

				// A cheaper path is found, so the tile predecesor must be replaced with the current tile.
				if (is_visited)
				{
					visited[tile] = current;
				}
			}

			if (!is_visited)
			{
				frontier.push(tile);
				visited.insert(std::make_pair(tile, current));
			}
		}
	} // UCS end.

	std::vector<const TILE*> path = this->get_path_to(end, visited);

	return path;
}

std::vector<const TILE*> GRID::find_path_astar(const TILE& start, const TILE& end) const
{
	// A* start.
	auto costs = this->get_initial_costs();
	costs[&start] = 0.0f;

	auto heuristic_comparer = [&](const TILE* lhs, const TILE* rhs)
	{
		const float lhs_priority = costs[lhs] + get_manhattan_distance(*lhs, end);
		const float rhs_priority = costs[rhs] + get_manhattan_distance(*rhs, end);

		return lhs_priority > rhs_priority;
	};

	auto frontier = std::priority_queue<const TILE*, std::vector<const TILE*>, decltype(heuristic_comparer)>(heuristic_comparer);
	frontier.push(&start);

	auto visited = std::unordered_map<const TILE*, const TILE*>();
	visited.insert(std::make_pair(&start, nullptr));

	while (!frontier.empty())
	{
		const TILE* current = frontier.top();
		frontier.pop();

		if (*current == end)
		{
			break;
		}

		std::vector<const TILE*> neighbors = this->get_tile_neighbors(*current);

		for (const auto& tile : neighbors)
		{
			const float current_cost = costs[tile];
			const float new_cost = costs[current] + tile->get_weight();

			const bool is_visited = visited.count(tile) > 0;

			if (new_cost < current_cost)
			{
				costs[tile] = new_cost;

				// A cheaper path is found, so the tile predecesor must be replaced with the current tile.
				if (is_visited)
				{
					visited[tile] = current;
				}
			}

			if (!is_visited)
			{
				frontier.push(tile);
				visited.insert(std::make_pair(tile, current));
			}
		}
	} // A* end.

	std::vector<const TILE*> path = this->get_path_to(end, visited);

	return path;
}

std::unordered_map<const TILE*, float> GRID::dijkstra_algorithm(const TILE& start) const
{
	// Dijkstra start.
	auto costs = this->get_initial_costs();
	costs[&start] = 0.0f;

	auto heuristic_comparer = [&](const TILE* lhs, const TILE* rhs)
	{
		const float lhs_priority = costs[lhs];
		const float rhs_priority = costs[rhs];

		return lhs_priority > rhs_priority;
	};

	auto frontier = std::priority_queue<const TILE*, std::vector<const TILE*>, decltype(heuristic_comparer)>(heuristic_comparer);
	frontier.push(&start);

	auto visited = std::unordered_map<const TILE*, const TILE*>();
	visited.insert(std::make_pair(&start, nullptr));

	while (!frontier.empty())
	{
		const TILE* current = frontier.top();
		frontier.pop();

		std::vector<const TILE*> neighbors = this->get_tile_neighbors(*current);

		for (const auto& tile : neighbors)
		{
			const float current_cost = costs[tile];
			const float new_cost = costs[current] + tile->get_weight();

			if (new_cost < current_cost)
			{
				costs[tile] = new_cost;
			}

			const bool is_visited = visited.count(tile) > 0;

			if (!is_visited)
			{
				frontier.push(tile);
				visited.insert(std::make_pair(tile, current));
			}
		}
	} // Dijkstra end.

	return costs;
}

std::vector<const TILE*> GRID::get_tile_neighbors(const TILE& tile, bool should_include_diagonals) const
{
	const unsigned row = tile.get_row();
	const unsigned column = tile.get_column();

	return this->get_tile_neighbors(row, column, should_include_diagonals);
}

std::vector<const TILE*> GRID::get_tile_neighbors(unsigned row, unsigned column, bool should_include_diagonals) const
{
	assert(is_row_in_range(row));
	assert(is_column_in_range(column));

	std::vector<const TILE*> neighbors;

	const bool can_get_upper_neighbor = this->can_get_tile(row - 1, column);
	if (can_get_upper_neighbor)
	{
		const TILE* upper_neighbor = &m_grid[row - 1][column];
		neighbors.push_back(upper_neighbor);
	}

	const bool can_get_right_neighbor = this->can_get_tile(row, column + 1);
	if (can_get_right_neighbor)
	{
		const TILE* right_neighbor = &m_grid[row][column + 1];
		neighbors.push_back(right_neighbor);
	}

	const bool can_get_lower_neighbor = this->can_get_tile(row + 1, column);
	if (can_get_lower_neighbor)
	{
		const TILE* lower_neighbor = &m_grid[row + 1][column];
		neighbors.push_back(lower_neighbor);
	}

	const bool can_get_left_neighbor = this->can_get_tile(row, column - 1);
	if (can_get_left_neighbor)
	{
		const TILE* left_neighbor = &m_grid[row][column - 1];
		neighbors.push_back(left_neighbor);
	}

	if (should_include_diagonals)
	{
		const bool can_get_upper_right_neighbor = this->can_get_tile(row - 1, column + 1);
		if (can_get_upper_right_neighbor)
		{
			const TILE* upper_right_neighbor = &m_grid[row - 1][column + 1];
			neighbors.push_back(upper_right_neighbor);
		}

		const bool can_get_lower_right_neighbor = this->can_get_tile(row + 1, column + 1);
		if (can_get_lower_right_neighbor)
		{
			const TILE* lower_right_neighbor = &m_grid[row + 1][column + 1];
			neighbors.push_back(lower_right_neighbor);
		}

		const bool can_get_lower_left_neighbor = this->can_get_tile(row + 1, column - 1);
		if (can_get_lower_left_neighbor)
		{
			const TILE* lower_left_neighbor = &m_grid[row + 1][column - 1];
			neighbors.push_back(lower_left_neighbor);
		}

		const bool can_get_upper_left_neighbor = this->can_get_tile(row - 1, column - 1);
		if (can_get_upper_left_neighbor)
		{
			const TILE* upper_left_neighbor = &m_grid[row - 1][column - 1];
			neighbors.push_back(upper_left_neighbor);
		}
	}

	return neighbors;
}

float GRID::get_manhattan_distance(const TILE& a, const TILE& b) const
{
	const float a_row = static_cast<float>(a.get_row());
	const float a_column = static_cast<float>(a.get_column());

	const float b_row = static_cast<float>(b.get_row());
	const float b_column = static_cast<float>(b.get_column());

	const float manhattan_distance = fabs(a_row - b_row) + fabs(a_column - b_column);

	return manhattan_distance;
}

std::string GRID::to_string() const
{
	std::string result;

	for (unsigned row = 0; row < m_rows; row++)
	{
		for (unsigned column = 0; column < m_columns; column++)
		{
			result.append(m_grid[row][column].to_string() + " ");
		}

		result.push_back('\n');
	}

	return result;
}

std::vector<const TILE*> GRID::get_path_to(const TILE& end, const std::unordered_map<const TILE*, const TILE*>& visited) const
{
	assert(visited.size() > 0);

	std::vector<const TILE*> path;

	const TILE* current = nullptr;
	const TILE* previous = nullptr;

	const bool is_contained = visited.count(&end) > 0;

	if (is_contained)
	{
		current = &end;
		previous = visited.at(current); // Getting the previous tile of the end tile.
	}

	while (previous != nullptr) // While we reach the start.
	{
		path.push_back(current);

		current = previous;
		previous = visited.at(current);
	}

	path.push_back(current);

	reverse(path.begin(), path.end());

	return path;
}

std::unordered_map<const TILE*, float> GRID::get_initial_costs() const
{
	std::unordered_map<const TILE*, float> costs;

	for (unsigned row = 0; row < m_rows; row++)
	{
		for (unsigned column = 0; column < m_columns; column++)
		{
			const TILE* tile = &m_grid[row][column];
			constexpr float cost = std::numeric_limits<float>::infinity();

			costs.insert(std::make_pair(tile, cost));
		}
	}

	return costs;
}
