#include "Grid.h"
#include <cassert>
#include <queue>
#include <stack>
#include <unordered_set>
#include <algorithm>
#include <utility>

Grid::Grid(unsigned rows, unsigned columns)
{
	this->setRows(rows);
	this->setColumns(columns);

	m_grid = std::vector<std::vector<Tile>>(rows);

	for (unsigned row = 0; row < rows; row++)
	{
		m_grid[row] = std::vector<Tile>(columns);

		for (unsigned column = 0; column < columns; column++)
		{
			m_grid[row][column] = Tile(row, column);
		}
	}
}

std::vector<Tile>& Grid::operator[](unsigned index)
{
	// A trick I learned from Scott Meyers. The idea is to avoid code duplication.
	// 1. Treat *this as a const in order to call the const version of operator[].
	// 2. Cast away the constness of the result with const_cast.
	return const_cast<std::vector<Tile>&>(static_cast<const Grid&>(*this)[index]);
}

const std::vector<Tile>& Grid::operator[](unsigned index) const
{
	return m_grid[index];
}

std::vector<const Tile*> Grid::findPathBFS(const Tile& start, const Tile& end) const
{
	// BFS start.
	std::queue<const Tile*> frontier;
	frontier.push(&start);

	std::unordered_map<const Tile*, const Tile*> visited;
	visited.insert(std::make_pair(&start, nullptr));

	while (!frontier.empty())
	{
		const Tile* current = frontier.front();
		frontier.pop();

		if (*current == end)
		{
			break;
		}

		std::vector<const Tile*> neighbors = this->getTileNeighbors(*current);

		for (const auto& tile : neighbors)
		{
			const bool isContained = visited.count(tile) > 0;

			if (!isContained)
			{
				frontier.push(tile);
				visited.insert(std::make_pair(tile, current));
			}
		}
	} // BFS end.

	std::vector<const Tile*> path = this->getPathTo(end, visited);

	return path;
}

std::vector<const Tile*> Grid::findPathDFS(const Tile& start, const Tile& end) const
{
	// DFS start.
	std::stack<const Tile*> frontier;
	frontier.push(&start);

	std::unordered_map<const Tile*, const Tile*> visited;
	visited.insert(std::make_pair(&start, nullptr));

	while (!frontier.empty())
	{
		const Tile* current = frontier.top();
		frontier.pop();

		if (*current == end)
		{
			break;
		}

		std::vector<const Tile*> neighbors = this->getTileNeighbors(*current);

		for (const auto& tile : neighbors)
		{
			const bool isContained = visited.count(tile) > 0;

			if (!isContained)
			{
				frontier.push(tile);
				visited.insert(std::make_pair(tile, current));
			}
		}
	} // DFS end.

	std::vector<const Tile*> path = this->getPathTo(end, visited);

	return path;
}

std::vector<const Tile*> Grid::findPathUCS(const Tile& start, const Tile& end) const
{
	// UCS start.
	auto costs = this->getInitialCosts();
	costs[&start] = 0.0f;

	auto heuristicComparer = [&](const Tile* lhs, const Tile* rhs)
	{
		const float lhsPriority = costs[lhs];
		const float rhsPriority = costs[rhs];

		return lhsPriority > rhsPriority;
	};

	auto frontier = std::priority_queue<const Tile*, std::vector<const Tile*>, decltype(heuristicComparer)>(heuristicComparer);
	frontier.push(&start);

	auto visited = std::unordered_map<const Tile*, const Tile*>();
	visited.insert(std::make_pair(&start, nullptr));

	while (!frontier.empty())
	{
		const Tile* current = frontier.top();
		frontier.pop();

		if (*current == end)
		{
			break;
		}

		std::vector<const Tile*> neighbors = this->getTileNeighbors(*current);

		for (const auto& tile : neighbors)
		{
			const float currentCost = costs[tile];
			const float newCost = costs[current] + tile->getWeight();

			const bool isVisited = visited.count(tile) > 0;

			if (newCost < currentCost)
			{
				costs[tile] = newCost;

				// A cheaper path is found, so the tile predecesor must be replaced with the current tile.
				if (isVisited)
				{
					visited[tile] = current;
				}
			}

			if (!isVisited)
			{
				frontier.push(tile);
				visited.insert(std::make_pair(tile, current));
			}
		}
	} // UCS end.

	std::vector<const Tile*> path = this->getPathTo(end, visited);

	return path;
}

std::vector<const Tile*> Grid::findPathAStar(const Tile& start, const Tile& end) const
{
	// A* start.
	auto costs = this->getInitialCosts();
	costs[&start] = 0.0f;

	auto heuristicComparer = [&](const Tile* lhs, const Tile* rhs)
	{
		const float lhsPriority = costs[lhs] + getManhattanDistance(*lhs, end);
		const float rhsPriority = costs[rhs] + getManhattanDistance(*rhs, end);

		return lhsPriority > rhsPriority;
	};

	auto frontier = std::priority_queue<const Tile*, std::vector<const Tile*>, decltype(heuristicComparer)>(heuristicComparer);
	frontier.push(&start);

	auto visited = std::unordered_map<const Tile*, const Tile*>();
	visited.insert(std::make_pair(&start, nullptr));

	while (!frontier.empty())
	{
		const Tile* current = frontier.top();
		frontier.pop();

		if (*current == end)
		{
			break;
		}

		std::vector<const Tile*> neighbors = this->getTileNeighbors(*current);

		for (const auto& tile : neighbors)
		{
			const float currentCost = costs[tile];
			const float newCost = costs[current] + tile->getWeight();

			const bool isVisited = visited.count(tile) > 0;

			if (newCost < currentCost)
			{
				costs[tile] = newCost;

				// A cheaper path is found, so the tile predecesor must be replaced with the current tile.
				if (isVisited)
				{
					visited[tile] = current;
				}
			}

			if (!isVisited)
			{
				frontier.push(tile);
				visited.insert(std::make_pair(tile, current));
			}
		}
	} // A* end.

	std::vector<const Tile*> path = this->getPathTo(end, visited);

	return path;
}

std::unordered_map<const Tile*, float> Grid::dijkstraAlgorithm(const Tile& start) const
{
	// Dijkstra start.
	auto costs = this->getInitialCosts();
	costs[&start] = 0.0f;

	auto heuristicComparer = [&](const Tile* lhs, const Tile* rhs)
	{
		const float lhsPriority = costs[lhs];
		const float rhsPriority = costs[rhs];

		return lhsPriority > rhsPriority;
	};

	auto frontier = std::priority_queue<const Tile*, std::vector<const Tile*>, decltype(heuristicComparer)>(heuristicComparer);
	frontier.push(&start);

	auto visited = std::unordered_map<const Tile*, const Tile*>();
	visited.insert(std::make_pair(&start, nullptr));

	while (!frontier.empty())
	{
		const Tile* current = frontier.top();
		frontier.pop();

		std::vector<const Tile*> neighbors = this->getTileNeighbors(*current);

		for (const auto& tile : neighbors)
		{
			const float currentCost = costs[tile];
			const float newCost = costs[current] + tile->getWeight();

			if (newCost < currentCost)
			{
				costs[tile] = newCost;
			}

			const bool isVisited = visited.count(tile) > 0;

			if (!isVisited)
			{
				frontier.push(tile);
				visited.insert(std::make_pair(tile, current));
			}
		}
	} // Dijkstra end.

	return costs;
}

std::vector<const Tile*> Grid::getTileNeighbors(const Tile& tile) const
{
	const unsigned row = tile.getRow();
	const unsigned column = tile.getColumn();

	return this->getTileNeighbors(row, column);
}

std::vector<const Tile*> Grid::getTileNeighbors(unsigned row, unsigned column) const
{
	assert(isRowInRange(row));
	assert(isColumnInRange(column));

	std::vector<const Tile*> neighbors;

	bool canGetUpperNeighbor = this->canGetTile(row - 1, column);
	//bool canGetUpperRightNeighbor = this->canGetTile(row - 1, column + 1);
	bool canGetRightNeighbor = this->canGetTile(row, column + 1);
	//bool canGetLowerRightNeighbor = this->canGetTile(row + 1, column + 1); 
	bool canGetLowerNeighbor = this->canGetTile(row + 1, column);
	//bool canGetLowerLeftNeighbor = this->canGetTile(row + 1, column - 1); 
	bool canGetLeftNeighbor = this->canGetTile(row, column - 1);
	//bool canGetUpperLeftNeighbor = this->canGetTile(row - 1, column - 1); 

	if (canGetUpperNeighbor)
	{
		const Tile* upperNeighbor = &m_grid[row - 1][column];
		neighbors.push_back(upperNeighbor);
	}

	//if (canGetUpperRightNeighbor) //
	//{
	//    const Tile* upperRightNeighbor = &grid[row - 1][column + 1];
	//    neighbors.push_back(upperRightNeighbor);
	//}

	if (canGetRightNeighbor)
	{
		const Tile* rightNeighbor = &m_grid[row][column + 1];
		neighbors.push_back(rightNeighbor);
	}

	//if (canGetLowerRightNeighbor) //
	//{
	//    const Tile* lowerRightNeighbor = &grid[row + 1][column + 1];
	//    neighbors.push_back(lowerRightNeighbor);
	//}

	if (canGetLowerNeighbor)
	{
		const Tile* lowerNeighbor = &m_grid[row + 1][column];
		neighbors.push_back(lowerNeighbor);
	}

	//if (canGetLowerLeftNeighbor) //
	//{
	//    const Tile* lowerLeftNeighbor = &grid[row + 1][column - 1];
	//    neighbors.push_back(lowerLeftNeighbor);
	//}

	if (canGetLeftNeighbor)
	{
		const Tile* leftNeighbor = &m_grid[row][column - 1];
		neighbors.push_back(leftNeighbor);
	}

	//if (canGetUpperLeftNeighbor) //
	//{
	//    const Tile* upperLeftNeighbor = &grid[row - 1][column - 1];
	//    neighbors.push_back(upperLeftNeighbor);
	//}

	return neighbors;
}

float Grid::getManhattanDistance(const Tile& a, const Tile& b) const
{
	const float aRow = static_cast<float>(a.getRow());
	const float aColomn = static_cast<float>(a.getColumn());

	const float bRow = static_cast<float>(b.getRow());
	const float bColumn = static_cast<float>(b.getColumn());

	const float manhattanDistance = fabs(aRow - bRow) + fabs(aColomn - bColumn);

	return manhattanDistance;
}

std::string Grid::toString() const
{
	std::string result;

	for (unsigned row = 0; row < m_rows; row++)
	{
		for (unsigned column = 0; column < m_columns; column++)
		{
			result.append(m_grid[row][column].toString() + " ");
		}

		result.push_back('\n');
	}

	return result;
}

std::vector<const Tile*> Grid::getPathTo(const Tile& end, const std::unordered_map<const Tile*, const Tile*>& visited) const
{
	assert(visited.size() > 0);

	std::vector<const Tile*> path;

	const Tile* current = nullptr;
	const Tile* previous = nullptr;

	const bool isContained = visited.count(&end) > 0;

	if (isContained)
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

std::unordered_map<const Tile*, float> Grid::getInitialCosts() const
{
	std::unordered_map<const Tile*, float> costs;

	for (unsigned row = 0; row < m_rows; row++)
	{
		for (unsigned column = 0; column < m_columns; column++)
		{
			const Tile* tile = &m_grid[row][column];
			const float cost = std::numeric_limits<float>::infinity();

			costs.insert(std::make_pair(tile, cost));
		}
	}

	return costs;
}
