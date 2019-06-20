#include "Grid.h"
#include <cassert>
#include <queue>
#include <stack>
#include <unordered_set>
#include <algorithm>

Grid::Grid(int rows, int columns)
{
	this->setRows(rows);
	this->setColumns(columns);

	this->grid = vector<vector<Tile>>(rows);

	for (int row = 0; row < rows; row++)
	{
		this->grid[row] = vector<Tile>(columns);

		for (int column = 0; column < columns; column++)
		{
			grid[row][column] = Tile(row, column);
		}
	}
}

vector<Tile>& Grid::operator[](const int index)
{
	return this->grid[index];
}

int Grid::getRows() const
{
	return this->rows;
}

int Grid::getColumns() const
{
	return this->columns;
}

std::vector<const Tile*> Grid::findPathBFS(const Tile& start, const Tile& end) const
{
	assert(&start != nullptr);
	assert(&end != nullptr);

	// BFS start.
	queue<const Tile*> frontier;
	frontier.push(&start);

	unordered_map<const Tile*, const Tile*> visited;
	visited.insert(make_pair(&start, nullptr));

	while (!frontier.empty())
	{
		const Tile* current = frontier.front();
		frontier.pop();

		if (*current == end)
		{
			break;
		}

		vector<const Tile*> neighbors = this->getTileNeighbors(*current);
		for (const auto& tile : neighbors)
		{
			bool isContained = visited.count(tile) > 0;

			if (!isContained)
			{
				frontier.push(tile);
				visited.insert(make_pair(tile, current));
			}
		}
	} // BFS end.

	vector<const Tile*> path = this->getPathTo(end, visited);

	return path;
}

std::vector<const Tile*> Grid::findPathDFS(const Tile& start, const Tile& end) const
{
	assert(&start != nullptr);
	assert(&end != nullptr);

	// DFS start.
	stack<const Tile*> frontier;
	frontier.push(&start);

	unordered_map<const Tile*, const Tile*> visited;
	visited.insert(make_pair(&start, nullptr));

	while (!frontier.empty())
	{
		const Tile* current = frontier.top();
		frontier.pop();

		if (*current == end)
		{
			break;
		}

		vector<const Tile*> neighbors = this->getTileNeighbors(*current);
		for (const auto& tile : neighbors)
		{
			bool isContained = visited.count(tile) > 0;

			if (!isContained)
			{
				frontier.push(tile);
				visited.insert(make_pair(tile, current));
			}
		}
	} // DFS end.

	vector<const Tile*> path = this->getPathTo(end, visited);

	return path;
}

std::vector<const Tile*> Grid::findPathDijkstra(const Tile& start, const Tile& end) const
{
	assert(&start != nullptr);
	assert(&end != nullptr);

	// Dijkstra start.
	queue<const Tile*> frontier;
	frontier.push(&start);

	unordered_map<const Tile*, const Tile*> visited;
	visited.insert(make_pair(&start, nullptr));

	unordered_map<const Tile*, float> costs = this->getInitialCosts();
	costs[&start] = 0.0f;

	while (!frontier.empty())
	{
		const Tile* current = frontier.front();
		frontier.pop();

		if (*current == end)
		{
			break;
		}

		vector<const Tile*> neighbors = this->getTileNeighbors(*current);
		priority_queue<const Tile*> prioritizedNeighbors = priority_queue<const Tile*>(neighbors.cbegin(), neighbors.cend());

		while (!prioritizedNeighbors.empty())
		{
			const Tile* tile = prioritizedNeighbors.top();
			prioritizedNeighbors.pop();

			float currentCost = costs[tile];
			float newCost = costs[current] + tile->getWeight();

			bool isVisited = visited.count(tile) > 0;

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
				visited.insert(make_pair(tile, current));
			}
		}
	} // Dijkstra end.

	vector<const Tile*> path = this->getPathTo(end, visited);

	return path;
}

std::vector<const Tile*> Grid::findPathAStar(const Tile& start, const Tile& end) const
{
	assert(&start != nullptr);
	assert(&end != nullptr);

	// A* start.
	queue<const Tile*> frontier;
	frontier.push(&start);

	unordered_map<const Tile*, const Tile*> visited;
	visited.insert(make_pair(&start, nullptr));

	unordered_map<const Tile*, float> costs = this->getInitialCosts();
	costs[&start] = 0.0f;

	while (!frontier.empty())
	{
		const Tile* current = frontier.front();
		frontier.pop();

		if (*current == end)
		{
			break;
		}

		auto heuristicComparer = [&](const Tile* left, const Tile* right) 
		{
			float leftHeuristic = left->getWeight() + getManhattanDistance(*left, end);
			float rightHeuristic = right->getWeight() + getManhattanDistance(*right, end);

			return leftHeuristic > rightHeuristic;
		};

		auto prioritizedNeighbors = priority_queue<const Tile*, vector<const Tile*>, decltype(heuristicComparer)>(heuristicComparer);
		auto neighbors = this->getTileNeighbors(*current);

		for (auto& item : neighbors)
		{
			prioritizedNeighbors.push(item);
		}

		while (!prioritizedNeighbors.empty())
		{
			const Tile* tile = prioritizedNeighbors.top();
			prioritizedNeighbors.pop();

			float currentCost = costs[tile];
			float newCost = costs[current] + tile->getWeight();

			bool isVisited = visited.count(tile) > 0;

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
				visited.insert(make_pair(tile, current));
			}
		}
	} // A* end.

	vector<const Tile*> path = this->getPathTo(end, visited);

	return path;
}

std::unordered_map<const Tile*, float> Grid::dijkstraAlgorithm(const Tile& start)
{
	assert(&start != nullptr);

	queue<const Tile*> frontier;
	frontier.push(&start);

	unordered_set<const Tile*> visited;
	visited.insert(&start);

	unordered_map<const Tile*, float> costs = this->getInitialCosts();
	costs[&start] = 0.0f;

	while (!frontier.empty())
	{
		const Tile* current = frontier.front();
		frontier.pop();

		vector<const Tile*> neighbors = this->getTileNeighbors(*current);
		priority_queue<const Tile*> prioritizedNeighbors = priority_queue<const Tile*>(neighbors.cbegin(), neighbors.cend());

		while (!prioritizedNeighbors.empty())
		{
			const Tile* tile = prioritizedNeighbors.top();
			prioritizedNeighbors.pop();

			float oldCost = costs[tile];
			float newCost = costs[current] + tile->getWeight();

			if (newCost < oldCost)
			{
				costs[tile] = newCost;
			}

			bool isVisited = visited.count(tile) > 0;

			if (!isVisited)
			{
				frontier.push(tile);
				visited.insert(tile);
			}
		}
	}

	return costs;
}

std::vector<const Tile*> Grid::getTileNeighbors(const Tile& tile) const
{
	assert(&tile != nullptr);

	return this->getTileNeighbors(tile.getRow(), tile.getColumn());
}

std::vector<const Tile*> Grid::getTileNeighbors(int row, int column) const
{
	assert(isRowInRange(row));
	assert(isColumnInRange(column));

	vector<const Tile*> neighbors;

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
		auto* upperNeighbor = &grid[row - 1][column];
		neighbors.push_back(upperNeighbor);
	}

	//if (canGetUpperRightNeighbor) //
	//{
	//    auto* upperRightNeighbor = &grid[row - 1][column + 1];
	//    neighbors.push_back(upperRightNeighbor);
	//}

	if (canGetRightNeighbor)
	{
		auto* rightNeighbor = &grid[row][column + 1];
		neighbors.push_back(rightNeighbor);
	}

	//if (canGetLowerRightNeighbor) //
	//{
	//    auto* lowerRightNeighbor = &grid[row + 1][column + 1];
	//    neighbors.push_back(lowerRightNeighbor);
	//}

	if (canGetLowerNeighbor)
	{
		auto* lowerNeighbor = &grid[row + 1][column];
		neighbors.push_back(lowerNeighbor);
	}

	//if (canGetLowerLeftNeighbor) //
	//{
	//    auto* lowerLeftNeighbor = &grid[row + 1][column - 1];
	//    neighbors.push_back(lowerLeftNeighbor);
	//}

	if (canGetLeftNeighbor)
	{
		auto* leftNeighbor = &grid[row][column - 1];
		neighbors.push_back(leftNeighbor);
	}

	//if (canGetUpperLeftNeighbor) //
	//{
	//    auto* upperLeftNeighbor = &grid[row - 1][column - 1];
	//    neighbors.push_back(upperLeftNeighbor);
	//}

	return neighbors;
}

float Grid::getManhattanDistance(const Tile& a, const Tile& b) const
{
	assert(&a != nullptr);
	assert(&b != nullptr);

	float aRow = static_cast<float>(a.getRow());
	float aColomn = static_cast<float>(a.getColumn());

	float bRow = static_cast<float>(b.getRow());
	float bColumn = static_cast<float>(b.getColumn());

	float manhattanDistance = fabs(aRow - bRow) + fabs(aColomn - bColumn);

	return manhattanDistance;
}

std::string Grid::toString() const
{
	string result;

	for (int row = 0; row < this->getRows(); row++)
	{
		for (int column = 0; column < this->getColumns(); column++)
		{
			result.append(grid[row][column].toString() + " ");
		}

		result.push_back('\n');
	}

	return result;
}

void Grid::setRows(int rows)
{
	assert(rows > 0);

	this->rows = rows;
}

void Grid::setColumns(int columns)
{
	assert(columns > 0);

	this->columns = columns;
}

std::vector<const Tile*> Grid::getPathTo(const Tile& end, const unordered_map<const Tile*, const Tile*>& visited) const
{
	assert(&end != nullptr);
	assert(visited.size() > 0);

	vector<const Tile*> path;

	const Tile* current = nullptr;
	const Tile* previous = nullptr;

	bool isContained = visited.count(&end) > 0;

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
	unordered_map<const Tile*, float> costs;

	for (int row = 0; row < this->rows; row++)
	{
		for (int column = 0; column < this->columns; column++)
		{
			costs.insert(make_pair(&grid[row][column], std::numeric_limits<float>::infinity()));
		}
	}

	return costs;
}

bool Grid::canGetTile(int row, int column) const
{
	bool isInRange = isRowInRange(row) && isColumnInRange(column);

	return isInRange;
}

bool Grid::isRowInRange(int row) const
{
	bool isInRange = (0 <= row && row < rows);

	return isInRange;
}

bool Grid::isColumnInRange(int column) const
{
	bool isInRange = (0 <= column && column < columns);

	return isInRange;
}
