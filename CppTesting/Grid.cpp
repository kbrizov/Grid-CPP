#include "Grid.h"
#include <cassert>
#include <queue>
#include <stack>
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

std::vector<const Tile*> Grid::findPathUsingBFS(const Tile& start, const Tile& end)
{
	// Find the path using BFS.
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
	}

	// Return the path.
	vector<const Tile*> path = this->getPathTo(end, visited);

	return path;
}

std::vector<const Tile*> Grid::findPathUsingDFS(const Tile& start, const Tile& end)
{
	// Find the path using DFS.
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
	}

	// Return the path.
	vector<const Tile*> path = this->getPathTo(end, visited);

	return path;
}

std::vector<const Tile*> Grid::getTileNeighbors(const Tile& tile) const
{
	return this->getTileNeighbors(tile.getRow(), tile.getColumn());
}

std::vector<const Tile*> Grid::getTileNeighbors(int row, int column) const
{
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

bool Grid::canGetTile(int row, int column) const
{
	int minIndex = 0;
	int maxRowIndex = this->rows - 1;
	int maxColumnIndex = this->columns - 1;

	if (row < minIndex || row > maxRowIndex)
	{
		return false;
	}

	if (column < minIndex || column > maxColumnIndex)
	{
		return false;
	}

	return true;
}
