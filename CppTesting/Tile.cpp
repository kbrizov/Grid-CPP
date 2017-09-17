#include "Tile.h"

Tile::Tile(int row, int column, float weight)
{
	this->row = row;
	this->column = column;
	this->setWeight(weight);
}

int Tile::getRow() const
{
	return this->row;
}

int Tile::getColumn() const
{
	return this->column;
}

float Tile::getWeight() const
{
	return this->weight;
}

void Tile::setWeight(float weight)
{
	this->weight = weight;
}

bool Tile::operator<(const Tile& other) const
{
	bool isLesser = this->weight < other.weight;

	return isLesser;
}

std::string Tile::toString() const
{
	string result("(" + to_string(this->row) + ", " + to_string(this->column) + ")");

	return result;
}

bool Tile::operator==(const Tile& other) const
{
	bool equalRows = this->row == other.row;
	bool equalColumns = this->column == other.column;
	bool equalWeight = this->weight == other.weight;

	bool areEqual = equalRows && equalColumns && equalWeight;

	if (areEqual)
	{
		return true;
	}

	return false;
}
