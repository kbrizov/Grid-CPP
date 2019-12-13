#include "Tile.h"
#include <cmath>

Tile::Tile(unsigned row, unsigned column, float weight) :
	m_row(row),
	m_column(column)
{
	this->setWeight(weight);
}

bool Tile::operator<(const Tile& other) const
{
	bool isLesser = m_weight < other.m_weight;

	return isLesser;
}

bool Tile::operator>(const Tile& other) const
{
	bool isGreater = m_weight > other.m_weight;

	return isGreater;
}

bool Tile::operator==(const Tile& other) const
{
	const bool equalRows = (m_row == other.m_row);
	const bool equalColumns = (m_column == other.m_column);

	const float deltaWeight = std::fabs(m_weight - other.m_weight);
	const bool equalWeight = (deltaWeight <= std::numeric_limits<float>::epsilon());

	const bool areEqual = equalRows && equalColumns && equalWeight;

	return areEqual;
}

bool Tile::operator!=(const Tile& other) const
{
	return !(*this == other);
}

std::string Tile::toString() const
{
	std::string row = std::to_string(m_row);
	std::string column = std::to_string(m_column);

	std::string result = std::string("(" + row + ", " + column + ")");

	return result;
}
