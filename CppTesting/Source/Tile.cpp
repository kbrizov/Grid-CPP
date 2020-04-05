#include "Tile.h"
#include <cmath>

TILE::TILE(unsigned row, unsigned column, float weight) :
	m_row(row),
	m_column(column)
{
	this->set_weight(weight);
}

bool TILE::operator<(const TILE& other) const
{
	const bool is_lesser = m_weight < other.m_weight;

	return is_lesser;
}

bool TILE::operator>(const TILE& other) const
{
	const bool is_greater = m_weight > other.m_weight;

	return is_greater;
}

bool TILE::operator==(const TILE& other) const
{
	const bool equal_rows = (m_row == other.m_row);
	const bool equal_columns = (m_column == other.m_column);

	const float delta_weight = std::fabs(m_weight - other.m_weight);
	const bool equal_weight = (delta_weight <= std::numeric_limits<float>::epsilon());

	const bool are_equal = equal_rows && equal_columns && equal_weight;

	return are_equal;
}

bool TILE::operator!=(const TILE& other) const
{
	return !(*this == other);
}

std::string TILE::to_string() const
{
	const std::string row = std::to_string(m_row);
	const std::string column = std::to_string(m_column);
	
	const std::string result = std::string("(" + row + ", " + column + ")");

	return result;
}
