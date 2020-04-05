#pragma once

#include <cassert>
#include <functional>
#include <string>

class TILE
{
public:

	TILE(unsigned row = 0, unsigned column = 0, float weight = 1.0f);

	unsigned get_row() const;

	unsigned get_column() const;

	float get_weight() const;

	void set_weight(float weight);

	bool operator<(const TILE& other) const;

	bool operator>(const TILE& other) const;

	bool operator==(const TILE& other) const;

	bool operator!=(const TILE& other) const;

	std::string to_string() const;

private:

	unsigned m_row;
	unsigned m_column;
	float m_weight;
};

inline unsigned TILE::get_row() const
{
	return m_row;
}

inline unsigned TILE::get_column() const
{
	return m_column;
}

inline float TILE::get_weight() const
{
	return m_weight;
}

inline void TILE::set_weight(float weight)
{
	assert(0.0f < weight);
	m_weight = weight;
}

namespace std
{
	template<>
	struct hash<TILE>
	{
		size_t operator()(const TILE& tile) const
		{
			size_t row_hash = hash<size_t>()(tile.get_row());
			size_t column_hash = hash<size_t>()(tile.get_column());

			size_t tile_hash = hash<size_t>()((row_hash << 1) ^ (column_hash >> 1));

			return tile_hash;
		}
	};
}
