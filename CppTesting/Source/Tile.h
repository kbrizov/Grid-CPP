#pragma once

#include <cassert>
#include <functional>
#include <string>

class Tile
{
public:

	Tile(unsigned row = 0, unsigned column = 0, float weight = 1.0f);

	unsigned getRow() const;

	unsigned getColumn() const;

	float getWeight() const;

	void setWeight(float weight);

	bool operator<(const Tile& other) const;

	bool operator>(const Tile& other) const;

	bool operator==(const Tile& other) const;

	bool operator!=(const Tile& other) const;

	std::string toString() const;

private:

	unsigned m_row;
	unsigned m_column;
	float m_weight;
};

inline unsigned Tile::getRow() const
{
	return m_row;
}

inline unsigned Tile::getColumn() const
{
	return m_column;
}

inline float Tile::getWeight() const
{
	return m_weight;
}

inline void Tile::setWeight(float weight)
{
	assert(0.0f < weight);
	m_weight = weight;
}

namespace std
{
	template<>
	struct hash<Tile>
	{
		size_t operator()(const Tile& tile) const
		{
			size_t rowHash = hash<size_t>()(tile.getRow());
			size_t columnHash = hash<size_t>()(tile.getColumn());

			size_t tileHash = hash<size_t>()((rowHash << 1) ^ (columnHash >> 1));

			return tileHash;
		}
	};

	template<>
	struct hash<const Tile*>
	{
		size_t operator()(const Tile* tile) const
		{
			size_t tileHash = hash<Tile>()(*tile);

			return tileHash;
		}
	};
}
