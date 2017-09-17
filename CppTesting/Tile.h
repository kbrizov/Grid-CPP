#pragma once

#include <functional>
#include <string>

using namespace std;

class Tile
{
public:

	Tile(int row = 0, int column = 0, float weight = 1.0f);

	int getRow() const;

	int getColumn() const;

	float getWeight() const;

	void setWeight(float weight);

	bool operator<(const Tile& other) const;

	bool operator==(const Tile& other) const;

	string toString() const;

private:
	
	int row;
	int column;
	float weight;
};

namespace std
{
	template<>
	struct hash<Tile>
	{
		std::size_t operator()(const Tile& tile) const
		{
			int rowHash = hash<int>()(tile.getRow());
			int columnHash = hash<int>()(tile.getColumn());

			size_t tileHash = hash<int>()((rowHash << 1) ^ (columnHash >> 1));

			return tileHash;
		}
	};
}
