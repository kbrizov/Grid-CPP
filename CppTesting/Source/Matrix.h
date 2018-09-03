#pragma once
#include <cassert>
#include <string>
#include <vector>

using namespace std;

class Matrix
{
public:
	Matrix(int rows, int columns, int value = 0);
	Matrix(const initializer_list<vector<int>>& parameters);
	Matrix(const Matrix& other);
	Matrix(Matrix&& other) noexcept;
	~Matrix() = default;
	Matrix& operator=(const Matrix& other);
	Matrix& operator=(Matrix&& other) noexcept;
	vector<int>& operator[](const int index);
	friend ostream& operator<<(ostream& outputStream, const Matrix& matrix);
	int getRows() const;
	int getColumns() const;
	string toString() const;

private:
	void setRows(const int rows);
	void setColumns(const int columns);
	void copyMatrix(const Matrix& other);
	void moveMatrix(Matrix& other);

private:
	int rows;
	int columns;
	vector<vector<int>> matrix;
};

