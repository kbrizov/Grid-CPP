#include <iostream>
#include "Matrix.h"

Matrix::Matrix(int rows, int columns, int value)
{
	this->setRows(rows);
	this->setColumns(columns);

	this->matrix = vector<vector<int>>(rows);

	for (int row = 0; row < rows; row++)
	{
		this->matrix[row] = vector<int>(columns);

		for (int column = 0; column < columns; column++)
		{
			matrix[row][column] = value;
		}
	}
}

Matrix::Matrix(const initializer_list<vector<int>>& parameters)
{
	this->setRows(parameters.size());
	this->setColumns((*parameters.begin()).size());
	this->matrix = vector<vector<int>>(parameters.begin(), parameters.end());
}

Matrix::Matrix(const Matrix& other)
{
	this->copyMatrix(other);
}

Matrix::Matrix(Matrix&& other) noexcept
{
	this->moveMatrix(other);
}

Matrix& Matrix::operator=(const Matrix& other)
{
	if (this != &other)
	{
		this->copyMatrix(other);
	}

	return *this;
}

Matrix& Matrix::operator=(Matrix&& other) noexcept
{
	if (this != &other)
	{
		this->moveMatrix(other);
	}

	return *this;
}

vector<int>& Matrix::operator[](const int index)
{
	return this->matrix[index];
}

int Matrix::getRows() const
{
	return this->rows;
}

int Matrix::getColumns() const
{
	return this->columns;
}

string Matrix::toString() const
{
	string result;

	for (int row = 0; row < this->getRows(); row++)
	{
		for (int column = 0; column < this->getColumns(); column++)
		{
			result.append(to_string(matrix[row][column]) + " ");
		}

		result.push_back('\n');
	}

	return result;
}

void Matrix::setRows(const int rows)
{
	assert(rows > 0);

	this->rows = rows;
}

void Matrix::setColumns(const int columns)
{
	assert(columns > 0);
	
	this->columns = columns;
}

void Matrix::copyMatrix(const Matrix& other)
{
	this->setRows(other.rows);
	this->setColumns(other.columns);

	this->matrix = other.matrix;
}

void Matrix::moveMatrix(Matrix& other)
{
	this->setRows(other.rows);
	this->setColumns(other.columns);
	
	other.rows = 0;
	other.columns = 0;
	this->matrix = move(other.matrix);
}

ostream& operator<<(ostream& outputStream, const Matrix& matrix)
{
	outputStream << matrix.toString();

	return outputStream;
}
