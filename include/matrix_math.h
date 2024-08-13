#pragma once

#include <vector>
using namespace std;

class Matrix3X3 {
public:
	Matrix3X3(const vector<vector<double>> matrix);
	Matrix3X3 operator*(const Matrix3X3& other) const;
	vector<double> operator*(const vector<double>& vec) const;
	const vector<double>& operator[](const size_t index) const;
	vector<double>& operator[](const size_t index);
	

private:
	vector<vector<double>> matrix;

};