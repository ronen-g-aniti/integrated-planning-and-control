#include <vector>
#include "../include/matrix_math.h"
using namespace std;

Matrix3X3::Matrix3X3(vector<vector<double>> matrix) :
	matrix(matrix) {}

Matrix3X3 Matrix3X3::operator*(const Matrix3X3& other) const {

	// Implement matrix multiplication
    vector<vector<double>> resultMatrix(3, vector<double>(3, 0.0));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            resultMatrix[i][j] = matrix[i][0] * other.matrix[0][j] +
                matrix[i][1] * other.matrix[1][j] +
                matrix[i][2] * other.matrix[2][j];
        }
    }
    return Matrix3X3(resultMatrix);
}

vector<double> Matrix3X3::operator*(const vector<double>& vec) const {
    // Implement matrix-vector multiplication
    vector<double> resultVector(3, 0.0);
    for (int i = 0; i < 3; ++i) {
		resultVector[i] = matrix[i][0] * vec[0] +
			matrix[i][1] * vec[1] +
			matrix[i][2] * vec[2];
	}
    return resultVector;
}

const vector<double>& Matrix3X3::operator[](const size_t index) const {
	// Access the index-th row of the matrix without allowing modification
	return matrix[index];
}

vector<double>& Matrix3X3::operator[](const size_t index) {
	// Access the index-th row of the matrix with the ability to modify it
	return matrix[index];
}




