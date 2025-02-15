#ifndef MATH_MATRIX_H
#define MATH_MATRIX_H

int matrix_inverse(float src[3][3], int n, float des[3][3]);
void matrix_mult(float matrix1[3][3], float matrix2[3][3], float ans[3][3]);
#endif