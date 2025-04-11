#ifndef MATH_MATRIX_H
#define MATH_MATRIX_H

#include "math/math.h"

int matrix_inverse(float src[3][3], int n, float des[3][3]);
void matrix_mult_33(float matrix1[3][3], float matrix2[3][3], float ans[3][3]);
void matrix_mult(float *matrix1, int row1, int col1, float *matrix2, int row2, int col2, float *ans);
void matrix_add(float *matrix1, float *matrix2, float *ans, int row, int col);
void matrix_sub(float *matrix1, float *matrix2, float *ans, int row, int col);
void matrix_t(float *matrix1, int row, int col, float *ans);
void matrix_clear(float *mat, int row, int col);
void matrix_mult_number(float *mat, int row, int col, float num, float *ans);
#endif