#ifndef CORDICCART2POL_H
#define CORDICCART2POL_H

#define NO_ITER 16
#define PI 3.14159265359
typedef int   coef_t;
typedef float data_t;
typedef float acc_t;

void cordiccart2pol(data_t *x, data_t *y, data_t *r,  data_t *theta);

#endif