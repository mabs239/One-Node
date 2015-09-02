#ifndef helper
#define helper
#define f_size 2
#define groups 28
void printArray(double a[], int len);
void printArray2D(int m, int n,double a[3][3]);
void array_Mult(double array[f_size],double x,double result[f_size],int size);
void array_add(double array[f_size],double x,double result[f_size]);
void array_square(double array[f_size],double result[f_size]);
void arrayS_Mult(double array1[],double array2[], double result[],int size);
double  array_sum(double array[f_size]);
void arrayS_sum(double array1[],double array2[],double result[],int size);
void Matrix_sum(double array1[],double array2[],double result[groups][groups],int size);
void array_int_sum(double array[],double a,double result[],int size);
void Matrix_int_sum(double array[groups][groups],double a,double result[groups][groups],int size);
double gaussianKernel(double x1,double x2,double sigma);
void Matrix(double array1[] , double array2[], double result[groups][groups],int size);
void Mat_array_mult(double mat[groups][groups],double array[],double result[groups][groups],int size);
#endif 

