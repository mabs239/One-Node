#ifndef HelperFunctions
#define HelperFunctions
//HelperFunctions.h
int add239(int x, int y);
double gaussianKernel(double x1,double x2,double sigma);
void printArray(double a[], int len);
void printArray2D(int m, int n,double a[3][3]);
int svmPredict(double X[]);

extern const int f_size;
extern const int groups;
extern double modelX[f_size][groups];

extern double modelY[];
extern double modelAlphas[];
extern double modelW[];
extern double modelB;
extern int modelLength;
#endif
