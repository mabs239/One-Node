#include "stm32f4xx.h"
#include <string.h>
//#include "model.c"
//#include "array.c"
#include <math.h>


#define f_size 2
#define groups 28
#define sig 2



struct Model
{
    int w,b;
    double X [groups] [f_size];
    double y[groups];
    double alphas [groups];
		char kernelFunction[] ;
};




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
void svm_predict(struct Model a , double sample[f_size]);





/*
int main()
{
	//printf("Starting main...");
	
	struct Model model_1;
	double x[f_size] = { 15 , 1};
	
		
	svm_predict(model_1,x);
	return 0;
}
*/


/**********************************************************************/
/* 																																		*/
/*																																		*/
/**********************************************************************/
svm_predict(struct Model a, double sample[f_size])
{
	double p[groups],pred[groups];
	int l;

	
		double X1,X2[groups];
		int i,j;
		double K[groups][groups],KernelOut;
		double temp[f_size],temp2[groups],temp3[f_size],chuss[groups][groups]; //temp = sample.^2; temp2 = -2*sample*model.X'
		array_square(sample,temp);
		X1 = array_sum(temp);
		
		for(i=0;i<groups;i++)																		// this will add the square of all the features and put them
		{																												// in X2[i]
			array_square(a.X[i],temp);
			X2[i] = array_sum(temp);
		}
		for(i=0;i<groups;i++)
		{
			arrayS_Mult(sample,a.X[i],temp3,f_size);					// temp3 = sample*model.X[i] i.e multiplication of samples with column of model.X
			temp2[i] = -2*array_sum(temp3);
		}
		Matrix_sum(X2, temp2,chuss,groups);
		Matrix_int_sum(chuss,X1,K,groups);
		
		KernelOut = gaussianKernel(1,1,2);
		for(i=0;i<groups;i++)
		{
			for(j=0;j<groups;j++)
			{
				K[i][j] = pow(KernelOut,K[i][j]);
			}
		}
		Mat_array_mult(K,a.y,K,groups);
		Mat_array_mult(K,a.alphas,K,groups);
		
		for(j=0;j<groups;j++)
		{
			p[j] = array_sum(K[j]);
		}
		
	

	for(l=0;l<groups;l++)
	{
		if(p[l]>=0)
			pred[l] = 1;
		else
			pred[l] = 0;
	}
}

/**********************************************************************/
/* 																																		*/
/*																																		*/
/**********************************************************************/
void array_Mult(double array[f_size],double x,double result[f_size],int size)
{
	int i;
	for(i = 0; i<size;i++)
			result[i] = array[i] * x;
}

/**********************************************************************/
/* 																																		*/
/*																																		*/
/**********************************************************************/
void array_add(double array[f_size],double x,double result[f_size])
{
	int i;
	for(i=0;i<f_size;i++)
			result[i] = array[i] + x;
}

/**********************************************************************/
/* 																																		*/
/*																																		*/
/**********************************************************************/
double array_sum(double array[f_size])
{
	int i;
	double result=0;
	for(i=0;i<f_size;i++)
		result = result+array[i];
	return result;
}


/**********************************************************************/
/* 																																		*/
/*																																		*/
/**********************************************************************/
void array_square(double array[f_size],double result[f_size])
{
	int i;
	for(i=0;i<f_size;i++)
		result[i] = array[i] * array[i];
}


/**********************************************************************/
/* Simple Multiply: One to One element Multiplication									*/
/*																																		*/
/**********************************************************************/
void arrayS_Mult(double array1[],double array2[], double result[],int size)
{
	int i;
	for(i=0;i<size;i++)
		result[i] = array1[i] * array2[i];
}


/**********************************************************************/
/* 																																		*/
/*																																		*/
/**********************************************************************/
void Matrix_sum(double array1[],double array2[],double result[groups][groups],int size)
{
	int j;
	for(j=0;j<size;j++)
	{
		array_int_sum(array2,array1[j],result[j],groups);
	}
}

/**********************************************************************/
/* 																																		*/
/*																																		*/
/**********************************************************************/
void array_int_sum(double array[],double a,double result[],int size)
{
	int l;
	for(l=0;l<size;l++)
	{
		result[l] = array[l] + a;
	}
}

/**********************************************************************/
/* 																																		*/
/*																																		*/
/**********************************************************************/
double gaussianKernel(double x1,double x2,double sigma)
{
	double t,d,sim;
	t = (x1 - x2);
	t = -1*t*t;
	d = 2*sigma*sigma;
	sim = exp(t/d);
	
	return sim;
}

/**********************************************************************/
/* 																																		*/
/*																																		*/
/**********************************************************************/
void Matrix(double array1[] , double array2[], double result[groups][groups],int size)
{
	int i;
	for(i=0;i<size;i++)
	{
		array_Mult(array2,array1[i],result[i],size);
	}
}

/**********************************************************************/
/* 																																		*/
/*																																		*/
/**********************************************************************/
void Matrix_int_sum(double array[groups][groups],double a,double result[groups][groups],int size)
{
	int i,j;
	for(i=0;i<size;i++)
	{
		for(j=0;j<size;j++)
		{
			result[i][j] = array[i][j]+a;
		}
	}
}

/**********************************************************************/
/* 																																		*/
/*																																		*/
/**********************************************************************/
void Mat_array_mult(double mat[groups][groups],double array[],double result[groups][groups],int size)
{
	int i,j;
	for(i=0;i<groups;i++)
	{
		for(j=0;j<groups;j++)
		{
			result[i][j] = mat[i][j] * array[i];
		}
	}
}







Bismillah Hir Rahman Nir Raheem 
mX1 = 0.980000
 3.600062e-01  6.469466e-01  1.891016e-01  6.496035e-02  9.445053e-01  6.000573e-01  2.402379e-01  5.111259e-02  5.061829e-02  8.497099e-03  2.500013e-03  1.817460e-03  2.096676e-03  3.171468e-03  5.121884e-04  8.077563e-04  2.877752e+00  5.165369e+00  2.130508e-01  5.731233e-02  4.333625e+00  5.401824e+00  6.341390e+00  6.712073e+00  7.369369e+00  6.339142e+00  8.397354e+00  4.487579e+00 

temp = [ 8.820513e-08  5.220274e-09  4.975411e-09  1.713333e-12  7.695968e-11  1.422657e-09  3.578902e-09  1.309129e-13  5.076217e-14  1.327454e-19  2.146797e-37  2.067542e-37  3.435276e-37  1.201186e-35  1.041368e-21  3.378501e-24  -3.564498e-31  -6.778602e-63  -8.795298e-09  -8.943941e-13  -4.738310e-49  -1.713025e-62  -1.284441e-78  -9.657838e-83  -4.498925e-96  -8.252063e-77  -8.712138e-113  -3.080026e-50 ]
Decision Variable = 3.857271e-270
