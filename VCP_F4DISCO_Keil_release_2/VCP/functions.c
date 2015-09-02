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


