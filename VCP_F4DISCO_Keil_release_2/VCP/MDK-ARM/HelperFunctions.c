//HelperFunctions.c
#include "HelperFunctions.h"
//#include "uart.h"
#include "math.h"

//#include "stm32f4xx.h"
//#include "main.h"
//#include "cc1101.h"
//#include "stm32f4_discovery.h"
//#include "usbd_cdc_vcp.h"
//#include "usbd_cdc_core.h"
//#include "usbd_usr.h"
//#include "usb_conf.h"
//#include "usbd_desc.h"
//#include "GDO2_Interrupt.h"
//#include "three_adcs_nwn.h"

//#include "time_nwn.h"
#include <stdio.h>



/**********************************************************************/
/* 																																		*/
/*																																		*/
/**********************************************************************/
int add239(int x, int y)
{ 
	return x+y;
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
void printArray(double a[], int len)
{
	int i = 0;
	printf("\r\n");
	for(i=0;i<len;i++){
		printf(" %e ",a[i]);
	}
	printf("\r\n");
}

/**********************************************************************/
/* 																																		*/
/*																																		*/
/**********************************************************************/
void printArray2D(int m, int n,double a[3][3])
{
	int i = 0, j = 0;
	printf("\r\n");
	for(i=0;i<m;i++){
			for(j=0;j<n;j++){
				printf(" %f ",a[i][j]);
			}
			printf("\r\n");
	}
	printf("\r\n");
}


/**********************************************************************/
/* 																																		*/
/*																																		*/
/**********************************************************************/
