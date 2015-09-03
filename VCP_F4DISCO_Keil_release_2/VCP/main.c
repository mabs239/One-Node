#include "stm32f4xx.h"
#include "main.h"
#include "cc1101.h"
#include "stm32f4_discovery.h"
#include "usbd_cdc_vcp.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "GDO2_Interrupt.h"
#include "time_nwn.h"
#include "three_adcs_nwn.h"
#include "uart.h"
#include "math.h"
#define Rise_Const 12
#define Fall_Const 2
#define Samples 500
#define Noise_Samples 500
#define Corr_Result_Size (2*Samples)-1
#define Fs  45000
#define Vs  333



const int f_size = 2;
const int groups = 28;


struct Model
{
    double w [f_size];
		double b;
    double X [groups] [f_size];
    double y[groups];
    double alphas [groups];
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



__ALIGN_BEGIN USB_OTG_CORE_HANDLE     USB_OTG_dev  __ALIGN_END ;
volatile uint32_t time_var1;
unsigned char ADC0_Out_8,ADC1_Out_8,ADC2_Out_8;
volatile int ADC0_Out,ADC1_Out,ADC2_Out;


/* Define Model parameters*/
double modelX[f_size][groups] ={{0.579000344, 0.796917226, 0.422056676, 0.237648833, 0.968937944, 0.770653925, 0.484163008 , 0.213185483	,0.219116214	,0.003703923	,0.000113883	,9.42E-05	,2.30E-06	,1.16E-06	,1.75E-07	,1.04E-08,	1.695387474,	2.271354862,	0.452291288,	0.227607391,	2.077405125,	2.313796625,	2.512395494,	2.580537307,	2.71198118,	2.506749483,	2.895398071	,2.110320158},
{	0.157368421,	0.108947368,	0.104736842	,0.092105263,	0.075263158,	0.078421053,	0.076315789,	0.075263158,	0.051052632	,0.092105263,	0.05,	0.042631579	,0.045789474,	0.056315789	,0.022631579,	0.028421053,	0.058421053	,0.079473684,	0.092105263,	0.074210526,	0.134210526,	0.219473684,	0.171052632,	0.23,	0.120526316	,0.235263158,	0.118421053,	0.184736842}};


//double X1[] ={0.579000344, 0.796917226, 0.422056676, 0.237648833, 0.968937944, 0.770653925, 0.484163008 , 0.213185483	,0.219116214	,0.003703923	,0.000113883	,9.42E-05	,2.30E-06	,1.16E-06	,1.75E-07	,1.04E-08,	1.695387474,	2.271354862,	0.452291288,	0.227607391,	2.077405125,	2.313796625,	2.512395494,	2.580537307,	2.71198118,	2.506749483,	2.895398071	,2.110320158};
//double X2[] ={	0.157368421,	0.108947368,	0.104736842	,0.092105263,	0.075263158,	0.078421053,	0.076315789,	0.075263158,	0.051052632	,0.092105263,	0.05,	0.042631579	,0.045789474,	0.056315789	,0.022631579,	0.028421053,	0.058421053	,0.079473684,	0.092105263,	0.074210526,	0.134210526,	0.219473684,	0.171052632,	0.23,	0.120526316	,0.235263158,	0.118421053,	0.184736842};
double modelY[] = {	1	,1	,1	,1	,1	,1	,1	,1	,1	,1	,1	,1	,1	,1	,1	,1	,-1,	-1	,-1	,-1	,-1	,-1	,-1	,-1	,-1	,-1	,-1	,-1};
double modelAlphas[] = {	0.454251936	,0.321773341,	11.7139214,	7.945854578	,0.855086621,	0.447928742,	10.27969317,	5.47384088,	7.444460018,	0.473481755,	1.39E-17,	2.17E-17,	2.95E-17,	5.21E-16,	0.417961766,	0.000917648,	1.011944606	,0.644913297,	20	,20,	0.668406582,	0.631434696,	0.325245149,	0.375024182	,0.703681435,	0.309626792,	0.8806807,	0.278214418};
double modelW[] = {-10.93210935, -0.325816793};
double modelB = 0.0116;
int modelLength = 28;


int index=0,Sig_index=0,Noise_Index=0,delay_index;
uint32_t Noise_0=0,Noise_1=0,Noise_2=0,sum0=0,sum1=0,sum2=0;
uint32_t AvgNoise_0=0,AvgNoise_1=0,AvgNoise_2=0;
uint32_t Thresh_Rising_0=0,Thresh_Rising_1=0,Thresh_Rising_2=0;
int8_t Mic0_data[Samples],Mic1_data[Samples],Mic2_data[Samples]; 
unsigned int Signal_Mean0,Signal_Mean1,Signal_Mean2;
uint32_t Arr[Noise_Samples];
long int Corr_Result[Corr_Result_Size];
uint8_t GetSignal=0,No_Noise_Eval=0;
uint32_t Max_Index=0,Corr_Max=0;
float arg,Angle,Angle1,Angle2,Angle3,d=0.3,Env_Const=1.2,pi=3.14;
float Angle12,Angle23,Angle31;
uint8_t Angle_Negative,Angle_FirstNum,Angle_SecondNum,Angle_ThirdNum,Integer_Part1,Integer_Part2;
float Floating_Part;
char Angle_Str[6] = {'A','N','G','L','E','='};
uint32_t var;
int Temp=0;
uint8_t Edge_Angle1,Edge_Angle2,Edge_Angle3,Region_Angle1,Region_Angle2,Region_Angle3,Angle3_Invalid;
uint8_t Edge_Angle1,Edge_Angle2,Edge_Angle3,Region_Angle12,Region_Angle23,Region_Angle31;
uint8_t Angle2_Invalid,Angle1_Invalid,Angle12_Invalid,Angle31_Invalid,Angle23_Invalid;
uint8_t Region,Edge;
uint8_t EdgeDefAngle[6]={330,30,90,150,210,270}; /////yet to be modified (Akif)
/////Can be better approximated (Hakeem)
uint8_t false_exti,timefirst=0;
long int result=0;

void printArray();
//void printArray2D(int m, int n,double a[m][n]);
//void printArray2D(int m, int n,double **a);
							/**********************************************************************/
							/* Add code to view results in debug window														*/
							/*																																		*/
							/**********************************************************************/
							 // Add ITM Port register definitions to the source code.

							#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
							#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
							#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

							#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
							#define TRCENA          0x01000000

							//Add an fputc function to the source code that writes to the ITM Port 0 register. The fputc function enables printf to output messages.

							struct __FILE { int handle; /* Add whatever is needed */ };
							FILE __stdout;
							FILE __stdin;

							int fputc(int ch, FILE *f) {
								if (DEMCR & TRCENA) {
									while (ITM_Port32(0) == 0);
									ITM_Port8(0) = ch;
								}
								return(ch);
							}

							//Add a debugging trace messages using printf.
							//printf("AD value = 0x%04X\r\n", AD_value);

							/**********************************************************************/
							/* Above code is useful for debug printf function.										*/
							/**********************************************************************/


void USART_puts(USART_TypeDef* USARTx, volatile int s) /// Dosri file (Usart)
{
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) ); 
		USART_SendData(USARTx,s);
}
void Make_Receiver(){
	    halRfWriteRfSettings(CC1101_TYPE_RECEIVE);
		  CC1101_Receive_Main();
}
void Make_Transmitter(){
	   halRfWriteRfSettings(CC1101_TYPE_SEND);
	   TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
}

void EXTI15_10_IRQHandler(void){	  
  if(EXTI_GetITStatus(EXTI_Line10) != RESET)  {
    EXTI_ClearITPendingBit(EXTI_Line10);
		Recieve();
	  false_exti=getExtiFlag();
	  if(false_exti==1)		{
			false_exti=0;
			STM_EVAL_LEDToggle(LED3);
      STM_EVAL_LEDToggle(LED4);
      STM_EVAL_LEDToggle(LED5);
 
	    halRfWriteRfSettings(CC1101_TYPE_SEND);
	    Make_Transmitter();
		}
	}
}
void Startup_Delay(){
	index++;
}
void Noise_Eval(){
  Temp=ADC0_Out_8;
  if(Temp > Noise_0)  {
   Noise_0=Temp;
  }	

	Temp=ADC1_Out_8;
	if(Temp > Noise_1)	{
    Noise_1=Temp;
  }
	
  Temp=ADC2_Out_8;
	if(Temp > Noise_2)	{
    Noise_2=Temp;
  }
	index++;
}

void Cross_Correlation(int8_t * Mic_data0, int8_t * Mic_data1 ){
  uint16_t i,j,l,k;
	uint32_t movingIndexCount=1;
	int8_t *ptr;
	int8_t *movingptr  = &Mic_data1[Samples-1];
	int8_t *movingptr1 = &Mic_data0[Samples-1];
	
	for(i = 0; i < Samples; i++)    /////Samples=500
  {
     ptr=movingptr;
     for(j = 0; j < movingIndexCount; j++ )     {                                                           
        result += Mic_data0[j] * (*movingptr);
        movingptr++; //wrong assignment
     }
     ptr--;
        movingptr=ptr;
        movingIndexCount++;
        Corr_Result[i] = result;
		 if((result & 0x80000000)== 0x00000000)		 {   
   		 if(Corr_Max < result)				{
					Max_Index=i;
					Corr_Max=result;
				}
		 }
		 result=0;
  }
	movingIndexCount=1;
	for(k =(Corr_Result_Size-1); k>Samples-1 ;k--){
     ptr=movingptr1;
     for(l = 0; l < movingIndexCount; l++ ){
       result += Mic_data1[l] * (*movingptr1);
       movingptr1++; //wrong assignment
     }
     ptr--;
     movingptr1=ptr;
     movingIndexCount++;
     Corr_Result[k] = result;
		 if((result & 0x80000000)== 0x00000000)
		 {
				if(Corr_Max < result)
				{
					Max_Index=k;
					Corr_Max=result;
				}
			}
			result=0;
	}
}

void Set_Threshold(){
	//Thresh_Rising_0 = 160 ;
	Thresh_Rising_0 = (Rise_Const * Noise_0)/10;
	Thresh_Rising_1 = (Rise_Const * Noise_1)/10;
	Thresh_Rising_2 = (Rise_Const * Noise_2)/10;
	index++;
}

void Send_Angle(){
	uint8_t i;
	USART_puts(USART2,'\n');
	USART_puts(USART2,'\r');
	for(i=0;i<6;i++)  {
    USART_puts(USART2,Angle_Str[i]);
	}		
	Integer_Part1  = (uint8_t)Angle;
	Floating_Part = (Angle - Integer_Part1)*10;
	Angle_FirstNum = Integer_Part1/100;
  if(Angle_FirstNum!=0)	{
		Angle_FirstNum+=48; 
		USART_puts(USART2,Angle_FirstNum);
	}
	Angle_SecondNum = ((Integer_Part1%100)/10);
	if(Angle_SecondNum!=0)	{
		Angle_SecondNum+=48; 
		USART_puts(USART2,Angle_SecondNum);
	}
	Angle_ThirdNum = ((Integer_Part1%10))+48;
	USART_puts(USART2,Angle_ThirdNum);
	USART_puts(USART2,'.');
  Integer_Part2= ((uint8_t)Floating_Part)+48;
	USART_puts(USART2,Integer_Part2);
	USART_puts(USART2,248);
}

void Angle_Calculation(){
  delay_index=(Samples-1)-Max_Index;
	Angle_Negative=0;
	if ( (delay_index & 0x80000000) == 0x80000000 )	{
     delay_index = (~delay_index)+1;
		 Angle_Negative=1;
	}
	arg= (delay_index * Vs) / (Fs * d);
	//arg= arg * Env_Const;
	if ( arg > 1)
		  arg=1.00;
	else if (arg < -1)
		  arg=-1.00;
  
  Angle= acos(arg);  
  Angle = (Angle * 180)/pi;

  if(Angle_Negative == 1)  {
    Angle_Negative=0;
    Angle=180-Angle;
  }		
 // Send_Angle();
	
}

void Disable_ADCs(){
	    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
}

void Enable_ADCs(){
	    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

void Send_Signal_To_Mat(){
	uint16_t i;
	for(i=0; i<Samples ; i++)	{
		USART_puts(USART2,Mic0_data[i]);
	}
	for(i=0; i<Samples ; i++){
		USART_puts(USART2,Mic1_data[i]);
	}
	for(i=0; i<Samples ; i++)	{
		USART_puts(USART2,Mic2_data[i]);
	}
}


void Calculate_Average(){
	Signal_Mean0 =  (sum0/Samples) +1;
	Signal_Mean1 =  (sum1/Samples) +1;
	Signal_Mean2 =  (sum2/Samples) +1;
}

void  Subtract_Mean(){
	uint16_t i;
	for(i=0; i<Samples ; i++)	{
	Mic0_data[i]-= Signal_Mean0;
	Mic1_data[i]-= Signal_Mean1;
	Mic2_data[i]-= Signal_Mean2;
	}
}

void flush(){
	GetSignal=0;
	Sig_index=0;
	sum0=0;
	sum1=0;
	sum2=0;
	Signal_Mean0=0;
	Signal_Mean1=0;
	Signal_Mean2=0;
	Corr_Max=0;
	Max_Index=0;
	delay_index=0;
	Angle_Negative=0;
	Angle=0;
	arg=0;
	
	for(var=0; var < 16800000 ; var++)	{}
}

void Find_Region()
{
	Edge_Angle1=0x00;
/*******************Angle1 Regions****************/	
  if(Angle31 < 90)	{
		 Region_Angle31=0x31;			/////[1 1 0 0 0 1]  Region is 1 and 2 and 6
	}
	else if (Angle31 > 90)	{
		Region_Angle31=0xE;		//[0 0 1 1 1 0]  Region is 3 or 4 or 5
	}
  else                       /////Angle1=90
  {
   	Region_Angle31=0x00;
		Edge = 0x09;	     	//[0 0 1 0 0 1] Edge is 3 or 6
		if(Angle23<30)		{
			Edge=3;		///THird Edge
			Angle31=90;
		}
		else if (Angle23>120)		{
			Edge=6;		///Sixth Edge
			Angle31=270;
		}
  }

/*******************Angle2 Regions****************/			
	if(Angle12 < 90)	{
		 Region_Angle12=0x07;		//[0 0 0 1 1 1]  Region is 4 or 5 or 6
	}
	else if (Angle12 > 90)	{
		Region_Angle12=0x38;		// [1 1 1 0 0 0] Region is 1 or 2 or 3
	}
  else												////////Angle2 is 90( from Aqeel)
  {
		Region_Angle2=0x00;
	//	Edge = 0x12;		//0b010010		////////Edge is 5 or 2
		if(Angle23>150)		{
			Edge=1;		///First Edge
			Angle31=360-Angle31;
		}
		else if (Angle23<60)		{
			Edge=4;		///Fourth Edge
			Angle31=Angle31;
		}
  }
		
/*******************Angle3 Regions****************/		
	if(Angle23 < 90)	{
		 Region_Angle23=0x1C;		//[0 1 1 1 0 0]  Region is 2 and 3 and 4
	}
	else if (Angle23 > 90)	{
		Region_Angle23=0x23;		//[1 0 0 0 1 1]  Region is 1 and 5 and 6
	}
  else  {
		Region_Angle2=0x00;
		Edge = 0x09;		//0b001001	Edge is 1 or 4
		if(Angle12>120)		{
			Edge=2;		///Second Edge
			Angle31=Angle31;
		}
		else if (Angle12<60)		{
			Edge=5;		///Fifth Edge
			Angle31=360-Angle31;
		}
  }
	Region=Region_Angle12 & Region_Angle23 & Region_Angle31;
}

void Angle_Correction(){
 if(Region == 0x01)   //////Region=6
	{
		Angle12=Angle12;   ////Tick
		Angle31=360-Angle31; ///Tick
		Angle23_Invalid=1; ///Tick
	}
	else if(Region == 0x02)   ///Region=5
	{
		Angle23=Angle23;
		Angle31=360-Angle31;
		Angle12_Invalid=1;
	}

  else if(Region == 0x04)   ///Region=4
	{
		Angle12=360-Angle12;
		Angle23=Angle23;
		Angle31_Invalid=1;
	}	
	else if(Region == 0x08)  ///Region=3
	{
		Angle12=360-Angle12;
		Angle31=Angle31;
		Angle23_Invalid=1;
	}	
	else if(Region == 0x10)  ///Region=2
	{
	  Angle23=360-Angle23;
	  Angle31=Angle31;
		Angle12_Invalid=1;
	}
	else if(Region == 0x20)  ///Region=1
	{
		Angle12=Angle12;
		Angle23=360-Angle23;
		Angle31_Invalid=1;
	}
	if(Angle31_Invalid)	{
		Angle31=Angle12+240;
	}
}

void AngleGt300(){
	if(Angle31 >360 )	{
		Angle31-=360;
	}
}

void Source_Angle(){
	Find_Region();
	Angle_Correction();
	AngleGt300();
	CC1101_Send_Main(Angle31);
}
		
void Get_Signal(){
	/*Thresh_Falling_0 = Fall_Const * AvgNoise_0;
	Thresh_Falling_1 = Fall_Const * AvgNoise_1;
	Thresh_Falling_2 = Fall_Const * AvgNoise_2;*/
//STM_EVAL_LEDOff(LED6);
	
if((GetSignal==0) && (ADC0_Out_8 > Thresh_Rising_0 || ADC1_Out_8 > Thresh_Rising_1  || ADC2_Out_8 > Thresh_Rising_2)) 
{
	 STM_EVAL_LEDToggle(LED6);
   GetSignal=1;	
}	
	
if (GetSignal && Sig_index < Samples){
		Mic0_data[Sig_index] = ADC0_Out_8;
	  sum0+=ADC0_Out_8;
  	Mic1_data[Sig_index] = ADC1_Out_8;
	  sum1+=ADC1_Out_8;
    Mic2_data[Sig_index] = ADC2_Out_8;
	  sum2+=ADC2_Out_8;
    Sig_index++;		
}

if(Sig_index == Samples){
		  Disable_ADCs();
	    Send_Signal_To_Mat(); //Send to serial port
	    Calculate_Average(); 	
      Subtract_Mean();
     	Cross_Correlation(Mic0_data,Mic1_data);
	    Corr_Max=0;
	    result=0;
	    Angle_Calculation();
	    Angle12=Angle;
	    Angle=0;
	    Cross_Correlation(Mic1_data,Mic2_data);
	    Corr_Max=0;
	    result=0;
	    Angle_Calculation();
	    Angle23=Angle;
	    Angle=0;
	    Cross_Correlation(Mic2_data,Mic0_data);
	    Corr_Max=0;
	    result=0;
	    Angle_Calculation();
	    Angle31=Angle;
			USART_puts(USART2,(int)Angle31);
			USART_puts(USART2,'\n');
			
			printf("Source Angle(31) = %f \n",Angle31);
			
	   // Angle=0;
		  Source_Angle();
			printf("Source Angle(31) = %f \n",Angle31);
      flush();	
	    Enable_ADCs();
	}
	index++;
}


void Start_Processing(){
	if(No_Noise_Eval==0){		
		 if(index <=5000 ){
		   Startup_Delay();
		 }
		 else if (index > 5000 && index <= 5500)   // index between 5000 and 5500
		 {
		  Noise_Eval();
			Arr[Noise_Index]=ADC0_Out_8;
			Noise_Index++;
		 }
		 else if ( index == 5501)		 {
			 Set_Threshold();
			 No_Noise_Eval=1;
		 }
	}	
	else	{
		 STM_EVAL_LEDOn(LED4);
	   Get_Signal();
	}
}

void Get_Adc_Value(){
	    ADC0_Out=aADCDualConvertedValue[0];
      ADC1_Out=aADCDualConvertedValue[1];
      ADC2_Out=aADCDualConvertedValue[2];   
       
		  ADC0_Out_8= ((unsigned char)(ADC0_Out>>4));
      ADC1_Out_8= ((unsigned char)(ADC1_Out>>4));
		  ADC2_Out_8= ((unsigned char)(ADC2_Out>>4));
}

void TIM3_IRQHandler(void) //Timer Interupt for sending data
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    Get_Adc_Value();		 
		Start_Processing();  
	}
		
}

void Init_Recv_Node(){
	adc_config();  ///// Configure three adcs ( Regular Continous triple Mode 12 bit) 
  init_USART2(9600);     ////////Initialize USART with baud Rate 921600
  INTTIM3_Config();       /////////      Timer for ADC Sampling
	CC1101_Init();
  CC1101_POWER_RESET();
	halRfWriteRfSettings(CC1101_TYPE_SEND);
}

void printArray(double a[], int len){
	int i = 0;
	printf("\r\n");
	for(i=0;i<len;i++){
		printf(" %e ",a[i]);
	}
	printf("\r\n");
}


void printArray2D(int m, int n,double a[3][3]){
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
double gaussianKernel(double x1,double x2,double sigma)
{
	double t,d,sim;
	t = (x1 - x2);
	t = -1*t*t;
	d = 2*sigma*sigma;
	sim = exp(t/d);
	
	return sim;
}

int main(void) {
	__IO uint32_t i = 0, j =0;
		double X[] = {.7,0.7};
		double X1;
		double X2[groups], temp[groups];
		int idx;
		double p = 0;
		
		/*
		struct Model m1;
		for (idx=0; idx<groups; idx++){
			m1.alphas[idx] = alpha[idx];
			m1.y[idx] = alpha[idx];
			m1.X[idx][0] = X1[idx];
			m1.X[idx][1] = X2[idx];
		}	
		m1.w[0] = w[0]; m1.w[1] = w[1];
		m1.b = b;
		*/
		printf("Bismillah Hir Rahman Nir Raheem \n");

		for(j=0; j < f_size ; ++j)
			X1 = X1 + X[j]*X[j];
		
		printf("mX1 = %f",X1);
				
		p = 0;
		for(i=0; i<groups; ++i){
			temp[i] = 0;
			for(j=0; j < f_size ; ++j)
			{
				//X2 = sum(model.X.^2, 2)'
				X2[i] += modelX[j][i] * modelX[j][i];
				
				//K1 = - 2 * X * model.X'
				temp[i] += X[j] * modelX[j][i];
			}
			
			//K2 = bsxfun(@plus, X2, K1)
			temp[i] = -2 * temp[i] + X2[i];// + X1;//K3
			
			//K3 = bsxfun(@plus, X1, K2)
			temp[i] = temp[i] + X1;
			
			//K4 = model.kernelFunction(1, 0) .^ K3
			temp[i] = pow(gaussianKernel(1,0,.1), temp[i]);
			
			//K5 = bsxfun(@times, model.y', K4)
			temp[i] = modelY[i] * temp[i];
			
			//K6 = bsxfun(@times, model.alphas', K5)
			temp[i] = modelAlphas[i] * temp[i];
			
			//p = sum(K6, 2)
			p = p + temp[i];
		}
		
		printArray(temp,groups);

		printf("Decision Variable = %e", p);

		time_var1=0;
		STM_EVAL_LEDInit(LED6);
		STM_EVAL_LEDInit(LED4);
		Init_Recv_Node();

		/*Angle31=0x33;	
		USART_puts(USART2,0x35);
		USART_puts(USART2,'\n');*/
		while(1)	{	}	
		return 0;
}





