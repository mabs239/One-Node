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

#include "HelperFunctions.h"


#include "arm_math.h" 
#include "math_helper.h"	

//int f_size ;
//int groups ;

#define Rise_Const 12
#define Fall_Const 2
#define Samples 500
#define Noise_Samples 500
#define Corr_Result_Size (2*Samples)-1
#define Fs  45000
#define Vs  333


/*
struct Model
{
    double w [f_size];
		double b;
    double X [groups] [f_size];
    double y[groups];
    double alphas [groups];
};
*/

/*
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
*/

void printArray(double a[], int len);
//void Recieve();
//uint8_t getFlagID();
//uint8_t getExtiFlag();


//void svm_predict(struct Model a , double sample[f_size], int f_size);



__ALIGN_BEGIN USB_OTG_CORE_HANDLE     USB_OTG_dev  __ALIGN_END ;
volatile uint32_t time_var1;
unsigned char ADC0_Out_8,ADC1_Out_8,ADC2_Out_8;
volatile int ADC0_Out,ADC1_Out,ADC2_Out;


/* Define Model parameters*/


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

int Edge_Angle1,Edge_Angle2,Edge_Angle3,Region_Angle1,Region_Angle2,Region_Angle3,Angle3_Invalid;
int Edge_Angle1,Edge_Angle2,Edge_Angle3,Region_Angle12,Region_Angle23,Region_Angle31;
int Angle2_Invalid,Angle1_Invalid,Angle12_Invalid,Angle31_Invalid,Angle23_Invalid;
int Region,Edge;
int EdgeDefAngle[6]={330,30,90,150,210,270}; /////yet to be modified (Akif)

/////Can be better approximated (Hakeem)
uint8_t false_exti,timefirst=0;
long int result=0;

//double modelX[f_size][groups] ={{0.579000344, 0.796917226, 0.422056676, 0.237648833, 0.968937944, 0.770653925, 0.484163008 , 0.213185483	,0.219116214	,0.003703923	,0.000113883	,9.42E-05	,2.30E-06	,1.16E-06	,1.75E-07	,1.04E-08,	1.695387474,	2.271354862,	0.452291288,	0.227607391,	2.077405125,	2.313796625,	2.512395494,	2.580537307,	2.71198118,	2.506749483,	2.895398071	,2.110320158},
//{0.157368421,	0.108947368,	0.104736842	,0.092105263,	0.075263158,	0.078421053,	0.076315789,	0.075263158,	0.051052632	,0.092105263,	0.05,	0.042631579	,0.045789474,	0.056315789	,0.022631579,	0.028421053,	0.058421053	,0.079473684,	0.092105263,	0.074210526,	0.134210526,	0.219473684,	0.171052632,	0.23,	0.120526316	,0.235263158,	0.118421053,	0.184736842}};


//void printArray2D(int m, int n,double a[m][n]);
//void printArray2D(int m, int n,double **a);
							
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






#define USE_STATIC_INIT 
 
 /* ---------------------------------------------------------------------- 
** Global defines  
** ------------------------------------------------------------------- */ 
 
#define TEST_LENGTH_SAMPLES 	(20*4) 
 
/* ---------------------------------------------------------------------- 
** List of Marks scored by 20 students for 4 subjects 
** ------------------------------------------------------------------- */  
const float32_t testMarks_f32[TEST_LENGTH_SAMPLES] =  
{    
	42.000000,	37.000000,	81.000000,	28.000000,	 
	83.000000,	72.000000,	36.000000,	38.000000,	 
	32.000000,	51.000000,	63.000000,	64.000000,	 
	97.000000,	82.000000,	95.000000,	90.000000,	 
	66.000000,	51.000000,	54.000000,	42.000000,	 
	67.000000,	56.000000,	45.000000,	57.000000,	 
	67.000000,	69.000000,	35.000000,	52.000000,	 
	29.000000,	81.000000,	58.000000,	47.000000,	 
	38.000000,	76.000000,	100.000000,	29.000000,	 
	33.000000,	47.000000,	29.000000,	50.000000,	 
	34.000000,	41.000000,	61.000000,	46.000000,	 
	52.000000,	50.000000,	48.000000,	36.000000,	 
	47.000000,	55.000000,	44.000000,	40.000000,	 
	100.000000,	94.000000,	84.000000,	37.000000,	 
	32.000000,	71.000000,	47.000000,	77.000000,	 
	31.000000,	50.000000,	49.000000,	35.000000,	 
	63.000000,	67.000000,	40.000000,	31.000000,	 
	29.000000,	68.000000,	61.000000,	38.000000,	 
	31.000000,	28.000000,	28.000000,	76.000000,	 
	55.000000,	33.000000,	29.000000,	39.000000 
};  
 
 
/* ---------------------------------------------------------------------- 
* Number of subjects X 1  
* ------------------------------------------------------------------- */  
const float32_t testUnity_f32[4] =  
{    
	1.000,  1.000, 	1.000,  1.000 
}; 
 
 
/* ---------------------------------------------------------------------- 
** f32 Output buffer 
** ------------------------------------------------------------------- */  
static float32_t testOutput[TEST_LENGTH_SAMPLES]; 
 
 
/* ------------------------------------------------------------------ 
* Global defines  
*------------------------------------------------------------------- */ 
#define 	NUMSTUDENTS  20 
#define     NUMSUBJECTS  4 
 
/* ------------------------------------------------------------------ 
* Global variables  
*------------------------------------------------------------------- */ 
 
uint32_t  	numStudents = 20; 
uint32_t  	numSubjects = 4;  
//float32_t	max_marks, min_marks, mean, std; 
float	max_marks, min_marks, mean, std; 
uint32_t 	student_num;    
 
/* ---------------------------------------------------------------------------------- 
* Main f32 test function.  It returns maximum marks secured and student number 
* ------------------------------------------------------------------------------- */ 









int main(void) {
		double X[] = {.7,0.7};
		int idx;

  	arm_matrix_instance_f32 srcA; 
  	arm_matrix_instance_f32 srcB; 
  	arm_matrix_instance_f32 dstC;  
 
	/* Input and output matrices initializations */  
	arm_mat_init_f32(&srcA, numStudents, numSubjects, (float32_t *)testMarks_f32);  
	arm_mat_init_f32(&srcB, numSubjects, 1, (float32_t *)testUnity_f32);  
	arm_mat_init_f32(&dstC, numStudents, 1, testOutput);  
		
		arm_mat_mult_f32(&srcA, &srcB, &dstC); 
		arm_max_f32(testOutput, numStudents, &max_marks, &student_num);  
		arm_min_f32(testOutput, numStudents, &min_marks, &student_num);  
		arm_mean_f32(testOutput, numStudents, &mean); 
		arm_std_f32(testOutput, numStudents, &std); 
		
		svmPredict(X);
		printf("Result of addition: %i",add239(3,4));	
	
		for(idx = 0; idx <30; idx ++ )
			printf("\n Max Marks: %f",testOutput[idx]);	
			
			printf("\n Students:%i, Max=%f, Min=%f, Mean=%f, Std=%f",student_num, max_marks, min_marks, mean, std);	
	
		
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





