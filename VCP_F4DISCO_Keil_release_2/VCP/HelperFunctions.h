#ifndef HelperFunctions
#define HelperFunctions
//HelperFunctions.h
int add239(int x, int y);
double gaussianKernel(double x1,double x2,double sigma);
void printArray(double a[], int len);
void printArray2D(int m, int n,double a[3][3]);
int svmPredict(double X[]);

//extern  int f_size;
//extern int groups;
#define groups 28
#define f_size 2

extern double modelX[2][28];
extern const double modelY[];
extern double modelAlphas[];
extern double modelW[];
extern double modelB;
extern int modelLength;


/*
const int f_size = 2;
const int groups = 28;
double modelX[f_size][groups] ={{0.579000344, 0.796917226, 0.422056676, 0.237648833, 0.968937944, 0.770653925, 0.484163008 , 0.213185483	,0.219116214	,0.003703923	,0.000113883	,9.42E-05	,2.30E-06	,1.16E-06	,1.75E-07	,1.04E-08,	1.695387474,	2.271354862,	0.452291288,	0.227607391,	2.077405125,	2.313796625,	2.512395494,	2.580537307,	2.71198118,	2.506749483,	2.895398071	,2.110320158},
{0.157368421,	0.108947368,	0.104736842	,0.092105263,	0.075263158,	0.078421053,	0.076315789,	0.075263158,	0.051052632	,0.092105263,	0.05,	0.042631579	,0.045789474,	0.056315789	,0.022631579,	0.028421053,	0.058421053	,0.079473684,	0.092105263,	0.074210526,	0.134210526,	0.219473684,	0.171052632,	0.23,	0.120526316	,0.235263158,	0.118421053,	0.184736842}};
double modelY[] = {	1	,1	,1	,1	,1	,1	,1	,1	,1	,1	,1	,1	,1	,1	,1	,1	,-1,	-1	,-1	,-1	,-1	,-1	,-1	,-1	,-1	,-1	,-1	,-1};
double modelAlphas[] = {	0.454251936	,0.321773341,	11.7139214,	7.945854578	,0.855086621,	0.447928742,	10.27969317,	5.47384088,	7.444460018,	0.473481755,	1.39E-17,	2.17E-17,	2.95E-17,	5.21E-16,	0.417961766,	0.000917648,	1.011944606	,0.644913297,	20	,20,	0.668406582,	0.631434696,	0.325245149,	0.375024182	,0.703681435,	0.309626792,	0.8806807,	0.278214418};
double modelW[] = {-10.93210935, -0.325816793};
double modelB = 0.0116;
int modelLength = 28;
*/
#endif
