#define PRIMARY_PATH_SIZE		7
#define SECONDARY_PATH_SIZE		7
#define SECONDARY_EST_PATH		16
#define PRIM_EST_PATH			16
#define W_SIZE					32   // NLMS filters
#define X_SIZE					64
#define A_SIZE					64
#define Y_SIZE					64


// Primary and Secondary Path Filters
double Prim_filt[PRIMARY_PATH_SIZE] = { 0.1, 0.25, 0.15, 0.1, 0.25, 0.25, 0.01 };//global variables
double Sec_filt[SECONDARY_PATH_SIZE] = { 0.1, 0.25, 0.15, 0.1, 0.25, 0.25, 0.01 };


typedef struct sANC {
	double X_States[X_SIZE];
	double Y_States[Y_SIZE];


	double Secondary_States[SECONDARY_PATH_SIZE];

	double A_States[A_SIZE];


	double shat_buf[SECONDARY_EST_PATH];
	double s_buf[SECONDARY_EST_PATH];
	double p_buf[PRIM_EST_PATH];
	double x_buf[PRIM_EST_PATH];
	double y_buf[Y_SIZE];
	double W_filt[W_SIZE];
}tANC;

// Memory to store Primary and Secondary Filter States

