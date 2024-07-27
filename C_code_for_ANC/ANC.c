#include "anc.h"

int AncGetMem()
{
	return sizeof(tANC);
}

void Initialize_ANC(void *ptr)
{
	int i;

	tANC* anc = (tANC *)ptr;

	for (i = 0; i < X_SIZE; i++)
		anc->X_States[i] = 0.0;

	for (i = 0; i < SECONDARY_PATH_SIZE; i++)
		anc->Secondary_States[i] = 0.0;

	for (i = 0; i < A_SIZE; i++)
		anc->A_States[i] = 0.0;

	for (i = 0; i < Y_SIZE; i++)
		anc->Y_States[i] = 0.0;

	for (i = 0; i < W_SIZE; i++)
		anc->W_filt[i] = 0.0;


	for (i = 0; i < SECONDARY_EST_PATH; i++)
	{
		anc->shat_buf[i] = 0.0;
		anc->s_buf[i] = 0.0;
	}
	for (i = 0; i < PRIM_EST_PATH; i++)
	{
		anc->p_buf[i] = 0.0;
		anc->x_buf[i] = 0.0;
	}
}


//for the updation of these buffers
void Update_States(double* buf, int size)
{
	int i;
	for (i = 1; i < size; i++)
	{
		buf[i] = buf[i - 1];
	}
}

// The following function applies filter


double Apply_Filter(double* states, double* filt, int size)
{
	int i;
	double output = 0.0;
	for (i = 0; i < size; i++)
		output += (filt[i] * states[i]);
	return output;
}

void ANC_main_loop(void *ptr, double* x_buf, double* a_buf, int size)
{
	int i;
	tANC* anc = (tANC*)ptr;

	for (i = 0; i < size; i++)
	{
		double d, y;

		Update_States(anc->X_States, X_SIZE); // Shift the filter state by 1
		anc->X_States[0] = x_buf[i]; // Read next input sample
		d = Apply_Filter(anc->X_States, Prim_filt, PRIMARY_PATH_SIZE);
		y = Apply_Filter(anc->X_States, anc->W_filt, W_SIZE);

		Update_States(anc->A_States, A_SIZE);
		anc->A_States[0] = a_buf[i];

		Update_States(anc->Y_States, Y_SIZE);
		anc->Y_States[0] = y;
	}

}

