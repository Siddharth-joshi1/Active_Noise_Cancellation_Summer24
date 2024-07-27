#include <stdio.h>
#include <stdlib.h>
#include "ancheader.h"




int main(int argc, char *argv[])
{
	void* ptr;

	int memsize;

	double x_buf[BLOCK_SIZE]; // Noise
	
	double a_buf[BLOCK_SIZE];  // Audio

	//read noise, audio and write output 
	
	FILE *fnoise, *faudio, *fout;

	short temp_x[BLOCK_SIZE], temp_a[BLOCK_SIZE];

	memsize = AncGetMem();

	ptr = malloc(memsize);


	Initialize_ANC(ptr);

//	printf("\n argc = %d \n", argc);
//	printf("\nargv[0] = %s argv[1] = %s argv[2] = %s argv[3] = %s\n", argv[0], argv[1], argv[2], argv[3]);

	if (argc < 4)
	{
		printf("\n Anc <noise_file> <audio_file> <output_file>\n");
		exit(1);
	}

	if ((fnoise = fopen(argv[1], "rb")) == NULL)
	{
		printf("\n Could not open file %s", argv[1]);
		exit(1);
	}

	if ((faudio = fopen(argv[2], "rb")) == NULL)
	{
		printf("\n Could not open file %s", argv[2]);
		exit(1);
	}

	if ((fout = fopen(argv[3], "wb")) == NULL)
	{
		printf("\n Could not open file %s", argv[3]);
		exit(1);
	}

	
	while (1)
	{
		int i;
		if (fread(temp_x, sizeof(short), BLOCK_SIZE, fnoise) == BLOCK_SIZE)
			break;

		if (fread(temp_a, sizeof(short), BLOCK_SIZE, faudio) == BLOCK_SIZE)
			break;

		for (i = 0; i < BLOCK_SIZE; i++)
		{
			x_buf[i] = (double)temp_x[i] / 32768.0; //Normalize the input to be between -1.0 to 1.0
			a_buf[i] = (double)temp_a[i] / 32768.0;
		}

		ANC_main_loop(ptr, x_buf, a_buf, BLOCK_SIZE);
	}
}
