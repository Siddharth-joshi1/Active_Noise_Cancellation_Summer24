#define BLOCK_SIZE				32


extern void Initialize_ANC(void *ptr);

extern void ANC_main_loop(void *ptr, double* x_buf, double* a_buf, int size);

extern int AncGetMem();

