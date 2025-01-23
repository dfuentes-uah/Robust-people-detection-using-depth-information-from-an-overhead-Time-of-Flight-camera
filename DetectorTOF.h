typedef struct
{
    int contMuestras;
    int acumulate[8];
}muestrasEntrenamiento;

typedef struct
{
    int **vectorList;
    int numVectors;
}readFiles;

int  TOFdetector(int16_t *z2D16, int cont_framesR, char *sequenceName);
void writeVectorFile(int *vector, int num);
readFiles readVectorFile(readFiles structRead);
int detector(int16_t *z2D16, int cont_framesR);
