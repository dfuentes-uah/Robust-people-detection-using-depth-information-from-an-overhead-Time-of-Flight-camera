typedef struct
{
    double **vectorList;
    int numVectors;
}readFiles;

void writePointsHeigh(float *vector);
void writeVectorFile(double *vector, int num);
readFiles readVectorFile(readFiles structRead, char *FileName);

