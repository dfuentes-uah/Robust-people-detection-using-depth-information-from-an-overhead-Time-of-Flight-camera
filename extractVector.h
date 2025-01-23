typedef struct
  {
    int contMuestras;
    int acumulate[8];
  } muestrasEntrenamientoo;

typedef struct
  {
    int **vectorList;
    int numVectors;
  } readFiless;


muestrasEntrenamientoo  acumulateVector(int16_t *z2D16, int cont_framesR, muestrasEntrenamientoo infEntren,TSrcLocationResult pmd3GT);

void writeVectorFile(int *vector, int num);
readFiless readVectorFile(readFiless structRead);
