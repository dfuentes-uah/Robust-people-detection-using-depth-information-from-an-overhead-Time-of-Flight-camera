typedef struct
{
    double preprocesado;
    double detectormaximos;
    double Extraccioncaracteristicas;
    double busquedaSR;
    double Clasificacion;
    double filtromex;
    double detectorpicos;
    int frame;
    int numeromaximos;
    int numeropersonas;

} tiempos;

int fintiempo(clock_t inicio);
clock_t comienzotiempo();
int guardatemporizacion(tiempos algoritmo);
int mextemporizacion(tiempos algoritmo);
void saveTimes(double time, char tipeTime, int cont_frames);

