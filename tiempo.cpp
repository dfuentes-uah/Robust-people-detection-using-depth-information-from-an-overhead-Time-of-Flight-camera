#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <time.h>
using namespace cv;
using namespace std;

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

}tiempos;

int mextemporizacion(tiempos algoritmo)
{
    char name[100] = "temporizacionmex.data";
    FILE *fp;
    //chapuza para poder meter las posiciones de los máximos en la estructura
    if ((fp = fopen(name, "at")) == NULL)
      {
           cout << "[ERROR] File " << name << " not open" << endl;
      }
    else
        //fwrite(&Results, sizeof(st_resultSave), 1, fp);
        fprintf(fp,"%06ld", (long int)algoritmo.frame);
    fprintf(fp, " %4f %4f %4f %d", algoritmo.preprocesado,algoritmo.filtromex,algoritmo.detectorpicos,algoritmo.numeromaximos);
      fprintf(fp, "\n");
    fclose(fp);

}
int guardatemporizacion(tiempos algoritmo)
{
    char name[100] = "temporizacion.data";
    FILE *fp;
    //chapuza para poder meter las posiciones de los máximos en la estructura
    if ((fp = fopen(name, "at")) == NULL)
      {
           cout << "[ERROR] File " << name << " not open" << endl;
      }
    else
        //fwrite(&Results, sizeof(st_resultSave), 1, fp);
        fprintf(fp,"%06ld", (long int)algoritmo.frame);
    fprintf(fp, " %4f %4f %4f %4f %4f %d %d", algoritmo.preprocesado,algoritmo.detectormaximos,(algoritmo.busquedaSR*algoritmo.numeromaximos),(algoritmo.Extraccioncaracteristicas*algoritmo.numeromaximos),(algoritmo.Clasificacion*algoritmo.numeromaximos),algoritmo.numeromaximos,algoritmo.numeropersonas);
      fprintf(fp, "\n");
    fclose(fp);

}

clock_t comienzotiempo()
{

  clock_t comienzo = clock();

  return comienzo;
}

int fintiempo(clock_t comienzo)
{
  clock_t final = clock();

   float diff = (((float)final - (float)comienzo / 1000000.0F) * 1000);
  printf("El tiempo transcurrido es de %.4f segundos\n" ,diff);
  return 0;
}
void saveTimes(double time, char tipeTime, int cont_frames) //0 filterTime - 1 maxDetectorTime - 2 clasificatorTime 3
{
    FILE *fp;
    switch (tipeTime)
    {       case '0':
                if ((fp = fopen("Tiempos/FilterTime.data", "at")) == NULL)
                    {
                     cout << "[ERROR] File " << "FilterTime.data" << " not open" << endl;
                    }
                else
                    fprintf(fp,"%4d %f \n",(long int)cont_frames,time);
                fclose(fp);
            break;

            case '1':
                if ((fp = fopen("Tiempos/maxDetectorTime.data", "at")) == NULL)
                    {
                     cout << "[ERROR] File " << "FilterTime.data" << " not open" << endl;
                    }
                else
                    fprintf(fp,"%4d %f \n",(long int)cont_frames,time);
                fclose(fp);
            break;

            case '2':
                if ((fp = fopen("Tiempos/ClasificatorTime.data", "at")) == NULL)
                    {
                     cout << "[ERROR] File " << "FilterTime.data" << " not open" << endl;
                    }
                else
                    fprintf(fp,"%4d %f \n",(long int)cont_frames,time);
                fclose(fp);
            break;

            case '3':
                if ((fp = fopen("Tiempos/direccionesTime.data", "at")) == NULL)
                    {
                     cout << "[ERROR] File " << "FilterTime.data" << " not open" << endl;
                    }
                else
                    fprintf(fp,"%4d %f \n",(long int)cont_frames,time);
                fclose(fp);
            break;

            case '4':
                if ((fp = fopen("Tiempos/vecCaractTime.data", "at")) == NULL)
                    {
                     cout << "[ERROR] File " << "FilterTime.data" << " not open" << endl;
                    }
                else
                    fprintf(fp,"%4d %f \n",(long int)cont_frames,time);
                fclose(fp);
            break;
    case '5':
        if ((fp = fopen("Tiempos/mexpreprocess.data", "at")) == NULL)
            {
             cout << "[ERROR] File " << "mexpreprocess.data" << " not open" << endl;
            }
        else
            fprintf(fp,"%4d %f \n",(long int)cont_frames,time);
        fclose(fp);
    break;
    case '6':
        if ((fp = fopen("Tiempos/mexfilter.data", "at")) == NULL)
            {
             cout << "[ERROR] File " << "mexfilter.data" << " not open" << endl;
            }
        else
            fprintf(fp,"%4d %f \n",(long int)cont_frames,time);
        fclose(fp);
    break;
    case '7':
        if ((fp = fopen("Tiempos/mexpicosfinales.data", "at")) == NULL)
            {
             cout << "[ERROR] File " << "mexfilter.data" << " not open" << endl;
            }
        else
            fprintf(fp,"%4d %f \n",(long int)cont_frames,time);
        fclose(fp);
    break;
    }
}
