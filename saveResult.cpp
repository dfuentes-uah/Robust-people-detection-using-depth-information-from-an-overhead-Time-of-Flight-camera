#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <iostream>
#include <fstream>
#include "direccion.h"
#include "secciones.h"
#include "tiempo.h"
#include "defines.h"
#include "histogramasec.h"

#include "maxDetector.h"

#define maxSUELO 3400

using namespace cv;
using namespace std;
char heightsFile[20];

typedef struct
{
    int Frame;
    int PCAresult[maxDETECTIONS];
    Point position[maxDETECTIONS];
    int numMax;
}st_resultSave;





/////////////////////////////////////////////////////////////////////////////////
///////////////////////      GUARDAR DATOS EN PDF       /////////////////////////
/////////////////////////////////////////////////////////////////////////////////
int saveResults(char *FileName, int contFrame,double PCAres, int posX, int posY, int altura)
{
    char name[30];
    FILE *fp;
    sprintf(name,FileName);

    if ((fp = fopen(name, "at")) == NULL)
      {
           cout << "[ERROR] File " << name << " not open" << endl;
      }
    else
       {
        fprintf(fp,"FRAME %d --------------------------------------\n", contFrame);
        fprintf(fp, "\t PCA = %f \t Altura = %d \t Posición XY = (%d, %d) \n", PCAres, maxSUELO-altura, posX, posY);
       }
      fclose(fp);
      return 1;

}

int saveResultsCounting(const char *FileName, maxData MaxResults, int Frame)
{

    char name[100] = ""
                     "";
    strcat(name, FileName);
    FILE *fp;
    st_resultSave Results;
    //chapuza para poder meter las posiciones de los máximos en la estructura
    Point aux[] = {Point(0,0),Point(0,0),Point(0,0),Point(0,0),Point(0,0),Point(0,0),Point(0,0),Point(0,0),Point(0,0),Point(0,0)};
    for(int i=0;i<MaxResults.numMax;i++)
        aux[i] = MaxResults.position.at(i);

    if ((fp = fopen(name, "at")) == NULL)
      {
           cout << "[ERROR] File " << name << " not open" << endl;
      }
    else
        //fwrite(&Results, sizeof(st_resultSave), 1, fp);
        fprintf(fp,"%06ld", (long int)Frame);
     for(int i=0;i<MaxResults.numMax;i++)fprintf(fp, " %d %4d %4d %4f ", (int)1, aux[i].x*20+MaxResults.subroipos.at(i).x, aux[i].y*20+MaxResults.subroipos.at(i).y,(double)MaxResults.PCAresult[i]);
      fprintf(fp, "\n");
    fclose(fp);
    return 1;
}

/////////////////////////////////////////////////////////////////////////////////
///////////////////////      RELACIÓN PUNTOS Y ALTURA       /////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/// 0 = leer / 1 = escribir
int pointsHeighRelation (int height, int points, int WR)
{
    FILE *fp;
    sprintf(heightsFile,"heightsFile.data");
    Mat grafica (500, 1000, CV_8UC3,Scalar( 0,0,0) );
    for(int i=0;i<20;i++)
        line (grafica, Point(i*50,0),Point(i*50,500),Scalar(255,255,255),1,8,0);

    if (WR == 1)
    {
        if ((fp = fopen(heightsFile, "ab")) == NULL)
            {
             cout << "[ERROR] File " << heightsFile << " not open" << endl;
            }
        else
           { fwrite(&height,1, sizeof(int),fp);
             fwrite(&points,1, sizeof(int),fp);}
    }

    else
    {
        if ((fp = fopen(heightsFile, "rb")) == NULL)
            {
             cout << "[ERROR] File " << heightsFile << " not open" << endl;

            }
        else
        {
            fseek(fp, 0L, SEEK_END);
            int numData = ftell(fp)/(sizeof(int)*2); //número de datos que he guardado (mitad puntos y mitad alturas)
            rewind(fp);
            vector<int> alturas;
            vector<int> puntos;
            int aux1,aux2;
            for (int i=0;i < numData; i++)
            {
               fread(&aux1, 1, sizeof(int), fp);
               alturas.push_back(aux1);
               //cout << "alturas = " << alturas.at(i);
               fread(&aux2, 1, sizeof(int), fp);
               puntos.push_back(aux2);
               //cout << "puntos = " << puntos.at(i)<< endl;

               circle( grafica,Point(puntos.at(i)/2,500 - alturas.at(i)/2),3,Scalar( 0, 0, 255 ),-1,8 );
            }

             //imshow("grafica de puntos", grafica);
             waitKey();

        }
    }
    fclose(fp);
}
