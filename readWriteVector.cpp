//para operaciones opencv
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include "defines.h"

using namespace cv;
using namespace std;

char refFileName[100];
char vectorFileName[20];


typedef struct
{
    double **vectorList;
    int numVectors;
}readFiles;

void writePointsHeigh(float *vector)
{
      FILE *fp;
    sprintf(vectorFileName,"PuntosAltura.txt");
    int puntos = (int)vector[0];
    int altura = vector[6];

      if ((fp = fopen(vectorFileName, "at")) == NULL)
        {
         cout << "[ERROR] File " << vectorFileName << " not open" << endl;
        }
    else
       { fprintf(fp, "%d , %d; \n",puntos, altura);}
    fclose(fp);

}


void writeVectorFile(double *vector, int num)
{
    //char vectorFileName[100];
      FILE *fp;

sprintf(vectorFileName,"sombreropca.data");
       double arrayAux[NUMCARAC];
    for (int i=0; i<NUMCARAC; i++)
        arrayAux[i]=vector[i];
     if ((fp = fopen(vectorFileName, "ab")) == NULL)
        {
         cout << "[ERROR] File " << vectorFileName << " not open" << endl;
        }
    else
       { fwrite(arrayAux, 1, sizeof(arrayAux), fp);}
    fclose(fp);

}

readFiles readVectorFile(readFiles structRead, char *FileName)
{
  FILE *fp;

  sprintf(vectorFileName,FileName);
  if ((fp = fopen(vectorFileName, "rb")) == NULL)
      {
       cout << "[ERROR] File " << vectorFileName << " not open" << endl;
       structRead.numVectors = 0;
      }
  else
  {
      fseek(fp, 0L, SEEK_END);
      int numVectors = ftell(fp)/(sizeof(double)*NUMCARAC);
      rewind(fp);
      structRead.vectorList = new double*[numVectors];
      //cout << "numero de vectores guardados = " << numVectors << endl;
      for (int i=0; i< numVectors; i++)
      {
        structRead.vectorList[i] = new double[NUMCARAC];
        fread(structRead.vectorList[i], 1, sizeof(double)*NUMCARAC, fp);

      }

      //cout << "ha leido todos los datos del fichero" << endl;
      structRead.numVectors = numVectors;

   }

 fclose(fp);

  return structRead;
}

