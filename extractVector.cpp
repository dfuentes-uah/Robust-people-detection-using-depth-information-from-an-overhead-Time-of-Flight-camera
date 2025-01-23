#include <iostream>
//para operaciones opencv
#include <stdio.h>
#include <stdlib.h>
//#include <pmdsdk2.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
///SEGMENTACION
#include <vector>
//#include "head_shoulder.h"
#include "Funciones_opencv.h"
//#include "Funciones_pcl.h"
//#include "geom_forms.h"
#include "geom_filters.h"
#include "clasificador.h"
#include "maxDetector.h"
#include "direccion.h"
#include "histogramasec.h"
#include "tiempo.h"
#include "saveResult.h"



//lectura de .gt
#include <math.h>
#include <string.h>
//#include "memoria/memoria.h"
#include "geometryLib/geometrylib.h"
#include "tofCountingGeneralLib/tofCountingGeneralLib.h"
#include "tofCountingScoring/tofScoringFunction.h"
#include "../../far-field/sourceLocationResultsLib/sourcelocationresultslib.h"

#define SHOW_FRAME 1
#define READWRITE 0//0 READ 1 WRITE
#define maxSUELO 3400


double minValv2,maxValv2;
char refFileName2[100];
char vectorFileName2[20];


typedef struct
{
    int contMuestras;
    int acumulate[8];
}muestrasEntrenamientoo;

typedef struct
{
    int **vectorList;
    int numVectors;
}readFiless;


void writeVectorFile(int *vector, int num)
{
      FILE *fp;
    sprintf(vectorFileName2,"vectorAcumNorm2.data");
    int arrayAux[num];
    for (int i=0; i<num; i++)
        arrayAux[i]=vector[i];
    if ((fp = fopen(vectorFileName2, "ab")) == NULL)
        {
         cout << "[ERROR] File " << vectorFileName2 << " not open" << endl;
        }
    else
       { fwrite(arrayAux, 1, sizeof(arrayAux), fp);}
    fclose(fp);


}

readFiless readVectorFile(readFiless structRead)
{
  FILE *fp;

  sprintf(vectorFileName2,"vectorAcumNorm.data");
  if ((fp = fopen(vectorFileName2, "rb")) == NULL)
      {
       cout << "[ERROR] File " << vectorFileName2 << " not open" << endl;
       structRead.numVectors = 0;
      }
  else
  {
      fseek(fp, 0L, SEEK_END);
      int numVectors = ftell(fp)/(sizeof(int)*5);
      rewind(fp);
      structRead.vectorList = new int*[numVectors];
      cout << "numero de vectores guardados = " << numVectors << endl;
      for (int i=0; i< numVectors; i++)
      {
        structRead.vectorList[i] = new int[5];
        fread(structRead.vectorList[i], 1, sizeof(int)*5, fp);

      }

      cout << "ha leido todos los datos del fichero" << endl;
      structRead.numVectors = numVectors;

   }

 fclose(fp);
 cout << "cierra el fichero" << endl;

  return structRead;
}
muestrasEntrenamientoo  acumulateVector(int16_t *z2D16, int cont_framesR, muestrasEntrenamientoo infEntren,TSrcLocationResult pmd3GT)
{


///VARIABLES

    int width = 423;
    int height = 511;
    int size_array = 217088;
    int numCarac = 5;

    ///vector acumulativo para ir añadiendo los vectores de puntos calculados(BORRAR)
    double arrayVector[numCarac];
    for (int a=0; a < numCarac; a++)
        arrayVector[a]=infEntren.acumulate[a];




    cout << "[NUMERO DEL FRAME = "<< cont_framesR << " ]" << endl;
  // if(pmd3GT.numSimultaneousSpeakers[cont_framesR] == 1 && pmd3GT.sourcePosition[cont_framesR][0].x < 420 && pmd3GT.sourcePosition[cont_framesR][0].x >90 && pmd3GT.sourcePosition[cont_framesR][0].y < 330 && pmd3GT.sourcePosition[cont_framesR][0].y > 90)
    //if(pmd3GT.numSimultaneousSpeakers[cont_framesR] == 0)
    //   {
   if (cont_framesR > 475)
        {
            cout <<"[INFO] frame 1" << endl;
            int16_t* ZsinCeros = new int16_t[size_array];
            Mat zMat(424,512,CV_16UC1,z2D16);
            Mat zMatMaximos;

            pixelesErroneosSuelo(zMat,&zMat);
            minMaxLoc(zMat, &minValv2, &maxValv2);
            cout << "maximo suelo = " << maxValv2 << endl;
            zMat.convertTo(zMatMaximos, CV_8U, 255.0/(maxValv2 - minValv2), -minValv2 * 255.0/(maxValv2 - minValv2));




            ///DETECTOR DE MÁXIMOS POR SOMBRERO MEXICANO//
            /*
            Mat propro = zMat.clone();
            ruido_uint16(propro, &propro);
            Mat preproces = preProcess(propro);
            Mat mexCovolution = mexican_filter(preproces, 1);
            mexicanMaximos(mexCovolution,preproces );
            */

            if (SHOW_FRAME)
            {
                Mat zMat_SHOW = zMat.clone();
                minMaxLoc(zMat_SHOW, &minValv2, &maxValv2);
                zMat_SHOW.convertTo(zMat_SHOW, CV_8U, 255.0/(maxValv2 - minValv2), -minValv2 * 255.0/(maxValv2 - minValv2));
                imshow("imagen sin filtro", zMat_SHOW);
            }


            ////////////////////////////////////7DETECTOR DE MÁXIMOS//
            ///array de secciones para el detector de maximos
                 ///DETECTOR DE MÁXIMOS//
            ///array de secciones para el detector de maximos
            int **pointsReturn;
            maxData *maxReturn;
            pointsReturn = new int *[10];
            for (int i=0;i<10;i++)
                pointsReturn[i]=new int[5];

            int numMax=0;


             cout << "antes de maxDetector" << endl;
            //maxDetector(zMat, maxReturn, &numMax,cont_frames);
            cout << "despues de maxDetector" << endl;

            cout<< "vectorees que ha obtneido = ..." << endl;
            if (READWRITE)
              {
                //for (int i =0;i <numMax;i++)
                  //  writeVectorFile(maxReturn->pointsReturn[i], numCarac);
              }
              else
              {
                readFiless structRead;
                structRead = readVectorFile(structRead);
                Mat_<float>samples = Mat_<float>(structRead.numVectors,numCarac);
                for(int s1=0; s1<structRead.numVectors; s1++)
                   {
                     for(int s2=0; s2<numCarac; s2++)
                     samples.at<float>(s1,s2) = (float)structRead.vectorList[s1][s2];
                   }
                //cout << "matriz guardada = " << endl;
                //cout <<samples << endl;
               }


        }

   /*
            Rect ROI =   Rect ( 50 , 50 , height - 100 , width-100);
            Mat zMatRoi = zMat(ROI);
            Mat sinCeros=zMatRoi.clone();

            for (int i=0; i<zMatRoi.rows; i++)
            {
                for(int j=0; j<zMatRoi.cols; j++)
                {
                    if (sinCeros.at<unsigned short>(i,j)==0)
                    {
                       sinCeros.at<unsigned short>(i,j)=nnSearch(sinCeros, i,j,10);
                    }
                }
            }

            Mat sinCerosMedian = sinCeros.clone();
            medianBlur(sinCeros,sinCerosMedian,3);
            //struct_count contornos = contador_contour(sinCeros);


            if (SHOW_FRAME)
            {
                Mat sinCerosMedian_SHOW = sinCerosMedian.clone();

                minMaxLoc(sinCerosMedian_SHOW, &minValv2, &maxValv2);
                sinCerosMedian_SHOW.convertTo(sinCerosMedian_SHOW, CV_8U, 255.0/(maxValv2 - minValv2), -minValv2 * 255.0/(maxValv2 - minValv2));
                imshow("frame",sinCerosMedian_SHOW );
                waitKey();

            }

            int *actualVector;
            actualVector = heighNumPoints(sinCerosMedian, actualVector);
            /*
            cout << "VECTOR DE MEDIDAS = " << endl;
            cout << actualVector[0] << endl;
            cout << actualVector[1] << endl;
            cout << actualVector[2] << endl;
            cout << actualVector[3] << endl;
            cout << actualVector[4] << endl;
            cout << actualVector[5] << endl;
            */
/*
            double maxHeight, minHeight;
            minMaxLoc(sinCerosMedian, &minHeight, &maxHeight);


            Mat actualVector_mat =  Mat::zeros(numCarac, 1, CV_64FC1);
            double variableNorm = 0.71*(maxHeight-minHeight)+1033;
            cout<< "variableNorm = " << variableNorm << endl;

            for (int hh =0;hh<numCarac; hh++)
                actualVector_mat.at<double>(hh,0) = actualVector[hh] / variableNorm;
            for (int hh =0;hh<numCarac; hh++)
                arrayVector[hh] = actualVector[hh] / variableNorm;

            /*
            cout << "actual antes de entrar = " << actualVector_mat << endl;
            cout << "VECTOR DE MEDIDAS = " << endl;
            cout << actualVector_mat.at<double>(0,0) << endl;
            cout << actualVector_mat.at<double>(1,0) << endl;
            cout << actualVector_mat.at<double>(2,0) << endl;
            cout << actualVector_mat.at<double>(3,0) << endl;
            cout << actualVector_mat.at<double>(4,0) << endl;
            cout << actualVector_mat.at<double>(5,0) << endl;
            */


            //if (actualVector[0] > 100)
            //{

            ///////////////// LECTURA O ESCRITURA DE LOS VECTORES DE ENTRENAMIENTO
            /*               ///GRABACIÓN Y LECTURA DE FICHEROS
                /*
                for (int p=0; p<numCarac; p++)
                    actualVector[p] =
                */

 /*               if (READWRITE)
                {
                    cout << "VECTOR DE MEDIDAS A GUARDAR = " << endl;
                    cout << arrayVector[0] << endl;
                    cout << arrayVector[1] << endl;
                    cout << arrayVector[2] << endl;
                    cout << arrayVector[3] << endl;
                    cout << arrayVector[4] << endl;
                    cout << arrayVector[5] << endl;
                    writeVectorFile(arrayVector, numCarac);
                }
                else
                {
                    readFiless structRead;
                    structRead = readVectorFile(structRead);



                   Mat_<float>samples = Mat_<float>(structRead.numVectors,numCarac);
                    for(int s1=0; s1<structRead.numVectors; s1++)
                    {
                        for(int s2=0; s2<numCarac; s2++)
                            samples.at<float>(s1,s2) = (float)structRead.vectorList[s1][s2];
                    }



                    Mat cov, mu;
                    calcCovarMatrix(samples, cov, mu, CV_COVAR_NORMAL | CV_COVAR_ROWS);

                    float distancia = mahalanovisDist(actualVector_mat, mu, cov, numCarac);
                    PCAclasification(samples, actualVector_mat, numCarac);

                    if (distancia < 1.5)
                    {
                        ///Pintar un punto cuando es persona
                        Mat ZmatColor;
                        Mat aux = sinCerosMedian.clone();
                        Point2f maxPoint;
                        cvtColor(sinCerosMedian, ZmatColor, CV_GRAY2RGB);
                        minMaxLoc(ZmatColor, &minValv2, &maxValv2);
                        ZmatColor.convertTo(ZmatColor, CV_8U, 255.0/(maxValv2 - minValv2), -minValv2 * 255.0/(maxValv2 - minValv2));
                        maxPoint =  maxPointMat(aux);
                        circle( ZmatColor, maxPoint, 10, Scalar(255,0,0), -1, 8, 0 );

                        imshow("ZmatColor",ZmatColor);
                        waitKey();
                    }

                    for (int a=0; a < 6; a++)
                    {

                        infEntren.acumulate[a]=infEntren.acumulate[a] + actualVector[a];
                        //arrayVector[a]=infEntren.acumulate[a];
                        arrayVector[a]=actualVector[a];

                    }
                }

                infEntren.contMuestras++;



            }

//       }


*/


    return infEntren;
}



