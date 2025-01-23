#include <iostream>

#include <iostream>



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


///SEGMENTACION

#include <vector>

#include "Funciones_opencv.h"
#include "geom_filters.h"
#include "clasificador.h"
#include "maxDetector.h"
#include "direccion.h"
#include "secciones.h"
#include "histogramasec.h"
#include "saveResult.h"
#include "defines.h"
#include "readWriteVector.h"
#include "tiempo.h"
#include "SVM.h"


//lectura de .gt
#include <math.h>
#include <string.h>
#include "memoria/memoria.h"
#include "geometryLib/geometrylib.h"
#include "tofCountingGeneralLib/tofCountingGeneralLib.h"
#include "tofCountingScoring/tofScoringFunction.h"
#include "../../far-field/sourceLocationResultsLib/sourcelocationresultslib.h"
#define VISUALIZACION 1
#define MEDIA 0
#define MEDIANA 1
#define READWRITE 0 //0 READ 1 WRITE
#define SAVE_RESULT 1//0 noSave - 1 Save
#define SHOW_VECTOR 0
#define ENTRENA 0
#define PCA 1
#define SVM 0
Mat resultadoo(424,512,CV_8UC1,Scalar(255));
const int alpha_slider_max = 1;
int alpha_slider=0;
double alpha;
double beta;
double minValv,maxValv;
tiempos algoritmo;
void on_trackbar( int, void* )
{
}
FILE * pFile;
 int buffer[1];

readFiles sombrero,persona,objeto,structRead,estructura;
int detector(int16_t *z2D16, int cont_frames)
{

    if(ENTRENA==1 && cont_frames==1)
    {

        char *FileName="sombrero2.data";
        char *FileName2="persona2.data";
        char *FileName3="objetos.data";
      sombrero=readVectorFile(structRead,FileName);
      persona=readVectorFile(structRead,FileName2);
      objeto=readVectorFile(structRead,FileName3);

      entrenasvm(sombrero.vectorList,sombrero.numVectors,persona.vectorList,persona.numVectors,objeto.vectorList,objeto.numVectors);
    }
    if(ENTRENA==0)
    {

         if(VISUALIZACION==1)
         {
         /// Create Windows
         namedWindow("menu", 1);

         /// Create Trackbars
         char TrackbarName[50];
         sprintf( TrackbarName, "Alpha x %d", alpha_slider_max );

         createTrackbar( TrackbarName, "Linear Blend", &alpha_slider, alpha_slider_max, on_trackbar );

         /// Show some stuff
         on_trackbar( alpha_slider, 0 );
         }
///////////////////////////////////////
    string resultPath("seq-P04-M04-A0001-G01-C00-S0033.result");
    int position = resultPath.find('.');
    resultPath  =resultPath.substr(0,position);
    resultPath = resultPath + ".result";
    int size_array = 217088;

        cout << "[NUMERO DEL FRAME = "<< cont_frames << " ]" << endl;

                int16_t* ZsinCeros = new int16_t[size_array];
                Mat zMat(424,512,CV_16UC1,z2D16);
/*char nombre[80];
sprintf(nombre,"dep-%d.png",cont_frames);
imwrite(nombre,zMat);*/

                    Mat zMatInit_SHOW = zMat.clone();
                    minMaxLoc(zMatInit_SHOW, &minValv, &maxValv);
                    zMatInit_SHOW.convertTo(zMatInit_SHOW, CV_8U, 255.0/(maxValv - minValv), -minValv * 255.0/(maxValv - minValv));
                    if(VISUALIZACION==1)
                     {
                    imshow("imagen incial", zMatInit_SHOW);
                     }

                ///===========================================================FILTRADO DE LA IMAGEN
                int64 t0= getTickCount();
                pixelesErroneosSuelo(zMat,&zMat);
                ruido_uint16(zMat, &zMat);
                if(MEDIANA==1)medianBlur(zMat,zMat,3);
                  if(MEDIA==1)blur( zMat,zMat, Size( 3, 3 ), Point(0,0) );
                int64 t1 = getTickCount();
                double secs = (t1-t0)/getTickFrequency();
                algoritmo.preprocesado=secs;
                saveTimes(secs, '0',cont_frames);
                Mat zMat_SHOW = zMat.clone();
                minMaxLoc(zMat_SHOW, &minValv, &maxValv);
                zMat_SHOW.convertTo(zMat_SHOW, CV_8U, 255.0/(maxValv - minValv), -minValv * 255.0/(maxValv - minValv));
                if(VISUALIZACION==1)
                 {
                imshow("imagen sin filtro", zMat_SHOW);
                 }
                ///===========================================================DETECTOR DE MÁXIMOS
                ///Inicialización estructura de máximos
                maxData maxReturn;

                maxReturn.pointsReturn = new float *[maxDETECTIONS];
                double **pointsNorm;
                pointsNorm = new double*[maxDETECTIONS];
                for (int i=0;i<maxDETECTIONS;i++)
                   { maxReturn.pointsReturn[i]=new float[NUMCARAC+1];
                    pointsNorm[i]= new double[NUMCARAC];}
                for(int i=0;i<maxDETECTIONS;i++)
                {maxReturn.PCAresult[i]=0;
                    for(int c=0; c<NUMCARAC;c++)
                    {maxReturn.pointsReturn[i][c]=0;
                     pointsNorm[i][c]=0;}
                    maxReturn.pointsReturn[i][NUMCARAC]=0;
                }
                int numMax=0;

                //t0= getTickCount();
                maxDetector(zMat, &maxReturn, &numMax, cont_frames,&algoritmo.busquedaSR,&algoritmo.detectormaximos,&algoritmo.Extraccioncaracteristicas);
                /*t1 = getTickCount();
                secs = (t1-t0)/getTickFrequency();
                saveTimes(secs, '1',cont_frames);*/

                maxReturn.numMax = numMax;
                algoritmo.numeromaximos=numMax;
                for (int i = 0; i< numMax; i++)
                   {
                    cout << endl;
                    cout << "Máximo " << i << " = " << " Altura = " << maxReturn.pointsReturn[i][6] << " -> ";
                    for (int j=0; j< NUMCARAC; j++ )
                        cout << maxReturn.pointsReturn[i][j] << " " ;
                    }
                    cout << endl;
                    normalizationVector(maxReturn.pointsReturn, pointsNorm, numMax);


                    for (int i = 0; i< numMax; i++)
                       { cout << endl;
                        cout << "Máximo " << i << " Normalizado " << " -> ";
                        for (int j=0; j< NUMCARAC; j++ )
                            cout << pointsNorm[i][j] << " " ;
                         }
                    int hist_w = 500; int hist_h = 500;
                      Mat histImage( hist_h, hist_w, CV_8UC3,Scalar( 0,0,0) );
           /*         line (histImage, Point(0,50), Point(1000,50),Scalar(150,150,150),1,8,0);
                      line (histImage, Point(0,100), Point(1000,100),Scalar(150,150,150),1,8,0);
                      line (histImage, Point(0,150), Point(1000,150),Scalar(150,150,150),1,8,0);
                      line (histImage, Point(0,200), Point(1000,200),Scalar(150,150,150),1,8,0);
                      line (histImage, Point(0,250), Point(1000,250),Scalar(150,150,150),1,8,0);
                      line (histImage, Point(0,300), Point(1000,300),Scalar(150,150,150),1,8,0);
                      line (histImage, Point(0,350), Point(1000,350),Scalar(150,150,150),1,8,0);
                      line (histImage, Point(0,400), Point(1000,400),Scalar(150,150,150),1,8,0);
                      line (histImage, Point(0,450), Point(1000,450),Scalar(150,150,150),1,8,0);
                      */
                      //imwrite ("histimageBlanco.png", histImage);





                      histogramasec( pointsNorm[0], &histImage,(255,0,255));
                      Mat imageout;
                      cvtColor(zMat_SHOW,imageout,CV_GRAY2RGB);
                      algoritmo.numeropersonas=0;
                     if(SVM==1)
                     {
                         if(READWRITE==1)
                         {
                             for (int i = 0; i< numMax; i++)
                                {
                         if(alpha_slider==1)

                         {
                             writeVectorFile(pointsNorm[i], NUMCARAC);

                         }
                             }
                         }
                         if(READWRITE==0)
                         {
                         for (int i = 0; i< numMax; i++)
                            {
                         char *FileName="sombrero2.data";
                         char *FileName2="persona2.data";
                         char *FileName3="objetos.data";

                       sombrero=readVectorFile(structRead,FileName);
                       persona=readVectorFile(structRead,FileName2);
                       objeto=readVectorFile(structRead,FileName3);
                        t0= getTickCount();
                        printf("sombrero %d persona %d objeto %d \n", sombrero.numVectors,persona.numVectors,objeto.numVectors);
                             //clasificasdos(persona.vectorList,persona.numVectors,objeto.vectorList,objeto.numVectors,pointsNorm[i],zMat_SHOW,maxReturn.position.at(i));
                     clasificasvm(persona.vectorList,persona.numVectors,objeto.vectorList,objeto.numVectors,sombrero.vectorList,sombrero.numVectors,pointsNorm[i],&imageout,maxReturn.position.at(i),maxReturn.subroipos.at(i),maxReturn.PCAresult,i);
                     t1 = getTickCount();
                     secs = (t1-t0)/getTickFrequency();
                     algoritmo.Clasificacion=secs;
                     saveTimes(secs, '2',cont_frames);
if (maxReturn.PCAresult[i]==0)
{
    circle(resultadoo,Point((maxReturn.position.at(i).x*20+maxReturn.subroipos.at(i).x),(maxReturn.position.at(i).y*20+maxReturn.subroipos.at(i).y)),2,Scalar(0),-1,8,0);
    algoritmo.numeropersonas++;
}

                         }
                         if(VISUALIZACION==1)
                         {
                         imshow("resultadofinal",imageout);
                         char video[50];
                         sprintf (video, "video/video%d.png",cont_frames);
                         imwrite( video, imageout );
                         imshow("resultadofinal2",resultadoo);
                            waitKey(1);
                         }
                            saveResultsCounting(resultPath.c_str(), maxReturn, cont_frames);
                         }
                     }
                     if(PCA==1)
                     {
                      float vectorHistograma[6];
                      Mat histClass;
                      ///////////////// LECTURA O ESCRITURA DE LOS VECTORES DE ENTRENAMIENTO
                      Mat_<float>samplesClass1, samplesClass2, samplesClass3, samplesClass4, samplesClass5;
                      for (int i =0;i <numMax;i++)
                      {
                          if(VISUALIZACION==1)
                          {
                          imshow("vector",histImage);
                          waitKey(100);
                          }


                          if (maxReturn.position[i].x > 4 && maxReturn.position[i].x < 20 && maxReturn.position[i].y > 4 && maxReturn.position[i].y < 16)
                                      //pointsHeighRelation(maxReturn.pointsReturn[i][0], maxReturn.alturas.at(i),1);
                          { if (READWRITE && maxReturn.position[i].x > 4 && maxReturn.position[i].x < 21 && maxReturn.position[i].y > 4 && maxReturn.position[i].y < 17)
                            {

                              if (pointsNorm[i][0] < 1.6)


                                  writeVectorFile(pointsNorm[i], NUMCARAC);
                              cout << "[INFO]GUARDADO"<< endl;
                             }
                           }
                      }
                      if (!READWRITE)
                       {

                         readFiles structShortHair, structShortTall, structLongHair, structHat, structCap;

                         structShortHair = readVectorFile(structShortHair, "pelocorto.data");
                         samplesClass1 = Mat_<float>(structShortHair.numVectors,NUMCARAC);
                         //cout << "numero de elementos clase corto = " <<structShortHair.numVectors <<endl;
                         for(int s1=0; s1<structShortHair.numVectors; s1++)
                            {
                              for(int s2=0; s2<NUMCARAC; s2++)
                              {
                                  samplesClass1.at<float>(s1,s2) = (float)structShortHair.vectorList[s1][s2];
                               vectorHistograma[s2] = (float)structShortHair.vectorList[s1][s2];}
                               //REPRESENTACIÓN DE LOS VECTORES DE LA CLASE
                              /*histClass = imread("../raquelTOFAlgs/Graficas/histogramaClaseCorto(2grado).png",CV_LOAD_IMAGE_COLOR);
                              histogramasec6cmClases(vectorHistograma,&histClass);
                              imwrite("../raquelTOFAlgs/Graficas/histogramaClaseCorto(2grado).png", histClass);
                              waitKey();*/

}

                      structHat = readVectorFile(structHat, "sombreropca.data");
                      samplesClass4 = Mat_<float>(structHat.numVectors,NUMCARAC);
                      //cout << "numero de elementos clase largo = " <<structHat.numVectors <<endl;
                      for(int s1=0; s1<structHat.numVectors; s1++)
                         {
                           for(int s2=0; s2<NUMCARAC; s2++)
                           {
                               samplesClass4.at<float>(s1,s2) = (float)structHat.vectorList[s1][s2];
                               vectorHistograma[s2] = (float)structHat.vectorList[s1][s2];}

                           //REPRESENTACIÓN DE LOS VECTORES DE LA CLASE
                          /*histClass = imread("../raquelTOFAlgs/Graficas/histogramaClaseSombrero(2grado).png",CV_LOAD_IMAGE_COLOR);
                          histogramasec6cmClases(vectorHistograma,&histClass);
                          imwrite("../raquelTOFAlgs/Graficas/histogramaClaseSombrero(2grado).png", histClass);
                          waitKey();*/
                      }
                      }
                      ////////////////////////////////////////////////////REPRESENTACIÓN DE LOS VECTORES DE LAS CLASES



                      /////////////////PROYECCIONES PCA CON LAS DIFERENTES CLASES
                       Mat deteccion = zMat.clone();
                       minMaxLoc(deteccion, &minValv, &maxValv);
                       deteccion.convertTo(deteccion, CV_8U, 255.0/(maxValv - minValv), -minValv * 255.0/(maxValv - minValv));
                      cvtColor(deteccion, deteccion, CV_GRAY2RGB);
                      if (!READWRITE && numMax > 0)
                      {
                          Mat scene = Mat::zeros(400,500,CV_32FC1);
                          cvtColor(scene, scene, CV_GRAY2RGB );
                          Mat covClass1, muClass1, covClass2, muClass2,covClass3, muClass3, covClass4, muClass4, muClass5, covClass5, muClass6, covClass6;
                          calcCovarMatrix(samplesClass1, covClass1, muClass1, CV_COVAR_NORMAL | CV_COVAR_ROWS);
                         // calcCovarMatrix(samplesClass2, covClass2, muClass2, CV_COVAR_NORMAL | CV_COVAR_ROWS);
                          //calcCovarMatrix(samplesClass3, covClass3, muClass3, CV_COVAR_NORMAL | CV_COVAR_ROWS);
                          calcCovarMatrix(samplesClass4, covClass4, muClass4, CV_COVAR_NORMAL | CV_COVAR_ROWS);
                          //calcCovarMatrix(samplesClass5, covClass5, muClass5, CV_COVAR_NORMAL | CV_COVAR_ROWS);

                          Mat pointsNorm_Mat =  Mat::zeros(NUMCARAC, 1, CV_32FC1);
                          for (int n =0;n <numMax;n++)
                            {
                              for (int i=0;i<NUMCARAC;i++)
                               {
                                  pointsNorm_Mat.at<float>(i,0) = pointsNorm[n][i];}

                              //scene = imread("pointsScene.png",CV_LOAD_IMAGE_COLOR);

                              cout<< "[ MAXIMO " << n << "]" << endl;
                                 /*         cout << pointsNorm[n][0] << endl;
                                          cout << pointsNorm[n][1] << endl;
                                          cout << pointsNorm[n][2] << endl;
                                          cout << pointsNorm[n][3] << endl;
                                          cout << pointsNorm[n][4] << endl;
                                          cout << pointsNorm[n][5] << endl;
                              namedWindow("histograma", CV_WINDOW_AUTOSIZE );

                              //histograma6sec( pointsNorm[n], &histImage);

                              */

                               cout << "vector = " << pointsNorm_Mat << endl;
                               t0= getTickCount();
                               double PCA1,PCA2,PCA3,PCA4,PCA5, resPCA;
                               PCA1 = PCAclasification(samplesClass1, pointsNorm_Mat);//NUMCARAC);
                            //   PCA2 = PCAclasification(samplesClass2, pointsNorm_Mat);//, NUMCARAC);
                              // PCA3 = PCAclasification(samplesClass3, pointsNorm_Mat);
                               PCA4 = PCAclasification(samplesClass4, pointsNorm_Mat);
                               //PCA5 = PCAclasification(samplesClass5, pointsNorm_Mat);
                               resPCA = minValue (PCA1, PCA4);
                               /*resPCA = minValue (resPCA,PCA3);
                               resPCA = minValue (resPCA,PCA4);
                               resPCA = minValue (resPCA,PCA5);*/
                               //resPCA=PCA1;
                               t1 = getTickCount();
                               secs = (t1-t0)/getTickFrequency();
                               algoritmo.Clasificacion=secs;
                               saveTimes(secs, '2',cont_frames);
                               //cout << "resultados PCA = " << PCA1 << " - " << PCA2 << endl;
                               cout << "resultados PCA = " << resPCA <<  endl;
                               if (resPCA < 0.5)
                                  { circle( scene,Point(maxReturn.position.at(n).x*20,maxReturn.position.at(n).y*20),5,Scalar( 0, 255, 0 ),-1,8 );
                                    circle(deteccion, Point(maxReturn.position.at(n).x*20+4,maxReturn.position.at(n).y*20+12),5,Scalar( 255, 0, 0 ),-1,8 );
                                    maxReturn.PCAresult[n] = 0;
                               algoritmo.numeropersonas++;}
                               else
                                    {circle( scene,Point(maxReturn.position.at(n).x*20-10,maxReturn.position.at(n).y*20-5),5,Scalar( 0, 0, 255 ),-1,8 );
                                     maxReturn.PCAresult[n] = 1;}
                               /*if (resPCA < 0.26)
                                  { circle( scene,Point(maxReturn.position.at(n).x*20,maxReturn.position.at(n).y*20),5,Scalar( 0, 255, 0 ),-1,8 );
                                    circle(deteccion, Point(maxReturn.position.at(n).x*20+4,maxReturn.position.at(n).y*20+12),5,Scalar( 255, 0, 0 ),-1,8 );
                                    maxReturn.PCAresult[n] = 0;}
                               else
                                    {circle( scene,Point(maxReturn.position.at(n).x*20-10,maxReturn.position.at(n).y*20-5),5,Scalar( 0, 0, 255 ),-1,8 );
                                     maxReturn.PCAresult[n] = 1;}*/
                             }

                      }
                      if(VISUALIZACION==1)
                      {
                       imshow("deteccion", deteccion);
                       waitKey(1);
                      }
                      // if(SAVE_RESULT)
                         saveResultsCounting(resultPath.c_str(), maxReturn, cont_frames);




}
//svm();
                waitKey(1);
                algoritmo.frame=cont_frames;
                guardatemporizacion(algoritmo);
    }
    return 0;
}

int TOFdetector(int16_t *z2D16, int cont_frames, char *sequenceName)
{

    ///Extraer nombre de la secuencia para Resultados
    string resultPath(sequenceName);
    int position = resultPath.find('.');
    resultPath  =resultPath.substr(0,position);
    resultPath = resultPath + "_Counter.result";

    int size_array = 217088;

        cout << "[NUMERO DEL FRAME = "<< cont_frames << " ]" << endl;
        if  (cont_frames >= 0)
        {
                int16_t* ZsinCeros = new int16_t[size_array];
                Mat zMat(424,512,CV_16UC1,z2D16);

                if (SHOW_FRAME)
                {
                    Mat zMatInit_SHOW = zMat.clone();
                    minMaxLoc(zMatInit_SHOW, &minValv, &maxValv);
                    zMatInit_SHOW.convertTo(zMatInit_SHOW, CV_8U, 255.0/(maxValv - minValv), -minValv * 255.0/(maxValv - minValv));
                    imshow("imagen incial", zMatInit_SHOW);
                }

                ///===========================================================FILTRADO DE LA IMAGEN
                int64 t0= getTickCount();
                pixelesErroneosSuelo(zMat,&zMat);
                ruido_uint16(zMat, &zMat);
                medianBlur(zMat,zMat,3);
                int64 t1 = getTickCount();
                double secs = (t1-t0)/getTickFrequency();
                saveTimes(secs, '0',cont_frames);


                ///===========================================================DETECTOR DE MÁXIMOS POR SOMBRERO MEXICANO
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
                    minMaxLoc(zMat_SHOW, &minValv, &maxValv);
                    zMat_SHOW.convertTo(zMat_SHOW, CV_8U, 255.0/(maxValv - minValv), -minValv * 255.0/(maxValv - minValv));
                    imshow("imagen sin filtro", zMat_SHOW);
                }




                ///===========================================================DETECTOR DE MÁXIMOS
                ///Inicialización estructura de máximos
                maxData maxReturn;

                maxReturn.pointsReturn = new float *[maxDETECTIONS];
                double **pointsNorm;
                pointsNorm = new double*[maxDETECTIONS];
                for (int i=0;i<maxDETECTIONS;i++)
                   { maxReturn.pointsReturn[i]=new float[NUMCARAC+1];
                    pointsNorm[i]= new double[NUMCARAC];}
                for(int i=0;i<maxDETECTIONS;i++)
                {maxReturn.PCAresult[i]=0;
                    for(int c=0; c<NUMCARAC;c++)
                    {maxReturn.pointsReturn[i][c]=0;
                     pointsNorm[i][c]=0;}
                    maxReturn.pointsReturn[i][NUMCARAC]=0;
                }





                int numMax=0;

                t0= getTickCount();
                //maxDetector(zMat, &maxReturn, &numMax, cont_frames);
                t1 = getTickCount();
                secs = (t1-t0)/getTickFrequency();
                saveTimes(secs, '1',cont_frames);

                maxReturn.numMax = numMax;

             if(SHOW_VECTOR)
              {
                for (int i = 0; i< numMax; i++)
                   {
                    cout << endl;
                    cout << "Máximo " << i << " = " << " Altura = " << maxReturn.pointsReturn[i][6] << " -> ";
                    for (int j=0; j< NUMCARAC; j++ )
                        cout << maxReturn.pointsReturn[i][j] << " " ;
                    }
                    cout << endl;
             }

                normalizationVector(maxReturn.pointsReturn, pointsNorm, numMax);

              if(SHOW_VECTOR)
              {
                for (int i = 0; i< numMax; i++)
                   { cout << endl;
                    cout << "Máximo " << i << " Normalizado " << " -> ";
                    for (int j=0; j< NUMCARAC; j++ )
                        cout << pointsNorm[i][j] << " " ;
                     }
              }


              int hist_w = 1000; int hist_h = 500;
                Mat histImage( hist_h, hist_w, CV_8UC3,Scalar( 0,0,0) );
     /*         line (histImage, Point(0,50), Point(1000,50),Scalar(150,150,150),1,8,0);
                line (histImage, Point(0,100), Point(1000,100),Scalar(150,150,150),1,8,0);
                line (histImage, Point(0,150), Point(1000,150),Scalar(150,150,150),1,8,0);
                line (histImage, Point(0,200), Point(1000,200),Scalar(150,150,150),1,8,0);
                line (histImage, Point(0,250), Point(1000,250),Scalar(150,150,150),1,8,0);
                line (histImage, Point(0,300), Point(1000,300),Scalar(150,150,150),1,8,0);
                line (histImage, Point(0,350), Point(1000,350),Scalar(150,150,150),1,8,0);
                line (histImage, Point(0,400), Point(1000,400),Scalar(150,150,150),1,8,0);
                line (histImage, Point(0,450), Point(1000,450),Scalar(150,150,150),1,8,0);
                */
                //imwrite ("histimageBlanco.png", histImage);



                //histogramasec( maxReturn.pointsReturn[0], &histImage);
                //waitKey();
                float vectorHistograma[6];
                Mat histClass;



                ///////////////// LECTURA O ESCRITURA DE LOS VECTORES DE ENTRENAMIENTO
                Mat_<float>samplesClass1, samplesClass2, samplesClass3, samplesClass4, samplesClass5;
                for (int i =0;i <numMax;i++)
                {
                    if (maxReturn.position[i].x > 4 && maxReturn.position[i].x < 20 && maxReturn.position[i].y > 4 && maxReturn.position[i].y < 16)
                                //pointsHeighRelation(maxReturn.pointsReturn[i][0], maxReturn.alturas.at(i),1);
                    { if (READWRITE && maxReturn.position[i].x > 4 && maxReturn.position[i].x < 21 && maxReturn.position[i].y > 4 && maxReturn.position[i].y < 17)
                      {
                        if (pointsNorm[i][0] < 1.4)
                            writeVectorFile(pointsNorm[i], NUMCARAC);
                        cout << "[INFO]GUARDADO"<< endl;
                       }
                     }
                }

                 if (!READWRITE)
                  {

                    readFiles structShortHair, structShortTall, structLongHair, structHat, structCap;

                    structShortHair = readVectorFile(structShortHair, "../raquelTOFAlgs/ClasesPCA/peloCorto(2grado).data");
                    samplesClass1 = Mat_<float>(structShortHair.numVectors,NUMCARAC);
                    //cout << "numero de elementos clase corto = " <<structShortHair.numVectors <<endl;
                    for(int s1=0; s1<structShortHair.numVectors; s1++)
                       {
                         for(int s2=0; s2<NUMCARAC; s2++)
                         {
                             samplesClass1.at<float>(s1,s2) = (float)structShortHair.vectorList[s1][s2];
                          vectorHistograma[s2] = (float)structShortHair.vectorList[s1][s2];}
                          //REPRESENTACIÓN DE LOS VECTORES DE LA CLASE
                         /*histClass = imread("../raquelTOFAlgs/Graficas/histogramaClaseCorto(2grado).png",CV_LOAD_IMAGE_COLOR);
                         histogramasec6cmClases(vectorHistograma,&histClass);
                         imwrite("../raquelTOFAlgs/Graficas/histogramaClaseCorto(2grado).png", histClass);
                         waitKey();*/


                       }


                    structShortTall = readVectorFile(structShortTall, "../raquelTOFAlgs/ClasesPCA/peloCortoAlto(2grado).data");
                    samplesClass2 = Mat_<float>(structShortTall.numVectors,NUMCARAC);
                    //cout << "numero de elementos clase corto alto = " <<structShortTall.numVectors <<endl;
                    for(int s1=0; s1<structShortTall.numVectors; s1++)
                       {
                         for(int s2=0; s2<NUMCARAC; s2++)
                         {
                             samplesClass2.at<float>(s1,s2) = (float)structShortTall.vectorList[s1][s2];
                             vectorHistograma[s2] = (float)structShortTall.vectorList[s1][s2];}

                         //REPRESENTACIÓN DE LOS VECTORES DE LA CLASE
                       /* histClass = imread("../raquelTOFAlgs/Graficas/histogramaClaseCortoAlto(2grado).png",CV_LOAD_IMAGE_COLOR);
                        histogramasec6cmClases(vectorHistograma,&histClass);
                        imwrite("../raquelTOFAlgs/Graficas/histogramaClaseCortoAlto(2grado).png", histClass);
                        waitKey();*/
                    }

                    structLongHair = readVectorFile(structLongHair, "../raquelTOFAlgs/ClasesPCA/peloLargo(2grado).data");
                    samplesClass3 = Mat_<float>(structLongHair.numVectors,NUMCARAC);
                    //cout << "numero de elementos clase largo = " <<structLongHair.numVectors <<endl;
                    for(int s1=0; s1<structLongHair.numVectors; s1++)
                       {
                         for(int s2=0; s2<NUMCARAC; s2++)
                         {
                             samplesClass3.at<float>(s1,s2) = (float)structLongHair.vectorList[s1][s2];
                             vectorHistograma[s2] = (float)structLongHair.vectorList[s1][s2];}

                         //REPRESENTACIÓN DE LOS VECTORES DE LA CLASE
                       /*histClass = imread("../raquelTOFAlgs/Graficas/histogramaClaseLargo(2grado).png",CV_LOAD_IMAGE_COLOR);
                        histogramasec6cmClases(vectorHistograma,&histClass);
                        imwrite("../raquelTOFAlgs/Graficas/histogramaClaseLargo(2grado).png", histClass);
                        waitKey();*/
                    }

                    structHat = readVectorFile(structHat, "../raquelTOFAlgs/ClasesPCA/sombrero(2grado).data");
                    samplesClass4 = Mat_<float>(structHat.numVectors,NUMCARAC);
                    //cout << "numero de elementos clase largo = " <<structHat.numVectors <<endl;
                    for(int s1=0; s1<structHat.numVectors; s1++)
                       {
                         for(int s2=0; s2<NUMCARAC; s2++)
                         {
                             samplesClass4.at<float>(s1,s2) = (float)structHat.vectorList[s1][s2];
                             vectorHistograma[s2] = (float)structHat.vectorList[s1][s2];}

                         //REPRESENTACIÓN DE LOS VECTORES DE LA CLASE
                        /*histClass = imread("../raquelTOFAlgs/Graficas/histogramaClaseSombrero(2grado).png",CV_LOAD_IMAGE_COLOR);
                        histogramasec6cmClases(vectorHistograma,&histClass);
                        imwrite("../raquelTOFAlgs/Graficas/histogramaClaseSombrero(2grado).png", histClass);
                        waitKey();*/
                    }

                    structCap = readVectorFile(structCap, "../raquelTOFAlgs/ClasesPCA/gorra(2grado).data");
                    samplesClass5 = Mat_<float>(structCap.numVectors,NUMCARAC);
                    //cout << "numero de elementos clase largo = " <<structCap.numVectors <<endl;
                    for(int s1=0; s1<structCap.numVectors; s1++)
                       {
                         for(int s2=0; s2<NUMCARAC; s2++)
                         {
                             samplesClass5.at<float>(s1,s2) = (float)structCap.vectorList[s1][s2];
                             vectorHistograma[s2] = (float)structCap.vectorList[s1][s2];}

                         //REPRESENTACIÓN DE LOS VECTORES DE LA CLASE
                        /*histClass = imread("../raquelTOFAlgs/Graficas/histogramaClaseGorra(grado2).png",CV_LOAD_IMAGE_COLOR);
                        histogramasec6cmClases(vectorHistograma,&histClass);
                        imwrite("../raquelTOFAlgs/Graficas/histogramaClaseGorra(grado2).png", histClass);
                        waitKey();*/
                    }

                   }


                ////////////////////////////////////////////////////REPRESENTACIÓN DE LOS VECTORES DE LAS CLASES



                /////////////////PROYECCIONES PCA CON LAS DIFERENTES CLASES
                 Mat deteccion = zMat.clone();
                 minMaxLoc(deteccion, &minValv, &maxValv);
                 deteccion.convertTo(deteccion, CV_8U, 255.0/(maxValv - minValv), -minValv * 255.0/(maxValv - minValv));
                cvtColor(deteccion, deteccion, CV_GRAY2RGB);


                if (!READWRITE && numMax > 0)
                {
                    Mat scene = Mat::zeros(400,500,CV_32FC1);
                    cvtColor(scene, scene, CV_GRAY2RGB );
                    Mat covClass1, muClass1, covClass2, muClass2,covClass3, muClass3, covClass4, muClass4, muClass5, covClass5, muClass6, covClass6;
                    calcCovarMatrix(samplesClass1, covClass1, muClass1, CV_COVAR_NORMAL | CV_COVAR_ROWS);
                    calcCovarMatrix(samplesClass2, covClass2, muClass2, CV_COVAR_NORMAL | CV_COVAR_ROWS);
                    calcCovarMatrix(samplesClass3, covClass3, muClass3, CV_COVAR_NORMAL | CV_COVAR_ROWS);
                    calcCovarMatrix(samplesClass4, covClass4, muClass4, CV_COVAR_NORMAL | CV_COVAR_ROWS);
                    calcCovarMatrix(samplesClass5, covClass5, muClass5, CV_COVAR_NORMAL | CV_COVAR_ROWS);

                    Mat pointsNorm_Mat =  Mat::zeros(NUMCARAC, 1, CV_32FC1);
                    for (int n =0;n <numMax;n++)
                      {
                        for (int i=0;i<NUMCARAC;i++)
                         {
                            pointsNorm_Mat.at<float>(i,0) = pointsNorm[n][i];}

                        //scene = imread("pointsScene.png",CV_LOAD_IMAGE_COLOR);

                        cout<< "[ MAXIMO " << n << "]" << endl;
                           /*         cout << pointsNorm[n][0] << endl;
                                    cout << pointsNorm[n][1] << endl;
                                    cout << pointsNorm[n][2] << endl;
                                    cout << pointsNorm[n][3] << endl;
                                    cout << pointsNorm[n][4] << endl;
                                    cout << pointsNorm[n][5] << endl;
                        namedWindow("histograma", CV_WINDOW_AUTOSIZE );

                        //histograma6sec( pointsNorm[n], &histImage);

                        */

                         cout << "vector = " << pointsNorm_Mat << endl;
                         t0= getTickCount();
                         double PCA1,PCA2,PCA3,PCA4,PCA5, resPCA;
                         PCA1 = PCAclasification(samplesClass1, pointsNorm_Mat);//NUMCARAC);
                         PCA2 = PCAclasification(samplesClass2, pointsNorm_Mat);//, NUMCARAC);
                         PCA3 = PCAclasification(samplesClass3, pointsNorm_Mat);
                         PCA4 = PCAclasification(samplesClass4, pointsNorm_Mat);
                         PCA5 = PCAclasification(samplesClass5, pointsNorm_Mat);
                         resPCA = minValue (PCA1, PCA2);
                         resPCA = minValue (resPCA,PCA3);
                         resPCA = minValue (resPCA,PCA4);
                         resPCA = minValue (resPCA,PCA5);
                         t1 = getTickCount();
                         secs = (t1-t0)/getTickFrequency();
                         saveTimes(secs, '2',cont_frames);
                         //cout << "resultados PCA = " << PCA1 << " - " << PCA2 << endl;
                         cout << "resultados PCA = " << resPCA <<  endl;


                         if (resPCA < 0.26)
                            { circle( scene,Point(maxReturn.position.at(n).x*20,maxReturn.position.at(n).y*20),5,Scalar( 0, 255, 0 ),-1,8 );
                              circle(deteccion, Point(maxReturn.position.at(n).x*20+4,maxReturn.position.at(n).y*20+12),5,Scalar( 255, 0, 0 ),-1,8 );
                              maxReturn.PCAresult[n] = 1;}
                         else
                              {circle( scene,Point(maxReturn.position.at(n).x*20-10,maxReturn.position.at(n).y*20-5),5,Scalar( 0, 0, 255 ),-1,8 );
                               maxReturn.PCAresult[n] = 0;}


                       }

                }


               /*if (SHOW_RESULT)
               {
                imshow("deteccion", deteccion);
                waitKey();
               }*/
               // if(SAVE_RESULT)
                 // saveResultsCounting(resultPath.c_str(), &maxReturn, cont_frames);

            }



    return 1;
}

