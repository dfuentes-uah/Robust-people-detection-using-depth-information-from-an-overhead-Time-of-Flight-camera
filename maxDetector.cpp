#include <stdio.h>


//#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <iostream>
#include "direccion.h"
#include "tiempo.h"
#include "secciones.h"
#include "defines.h"
#include "histogramasec.h"

<<<<<<< maxDetector.cpp
#define VISUALIZACION 1
=======
#define VISUALIZACION 0
>>>>>>> 1.5
#define sectionsPRINT 0
#define SHOW_MAXIMOS 1
#define maxSUELO 3400

using namespace cv;
using namespace std;


double minValue, maxValue;

typedef struct
{
    Point loc;
    int numSubROI;

}maxCandidateSt;

typedef struct
{
    float **pointsReturn;
    vector <Point> position;
    vector < Point > subroipos;
    vector <int> alturas;
 int PCAresult[maxDETECTIONS];
    int numMax;
}maxData;

void maxDetector(Mat& image16, maxData *maxReturn, int *numMax,int cont_frames, double *busquedaroi, double *detectormaximos,double  *extractcaract)
{


     uchar valormin[1000];
    int coordx[1000],coordy[1000],n=0;
     int maximx=0,maximy=0;

     Mat Z = Mat::zeros(21,25, CV_8UC1);
     Mat Z16 = Mat::zeros(21,25, CV_16UC1);
     //cv::Mat image = cv::imread("suavizado5.png", CV_LOAD_IMAGE_GRAYSCALE);


     Mat image = image16.clone();
     minMaxLoc(image, &minValue, &maxValue);
     image.convertTo(image, CV_8U, 255.0/(maxValue - minValue), -minValue * 255.0/(maxValue - minValue));
     if(VISUALIZACION==1)
     {
imshow("imagen que llega ", image);
     }


     Mat image2;
     cvtColor(image, image2, CV_GRAY2RGB );
 //imshow("Image en color", image2);


   

     ///===========================================================CREACIÓN DEL ARRAY DE SUBROIS

     cv :: Size smallSize ( 20 , 20);

     minMaxLoc(image16, &minValue, &maxValue);

     vector<Mat> smallImages16; //array de secciones (en metros)

     for  ( int y =  0 ; y+smallSize . height < image16 . rows ; y += smallSize . height )
     {

          for  ( int x =  0 ; x + smallSize . width< image16 . cols ; x += smallSize . width )
          {

               Rect rect = Rect ( x , y , smallSize.width ,smallSize.height );
               smallImages16.push_back(Mat(image16,rect));
               double minVal16; double maxVal16; Point minLoc16; Point maxLoc16;
               minMaxLoc( cv :: Mat ( image16 , rect ), &minVal16, &maxVal16, &minLoc16, &maxLoc16, Mat() );
               Z16.at<unsigned short>(Point(maximx, maximy))=cv :: Mat ( image16 , rect ).at<unsigned short>(minLoc16);



               maximx++;

               n++;

               }
          maximx=0;
          maximy++;
     }


     ///=============================================================LOCALIZACIÓN DE TODOS LOS MÁXIMOS DE LA IMAGEN

     vector<maxCandidateSt> maxCandidatesData;
     maxCandidateSt maxCadidate_aux;
     bool maxCandidate = true;

   int64 t0= getTickCount();

     for(int i=0;i<Z16.rows;i++)
     {
         for(int j=0;j<Z16.cols;j++)
         {
             int inix=i-1;
             int iniy=j-1;
             int finx=i+1;
             int finy=j+1;
             if (inix < 0)inix=0;
             if (iniy < 0)iniy=0;
             if (finx >= Z16.rows) finx = Z16.rows-1;
             if (finy >= Z16.cols) finy = Z16.cols-1;

            if(Z16.at<unsigned short>(i,j) <= maxSUELO-500) //if(Z16.data[Z16.cols*i +j] <= maxSUELO-100)
            {
                for(int a=inix;a<=finx;a++)
                 {
                     for(int b=iniy;b<=finy;b++)
                     {

                         if (Z16.at<unsigned short>(a,b) < Z16.at<unsigned short>(i,j) )//if (Z16.data[Z16.cols*a + b] < Z16.data[Z16.cols*i +j])
                         {

                             maxCandidate = false;

                         }

                     }
                 }
            }
            else
                maxCandidate=false;




             if(maxCandidate)
             {
                 if (maxCandidatesData.size() == 0)
                 {
                     maxCadidate_aux.loc = Point(i,j);
                     maxCadidate_aux.numSubROI = (Z16.cols*i +j);
                     maxCandidatesData.push_back(maxCadidate_aux);
                  }
                 else
                 {
                     float disMin_th = 20;
                     int positionNearest = 0;
                     for (int p=0;p<maxCandidatesData.size();p++)
                     {

                         float disMin = sqrt((maxCandidatesData.at(p).loc.x - i)*(maxCandidatesData.at(p).loc.x - i) + (maxCandidatesData.at(p).loc.y - j)*(maxCandidatesData.at(p).loc.y - j));
                         if (disMin < disMin_th)
                         { disMin_th= disMin;
                           positionNearest = p;}
                      }

                     if(disMin_th < 3 && Z16.at<unsigned short>(maxCandidatesData.at(positionNearest).loc) < Z16.at<unsigned short>(i,j))
                     {
                         maxCandidatesData.at(positionNearest).loc = Point(i,j);
                         maxCandidatesData.at(positionNearest).numSubROI = (Z16.cols*i +j);
                     }

                     if (disMin_th >= 3)
                     {

                             maxCadidate_aux.loc = Point(i,j);
                             maxCadidate_aux.numSubROI = (Z16.cols*i +j);
                             cout << maxCadidate_aux.numSubROI << " i " << i << " j" << j << endl;
                             maxCandidatesData.push_back(maxCadidate_aux);
                     }
                 }

             }
             maxCandidate = true;


         }
     }
int64 t1 = getTickCount();
     double secs = (t1-t0)/getTickFrequency();
     *detectormaximos=secs;
     saveTimes(secs, '1', cont_frames);

     ///========================================================================================================///


     Mat ZmatColor;
     Mat aux = image.clone();
     Point maxPoint;
     cvtColor(aux, ZmatColor, CV_GRAY2RGB);
     Mat imageReconstruct =  Mat::zeros(424,512, CV_8UC1);
     double minVal=0; double maxVal; Point minLoc; Point maxLoc;

      vector < Mat > smallImages_aux(smallImages16.size());


     for (int i=0;i<smallImages16.size();i++)
         smallImages_aux.at(i)= smallImages16.at(i).clone();

     cout <<"NUMERO DE MAXIMOS ENCONTRADOS = " << maxCandidatesData.size() << endl;

     for(int k=0;k<maxCandidatesData.size();k++)
     {
         
         maxPoint =  maxCandidatesData.at(k).loc;

       

          if(SHOW_MAXIMOS)
         {
             Mat imageReconstruct =  Mat::zeros(424,512, CV_16UC1);
             delecteSecciones(maxCandidatesData.at(k).numSubROI,maxCandidatesData.at(k).loc,smallImages_aux, maxVal);
             reconstruccionImage(smallImages_aux, imageReconstruct);
             minMaxLoc(imageReconstruct, &minValue, &maxValue);
             imageReconstruct.convertTo(imageReconstruct, CV_8U, 255.0/(maxValue - minValue), -minValue * 255.0/(maxValue - minValue));
             if(VISUALIZACION==1)
             {
             imshow("Maximos encontrados", imageReconstruct);
             }
         }


         minMaxLoc(smallImages16.at(maxCandidatesData.at(k).numSubROI),&minVal, &maxVal, &minLoc, &maxLoc);

         int incrY = (int)maxCandidatesData.at(k).numSubROI/25;
         int incrX = maxCandidatesData.at(k).numSubROI - (25*incrY);
         maxCandidatesData.at(k).loc.x=minLoc.x;
         maxCandidatesData.at(k).loc.y=minLoc.y;
         minLoc.x = incrX;
         minLoc.y = incrY;

         int alturaInvertida = minVal;


         Point Point_aux;
         Point_aux.x=incrX;
         Point_aux.y=incrY;



         maxReturn->position.push_back (Point_aux);
         maxReturn->subroipos.push_back(maxCandidatesData.at(k).loc);


  ///=============================================================BUSQUEDA Y ASIGNACIÓN DE ROIS A LA PERSONA
        t0= getTickCount();







     /////////////////////////////////////////////////////////////////////////////////////////////////////
     ////////////////////////////Funciones de busqueda y asignacion de rois a la persona////////////////
     ////////////////////////////////////////////////////////////////////////////////////////////////////



         if (minVal < 3000)
        {

        int contador1=0,contador2=0,contador3=0,contador4=0,contador5=0;


         int seccionesdir1[17];
         direction1(Z16,image2,&image2,minLoc,minVal,smallImages16,seccionesdir1,&contador1);

         int seccionesdir2[17];
         direction2(Z16,image2,&image2,minLoc,minVal,smallImages16,seccionesdir2,&contador2);

         int seccionesdir3[17];
         direction3(Z16,image2,&image2,minLoc,minVal,smallImages16,seccionesdir3,&contador3);

         int seccionesdir4[17];
         direction4(Z16,image2,&image2,minLoc,minVal,smallImages16,seccionesdir4,&contador4);

         int seccionesdiag[17];
         diagonal(Z16,image2,&image2,minLoc,minVal,smallImages16,seccionesdiag,&contador5);

         t1 = getTickCount();
         secs = (t1-t0)/getTickFrequency();
         *busquedaroi=secs;
         saveTimes(secs, '3',cont_frames);
         if(VISUALIZACION==1)
         {
        imshow("secciones1", image2);
        //waitKey();
         }

        cvtColor(image, image2, CV_GRAY2RGB );


///=============================================================ACUMULADOR DE PUNTOS PERTENECIENTES A LA PERSONA (VECTOR 5 COMPONENTES)
      //t0= getTickCount();

     int resultado2cm[20];
     vector<Point> extremosHead;
     vector<Point> puntosCabeza;
     extremosHead.push_back (Point(600,0));
     extremosHead.push_back(Point(0,0));
     extremosHead.push_back(Point(0,600));
     extremosHead.push_back(Point(0,0));
t0= getTickCount();
     for (int r=0; r<20; r++)resultado2cm[r]=0;


     //cout << "contadores = " << contador1 << " " << contador2 << " " << contador3 << " " << contador4 << " " << contador5 << endl;

     ///suma de los puntos, Nucleo y todas las direcciones
     seccion(smallImages16,minLoc,minVal,maxCandidatesData.at(k).numSubROI,resultado2cm, &extremosHead, &puntosCabeza);


          for(int i=0;i<contador1;i++)
     {
         seccion(smallImages16,minLoc,minVal,seccionesdir1[i],resultado2cm, &extremosHead, &puntosCabeza);
         //delecteSecciones(seccionesdir1[i],smallImages, maxVal);
     }
     for(int i=0;i<contador2;i++)
     {
         seccion(smallImages16,minLoc,minVal,seccionesdir2[i],resultado2cm, &extremosHead, &puntosCabeza);
         //delecteSecciones(seccionesdir2[i],smallImages, maxVal);
     }
     for(int i=0;i<contador3;i++)
     {
         seccion(smallImages16,minLoc,minVal,seccionesdir3[i],resultado2cm, &extremosHead, &puntosCabeza);
         //delecteSecciones(seccionesdir3[i],smallImages, maxVal);
     }
     for(int i=0;i<contador4;i++)
     {
         seccion(smallImages16,minLoc,minVal,seccionesdir4[i],resultado2cm, &extremosHead, &puntosCabeza);
         //delecteSecciones(seccionesdir4[i],smallImages, maxVal);
     }
     for(int i=0;i<contador5;i++)
     {
         seccion(smallImages16,minLoc,minVal,seccionesdiag[i],resultado2cm, &extremosHead, &puntosCabeza);
         //delecteSecciones(seccionesdir4[i],smallImages, maxVal);
     }


    Mat extremos;
    cvtColor(aux, extremos, CV_GRAY2RGB);


    /* //Visual para ver los máximos y mínimos de la cabeza (puntos extremos)
     for (int pp=0;pp<4;pp++)
     { circle( extremos, extremosHead.at(pp), 2, Scalar(0,0,0), -1, 8, 0 );}
     line (extremos, extremosHead.at(0), extremosHead.at(1),Scalar(0,0,0),1, 8, 0);
     line (extremos, extremosHead.at(2), extremosHead.at(3),Scalar(0,0,0),1, 8, 0);
     */

    int resultado6cm[5];
     for (int r=0; r<5; r++)resultado6cm[r]=0;
     seccionAgrup (resultado2cm, resultado6cm);
     t1 = getTickCount();
     secs = (t1-t0)/getTickFrequency();
     *extractcaract=secs;
     saveTimes(secs, '4',cont_frames);

    ///=============================================================CALCULO COMPONENTE 6 DE V

     Mat auxAxisMat = extremos.clone();
     float relationAxis = headAxis (extremosHead, puntosCabeza,auxAxisMat);
     //cout << "relationAxis = " << relationAxis << endl;
     //imshow("puntos cabeza", image2);
     //waitKey();



     ///==================================================================================

     if (sectionsPRINT)
      {
         cout << "[ ESCALAS MÁXIMO NÚMERO = " <<k << " ]" << endl;
         cout << "[ ESCALAS SECCIONES DE 2CM ]  "<< endl;
         cout << resultado2cm[0] << endl;
         cout << resultado2cm[1] << endl;
         cout << resultado2cm[2] << endl;
         cout << resultado2cm[3] << endl;
         cout << resultado2cm[4] << endl;
         cout << resultado2cm[5] << endl;
         cout << resultado2cm[6] << endl;
         cout << resultado2cm[7] << endl;
         cout << resultado2cm[8] << endl;
         cout << resultado2cm[9] << endl;
         cout << resultado2cm[10] << endl;
         cout << resultado2cm[11] << endl;
         cout << resultado2cm[12] << endl;
         cout << resultado2cm[13] << endl;
         cout << resultado2cm[14] << endl;
         cout << resultado2cm[15] << endl;
         cout << resultado2cm[16] << endl;
         cout << resultado2cm[17] << endl;
         cout << resultado2cm[18] << endl;
         cout << resultado2cm[19] << endl;
     }

/*
     int hist_w = 1000; int hist_h = 500;
     Mat histImage( hist_h, hist_w, CV_8UC3,Scalar( 0,0,0) );
     histImage = imread("histimage2sec.png",CV_LOAD_IMAGE_COLOR);
     histogramasec2cm( resultado2cm, &histImage);
     imwrite ("histimage2sec.png", histImage);
*/




      if (sectionsPRINT)
      {
         cout << "[ ESCALAS SECCIONES DE 6CM ]  "<< endl;
         cout << resultado6cm[0] << endl;
         cout << resultado6cm[1] << endl;
         cout << resultado6cm[2] << endl;
         cout << resultado6cm[3] << endl;
         cout << resultado6cm[4] << endl;
         //cout << "altura-> " << alturaInvertida << endl;
       }






     for(int i=0; i<5;i++)
        maxReturn->pointsReturn[k][i]=(float)resultado6cm[i];
     maxReturn->pointsReturn[k][5] = relationAxis;
     maxReturn->pointsReturn[k][6]=alturaInvertida; ///guarda la altura de la persona en la última posición
     maxReturn->alturas.push_back(alturaInvertida);

    }
     else
         {
             cout << "[ MÁXIMO " << k << " DEMASIADO PEQUEÑO ]" << endl;
         }
    }

     cout << "[INFORMACIÓN BUENA] HA ANALIZADO TODOS LOS MÁXIMOS " << endl;


    *numMax=maxCandidatesData.size();




}


