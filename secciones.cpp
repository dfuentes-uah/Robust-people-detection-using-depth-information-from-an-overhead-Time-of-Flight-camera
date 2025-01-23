#include <stdio.h>


//#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <iostream>

#include"Funciones_opencv.h"
#include"defines.h"


using namespace cv;
using namespace std;
#define VISUALIZACION 0

///////////////////////////////////////////////////////////////////7
///////////////////////////obtener secciones persona/////////////////////
//////////////////////////////////////////////////////////////7
int seccion(std :: vector < Mat > smallImages,cv::Point minLoc,double minVal,int posicion,int secciones[20], vector<Point> *extremosHead,  vector<Point> *puntosCabeza)
{
    int ptocabeza=(minVal);
    int scale2CM = 2;//scale1MM*20;
    int measures[20];

    for (int x=0;x<20;x++){
        measures[x] = ptocabeza + (x+1)*20;
        //cout << measures[x] << "  ";
    }

    int row = posicion/25;
    int col = posicion - row*25;


    Point minX, maxX, minY, maxY;
    minX = extremosHead->at(0);
    maxX = extremosHead->at(1);
    minY = extremosHead->at(2);
    maxY = extremosHead->at(3);
    int valX, valY;


    for(int y=0;y<=(smallImages.at(posicion)).rows;y++)
    {
        for(int x=0;x<=(smallImages.at(posicion)).cols;x++)
        {

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y)) <= measures[0])
            {
            secciones[0]++;
            valX = 20*col + x;
            valY = 20*row + y;
            if (valX < minX.x)minX = Point(valX,valY);
            if (valX > maxX.x)maxX = Point(valX,valY);
            if (valY < minY.y)minY = Point(valX,valY);
            if (valY > maxY.y)maxY = Point(valX,valY);
            puntosCabeza->push_back(Point(valX,valY));
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[1] && smallImages.at(posicion).at<unsigned short>(Point(x, y))> measures[0])
            {
            secciones[1]++;
            valX = 20*col + x;
            valY = 20*row + y;
            if (valX < minX.x)minX = Point(valX,valY);
            if (valX > maxX.x)maxX = Point(valX,valY);
            if (valY < minY.y)minY = Point(valX,valY);
            if (valY > maxY.y)maxY = Point(valX,valY);
            puntosCabeza->push_back(Point(valX,valY));
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[2] && smallImages.at(posicion).at<unsigned short>(Point(x, y))> measures[1])
            {
            secciones[2]++;
            valX = 20*col + x;
            valY = 20*row + y;
            if (valX < minX.x)minX = Point(valX,valY);
            if (valX > maxX.x)maxX = Point(valX,valY);
            if (valY < minY.y)minY = Point(valX,valY);
            if (valY > maxY.y)maxY = Point(valX,valY);
            puntosCabeza->push_back(Point(valX,valY));
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[3] && smallImages.at(posicion).at<unsigned short>(Point(x, y)) > measures[2])
            {
            secciones[3]++;
            valX = 20*col + x;
            valY = 20*row + y;
            if (valX < minX.x)minX = Point(valX,valY);
            if (valX > maxX.x)maxX = Point(valX,valY);
            if (valY < minY.y)minY = Point(valX,valY);
            if (valY > maxY.y)maxY = Point(valX,valY);
            puntosCabeza->push_back(Point(valX,valY));
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[4] && smallImages.at(posicion).at<unsigned short>(Point(x, y))> measures[3])
            {
            secciones[4]++;
            valX = 20*col + x;
            valY = 20*row + y;
            if (valX < minX.x)minX = Point(valX,valY);
            if (valX > maxX.x)maxX = Point(valX,valY);
            if (valY < minY.y)minY = Point(valX,valY);
            if (valY > maxY.y)maxY = Point(valX,valY);
            puntosCabeza->push_back(Point(valX,valY));
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[5] && smallImages.at(posicion).at<unsigned short>(Point(x, y))> measures[4])
            {
            secciones[5]++;
            valX = 20*col + x;
            valY = 20*row + y;
            if (valX < minX.x)minX = Point(valX,valY);
            if (valX > maxX.x)maxX = Point(valX,valY);
            if (valY < minY.y)minY = Point(valX,valY);
            if (valY > maxY.y)maxY = Point(valX,valY);
            puntosCabeza->push_back(Point(valX,valY));
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[6] && smallImages.at(posicion).at<unsigned short>(Point(x, y))> measures[5])
            {
            secciones[6]++;
            valX = 20*col + x;
            valY = 20*row + y;
            if (valX < minX.x)minX = Point(valX,valY);
            if (valX > maxX.x)maxX = Point(valX,valY);
            if (valY < minY.y)minY = Point(valX,valY);
            if (valY > maxY.y)maxY = Point(valX,valY);
            puntosCabeza->push_back(Point(valX,valY));
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[7] && smallImages.at(posicion).at<unsigned short>(Point(x, y)) > measures[6])
            {
            secciones[7]++;
            valX = 20*col + x;
            valY = 20*row + y;
            if (valX < minX.x)minX = Point(valX,valY);
            if (valX > maxX.x)maxX = Point(valX,valY);
            if (valY < minY.y)minY = Point(valX,valY);
            if (valY > maxY.y)maxY = Point(valX,valY);
            puntosCabeza->push_back(Point(valX,valY));
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[8] && smallImages.at(posicion).at<unsigned short>(Point(x, y))> measures[7])
            {
            secciones[8]++;
            valX = 20*col + x;
            valY = 20*row + y;
            if (valX < minX.x)minX = Point(valX,valY);
            if (valX > maxX.x)maxX = Point(valX,valY);
            if (valY < minY.y)minY = Point(valX,valY);
            if (valY > maxY.y)maxY = Point(valX,valY);
            puntosCabeza->push_back(Point(valX,valY));
            }
        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[9] && smallImages.at(posicion).at<unsigned short>(Point(x, y))> measures[8])
            {
            secciones[9]++;
            /*valX = 20*col + x;
            valY = 20*row + y;
            if (valX < minX.x)minX = Point(valX,valY);
            if (valX > maxX.x)maxX = Point(valX,valY);
            if (valY < minY.y)minY = Point(valX,valY);
            if (valY > maxY.y)maxY = Point(valX,valY);
            puntosCabeza->push_back(Point(valX,valY));*/
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[10] && smallImages.at(posicion).at<unsigned short>(Point(x, y))> measures[9])
            {
            secciones[10]++;
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[11] && smallImages.at(posicion).at<unsigned short>(Point(x, y)) > measures[10])
            {
            secciones[11]++;
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[12] && smallImages.at(posicion).at<unsigned short>(Point(x, y))> measures[11])
            {
            secciones[12]++;
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[13] && smallImages.at(posicion).at<unsigned short>(Point(x, y))> measures[12])
            {
            secciones[13]++;
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[14] && smallImages.at(posicion).at<unsigned short>(Point(x, y))> measures[13])
            {
            secciones[14]++;
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[15] && smallImages.at(posicion).at<unsigned short>(Point(x, y)) > measures[14])
            {
            secciones[15]++;
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[16] && smallImages.at(posicion).at<unsigned short>(Point(x, y))> measures[15])
            {
            secciones[16]++;
            }
        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[17] && smallImages.at(posicion).at<unsigned short>(Point(x, y))> measures[16])
            {
            secciones[17]++;
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[18] && smallImages.at(posicion).at<unsigned short>(Point(x, y))> measures[17])
            {
            secciones[18]++;
            }

        if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<= measures[19] && smallImages.at(posicion).at<unsigned short>(Point(x, y)) > measures[18])
            {
            secciones[19]++;
            }


        }
    }


    extremosHead->at(0) = (minX);
    extremosHead->at(1) = (maxX);
    extremosHead->at(2) = (minY);
    extremosHead->at(3) = (maxY);

}

/////////////////////////////////////////////////////////////////////////
///////////////////////////Agrupa secciones de 2cm en 6cm/////////////////////
/////////////////////////////////////////////////////////////////////////
 int seccionAgrup (int secciones2cm[20], int secciones6cm[5])
 {

     ///BUSQUEDA ENTRE LAS 3 PRIMERAS DE LA RODAJA CON MÁS PUNTOS
     int maxSeccionHead = 0;
     int indMaxHead=0;

     int maxSeccionShoulder = 0;
     int indMaxShoulder=9;

     for (int i=0; i<3; i++)
     {
         if (maxSeccionHead < secciones2cm[i])
         {
            maxSeccionHead = secciones2cm[i];
            indMaxHead = i;

         }
     }


     //cout <<"indMaxHead = " << indMaxHead << endl;

     ///ASIGNACIÓN DE LAS RODAJAS DE 2CM A LOS VECTORES 0 1 2

     if (indMaxHead == 0)
     {
         secciones6cm[0]= secciones2cm[indMaxHead] + secciones2cm[indMaxHead+1] + secciones2cm[indMaxHead+2];
         secciones6cm[1]= secciones2cm[indMaxHead+3] + secciones2cm[indMaxHead+4] + secciones2cm[indMaxHead+5];
         secciones6cm[2]= secciones2cm[indMaxHead+6] + secciones2cm[indMaxHead+7] + secciones2cm[indMaxHead+8];
     }
     else if (indMaxHead > 0)
     {
         indMaxHead = indMaxHead -1;
         secciones6cm[0]= secciones2cm[indMaxHead] + secciones2cm[indMaxHead+1] + secciones2cm[indMaxHead+2];
         secciones6cm[1]= secciones2cm[indMaxHead+3] + secciones2cm[indMaxHead+4] + secciones2cm[indMaxHead+5];
         secciones6cm[2]= secciones2cm[indMaxHead+6] + secciones2cm[indMaxHead+7] + secciones2cm[indMaxHead+8];
     }


     ///BUSQUEDA DE LAS RODAJAS DE LOS HOMBROS

     for (int i= (indMaxHead+9); i< 20; i++)
     {
         //cout << "bucle 2 => i = " << i << endl;
         if (maxSeccionShoulder < secciones2cm[i])
         {
            maxSeccionShoulder = secciones2cm[i];
            indMaxShoulder = i;

         }
     }


     ///ASIGNACIÓN DE LAS RODAJAS DE 2CM A LOS VECTORES 3 4 5

      if (indMaxShoulder<14)
        {
          secciones6cm[3]= secciones2cm[indMaxShoulder] + secciones2cm[indMaxShoulder+1] + secciones2cm[indMaxShoulder+2];
          secciones6cm[4]= secciones2cm[indMaxShoulder+3] + secciones2cm[indMaxShoulder+4] + secciones2cm[indMaxShoulder+5];
        }
      else
         {
          indMaxShoulder = 14;
          secciones6cm[3]= secciones2cm[indMaxShoulder] + secciones2cm[indMaxShoulder+1] + secciones2cm[indMaxShoulder+2];
          secciones6cm[4]= secciones2cm[indMaxShoulder+3] + secciones2cm[indMaxShoulder+4] + secciones2cm[indMaxShoulder+5];
        }

 }



 /////////////////////////////////////////////////////////////////////////
 ///////////////////////////Vector 6 EJES DE LA CABEZA/////////////////////
 /////////////////////////////////////////////////////////////////////////

 Mat Gradiente(Mat &original)
 {

     int scale = 1;
     int delta = 0;
     int ddepth = CV_16S;


     //________parte relativa al filtro sobel________//

     Mat grad;
     Mat grad_x, grad_y;
     Mat abs_grad_x, abs_grad_y;

     /// Gradient X
     Sobel( original, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
     convertScaleAbs( grad_x, abs_grad_x );

     /// Gradient Y
     Sobel( original, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
     convertScaleAbs( grad_y, abs_grad_y );

     addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

     return grad;
}

 float headAxis (vector<Point> extremosHead, vector<Point> puntosCabeza, Mat img)
 {
    float d[2];
    d[0] = sqrt((extremosHead.at(0).x - extremosHead.at(1).x)*(extremosHead.at(0).x - extremosHead.at(1).x) + (extremosHead.at(0).y - extremosHead.at(1).y)*(extremosHead.at(0).y - extremosHead.at(1).y));
    d[1] = sqrt((extremosHead.at(2).x - extremosHead.at(3).x)*(extremosHead.at(2).x - extremosHead.at(3).x) + (extremosHead.at(2).y - extremosHead.at(3).y)*(extremosHead.at(2).y - extremosHead.at(3).y));
    int ind,dmax;
    if (d[0]>=d[1]){ind = 0;dmax = 0;}
    else{ind = 2;dmax=1;}



    //calcular la recta entre los puntos Xmax y Xmin
     vector <float> vector_rect (2);
         vector_rect[0] = (extremosHead.at(ind).x - extremosHead.at(ind+1).x)/d[dmax];
         vector_rect[1] = (extremosHead.at(ind).y - extremosHead.at(ind+1).y)/d[dmax];


    vector <float> vector_perp (2);
         vector_perp[0]= -vector_rect[1];
         vector_perp[1]= vector_rect[0];
    Point medio = Point (abs(extremosHead.at(ind).x + extremosHead.at(ind+1).x)/2, abs(extremosHead.at(ind).y + extremosHead.at(ind+1).y)/2);

    //cout << "vector de la recta de las X = " << vector_rect[0] <<" " << vector_rect[1] <<" vector perpendicualr = " << vector_perp[0] <<" " << vector_perp[1]<< " punto del medio = " << medio << endl;

    //creación de una imagen solo con los píxeles de la cabeza para calcular gradiente
    Mat zeros = Mat::zeros(img.size(),CV_16UC1);
    for (int a = 0; a<puntosCabeza.size(); a++ )
    {
        circle( zeros, puntosCabeza.at(a), 2, Scalar(255,255,255), -1, 8, 0 );
    }
    Mat grad = Gradiente(zeros);


    //acumulación de los puntos que pertenecen al gradiente
    float distMax = 0;
    Point pointExtremo;
    for (int y = 0; y<grad.rows; y++ )
    {
        for (int x = 0; x<grad.cols; x++ )
        {
            if(grad.at<uchar>(y,x) > 0)
            {
             if (round(vector_perp[0]*y - vector_perp[0]*medio.y - vector_perp[1]*x + vector_perp[1]*medio.x) == 0)
                {
                    if (distMax < sqrt((medio.x-x)*(medio.x-x)+(medio.y-y)*(medio.y-y)))
                    {
                        distMax = sqrt((medio.x-x)*(medio.x-x)+(medio.y-y)*(medio.y-y));
                        pointExtremo = Point(x,y);
                    }

                }

            }
        }
    }

    float relationV6;
    if (distMax > d[dmax]/2)
        relationV6=  (d[dmax]/2) /distMax;
    else
        relationV6= distMax / (d[dmax]/2);

if(VISUALIZACION==1)
{
   // circle (img, pointExtremo,2,Scalar (0,0,255), -1, 8, 0);
    line (img, medio,extremosHead.at(ind),Scalar(255,0,0),1,8,0);
    line (img, medio, pointExtremo,Scalar(0,0,255),1, 8, 0);
  // for(int hh = 0; hh < 200; hh++)
  //     circle(img, Point(medio.x+hh*vector_perp[0], medio.y+hh*vector_perp[1]), 2, Scalar(0,0,0), -1, 8, 0 );

    //imshow("ejes", img);
}
    return relationV6;
 }



 ////////////////////////////////////////////////////////
 /////////////////NORMALIZAR VECTOR////////////////////////
 //////////////////////////////////////////////////////////////////
 ///

 int normalizationVector(float **points, double **pointNorm, int numMax)
 {

     for (int i=0;i<numMax;i++)
     {
        int altura = (maxSUELO - points[i][NUMCARAC])/10;
        //double normFactor = altura*1.2 - 1083.2;
        double normFactor = altura*altura*0.137974 - altura*36.393962 + 2997.171477;
        for(int j=0;j<NUMCARAC-1;j++)
        {
            pointNorm[i][j] = points[i][j]/normFactor;
        }
        pointNorm[i][NUMCARAC-1] = points[i][NUMCARAC-1];
     }
     return 0;
 }
