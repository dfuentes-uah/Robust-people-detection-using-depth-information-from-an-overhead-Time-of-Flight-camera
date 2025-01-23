#include <stdio.h>


#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <iostream>

#include"Funciones_opencv.h"
#include "defines.h"


using namespace cv;
using namespace std;
///////////////////////////////////////////////////////
//////////// ELIMINACION DE RUIDO ////////////////////
//////////////////////////////////////////////////////
int ruido(cv::Mat imgIn, cv::Mat *imgOut)
{
  rectangle(imgIn, Point(20,20), Point(492,404), Scalar(255),60, 8, 0);
  for  ( int y =  0 ; y < imgIn.rows ; y++ )
    {
      for  ( int x =  0 ; x< imgIn.cols ; x++ )
        {
          if(imgIn.at<uchar>(Point(x,y))<40)
            {
              int pixel=0;
              while(imgIn.at<uchar>(Point(x+pixel, y))<40 )pixel++;
              imgIn.at<uchar>(Point(x, y))=imgIn.at<uchar>(Point(x+pixel, y));
            }
        }
    }

  *imgOut=imgIn;
  return 0;
}

///////////////////////////////////////////////////////
//////////// ELIMINACION DE RUIDO uint16////////////////////
//////////////////////////////////////////////////////
int ruido_uint16(cv::Mat imgIn, cv::Mat *imgOut)
{
<<<<<<< direccion.cpp
   double min,max;
   Point minLoc,maxLoc;
   minMaxLoc(imgIn,&min,&max,&minLoc,&maxLoc);
      rectangle(imgIn, Point(20,20), Point(492,404), max,60, 8, 0);
=======
  double min,max;
  Point minLoc,maxLoc;
  minMaxLoc(imgIn,&min,&max,&minLoc,&maxLoc);
  rectangle(imgIn, Point(20,20), Point(492,404), max,60, 8, 0);
>>>>>>> 1.7


  for (int i=0; i<imgIn.rows; i++)
    {
      for(int j=0; j<imgIn.cols; j++)
        {
          if (imgIn.at<unsigned short>(i,j)<400)
            {
              imgIn.at<unsigned short>(i,j)=nnSearch(imgIn, i,j,5);

            }
        }
    }
  //Mat imageAux;
  //imgIn(Rect(20, 20, 472,384)).copyTo(imageAux);
  *imgOut=imgIn;
  return 0;


}

///////////////////////////////////////////////////////////////////////////
//////////// ELIMINACION DE PÍXELES POR DEBAJO DEL SUELO////////////////////
////////////////////////////////////////////////////////////////////////////
int pixelesErroneosSuelo(Mat imgIn,Mat *imgOut)
{
<<<<<<< direccion.cpp
    for(int i=0; i<imgIn.rows; i++)
=======
  for(int i=0; i<imgIn.rows; i++)
>>>>>>> 1.7
    {
      for(int j=0; j<imgIn.cols; j++)
        {
          if(imgIn.at<unsigned short>(i,j) > maxSUELO)
            imgIn.at<unsigned short>(i,j)= maxSUELO;
        }
    }
  *imgOut = imgIn;

  return 0;
    
}


/////////////////////////////////////////////////////////////////
/////////////////////////////PINTAR NUCLEO EN MATRIZ DE MAXIMOS//////////
///////////////////////////////////////////////////////////////////
int pintanucleomaximos(cv::Mat imgIn, cv::Mat *imgOut,cv::Point minLoc)
{
  imgIn.at<uchar>(Point(minLoc.x+1,minLoc.y))=255;
  imgIn.at<uchar>(Point(minLoc.x,minLoc.y+1))=255;
  imgIn.at<uchar>(Point(minLoc.x+1,minLoc.y+1))=255;
  imgIn.at<uchar>(Point(minLoc.x-1,minLoc.y))=255;
  imgIn.at<uchar>(Point(minLoc.x,minLoc.y-1))=255;
  imgIn.at<uchar>(Point(minLoc.x-1,minLoc.y-1))=255;
  imgIn.at<uchar>(Point(minLoc.x+1,minLoc.y-1))=255;
  imgIn.at<uchar>(Point(minLoc.x-1,minLoc.y+1))=255;
  *imgOut=imgIn;
  return 0;
}



/////////////////////////////////////////////////////////////////////////
///////////////////////////obtener secciones persona/////////////////////
/////////////////////////////////////////////////////////////////////////
int numPointsSeccion(std :: vector < Mat > smallImages,cv::Point minLoc,double minVal,int posicion,int secciones[5])
{

  uint16_t increment2cm = 20;
  uint16_t increment6cm = 60;


  uint16_t ptocabeza16 = (uint16_t)(minVal);

  for(int y=0;y<=(smallImages.at(posicion)).rows;y++)
    {
      for(int x=0;x<=(smallImages.at(posicion)).cols;x++)
        {

          if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<ptocabeza16+(increment6cm*1))
            {
              secciones[0]++;
            }

          if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<ptocabeza16+(increment6cm*2) & smallImages.at(posicion).at<unsigned short>(Point(x, y))>ptocabeza16+(increment6cm*1))
            {
              secciones[1]++;
            }

          if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<ptocabeza16+(increment6cm*3) & smallImages.at(posicion).at<unsigned short>(Point(x, y))>ptocabeza16+(increment6cm*2))
            {
              secciones[2]++;
            }

          if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<ptocabeza16+(increment6cm*4) & smallImages.at(posicion).at<unsigned short>(Point(x, y))>ptocabeza16+(increment6cm*3))
            {
              secciones[3]++;
            }

          if(smallImages.at(posicion).at<unsigned short>(Point(x, y))<ptocabeza16+(increment6cm*5) & smallImages.at(posicion).at<unsigned short>(Point(x, y))>ptocabeza16+(increment6cm*4))
            {
              secciones[4]++;
            }
        }
    }
}



////////////////////////////////////////////////////////
/////////////////ELIMINA SECCIONES////////////////////////
//////////////////////////////////////////////////////////////////
///
int delecteSecciones(int position,Point coord,std :: vector < Mat > smallImages, double maxVal)
{
  for(int y=0;y<=(smallImages.at(position)).rows;y++)
    {
      for(int x=0;x<=(smallImages.at(position)).cols;x++)
        {
<<<<<<< direccion.cpp
            //smallImages.at(position).at<uchar>(Point(x, y)) = maxSUELO;
            circle(smallImages.at(position),Point(x, y),1,Scalar(maxSUELO),-1,8,0);
            //circle(smallImages.at(position),Point(coord.x-20, coord.y-20),3,Scalar(0),-1,8,0);
=======
          //smallImages.at(position).at<uchar>(Point(x, y)) = maxSUELO;
          circle(smallImages.at(position),Point(x, y),1,Scalar(maxSUELO),-1,8,0);
          //circle(smallImages.at(position),Point(coord.x-20, coord.y-20),3,Scalar(0),-1,8,0);
>>>>>>> 1.7
        }
    }

  return 0;
}


////////////////////////////////////////////////////////
/////////////////RECONSTRUIR IMAGEN SIN SUBROIS////////////////////////
//////////////////////////////////////////////////////////////////
///

int reconstruccionImage(std :: vector < Mat > smallImages, Mat& imageReconstruct)
{
  cv :: Size smallSize ( 20 , 20);
  int position=0;

  for  ( int y =  0 ; y+smallSize . height <= imageReconstruct . rows ; y += smallSize . height )
    {

      for  ( int x =  0 ; x + smallSize . width<= imageReconstruct . cols ; x += smallSize . width )
        {
          smallImages.at(position).copyTo(imageReconstruct(Rect(x, y, smallImages.at(position).rows,smallImages.at(position).cols)));
          position++;

        }
    }
  return 0;
}




///*****************************************************************//
///*******************DIRECIONES SEGUN PAPER************************//
///*****************************************************************//


//////////////////////////// DIRECCION 1 /////////////////////////////
/////////////////////////////////////////////////////////////////////
int direction1(cv::Mat img, cv::Mat imgOut,cv::Mat *imgOut2,cv::Point minLoc,double minVal,std :: vector < Mat > smallImages,int aceptadas[17],int *contador1)
{
  int level1=1, level2=2, level3=2,level4=4;
  int limitX =25;
  int limitY = 20;
  int minimo = (minVal + 400);
  int k;
  int acumSR[4] = {0, 0, 0,0} , contador = 0;
  int noValidos[7] = {0,0,0,0,0,0,0};
  int v=0, bandera=0;

  circle( imgOut,Point(((minLoc.x)*20)+4,((minLoc.y)*20)+12),10,Scalar( 0, 0, 255 ),-1,8 );

  ///LEVEL1
  for(int j = minLoc.x; j<=minLoc.x;j++)
    {
      k = minLoc.y-1;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          //cout << img.at<unsigned short>(Point(j,k)) << endl;
          if (/*acumSR[0] >= (level1-1) &&*/(img.at<unsigned short>(Point(j,k)) <= minimo ))
            {
              aceptadas[contador]=((k*25)+j);
              contador++;
              acumSR[0]++;
              circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );
              //cout << img.at<unsigned short>(Point(j,k+1)) << " " << img.at<unsigned short>(Point(j,k)) << " " << img.at<unsigned short>(Point(j,k-1)) << endl;

              /*if ((img.at<unsigned short>(Point(j,k+1)) <= img.at<unsigned short>(Point(j,k))) && (img.at<unsigned short>(Point(j,k)) <= img.at<unsigned short>(Point(j,k-1))))
                {
                aceptadas[contador]=((k*25)+j);
                contador++;
                acumSR[0]++;
                circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

                }
                else
                {
                //cout << "pertenece a dos personas como lo hago?? " << endl;
                noValidos[v]=j;
                v++;

                }*/
            }
          else
            {
              noValidos[v]=j;
              v++;
            }
        }
    }



  ///lEVEL2
  for(int j = minLoc.x-1; j<=minLoc.x+1;j++)
    {
      bandera = 0;
      //cout << "k = " << k;
      for (int w = 0; w < v; w++)
        {   //cout << "no validos = " << noValidos[w];
          if (k == noValidos[w]){bandera=1;break;}
        }
      if(bandera)continue;
      k = minLoc.y -2;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          if (acumSR[0] >= (level1-1) && (img.at<unsigned short>(Point(j,k)) <= minimo ))
            {
              //circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

              if ((img.at<unsigned short>(Point(j,k+1)) <= img.at<unsigned short>(Point(j,k))) && (img.at<unsigned short>(Point(j,k)) <= img.at<unsigned short>(Point(j,k-1))))
                {
                  aceptadas[contador]=((k*25)+j);
                  contador++;
                  acumSR[1]++;
                  circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );
                }
              else
                {
                  ////cout << "pertenece a dos personas como lo hago?? " << endl;
                  noValidos[v]=j;
                  v++;
                }
            }
          else
            {
              noValidos[v]=j;
              v++;
            }
        }
    }



  ///LEVEL3
  for(int j = minLoc.x-2; j<=minLoc.x+2;j++)
    {
      bandera = 0;
      for (int w = 0; w < v; w++)
        {
          if (j == noValidos[w]){bandera=1;break;}
        }
      if(bandera)continue;
      k = minLoc.y -3;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          if (acumSR[1] >= (level2-1) && (img.at<unsigned short>(Point(j,k)) <= minimo ))
            {
              //circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

              if ((img.at<unsigned short>(Point(j,k+1)) <= img.at<unsigned short>(Point(j,k))) && ( img.at<unsigned short>(Point(j,k)) <= img.at<unsigned short>(Point(j,k-1))))
                {
                  aceptadas[contador]=((k*25)+j);
                  contador++;
                  acumSR[2]++;
                  circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );
                }
              else
                {
                  //cout << "pertenece a dos personas como lo hago?? " << endl;
                  noValidos[v]=j;
                  v++;
                }
            }
          else
            {
              noValidos[v]=j;
              v++;
            }
        }
    }


  ///LEVEL4
  for(int j = minLoc.x-3; j<=minLoc.x+3;j++)
    {
      bandera = 0;
      for (int w = 0; w < v; w++)
        {
          if (j == noValidos[w]){bandera=1;break;}
        }
      if(bandera)continue;
      k = minLoc.y -4;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          if (acumSR[2] >= (level3-1) && (img.at<unsigned short>(Point(j,k)) <= minimo ))
            {
              //circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

              if (img.at<unsigned short>(Point(j,k+1)) >= img.at<unsigned short>(Point(j,k)) >= img.at<unsigned short>(Point(j,k-1)))
                {
                  aceptadas[contador]=((k*25)+j);
                  contador++;
                  acumSR[3]++;
                  circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );
                }
              else
                {
                  //cout << "pertenece a dos personas como lo hago?? " << endl;
                  noValidos[v]=j;
                  v++;
                }
            }
          else
            {
              noValidos[v]=j;
              v++;
            }
        }
    }

  *contador1 = contador;
  *imgOut2=imgOut;

  return 0;
}


//////////////////////////// DIRECCION 2 /////////////////////////////
/////////////////////////////////////////////////////////////////////
int direction2(cv::Mat img, cv::Mat imgOut,cv::Mat *imgOut2,cv::Point minLoc,double minVal,std :: vector < Mat > smallImages,int aceptadas[17],int *contador2)
{
  int level1=1, level2=2, level3=2,level4=4;
  int limitX =25;
  int limitY = 20;
  int minimo = (minVal + 400);
  int j;
  int acumSR[4] = {0, 0, 0,0} , contador = 0;
  int noValidos[7] = {0,0,0,0,0,0,0};
  int v=0, bandera=0;

  circle( imgOut,Point(((minLoc.x)*20)+4,((minLoc.y)*20)+12),10,Scalar( 0, 0, 255 ),-1,8 );

  ///LEVEL1
  for(int k = minLoc.y; k<=minLoc.y;k++)
    {
      j = minLoc.x+1;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          if (/*acumSR[0] >= (level1-1) &&*/(img.at<unsigned short>(Point(j,k)) <= minimo ))
            {
              aceptadas[contador]=((k*25)+j);
              contador++;
              acumSR[0]++;
              circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

              /*  if ((img.at<unsigned short>(Point(j-1,k)) <= img.at<unsigned short>(Point(j,k))) && (img.at<unsigned short>(Point(j,k)) <= img.at<unsigned short>(Point(j+1,k))))
                  {
                  aceptadas[contador]=((k*25)+j);
                  contador++;
                  acumSR[0]++;
                  circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

                  }
                  else
                  {
                  //cout << "pertenece a dos personas como lo hago?? " << endl;
                  noValidos[v]=k;
                  v++;
                  }*/
            }
          else
            {
              noValidos[v]=k;
              v++;
            }
        }
    }

  ///LEVEL2
  for(int k = minLoc.y-1; k<=minLoc.y+1;k++)
    {
      bandera = 0;
      for (int w = 0; w < v; w++)
        {
          if (k == noValidos[w]){bandera=1;break;}
        }
      //cout << endl;
      if(bandera)continue;
      j = minLoc.x +2;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          if (acumSR[0] >= (level1-1) && (img.at<unsigned short>(Point(j,k)) <= minimo ))
            {
              //circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 25555, 255, 0 ),-1,8 );

              if ((img.at<unsigned short>(Point(j-1,k)) <= img.at<unsigned short>(Point(j,k))) && ( img.at<unsigned short>(Point(j,k))<= img.at<unsigned short>(Point(j+1,k))))
                {
                  //cout << "level 2 " << img.at<unsigned short>(Point(j-1,k)) << " " << img.at<unsigned short>(Point(j,k)) << " " << img.at<unsigned short>(Point(j+1,k)) << endl;
                  aceptadas[contador]=((k*25)+j);
                  contador++;
                  acumSR[1]++;
                  circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );
                }
              else
                {
                  //cout << "pertenece a dos personas como lo hago?? " << endl;
                  noValidos[v]=k;
                  v++;
                }
            }
          else
            {
              noValidos[v]=k;
              v++;
            }
        }
    }


  ///LEVEL3
  for(int k = minLoc.y-2; k<=minLoc.y+2;k++)
    {
      bandera = 0;
      for (int w = 0; w < v; w++)
        {
          if (k == noValidos[w]){bandera=1;break;}
        }
      if(bandera)continue;
      j = minLoc.x +3;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          if (acumSR[1] >= (level2-1) && (img.at<unsigned short>(Point(j,k)) <= minimo ))
            {
              //circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

              if ((img.at<unsigned short>(Point(j-1,k)) <= img.at<unsigned short>(Point(j,k))) && (img.at<unsigned short>(Point(j,k))<= img.at<unsigned short>(Point(j+1,k))))
                {
                  //cout << "level 3 " << img.at<unsigned short>(Point(j-1,k)) << " " << img.at<unsigned short>(Point(j,k)) << " " << img.at<unsigned short>(Point(j+1,k)) << endl;
                  aceptadas[contador]=((k*25)+j);
                  contador++;
                  acumSR[2]++;
                  circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );
                }
              else
                {
                  //cout << "pertenece a dos personas como lo hago?? " << endl;
                  noValidos[v]=k;
                  v++;
                }
            }
          else
            {
              noValidos[v]=k;
              v++;
            }
        }
    }



  ///LEVEL4
  for(int k = minLoc.y-3; k<=minLoc.y+3;k++)
    {
      bandera = 0;
      for (int w = 0; w < v; w++)
        {
          if (k == noValidos[w]){bandera=1;break;}
        }
      if(bandera)continue;
      j = minLoc.x +4;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          if (acumSR[2] >= (level3-1) && (img.at<unsigned short>(Point(j,k)) <= minimo ))
            {
              //circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

              if ((img.at<unsigned short>(Point(j-1,k)) <= img.at<unsigned short>(Point(j,k))) && ( img.at<unsigned short>(Point(j,k))<= img.at<unsigned short>(Point(j+1,k))))
                {
                  //cout << "level 4 " << img.at<unsigned short>(Point(j-1,k)) << " " << img.at<unsigned short>(Point(j,k)) << " " << img.at<unsigned short>(Point(j+1,k)) << endl;
                  aceptadas[contador]=((k*25)+j);
                  contador++;
                  acumSR[3]++;
                  circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );
                }
              else
                {
                  //cout << "pertenece a dos personas como lo hago?? " << endl;
                  noValidos[v]=k;
                  v++;
                }
            }
          else
            {
              noValidos[v]=k;
              v++;
            }
        }
    }

  *contador2 = contador;
  *imgOut2=imgOut;

  return 0;
}

//////////////////////////// DIRECCION 3 /////////////////////////////
/////////////////////////////////////////////////////////////////////255
int direction3(cv::Mat img, cv::Mat imgOut,cv::Mat *imgOut2,cv::Point minLoc,double minVal,std :: vector < Mat > smallImages,int aceptadas[17],int *contador3)
{
  int level1=1, level2=2, level3=2,level4=4;
  int limitX =25;
  int limitY = 20;
  int minimo = (minVal + 400);
  int k;
  int acumSR[4] = {0, 0, 0,0} , contador = 0;
  int noValidos[7] = {0,0,0,0,0,0,0};
  int v=0, bandera=0;

  circle( imgOut,Point(((minLoc.x)*20)+4,((minLoc.y)*20)+12),10,Scalar( 0, 0, 255 ),-1,8 );

  ///LEVEL1
  for(int j = minLoc.x; j<=minLoc.x;j++)
    {

      k = minLoc.y+1;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          if (/*acumSR[0] >= (level1-1) &&*/(img.at<unsigned short>(Point(j,k)) <= minimo ))
            {
              aceptadas[contador]=((k*25)+j);
              contador++;
              acumSR[0]++;
              circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

              /* if ((img.at<unsigned short>(Point(j,k-1)) <= img.at<unsigned short>(Point(j,k))) && ( img.at<unsigned short>(Point(j,k)) <= img.at<unsigned short>(Point(j,k+1))))
                 {
                 aceptadas[contador]=((k*25)+j);
                 contador++;
                 acumSR[0]++;
                 circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

                 }
                 else
                 {
                 //cout << "pertenece a dos personas como lo hago?? " << endl;
                 noValidos[v]=j;
                 v++;
                 }
              */
            }
          else
            {
              noValidos[v]=j;
              v++;
            }

        }
    }


  ///LEVEL2
  for(int j = minLoc.x-1; j<=minLoc.x+1;j++)
    {
      bandera = 0;
      for (int w = 0; w < v; w++)
        {
          if (j == noValidos[w]){bandera=1;break;}
        }
      if(bandera)continue;
      k = minLoc.y +2;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          if (acumSR[0] >= (level1-1) && (img.at<unsigned short>(Point(j,k)) <= minimo ))
            {
              //circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

              if ((img.at<unsigned short>(Point(j,k-1)) <= img.at<unsigned short>(Point(j,k))) && ( img.at<unsigned short>(Point(j,k))<= img.at<unsigned short>(Point(j,k+1))))
                {
                  aceptadas[contador]=((k*25)+j);
                  contador++;
                  acumSR[1]++;
                  circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );
                }
              else
                {
                  //cout << "pertenece a dos personas como lo hago?? " << endl;
                  noValidos[v]=j;
                  v++;
                }
            }
          else
            {
              noValidos[v]=j;
              v++;
            }
        }
    }


  ///LEVEL3
  for(int j = minLoc.x-2; j<=minLoc.x+2;j++)
    {
      bandera = 0;
      for (int w = 0; w < v; w++)
        {
          if (j == noValidos[w]){bandera=1;break;}
        }
      if(bandera)continue;
      k = minLoc.y +3;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          if (acumSR[1] >= (level2-1) && (img.at<unsigned short>(Point(j,k)) <= minimo ))
            {
              //circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

              if ((img.at<unsigned short>(Point(j,k-1)) <= img.at<unsigned short>(Point(j,k))) && ( img.at<unsigned short>(Point(j,k))<= img.at<unsigned short>(Point(j,k+1))))
                {
                  aceptadas[contador]=((k*25)+j);
                  contador++;
                  acumSR[2]++;
                  circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );
                }
              else
                {
                  //cout << "pertenece a dos personas como lo hago?? " << endl;
                  noValidos[v]=j;
                  v++;
                }
            }
          else
            {
              noValidos[v]=j;
              v++;
            }
        }
    }



  ///LEVEL4
  for(int j = minLoc.x-3; j<=minLoc.x+3;j++)
    {
      bandera = 0;
      for (int w = 0; w < v; w++)
        {
          if (j == noValidos[w]){bandera=1;break;}
        }
      if(bandera)continue;
      k = minLoc.y +4;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          if (acumSR[2] >= (level3-1) && (img.at<unsigned short>(Point(j,k)) <= minimo ))
            {
              //circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

              if ((img.at<unsigned short>(Point(j,k-1)) >= img.at<unsigned short>(Point(j,k))) && (img.at<unsigned short>(Point(j,k))>= img.at<unsigned short>(Point(j,k+1))))
                {
                  aceptadas[contador]=((k*25)+j);
                  contador++;
                  acumSR[3]++;
                  circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );
                }
              else
                {
                  //cout << "pertenece a dos personas como lo hago?? " << endl;
                  noValidos[v]=j;
                  v++;
                }
            }
          else
            {
              noValidos[v]=j;
              v++;
            }
        }
    }

  *contador3 = contador;
  *imgOut2=imgOut;

  return 0;
}

//////////////////////////// DIRECCION 4 /////////////////////////////
/////////////////////////////////////////////////////////////////////
int direction4(cv::Mat img, cv::Mat imgOut,cv::Mat *imgOut2,cv::Point minLoc,double minVal,std :: vector < Mat > smallImages,int aceptadas[17],int *contador4)
{
  int level1=1, level2=2, level3=2,level4=4;
  int limitX =25;
  int limitY = 20;
  int minimo = (minVal + 400);
  int j;
  int acumSR[4] = {0, 0, 0,0} , contador = 0;
  int noValidos[7] = {0,0,0,0,0,0,0};
  int v=0, bandera=0;

  circle( imgOut,Point(((minLoc.x)*20)+4,((minLoc.y)*20)+12),10,Scalar( 0, 0, 255 ),-1,8 );

  ///LEVEL1
  for(int k = minLoc.y; k<=minLoc.y;k++)
    {
      j = minLoc.x-1;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          if (/*acumSR[0] >= (level1-1) &&*/(img.at<unsigned short>(Point(j,k)) <= minimo ))
            {    aceptadas[contador]=((k*25)+j);
              contador++;
              acumSR[0]++;
              circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

              //cout << img.at<unsigned short>(Point(j+1,k)) << " " << img.at<unsigned short>(Point(j,k)) << " " << img.at<unsigned short>(Point(j-1,k)) << endl;

              /* if ((img.at<unsigned short>(Point(j+1,k)) <= img.at<unsigned short>(Point(j,k))) && (img.at<unsigned short>(Point(j,k)) <= img.at<unsigned short>(Point(j-1,k))))
                 {
                 aceptadas[contador]=((k*25)+j);
                 contador++;
                 acumSR[0]++;
                 circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

                 }
                 else
                 {
                 //cout << "pertenece a dos personas como lo hago?? " << endl;
                 noValidos[v]=k;
                 v++;
                 }*/
            }
          else
            {
              noValidos[v]=k;
              v++;
            }
        }
    }

  ///LEVEL2
  for(int k = minLoc.y-1; k<=minLoc.y+1;k++)
    {
      bandera = 0;
      for (int w = 0; w < v; w++)
        {
          if (k == noValidos[w]){bandera=1;break;}
        }
      if(bandera)continue;
      j = minLoc.x -2;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          if (acumSR[0] >= (level1-1) && (img.at<unsigned short>(Point(j,k)) <= minimo ))
            {
              //circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 25555, 255, 0 ),-1,8 );

              if ((img.at<unsigned short>(Point(j+1,k)) <= img.at<unsigned short>(Point(j,k))) && ( img.at<unsigned short>(Point(j,k))<= img.at<unsigned short>(Point(j-1,k))))
                {
                  aceptadas[contador]=((k*25)+j);
                  contador++;
                  acumSR[1]++;
                  circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );
                }
              else
                {
                  //cout << "pertenece a dos personas como lo hago?? " << endl;
                  noValidos[v]=k;
                  v++;
                }
            }
          else
            {
              noValidos[v]=k;
              v++;
            }
        }
    }


  ///LEVEL3
  for(int k = minLoc.y-2; k<=minLoc.y+2;k++)
    {
      bandera = 0;
      for (int w = 0; w < v; w++)
        {
          if (k == noValidos[w]){bandera=1;break;}
        }
      if(bandera)continue;
      j = minLoc.x -3;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          if (acumSR[1] >= (level2-1) && (img.at<unsigned short>(Point(j,k)) <= minimo ))
            {
              //circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

              if ((img.at<unsigned short>(Point(j+1,k)) <= img.at<unsigned short>(Point(j,k))) && ( img.at<unsigned short>(Point(j,k))<= img.at<unsigned short>(Point(j-1,k))))
                {
                  aceptadas[contador]=((k*25)+j);
                  contador++;
                  acumSR[2]++;
                  circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );
                }
              else
                {
                  //cout << "pertenece a dos personas como lo hago?? " << endl;
                  noValidos[v]=k;
                  v++;
                }
            }
          else
            {
              noValidos[v]=k;
              v++;
            }
        }
    }


  ///LEVEL4
  for(int k = minLoc.y-3; k<=minLoc.y+3;k++)
    {
      bandera = 0;
      for (int w = 0; w < v; w++)
        {
          if (k == noValidos[w]){bandera=1;break;}
        }
      if(bandera)continue;
      j = minLoc.x -4;
      circle(imgOut, Point(((j)*20)+4,((k)*20)+12),10,Scalar(0,255,255),-1,8);

      if(j>=0 && j <limitX && k>=0 && k<limitY)
        {
          if (acumSR[2] >= (level3-1) && (img.at<unsigned short>(Point(j,k)) <= minimo ))
            {
              //circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

              if ((img.at<unsigned short>(Point(j+1,k)) <= img.at<unsigned short>(Point(j,k))) && (img.at<unsigned short>(Point(j,k))<= img.at<unsigned short>(Point(j-1,k))))
                {
                  aceptadas[contador]=((k*25)+j);
                  contador++;
                  acumSR[3]++;
                  circle( imgOut,Point(((j)*20)+4,((k)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );
                }
              else
                {
                  //cout << "pertenece a dos personas como lo hago?? " << endl;
                  noValidos[v]=k;
                  v++;
                }
            }
          else
            {
              noValidos[v]=k;
              v++;
            }
        }
    }


  *contador4 = contador;
  *imgOut2=imgOut;

  return 0;
}


//////////////////////////// DIAGONAL /////////////////////////////
/////////////////////////////////////////////////////////////////////
int diagonal(cv::Mat img, cv::Mat imgOut,cv::Mat *imgOut2,cv::Point minLoc,double minVal,std :: vector < Mat > smallImages,int aceptadas[12],int *contador5)
{
  int level1=1, level2=2, level3=2,level4=4;
  int limitX =25;
  int limitY = 20;
  int minimo = (minVal + 400);
  Point evPoint;
  int acumSR[4] = {0, 0, 0,0} , contador = 0;
  int noValidos[4] = {1,1,1,1};

  circle( imgOut,Point(((minLoc.x)*20)+4,((minLoc.y)*20)+12),10,Scalar( 0, 0, 255 ),-1,8 );


  for(int d = 1; d<=4; d++)
    {
      ///DIRECCIÓN 5
      if (noValidos[0])
        {
          evPoint.x = minLoc.x +d;
          evPoint.y = minLoc.y -d;

          circle(imgOut, Point(((evPoint.x)*20)+4,(evPoint.y*20)+12),10,Scalar(0,255,0),-1,8);

          if(evPoint.x>=0 && evPoint.x<limitX && evPoint.y>=0 && evPoint.y<limitY)
            {
              if ((img.at<unsigned short>(evPoint) <= minimo ))
                {
                  if(d == 1)
                    {
                      aceptadas[contador]=((evPoint.y*25)+evPoint.x);
                      contador++;
                      acumSR[0]++;
                      circle( imgOut,Point(((evPoint.x)*20)+4,((evPoint.y)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );
                    }

                  else if ((img.at<unsigned short>(Point(evPoint.x-1,evPoint.y+1)) <= img.at<unsigned short>(evPoint)) && (img.at<unsigned short>(evPoint)<= img.at<unsigned short>(Point(evPoint.x+1,evPoint.y-1))))
                    {
                      aceptadas[contador]=((evPoint.y*25)+evPoint.x);
                      contador++;
                      acumSR[0]++;
                      circle( imgOut,Point(((evPoint.x)*20)+4,((evPoint.y)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

                    }
                  else
                    {
                      //cout << "pertenece a dos personas como lo hago?? " << endl;
                      noValidos[0]=0;
                    }
                }
              else noValidos[0]=0;
            }
        }

      ///DIRECCIÓN 6
      if (noValidos[1])
        {

          evPoint.x = minLoc.x +d;
          evPoint.y = minLoc.y +d;
          //cout << "x y analisis = "<< evPoint << endl;

          circle(imgOut, Point(((evPoint.x)*20)+4,(evPoint.y*20)+12),10,Scalar(0,255,0),-1,8);

          if(evPoint.x>=0 && evPoint.x<limitX && evPoint.y>=0 && evPoint.y<limitY)
            {
              if ((img.at<unsigned short>(evPoint) <= minimo ))
                {
                  if (d==1)
                    {
                      aceptadas[contador]=((evPoint.y*25)+evPoint.x);
                      contador++;
                      acumSR[1]++;
                      circle( imgOut,Point(((evPoint.x)*20)+4,((evPoint.y)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

                    }
                  else if ((img.at<unsigned short>(Point(evPoint.x-1,evPoint.y-1)) <= img.at<unsigned short>(evPoint))&&( img.at<unsigned short>(evPoint)<= img.at<unsigned short>(Point(evPoint.x+1,evPoint.y+1))))
                    {

                      aceptadas[contador]=((evPoint.y*25)+evPoint.x);
                      contador++;
                      acumSR[1]++;
                      circle( imgOut,Point(((evPoint.x)*20)+4,((evPoint.y)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

                    }
                  else
                    {
                      //cout << "pertenece a dos personas como lo hago?? " << endl;
                      noValidos[1]=0;
                    }
                }
              else noValidos[1]=0;
            }
        }

      ///DIRECCIÓN 7
      if (noValidos[2])
        {

          evPoint.x = minLoc.x -d;
          evPoint.y = minLoc.y +d;

          circle(imgOut, Point(((evPoint.x)*20)+4,(evPoint.y*20)+12),10,Scalar(0,255,0),-1,8);

          if(evPoint.x>=0 && evPoint.x<limitX && evPoint.y>=0 && evPoint.y<limitY)
            {
              if ((img.at<unsigned short>(evPoint) <= minimo ))
                {
                  if (d==1)
                    {
                      aceptadas[contador]=((evPoint.y*25)+evPoint.x);
                      contador++;
                      acumSR[2]++;
                      circle( imgOut,Point(((evPoint.x)*20)+4,((evPoint.y)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

                    }
                  else if ((img.at<unsigned short>(Point(evPoint.x+1,evPoint.y-1)) <= img.at<unsigned short>(evPoint)) && (img.at<unsigned short>(evPoint) <= img.at<unsigned short>(Point(evPoint.x-1,evPoint.y+1))))
                    {
                      aceptadas[contador]=((evPoint.y*25)+evPoint.x);
                      contador++;
                      acumSR[2]++;
                      circle( imgOut,Point(((evPoint.x)*20)+4,((evPoint.y)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

                    }
                  else
                    {
                      //cout << "pertenece a dos personas como lo hago?? " << endl;
                      noValidos[2]=0;
                    }
                }
              else noValidos[2]=0;
            }
        }

      ///DIRECCIÓN 8
      if (noValidos[3])
        {

          evPoint.x = minLoc.x -d;
          evPoint.y = minLoc.y -d;

          circle(imgOut, Point(((evPoint.x)*20)+4,(evPoint.y*20)+12),10,Scalar(0,255,0),-1,8);

          if(evPoint.x>=0 && evPoint.x<limitX && evPoint.y>=0 && evPoint.y<limitY)
            {
              if ((img.at<unsigned short>(evPoint) <= minimo ))
                {
                  if(d==1)
                    {
                      aceptadas[contador]=((evPoint.y*25)+evPoint.x);
                      contador++;
                      acumSR[3]++;
                      circle( imgOut,Point(((evPoint.x)*20)+4,((evPoint.y)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

                    }
                  if ((img.at<unsigned short>(Point(evPoint.x+1,evPoint.y+1)) <= img.at<unsigned short>(evPoint)) && (img.at<unsigned short>(evPoint)<= img.at<unsigned short>(Point(evPoint.x-1,evPoint.y-1))))
                    {
                      aceptadas[contador]=((evPoint.y*25)+evPoint.x);
                      contador++;
                      acumSR[3]++;
                      circle( imgOut,Point(((evPoint.x)*20)+4,((evPoint.y)*20)+12),10,Scalar( 255, 255, 0 ),-1,8 );

                    }
                  else
                    {
                      //cout << "pertenece a dos personas como lo hago?? " << endl;
                      noValidos[3]=0;
                    }
                }
              else noValidos[3]=0;
            }
        }
    }

  *contador5 = contador;
  *imgOut2=imgOut;

  return 0;


}


