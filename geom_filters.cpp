//para operaciones opencv
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include <pmdsdk2.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include "Funciones_opencv.h"







#define _USE_MATH_DEFINES
#define  IMAGE_CONV 0
#define IMAGE_PREPROCESS 1
#define MEXICAN_CONTOUR_RESULT 0
#define argumentosmexic 0
#define visualizarois 0

using namespace cv;
using namespace std;

 double minValg, maxValg;
 double minValgaus, maxValgaus;
 double minValLocal, maxValLocal;

 char fileName[20];
 char buffer_contorno[10];




///==========================================================================///
///                              FILTRO GAUSSIANO                            ///
///                                                                          ///
///==========================================================================///

void gaussian_filter(Mat& original_send)
{
      Mat original = original_send.clone();
      Mat src;
      imshow("src1", original);

      minMaxLoc(original, &minValg, &maxValg);
      //original.convertTo(original, CV_8U, 255.0/(maxValg - minValg), -minValg * 255.0/(maxValg - minValg));




    ///Declaración de las variables para crear el filtro
      Mat gaus_kernel; //= Mat(100,100, CV_32FC2); //matriz del filtro
      Point anchor; //punto de fijación del píxel que variamos
      double delta; //offset al píxel
      int ddepth; //tamaño imagen salida
      int kernel_size; //tamaño del filtro

      src = maxValg- original;///imread("../raquelTOFAlgs/objets_frames/frame_silla291.jpg",CV_LOAD_IMAGE_GRAYSCALE);
      //resize(src, src_r, Size2f(100,100), 0 ,0 ,INTER_LINEAR);
      imshow("src2", src);


      anchor = Point(-1,-1);
      delta = 0;
      ddepth = -1;

      kernel_size = 300;
      float mu_x = kernel_size/2;
      float mu_y = kernel_size/2;
      float ro_x= 30;
      float ro_y= 30;
      float ampl = (1/(ro_y*sqrt(2*M_PI)));

      double exp_aux_x, exp_aux_y;
      int j_bucle=0;
      int k_bucle=0;



      float* x_array = new float[kernel_size*kernel_size];
      float* y_array = new float[kernel_size*kernel_size];
      float* z_array = new float[kernel_size*kernel_size];

      for(int j=0; j < kernel_size; j++)
      {
          for (int k=0; k< kernel_size; k++)
          {

          x_array[kernel_size*j_bucle+k_bucle]=k;
          y_array[kernel_size*j_bucle+k_bucle]=j;

          exp_aux_x = (((k-mu_x)/ro_x)*((k-mu_x)/ro_x))*(-0.5);
          exp_aux_y = (((j-mu_y)/ro_y)*((j-mu_y)/ro_y))*(-0.5);

           z_array[kernel_size*j_bucle+k_bucle] = ampl* exp(exp_aux_x + exp_aux_y);

          if (z_array[kernel_size*j_bucle+k_bucle] < 0.005)
              z_array[kernel_size*j_bucle+k_bucle]=0;

          k_bucle++;
          }
        j_bucle++;
        k_bucle=0;
      }



  gaus_kernel =  Mat(kernel_size,kernel_size,CV_32FC1,z_array);

    if (IMAGE_CONV)
  imshow("ggaus", gaus_kernel);



  ///------------intento convolución manual


  Mat out = Mat::zeros(src.rows, src.cols, CV_32FC1);


/* CONVOLUCIÓN MANUAL


  int kCenterX = kernel_size/2;
  int kCenterY = kernel_size/2;

  int nn, mm, ii, jj, i, j, m, n;

  for(i=0; i < src.rows; ++i)              // rows
  {
      for(j=0; j < src.cols; ++j)          // columns
      {
          for(m=0; m < kernel_size; ++m)     // kernel rows
          {
              mm = kernel_size - 1 - m;      // row index of flipped kernel

              for(n=0; n < kernel_size; ++n) // kernel columns
              {
                  nn = kernel_size - 1 - n;  // column index of flipped kernel

                  // index of input signal, used for checking boundary
                  ii = i + (m - kCenterY);
                  jj = j + (n - kCenterX);

                  // ignore input samples which are out of bound
                  if( ii >= 0 && ii < src.rows && jj >= 0 && jj < src.cols )
                   {
                      out.at<float>(i,j) += src.at<uchar>(ii,jj)* gaus_kernel.at<uchar>(mm,nn);}

              }
          }
      }
  }

*/



  int borderMode = BORDER_DEFAULT;
  filter2D(src, out, ddepth , gaus_kernel, anchor, delta, borderMode);


  cout << "convoluciona" << endl;

  minMaxLoc(out, &minValgaus, &maxValgaus);
  cout << "Valor máximo = " << maxValgaus << endl;
  out.convertTo(out, CV_8U, 255.0/(maxValgaus - minValgaus), -minValgaus * 255.0/(maxValgaus - minValgaus));


  if (IMAGE_CONV)
  imshow("out", out);


}

///==========================================================================///
///                      FILTRO SOMBRERO MEXICANO                            ///
///                                                                          ///
///==========================================================================///

Mat mexican_filter(Mat& original_send, int newFilter)
{

      Mat original = original_send.clone();
      Mat src;
      FILE *fp;
      sprintf(fileName,"mexArray.data");



    ///Declaración de las variables para crear el filtro
      Mat mex_kernel; //= Mat(100,100, CV_32FC2); //matriz del filtro
      Point anchor; //punto de fijación del píxel que variamos
      double delta; //offset al píxel
      int ddepth; //tamaño imagen salida
      int kernel_size; //tamaño del filtro

      minMaxLoc(original, &minValg, &maxValg);
      cout << "maximos y minimos del la imagen a filtrar = " << maxValgaus << " " << minValgaus << endl;
      original.convertTo(original, CV_16UC1, 255.0/(maxValg - minValg), -minValg * 255.0/(maxValg - minValg));
      minMaxLoc(original, &minValg, &maxValg);


      src = original;//maxValg- original;
      //imshow("figura antes del filtro", src);

      anchor = Point(-1,-1);
      delta = 0;
      ddepth = -1;

      kernel_size = 100;
      float mu_x = kernel_size/2;
      float mu_y = kernel_size/2;
      float ro_x= 40;
      float ro_y= 40;
      //float ampl = (1/(ro_y*sqrt(2*M_PI)));
      float ampl = (2/(sqrt(3*ro_x)*1.33133));//*120;

      double exp_aux_x, exp_aux_y,ampli_comp;
      int j_bucle=0;
      int k_bucle=0;



      float* x_array = new float[kernel_size*kernel_size];
      float* y_array = new float[kernel_size*kernel_size];
      float* z_array = new float[kernel_size*kernel_size];

    if (newFilter == 1)

    {
          for(int j=0; j < kernel_size; j++)
          {
              for (int k=0; k< kernel_size; k++)
              {

              x_array[kernel_size*j_bucle+k_bucle]=k;
              y_array[kernel_size*j_bucle+k_bucle]=j;

              exp_aux_x = (((k-mu_x)/ro_x)*((k-mu_x)/ro_x))*(-0.5);
              exp_aux_y = (((j-mu_y)/ro_y)*((j-mu_y)/ro_y))*(-0.5);
              ampli_comp = 1-(((k-mu_x)/ro_x)*((k-mu_x)/ro_x) + (((j-mu_y)/ro_y)*((j-mu_y)/ro_y)));

              z_array[kernel_size*j_bucle+k_bucle] = ampl*ampli_comp*exp(exp_aux_x + exp_aux_y);

              k_bucle++;
              }
            j_bucle++;
            k_bucle=0;
          }

         //GRABACIÓN DEL ARRAY EN UN FICHERO



         if ((fp = fopen(fileName, "wb")) == NULL)
             {
              cout << "[ERROR] File " << fileName << " not open" << endl;
             }
         else
             fwrite(z_array, 1, kernel_size*kernel_size * sizeof(float), fp);

         fclose(fp);

    }
    else
    {

        if ((fp = fopen(fileName, "rb")) == NULL)
            {
             cout << "[ERROR] File " << fileName << " can not read" << endl;
            }
        else
          fread(z_array, 1, kernel_size*kernel_size * sizeof(float), fp);

          fclose(fp);
    }

  cout <<  "[INFO] Cerrando fichero " << fileName <<endl;
  mex_kernel =  Mat(kernel_size,kernel_size,CV_32FC1,z_array);

   if (IMAGE_CONV)
   {
      Mat mex_kernel_SHOW = mex_kernel.clone();
      minMaxLoc(mex_kernel_SHOW, &minValgaus, &maxValgaus);
      mex_kernel_SHOW.convertTo(mex_kernel_SHOW, CV_8U, 255.0/(maxValgaus - minValgaus), -minValgaus * 255.0/(maxValgaus - minValgaus));
      cout << "maximos y minimos del filtro = " << maxValgaus << " " << minValgaus << endl;
      imshow("mex", mex_kernel_SHOW);
    }



  Mat out = Mat::zeros(src.rows, src.cols, CV_32FC1);


/*CONVOLUCIÓN MANUAL


  int kCenterX = kernel_size/2;
  int kCenterY = kernel_size/2;

  int nn, mm, ii, jj, i, j, m, n;




  for(i=0; i < src.rows; ++i)              // rows
  {
      for(j=0; j < src.cols; ++j)          // columns
      {
          for(m=0; m < kernel_size; ++m)     // kernel rows
          {
              mm = kernel_size - 1 - m;      // row index of flipped kernel

              for(n=0; n < kernel_size; ++n) // kernel columns
              {
                  nn = kernel_size - 1 - n;  // column index of flipped kernel

                  // index of input signal, used for checking boundary
                  ii = i + (m - kCenterY);
                  jj = j + (n - kCenterX);

                  // ignore input samples which are out of bound
                  if( ii >= 0 && ii < src.rows && jj >= 0 && jj < src.cols )
                   {out.at<float>(i,j) += src.at<uchar>(ii,jj)* gaus_kernel.at<uchar>(mm,nn);}

              }
          }
      }
  }

*/


  int borderMode = BORDER_DEFAULT;
  filter2D(src, out, ddepth , mex_kernel, anchor, delta, borderMode);


  minMaxLoc(out, &minValgaus, &maxValgaus);
  out.convertTo(out, CV_8U, 255.0/(maxValgaus - minValgaus), -minValgaus * 255.0/(maxValgaus - minValgaus));
  //minMaxLoc(out, &minValgaus, &maxValgaus);

  if (IMAGE_CONV)
{
      imshow("out2", out);
      minMaxLoc(src, &minValgaus, &maxValgaus);
      src.convertTo(src, CV_8U, 255.0/(maxValgaus - minValgaus), -minValgaus * 255.0/(maxValgaus - minValgaus));
      imshow("src_dentro", src);
}


  return out;

}


///==========================================================================///
///                       DIVIDIR IMAGEN EN RODAJAS                          ///
///                                                                          ///
///==========================================================================///
///

void slice_image ()
{
    Mat src, src_th;
    int slice_th = 255;
    src = 255- imread("../raquelTOFAlgs/objets_frames/frame_silla291.jpg",CV_LOAD_IMAGE_GRAYSCALE);
    //resize(src, src_r, Size2f(100,100), 0 ,0 ,INTER_LINEAR);

    threshold( src, src_th, 128, 255, THRESH_BINARY_INV );
    src = src - src_th;

    int c = 0;
    while (c < 3)
{
    for (int i =0 ; i< src.rows ; i++)
    {
        for (int j=0;j<src.cols; j++)
        {
            if(src.at<uchar>(i,j) < slice_th)
                src.at<uchar>(i,j) = 255;
            else
                src.at<uchar>(i,j) = 0;
        }
    }

    imshow("slice", src);
    waitKey();
    c++;
    slice_th = slice_th-50;
}
}



///==========================================================================///
///                        FUNCIÓN PREPROCESADO_ROI                          ///
///                                                                          ///
///==========================================================================///
Mat preProcessROI (Mat& originalROI)
{
    int i, j;
    int16_t min = 0;
    int16_t max = 10000;

    Mat ROIStep = Mat::zeros(originalROI.rows, originalROI.cols, CV_32FC1) ;
    float siStep = 0;
    int numStep = 10;


    for (i=0; i< originalROI.rows; i++)
    {
        for (j=0; j< originalROI.cols; j++)
        {
            if ((originalROI.at<unsigned short>(i,j) < max) && (originalROI.at<unsigned short>(i,j) != 0))
                max=originalROI.at<unsigned short>(i,j);
            if (originalROI.at<unsigned short>(i,j) > min)
                min = originalROI.at<unsigned short>(i,j) ;
        }
    }
    //cout << "minimo = " << min << "maximo = " << max << endl;

    siStep = (min - max)/numStep;

    for (i=0; i< originalROI.rows; i++)
    {
        for (j=0; j< originalROI.cols; j++)
        {
           if (originalROI.at<unsigned short>(i,j) != 0)
           ROIStep.at<float >(i,j) = fabs(floor(((originalROI.at<unsigned short>(i,j) - min)/siStep))) +1;
           else
           ROIStep.at<float >(i,j) = 0;
        }

    }

    minMaxLoc(ROIStep, &minValg, &maxValg);

    Mat ROIStep_SHOW = ROIStep.clone();
    if (IMAGE_PREPROCESS)
    {

    ROIStep_SHOW.convertTo(ROIStep_SHOW, CV_8U, 255.0/(maxValg - minValg), -minValg * 255.0/(maxValg - minValg));
    imshow("ROIreconstruida", ROIStep_SHOW);

    Mat originalROI_SHOW = originalROI.clone();
    originalROI_SHOW.convertTo(originalROI_SHOW, CV_8U, 255.0/(maxValg - minValg), -minValg * 255.0/(maxValg - minValg));
    imshow("ROIoriginal", originalROI_SHOW);
    }

    return ROIStep_SHOW;
}


///==========================================================================///
///                         FUNCIÓN PREPROCESADO                             ///
///                                                                          ///
///==========================================================================///
Mat preProcess (Mat& original)
  {
     int i, j, c;

     if (IMAGE_PREPROCESS)
     {
         Mat original_SHOWa = original.clone();
         minMaxLoc(original_SHOWa, &minValg, &maxValg);
         original_SHOWa.convertTo(original_SHOWa, CV_8U, 255.0/(maxValg - minValg), -minValg * 255.0/(maxValg - minValg));
         imshow("consuelo", original_SHOWa);
     }

     ///1. ELIMINAMOS EL SUELO DE LA IMAGEN
     minMaxLoc(original, &minValg, &maxValg);
     //cout << "ANTES minVal = " <<minValg << " maxVal = " << maxValg << endl;
     int floor = maxValg*0.6;
     for (i=0; i<original.rows; i++)
     {
         for(j=0; j<original.cols; j++)
         {
             if (original.at<unsigned short>(i,j)> floor)
             {
                original.at<unsigned short>(i,j)=maxValg;
             }
         }
     }

     if (IMAGE_PREPROCESS)
     {
         Mat original_SHOW = original.clone();
         minMaxLoc(original_SHOW, &minValg, &maxValg);
         original_SHOW.convertTo(original_SHOW, CV_8U, 255.0/(maxValg - minValg), -minValg * 255.0/(maxValg - minValg));
         imshow("sinsuelo", original_SHOW);
     }

     ///2. DEFINIR UNA ROI POR CADA CONTORNO DE LA IMAGEN

     Mat original_aux = original.clone();
     Mat originalStep = Mat::ones(original.rows, original.cols,CV_32FC1);

     struct_count contornos = contador_contour(original_aux);


     data_ROI analizeROI;


     if (contornos.cont_ok)
     {
         cout << "Numero de contornos = " << contornos.contours.size() << endl;

         bool* contornosOK = new bool[contornos.contours.size()];
         int suma_contornosOK = 0;

         for(c = 0; c <contornos.contours.size(); c++)
         {
             if (contornos.radius_count[c] > 40)
             {
                 contornosOK[c]=true;
                 suma_contornosOK ++;
             }
             else
                 contornosOK[c]=false;
         }

         vector<Mat> images_ROI(suma_contornosOK);
         vector<Mat> ROIStep (suma_contornosOK);
         int ind = 0;


         for (c = 0; c <contornos.contours.size(); c++)
         {
             if (contornosOK[c])
             {
             cout << "Radio del contorno " << c << " = " << contornos.radius_count[c] << endl;

             analizeROI = cutROI(original, contornos.center_count[c], contornos.radius_count[c]);
             images_ROI[ind] = analizeROI.ROI;


             ROIStep[ind] = preProcessROI(images_ROI[ind]);

             //minMaxLoc(originalStep, &minValg, &maxValg);
             //originalStep.convertTo(originalStep, CV_8U, 255.0/(maxValg - minValg), -minValg * 255.0/(maxValg - minValg));


             ROIStep[ind].copyTo(originalStep(Rect(analizeROI.rect_start.x, analizeROI.rect_start.y, analizeROI.rect_add.x, analizeROI.rect_add.y)));
             minMaxLoc(ROIStep[ind], &minValg, &maxValg);
             ROIStep[ind].convertTo(ROIStep[ind], CV_8U, 255.0/(maxValg - minValg), -minValg * 255.0/(maxValg - minValg));

             ind++;

             }
         }


     }




      if (IMAGE_PREPROCESS)
      {
        Mat originalStep_SHOW = originalStep.clone();
        originalStep_SHOW.convertTo(originalStep_SHOW, CV_8U, 255.0/(maxValg - minValg), -minValg * 255.0/(maxValg - minValg));
        imshow("pegada el cacho", originalStep_SHOW);
      }


     return originalStep;
 }


///==========================================================================///
///                 LOCALIZACIÓN DE MÁXIMOS POR MEXICANO                     ///
///                                                                          ///
///==========================================================================///
int mexResult(char *FileName, int contFrame, int posX[], int posY[],int personas)
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

        fprintf(fp,"%d", contFrame);
 for(int i=0;i<personas;i=i+2)fprintf(fp, "\t 0 \t %d %d  \t 0.000", posX[i], posY[i]);
       }
  fprintf(fp, "\n");
      fclose(fp);
      return 1;

}
void eliminasuelo(Mat& original)
{
  minMaxLoc(original, &minValg, &maxValg);
     //cout << "ANTES minVal = " <<minValg << " maxVal = " << maxValg << endl;
     int floor = maxValg*0.6;
     for (int i=0; i<original.rows; i++)
     {
         for(int j=0; j<original.cols; j++)
         {
             if (original.at<unsigned short>(i,j)> floor)
             {
                original.at<unsigned short>(i,j)=maxValg;
             }
         }
     }
}
void erosion(Mat &image,int erosion_size)
{
Mat aux=image.clone();
Mat element = getStructuringElement( MORPH_ELLIPSE,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

  /// Apply the erosion operation
  dilate( aux, image, element );
}
 void mexic(Mat& mexicanFilter,Mat& image ,int frame,int *numpersonas)
{
double minValg,maxValg;
   minMaxLoc(image, &minValg, &maxValg);
     int floor = maxValg*0.6;
     for (int i=0; i<image.rows; i++)
     {
         for(int j=0; j<image.cols; j++)
         {
             if (image.at<uchar>(i,j)> floor)
             {
               image.at<uchar>(i,j)=maxValg;
             }
         }
     }
image.convertTo(image, CV_8U, 255.0/(maxValg - minValg), -minValg * 255.0/(maxValg - minValg));
//erosion(mexicanFilter,11);
if(argumentosmexic)
{
imshow("imagen entrada",mexicanFilter);
imshow("frame 8 bits",image);
}
 int i, j;
      double minVal; double maxVal; Point minLoc; Point maxLoc;
    ///Umbralización de la imagen del sombrero mexicano
     Mat threshold_output,filtro,canny;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

     int thresh = 50;
threshold( mexicanFilter, filtro, thresh, 255, 0);
filtro=filtro+image;
threshold( filtro,filtro, 200, 255, 4);
     threshold( mexicanFilter, threshold_output, thresh, 255, 1);
Canny(threshold_output,canny,100,200,3);
findContours(canny,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_NONE,Point(0,0));

vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  vector<Point2f>center( contours.size() );
  vector<float>radius( contours.size() );
vector<Mat> Rois( contours.size() );
double minim[contours.size()],maxim[contours.size()];
int maxx[contours.size()],maxy[contours.size()],minx[contours.size()],miny[contours.size()];
cv::Point mini,maxi;
double a,b;
Mat auxiliarcolor;
cvtColor(image,auxiliarcolor,CV_GRAY2BGR);
  for( int i = 0; i < contours.size(); i++ )
     { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
       boundRect[i] = boundingRect( Mat(contours_poly[i]) );

     }
cvtColor(filtro, filtro, CV_GRAY2RGB);
*numpersonas=contours.size()/2;
  for( int i = 0; i< contours.size(); i=i+2)
     {
       rectangle( filtro, boundRect[i].tl(), boundRect[i].br(), Scalar( 255 ), 2, 8, 0 );
Rois[i] = image(boundRect[i]);
minMaxLoc(Rois[i],&minim[i],&maxim[i],&mini,&maxi, Mat() );

minx[i]=boundRect[i].tl().x+mini.x;
miny[i]=boundRect[i].tl().y+mini.y;



circle(filtro,
                             Point(minx[i],miny[i]),
                             2, Scalar(0,0,255), -1, 8, 0);
circle(Rois[i],
                             mini,
                             2, Scalar(0), -1, 8, 0);
circle(auxiliarcolor, Point(minx[i],miny[i]),2, Scalar(0,255,0), -1, 8, 0);
if(visualizarois)
{
imshow("1h",Rois[i]);
waitKey();
}
     }
  char name[100] = "seq-P01-M02-A0032-G00-C00-S0036.result";
  FILE *fp;
  //chapuza para poder meter las posiciones de los máximos en la estructura
  Point aux[] = {Point(0,0),Point(0,0),Point(0,0),Point(0,0),Point(0,0),Point(0,0),Point(0,0),Point(0,0),Point(0,0),Point(0,0)};
   for( int i = 0; i< contours.size()/2; i++)aux[i] = Point(minx[i],miny[i]);
  if ((fp = fopen(name, "at")) == NULL)
    {
         cout << "[ERROR] File " << name << " not open" << endl;
    }
  else
      //fwrite(&Results, sizeof(st_resultSave), 1, fp);
      fprintf(fp,"%06ld", (long int)frame);
  for( int i = 0; i< contours.size()/2; i++)
 {
  if(aux[i].x>0 && aux[i].x<500 && aux[i].y>0 && aux[i].y<500)
  {
    fprintf(fp, " %d %4d %4d %4f ", (int)1, aux[i].x, aux[i].y,0);
  }
 }
    fprintf(fp, "\n");
  fclose(fp);
mexResult("RESULTADO.result",frame,minx,miny,contours.size());
 //imshow("1h",image);


      imshow("resultadofinal",auxiliarcolor);//Para ver resultado en imagen sin roi y cuadrado
      imshow("1d",filtro);
         //imshow("1c",threshold_output);
// imshow("canny",canny);
mexicanFilter=threshold_output;
printf("numero de personas=%d",contours.size()/2);
}
 void mexicanMaximos(Mat& mexicanFilter, Mat& Imprepro )
{
     int i, j;
      double minVal; double maxVal; Point minLoc; Point maxLoc;
    ///Umbralización de la imagen del sombrero mexicano
     Mat threshold_output;
     int thresh = 165;
     threshold( mexicanFilter, threshold_output, thresh, 255, THRESH_BINARY );
     if (MEXICAN_CONTOUR_RESULT)
         imshow("1c",threshold_output);


     ///Cálculo de los contornos de imagen
     struct_count contMex = Calculo_contorno(threshold_output, Imprepro);
      vector<Point2f>maxPoints_array(contMex.contours.size());

    ///Análisis de los máximos de la imagen
     Mat ZmatColor;
     Mat mexColor;
     data_ROI contRoi;
     Point2f maxPoint;
     cvtColor(Imprepro, ZmatColor, CV_GRAY2RGB);
     cvtColor(mexicanFilter, mexColor, CV_GRAY2RGB);
     Mat Imprepro_aux = Imprepro.clone();
     minMaxLoc(ZmatColor, &minValLocal, &maxValLocal);
     ZmatColor.convertTo(ZmatColor, CV_8U, 255.0/(maxValLocal - minValLocal), -minValLocal * 255.0/(maxValLocal - minValLocal));
     minMaxLoc(Imprepro, &minValLocal, &maxValLocal);

     ///Análisis contorno a contorno, eliminando el resto y analizándolos individualmente calculando el máximo
     for (i=0; i< contMex.contours.size(); i++)
     {
        //tengo que conseguir que tape todos los contornos menos el que evaluo ¿un for que coja todos menos el que estoy usando ahora?????
        Imprepro_aux = Imprepro.clone();

    ///pegando el contorno que anlizo en una imagen sola
    /* lo está escribiendo solo en la esquina hay que echar la cuenta para que busque el máximo en el lugar adecuado de la imagen
        contRoi = cutROI(Imprepro,contMex.center_count[j], contMex.radius_count[j] + 20 );
        imshow("ROI para maximo", contRoi.ROI);
        maxPoint =  maxPointMat(contRoi.ROI);
    */


    ///borrando el resto de contornos menos el que analizo
        for (j=0; j< contMex.contours.size(); j++)
         {
            if (j != i)
             {

                 //cout << "Datos para la ROI --> " <<contMex.center_count[j].x-contMex.radius_count[j] << " " <<contMex.center_count[j].y-contMex.radius_count[j] << " " << contMex.radius_count[j]*2<< endl;
                 contRoi = cutROI(Imprepro,contMex.center_count[j], contMex.radius_count[j] + 27 );
                 Imprepro_aux(Rect(contRoi.rect_start.x ,contRoi.rect_start.y, contRoi.rect_add.x, contRoi.rect_add.y))= minValLocal;

                 if (MEXICAN_CONTOUR_RESULT)
                 {
                     Mat Imprepro_aux_SHOW= Imprepro_aux.clone();
                     minMaxLoc(Imprepro_aux_SHOW, &minVal, &maxVal);
                     Imprepro_aux_SHOW.convertTo(Imprepro_aux_SHOW, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
                     imshow("quitandoROIS", Imprepro_aux_SHOW);
                     waitKey();
                 }
             }
         }


         imshow("quitando ROIS", Imprepro_aux);


         minMaxLoc( Imprepro_aux, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
         maxPoints_array[j] = maxLoc;

         circle( ZmatColor, maxLoc, 7, Scalar(255,0,0), -1, 8, 0 );
         circle( mexColor, maxLoc, 7, Scalar(255,0,0), -1, 8, 0 );
         
             imshow("ZmatColor",ZmatColor);
             imshow("mexColor",mexColor);
         

     }

     if (MEXICAN_CONTOUR_RESULT)
        imshow("ZmatColor",ZmatColor);
     cout << "numero de contornos = " << contMex.contours.size() << endl;
     cout <<"--fin"<<endl;

     waitKey();
        //  return maxPoints_array;

}
