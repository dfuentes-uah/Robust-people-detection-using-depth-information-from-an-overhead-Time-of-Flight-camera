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


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include "histogramasec.h"
#define VISUALIZACION 1
#define AUTOTRAIN 0
#define TESTSVM 1
#define ENTRSVM 0
#define UNACLASE 2
#define SOMBRERO 1
#define NEGATIVO 1
#define DOSCLASES 0
using namespace cv;
int clasificasdos(double** persona,int npersona,double** objeto,int nobjeto,double* candidato,Mat Imageout,Point position)
{
    float personados[npersona][2];
     float objetodos[nobjeto][2];
    Mat imageaux;
  int hist_w = 500; int hist_h = 500;
    Mat histImage( hist_h, hist_w, CV_8UC3,Scalar( 0,0,0) );
          for(int k=0;k<npersona;k++)
  {

    histogramasec( persona[k], &histImage,Scalar(255,0,0));

for(int i=0;i<NUMCARAC-4;i++)
{
  //cout<<"vector"<<vectores[k][i]<<endl;
  personados[k][i]=persona[k][i]*100;
 // cout <<"personados"<<personados[k][i]<<endl;
}
  }
          for(int k=0;k<nobjeto;k++)
  {

    histogramasec( objeto[k], &histImage,Scalar(255,0,255));
    for(int i=0;i<NUMCARAC-4;i++)
    {
      //cout<<"vector"<<vectores[k][i]<<endl;
      objetodos[k][i]=objeto[k][i]*100;
      //cout <<"objetodos"<<objetodos[k][i]<<endl;
    }
}
if (VISUALIZACION==1)
{
              imshow("hist",histImage);
}
              float result[npersona+nobjeto][2];


              Mat labelsMat=Mat::ones((npersona+nobjeto),1,CV_32FC1);
              for(long int k=0;k<(npersona+nobjeto);k++)
              {
    //              cout <<"personados"<<result[k][0]<<endl;
                  if(k<=nobjeto)
                  {
                     labelsMat.at<float>(k,1)=2;
                      result[k][0]=objetodos[k][0];
                      result[k][1]=objetodos[k][1];

                  }
                   if(k>nobjeto & k<=(nobjeto+npersona))
                   {

                      labelsMat.at<float>(k,1)=1;
                       result[k][0]=personados[k-nobjeto][0];
                        result[k][1]=personados[k-nobjeto][1];
                   }

              }
              Mat trainingDataMat=Mat((npersona+nobjeto), NUMCARAC-4, CV_32FC1,result);

              CvSVM svm;
              CvSVMParams params;
              // Set up SVM's parameters
              params.svm_type    = CvSVM::C_SVC;//CvSVM::C_SVC;
              params.kernel_type = CvSVM::RBF;
              params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
              params.nu=0.05;//nu;
              params.gamma=0.00015;//gamma;
              params.coef0=0.1;//coef;
              params.C=0.5;
              params.degree=2;//coef;
              // Train the SVM
              svm.train(trainingDataMat, labelsMat, Mat(), Mat(), params);
              int c=svm.get_support_vector_count();
              Mat support=Mat( 500, 500, CV_8UC3,Scalar( 0,0,0) );
                          for (int i = 0; i < c; ++i)
                          {
                              const float* v = svm.get_support_vector(i);
                              cout <<"valor " << v[0] <<" " <<v[1]<<endl;
 circle( support,  Point( (int) v[0], (int) v[1]),   6,  Scalar(128, 128, 128), -1, 8);
                          }
                             Vec3b green(0,255,0), blue (255,0,0);
                          for (int i = 0; i < support.rows; ++i)
                                  for (int j = 0; j < support.cols; ++j)
                                  {
                                      Mat sampleMat = (Mat_<float>(1,2) << j,i);
                                      float response = svm.predict(sampleMat);

                                      if (response == 1)
                                          support.at<Vec3b>(i,j)  = green;
                                      else if (response == 2)
                                           support.at<Vec3b>(i,j)  = blue;
                                  }
                          if (VISUALIZACION==1)
                          {
                          imshow("support",support);
                          waitKey();
                          }
}

int clasificasvm(double** persona,int npersona,double** objeto,int nobjeto,double** sombrero,int nsombrero,double* candidato,Mat *Imageout,Point position,Point subroipos,int *resultado,int mn)
{
    float persona2[npersona][NUMCARAC];
    float objeto2[npersona][NUMCARAC];
    float candidato2[NUMCARAC];
    float result2[(npersona+nobjeto+nsombrero)][6];
    float result[(npersona+nobjeto)][NUMCARAC];
      Mat imageaux;
    int hist_w = 500; int hist_h = 500;
      Mat histImage( hist_h, hist_w, CV_8UC3,Scalar( 0,0,0) );
            for(int k=0;k<npersona;k++)
    {
                if (VISUALIZACION==1)
                {
      histogramasec( persona[k], &histImage,Scalar(255,0,0));
                }
for(int i=0;i<NUMCARAC;i++)
{
    //cout<<"vector"<<vectores[k][i]<<endl;
    persona[k][i]=persona[k][i]*100;
}
    }
            for(int k=0;k<nobjeto;k++)
    {
                if (VISUALIZACION==1)
                {
      histogramasec( objeto[k], &histImage,Scalar(255,0,255));
                }

      for(int i=0;i<NUMCARAC;i++)
      {
          //cout<<"vector"<<vectores[k][i]<<endl;
          objeto[k][i]=objeto[k][i]*100;
      }
}
            for(int k=0;k<nsombrero;k++)
    {
                if (VISUALIZACION==1)
                {
      histogramasec( sombrero[k], &histImage,Scalar(255,255,255));
                }
      for(int i=0;i<NUMCARAC;i++)
      {
          //cout<<"vector"<<vectores[k][i]<<endl;
          sombrero[k][i]=sombrero[k][i]*100;
      }
}
            if (VISUALIZACION==1)
            {
                imshow("hist",histImage);
            }
 ////////////////////           ////////////////////////////////
            if (VISUALIZACION==1)
            {
      histogramasec( candidato, &histImage,Scalar(0,0,255));
            }
for(int i=0;i<NUMCARAC;i++)
{
    candidato2[i]=candidato[i]*100;
}
///////////////////////            ////////////////////////////
if (VISUALIZACION==1)
{
imshow("hist",histImage);
}
Mat trainingDataMat;
Mat labelsMat;
if(DOSCLASES==1)
{
    labelsMat=Mat::ones((npersona+nobjeto),1,CV_32FC1);
    for(long int k=0;k<(npersona+nobjeto);k++)
    {

        if(k<nobjeto)
        {
           labelsMat.at<float>(k,1)=2;
for(int i=0;i<NUMCARAC;i++)result[k][i]=objeto[k][i];



  //           cout <<"personados"<<result[k][0]<<endl;


        }
         if(k>=nobjeto & k<=(nobjeto+npersona))
         {

            labelsMat.at<float>(k,1)=1;
for(int i=0;i<NUMCARAC;i++)result[k][i]=persona[k-nobjeto][i];
//cout <<"personados"<<result[k][0]<<endl;
         }

    }
trainingDataMat=Mat((npersona+nobjeto), NUMCARAC, CV_32FC1,result);
}
if(DOSCLASES==0)
{
    labelsMat=Mat::ones((npersona+nobjeto+nsombrero),1,CV_32FC1);
    for(long int k=0;k<(npersona+nobjeto+nsombrero);k++)
    {

        if(k<nobjeto)
        {
           labelsMat.at<float>(k,1)=2;
    for(int i=0;i<NUMCARAC;i++)result2[k][i]=objeto[k][i];
             //cout <<"personados"<<result2[k][0]<<endl;


        }
         if(k>=nobjeto & k<(nobjeto+npersona))
         {

            labelsMat.at<float>(k,1)=1;
for(int i=0;i<NUMCARAC;i++)result2[k][i]=persona[k-nobjeto][i];
//cout <<"personados2"<<result2[k][0]<<endl;
         }
         if(k>=(nobjeto+npersona) & k<=(nobjeto+npersona+nsombrero))
         {
             labelsMat.at<float>(k,1)=3;
             for(int i=0;i<NUMCARAC;i++)result2[k][i]=sombrero[k-nobjeto-npersona][i];
  //           cout <<"personados3"<<result2[k][0]<<endl;
         }

    }
    trainingDataMat=Mat((npersona+nobjeto+nsombrero), NUMCARAC, CV_32FC1,result2);
}
  /* Mat labelsMat=Mat::ones(numvectores,1,CV_32FC1);
 Mat trainingDataMat(numvectores, NUMCARAC, CV_32FC1, vectores);*/
CvSVM svm;
CvSVMParams params;
// Set up SVM's parameters
params.svm_type    = CvSVM::C_SVC;//CvSVM::C_SVC;
params.kernel_type = CvSVM::RBF;
params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
params.nu=0;//nu;
params.gamma=0.00225;//VALOR IDEAL 2 CLASES
params.coef0=0.1;//coef;
params.C=2.5;
params.degree=2;//coef;
// Train the SVM
svm.train(trainingDataMat, labelsMat, Mat(), Mat(), params);
int ceros=0,unos=0;
Mat sampleMat(1,NUMCARAC,CV_32FC1,candidato2);
float response = svm.predict(sampleMat);
if(response==2)
{
    cout << "NO PERSONA\n";
    int auxy=position.y*20+subroipos.y;
    int auxx=((position.x*20)+subroipos.x);
    //cout << "PERSONA\n";
     //cout << "maximo x"<< auxx <<"maximo y" << auxy <<"\n";

     //cvtColor(Imageout,imageaux,CV_GRAY2RGB);
    if(VISUALIZACION==1)
    {
     circle(*Imageout,Point(auxx,auxy),10,Scalar(0,0,255),-1,8,0);

     imshow("resultados",*Imageout);
    }
     resultado[mn]=1;
}
if(response==1 | response==3)
{
    int auxy=position.y*20+subroipos.y;
    int auxx=((position.x*20)+subroipos.x);
    //cout << "PERSONA\n";
    // cout << "maximo x"<< auxx <<"maximo y" << auxy <<"\n";

     //cvtColor(Imageout,imageaux,CV_GRAY2RGB);
    if(VISUALIZACION==1)
    {
     circle(*Imageout,Point(auxx,auxy),10,Scalar(0,255,0),-1,8,0);

     //if(response==3)circle(imageaux,Point(auxx,auxy),5,Scalar(255,0,0),-1,8,0);
     imshow("resultados",*Imageout);
    }
     resultado[mn]=0;
}

waitKey(1);
}

int entrenasvm(double** sombrero,int nsombrero,double** persona,int npersona,double** objeto,int nobjeto)
{
    float result[(npersona+nobjeto)][6];
     float result2[(npersona+nobjeto+nsombrero)][6];
    int hist_w = 500; int hist_h = 500;
      Mat histImage( hist_h, hist_w, CV_8UC3,Scalar( 0,0,0) );
      if(SOMBRERO==1)
      {
            for(int k=0;k<nsombrero;k++)
    {

      //histogramasec( sombrero[k], &histImage,Scalar(255,0,0));

for(int i=0;i<NUMCARAC;i++)
{
    //cout<<"vector"<<vectores[k][i]<<endl;
    sombrero[k][i]=sombrero[k][i]*100;
}
    }
           printf("ha llegado");
            imshow("hist",histImage);
            waitKey();
      }
 ////////////////////           ////////////////////////////////
            for(int k=0;k<npersona;k++)
    {

      //histogramasec( persona[k], &histImage,Scalar(0,255,0));

for(int i=0;i<NUMCARAC;i++)
{
   // cout<<"vector"<<vectorestest[k][i]<<endl;
    persona[k][i]=persona[k][i]*100;
}
    }
            ///////////////////////            ////////////////////////////
imshow("hist",histImage);
waitKey();
////////////////////           ////////////////////////////////
if(NEGATIVO==1)
{
           for(int k=0;k<nobjeto;k++)
   {

     histogramasec( objeto[k], &histImage,Scalar(0,255,255));

for(int i=0;i<NUMCARAC;i++)
{
  // cout<<"vector"<<vectorestest[k][i]<<endl;
   objeto[k][i]=objeto[k][i]*100;
}
   }
           ///////////////////////            ////////////////////////////
imshow("hist",histImage);
waitKey();
}
if(TESTSVM==1)
{


    if(AUTOTRAIN==1)
    {
        CvSVM svm;
        CvSVMParams paramz;
        paramz.kernel_type = CvSVM::RBF;
        paramz.svm_type = CvSVM::C_SVC;
        paramz.term_crit = cvTermCriteria(CV_TERMCRIT_ITER,100,0.000001);
        paramz.degree=2;
        Mat trainingDataMat;
  Mat labelsMat;
        if(DOSCLASES==1)
        {

            labelsMat=Mat::ones((npersona+nobjeto),1,CV_32FC1);
            for(long int k=0;k<(npersona+nobjeto);k++)
            {

                if(k<nobjeto)
                {
                   labelsMat.at<float>(k,1)=2;
                   for(int i=0;i<NUMCARAC;i++)result[k][i]=objeto[k][i];
                     cout <<"personados"<<result[k][0]<<endl;


                }
                 if(k>=nobjeto & k<=(nobjeto+npersona))
                 {

                    labelsMat.at<float>(k,1)=1;
                 for(int i=0;i<NUMCARAC;i++)result[k][i]=persona[k-nobjeto][i];
                   cout <<"personados"<<result[k][0]<<endl;
                 }

            }
            trainingDataMat=Mat((npersona+nobjeto), NUMCARAC, CV_32FC1,result);
        }
        if(DOSCLASES==0)
        {
            labelsMat=Mat::ones((npersona+nobjeto+nsombrero),1,CV_32FC1);
            for(long int k=0;k<(npersona+nobjeto+nsombrero);k++)
            {

                if(k<nobjeto)
                {
                   labelsMat.at<float>(k,1)=2;
            for(int i=0;i<NUMCARAC;i++)result2[k][i]=objeto[k][i];
                     cout <<"personados"<<result2[k][0]<<endl;


                }
                 if(k>=nobjeto & k<(nobjeto+npersona))
                 {

                    labelsMat.at<float>(k,1)=1;
for(int i=0;i<NUMCARAC;i++)result2[k][i]=persona[k-nobjeto][i];
cout <<"personados2"<<result2[k][0]<<endl;
                 }
                 if(k>=(nobjeto+npersona) & k<=(nobjeto+npersona+nsombrero))
                 {
                     labelsMat.at<float>(k,1)=3;
                     for(int i=0;i<NUMCARAC;i++)result2[k][i]=sombrero[k-nobjeto-npersona][i];
                     cout <<"personados3"<<result2[k][0]<<endl;
                 }

            }
            trainingDataMat=Mat((npersona+nobjeto+nsombrero), NUMCARAC, CV_32FC1,result2);
        }


    cout<<"CREADO"<<endl;
//(min_val, min_val*step, min_val*(step)^2, dots, min_val*(step)^n),
        CvParamGrid CvParamGrid_C(pow(2.0,-5), pow(2.0,15), pow(2.0,2));
          CvParamGrid CvParamGrid_gamma(pow(2.0,-15), pow(2.0,3), pow(2.0,2));
          CvParamGrid CvParamGrid_nu(pow(1,-2), pow(1,1), 0.05);

          if (!CvParamGrid_C.check() || !CvParamGrid_gamma.check())
              cout<<"The grid is NOT VALID."<<endl;

         svm.train_auto(trainingDataMat, labelsMat, Mat(), Mat(), paramz,2, CvSVM::get_default_grid(CvSVM::C), CvSVM::get_default_grid(CvSVM::GAMMA), CvSVM::get_default_grid(CvSVM::P), CvSVM::get_default_grid(CvSVM::NU), CvSVM::get_default_grid(CvSVM::COEF), CvSVM::get_default_grid(CvSVM::DEGREE), true);
          paramz=svm.get_params();
          cout<<"gamma:"<<paramz.gamma<<endl;
          cout<<"C:"<<paramz.C<<endl;
          cout<<"nu:"<<paramz.nu<<endl;
          cout<<"coef:"<<paramz.coef0<<endl;
          cout<<"p:"<<paramz.p<<endl;
        int c = svm.get_support_vector_count();

        int hist_w = 500; int hist_h = 500;
          Mat histImage2( hist_h, hist_w, CV_8UC3,Scalar( 0,0,0) );
            for (int i = 0; i < c; ++i)
            {
                const float* v = svm.get_support_vector(i);
                double vv[6];
                //cout <<"valor " << v[0] <<" " <<v[1]<<" "<<v[2]<<" "<<v[3]<<endl;
                for(int j=0;j<NUMCARAC;j++)vv[j]=v[j]/100;
                      histogramasec(vv , &histImage2,Scalar(255,255,0));
            }
        cout<<"support vector"<<c<<endl;

        imshow("hist",histImage2);
      waitKey();
    }

}

///////////////////////////////



waitKey();
}

