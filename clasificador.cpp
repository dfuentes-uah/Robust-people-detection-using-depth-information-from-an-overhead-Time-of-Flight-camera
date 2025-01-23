
#include <iostream>

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







#include "Funciones_opencv.h"

//#include "geom_forms.h"
//#include "geom_filters.h"
#include "defines.h"

//#include "geom_forms.h"
//#include "geom_filters.h"

double minValSec_d, maxValSec_d;


int* heighNumPoints(Mat& original, int *pointsCarac)
{
    int i,j, x;

    minMaxLoc(original, &minValSec_d, &maxValSec_d);
    cout << "maxValorde la imagen = "  << minValSec_d<< " - " << maxValSec_d << endl;

    uint16_t increment2cm = 20;
    uint16_t increment6cm = 60;
    uint16_t minValSec = (uint16_t)minValSec_d;
    uint16_t maxValSec = (uint16_t)minValSec_d;

    uint16_t measures6cm[9] = {minValSec, minValSec+1*increment6cm, minValSec+2*increment6cm, minValSec+3*increment6cm, minValSec+4*increment6cm, minValSec+5*increment6cm, minValSec+6*increment6cm, minValSec+7*increment6cm, minValSec+8*increment6cm};
    uint16_t measures2cm[19] = {minValSec, minValSec+1*increment2cm, minValSec+2*increment2cm, minValSec+3*increment2cm, minValSec+4*increment2cm, minValSec+5*increment2cm, minValSec+6*increment2cm, minValSec+7*increment2cm, minValSec+8*increment2cm, minValSec+9*increment2cm, minValSec+10*increment2cm, minValSec+11*increment2cm, minValSec+12*increment2cm, minValSec+13*increment2cm, minValSec+14*increment2cm, minValSec+15*increment2cm, minValSec+16*increment2cm, minValSec+17*increment2cm, minValSec+18*increment2cm};


    pointsCarac = new int[6];
    int *slice2Cm = new int[19];

    int maxSlicePoint_head = 0;
    int indSlicePoint_head;

    int maxSlicePoint_shoulder = 0;
    int indSlicePoint_shoulder;

    for (x=0; x<6; x++) pointsCarac[x] = 0;
    for (x=0; x<19; x++) slice2Cm[x] = 0;

  
    ///DIVISIÓN DE LA IMAGEN EN RODAJAS DE 2 CM

    for (i =0 ; i< original.rows ; i++)
    {
        for (j=0; j<original.cols; j++)
        {
            if (original.at<unsigned short>(i,j) >= measures2cm[0] && original.at<unsigned short>(i,j) < measures2cm[1])
                slice2Cm[0] = slice2Cm[0] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[1]  && original.at<unsigned short>(i,j) < measures2cm[2] )
                slice2Cm[1] = slice2Cm[1] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[2]  && original.at<unsigned short>(i,j) < measures2cm[3] )
                slice2Cm[2] = slice2Cm[2] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[3]  && original.at<unsigned short>(i,j) < measures2cm[4] )
                slice2Cm[3] = slice2Cm[3] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[4]  && original.at<unsigned short>(i,j) < measures2cm[5] )
                slice2Cm[4] = slice2Cm[4] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[5]  && original.at<unsigned short>(i,j) < measures2cm[6] )
                slice2Cm[5] = slice2Cm[5] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[6]  && original.at<unsigned short>(i,j) < measures2cm[7] )
                slice2Cm[6] = slice2Cm[6] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[7]  && original.at<unsigned short>(i,j) < measures2cm[8] )
                slice2Cm[7] = slice2Cm[7] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[8]  && original.at<unsigned short>(i,j) < measures2cm[9] )
                slice2Cm[8] = slice2Cm[8] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[9]  && original.at<unsigned short>(i,j) < measures2cm[10] )
                slice2Cm[9] = slice2Cm[9] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[10]  && original.at<unsigned short>(i,j) < measures2cm[11] )
                slice2Cm[10] = slice2Cm[10] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[11]  && original.at<unsigned short>(i,j) < measures2cm[12] )
                slice2Cm[11] = slice2Cm[11] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[12]  && original.at<unsigned short>(i,j) < measures2cm[13] )
                slice2Cm[12] = slice2Cm[12] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[13]  && original.at<unsigned short>(i,j) < measures2cm[14] )
                slice2Cm[13] = slice2Cm[13] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[14]  && original.at<unsigned short>(i,j) < measures2cm[15] )
                slice2Cm[14] = slice2Cm[14] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[15]  && original.at<unsigned short>(i,j) < measures2cm[16] )
                slice2Cm[15] = slice2Cm[15] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[16]  && original.at<unsigned short>(i,j) < measures2cm[17] )
                slice2Cm[16] = slice2Cm[16] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[17]  && original.at<unsigned short>(i,j) < measures2cm[18] )
                slice2Cm[17] = slice2Cm[17] + 1;
            else if (original.at<unsigned short>(i,j) >= measures2cm[18]  && original.at<unsigned short>(i,j) < measures2cm[19] )
                slice2Cm[18] = slice2Cm[18] + 1;


        }
    }

    /*
    cout << "[INFO] Escalas"<< endl;
    cout << measures2cm[0] << "-" << measures2cm[1] << " = " << slice2Cm[0] << endl;
    cout << measures2cm[1] << "-" << measures2cm[2] << " = " << slice2Cm[1] << endl;
    cout << measures2cm[2] << "-" << measures2cm[3] << " = " << slice2Cm[2] << endl;
    cout << measures2cm[3] << "-" << measures2cm[4] << " = " << slice2Cm[3] << endl;
    cout << measures2cm[4] << "-" << measures2cm[5] << " = " << slice2Cm[4] << endl;
    cout << measures2cm[5] << "-" << measures2cm[6] << " = " << slice2Cm[5] << endl;
    cout << measures2cm[6] << "-" << measures2cm[7] << " = " << slice2Cm[6] << endl;
    cout << measures2cm[7] << "-" << measures2cm[8] << " = " << slice2Cm[7] << endl;
    */
    

    ///BUSQUEDA ENTRE LAS 3 PRIMERAS DE LA RODAJA CON MÁS PUNTOS

    for (i=0; i<3; i++)
    {
        //cout << "bucle 1 => i = " << i << endl;
        if (maxSlicePoint_head < pointsCarac[i])
        {
           maxSlicePoint_head = pointsCarac[i];
           indSlicePoint_head = i;

        }
    }


    //cout <<"indSlicePoint_head = " << indSlicePoint_head << endl;

    ///ASIGNACIÓN DE LAS RODAJAS DE 2CM A LOS VECTORES 0 1 2

    if (indSlicePoint_head == 0)
    {
        pointsCarac[0]= slice2Cm[indSlicePoint_head] + slice2Cm[indSlicePoint_head+1] + slice2Cm[indSlicePoint_head+2];
        pointsCarac[1]= slice2Cm[indSlicePoint_head+3] + slice2Cm[indSlicePoint_head+4] + slice2Cm[indSlicePoint_head+5];
        pointsCarac[2]= slice2Cm[indSlicePoint_head+6] + slice2Cm[indSlicePoint_head+7] + slice2Cm[indSlicePoint_head+8];
    }
    else if (indSlicePoint_head > 0)
    {
        indSlicePoint_head = indSlicePoint_head -1;
        pointsCarac[0]= slice2Cm[indSlicePoint_head] + slice2Cm[indSlicePoint_head+1] + slice2Cm[indSlicePoint_head+2];
        pointsCarac[1]= slice2Cm[indSlicePoint_head+3] + slice2Cm[indSlicePoint_head+4] + slice2Cm[indSlicePoint_head+5];
        pointsCarac[2]= slice2Cm[indSlicePoint_head+6] + slice2Cm[indSlicePoint_head+7] + slice2Cm[indSlicePoint_head+8];
    }


    ///BUSQUEDA DE LAS RODAJAS DE LOS HOMBROS

    for (i= (indSlicePoint_head+9); i< (indSlicePoint_head+12); i++)
    {
        //cout << "bucle 2 => i = " << i << endl;
        if (maxSlicePoint_shoulder < pointsCarac[i])
        {
           maxSlicePoint_shoulder = pointsCarac[i];
           indSlicePoint_shoulder = i;

        }
    }


    ///ASIGNACIÓN DE LAS RODAJAS DE 2CM A LOS VECTORES 3 4 5

    if ((indSlicePoint_shoulder - (indSlicePoint_head+9)) == 0)
    {
        pointsCarac[3]= slice2Cm[indSlicePoint_head] + slice2Cm[indSlicePoint_head+1] + slice2Cm[indSlicePoint_head+2];
        pointsCarac[4]= slice2Cm[indSlicePoint_head+3] + slice2Cm[indSlicePoint_head+4] + slice2Cm[indSlicePoint_head+5];
        pointsCarac[5]= slice2Cm[indSlicePoint_head+6] + slice2Cm[indSlicePoint_head+7] + slice2Cm[indSlicePoint_head+8];
    }
    else if ((indSlicePoint_shoulder - (indSlicePoint_head+9)) > 0)
    {
        indSlicePoint_head = indSlicePoint_head -1;
        pointsCarac[3]= slice2Cm[indSlicePoint_head] + slice2Cm[indSlicePoint_head+1] + slice2Cm[indSlicePoint_head+2];
        pointsCarac[4]= slice2Cm[indSlicePoint_head+3] + slice2Cm[indSlicePoint_head+4] + slice2Cm[indSlicePoint_head+5];
        pointsCarac[5]= slice2Cm[indSlicePoint_head+6] + slice2Cm[indSlicePoint_head+7] + slice2Cm[indSlicePoint_head+8];
    }

    return pointsCarac;
}





float mahalanovisDist(Mat actual,Mat& mediaVector, Mat& covariance, int numCarac)
{
    Mat coef1 = Mat::zeros(1, numCarac, CV_64FC1);
    Mat coef2 = Mat::zeros(numCarac, 1, CV_64FC1);
    Mat multi1 = Mat::zeros(1, numCarac, CV_64FC1);
    Mat multi2  = Mat::zeros(1, 1, CV_64FC1);
    Mat covarianceInv  = Mat::zeros(numCarac, numCarac, CV_64FC1);

    covarianceInv = covariance.inv();

    Mat aux= covariance.clone().reshape(1,1);
    cout <<"valores del vector actual de mahalovis " << endl;

    for (int i=0; i<numCarac; i++)
    {
        cout << actual.at<double>(0,i) << endl;
        coef1.at<double>(0,i) = actual.at<double>(0,i)- mediaVector.at<double>(0,i);
        coef2.at<double>(i,0) = actual.at<double>(0,i)- mediaVector.at<double>(0,i);
    }


    multi1 = coef1* covarianceInv;


    multi2 = multi1*coef2;


    float distancia = sqrt(multi2.at<double>(0,0));

    cout <<"resultado  = " << distancia << endl;
    return distancia;

}

double PCAclasification (Mat samples, Mat actualCol)
{
    //cout << "samples = " << samples << endl;
    Mat actualRow = actualCol.t();

    PCA pca(samples, Mat(), CV_PCA_DATA_AS_ROW, 3);
    Mat mean = pca.mean.clone();
    Mat eigenvalues = pca.eigenvalues.clone();
    Mat eigenvectors = pca.eigenvectors.clone();

    //cout << "Mean = " << mean << endl;
    //cout << "eigenvalues = " << eigenvalues << endl;

    //cout << "eigenvectors = " << eigenvectors << endl;
    //cout << endl;

    Mat imProject = pca.project(actualRow);

    Mat imReconstruct = pca.backProject(imProject);

    //cout << "vector = " << actualRow << endl;
    //cout << "imReconstruct = " << imReconstruct << endl;
    //cout << "diference = " ;

    Mat difference = Mat::zeros(1, NUMCARAC, CV_32FC1);
    for (int i=0; i<NUMCARAC; i++)
        difference.at<float>(0,i) = actualRow.at<float>(0,i) - imReconstruct.at<float>(0,i);


    double normDifference = 0;

    for(int p=0;p<NUMCARAC; p++)
        normDifference = normDifference + (difference.at<float>(0,p)*difference.at<float>(0,p));
    normDifference = sqrt(normDifference);
    //cout <<"normDifference = " << normDifference << endl;
    //waitKey();
    return normDifference;
}


double minValue (double a, double b)
{
   if (a < b)return a;
   else return b;
}




