
#include <stdio.h>
#include <stdlib.h>
//#include <pmdsdk2.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>

#include <stdarg.h>

#define scal_altura_aux 0.76

#define IMAGES_VISUAL_FILTROS 0
#define IMAGES_VISUAL_MASK 0
#define IMAGE_CONTOUR 0
  

using namespace cv;
using namespace std;

typedef struct
{   int cont_ok;
    cv::Mat mat_contornos;
    vector<vector<Point> > contours;
    vector<Point2f>center_count;
    vector<float>radius_count;

}struct_count;

typedef struct
{   cv::Mat mat_contorno;
    float radio;
    float altura_cont;

}datos_contorno;


typedef struct
{
    Mat ROI;
    Point2f rect_start;
    Point2f rect_add;

}data_ROI;


 char name_image[100];
 double minVal, maxVal;



 ///==========================================================================///
 ///                            CALCULO HISTOGRAMA                            ///
 ///                                                                          ///
 ///==========================================================================///

void Calculo_histograma (cv::Mat& mat, cv::Mat& mask, char* nombre)
{

    // Initialize parameters
        int histSize = 256;    // bin size
        float range[] = { 0, 255 };
        const float *ranges[] = { range };

        // Calculate histogram
        MatND hist;
        calcHist( &mat, 1, 0, mask, hist, 1, &histSize, ranges, true, false );

        // Show the calculated histogram in command window
        double total;
        total = mat.rows * mat.cols;

///        for( int h = 0; h < histSize; h++ )
///             {
///                float binVal = hist.at<float>(h);
///                cout<<" "<<binVal;
///             }

        // Plot the histogram
        int hist_w = 512; int hist_h = 400;
        int bin_w = cvRound( (double) hist_w/histSize );

        Mat histImage( hist_h, hist_w, CV_8UC1, Scalar( 0,0,0) );
        normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

        for( int i = 1; i < histSize; i++ )
        {
          line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
                           Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
                           Scalar( 255, 0, 0), 2, 8, 0  );
        }

        namedWindow( "Result", 1 );    imshow( "Result", histImage );

       // waitKey(0);


}






///==========================================================================///
///                 CALCULO AUTOVALORES Y AUVECTORES PCA                     ///
///                                                                          ///
///==========================================================================///
double getOrientation(vector<Point> &pts, Mat &img)
{
    //Construct a buffer used by the pca analysis
    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

    //Store the position of the object
    Point pos = Point(pca_analysis.mean.at<double>(0, 0),
                      pca_analysis.mean.at<double>(0, 1));

    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));

        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }


    Mat color_image;

    cvtColor(img, color_image, CV_GRAY2RGB);

    applyColorMap(color_image, color_image, COLORMAP_BONE);



    // Draw the principal components
    circle(color_image, pos, 1, CV_RGB(255, 0, 255), 2);


    line(color_image, pos, pos + 0.3 * Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]) , CV_RGB(255, 0, 0));
    line(color_image, pos, pos + 0.3 * Point(eigen_vecs[1].x * eigen_val[1], eigen_vecs[1].y * eigen_val[1]) , CV_RGB(0, 0, 255));

     imshow("PCa", color_image);

    return atan2(eigen_vecs[0].y, eigen_vecs[0].x);
}





///==========================================================================///
///                          CALCULO GRADIENTE                               ///
///                                                                          ///
///==========================================================================///

Mat Calculo_gradiente(Mat &original)
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



///==========================================================================///
///                           CALCULO CONTORNOS                              ///
///                                                                          ///
///==========================================================================///


struct_count Calculo_contorno (Mat mat1, Mat mat2)
{



Mat grad = mat1.clone();
Mat original = mat2.clone();


 struct_count aux_struct;
 RNG rng(12345);

      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;


Canny(grad, grad,0,200,3,false );
   findContours( grad, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  cout << "aqui hay contornos = " << contours.size()<< endl;
   // Approximate contours to polygons + get bounding rects and circles
   vector<vector<Point> > contours_poly( contours.size() );
   vector<Rect> boundRect( contours.size() );
   vector<Point2f>center_count( contours.size() );
   vector<float>radius_count( contours.size() );

   for( int i = 0; i < contours.size(); i++ )
       { approxPolyDP( Mat(contours[i]), contours_poly[i], 1, false );
         boundRect[i] = boundingRect( Mat(contours_poly[i]) );
         minEnclosingCircle( (Mat)contours_poly[i], center_count[i], radius_count[i] );
       }



    /// Draw polygonal contour + bonding rects + circles

     Mat mask;
     for( int i = 0; i< contours.size(); i++ )
        {
           mask = Mat::zeros (45, 64, CV_8UC3);
           circle( original, center_count[i], (int)radius_count[i], Scalar(0,0,0), -1, 8, 0 );

     }

   ///convierto la imagen a CV_8U para poder analizar otra vez el contorno



  double minVal, maxVal;
  minMaxLoc(original, &minVal, &maxVal);
  original.convertTo(original, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));


  aux_struct.cont_ok = 1;
  aux_struct.mat_contornos = original;
  aux_struct.contours = contours;
  aux_struct.center_count = center_count;
  aux_struct.radius_count = radius_count;



  return aux_struct;

}

///==========================================================================///
///                          CONTADOR DE CONTORNOS                           ///
///                                                                          ///
///==========================================================================///
struct_count contador_contour(Mat &original)
{
    Mat grad;
    struct_count aux_struct1;
    struct_count aux_struct2;
    struct_count aux_struct3;



    ///calcula los valores máximos y mínimos para ver si por altura hay algo en el espacio
    /// alturas por debajo de 80cm las considera nulas
     minMaxLoc(original, &minVal, &maxVal);
     cout << "ANTES minVal = " <<minVal << " maxVal = " << maxVal << endl;
    if (minVal >2500) //2.5
    {
        aux_struct1.cont_ok = 0;
        return aux_struct1;
    }
    else
    {
        minMaxLoc(original, &minVal, &maxVal);
        original.convertTo(original, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));




       //antes metia src_gray!!!

       grad = Calculo_gradiente(original);
       minMaxLoc(grad, &minVal, &maxVal);
       grad.convertTo(grad, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));


        //_______localización de los contornos_______//


       Mat threshold_output;
       Mat canny_output;
       Mat black_cont;//silueta de la persona en negro sobre fondo blanco
       int thresh = 30;


       //LLAMADAS A LA FUNCIÓN CALCULA CONTORNOS

    //PRIMER CÁLCULO DE CONTORNOS

    if (IMAGES_VISUAL_FILTROS)
       imshow("G1",grad);
    threshold( grad, threshold_output, thresh, 255, THRESH_BINARY );

    if (IMAGES_VISUAL_FILTROS)
       imshow("1c",threshold_output);

    aux_struct1 = Calculo_contorno(threshold_output, original);

    if (IMAGES_VISUAL_FILTROS)
       imshow("2c",aux_struct1.mat_contornos);


    //SEGUNDO CÁLCULO DE CONTORNOS

       Canny( aux_struct1.mat_contornos, canny_output, 80, 80*2, 3 );
    if (IMAGES_VISUAL_FILTROS)
       imshow("3c", canny_output);
       aux_struct2 = Calculo_contorno(canny_output,aux_struct1.mat_contornos);

    if (IMAGES_VISUAL_FILTROS)
       imshow("4c", aux_struct2.mat_contornos);


    aux_struct2.mat_contornos.convertTo(aux_struct2.mat_contornos, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

    return aux_struct2;
    }
}




///==========================================================================///
///                          RECORTE DE CONTORNOS                            ///
///                                                                          ///
///==========================================================================///

void cut_contour (struct_count data_contours, int numc, Mat mat1, int num_frame)
{
    Mat original = mat1.clone();
    Mat cut_contour;
    Point2f rect_start, rect_add;

        ///rectángulo con medidas de los contornos
   /*
    for (int i=0; i < data_contours.contours.size(); i++)
    {
        rect_start.x = round(data_contours.center_count[numc].x - data_contours.radius_count[numc]);
        if (rect_start.x < 0) rect_start.x = 0;

        rect_start.y = round(data_contours.center_count[numc].y - data_contours.radius_count[numc]);
        if (rect_start.y <0) rect_start.y = 0;

        rect_add.x = round(2*data_contours.radius_count[numc]);
        if (rect_start.x + rect_add.x > 64) rect_add.x = 64 - rect_start.x;

        rect_add.y = round(2*data_contours.radius_count[numc]);
        if (rect_start.y + rect_add.y > 48) rect_add.y = 48 - rect_start.y;

        cout << "dimensiones del rectangulo = " << rect_start.x << " " << rect_start.y << " " << rect_add.x << " " << rect_add.y << endl;

        cut_contour = cv::Mat(original, cv::Rect(rect_start.x, rect_start.y, rect_add.x, rect_add.y)).clone();
        imshow("ll", cut_contour);

    }

    */



    ///rectángulo con medidas aproximadas (27píxeles)

        rect_start.x = round(data_contours.center_count[numc].x - 8);
        if (rect_start.x < 0) rect_start.x = 0;

        rect_start.y = round(data_contours.center_count[numc].y - 8);
        if (rect_start.y <0) rect_start.y = 0;

        rect_add.x = 16;
        if (rect_start.x + rect_add.x > 64) rect_add.x = 64 - rect_start.x;

        rect_add.y = 16;
        if (rect_start.y + rect_add.y > 48) rect_add.y = 48 - rect_start.y;

        cout << "dimensiones del rectangulo = " << rect_start.x << " " << rect_start.y << " " << rect_add.x << " " << rect_add.y << endl;

        cut_contour = Mat(original, Rect(rect_start.x, rect_start.y, rect_add.x, rect_add.y)).clone();
        imshow("ll", cut_contour);

        sprintf(name_image, "../raquelTOFAlgs/objets_images/frame_%d_contorno_%d.jpg", num_frame, numc);

        imwrite( name_image, cut_contour );

        waitKey();


}



///==========================================================================///
///                       AISLAMIENTO DE CONTORNOS                           ///
///                                                                          ///
///==========================================================================///

datos_contorno alone_contour(Mat &dist_mat, Mat &original_aux, struct_count contornos, int numc)
{
        Mat grad;
        struct_count aux_struct;
        datos_contorno data_cont;


     //MÁSCARA PARA AISLAR A LA PERSONA

       Mat mask = Mat::zeros(48,64,CV_8U);
       Mat contorno_solo;
       Mat contorno_solo_original;
       Mat white_person;
       Mat black_cont;
       Mat scal_person;
       Mat auxiliar_PCA;
       bool true_scal = false;
       float minium_radio = 9;
       float radio_cont;
       float radio_mayor = 0;

       /////variables para PCA
       vector<vector<Point> > contours_pca;
       vector<Vec4i> hierarchy_pca;

        double minVal_real,maxVal_real;
        double minVal_escalado,maxVal_escalado;

       //EVALUACIÓN DE CADA CONTORNO ENCONTRADO

           cout << "------mascara número " << numc << endl;
           cout << " radio = " << contornos.radius_count[numc] <<endl;
           radio_cont= contornos.radius_count[numc];

           if (radio_cont<5)
           {
              data_cont.mat_contorno = Mat::zeros(48,64,CV_8U);
              data_cont.radio = 0;
              return data_cont;
           }

           if (radio_cont < 7)
               {radio_cont = minium_radio;}

           if (radio_cont < radio_mayor)
               {radio_cont = radio_mayor;}
           else
               radio_mayor=radio_cont;


           //cout << "radio mayor = " << radio_mayor << endl;
           //cout << " radio_final_contorno = " << radio_cont << endl;
           mask.setTo(255); //inicializacion de la mascara y de la imagen donde lo vamos a pegar
           white_person.setTo(255);


           circle (mask, contornos.center_count[numc], radio_cont, Scalar(0,0,0), -1,8,0);

            if (IMAGES_VISUAL_MASK)
                imshow("mm",mask);

            if (IMAGES_VISUAL_MASK)
                imshow("dm",dist_mat);

                minMaxLoc(dist_mat, &minVal_real, &maxVal_real);
                dist_mat.convertTo(dist_mat, CV_8U, 255.0/(maxVal_real - minVal_real), -minVal_real * 255.0/(maxVal_real - minVal_real));


           contorno_solo = dist_mat - mask;



            if (IMAGES_VISUAL_MASK)
                imshow("cc", contorno_solo);

           white_person = contorno_solo+mask;

            if (IMAGES_VISUAL_MASK)
                 imshow("ww", white_person);

           ///máscara para solo la persona
           minMaxLoc(white_person, &minVal, &maxVal);
           data_cont.altura_cont = minVal;

           threshold( white_person, black_cont, 235, 255, THRESH_BINARY );//silueta persona en negro
           black_cont = cv::Scalar::all(255)-black_cont;
            if (IMAGES_VISUAL_MASK)
                 imshow("bb", black_cont);


           ///reescalado de la imágen (cabeza = 0) y cálculo del gradiente
           double scal_altura = minVal/255;
           //cout << "coeficiente de escalado = " << scal_altura << endl;
              if (scal_altura == 0)
                 scal_altura = scal_altura_aux;
           scal_person = white_person*(scal_altura);



           scal_person.convertTo(scal_person, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
            if (IMAGES_VISUAL_MASK)
                imshow("pp",scal_person);


         ///Cálculo relativo para PCA
           auxiliar_PCA = black_cont.clone();
           findContours(auxiliar_PCA, contours_pca, hierarchy_pca, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
           for (size_t i = 0; i < contours_pca.size(); ++i)
           {
               // Calculate the area of each contour
               double area = contourArea(contours_pca[i]);
               getOrientation(contours_pca[i], black_cont);

           }

           true_scal = true;
           grad = Calculo_gradiente(scal_person);

            if (IMAGES_VISUAL_MASK)
           imshow("gg",grad);
           minMaxLoc(grad, &minVal, &maxVal);






       data_cont.mat_contorno = white_person;//scal_person;

       data_cont.radio = radio_cont;

      //waitKey();
          return data_cont;
}




///==========================================================================///
///                CALCULO VALORES MAX O MIN DE UNA SERIE                    ///
///                                                                          ///
///==========================================================================///


char calculo_max_values4(int A, int B, int C, int D)
{


    int max = A; /* assume x is the largest */
    char letra = 'A';

    if (B > max) { /* if y is larger than max, assign y to max */
        max = B;
        letra = 'B';
    } /* end if */

    if (C > max) { /* if z is larger than max, assign z to max */
        max = C;
        letra = 'C';
    } /* end if */

    if (D > max) { /* if z is larger than max, assign z to max */
        max = D;
        letra = 'D';
    } /* end if */


    return letra; /* max is the largest value */
} /* end function maximum */





char *calculo_min_values6(int A1, int B1, int C1, int D1, int A2, int B2, int C2, int D2)
{


    int min = A1; /* assume x is the largest */
    char *letra = "A1";

    if (B1 < min) { /* if y is larger than max, assign y to max */
        min = B1;
        letra = "B1";
    } /* end if */

    if (C1 < min) { /* if z is larger than max, assign z to max */
        min = C1;
        letra = "C1";
    } /* end if */

    if (D1 < min) { /* if z is larger than max, assign z to max */
        min = D1;
        letra = "D1";
    } /* end if */

    if (A2 < min) { /* if y is larger than max, assign y to max */
        min = A2;
        letra = "A2";
    } /* end if */

    if (B2 < min) { /* if z is larger than max, assign z to max */
        min = B2;
        letra = "B2";
    } /* end if */

    if (C2 < min) { /* if z is larger than max, assign z to max */
        min = C2;
        letra = "C2";
    } /* end if */

    if (D2 < min) { /* if z is larger than max, assign z to max */
        min = D2;
        letra = "D2";
    } /* end if */


    return letra; /* max is the largest value */
} /* end function maximum */



char calculo_min_values4(int A, int B, int C, int D)
{


    int min = A; /* assume x is the largest */
    char letra = 'A';

    if (B < min) { /* if y is larger than max, assign y to max */
        min = B;
        letra = 'B';
    } /* end if */

    if (C < min) { /* if z is larger than max, assign z to max */
        min = C;
        letra = 'C';
    } /* end if */

    if (D < min) { /* if z is larger than max, assign z to max */
        min = D;
        letra = 'D';
    } /* end if */

    return letra; /* max is the largest value */
} /* end function maximum */





///==========================================================================///
///                     DISTANCIA ENTRE PUNTO Y ESFERA                       ///
///                                                                          ///
///==========================================================================///
float sphera_point_distance (float xo, float yo,float zo, float radio, float px, float py, float pz)
/* esta función calcula la distancia que existe entre un punto de la nube de puntos a la esfera*/
{

/*primero calculamos la recta que une el punto con el centro de la esfera
  x = x1 + vx*t
  y = y1 + vy*t
  z = z1 + vz*t
*/
vector <float> vector_rect (3);
    vector_rect[0] = xo-px;
    vector_rect[1] = yo-py;
    vector_rect[2] = zo-pz;

/*ecuación de la esfera -> (x-i)²+(y-j)²+(z-k)² = r² , sustituyendo las ecuaciones de la recta en el plano
  obtenemos la variable t y calculamos el punto donde se cortan*/
float suma_cuadr_vect =  (vector_rect[0]* vector_rect[0])+(vector_rect[1]* vector_rect[1])+(vector_rect[2]* vector_rect[2]);

float t = sqrt((radio*radio)/suma_cuadr_vect);

/*Una vez sustituida la ecuación obtendremos dos puntos donde podrían cortarse*/
vector <float> intersection_point1 (3);
    intersection_point1[0] = xo + vector_rect[0]*t;
    intersection_point1[1] = yo + vector_rect[1]*t;
    intersection_point1[2] = zo + vector_rect[2]*t;

vector <float> intersection_point2 (3);
    intersection_point2[0] = xo - vector_rect[0]*t;
    intersection_point2[1] = yo - vector_rect[1]*t;
    intersection_point2[2] = zo - vector_rect[2]*t;

float distancia1 = sqrt((px-intersection_point1[0])*(px-intersection_point1[0]) + (py-intersection_point1[1])*(py-intersection_point1[1]) + (pz-intersection_point1[2])*(pz-intersection_point1[2]));
float distancia2 = sqrt((px-intersection_point2[0])*(px-intersection_point2[0]) + (py-intersection_point2[1])*(py-intersection_point2[1]) + (pz-intersection_point2[2])*(pz-intersection_point2[2]));

float distancia_mejor = sqrt((px-xo)*(px-xo) + (py-yo)*(py-yo) + (pz-zo)*(pz-zo));

/*
if (distancia1 > distancia2)
    return distancia2;
else
    return distancia1;
*/
return distancia_mejor;
}




///==========================================================================///
///                     VECINO DIFERENTE DE 0 MAS CERCANO                    ///
///                                                                          ///
///==========================================================================///

float nnSearch(Mat &original ,int x, int y, int width )
{
    float new_val = 0;
    int iniciox, inicioy, finalx, finaly;

    switch (x){
        case 0:
            iniciox = x;
            finalx = x+width;
            break;
        case (423):
            iniciox = x - width;
            finalx = x;
            break;
        default:
            iniciox = x-width;
            finalx = x + width;
                }

    switch (y){
        case 0:
            inicioy = y;
            finaly = y+width;
            break;
        case (511):
            inicioy = y - width;
            finaly = y;
            break;
        default:
            inicioy = y-width;
            finaly = y + width;
                }

   if (iniciox < 0) {iniciox = 0;}
   if (finalx > 423) {finalx = 423;}
   if (inicioy < 0) {inicioy = 0;}
   if (finaly > 511) {finaly = 511;}


while(new_val == 0)
{

    for(int i = iniciox; i<= finalx; i++)
    {
        for (int j=inicioy; j<=finaly; j++)
        {
           if (original.at<unsigned short>(i,j) >1000)
              {
               ///cout << "posición del pixel del que coje valor => x = " << i << " y = " << j << endl;
               new_val = original.at<unsigned short>(i,j);
               ///cout << "Valor que toma => " << new_val << endl;

               break;
              }

        }
        if(new_val !=0)break;

    }
    if(new_val !=0)break;

    iniciox = iniciox - 1;
    finalx = finalx + 1;
    inicioy = inicioy -1;
    finaly = finaly +1;

    if (iniciox < 0) {iniciox = 0;}
    if (finalx > 423) {finalx = 423;}
    if (inicioy < 0) {inicioy = 0;}
    if (finaly > 511) {finaly = 511;}


}

    return new_val;


}


///==========================================================================///
///                                RECORTE ROI                               ///
///                                                                          ///
///==========================================================================///

data_ROI cutROI (Mat &original, Point2f center, float radius)
{

    Point2f rect_start, rect_add;
    float rows = original.rows;
    float cols = original.cols;


    data_ROI analizeROI;

    ///rectángulo con medidas aproximadas (27píxeles)

        rect_start.x = round(center.x - radius);
        if (rect_start.x < 0) rect_start.x = 0;

        rect_start.y = round(center.y - radius);
        if (rect_start.y <0) rect_start.y = 0;

        rect_add.x = radius * 2;
        if (rect_start.x + rect_add.x > cols) rect_add.x = cols - rect_start.x;

        rect_add.y = radius * 2;
        if (rect_start.y + rect_add.y > rows) rect_add.y = rows - rect_start.y;

        //cout << "diemsiones de la imagen = " << rows << " " << cols<< endl;

        //cout << "dimensiones del rectangulo = " << rect_start.x << " " << rect_start.y << " " << rect_add.x << " " << rect_add.y << endl;

        analizeROI.ROI = Mat(original, Rect(rect_start.x, rect_start.y, rect_add.x, rect_add.y)).clone();

        analizeROI.rect_start = rect_start;
        analizeROI.rect_add = rect_add;

        return analizeROI;

}

///==========================================================================///
///                               PEGAR ROI                               ///
///                                                                          ///
///==========================================================================///

Mat pasteROI (Mat &original, data_ROI analizeROI)
{


}

///==========================================================================///
///                           MÁXIMO EN UNA MATRIZ                           ///
///                                                                          ///
///==========================================================================///
Point2f maxPointMat (Mat &original)
{
    int i, j;
    int16_t maxAlt = 10000;
    Point2f maxPoint;

     minMaxLoc(original, &minVal, &maxVal);

    for (i=0; i<original.rows; i++)
    {
        for (j=0; j<original.cols; j++)
        {
            if(original.at<unsigned short>(i,j)< maxAlt && original.at<unsigned short>(i,j)!=0)
               {
                maxPoint.x = j;
                maxPoint.y = i;
                maxAlt = original.at<unsigned short>(i,j);
               }
        }
    }


    return maxPoint;
}
