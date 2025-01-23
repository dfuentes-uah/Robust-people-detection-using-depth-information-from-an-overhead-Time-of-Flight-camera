#include <pcl/point_types.h>
#include <pcl/features/vfh.h>

#include <pcl/features/normal_3d.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/system/error_code.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

#include <pcl/io/vtk_io.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/histogram_visualizer.h>
#include <iostream>

//segmentacion
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/common/distances.h>

//para operaciones opencv
#include <stdio.h>
#include <stdlib.h>
#include <pmdsdk2.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>


///SEGMENTACION

#include <vector>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>


//search neighbors
#include <pcl/kdtree/kdtree_flann.h>


#include "head_shoulder.h"
#include "Funciones_opencv.h"
#include "Funciones_pcl.h"
#include "geom_forms.h"
#include "geom_filters.h"
#include "clasificador.h"

#define SEGMENTATION  0
#define _VIEWER_ELIMINA_PERSONA 0
#define MEXICAN_RESULT 1


typedef pcl::PointXYZ PointT;
using namespace cv;
using namespace pcl;
using namespace std;

typedef struct
{
   double fx;
   double fy;
   double cx;
   double cy;
   double k1;
   double k2;
   double k3;
   double p1;
   double p2;
}kinect_IRparams;


typedef struct
{
    double fx;
    double fy;
    double cx;
    double cy;
}kinect_RGBparams;


//variables conexion con la camara
int res;
char err[125];
 PMDHandle hnd;
bool head=false;
bool shoulder=false;
float head_rad, shoul_rad;
bool true_head = false;
static Mat dist_mat;
RNG rng(12345);

pcl::visualization::PCLVisualizer _viewer_person_suelo ("floor");
pcl::visualization::PCLVisualizer _viewer_gauss ("Gaussian_window");
pcl::visualization::PCLVisualizer _viewer_kinect ("kinect_window");


char buffer [30];
char buffer2 [50];
char nombre[50];
char nom_imagen[50];

 double minVal2, maxVal2;



int pcl_test_PMD(float *x2D,float *y2D,float *z2D, int cont_framesR)
{


    int c = 0;
    sprintf(buffer,"../cloud/cloud%d.pcd",c);
    cout << "-----------------------------------------------------------------------" << buffer << endl;
   //paint_mexican();
   //gaussian_filter();
   //mexican_filter();
   slice_image();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_suelo (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_resta (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_persona_suelo (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr gaussian_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  float size_array = 3072;

  posHighest Highest_point_cont;


///============================Eliminacion de pixeles erroneos==========================
// pone los pixeles erroneos en el suelo

float max_array = 0;
int ie;
float* z2D_limited = new float [48 * 64];

for (ie=0; ie<3072; ie++)
{

    if (z2D[ie] > max_array)
        max_array=z2D[ie];
}


  for (ie=0; ie<3072; ie++)
  {

      if (z2D[ie] <= 0)
          z2D[ie]=max_array;
  }

  cout << "minimo del array de distancias = " << max_array <<endl;



  Mat src_gray(48,64,CV_32FC1,z2D);

imshow("eee", src_gray);
minMaxLoc(src_gray, &minVal2, &maxVal2);
cout << "valores máximos y minimos de src_gray = " << minVal2 << " " << maxVal2 << endl;



  ///============================GUARDAR IMAGEN FONDO===============================

  FILE *f;

    Mat ZMAT_prueba;
    float aux = 3152.36;
    float* aux_rec = new float[3072];

    int m;

   if (cont_framesR == 1)
     {
       f = fopen("plano_suelo.dat","wb");
       fwrite (z2D, sizeof(float), 3072, f);
       fclose(f);
       ZMAT_prueba = Mat(48,64,CV_32FC1,z2D);
     }

     else
     {
        f = fopen("plano_suelo.dat","rb");
        fread(aux_rec, sizeof(float),3072,f);

        fclose(f);
        ZMAT_prueba= Mat(48,64,CV_32FC1,aux_rec);

        imshow("z1", ZMAT_prueba);


      }




  ///=====================CREAR IMAGEN DE DISTANCIAS A PARTIR DE Z=====================//

  /*1)creo la imágen de la variable Z (altura)
   *2)busco los diferentes contornos de la imagen y los separo en imágenes diferentes
   *3)en cada imagen (contornos independientes) busco si hay píxeles erroneos (por si hay dos personas de pelo negro y diferente altura
  */


  float dist_d,max_d=0;
  float*  array_dist = new float [48 * 64];
  float*  dist_norm = new float [48 * 64];
  int pos_max;


  for (int f=0; f<3072; f++)
  {
      dist_d = z2D_limited[f];
      if(max_d < dist_d)
        {max_d = dist_d; pos_max = f;}
  }
 /*
  //normalización del array de distancias
  for (int f2=0; f2< 3072; f2++)
  {
        dist_norm[f2] = z2D_limited[f2]/max_d;
  }

*/
  dist_mat= Mat(48,64,CV_32FC1,z2D);
  Mat auxiliar_dist = dist_mat.clone();

  minMaxLoc(dist_mat, &minVal2, &maxVal2);
  //cout << "valores máximos y minimos de dist_mat = " << minVal2 << " " << maxVal2 << endl;
  auxiliar_dist.convertTo(auxiliar_dist, CV_8U, 255.0/(maxVal2 - minVal2), -minVal2 * 255.0/(maxVal2 - minVal2));
  imshow("vv", auxiliar_dist);

  minMaxLoc(ZMAT_prueba, &minVal2, &maxVal2);
  //cout << "valores máximos y minimos de ZMAT_prueba = " << minVal2<< " " << maxVal2 << endl;

  Mat resta_fondo = dist_mat - ZMAT_prueba;

  minMaxLoc(resta_fondo, &minVal2, &maxVal2);
  //cout << "valores máximos y minimos de la resta = " << minVal2 << " " << maxVal2 << endl;


 ///===================================================================================================//


 ///================================PASO DE ARRAYS A NUBE DE PUNTOS===================================//
 ///                                   LOCALIZACIÓN PUNTO MÁXIMO
    //Creación de la nube de puntos a partir de los arrays X, Y, Z que obtenemos de la base de datos
    //..ajuste del ancho, largo y tamaño, y relleno de la nube de puntos


     cloud->width = 64;
     cloud->height = 48;

     cloud->is_dense = false;
     cloud->points.resize (cloud->width * cloud->height);

    for (int hh=0; hh<cloud->points.size(); hh++)
    {
        cloud->points[hh].x = x2D[hh];
        cloud->points[hh].y = y2D[hh];
        cloud->points[hh].z = z2D[hh];
    }



    shoulder = false;
    head = false;


    posHighest Highest_point = LocateMaxCloud(cloud);
  //  gaussian_cloud = paint_gaussian(Highest_point.x,Highest_point.y);
    RelationDimCloudData relDimPix=  relationDimCloud (cloud, x2D, y2D, z2D,size_array );





///======================BUSQUEDA CONTORNOS Y DEJAR SOLA A LA PERSONA EN UNA IMAGEN====================//

  Mat dist_mat_aux = dist_mat.clone();

  imshow("hola", dist_mat_aux);
  Mat original_aux = dist_mat.clone();
  Mat src_gray_aux = src_gray.clone();
  Mat resta_fondo_aux = resta_fondo.clone();

  datos_contorno alone_object;
  struct_count contornos_localizados;


  contornos_localizados = contador_contour(src_gray_aux);





  if (contornos_localizados.cont_ok == 0)
  {
     cout << "ningún objeto en el espacio" << endl;
  }

  else
  {
      cout << "*****=======NUMERO DE CONTORNOS FINALES = " << contornos_localizados.contours.size() << endl;


    for (int numc=0; numc < contornos_localizados.contours.size(); numc ++)
        {
            alone_object = alone_contour(dist_mat_aux, original_aux, contornos_localizados, numc);


            cout << "radio de los contornos localizados = " << alone_object.radio << endl;
            //cout << "altura de los contornos localizados = " << alone_object.altura_cont << endl;


        if (alone_object.radio == 0)
        {
            cout << "*****************CONTORNO NUMERO " << numc << "*******************"<< endl;
            cout <<  "OBJETO DEMASIADO PEQUEÑO" << endl;
        }
        else
        {
             cout << "*****************CONTORNO NUMERO " << numc << "*******************"<< endl;
             cout << "radio del contorno encontrado = " << alone_object.radio << endl;
             //imshow("kk", persona_solica);


             cut_contour(contornos_localizados, numc, src_gray_aux, cont_framesR);

             sprintf(nombre, "../raquelTOFAlgs/objets_frames/frame_silla%d.jpg", cont_framesR);

             imwrite( nombre, alone_object.mat_contorno );


      ///================================PASO DE LA IMAGEN DE z A ARRAY OTRA VEZ============================//
         float*  new_z_array = new float [48 * 64];
         int zz =0;

         for (int nn=0;nn<48;nn++)
             {
              for (int vv=0; vv<64; vv++)
                 {
                  new_z_array[zz]=alone_object.mat_contorno.at<uchar>(nn,vv);
                  zz++;
                  }
             }

          Mat pruebaZ= Mat(48,64,CV_32FC1,new_z_array);
          minMaxLoc(pruebaZ, &minVal2, &maxVal2);
          pruebaZ.convertTo(pruebaZ, CV_8U, 255.0/(maxVal2 - minVal2), -minVal2 * 255.0/(maxVal2 - minVal2));
         // imshow("kk", pruebaZ);



        ///=====================BUSQUEDA PUNTO MÁXIMO DE LA NUBE DE PUNTOS Y EXTREMOS X Y=====================//
            //..Recorrido de la nube de puntos en la variable Z para localizar

          float x,y,z, ro;
          int pos, posz;
          Highest_point_cont = LocateMaxCloud(cloud);

          x = Highest_point_cont.x;
          y = Highest_point_cont.y;
          z = Highest_point_cont.z;
          pos = Highest_point_cont.pos;
          posz = Highest_point_cont.posz;
          ro = alone_object.radio;

          cout << "posición -> x = " << x << " y = " << y << " z = "<< z << endl;
          cout << "altura al calculada en locateMaxCloud = " << cloud->points[pos].z << endl;

        /*
          gaussian_cloud = paint_gaussian(x * relDimPix.x, y * relDimPix.y, ro);

          _viewer_gauss.addCoordinateSystem(100.0,0);//, (const char *)"CS1", 0);
          _viewer_gauss.setBackgroundColor (0, 0, 0);
          _viewer_gauss.addPointCloud(gaussian_cloud, "gaussian");
          _viewer_gauss.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "gaussian");
          _viewer_gauss.spinOnce(10);
          waitKey(100);
          _viewer_gauss.removeShape("gaussian");
        */
         ///=================================================================================================//

    /*


            //correspondencia de píxeles entre pcl y imagen distancias
            //contorno del círculo en la imágen de distancias

            int aa = pos/64;
            int bb = pos-64*aa;
            cout << "aa_x = " << aa << "aa_y = " << pos - 64*aa << endl;
            CvPoint* center = new CvPoint();
            center->x = aa;
            center->y = bb;
            imwrite( "image_dist.jpg",255*dist_mat);

            //circle(dist_mat,*center, 10,  Scalar( 0, 0, 255 ), 1,8,0);
            //imshow("image",dist_mat);

            float alt_head = cloud->points[pos].z;
            float alt_suelo = cloud->points[posz].z;
            dist_mat = cv::imread("image_dist.jpg", CV_LOAD_IMAGE_GRAYSCALE);*/
            ////______________________________________________________________________________________//


            ////Calculo_histograma(dist_mat, "Histograma normal");
            Mat filter_mat = Mat::zeros(48, 64, CV_32FC1);
            medianBlur(dist_mat,filter_mat,3);






            ///=================================================================================================================

            /////=====================================DETECCION PCL





         // true_head = detection_pcl(cloud, pos, posz, true_head);
        //  _viewer_gauss.removeShape("gaussian");

            ///====================================PONER A NIVEL DEL SUELO EL RADIO DE ESE CONTORNO
           //proyección de toda la nube en el plano del suelo
            pcl::copyPointCloud( *cloud,*cloud_persona_suelo);

            for (int gg =0; gg < cloud_persona_suelo->size(); gg++)
               {
                  cloud_persona_suelo->points[gg].z = 0;
               }


            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_person;
            kdtree_person.setInputCloud (cloud_persona_suelo);
            pcl::PointXYZ searchPoint_suelo;

            searchPoint_suelo.x = cloud_persona_suelo->points[pos].x;
            searchPoint_suelo.x = cloud_persona_suelo->points[pos].y;
            searchPoint_suelo.x = cloud_persona_suelo->points[pos].z;

            std::vector<int> pointIdxRadiusSearch_suelo;
            std::vector<float> pointRadiusSquaredDistance_suelo;



             float radius_suelo = alone_object.radio*2.83/64;


            cout << "radio del contorno" << radius_suelo << endl;


              kdtree_person.radiusSearch (searchPoint_suelo, radius_suelo, pointIdxRadiusSearch_suelo, pointRadiusSquaredDistance_suelo);

            if(_VIEWER_ELIMINA_PERSONA)
            {
              _viewer_person_suelo.addCoordinateSystem(1.0, 0);//, (const char *)"CS1", 0);
              _viewer_person_suelo.setBackgroundColor (0, 0, 0);
              _viewer_person_suelo.addPointCloud(cloud_persona_suelo, "floor");
              _viewer_person_suelo.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 255.0, 1.0, "floor");
            }

              ///ciruclo para ver si funciona
              pcl::ModelCoefficients circle_coeff_suelo;
              circle_coeff_suelo.values.resize (3); // We need 3 values
              circle_coeff_suelo.values[0] = x;
              circle_coeff_suelo.values[1] = y;
              circle_coeff_suelo.values[2] = radius_suelo;

             if(_VIEWER_ELIMINA_PERSONA)
             {
              _viewer_person_suelo.addCircle(circle_coeff_suelo,"circulo",0);
              _viewer_person_suelo.spin();
              _viewer_person_suelo.removeShape("circulo",0);
              _viewer_person_suelo.removePointCloud("floor");
             }

              //recorro la nube de puntos original poniendo la z de los píxeles encontrados a 0
              for (int pix= 0; pix < pointIdxRadiusSearch_suelo.size(); pix++)
              {
                  cloud->points[pointIdxRadiusSearch_suelo[pix]].z = max_array;
              }
    }
        }
  }
//waitKey();




waitKey();


}

int pcl_test_Kinect2(int16_t *z2D16, int cont_framesR)
{

  kinect_IRparams kIR_params;
  kinect_RGBparams kRGB_params;
    // Variables configuración kinect
    kIR_params.fx = 365.775391;
    kIR_params.fy = 365.775391;
    kIR_params.cx = 255.634293;
    kIR_params.cy = 207.1539;
    kIR_params.k1 = 0.0935848877;
    kIR_params.k2 = -0.271052808;
    kIR_params.k3 = 0.0922934487;
    kIR_params.p1 = 0;
    kIR_params.p2 = 0;

    kRGB_params.fx = 1081.37207;
    kRGB_params.fy = 1081.37207;
    kRGB_params.cx = 959.5;
    kRGB_params.cy = 539.5;


    sprintf(buffer,"../cloud/cloud%d.pcd",cont_framesR);
    cout << "-----------------------------------------------------------------------" << buffer << endl;
    cout <<"--inicio"<<endl;
    int16_t max = 0;
    int16_t min = 10000;

    int size_array = 217088;
    int16_t* x2D16 = new int16_t[size_array];
    int16_t* y2D16 = new int16_t[size_array];
    int16_t* ZsinCeros = new int16_t[size_array];

    int i, j, px=0;





    cout <<kIR_params.cx << endl;

    //realX = (u - irCameraParams.cx) * *realZ / irCameraParams.fx;
    //realY = (v - irCameraParams.cy) * *realZ / irCameraParams.fy;





/*

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_suelo (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_resta (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_persona_suelo (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr gaussian_cloud (new pcl::PointCloud<pcl::PointXYZ>);

*/

  int width = 423;
  int height = 511;


  for (i=0; i<size_array; i++)
  {
      if (z2D16[i] > max)
          max = z2D16[i];
      if (z2D16[i] < min)
          min = z2D16[i];
  }

  cout << "valores máximos y mínimos array= " << max << " " << min << endl;

  Mat src_gray(424,512,CV_16UC1,z2D16);

  ///===============================Limitación nube de puntos==============================


  for(int u=0; u < 424; ++u)
  {
      for(int v=0; v < 512; ++v)
      {
          x2D16[px] = ((u - kIR_params.cx)* z2D16[px] ) / kIR_params.fx;
          y2D16[px] = ((v - kIR_params.cy)* z2D16[px] ) / kIR_params.fy;
          px++;
      }
  }


  Mat xMat(424,512,CV_16UC1,x2D16);
  Mat yMat(424,512,CV_16UC1,y2D16);
  Mat zMat(424,512,CV_16UC1,z2D16);

  //Limitación del cuadro efectivo de las matrices
  Rect ROI =   Rect ( 50 , 50 , height - 100 , width-100);

  Mat xMatRoi = xMat(ROI);
  Mat yMatRoi = yMat(ROI);
  Mat zMatRoi = zMat(ROI);

  int16_t* xROI = new int16_t[xMatRoi.rows*xMatRoi.cols];
  int16_t* yROI = new int16_t[xMatRoi.rows*xMatRoi.cols];
  int16_t* zROI = new int16_t[xMatRoi.rows*xMatRoi.cols];

  //Paso de las matrices limintadas a arrays para construir la nube de puntos
  for (i=0; i< xMatRoi.rows; i++)
  {
      for(j=0; j< xMatRoi.cols; j++)
      {
          xROI[xMatRoi.cols*i + j] = xMatRoi.at<unsigned short>(i,j);
          yROI[xMatRoi.cols*i + j] = yMatRoi.at<unsigned short>(i,j);
          zROI[xMatRoi.cols*i + j] = zMatRoi.at<unsigned short>(i,j);
      }
  }

   ///==========================PASO DE ARRAYS A NUBE DE PUNTOS===============================

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width = xMatRoi.rows;
  cloud->height = xMatRoi.cols;

  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);

 for (int hh=0; hh<cloud->points.size(); hh++)
 {
     cloud->points[hh].x = xROI[hh];
     cloud->points[hh].y = yROI[hh];
     cloud->points[hh].z = zROI[hh];
 }

 posHighest Highest_point = LocateMaxCloud(cloud);
/*
 cout << "sale de la función" << endl;
 _viewer_kinect.addCoordinateSystem(100.0,0);//, (const char *)"CS1", 0);
 _viewer_kinect.setBackgroundColor (0, 0, 0);
 _viewer_kinect.addPointCloud(cloud, "kinect");
 _viewer_kinect.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "kinect");
 _viewer_kinect.spinOnce(1);
// waitKey(1);
 _viewer_kinect.removeShape("kinect");
*/

int numImagen = 443;



/*LEER FRAME A FRAME
 while(numImagen < 485 )
{

  sprintf(nom_imagen, "../raquelTOFAlgs/framesGente/imagen%d.png", numImagen);
  numImagen++;

  Mat leida = imread (nom_imagen,CV_LOAD_IMAGE_GRAYSCALE);
  imshow("leida", leida);


  Mat sinCeros = leida(ROI);
*/
 ///============================Eliminacion de pixeles erroneos==========================
  Mat sinCeros=zMatRoi.clone();

  /*
  //FILTRO DE MEDIANA PARA ELIMINAR PÍXELES SUELTOS
  int MAX_KERNEL_LENGTH = 3;

  for ( i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
  { medianBlur ( src_gray, sinCeros, i );}
       // if( display_dst( DELAY_BLUR ) != 0 ) { return 0; } }
*/

  for (i=0; i<zMatRoi.rows; i++)
  {
      for(j=0; j<zMatRoi.cols; j++)
      {
          if (sinCeros.at<unsigned short>(i,j)==0)
          {
             sinCeros.at<unsigned short>(i,j)=nnSearch(sinCeros, i,j,10);
          }

      }

 }




  ///===========================Analisis del número de personas con sombrero mexicano==============

  ///Recortar la imágen

 // Mat smallImages = sinCeros(ROI);
Mat Imprepro = sinCeros.clone();
Mat smallImages = sinCeros.clone();
Mat imagenProcesada = preProcess(Imprepro);
Mat mexicanFilter = mexican_filter(imagenProcesada, 1);

Mat Imagen;
medianBlur(sinCeros,Imagen, 5);

vector<int> actualVector = heighNumPoints(Imagen);

waitKey();


 if (MEXICAN_RESULT)
 {

     Mat src_gray_SHOW = src_gray.clone();
     Mat smallImages_SHOW = smallImages.clone();
     Mat sinCeros_SHOW = Imagen.clone();

     minMaxLoc(src_gray_SHOW, &minVal2, &maxVal2);
     src_gray_SHOW.convertTo(src_gray_SHOW, CV_8U, 255.0/(maxVal2 - minVal2), -minVal2 * 255.0/(maxVal2 - minVal2));

     minMaxLoc(sinCeros_SHOW, &minVal2, &maxVal2);
     sinCeros_SHOW.convertTo(sinCeros_SHOW, CV_8U, 255.0/(maxVal2 - minVal2), -minVal2 * 255.0/(maxVal2 - minVal2));

     minMaxLoc(smallImages_SHOW, &minVal2, &maxVal2);
     smallImages_SHOW.convertTo(smallImages_SHOW, CV_8U, 255.0/(maxVal2 - minVal2), -minVal2 * 255.0/(maxVal2 - minVal2));

     //imshow("src", src_gray_SHOW);
     //imshow("media_filter", smallImages_SHOW);
     imshow("sinCeros", sinCeros_SHOW);
     //imshow("Filtro mexicano", mexicanFilter);
}


///sobredimensionar el array de puntos_maximos!!!!
//vector<Point2f> Puntos_maximos =  mexicanMaximos(mexicanFilter, Imprepro);
// cout << "SALEEEEEe" << endl;




  //QUEDA POR PONER LA IMAGEN EN NUVE DE PUNTOS, PARA INTENTAR PROBAR LO DE LA ESFERA Y PASAR LOS FILTRO DE LA GAUSSIANA Y EL SOMBRERO!!


}


