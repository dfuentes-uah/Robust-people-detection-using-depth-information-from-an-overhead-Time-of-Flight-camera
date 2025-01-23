#include <stdio.h>
#include <string.h>
#include <stdlib.h>
////////////////////////////////
#include <core/core.hpp>
#include <highgui/highgui.hpp>
#include <imgproc/imgproc.hpp>
#include <contrib/contrib.hpp>
//#include <pmdsdk2.h>
#include <iostream>

#include "sourceLocationResultsLib/sourcelocationresultslib.h"
#include "filehand/filehand.h"
#include "readPMD3sFilesLib/readPMD3sFilesLib.h"
#include "tofCountingGeneralLib/tofCountingGeneralLib.h"

/////////////////////////////////
//#include "head_shoulder.h"
#include "Funciones_opencv.h"
#include "Funciones.h"
//#include "Funciones_pcl.h"
//#include "geom_forms.h"
#include "geom_filters.h"
#include "clasificador.h"
#include "maxDetector.h"
#include "direccion.h"
#include "histogramasec.h"
#include "tiempo.h"
#include "saveResult.h"
#include "extractVector.h"
#include "DetectorTOF.h"
#include "raquelTOFAlgs/pcl_test.h"



#define VISUALIZACION 1
#define SHOW_FRAME 0
#define READWRITE 1             //0 READ 1 WRITE
#define maxSUELO 3400
#define NUMCARAC 5
//librerias PCL



using namespace cv;
using namespace std;

int _n = 0;                     // NEVER, EVER, EVER DO THIS :-) (JMG)


//TIDWINPCL pepe;

int
initRaquel(void)
{
  //pcl::visualization::PCLVisualizer _viewer ("Simple visualizing
  //head");

  return 0;
}

int
processRaquel(float *xData, float *yData, float *zData,
              TTofCountingResult * cResult, int cont_framesR)
{
  cResult->numUsers = 1;
  cResult->x[0] = 87;
  cResult->y[0] = 50;

  __printInfo("Running Raquel's alg");

  //pcl_test(xData, yData, zData, cont_framesR);

  return 0;
}

int
processDavid(int16_t * z16Data, int cont_framesR, TSrcLocationResult pmd3GT,
             TTofCountingResult * cResult, char *resultsFilename)
{
  __printInfo("Into the algorithm");
  char *sequenceName="yeah";
  //TOFdetector(z16Data,cont_framesR,sequenceName );//char *sequenceName)
  //TOFdetector(z16Data, cont_framesR, pmd3GT, cResult);
  detector(z16Data, cont_framesR, resultsFilename);
  
  return 0;
}

int
processmexican(int16_t * z2D16, int cont_framesR)
{

<<<<<<< tofCounting.c
tiempos tmex;
=======
  tiempos tmex;
>>>>>>> 1.12


<<<<<<< tofCounting.c
tmex.frame=cont_framesR;
=======
  tmex.frame=cont_framesR;
>>>>>>> 1.12
  Mat zMat2, zMat3, zMat4, zMat5, mexc, zx;
  Mat zMat(424, 512, CV_16UC1, z2D16);
<<<<<<< tofCounting.c
  zMat5 = zMat.clone();
  zMat4 = (zMat5 * 255 / 65535);
  zMat4.convertTo(zMat3, CV_8UC1, 15);
  if(VISUALIZACION==1)
  {
  imshow("mex23", zMat3);
  }
  int64 t0= getTickCount();
   preProcessROI(zMat);
   int64 t1 = getTickCount();
   double secs = (t1-t0)/getTickFrequency();
   tmex.preprocesado=secs;
   saveTimes(secs, '5',cont_framesR);
=======
  zMat5 = zMat.clone();
  zMat4 = (zMat5 * 255 / 65535);
  zMat4.convertTo(zMat3, CV_8UC1, 15);
  if(VISUALIZACION==1)
    {
      imshow("mex23", zMat3);
    }
  int64 t0= getTickCount();
  preProcessROI(zMat);
  int64 t1 = getTickCount();
  double secs = (t1-t0)/getTickFrequency();
  tmex.preprocesado=secs;
  saveTimes(secs, '5',cont_framesR);
>>>>>>> 1.12
  ruido_uint16(zMat, &zMat);
  eliminasuelo(zMat);
  zMat5 = zMat.clone();
  zMat4 = (zMat5 * 255 / 65535);
  zMat4.convertTo(zMat3, CV_8UC1, 15);
<<<<<<< tofCounting.c
  if(VISUALIZACION==1)
  {
  imshow("mex2", zMat3);
  }
  t0= getTickCount();
=======
  if(VISUALIZACION==1)
    {
      imshow("mex2", zMat3);
    }
  t0= getTickCount();
>>>>>>> 1.12
  Mat mex = mexican_filter(zMat, 1);
<<<<<<< tofCounting.c
  t1 = getTickCount();
  secs = (t1-t0)/getTickFrequency();
     tmex.filtromex=secs;
   saveTimes(secs, '6',cont_framesR);
   if(VISUALIZACION==1)
   {
  imshow("mex1", mex);
   }
   int numpersonas;
  t0= getTickCount();
  mexic(mex, zMat3, cont_framesR,&numpersonas);
   t1 = getTickCount();
  secs = (t1-t0)/getTickFrequency();
  tmex.detectorpicos=secs;
  tmex.numeropersonas=numpersonas;
   tmex.numeromaximos=numpersonas;
   saveTimes(secs, '7',cont_framesR);
=======
  t1 = getTickCount();
  secs = (t1-t0)/getTickFrequency();
  tmex.filtromex=secs;
  saveTimes(secs, '6',cont_framesR);
  if(VISUALIZACION==1)
    {
      imshow("mex1", mex);
    }
  int numpersonas;
  t0= getTickCount();
  mexic(mex, zMat3, cont_framesR,&numpersonas);
  t1 = getTickCount();
  secs = (t1-t0)/getTickFrequency();
  tmex.detectorpicos=secs;
  tmex.numeropersonas=numpersonas;
  tmex.numeromaximos=numpersonas;
  saveTimes(secs, '7',cont_framesR);
>>>>>>> 1.12
  cvtColor(mex, mexc, CV_GRAY2RGB);
<<<<<<< tofCounting.c
  mextemporizacion(tmex);
  if(VISUALIZACION==1)
  {
  imshow("mex", mexc);
  waitKey(1);
  }
=======
  mextemporizacion(tmex);
  if(VISUALIZACION==1)
    {
      imshow("mex", mexc);
      waitKey(1);
    }
>>>>>>> 1.12

  return 0;
}


int
main(int argc, char *argv[])
{

  /* Mat pointsScene;
     pointsScene = Mat::zeros(424, 512, CV_8UC1);
     imwrite( "pointsScene.png", pointsScene ); */
  int cameraType = CAMERA_TYPE_KINECTV2;  // 1 for PMD3S
  TSrcLocationResult pmd3GT;
  char *fileName,
    frameFilename[MAX_PATH], resultsFilename[MAX_PATH], *refFileName;
  FILE *fp, *resultsFp;
  TPMD3sStreamData pmd3Data;
  float *frameYData, *frameZData, *frameXData, *frameDepthData,
    *frameGrayData;
  unsigned *frameConfData;
  TTofCountingResult cResult;
  off_t numFrame;
  int numUser;
  int algorithm = 1;
  int streamType = CAMERA_Z16_STREAM_MASK;  // CAMERA_IR_STREAM_MASK; // CAMERA_XYZ_STREAM_MASK | 
  int16_t *z2D16, *ir2D16;
  int cont_framesR = 0;
  char pepe[100];
  char dirDataFiles[MAX_PATH] = "/usr/share/geintra/databases/mmodal/tofCountingK2/data/";
  char dirResultFiles[MAX_PATH] = "./";
  char auxFilename[MAX_PATH] = "";
  

  
  __printInfo("Running %s", argv[0]);
  if (argc != 3)
    {
      __printErr
        ("Missing arguments. Usage: %s <list_filename> <0|1> (0=Raquel alg, 1=David alg)",
         argv[0]);
      return 1;
    }
  else
    {
      fileName = argv[1];
      algorithm = atoi(argv[2]);
    }
  //___________Lectura .gt

  /*
  // JMG: It's not used. If if is needed, please use the same base
  //      filename to access the .gt file
  refFileName = (char *)"seq000045-t1.gt";
  __printInfo("Attempting to read tof counting ground truth (reference) file file %s",
  refFileName);
  if (__readSrcLocationResult(refFileName, &pmd3GT, (char *) "TOF_COUNTING_DATABASE_GROUND_TRUTH", 1,
  0, 1) != 0)
  {
  __printErr("Error while reading %s", refFileName);
  return 1;
  }
  */
  
  //_______________________

  fp = fopen((const char *) fileName, "r");
  if (fp == NULL)
    {
      __printErr("Unable to open sequence list file [%s]", fileName);
      return 1;
    }

  if (algorithm == 0)
    initRaquel();

  __printInfo("Reading list file...");

  while (!feof(fp))
    {
      if (fscanf(fp, "%s", frameFilename) != 1)
        continue;

      __printInfo("Reading frame file [%s]", frameFilename);

      // If you need additional files to be read, just add the
      // corresponding bitwise constants
      //      if (__readPMD3sFile(frameFilename, &pmd3Data, 
      //                          CAMERA_XYZ_STREAM_MASK | CAMERA_DEPTH_STREAM_MASK | CAMERA_CONF_STREAM_MASK | CAMERA_IR_STREAM_MASK, cameraType) != 0)
      sprintf(auxFilename, "%s%s", dirDataFiles, frameFilename);
      __ponDirectorio(frameFilename, auxFilename);
      if (__readPMD3sFile(frameFilename, &pmd3Data,
                          &streamType, cameraType) != 0)
        {
          __printErr("Error while reading %s", frameFilename);
          return 1;
        }

      strcpy(resultsFilename, frameFilename);
      __ponDirectorio(resultsFilename, dirResultFiles);
      __ponExtension(resultsFilename, "result");
<<<<<<< tofCounting.c
      /*resultsFp = fopen((const char *) resultsFilename, "w");
      if (resultsFp == NULL)
=======
      /*resultsFp = fopen((const char *) resultsFilename, "w");
        if (resultsFp == NULL)
>>>>>>> 1.12
        {
        __printErr("Unable to open results file [%s]", resultsFilename);
        return 1;
        }
<<<<<<< tofCounting.c
      else
        __printInfo("Writing counting results to file [%s]", resultsFilename);*/
=======
        else
        __printInfo("Writing counting results to file [%s]", resultsFilename);*/
>>>>>>> 1.12

      /* Process all frames */
      for (numFrame = 0; numFrame < pmd3Data.numFrames; numFrame++)
        {
          // Run per frame process
          /* frameZData = pmd3Data.z2D[numFrame];
             frameYData = pmd3Data.y2D[numFrame];
             frameXData = pmd3Data.x2D[numFrame];
             frameGrayData = pmd3Data.ir2D[numFrame];
             frameDepthData = pmd3Data.depth2D[numFrame];
             frameConfData = pmd3Data.conf2D[numFrame]; */

          z2D16 = pmd3Data.z2D16[numFrame];
          float respca[10], altura[10];
          if (algorithm == 2)
            {
              processmexican(z2D16, numFrame);
            }
          if (algorithm == 1)
            {
              __printInfo("Into the algorithm");
              processDavid(z2D16, numFrame, pmd3GT,&cResult, resultsFilename);
            }
          else
            cont_framesR++;
          //processRaquel(frameXData, frameYData, frameZData,&cResult, cont_framesR);

<<<<<<< tofCounting.c
         /* fprintf(resultsFp, "%06ld", (long int) numFrame);
          for (numUser = 0; numUser < cResult.numUsers; numUser++)
            {
              fprintf(resultsFp, " %d %4d %4d %4f", cResult.numUsers,
                      cResult.x[numUser], cResult.y[numUser],
                      cResult.resultado[numUser]);
            }
          fprintf(resultsFp, "\n");*/
=======
          /* fprintf(resultsFp, "%06ld", (long int) numFrame);
             for (numUser = 0; numUser < cResult.numUsers; numUser++)
             {
             fprintf(resultsFp, " %d %4d %4d %4f", cResult.numUsers,
             cResult.x[numUser], cResult.y[numUser],
             cResult.resultado[numUser]);
             }
             fprintf(resultsFp, "\n");*/
>>>>>>> 1.12
        }
      //fclose(resultsFp);

      __freePMD3data(&pmd3Data, cameraType);
    }

  fclose(fp);

  return 0;
}
