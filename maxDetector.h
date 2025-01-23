
typedef struct
  {
    Point loc;
    int numSubROI;

  } maxCandidateSt;

typedef struct
  {
       float **pointsReturn;
      vector < Point > position;
      vector < Point > subroipos;
      vector < int >alturas;
    int PCAresult[10];
    int numMax;
  } maxData;

int maxDetector(Mat& image, maxData *maxReturn, int *numMax, int cont_frames,  double *busquedaroi, double *detectormaximos,double  *extractcaract);
