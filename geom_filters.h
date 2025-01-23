void gaussian_filter(Mat & original_send);
Mat mexican_filter(Mat & original_send, int newFilter);
void slice_image(void);
Mat preProcess(Mat & original);
Mat preProcessROI(Mat & originalROI);
void mexicanMaximos(Mat & mexicanFilter, Mat & Imprepro);
void mexic(Mat & mexicanFilter, Mat & image, int frame,int *numpersonas);
int mexResult(char *FileName, int contFrame, int posX[], int posY[],
              int personas);
void eliminasuelo(Mat & original);
void erosion(Mat & image, int erosion_size);
