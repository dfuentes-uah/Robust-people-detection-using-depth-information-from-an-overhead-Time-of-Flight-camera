#include "defines.h"
typedef struct
{
    int Frame;
    int PCAresult[maxDETECTIONS];
    Point position[maxDETECTIONS];
    int numMax;
}st_resultSave;


int saveResults(char *FileName, int contFrame, double PCAres, int posX, int posY, int altura);
int saveResultsCounting(const char *FileName, maxData MaxResults, int Frame);
int pointsHeighRelation (int height, int points, int WR);
