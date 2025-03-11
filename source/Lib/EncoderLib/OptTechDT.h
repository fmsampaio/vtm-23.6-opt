#include <fstream>

#include "CommonLib/Picture.h"

#define DBG_REPORT_DEPTH_MAPS 1
#define DBG_REPORT_VARIANCE 0

#define DEPTH_MAP_RESOLUTION 4

#define ENCODER_RA_CONFIG 0
#define ENCODER_LD_CONFIG 1

const int REFERENCE_FRAME_ORDER[2][33] = {
    {-1, 2, 4, 2, 8, 6, 4, 6, 16, 10, 8, 10, 8,  14, 12, 14, 32, 18, 20, 18, 24, 22, 20, 22, 16, 26, 28, 26, 24, 30, 28, 30, 0},
    {-1, 0, 1, 2, 3, 4, 5, 6, 7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31}
};

class OptTechDT {
    private:
        static int width, height, numOfFrames, depthMapAllocSize;
        static std::map<int, int*> depthMaps;
        static int encoderConfig;

    public:
        static void init(int w, int h, int nf, std::string encCfg);

        static double calculateDiffVariance(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff, PelUnitBuf recoBuff);
        static double calculateBlockVariance(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff);
        static void debugVarianceCalculation(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff, PelUnitBuf recoBuff);

        static void updateDepthMap(int framePoc, int xBlk, int yBlk, int wBlk, int hBlk, int depth);
        static void reportDepthMap(int framePoc);

        static bool isPreviousSplit(int refFramePoc, int xCU, int yCU, int currDepth);
};