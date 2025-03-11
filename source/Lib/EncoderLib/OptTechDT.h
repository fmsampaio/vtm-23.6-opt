#include <fstream>

#include "CommonLib/Picture.h"

#define DBG_REPORT_DEPTH_MAPS 1
#define DBG_REPORT_VARIANCE 0

#define DEPTH_MAP_RESOLUTION 4

class OptTechDT {
    private:
        static int width, height, numOfFrames, depthMapAllocSize;
        static std::map<int, int*> depthMaps;

    public:

        static void init(int w, int h, int nf);

        static double calculateDiffVariance(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff, PelUnitBuf recoBuff);
        static double calculateBlockVariance(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff);
        static void debugVarianceCalculation(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff, PelUnitBuf recoBuff);

        static void updateDepthMap(int framePoc, int xBlk, int yBlk, int wBlk, int hBlk, int depth);
        static void reportDepthMap(int framePoc);

        static bool isPreviousSplit(int refFramePoc, int xCU, int yCU, int currDepth);

};