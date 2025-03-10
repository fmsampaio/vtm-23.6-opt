#include "CommonLib/Picture.h"

class OptTechDT {
    private:

    public:

        static double calculateDiffVariance(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff, PelUnitBuf recoBuff);
        static double calculateBlockVariance(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff);

        static void debugVarianceCalculation(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff, PelUnitBuf recoBuff);

};