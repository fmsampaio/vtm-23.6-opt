#include "OptTechDT.h"

double OptTechDT::calculateDiffVariance(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff, PelUnitBuf recoBuff) {
    int startx = xBlk;
    int endx = xBlk + wBlk;
    int starty = yBlk;
    int endy = yBlk + hBlk;

    int sum = 0;

    for (int i = startx; i < endx; i++) {
      for (int j = starty; j < endy; j++) {
        sum += abs((origBuff.Y().at(i,j) >> 2) - (recoBuff.Y().at(i,j) >> 2));
      }
    }

    double mean = (double) sum / (double) (wBlk * hBlk);

    double sqDiff = 0;

    for (int i = startx; i < endx; i++) {
      for (int j = starty; j < endy; j++) {
        sqDiff += abs(abs((origBuff.Y().at(i,j) >> 2) - (recoBuff.Y().at(i,j) >> 2)) - mean) *
                  abs(abs((origBuff.Y().at(i,j) >> 2) - (recoBuff.Y().at(i,j) >> 2)) - mean);
      }
    }

    double variance = (double) sqDiff / (double) (wBlk * hBlk);

    return variance;
}

double OptTechDT::calculateBlockVariance(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff) {
    int startx = xBlk;
    int endx = xBlk + wBlk;
    int starty = yBlk;
    int endy = yBlk + hBlk;

    int sum = 0;

    for (int i = startx; i < endx; i++) {
      for (int j = starty; j < endy; j++) {
        sum += (origBuff.Y().at(i,j) >> 2);
      }
    }

    double mean = (double) sum / (double) (wBlk * hBlk);

    double sqDiff = 0;

    for (int i = startx; i < endx; i++) {
      for (int j = starty; j < endy; j++) {
        sqDiff += abs((origBuff.Y().at(i,j) >> 2) - mean) *
                  abs((origBuff.Y().at(i,j) >> 2) - mean);
      }
    }

    double variance = (double) sqDiff / (double) (wBlk * hBlk);

    return variance;
}

void OptTechDT::debugVarianceCalculation(int xBlk, int yBlk, int wBlk, int hBlk, PelUnitBuf origBuff, PelUnitBuf recoBuff) {
    int startx = xBlk;
    int endx = xBlk + wBlk;
    int starty = yBlk;
    int endy = yBlk + hBlk;

    std::cout << "Orig:" << std::endl;
    for (int i = startx; i < endx; i++) {
      for (int j = starty; j < endy; j++) {
        std::cout << origBuff.Y().at(i,j) << ";";
      }
      std::cout << std::endl;
    }
    std::cout << "Reco:" << std::endl;
    for (int i = startx; i < endx; i++) {
      for (int j = starty; j < endy; j++) {
        std::cout << recoBuff.Y().at(i,j) << ";";
      }
      std::cout << std::endl;
    }
    std::cout << "BlockVar: " << OptTechDT::calculateBlockVariance(xBlk, yBlk, wBlk, hBlk, origBuff) << std::endl;
    std::cout << "DiffVar: " << OptTechDT::calculateDiffVariance(xBlk, yBlk, wBlk, hBlk, origBuff, recoBuff) << std::endl;
}



