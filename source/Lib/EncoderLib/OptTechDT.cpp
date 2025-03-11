#include "OptTechDT.h"

int OptTechDT::width, OptTechDT::height, OptTechDT::numOfFrames, OptTechDT::depthMapAllocSize, OptTechDT::quantPar;
std::map<int, int*> OptTechDT::depthMaps;
int OptTechDT::encoderConfig;

void OptTechDT::init(int w, int h, int nf, std::string encCfg, int qp) {
    width = w;
    height = h;
    numOfFrames = nf;
    encoderConfig = (encCfg == "RA") ? ENCODER_RA_CONFIG : ENCODER_LD_CONFIG;
    quantPar = qp;

    // std::cout << "[DBG] Encoder Configuration: " << encoderConfig << " " << encCfg << std::endl;

    depthMapAllocSize = (width / DEPTH_MAP_RESOLUTION) * (height / DEPTH_MAP_RESOLUTION);

    for (int f = 0; f < numOfFrames; f++) {
        depthMaps[f] = NULL;        
    }       
     
}


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

void OptTechDT::updateDepthMap(int framePoc, int xBlk, int yBlk, int wBlk, int hBlk, int depth) {

    if(depthMaps[framePoc] == NULL) {
        depthMaps[framePoc] = (int*) malloc(depthMapAllocSize * sizeof(int));
    }

    int xBegin = xBlk / DEPTH_MAP_RESOLUTION;
    int yBegin = yBlk / DEPTH_MAP_RESOLUTION;
    int xEnd = (xBlk + wBlk) / DEPTH_MAP_RESOLUTION;
    int yEnd = (yBlk + hBlk) / DEPTH_MAP_RESOLUTION;

    for (int x = xBegin; x < xEnd; x++) {
        for (int y = yBegin; y < yEnd; y++) {
            int pos = x + (y * (width / DEPTH_MAP_RESOLUTION));
            depthMaps[framePoc][pos] = depth;
        }
    }
}

int OptTechDT::isPreviousSplit(int currFramePoc, int xCU, int yCU, int currDepth) {
    int refFramePoc = REFERENCE_FRAME_ORDER[encoderConfig][currFramePoc];

    int pos = xCU + (yCU * (width / DEPTH_MAP_RESOLUTION));
    int refDepth = depthMaps[refFramePoc][pos];

    return (refDepth > currDepth) ? 1 : 0;
}

void OptTechDT::reportDepthMap(int framePoc) {
    if(depthMaps[framePoc] == NULL) {
        std::cout << "No depth map. Skipping...\n";
        return;
    }
    std::cout << "[DBG] DEPTH MAP REPORT\n";
    std::cout << "Frame " << framePoc << std::endl;
    for (int y = 0; y < (height / DEPTH_MAP_RESOLUTION); y++) {
        for (int x = 0; x < (width / DEPTH_MAP_RESOLUTION); x++) {
            int pos = x + (y * (width / DEPTH_MAP_RESOLUTION));
            std::cout << depthMaps[framePoc][pos] << " ";
        }
        std::cout << std::endl;
    }        
}

PelUnitBuf OptTechDT::getRefPicBuf(int currFramePoc, Slice* slice) {
    int refFramePoc = REFERENCE_FRAME_ORDER[encoderConfig][currFramePoc];

    for (int i = 0; i < MAX_NUM_REF; i++) {
        if(slice->getRefPOC(REF_PIC_LIST_0, i) == refFramePoc) {
            return slice->getRefPic(REF_PIC_LIST_0, i)->getRecoBuf(PIC_RECONSTRUCTION);
        }
    }

    std::cout << "[ERR] No reference picture found!\n";
    return slice->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf(PIC_RECONSTRUCTION);

}