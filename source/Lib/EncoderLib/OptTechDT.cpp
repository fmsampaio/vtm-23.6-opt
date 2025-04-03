#include "OptTechDT.h"

int OptTechDT::width, OptTechDT::height, OptTechDT::numOfFrames, OptTechDT::depthMapAllocSize, OptTechDT::quantPar;
std::map<int, int*> OptTechDT::depthMaps;
int OptTechDT::encoderConfig;

bool OptTechDT::skipCheckRD, OptTechDT::confidenceDT;

std::set<int> OptTechDT::dbgRefPics[100];

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

    skipCheckRD = false;

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
        // dbgRefPics[currFramePoc].insert(slice->getRefPOC(REF_PIC_LIST_0, i));
        if(slice->getRefPOC(REF_PIC_LIST_0, i) == refFramePoc) {
            return slice->getRefPic(REF_PIC_LIST_0, i)->getRecoBuf(PIC_RECONSTRUCTION);
        }
    }

    for (int i = 0; i < MAX_NUM_REF; i++) {
        // dbgRefPics[currFramePoc].insert(slice->getRefPOC(REF_PIC_LIST_1, i));
        if(slice->getRefPOC(REF_PIC_LIST_1, i) == refFramePoc) {
            return slice->getRefPic(REF_PIC_LIST_1, i)->getRecoBuf(PIC_RECONSTRUCTION);
        }
    }

    std::cout << "[ERR] No reference picture found!\n";
    return slice->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf(PIC_RECONSTRUCTION);
}

void OptTechDT::reportRefPicsDbg() {
  for (int f = 0; f < numOfFrames; f++) {
    std::cout << "FRAME " << f << " : ";
    for(auto x: dbgRefPics[f]) {
        std::cout << x << " ";
    }
    std::cout << std::endl;
  }  
}

void OptTechDT::performModelDT(int currQtDepth, int ft_qp, double ft_diffVar, int ft_previousSplit, int ft_height) {
#if ENABLE_TIME_PROFILE
  TimeProfiler::start(DT_MODEL);
#endif

  switch(currQtDepth) {
    case 0: //128x128
      if (ft_previousSplit <= 0.5000) {
          if (ft_diffVar <= 14.7253) {
              if (ft_qp <= 24.5000) {
                  skipCheckRD = true;
                  confidenceDT = false;
              } else {  // if ft_qp > 24.5000
                  skipCheckRD = true;
                  confidenceDT = true;
              }
          } else {  // if ft_diffVar > 14.7253
              skipCheckRD = true;
              confidenceDT = false;
          }
      } else {  // if ft_previousSplit > 0.5000
          if (ft_diffVar <= 36.2974) {
              if (ft_qp <= 24.5000) {
                  skipCheckRD = false;
                  confidenceDT = false;
              } else {  // if ft_qp > 24.5000
                  if (ft_diffVar <= 7.4873) {
                      skipCheckRD = true;
                      confidenceDT = false;
                  } else {  // if ft_diffVar > 7.4873
                      skipCheckRD = false;
                      confidenceDT = false;
                  }
              }
          } else {  // if ft_diffVar > 36.2974
              if (ft_qp <= 29.5000) {
                  if (ft_qp <= 24.5000) {
                      skipCheckRD = false;
                      confidenceDT = true;
                  } else {  // if ft_qp > 24.5000
                      skipCheckRD = false;
                      confidenceDT = true;
                  }
              } else {  // if ft_qp > 29.5000
                  skipCheckRD = false;
                  confidenceDT = false;
              }
          }
      }
      break;

    case 1: //64x64
      if (ft_previousSplit <= 0.5000) {
          if (ft_diffVar <= 27.3018) {
              if (ft_qp <= 24.5000) {
                  if (ft_diffVar <= 4.7214) {
                      skipCheckRD = true;
                      confidenceDT = true;
                  } else {  // if ft_diffVar > 4.7214
                      skipCheckRD = true;
                      confidenceDT = false;
                  }
              } else {  // if ft_qp > 24.5000
                  skipCheckRD = true;
                  confidenceDT = true;
              }
          } else {  // if ft_diffVar > 27.3018
              if (ft_qp <= 24.5000) {
                  skipCheckRD = false;
                  confidenceDT = false;
              } else {  // if ft_qp > 24.5000
                  if (ft_height <= 1620.0000) {
                      skipCheckRD = true;
                      confidenceDT = false;
                  } else {  // if ft_height > 1620.0000
                      if (ft_qp <= 29.5000) {
                          skipCheckRD = true;
                          confidenceDT = false;
                      } else {  // if ft_qp > 29.5000
                          skipCheckRD = true;
                          confidenceDT = false;
                      }
                  }
              }
          }
      } else {  // if ft_previousSplit > 0.5000
          if (ft_diffVar <= 28.4441) {
              skipCheckRD = false;
              confidenceDT = false;
          } else {  // if ft_diffVar > 28.4441
              if (ft_qp <= 29.5000) {
                  if (ft_height <= 1620.0000) {
                      skipCheckRD = false;
                      confidenceDT = true;
                  } else {  // if ft_height > 1620.0000
                      skipCheckRD = false;
                      confidenceDT = true;
                  }
              } else {  // if ft_qp > 29.5000
                  if (ft_height <= 1620.0000) {
                      skipCheckRD = false;
                      confidenceDT = true;
                  } else {  // if ft_height > 1620.0000
                      skipCheckRD = false;
                      confidenceDT = false;
                  }
              }
          }
      }
      break;

    case 2: //32x32
      if (ft_previousSplit <= 0.5000) {
          if (ft_diffVar <= 41.4639) {
              if (ft_diffVar <= 10.5852) {
                  if (ft_qp <= 24.5000) {
                      if (ft_diffVar <= 5.6595) {
                          skipCheckRD = true;
                          confidenceDT = true;
                      } else {  // if ft_diffVar > 5.6595
                          if (ft_height <= 1620.0000) {
                              skipCheckRD = true;
                              confidenceDT = false;
                          } else {  // if ft_height > 1620.0000
                              skipCheckRD = true;
                              confidenceDT = true;
                          }
                      }
                  } else {  // if ft_qp > 24.5000
                      skipCheckRD = true;
                      confidenceDT = true;
                  }
              } else {  // if ft_diffVar > 10.5852
                  if (ft_qp <= 29.5000) {
                      if (ft_height <= 1620.0000) {
                          skipCheckRD = true;
                          confidenceDT = false;
                      } else {  // if ft_height > 1620.0000
                          if (ft_qp <= 24.5000) {
                              skipCheckRD = true;
                              confidenceDT = false;
                          } else {  // if ft_qp > 24.5000
                              skipCheckRD = true;
                              confidenceDT = true;
                          }
                      }
                  } else {  // if ft_qp > 29.5000
                      skipCheckRD = true;
                      confidenceDT = true;
                  }
              }
          } else {  // if ft_diffVar > 41.4639
              if (ft_height <= 1620.0000) {
                  if (ft_qp <= 29.5000) {
                      if (ft_height <= 780.0000) {
                          skipCheckRD = false;
                          confidenceDT = true;
                      } else {  // if ft_height > 780.0000
                          if (ft_diffVar <= 187.1892) {
                              skipCheckRD = false;
                              confidenceDT = false;
                          } else {  // if ft_diffVar > 187.1892
                              skipCheckRD = false;
                              confidenceDT = false;
                          }
                      }
                  } else {  // if ft_qp > 29.5000
                      if (ft_diffVar <= 210.8311) {
                          if (ft_qp <= 34.5000) {
                              skipCheckRD = true;
                              confidenceDT = false;
                          } else {  // if ft_qp > 34.5000
                              skipCheckRD = true;
                              confidenceDT = true;
                          }
                      } else {  // if ft_diffVar > 210.8311
                          if (ft_qp <= 34.5000) {
                              skipCheckRD = false;
                              confidenceDT = false;
                          } else {  // if ft_qp > 34.5000
                              skipCheckRD = true;
                              confidenceDT = false;
                          }
                      }
                  }
              } else {  // if ft_height > 1620.0000
                  if (ft_qp <= 29.5000) {
                      if (ft_qp <= 24.5000) {
                          if (ft_diffVar <= 110.9316) {
                              skipCheckRD = true;
                              confidenceDT = false;
                          } else {  // if ft_diffVar > 110.9316
                              skipCheckRD = false;
                              confidenceDT = false;
                          }
                      } else {  // if ft_qp > 24.5000
                          if (ft_diffVar <= 111.0902) {
                              skipCheckRD = true;
                              confidenceDT = false;
                          } else {  // if ft_diffVar > 111.0902
                              skipCheckRD = true;
                              confidenceDT = false;
                          }
                      }
                  } else {  // if ft_qp > 29.5000
                      if (ft_qp <= 34.5000) {
                          skipCheckRD = true;
                          confidenceDT = false;
                      } else {  // if ft_qp > 34.5000
                          skipCheckRD = true;
                          confidenceDT = true;
                      }
                  }
              }
          }
      } else {  // if ft_previousSplit > 0.5000
          if (ft_height <= 1620.0000) {
              if (ft_diffVar <= 166.2231) {
                  skipCheckRD = false;
                  confidenceDT = true;
              } else {  // if ft_diffVar > 166.2231
                  skipCheckRD = false;
                  confidenceDT = true;
              }
          } else {  // if ft_height > 1620.0000
              if (ft_qp <= 24.5000) {
                  skipCheckRD = false;
                  confidenceDT = true;
              } else {  // if ft_qp > 24.5000
                  skipCheckRD = false;
                  confidenceDT = false;
              }
          }
      }
      break;

    case 3: //16x16
      if (ft_diffVar <= 164.5650) {
          if (ft_diffVar <= 22.9150) {
              skipCheckRD = true;
              confidenceDT = true;
          } else {  // if ft_diffVar > 22.9150
              if (ft_qp <= 24.5000) {
                  skipCheckRD = true;
                  confidenceDT = false;
              } else {  // if ft_qp > 24.5000
                  skipCheckRD = true;
                  confidenceDT = true;
              }
          }
      } else {  // if ft_diffVar > 164.5650
          if (ft_height <= 1620.0000) {
              if (ft_qp <= 29.5000) {
                  skipCheckRD = false;
                  confidenceDT = true;
              } else {  // if ft_qp > 29.5000
                  skipCheckRD = true;
                  confidenceDT = false;
              }
          } else {  // if ft_height > 1620.0000
              skipCheckRD = true;
              confidenceDT = false;
          }
      }
      
  }
#if ENABLE_TIME_PROFILE
  TimeProfiler::stop(DT_MODEL);
#endif
}