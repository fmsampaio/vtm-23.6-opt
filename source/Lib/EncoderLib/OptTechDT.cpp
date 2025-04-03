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

void OptTechDT::performModelDT(int currQtDepth, int ft_qp, double ft_diffVar, double ft_blockVar, int ft_previousSplit, int ft_height, int ft_config) {
#if ENABLE_TIME_PROFILE
  TimeProfiler::start(DT_MODEL);
#endif

  switch(currQtDepth) {
    case 0: //128x128
      if (ft_previousSplit <= 0.5000) {
        if (ft_config <= 0.5000) {
            if (ft_diffVar <= 51.9689) {
                skipCheckRD = true;
                confidenceDT = true;
            } else {  // if ft_diffVar > 51.9689
                if (ft_diffVar <= 322.4163) {
                    skipCheckRD = true;
                    confidenceDT = true;
                } else {  // if ft_diffVar > 322.4163
                    skipCheckRD = true;
                    confidenceDT = false;
                }
            }
        } else {  // if ft_config > 0.5000
            if (ft_diffVar <= 12.1774) {
                if (ft_qp <= 24.5000) {
                    if (ft_diffVar <= 2.9830) {
                        skipCheckRD = true;
                        confidenceDT = true;
                    } else {  // if ft_diffVar > 2.9830
                        skipCheckRD = false;
                        confidenceDT = false;
                    }
                } else {  // if ft_qp > 24.5000
                    if (ft_diffVar <= 6.8096) {
                        skipCheckRD = true;
                        confidenceDT = true;
                    } else {  // if ft_diffVar > 6.8096
                        skipCheckRD = true;
                        confidenceDT = true;
                    }
                }
            } else {  // if ft_diffVar > 12.1774
                if (ft_qp <= 24.5000) {
                    skipCheckRD = false;
                    confidenceDT = false;
                } else {  // if ft_qp > 24.5000
                    if (ft_blockVar <= 161.1719) {
                        skipCheckRD = true;
                        confidenceDT = true;
                    } else {  // if ft_blockVar > 161.1719
                        if (ft_qp <= 34.5000) {
                            skipCheckRD = true;
                            confidenceDT = false;
                        } else {  // if ft_qp > 34.5000
                            skipCheckRD = true;
                            confidenceDT = false;
                        }
                    }
                }
            }
        }
    } else {  // if ft_previousSplit > 0.5000
        if (ft_blockVar <= 251.9057) {
            if (ft_qp <= 24.5000) {
                if (ft_blockVar <= 51.8617) {
                    if (ft_config <= 0.5000) {
                        skipCheckRD = true;
                        confidenceDT = false;
                    } else {  // if ft_config > 0.5000
                        if (ft_diffVar <= 2.5479) {
                            skipCheckRD = true;
                            confidenceDT = true;
                        } else {  // if ft_diffVar > 2.5479
                            skipCheckRD = false;
                            confidenceDT = false;
                        }
                    }
                } else {  // if ft_blockVar > 51.8617
                    if (ft_diffVar <= 5.2750) {
                        skipCheckRD = false;
                        confidenceDT = false;
                    } else {  // if ft_diffVar > 5.2750
                        if (ft_config <= 0.5000) {
                            skipCheckRD = false;
                            confidenceDT = true;
                        } else {  // if ft_config > 0.5000
                            skipCheckRD = false;
                            confidenceDT = true;
                        }
                    }
                }
            } else {  // if ft_qp > 24.5000
                if (ft_diffVar <= 13.9045) {
                    skipCheckRD = true;
                    confidenceDT = true;
                } else {  // if ft_diffVar > 13.9045
                    if (ft_qp <= 29.5000) {
                        if (ft_blockVar <= 66.5874) {
                            skipCheckRD = true;
                            confidenceDT = false;
                        } else {  // if ft_blockVar > 66.5874
                            skipCheckRD = false;
                            confidenceDT = false;
                        }
                    } else {  // if ft_qp > 29.5000
                        if (ft_blockVar <= 65.2714) {
                            skipCheckRD = true;
                            confidenceDT = false;
                        } else {  // if ft_blockVar > 65.2714
                            skipCheckRD = false;
                            confidenceDT = false;
                        }
                    }
                }
            }
        } else {  // if ft_blockVar > 251.9057
            if (ft_qp <= 29.5000) {
                if (ft_qp <= 24.5000) {
                    if (ft_diffVar <= 4.8890) {
                        skipCheckRD = false;
                        confidenceDT = false;
                    } else {  // if ft_diffVar > 4.8890
                        if (ft_config <= 0.5000) {
                            if (ft_diffVar <= 153.2352) {
                                skipCheckRD = false;
                                confidenceDT = true;
                            } else {  // if ft_diffVar > 153.2352
                                skipCheckRD = false;
                                confidenceDT = true;
                            }
                        } else {  // if ft_config > 0.5000
                            skipCheckRD = false;
                            confidenceDT = true;
                        }
                    }
                } else {  // if ft_qp > 24.5000
                    if (ft_diffVar <= 7.6143) {
                        skipCheckRD = true;
                        confidenceDT = false;
                    } else {  // if ft_diffVar > 7.6143
                        if (ft_diffVar <= 108.1304) {
                            if (ft_config <= 0.5000) {
                                skipCheckRD = false;
                                confidenceDT = false;
                            } else {  // if ft_config > 0.5000
                                skipCheckRD = false;
                                confidenceDT = true;
                            }
                        } else {  // if ft_diffVar > 108.1304
                            skipCheckRD = false;
                            confidenceDT = true;
                        }
                    }
                }
            } else {  // if ft_qp > 29.5000
                if (ft_diffVar <= 43.4114) {
                    if (ft_config <= 0.5000) {
                        skipCheckRD = true;
                        confidenceDT = false;
                    } else {  // if ft_config > 0.5000
                        if (ft_diffVar <= 16.6720) {
                            skipCheckRD = true;
                            confidenceDT = false;
                        } else {  // if ft_diffVar > 16.6720
                            skipCheckRD = false;
                            confidenceDT = false;
                        }
                    }
                } else {  // if ft_diffVar > 43.4114
                    if (ft_height <= 1620.0000) {
                        skipCheckRD = false;
                        confidenceDT = true;
                    } else {  // if ft_height > 1620.0000
                        if (ft_blockVar <= 504.4937) {
                            if (ft_diffVar <= 111.6379) {
                                skipCheckRD = false;
                                confidenceDT = false;
                            } else {  // if ft_diffVar > 111.6379
                                skipCheckRD = false;
                                confidenceDT = false;
                            }
                        } else {  // if ft_blockVar > 504.4937
                            if (ft_qp <= 34.5000) {
                                skipCheckRD = false;
                                confidenceDT = false;
                            } else {  // if ft_qp > 34.5000
                                skipCheckRD = false;
                                confidenceDT = false;
                            }
                        }
                    }
                }
            }
        }
    }
      break;

    case 1: //64x64
      if (ft_previousSplit <= 0.5000) {
        if (ft_blockVar <= 149.3954) {
            if (ft_diffVar <= 14.0712) {
                if (ft_qp <= 24.5000) {
                    if (ft_blockVar <= 15.7463) {
                        skipCheckRD = true;
                        confidenceDT = true;
                    } else {  // if ft_blockVar > 15.7463
                        if (ft_config <= 0.5000) {
                            skipCheckRD = true;
                            confidenceDT = true;
                        } else {  // if ft_config > 0.5000
                            if (ft_diffVar <= 4.3160) {
                                skipCheckRD = true;
                                confidenceDT = true;
                            } else {  // if ft_diffVar > 4.3160
                                skipCheckRD = true;
                                confidenceDT = false;
                            }
                        }
                    }
                } else {  // if ft_qp > 24.5000
                    skipCheckRD = true;
                    confidenceDT = true;
                }
            } else {  // if ft_diffVar > 14.0712
                if (ft_qp <= 24.5000) {
                    if (ft_blockVar <= 38.9454) {
                        skipCheckRD = true;
                        confidenceDT = false;
                    } else {  // if ft_blockVar > 38.9454
                        if (ft_config <= 0.5000) {
                            skipCheckRD = true;
                            confidenceDT = false;
                        } else {  // if ft_config > 0.5000
                            skipCheckRD = false;
                            confidenceDT = false;
                        }
                    }
                } else {  // if ft_qp > 24.5000
                    if (ft_blockVar <= 44.7301) {
                        skipCheckRD = true;
                        confidenceDT = true;
                    } else {  // if ft_blockVar > 44.7301
                        if (ft_qp <= 29.5000) {
                            skipCheckRD = true;
                            confidenceDT = false;
                        } else {  // if ft_qp > 29.5000
                            skipCheckRD = true;
                            confidenceDT = true;
                        }
                    }
                }
            }
        } else {  // if ft_blockVar > 149.3954
            if (ft_qp <= 24.5000) {
                if (ft_config <= 0.5000) {
                    if (ft_diffVar <= 73.1256) {
                        if (ft_diffVar <= 5.8959) {
                            skipCheckRD = true;
                            confidenceDT = true;
                        } else {  // if ft_diffVar > 5.8959
                            skipCheckRD = true;
                            confidenceDT = false;
                        }
                    } else {  // if ft_diffVar > 73.1256
                        if (ft_diffVar <= 365.7495) {
                            skipCheckRD = true;
                            confidenceDT = false;
                        } else {  // if ft_diffVar > 365.7495
                            skipCheckRD = false;
                            confidenceDT = false;
                        }
                    }
                } else {  // if ft_config > 0.5000
                    if (ft_diffVar <= 27.4611) {
                        if (ft_height <= 1620.0000) {
                            if (ft_diffVar <= 3.3244) {
                                skipCheckRD = true;
                                confidenceDT = false;
                            } else {  // if ft_diffVar > 3.3244
                                skipCheckRD = false;
                                confidenceDT = false;
                            }
                        } else {  // if ft_height > 1620.0000
                            skipCheckRD = true;
                            confidenceDT = false;
                        }
                    } else {  // if ft_diffVar > 27.4611
                        skipCheckRD = false;
                        confidenceDT = false;
                    }
                }
            } else {  // if ft_qp > 24.5000
                if (ft_diffVar <= 30.5365) {
                    if (ft_diffVar <= 7.4251) {
                        skipCheckRD = true;
                        confidenceDT = true;
                    } else {  // if ft_diffVar > 7.4251
                        if (ft_qp <= 29.5000) {
                            if (ft_height <= 1620.0000) {
                                skipCheckRD = true;
                                confidenceDT = false;
                            } else {  // if ft_height > 1620.0000
                                skipCheckRD = true;
                                confidenceDT = false;
                            }
                        } else {  // if ft_qp > 29.5000
                            skipCheckRD = true;
                            confidenceDT = true;
                        }
                    }
                } else {  // if ft_diffVar > 30.5365
                    if (ft_height <= 1620.0000) {
                        if (ft_height <= 780.0000) {
                            skipCheckRD = false;
                            confidenceDT = false;
                        } else {  // if ft_height > 780.0000
                            if (ft_diffVar <= 370.7644) {
                                skipCheckRD = true;
                                confidenceDT = false;
                            } else {  // if ft_diffVar > 370.7644
                                skipCheckRD = false;
                                confidenceDT = false;
                            }
                        }
                    } else {  // if ft_height > 1620.0000
                        if (ft_qp <= 29.5000) {
                            if (ft_blockVar <= 276.8362) {
                                skipCheckRD = true;
                                confidenceDT = false;
                            } else {  // if ft_blockVar > 276.8362
                                skipCheckRD = true;
                                confidenceDT = false;
                            }
                        } else {  // if ft_qp > 29.5000
                            if (ft_qp <= 34.5000) {
                                if (ft_blockVar <= 329.9545) {
                                    skipCheckRD = true;
                                    confidenceDT = false;
                                } else {  // if ft_blockVar > 329.9545
                                    skipCheckRD = true;
                                    confidenceDT = false;
                                }
                            } else {  // if ft_qp > 34.5000
                                if (ft_blockVar <= 449.1371) {
                                    skipCheckRD = true;
                                    confidenceDT = true;
                                } else {  // if ft_blockVar > 449.1371
                                    skipCheckRD = true;
                                    confidenceDT = false;
                                }
                            }
                        }
                    }
                }
            }
        }
    } else {  // if ft_previousSplit > 0.5000
        if (ft_blockVar <= 100.3054) {
            if (ft_blockVar <= 34.5419) {
                if (ft_blockVar <= 20.8821) {
                    skipCheckRD = true;
                    confidenceDT = true;
                } else {  // if ft_blockVar > 20.8821
                    skipCheckRD = true;
                    confidenceDT = false;
                }
            } else {  // if ft_blockVar > 34.5419
                if (ft_qp <= 24.5000) {
                    if (ft_diffVar <= 15.9894) {
                        skipCheckRD = true;
                        confidenceDT = false;
                    } else {  // if ft_diffVar > 15.9894
                        skipCheckRD = false;
                        confidenceDT = false;
                    }
                } else {  // if ft_qp > 24.5000
                    skipCheckRD = true;
                    confidenceDT = false;
                }
            }
        } else {  // if ft_blockVar > 100.3054
            if (ft_qp <= 29.5000) {
                if (ft_height <= 1620.0000) {
                    if (ft_diffVar <= 6.9680) {
                        skipCheckRD = false;
                        confidenceDT = false;
                    } else {  // if ft_diffVar > 6.9680
                        if (ft_qp <= 24.5000) {
                            skipCheckRD = false;
                            confidenceDT = true;
                        } else {  // if ft_qp > 24.5000
                            if (ft_diffVar <= 37.2244) {
                                skipCheckRD = false;
                                confidenceDT = false;
                            } else {  // if ft_diffVar > 37.2244
                                skipCheckRD = false;
                                confidenceDT = true;
                            }
                        }
                    }
                } else {  // if ft_height > 1620.0000
                    if (ft_diffVar <= 43.1464) {
                        if (ft_diffVar <= 7.8173) {
                            skipCheckRD = true;
                            confidenceDT = true;
                        } else {  // if ft_diffVar > 7.8173
                            skipCheckRD = false;
                            confidenceDT = false;
                        }
                    } else {  // if ft_diffVar > 43.1464
                        if (ft_qp <= 24.5000) {
                            if (ft_blockVar <= 212.1818) {
                                skipCheckRD = false;
                                confidenceDT = false;
                            } else {  // if ft_blockVar > 212.1818
                                skipCheckRD = false;
                                confidenceDT = true;
                            }
                        } else {  // if ft_qp > 24.5000
                            if (ft_blockVar <= 294.6317) {
                                skipCheckRD = false;
                                confidenceDT = false;
                            } else {  // if ft_blockVar > 294.6317
                                skipCheckRD = false;
                                confidenceDT = true;
                            }
                        }
                    }
                }
            } else {  // if ft_qp > 29.5000
                if (ft_height <= 1620.0000) {
                    if (ft_diffVar <= 44.9782) {
                        skipCheckRD = true;
                        confidenceDT = false;
                    } else {  // if ft_diffVar > 44.9782
                        if (ft_qp <= 34.5000) {
                            skipCheckRD = false;
                            confidenceDT = true;
                        } else {  // if ft_qp > 34.5000
                            skipCheckRD = false;
                            confidenceDT = false;
                        }
                    }
                } else {  // if ft_height > 1620.0000
                    if (ft_diffVar <= 99.1020) {
                        skipCheckRD = true;
                        confidenceDT = false;
                    } else {  // if ft_diffVar > 99.1020
                        if (ft_qp <= 34.5000) {
                            skipCheckRD = false;
                            confidenceDT = false;
                        } else {  // if ft_qp > 34.5000
                            skipCheckRD = true;
                            confidenceDT = false;
                        }
                    }
                }
            }
        }
    }
      break;

    case 2: //32x32
      if (ft_blockVar <= 114.7089) {
        if (ft_blockVar <= 44.7852) {
            if (ft_blockVar <= 26.4709) {
                if (ft_blockVar <= 15.4942) {
                    skipCheckRD = true;
                    confidenceDT = true;
                } else {  // if ft_blockVar > 15.4942
                    if (ft_qp <= 24.5000) {
                        skipCheckRD = true;
                        confidenceDT = true;
                    } else {  // if ft_qp > 24.5000
                        skipCheckRD = true;
                        confidenceDT = true;
                    }
                }
            } else {  // if ft_blockVar > 26.4709
                if (ft_qp <= 24.5000) {
                    if (ft_diffVar <= 10.7010) {
                        skipCheckRD = true;
                        confidenceDT = true;
                    } else {  // if ft_diffVar > 10.7010
                        if (ft_previousSplit <= 0.5000) {
                            skipCheckRD = true;
                            confidenceDT = false;
                        } else {  // if ft_previousSplit > 0.5000
                            skipCheckRD = true;
                            confidenceDT = false;
                        }
                    }
                } else {  // if ft_qp > 24.5000
                    skipCheckRD = true;
                    confidenceDT = true;
                }
            }
        } else {  // if ft_blockVar > 44.7852
            if (ft_qp <= 24.5000) {
                if (ft_previousSplit <= 0.5000) {
                    if (ft_diffVar <= 21.0421) {
                        if (ft_diffVar <= 8.0051) {
                            skipCheckRD = true;
                            confidenceDT = true;
                        } else {  // if ft_diffVar > 8.0051
                            skipCheckRD = true;
                            confidenceDT = false;
                        }
                    } else {  // if ft_diffVar > 21.0421
                        if (ft_blockVar <= 56.4729) {
                            skipCheckRD = true;
                            confidenceDT = false;
                        } else {  // if ft_blockVar > 56.4729
                            if (ft_height <= 1620.0000) {
                                skipCheckRD = true;
                                confidenceDT = false;
                            } else {  // if ft_height > 1620.0000
                                skipCheckRD = true;
                                confidenceDT = false;
                            }
                        }
                    }
                } else {  // if ft_previousSplit > 0.5000
                    skipCheckRD = false;
                    confidenceDT = false;
                }
            } else {  // if ft_qp > 24.5000
                if (ft_qp <= 29.5000) {
                    if (ft_diffVar <= 19.1777) {
                        skipCheckRD = true;
                        confidenceDT = true;
                    } else {  // if ft_diffVar > 19.1777
                        if (ft_previousSplit <= 0.5000) {
                            if (ft_blockVar <= 81.9985) {
                                skipCheckRD = true;
                                confidenceDT = false;
                            } else {  // if ft_blockVar > 81.9985
                                skipCheckRD = true;
                                confidenceDT = false;
                            }
                        } else {  // if ft_previousSplit > 0.5000
                            skipCheckRD = false;
                            confidenceDT = false;
                        }
                    }
                } else {  // if ft_qp > 29.5000
                    if (ft_diffVar <= 73.1142) {
                        skipCheckRD = true;
                        confidenceDT = true;
                    } else {  // if ft_diffVar > 73.1142
                        if (ft_blockVar <= 97.1626) {
                            skipCheckRD = true;
                            confidenceDT = true;
                        } else {  // if ft_blockVar > 97.1626
                            skipCheckRD = true;
                            confidenceDT = false;
                        }
                    }
                }
            }
        }
    } else {  // if ft_blockVar > 114.7089
        if (ft_previousSplit <= 0.5000) {
            if (ft_height <= 1620.0000) {
                if (ft_diffVar <= 17.1810) {
                    if (ft_qp <= 24.5000) {
                        if (ft_diffVar <= 6.4512) {
                            if (ft_diffVar <= 4.7671) {
                                skipCheckRD = true;
                                confidenceDT = true;
                            } else {  // if ft_diffVar > 4.7671
                                skipCheckRD = true;
                                confidenceDT = false;
                            }
                        } else {  // if ft_diffVar > 6.4512
                            if (ft_config <= 0.5000) {
                                if (ft_blockVar <= 1181.7174) {
                                    skipCheckRD = true;
                                    confidenceDT = false;
                                } else {  // if ft_blockVar > 1181.7174
                                    skipCheckRD = false;
                                    confidenceDT = false;
                                }
                            } else {  // if ft_config > 0.5000
                                skipCheckRD = false;
                                confidenceDT = false;
                            }
                        }
                    } else {  // if ft_qp > 24.5000
                        if (ft_diffVar <= 10.7175) {
                            skipCheckRD = true;
                            confidenceDT = true;
                        } else {  // if ft_diffVar > 10.7175
                            if (ft_qp <= 29.5000) {
                                skipCheckRD = true;
                                confidenceDT = false;
                            } else {  // if ft_qp > 29.5000
                                skipCheckRD = true;
                                confidenceDT = true;
                            }
                        }
                    }
                } else {  // if ft_diffVar > 17.1810
                    if (ft_qp <= 29.5000) {
                        if (ft_blockVar <= 418.7058) {
                            if (ft_qp <= 24.5000) {
                                if (ft_height <= 780.0000) {
                                    skipCheckRD = false;
                                    confidenceDT = true;
                                } else {  // if ft_height > 780.0000
                                    if (ft_blockVar <= 256.7607) {
                                        if (ft_blockVar <= 122.9499) {
                                            skipCheckRD = true;
                                            confidenceDT = false;
                                        } else {  // if ft_blockVar > 122.9499
                                            skipCheckRD = false;
                                            confidenceDT = false;
                                        }
                                    } else {  // if ft_blockVar > 256.7607
                                        skipCheckRD = false;
                                        confidenceDT = false;
                                    }
                                }
                            } else {  // if ft_qp > 24.5000
                                if (ft_diffVar <= 37.5560) {
                                    skipCheckRD = true;
                                    confidenceDT = false;
                                } else {  // if ft_diffVar > 37.5560
                                    if (ft_height <= 780.0000) {
                                        skipCheckRD = false;
                                        confidenceDT = false;
                                    } else {  // if ft_height > 780.0000
                                        if (ft_blockVar <= 258.9149) {
                                            skipCheckRD = false;
                                            confidenceDT = false;
                                        } else {  // if ft_blockVar > 258.9149
                                            skipCheckRD = false;
                                            confidenceDT = false;
                                        }
                                    }
                                }
                            }
                        } else {  // if ft_blockVar > 418.7058
                            if (ft_height <= 780.0000) {
                                skipCheckRD = false;
                                confidenceDT = true;
                            } else {  // if ft_height > 780.0000
                                if (ft_diffVar <= 799.5457) {
                                    if (ft_diffVar <= 93.5568) {
                                        if (ft_config <= 0.5000) {
                                            skipCheckRD = false;
                                            confidenceDT = false;
                                        } else {  // if ft_config > 0.5000
                                            skipCheckRD = false;
                                            confidenceDT = false;
                                        }
                                    } else {  // if ft_diffVar > 93.5568
                                        skipCheckRD = false;
                                        confidenceDT = false;
                                    }
                                } else {  // if ft_diffVar > 799.5457
                                    skipCheckRD = false;
                                    confidenceDT = false;
                                }
                            }
                        }
                    } else {  // if ft_qp > 29.5000
                        if (ft_blockVar <= 403.6832) {
                            if (ft_qp <= 34.5000) {
                                if (ft_blockVar <= 192.7836) {
                                    skipCheckRD = true;
                                    confidenceDT = false;
                                } else {  // if ft_blockVar > 192.7836
                                    if (ft_diffVar <= 57.0165) {
                                        skipCheckRD = true;
                                        confidenceDT = false;
                                    } else {  // if ft_diffVar > 57.0165
                                        skipCheckRD = true;
                                        confidenceDT = false;
                                    }
                                }
                            } else {  // if ft_qp > 34.5000
                                skipCheckRD = true;
                                confidenceDT = true;
                            }
                        } else {  // if ft_blockVar > 403.6832
                            if (ft_qp <= 34.5000) {
                                if (ft_diffVar <= 130.8494) {
                                    skipCheckRD = true;
                                    confidenceDT = false;
                                } else {  // if ft_diffVar > 130.8494
                                    if (ft_blockVar <= 1159.4417) {
                                        if (ft_height <= 780.0000) {
                                            skipCheckRD = false;
                                            confidenceDT = false;
                                        } else {  // if ft_height > 780.0000
                                            skipCheckRD = false;
                                            confidenceDT = false;
                                        }
                                    } else {  // if ft_blockVar > 1159.4417
                                        skipCheckRD = false;
                                        confidenceDT = false;
                                    }
                                }
                            } else {  // if ft_qp > 34.5000
                                if (ft_diffVar <= 210.4262) {
                                    skipCheckRD = true;
                                    confidenceDT = false;
                                } else {  // if ft_diffVar > 210.4262
                                    if (ft_blockVar <= 1759.3176) {
                                        skipCheckRD = true;
                                        confidenceDT = false;
                                    } else {  // if ft_blockVar > 1759.3176
                                        skipCheckRD = false;
                                        confidenceDT = false;
                                    }
                                }
                            }
                        }
                    }
                }
            } else {  // if ft_height > 1620.0000
                if (ft_qp <= 29.5000) {
                    if (ft_diffVar <= 83.2103) {
                        if (ft_diffVar <= 12.4551) {
                            skipCheckRD = true;
                            confidenceDT = true;
                        } else {  // if ft_diffVar > 12.4551
                            if (ft_qp <= 24.5000) {
                                if (ft_config <= 0.5000) {
                                    if (ft_diffVar <= 29.1540) {
                                        skipCheckRD = true;
                                        confidenceDT = true;
                                    } else {  // if ft_diffVar > 29.1540
                                        skipCheckRD = true;
                                        confidenceDT = false;
                                    }
                                } else {  // if ft_config > 0.5000
                                    skipCheckRD = true;
                                    confidenceDT = false;
                                }
                            } else {  // if ft_qp > 24.5000
                                skipCheckRD = true;
                                confidenceDT = false;
                            }
                        }
                    } else {  // if ft_diffVar > 83.2103
                        if (ft_qp <= 24.5000) {
                            if (ft_diffVar <= 177.7908) {
                                skipCheckRD = false;
                                confidenceDT = false;
                            } else {  // if ft_diffVar > 177.7908
                                skipCheckRD = false;
                                confidenceDT = false;
                            }
                        } else {  // if ft_qp > 24.5000
                            if (ft_diffVar <= 209.5625) {
                                skipCheckRD = true;
                                confidenceDT = false;
                            } else {  // if ft_diffVar > 209.5625
                                if (ft_blockVar <= 189.1062) {
                                    skipCheckRD = true;
                                    confidenceDT = false;
                                } else {  // if ft_blockVar > 189.1062
                                    skipCheckRD = true;
                                    confidenceDT = false;
                                }
                            }
                        }
                    }
                } else {  // if ft_qp > 29.5000
                    if (ft_qp <= 34.5000) {
                        if (ft_diffVar <= 65.6295) {
                            skipCheckRD = true;
                            confidenceDT = true;
                        } else {  // if ft_diffVar > 65.6295
                            if (ft_blockVar <= 535.3096) {
                                skipCheckRD = true;
                                confidenceDT = false;
                            } else {  // if ft_blockVar > 535.3096
                                skipCheckRD = true;
                                confidenceDT = false;
                            }
                        }
                    } else {  // if ft_qp > 34.5000
                        if (ft_diffVar <= 175.3829) {
                            skipCheckRD = true;
                            confidenceDT = true;
                        } else {  // if ft_diffVar > 175.3829
                            if (ft_blockVar <= 1882.3442) {
                                skipCheckRD = true;
                                confidenceDT = true;
                            } else {  // if ft_blockVar > 1882.3442
                                skipCheckRD = true;
                                confidenceDT = false;
                            }
                        }
                    }
                }
            }
        } else {  // if ft_previousSplit > 0.5000
            if (ft_height <= 1620.0000) {
                if (ft_blockVar <= 497.0945) {
                    if (ft_qp <= 24.5000) {
                        if (ft_diffVar <= 7.1120) {
                            skipCheckRD = true;
                            confidenceDT = false;
                        } else {  // if ft_diffVar > 7.1120
                            skipCheckRD = false;
                            confidenceDT = true;
                        }
                    } else {  // if ft_qp > 24.5000
                        if (ft_qp <= 29.5000) {
                            if (ft_diffVar <= 48.4122) {
                                skipCheckRD = true;
                                confidenceDT = false;
                            } else {  // if ft_diffVar > 48.4122
                                skipCheckRD = false;
                                confidenceDT = false;
                            }
                        } else {  // if ft_qp > 29.5000
                            skipCheckRD = true;
                            confidenceDT = false;
                        }
                    }
                } else {  // if ft_blockVar > 497.0945
                    if (ft_qp <= 29.5000) {
                        if (ft_diffVar <= 28.8711) {
                            if (ft_diffVar <= 6.2232) {
                                skipCheckRD = true;
                                confidenceDT = false;
                            } else {  // if ft_diffVar > 6.2232
                                skipCheckRD = false;
                                confidenceDT = true;
                            }
                        } else {  // if ft_diffVar > 28.8711
                            if (ft_qp <= 24.5000) {
                                skipCheckRD = false;
                                confidenceDT = true;
                            } else {  // if ft_qp > 24.5000
                                skipCheckRD = false;
                                confidenceDT = true;
                            }
                        }
                    } else {  // if ft_qp > 29.5000
                        if (ft_qp <= 34.5000) {
                            skipCheckRD = false;
                            confidenceDT = true;
                        } else {  // if ft_qp > 34.5000
                            skipCheckRD = false;
                            confidenceDT = false;
                        }
                    }
                }
            } else {  // if ft_height > 1620.0000
                if (ft_qp <= 29.5000) {
                    if (ft_qp <= 24.5000) {
                        skipCheckRD = false;
                        confidenceDT = true;
                    } else {  // if ft_qp > 24.5000
                        skipCheckRD = false;
                        confidenceDT = false;
                    }
                } else {  // if ft_qp > 29.5000
                    skipCheckRD = true;
                    confidenceDT = false;
                }
            }
        }
    }
      break;

    case 3: //16x16
      if (ft_blockVar <= 108.2016) {
        if (ft_blockVar <= 45.4132) {
            if (ft_blockVar <= 18.9170) {
                skipCheckRD = true;
                confidenceDT = true;
            } else {  // if ft_blockVar > 18.9170
                if (ft_qp <= 24.5000) {
                    skipCheckRD = true;
                    confidenceDT = true;
                } else {  // if ft_qp > 24.5000
                    skipCheckRD = true;
                    confidenceDT = true;
                }
            }
        } else {  // if ft_blockVar > 45.4132
            if (ft_qp <= 24.5000) {
                if (ft_diffVar <= 6.2840) {
                    skipCheckRD = true;
                    confidenceDT = true;
                } else {  // if ft_diffVar > 6.2840
                    skipCheckRD = true;
                    confidenceDT = false;
                }
            } else {  // if ft_qp > 24.5000
                if (ft_diffVar <= 43.3210) {
                    skipCheckRD = true;
                    confidenceDT = true;
                } else {  // if ft_diffVar > 43.3210
                    skipCheckRD = true;
                    confidenceDT = false;
                }
            }
        }
    } else {  // if ft_blockVar > 108.2016
        if (ft_height <= 1620.0000) {
            if (ft_blockVar <= 414.6919) {
                if (ft_qp <= 29.5000) {
                    if (ft_diffVar <= 26.1699) {
                        skipCheckRD = true;
                        confidenceDT = false;
                    } else {  // if ft_diffVar > 26.1699
                        if (ft_qp <= 24.5000) {
                            if (ft_height <= 780.0000) {
                                skipCheckRD = false;
                                confidenceDT = true;
                            } else {  // if ft_height > 780.0000
                                skipCheckRD = false;
                                confidenceDT = false;
                            }
                        } else {  // if ft_qp > 24.5000
                            skipCheckRD = true;
                            confidenceDT = false;
                        }
                    }
                } else {  // if ft_qp > 29.5000
                    skipCheckRD = true;
                    confidenceDT = true;
                }
            } else {  // if ft_blockVar > 414.6919
                if (ft_qp <= 29.5000) {
                    if (ft_diffVar <= 13.9252) {
                        skipCheckRD = true;
                        confidenceDT = false;
                    } else {  // if ft_diffVar > 13.9252
                        if (ft_blockVar <= 3167.3330) {
                            if (ft_qp <= 24.5000) {
                                skipCheckRD = false;
                                confidenceDT = true;
                            } else {  // if ft_qp > 24.5000
                                skipCheckRD = false;
                                confidenceDT = false;
                            }
                        } else {  // if ft_blockVar > 3167.3330
                            skipCheckRD = false;
                            confidenceDT = true;
                        }
                    }
                } else {  // if ft_qp > 29.5000
                    if (ft_diffVar <= 133.8202) {
                        skipCheckRD = true;
                        confidenceDT = true;
                    } else {  // if ft_diffVar > 133.8202
                        skipCheckRD = false;
                        confidenceDT = false;
                    }
                }
            }
        } else {  // if ft_height > 1620.0000
            if (ft_qp <= 24.5000) {
                if (ft_diffVar <= 28.0521) {
                    skipCheckRD = true;
                    confidenceDT = false;
                } else {  // if ft_diffVar > 28.0521
                    skipCheckRD = true;
                    confidenceDT = false;
                }
            } else {  // if ft_qp > 24.5000
                if (ft_diffVar <= 75.3485) {
                    skipCheckRD = true;
                    confidenceDT = true;
                } else {  // if ft_diffVar > 75.3485
                    skipCheckRD = true;
                    confidenceDT = false;
                }
            }
        }
    }      
  }
#if ENABLE_TIME_PROFILE
  TimeProfiler::stop(DT_MODEL);
#endif
}