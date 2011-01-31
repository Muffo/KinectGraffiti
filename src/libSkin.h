/*
 * libSkin.h
 *
 * Author: muffo
 */

#include "libMyKinect.h"

#define GLOBAL_SKIN_BLUE 207
#define GLOBAL_SKIN_GREEN 208
#define GLOBAL_SKIN_RED 239

#define GLOBAL_SKIN_COLOR_THRESHOLD 200

#define SAMPLE_VEC_SIZE 100000


void getSkinSeed(PclImage pcl, int* seedSkinX, int* seedSkinY);
void regionGrowing(PclImage src, PclImage dst, int threshold);
void resetSkinSample();

int grabSkinSample(PclImage pcl, int seedSkinX, int seedSkinY, int sampleSize);
void printSampleInfo();
