/*
 * libSkin.c
 *
 * Author: Andrea Grandi
 */

#include "libSkin.h"

double colorCovarMat[3][3];
double colorCovarMatInv[3][3];
double meanColorVec[3];

Point3d sampleVec[SAMPLE_VEC_SIZE];
int sampleVecIndex = 0;
 
void getSkinSeed(PointCloud pcl, int* seedSkinX, int* seedSkinY) {
	
	int maxSkinPixelCount = 0;
	int curSkinPixelCount;
	int u, v;
	
	Point3d globalSkinColor;
	globalSkinColor.blue = GLOBAL_SKIN_BLUE;
	globalSkinColor.green = GLOBAL_SKIN_GREEN;
	globalSkinColor.red = GLOBAL_SKIN_RED;
	
	
	
	for (u=0; u<FREENECT_FRAME_W; u++) {
		curSkinPixelCount = 0;
		for (v=0; v<FREENECT_FRAME_H; v++) {
			if (pcl[v][u].valid && colorRgbDistance(pcl[v][u], globalSkinColor) < GLOBAL_SKIN_COLOR_THRESHOLD) {
				curSkinPixelCount++;
			}
		}
		
		if (curSkinPixelCount > maxSkinPixelCount) {
			maxSkinPixelCount = curSkinPixelCount;
			*seedSkinX = u;
		}
	}
	
	
	maxSkinPixelCount = 0;
	for (v=0; v<FREENECT_FRAME_H; v++) {
		
		curSkinPixelCount = 0;
		for (u=0; u<FREENECT_FRAME_W; u++) {
			if (pcl[v][u].valid && colorRgbDistance(pcl[v][u], globalSkinColor) < GLOBAL_SKIN_COLOR_THRESHOLD) {
				curSkinPixelCount++;
			}
		}
		
		if (curSkinPixelCount > maxSkinPixelCount) {
			maxSkinPixelCount = curSkinPixelCount;
			*seedSkinY = v;
		}	
	}
}


void regionGrowing(PointCloud src, PointCloud dst, int threshold) {
	
	int i,j,m,n,again;
	
	do {
		
		again = 0;
		for(j=0; j<FREENECT_FRAME_H; j++) {
			for(i=0; i<FREENECT_FRAME_W; i++) {
				if (dst[j][i].valid) {
					for (n=j-1; n<=j+1; n++) {
						for (m=i-1; m<=i+1; m++) {
							if (!dst[n][m].valid && src[n][m].valid && 
								colorMahalanobisDistance(src[n][m], meanColorVec, colorCovarMatInv) < threshold ) {
								
								dst[n][m] = src[n][m];
								again = 1;
							}
						}
					}
				}
			}
		}
		
		for(j=FREENECT_FRAME_H-1; j>=0; j--) {
			for(i=FREENECT_FRAME_W-1; i>=0; i--) {
				if (dst[j][i].valid) {
					for (n=j-1; n<=j+1; n++) {
						for (m=i-1; m<=i+1; m++) {
							if (!dst[n][m].valid && src[n][m].valid && 
								colorMahalanobisDistance(src[n][m], meanColorVec, colorCovarMatInv) < threshold ) {
								
								dst[n][m] = src[n][m];
								again = 1;
							}
						}
					}
				}
			}
		} 
		
	} while (again);	
}


int grabSkinSample(PointCloud pcl, int seedSkinX, int seedSkinY, int sampleSize) {
	double meanY = 0;
	double meanU = 0;
	double meanV = 0;
	
	int i,j,m,n;

	
	for (n=(seedSkinY-sampleSize); n<=(seedSkinY+sampleSize); n++) {
		for (m=(seedSkinX-sampleSize); m<=(seedSkinX+sampleSize); m++) {
			if (pcl[n][m].valid && sampleVecIndex < SAMPLE_VEC_SIZE) {
				sampleVec[sampleVecIndex] = pcl[n][m]; 
				sampleVecIndex++;
				
				// printf("Y: %f  U: %f  V: %f\n", curPcl[n][m].Y, curPcl[n][m].U, curPcl[n][m].V);
			}
		}
	}
	
	for (i=0; i<sampleVecIndex; i++) {
		meanY += sampleVec[i].Y;
		meanU += sampleVec[i].U;
		meanV += sampleVec[i].V;
	}
	
	meanY = meanY / sampleVecIndex;
	meanU = meanU / sampleVecIndex;
	meanV = meanV / sampleVecIndex;
	
	
	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			colorCovarMat[i][j] = 0;
		}
	}
	
	
	
	for (i=0; i<sampleVecIndex; i++) {
		colorCovarMat[0][0] += (sampleVec[i].Y - meanY)*(sampleVec[i].Y - meanY);
		colorCovarMat[0][1] += (sampleVec[i].Y - meanY)*(sampleVec[i].U - meanU);
		colorCovarMat[0][2] += (sampleVec[i].Y - meanY)*(sampleVec[i].V - meanV);
		
		colorCovarMat[1][0] += (sampleVec[i].U - meanU)*(sampleVec[i].Y - meanY);
		colorCovarMat[1][1] += (sampleVec[i].U - meanU)*(sampleVec[i].U - meanU);
		colorCovarMat[1][2] += (sampleVec[i].U - meanU)*(sampleVec[i].V - meanV);
		
		colorCovarMat[2][0] += (sampleVec[i].V - meanV)*(sampleVec[i].Y - meanY);
		colorCovarMat[2][1] += (sampleVec[i].V - meanV)*(sampleVec[i].U - meanU);
		colorCovarMat[2][2] += (sampleVec[i].V - meanV)*(sampleVec[i].V - meanV);
	}
	
	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			colorCovarMat[i][j] = colorCovarMat[i][j] / sampleVecIndex;
		}
	}
	
	
	
	double detCovarMat;
	if (detCovarMat = invertMat(colorCovarMat, colorCovarMatInv)) {
		meanColorVec[0] = meanY;
		meanColorVec[1] = meanU;
		meanColorVec[2] = meanV;
		return 1;
	}
	return 0;
}

void printSampleInfo() {
	int i, j;
	
	printf("SampleSize = %d\n", sampleVecIndex);
	printf("Y = %f\tU = %f\tV = %f\n", meanColorVec[0], meanColorVec[1], meanColorVec[2]);
	
	
	printf("\n\n========== Covar =========");
	for(i=0;i<3;i++) {
		printf("\n");
		for(j=0;j<3;j++) {     
			printf(" C[%d][%d]= %f",i,j, colorCovarMat[i][j]);
		}
	}
	printf("\n===========================================================\n\n");
		
	printf("\n========== CovarInv ==========\n");
	for(i=0;i<3;i++) {
		printf("\n");
		for(j=0;j<3;j++) {     
			printf(" C[%d][%d]= %f",i,j, colorCovarMatInv[i][j]);
		}
	}
	printf("\n===========================================================\n\n");
}


void resetSkinSample() {
	sampleVecIndex = 0;
	
}



