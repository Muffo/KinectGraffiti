/*
 * graffiti.c
 *
 *  Created on: Dec 23, 2010
 *      Author: Andrea Grandi
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "libMyKinect.h"

int saveFlag;
int displayGuideFlag = 1;
int sampleSize = 30;
int grabSkinSampleFlag = 0;
int handFlag = 0;

int specSkinColorThreshold = 10;

double freenect_angle = 0;

PclImage curPcl;
PclImage handPcl;



#define SAMPLE_SIZE_STEP 3
#define COLOR_THRESHOLD_STEP 3

#define GLOBAL_SKIN_BLUE 207
#define GLOBAL_SKIN_GREEN 208
#define GLOBAL_SKIN_RED 239

#define GLOBAL_SKIN_COLOR_THRESHOLD 200

#define DISTANCE_THRESHOLD 0.1
#define SPEC_COLOR_THRESHOLD 10
#define CHANGE_COUNT_THRESHOLD 3
#define AREA_THRESHOLD 20
unsigned char changeCount[FREENECT_FRAME_H][FREENECT_FRAME_W];

int invertMat(double A[3][3], double X[3][3]) {
	
	
	double det = 0;
	
	det += A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]);
	det += A[0][1] * (A[1][2] * A[2][0] - A[2][2] * A[1][0]);
	det += A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

	X[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) / det;
	X[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) / det;
	X[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) / det;
	
	X[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / det;
	X[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det;
	X[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) / det;
	
	X[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det;
	X[1][2] = (A[1][2] * A[1][0] - A[0][0] * A[1][2]) / det;
	X[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det;


//	int i, j;
//	printf("\n========== Matrix A ==========================================\n");     
//	for(i=0;i<3;i++) {
//		printf("\n");
//		for(j=0;j<3;j++) {     
//			printf(" A[%d][%d]= %.4f  ",i,j,A[i][j]);
//		}
//	}
//	printf("\n \n");
//	
//	printf("The determinant of matrix A is %f ", det);
	
	if(det==0) {
		//printf("Division by 0, not good!\n");
//		printf("=====================================================================\n\n");
		return 0;
	}
	

//	printf("\n========== The inverse matrix of A ==========\n");
//	for(i=0;i<3;i++)
//	{     printf("\n");
//		for(j=0;j<3;j++)
//		{     
//			printf(" X[%d][%d]= %.4f",i,j,X[i][j]);
//			
//		}
//	}
//	printf("\n===========================================================\n\n");
	
	return 1;	
}

float distMahalanobis(Point3d point, double meanColorVec[], double inverseCovarMat[3][3]) {
		
	float auxVec[3];
	
	auxVec[0] = (point.Y - meanColorVec[0]) * inverseCovarMat[0][0] + (point.U - meanColorVec[1]) * inverseCovarMat[1][0] + (point.V - meanColorVec[2]) * inverseCovarMat[2][0];
	auxVec[1] = (point.Y - meanColorVec[0]) * inverseCovarMat[0][1] + (point.U - meanColorVec[1]) * inverseCovarMat[1][1] + (point.V - meanColorVec[2]) * inverseCovarMat[2][1];
	auxVec[2] = (point.Y - meanColorVec[0]) * inverseCovarMat[0][2] + (point.U - meanColorVec[1]) * inverseCovarMat[1][2] + (point.V - meanColorVec[2]) * inverseCovarMat[2][2];
	
	float dist = auxVec[0] * (point.Y - meanColorVec[0]) + auxVec[0] * (point.U - meanColorVec[1]) + auxVec[2] * (point.V - meanColorVec[2]);

	return sqrt(dist);

}


int processKeyPressed(int keyPressed) {
	if (keyPressed==27) return 0;

	if (keyPressed > 0) {
		switch (keyPressed) {
			case 'w':
				freenect_angle++;
				if (freenect_angle > 30) {
					freenect_angle = 30;
				}
				freenect_sync_set_tilt_degs(freenect_angle);
				break;
			case 's':
				freenect_angle--;
				if (freenect_angle < -30) {
					freenect_angle = -30;
				}
				freenect_sync_set_tilt_degs(freenect_angle);
				break;
			case 'c':
				saveFlag = 1;
				break;
			case 'd':
				displayGuideFlag = !displayGuideFlag;
				break;
			case 'f':
				if (sampleSize > SAMPLE_SIZE_STEP) 
					sampleSize -= SAMPLE_SIZE_STEP;
	
				break;
			case 'g':
				sampleSize += SAMPLE_SIZE_STEP;
				break;
			case 't':
				if (specSkinColorThreshold > COLOR_THRESHOLD_STEP)
					specSkinColorThreshold -= COLOR_THRESHOLD_STEP;
				break;
			case 'y':
				specSkinColorThreshold += COLOR_THRESHOLD_STEP;
				break;
			case 'x':
				grabSkinSampleFlag = 1;
				break;
			case 'r':
				// resetSample
				break;
		}
	}

	return 1;
}


void initWindows() {	
	cvNamedWindow("MyRGB", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Depth", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Hand", CV_WINDOW_AUTOSIZE);
	
	
	cvMoveWindow("MyRGB", 0, 0);
	cvMoveWindow("Depth", 0, 500);
	cvMoveWindow("Hand", 640, 0);
	
}

int main(int argc, char **argv) {

	IplImage *imageMyRgb = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	IplImage *imageDepth = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage *imageHand = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	
	double covarMat[3][3];
	double covarMatInv[3][3];
	double meanColorVec[3];
	
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1, 8);
	char outString[50];


	
//	freenect_sync_set_tilt_degs(freenect_angle);
 	freenect_sync_set_led(LED_RED);
	
	initWindows();
	initDepthLut();
	initUndistortMaps();
	
	while (processKeyPressed(cvWaitKey(10))) {
		
		uint32_t timestamp = createPclImage(curPcl);
		
		
		int u, v;
		float minDist = 100;
		for(v=0; v<FREENECT_FRAME_H; v++) {
			for(u=0; u<FREENECT_FRAME_W; u++) {
				
				if (curPcl[v][u].valid && curPcl[v][u].z < minDist  && curPcl[v][u].z > 0) {
					minDist = curPcl[v][u].z;
					// printf("u: %d v:%d x:%f y:%f z:%f\n", u, v, curPcl[v][u].x, curPcl[v][u].y, curPcl[v][u].z);
				}
			}
		}
		
		float curDistanceThreshold = minDist + DISTANCE_THRESHOLD;
		
		for(v=0; v<FREENECT_FRAME_H; v++) {
			for(u=0; u<FREENECT_FRAME_W; u++) {
				if (curPcl[v][u].valid && curPcl[v][u].z > curDistanceThreshold) {
					curPcl[v][u].valid = 0;
				}
			}
		}
		
		
		float colorDist;
		int maxSkinPixelCount = 0;
		int curSkinPixelCount;
		int seedSkinX = 0, seedSkinY = 0;
		
		Point3d globalSkinColor;
		globalSkinColor.blue = GLOBAL_SKIN_BLUE;
		globalSkinColor.green = GLOBAL_SKIN_GREEN;
		globalSkinColor.red = GLOBAL_SKIN_RED;

		
		
		for (u=0; u<FREENECT_FRAME_W; u++) {
			curSkinPixelCount = 0;
			for (v=0; v<FREENECT_FRAME_H; v++) {
				if (curPcl[v][u].valid && colorRgbDistance(curPcl[v][u], globalSkinColor) < GLOBAL_SKIN_COLOR_THRESHOLD) {
					curSkinPixelCount++;
				}
			}
			
			if (curSkinPixelCount > maxSkinPixelCount) {
				maxSkinPixelCount = curSkinPixelCount;
				seedSkinX = u;
			}
		}
		
		
		maxSkinPixelCount = 0;
		for (v=0; v<FREENECT_FRAME_H; v++) {
			
			curSkinPixelCount = 0;
			for (u=0; u<FREENECT_FRAME_W; u++) {
				if (curPcl[v][u].valid && colorRgbDistance(curPcl[v][u], globalSkinColor) < GLOBAL_SKIN_COLOR_THRESHOLD) {
						curSkinPixelCount++;
					}
			}
				
			if (curSkinPixelCount > maxSkinPixelCount) {
				maxSkinPixelCount = curSkinPixelCount;
				seedSkinY = v;
			}	
		}
		
		
		for (v=0; v<FREENECT_FRAME_H; v++)
			for (u=0; u<FREENECT_FRAME_W; u++)
				handPcl[v][u].valid = 0;
						
		
		handPcl[seedSkinY][seedSkinX] = curPcl[seedSkinY][seedSkinX];
		
		
		int i,j,m,n,again;
		
		if (handFlag) {
			do {
				
				again = 0;
				for(j=0; j<FREENECT_FRAME_H; j++) {
					for(i=0; i<FREENECT_FRAME_W; i++) {
						if (handPcl[j][i].valid) {
							for (n=j-1; n<=j+1; n++) {
								for (m=i-1; m<=i+1; m++) {
									if (!handPcl[n][m].valid && curPcl[n][m].valid && 
											distMahalanobis(curPcl[n][m], meanColorVec, covarMatInv) < specSkinColorThreshold ) {
										
										handPcl[n][m] = curPcl[n][m];
										again = 1;
									}
								}
							}
						}
					}
				}
				
				for(j=FREENECT_FRAME_H-1; j>=0; j--) {
					for(i=FREENECT_FRAME_W-1; i>=0; i--) {
						if (handPcl[j][i].valid) {
							for (n=j-1; n<=j+1; n++) {
								for (m=i-1; m<=i+1; m++) {
									if (!handPcl[n][m].valid && curPcl[n][m].valid && 
											distMahalanobis(curPcl[n][m], meanColorVec, covarMatInv) < SPEC_COLOR_THRESHOLD ) {
			
										handPcl[n][m] = curPcl[n][m];
										again = 1;
									}
								}
							}
						}
					}
				} 
				
				
			} while (again);
			
			float barycX, barycY, barycZ;
			barycenter(handPcl, &barycX, &barycY, &barycZ);
			
			rgbImage(handPcl, imageHand);
			
			sprintf(outString, "X: %.4f Y: %.4f  Z: %.4f",  barycX, barycY, barycZ);
			cvPutText(imageHand, outString, cvPoint(30,30), &font, cvScalar(0,200, 0, 0));		
			
			sprintf(outString, "Skin Color Threshold: %d",   specSkinColorThreshold);
			cvPutText(imageHand, outString, cvPoint(30,50), &font, cvScalar(0,200, 0, 0));			
			
			
			
		}
		
		
		
		
		rgbImage(curPcl, imageMyRgb);
		
		
		sprintf(outString, "%lu - minDist: %f m", timestamp, minDist);
		cvPutText(imageMyRgb, outString, cvPoint(30,30), &font, cvScalar(0,200, 0, 0));
		
		
		if (grabSkinSampleFlag) {
			grabSkinSampleFlag = 0;
				
			double meanY = 0;
			double meanU = 0;
			double meanV = 0;
			int count = 0;
			
			for (n=(seedSkinY-sampleSize); n<=(seedSkinY+sampleSize); n++) {
				for (m=(seedSkinX-sampleSize); m<=(seedSkinX+sampleSize); m++) {
					if (curPcl[n][m].valid) {
						meanY += curPcl[n][m].Y;
						meanU += curPcl[n][m].U;
						meanV += curPcl[n][m].V;
						count++;
						
						// printf("Y: %f  U: %f  V: %f\n", curPcl[n][m].Y, curPcl[n][m].U, curPcl[n][m].V);
					}
				}
			}
			
			meanY = meanY / count;
			meanU = meanU / count;
			meanV = meanV / count;
			
			
			for (i=0; i<3; i++) {
				for (j=0; j<3; j++) {
					covarMat[i][j] = 0;
				}
			}
			
			
			
			for (n=seedSkinY-sampleSize; n<=seedSkinY+sampleSize; n++) {
				for (m=seedSkinX-sampleSize; m<=seedSkinX+sampleSize; m++) {
					if (curPcl[n][m].valid) {
						covarMat[0][0] += (curPcl[n][m].Y - meanY)*(curPcl[n][m].Y - meanY);
						covarMat[0][1] += (curPcl[n][m].Y - meanY)*(curPcl[n][m].U - meanU);
						covarMat[0][2] += (curPcl[n][m].Y - meanY)*(curPcl[n][m].V - meanV);
						
						covarMat[1][0] += (curPcl[n][m].U - meanU)*(curPcl[n][m].Y - meanY);
						covarMat[1][1] += (curPcl[n][m].U - meanU)*(curPcl[n][m].U - meanU);
						covarMat[1][2] += (curPcl[n][m].U - meanU)*(curPcl[n][m].V - meanV);
						
						covarMat[2][0] += (curPcl[n][m].V - meanV)*(curPcl[n][m].Y - meanY);
						covarMat[2][1] += (curPcl[n][m].V - meanV)*(curPcl[n][m].U - meanU);
						covarMat[2][2] += (curPcl[n][m].V - meanV)*(curPcl[n][m].V - meanV);
					}
				}
			}
			
			for (i=0; i<3; i++) {
				for (j=0; j<3; j++) {
					covarMat[i][j] = covarMat[i][j] /count;
				}
			}
			
	
			
			
			if (invertMat(covarMat, covarMatInv)) {
				handFlag = 1;
				meanColorVec[0] = meanY;
				meanColorVec[1] = meanU;
				meanColorVec[2] = meanV;
			}
		}
		
		if (displayGuideFlag) {
			cvRectangle(imageMyRgb, cvPoint(seedSkinX-sampleSize, seedSkinY-sampleSize), 
						cvPoint(seedSkinX+sampleSize, seedSkinY+sampleSize), cvScalar(0, 255, 0, 0), 2, 8, 0);	
			
			sprintf(outString, "Seed X:%d Y%d", seedSkinX, seedSkinY);
			cvPutText(imageMyRgb, outString, cvPoint(30,50), &font, cvScalar(0,200, 0, 0));

		}
		
		
		cvShowImage("MyRGB", imageMyRgb);
		//cvShowImage("Depth", imageDepth);
		cvShowImage("Hand", imageHand);
		
		
		if (saveFlag) {
			saveFlag = 0;
			char fileName[30];
			sprintf(fileName, "img-%lu.jpg", timestamp);
			cvSaveImage(fileName, imageMyRgb, NULL);
		}

    }
	

	freenect_sync_set_led(LED_YELLOW);

	cvReleaseImage(&imageMyRgb);
	cvReleaseImage(&imageDepth);
	cvDestroyAllWindows();

    freenect_sync_stop();
	freenect_sync_set_led(LED_BLINK_GREEN);
	
	return 1;

}
