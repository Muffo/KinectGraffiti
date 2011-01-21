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
#include "eig3.h"

#define INITIAL_SAMPLE_SIZE 20
#define SAMPLE_VEC_SIZE 100000

#define GLOBAL_SKIN_BLUE 207
#define GLOBAL_SKIN_GREEN 208
#define GLOBAL_SKIN_RED 239

#define GLOBAL_SKIN_COLOR_THRESHOLD 200

#define DISTANCE_THRESHOLD 0.2
#define SPEC_COLOR_THRESHOLD 10
#define CHANGE_COUNT_THRESHOLD 3
#define AREA_THRESHOLD 20


int saveFlag;
int displayGuideFlag = 1;
int sampleSize = INITIAL_SAMPLE_SIZE;
int grabSkinSampleFlag = 0;
int handDetectionFlag = 0;

int specSkinColorThreshold = 8;

Point3d sampleVec[SAMPLE_VEC_SIZE];
int sampleVecIndex = 0;


double freenect_angle = 0;

PclImage curPcl;
PclImage handPcl;




unsigned char changeCount[FREENECT_FRAME_H][FREENECT_FRAME_W];

double invertMat(double A[3][3], double X[3][3]) {

	double det = 0;	

	det += A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]);
	det += A[0][1] * (A[1][2] * A[2][0] - A[2][2] * A[1][0]);
	det += A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
	
	if(det==0) {
		return 0;
	}

	X[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) / det;
	X[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) / det;
	X[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) / det;
	
	X[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / det;
	X[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det;
	X[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) / det;
	
	X[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det;
	X[1][2] = (A[1][2] * A[1][0] - A[0][0] * A[1][2]) / det;
	X[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det;

	return det;	
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
			case 'x':
				grabSkinSampleFlag = 1;
				break;
			case 'z':
				sampleVecIndex = 0;
				handDetectionFlag = 0;
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
	
	cvCreateTrackbar("SampleSize", "MyRGB", &sampleSize, 100, NULL);
	cvCreateTrackbar("Threshold", "Hand", &specSkinColorThreshold, 30, NULL);
}


#pragma mark -

int main(int argc, char **argv) {

	IplImage *imageMyRgb = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	IplImage *imageDepth = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	IplImage *imageHand = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	
	double colorCovarMat[3][3];
	double colorCovarMatInv[3][3];
	double meanColorVec[3];
	
	
	double posCovarMat[3][3];
	double posCovarMatEigVec[3][3];
	double posCovarMatEigVal[3];

	
	
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
		
		if (handDetectionFlag) {        // hand sample
			do {
				
				again = 0;
				for(j=0; j<FREENECT_FRAME_H; j++) {
					for(i=0; i<FREENECT_FRAME_W; i++) {
						if (handPcl[j][i].valid) {
							for (n=j-1; n<=j+1; n++) {
								for (m=i-1; m<=i+1; m++) {
									if (!handPcl[n][m].valid && curPcl[n][m].valid && 
											distMahalanobis(curPcl[n][m], meanColorVec, colorCovarMatInv) < specSkinColorThreshold ) {
										
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
											distMahalanobis(curPcl[n][m], meanColorVec, colorCovarMatInv) < specSkinColorThreshold ) {
			
										handPcl[n][m] = curPcl[n][m];
										again = 1;
									}
								}
							}
						}
					}
				} 
				
			} while (again);
			
			
			double meanX = 0;						// position covar matrix
			double meanY = 0;
			double meanZ = 0;
			int count = 0;
			
			for(j=0; j<FREENECT_FRAME_H; j++) {
				for(i=0; i<FREENECT_FRAME_W; i++) {
					if (handPcl[j][i].valid) {
						meanX += handPcl[j][i].x;
						meanY += handPcl[j][i].y;
						meanZ += handPcl[j][i].z;
						count++;
					}
				}
			}
			
			meanX = meanX / count;
			meanY = meanY / count;
			meanZ = meanZ / count;
			
			
			for (i=0; i<3; i++) {
				for (j=0; j<3; j++) {
					posCovarMat[i][j] = 0;
				}
			}
			
			
			for(j=0; j<FREENECT_FRAME_H; j++) {
				for(i=0; i<FREENECT_FRAME_W; i++) {
					if (handPcl[j][i].valid) {
						posCovarMat[0][0] += (handPcl[j][i].x - meanX)*(handPcl[j][i].x - meanX);
						posCovarMat[0][1] += (handPcl[j][i].x - meanX)*(handPcl[j][i].y - meanY);
						posCovarMat[0][2] += (handPcl[j][i].x - meanX)*(handPcl[j][i].z - meanZ);
						
						posCovarMat[1][0] += (handPcl[j][i].y - meanY)*(handPcl[j][i].x - meanX);
						posCovarMat[1][1] += (handPcl[j][i].y - meanY)*(handPcl[j][i].y - meanY);
						posCovarMat[1][2] += (handPcl[j][i].y - meanY)*(handPcl[j][i].z - meanZ);
						
						posCovarMat[2][0] += (handPcl[j][i].z - meanZ)*(handPcl[j][i].x - meanX);
						posCovarMat[2][1] += (handPcl[j][i].z - meanZ)*(handPcl[j][i].y - meanY);
						posCovarMat[2][2] += (handPcl[j][i].z - meanZ)*(handPcl[j][i].z - meanZ);
					}
				}
			}
			
			for (i=0; i<3; i++) {
				for (j=0; j<3; j++) {
					posCovarMat[i][j] = posCovarMat[i][j] / count;
				}
			}
			
			eigen_decomposition(posCovarMat, posCovarMatEigVec, posCovarMatEigVal);

			
			/*
			printf("\n\n========== Pos Covar =========");
			for(i=0;i<3;i++) {
				printf("\n{");
				for(j=0;j<3;j++) {     
					printf("%f",i,j, posCovarMat[i][j]);
					if (j<2) printf(", ");
				}
				printf("}");
			}
			*/
			
			
			
			/*
			printf("\n\n========== Pos Eigen Vec =========");
			for(i=0;i<3;i++) {
				printf("\n{");
				for(j=0;j<3;j++) {     
					printf("%f",i,j, posCovarMatEigVec[i][j]);
					if (j<2) printf(", ");
				}
				printf("}");
			}
			 */
			
			float barycX, barycY, barycZ;
			barycenter(handPcl, &barycX, &barycY, &barycZ);
			
			rgbImage(handPcl, imageHand);
			
			sprintf(outString, "Barycenter: X: %.4f Y: %.4f  Z: %.4f",  barycX, barycY, barycZ);
			cvPutText(imageHand, outString, cvPoint(30,30), &font, cvScalar(0,200, 0, 0));
			
			sprintf(outString, "Normal Vec: X: %.4f Y: %.4f  Z: %.4f",  
					posCovarMatEigVec[2][0] > 0 ? posCovarMatEigVec[0][0] : - posCovarMatEigVec[0][0], 
					posCovarMatEigVec[2][0] > 0 ? posCovarMatEigVec[1][0] : - posCovarMatEigVec[1][0], 
					posCovarMatEigVec[2][0] > 0 ? posCovarMatEigVec[2][0] : - posCovarMatEigVec[2][0]);
			cvPutText(imageHand, outString, cvPoint(30,50), &font, cvScalar(0,200, 0, 0));
			
			// sprintf(outString, "Skin Color Threshold: %d",   specSkinColorThreshold);
			// cvPutText(imageHand, outString, cvPoint(30,50), &font, cvScalar(0,200, 0, 0));			
			
		}
		
		rgbImage(curPcl, imageMyRgb);
		
		
		sprintf(outString, "%lu - minDist: %f m", timestamp, minDist);
		cvPutText(imageMyRgb, outString, cvPoint(30,30), &font, cvScalar(0,200, 0, 0));
		
		
		if (grabSkinSampleFlag) {
			grabSkinSampleFlag = 0;
				
			double meanY = 0;
			double meanU = 0;
			double meanV = 0;
			
			
			for (n=(seedSkinY-sampleSize); n<=(seedSkinY+sampleSize); n++) {
				for (m=(seedSkinX-sampleSize); m<=(seedSkinX+sampleSize); m++) {
					if (curPcl[n][m].valid && sampleVecIndex < SAMPLE_VEC_SIZE) {
						sampleVec[sampleVecIndex] = curPcl[n][m]; 
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
				handDetectionFlag = 1;
				meanColorVec[0] = meanY;
				meanColorVec[1] = meanU;
				meanColorVec[2] = meanV;
				
				
				

				printf("SampleSize = %d\n", sampleVecIndex);
				printf("Y = %f\tU = %fV = %f\n", meanY, meanU, meanV);
								
				
				printf("\n\n========== Covar =========");
				for(i=0;i<3;i++) {
					printf("\n");
					for(j=0;j<3;j++) {     
						printf(" C[%d][%d]= %f",i,j, colorCovarMat[i][j]);
					}
				}
				printf("\n===========================================================\n\n");
				
				printf("det = %f", detCovarMat);
				
				printf("\n========== CovarInv ==========\n");
				for(i=0;i<3;i++) {
					printf("\n");
					for(j=0;j<3;j++) {     
						printf(" C[%d][%d]= %f",i,j, colorCovarMatInv[i][j]);
					}
				}
				printf("\n===========================================================\n\n");
				
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
