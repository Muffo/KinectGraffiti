/*
 * libMyKinect.c
 *
 *  Created on: Dec 23, 2010
 *      Author: Andrea Grandi
 */

#include "libMyKinect.h"


static float depthLut[2048];
void initDepthLut() {
	int i;
	const float k1 = 1.1863;
	const float k2 = 2842.5;
	const float k3 = 0.1236;
	float depth;

	for (i=0; i<2048; i++)	{
		depth = k3 * tanf(i/k2 + k1);
		// depth = 1.0 / (i * -0.0030711016 + 3.3309495161);
		depthLut[i] = depth;
	}
}


float depthToMts(uint16_t depth) {
	if (depth < 2047)
		return depthLut[depth];
	return 0;

}


unsigned char metersToGreyscale(float meters) {
	const float minThres = 0.4;
	const float maxThres = 3;

	if (meters > maxThres)
		return 255;
	else if (meters < minThres)
		return 0;
	else
		return round(((meters - minThres) * 255/(maxThres - minThres)));
}

static CvMat *rgbIntrinsic;
static CvMat *rgbDistortion;
static CvMat *irIntrinsic;
static CvMat *irDistortion;
static CvMat *T_mat;
static CvMat *R_mat;

static IplImage* rgbMapX;
static IplImage* rgbMapY;
static IplImage* rgbMask;
static IplImage* irMapX;
static IplImage* irMapY;
static IplImage* irMask;

void loadParameters() {
	rgbIntrinsic = (CvMat*)cvLoad( "../data/Intrinsics_RGB.xml", NULL, NULL, NULL );
	rgbDistortion = (CvMat*)cvLoad( "../data/Distortion_RGB.xml", NULL, NULL, NULL);

	irIntrinsic = (CvMat*)cvLoad( "../data/Intrinsics_IR.xml", NULL, NULL, NULL );
	irDistortion = (CvMat*)cvLoad( "../data/Distortion_IR.xml", NULL, NULL, NULL );

	rgbMapX = cvCreateImage( cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_32F, 1 );
	rgbMapY = cvCreateImage( cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_32F, 1 );
	cvInitUndistortMap(rgbIntrinsic, rgbDistortion, rgbMapX, rgbMapY);
	
	
	irMapX = cvCreateImage( cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_32F, 1 );
	irMapY = cvCreateImage( cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_32F, 1 );
	cvInitUndistortMap(irIntrinsic, irDistortion, irMapX, irMapY);
	
	
	IplImage* irMaskDistort = cvCreateImage(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H), IPL_DEPTH_8U, 1);	
	irMask = cvCreateImage(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H), IPL_DEPTH_8U, 1);
	
	cvZero(irMaskDistort);
	cvRemap(irMaskDistort, irMask, irMapX, irMapY, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(255));
	cvReleaseImage(&irMaskDistort);
	
	
	IplImage* rgbMaskDistort = cvCreateImage(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H), IPL_DEPTH_8U, 1);	
	rgbMask = cvCreateImage(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H), IPL_DEPTH_8U, 1);
	
	cvZero(rgbMaskDistort);
	cvRemap(rgbMaskDistort, rgbMask, rgbMapX, rgbMapY, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(255));
	cvReleaseImage(&rgbMaskDistort);
	

	T_mat = (CvMat*)cvLoad( "../data/kinect_T.xml", NULL, NULL, NULL );
	R_mat = (CvMat*)cvLoad( "../data/kinect_R.xml", NULL, NULL, NULL );

}

freenect_context *f_ctx;
freenect_device *f_dev;
char *dataRgb;
uint16_t *dataDepth;
uint32_t timestamp;


int getKinectData() {
	if (freenect_sync_get_rgb(&dataRgb, &timestamp) < 0) {
		printf("Errore: freenect_sync_get_rgb\n");
		return -1;
	}
	
	if (freenect_sync_get_depth(&dataDepth, &timestamp) < 0) {
		printf("Errore: freenect_sync_get_depth\n");
		return -1;
	}
	return 0;
}


int createPointCloud(PointCloud dest) {
	
	while(getKinectData()<0);
	
	IplImage *rgbImage = cvCreateImageHeader(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_8U, 3);
	IplImage *rgbImageUndist = cvCreateImage(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_8U, 3);
	cvSetData(rgbImage, dataRgb, FREENECT_FRAME_W*3);

    IplImage *depthImage = cvCreateImageHeader(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_16U, 1);
    IplImage *depthImageUndist = cvCreateImage(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_16U, 1);
    cvSetData(depthImage, dataDepth, FREENECT_FRAME_W*2);

	cvRemap(rgbImage, rgbImageUndist, rgbMapX, rgbMapY, CV_WARP_FILL_OUTLIERS, cvScalarAll(255));
	cvRemap(depthImage, depthImageUndist, irMapX, irMapY, CV_WARP_FILL_OUTLIERS, cvScalarAll(255));
	
	int u, v;
	for(v=0; v<FREENECT_FRAME_H; v++) {
		for(u=0; u<FREENECT_FRAME_W; u++) {
			dest[v][u].z = depthToMts(((uint16_t *)(depthImage->imageData))[v*FREENECT_FRAME_W+u]);
			if (dest[v][u].z == 0 || ((unsigned char *)(irMask->imageData))[v*FREENECT_FRAME_W+u] == 255 ) {
				dest[v][u].valid = 0;
				continue;
			}

			dest[v][u].valid = 1;

			float fx_d = cvmGet(irIntrinsic, 0, 0);
			float fy_d = cvmGet(irIntrinsic, 1, 1);
			float cx_d = cvmGet(irIntrinsic, 0, 2);
			float cy_d = cvmGet(irIntrinsic, 1, 2);

			dest[v][u].x = (u - cx_d) * dest[v][u].z / fx_d;
			dest[v][u].y = (v - cy_d) * dest[v][u].z / fy_d;


			double Tx = cvmGet(T_mat, 0, 0) / 100;
			double Ty = cvmGet(T_mat, 1, 0) / 100;;
			double Tz = cvmGet(T_mat, 1, 0) / 100;;

			double rgbPointX = cvmGet(R_mat,0,0) * dest[v][u].x + cvmGet(R_mat,0,1) * dest[v][u].y + cvmGet(R_mat,0,2) * dest[v][u].z + Tx;
			double rgbPointY = cvmGet(R_mat,1,0) * dest[v][u].x + cvmGet(R_mat,1,1) * dest[v][u].y + cvmGet(R_mat,1,2) * dest[v][u].z + Ty;
			double rgbPointZ = cvmGet(R_mat,2,0) * dest[v][u].x + cvmGet(R_mat,2,1) * dest[v][u].y + cvmGet(R_mat,2,2) * dest[v][u].z + Tz;

			float fx_rgb = cvmGet(rgbIntrinsic, 0, 0);
			float fy_rgb = cvmGet(rgbIntrinsic, 1, 1);
			float cx_rgb = cvmGet(rgbIntrinsic, 0, 2);
			float cy_rgb = cvmGet(rgbIntrinsic, 1, 2);

			double invZ = 1.0f / rgbPointZ;

			int rgbU = round((rgbPointX * fx_rgb * invZ) + cx_rgb);   // TODO: mettere i bound tra 0 e 639
			int rgbV = round((rgbPointY * fy_rgb * invZ) + cy_rgb);

			if (rgbU < 0) rgbU = 0;
			if (rgbU > 639) rgbU = 639;
			if (rgbV < 0) rgbV = 0;
			if (rgbV > 479) rgbV = 479;
			
			if (((unsigned char *)(rgbMask->imageData))[rgbV*FREENECT_FRAME_W+rgbU] == 255 ) {
				dest[v][u].valid = 0;
				continue;
			}
			
			int rgbIndex = 3*(FREENECT_FRAME_W*rgbV + rgbU);


			dest[v][u].blue = ((unsigned char *)(rgbImage->imageData))[rgbIndex+2];
			dest[v][u].green = ((unsigned char *)(rgbImage->imageData))[rgbIndex + 1];
			dest[v][u].red = ((unsigned char *)(rgbImage->imageData))[rgbIndex];
			
			dest[v][u].Y = 0.299 * dest[v][u].red / 255.0 + 0.587 * dest[v][u].green / 255.0 + 0.114 * dest[v][u].blue / 255.0;
			dest[v][u].U = -0.14713 * dest[v][u].red / 255.0 - 0.28886 * dest[v][u].green / 255.0 + 0.436 * dest[v][u].blue / 255.0;
			dest[v][u].V = 0.615 * dest[v][u].red / 255.0 - 0.51499 * dest[v][u].green / 255.0 - 0.10001 * dest[v][u].blue / 255.0;
			
		}
	}
	
	cvReleaseImageHeader(&rgbImage);
	cvReleaseImage(&rgbImageUndist);
	cvReleaseImageHeader(&depthImage);
	cvReleaseImage(&depthImageUndist);
	
	free(dataDepth);
	free(dataRgb);

	return timestamp;
}

void rgbImage(PointCloud src, IplImage *dst) {
	int u, v;
	for(v=0; v<FREENECT_FRAME_H; v++) {
		for(u=0; u<FREENECT_FRAME_W; u++) {
			if (src[v][u].valid) {
				((unsigned char*)(dst->imageData))[3*(FREENECT_FRAME_W*v + u)+2] = src[v][u].red;
				((unsigned char*)(dst->imageData))[3*(FREENECT_FRAME_W*v + u)+1] = src[v][u].green;
				((unsigned char*)(dst->imageData))[3*(FREENECT_FRAME_W*v + u)] = src[v][u].blue;
			}
			else {
				((unsigned char*)(dst->imageData))[3*(FREENECT_FRAME_W*v + u)] = 255;
				((unsigned char*)(dst->imageData))[3*(FREENECT_FRAME_W*v + u)+1] = 255;
				((unsigned char*)(dst->imageData))[3*(FREENECT_FRAME_W*v + u)+2] = 255;
			}
		}
	}
}

void depthImage(PointCloud src, IplImage *dst) {
	int u, v;
	for(v=0; v<FREENECT_FRAME_H; v++) {
		for(u=0; u<FREENECT_FRAME_W; u++) {
			if (src[v][u].valid) {
				((unsigned char*)(dst->imageData))[FREENECT_FRAME_W*v + u] = metersToGreyscale(src[v][u].z);
			}
			else {
				((unsigned char*)(dst->imageData))[FREENECT_FRAME_W*v + u] = 255;
			}
		}
	}
}


void binaryImage(PointCloud src, IplImage *dst) {
	cvZero(dst);
	
	int u, v;
	for(v=0; v<FREENECT_FRAME_H; v++) {
		for(u=0; u<FREENECT_FRAME_W; u++) {
			if (src[v][u].valid) {
				((unsigned char*)(dst->imageData))[FREENECT_FRAME_W*v + u] = 255;
			}
			else {
				((unsigned char*)(dst->imageData))[FREENECT_FRAME_W*v + u] = 0;
			}
		}
	}
}

void invalidatePcl(PointCloud pcl) {
	int u, v;
	
	for (v=0; v<FREENECT_FRAME_H; v++)
		for (u=0; u<FREENECT_FRAME_W; u++)
			pcl[v][u].valid = 0;
}

float getMinDistance(PointCloud pcl) {
	float minDist = 100;
	int u, v;
	for(v=0; v<FREENECT_FRAME_H; v++) {
		for(u=0; u<FREENECT_FRAME_W; u++) {
			
			if (pcl[v][u].valid && pcl[v][u].z < minDist  && pcl[v][u].z > 0) {
				minDist = pcl[v][u].z;
			}
		}
	}
	return minDist;
}


void pclDistThreshold(PointCloud pcl, float minDist, float maxDist) {
	int u, v;
		
	for(v=0; v<FREENECT_FRAME_H; v++) {
		for(u=0; u<FREENECT_FRAME_W; u++) {
			if (pcl[v][u].valid && (pcl[v][u].z > maxDist || pcl[v][u].z < minDist)) {
				pcl[v][u].valid = 0;
			}
		}
	}
}
																

float spaceDistance(Point3d pt1, Point3d pt2) {
	return sqrt(pow(pt1.x-pt2.x,2)+pow(pt1.y-pt2.y,2)+pow(pt1.z-pt2.z,2));
}

float colorRgbDistance(Point3d pt1, Point3d pt2) {
	return sqrt(pow(pt1.blue-pt2.blue,2)+pow(pt1.red-pt2.red,2)+pow(pt1.green-pt2.green,2));
}

float colorUvDistance(Point3d pt1, Point3d pt2) {
	return sqrt(pow(pt1.U-pt2.U,2)+pow(pt1.V-pt2.V,2));
}

float colorMahalanobisDistance(Point3d point, double meanColorVec[], double inverseCovarMat[3][3]) {
	
	float auxVec[3];
	
	auxVec[0] = (point.Y - meanColorVec[0]) * inverseCovarMat[0][0] + (point.U - meanColorVec[1]) * inverseCovarMat[1][0] + (point.V - meanColorVec[2]) * inverseCovarMat[2][0];
	auxVec[1] = (point.Y - meanColorVec[0]) * inverseCovarMat[0][1] + (point.U - meanColorVec[1]) * inverseCovarMat[1][1] + (point.V - meanColorVec[2]) * inverseCovarMat[2][1];
	auxVec[2] = (point.Y - meanColorVec[0]) * inverseCovarMat[0][2] + (point.U - meanColorVec[1]) * inverseCovarMat[1][2] + (point.V - meanColorVec[2]) * inverseCovarMat[2][2];
	
	float dist = auxVec[0] * (point.Y - meanColorVec[0]) + auxVec[0] * (point.U - meanColorVec[1]) + auxVec[2] * (point.V - meanColorVec[2]);
	
	return sqrt(dist);
	
}


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


void printMatrix(char* name, float** matrix, int order) {
	printf("\n\n========== ");
	printf(name);
	printf("=========");
	
	int i, j;
	for(i=0;i<order;i++) {
		printf("\n{");
		for(j=0;j<order;j++) {     
			printf("%f",i,j, matrix[i][j]);
			if (j<2) printf(", ");
		}
		printf("}");
	}
}



int barycenter(PointCloud src, double* x, double* y, double* z) {
	*x = 0;
	*y = 0;
	*z = 0;
	
	int count = 0;
	int u, v;
	for(v=0; v<FREENECT_FRAME_H; v++) {
		for(u=0; u<FREENECT_FRAME_W; u++) {
			if (src[v][u].valid) {
				*x += src[v][u].x;
				*y += src[v][u].y;
				*z += src[v][u].z;
				count++;
			}
		}
	}
	
	*x /= count;
	*y /= count;
	*z /= count;	
	return count;
}

