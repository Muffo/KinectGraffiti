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
		// printf("depthLut[%d] = %f\n", i, depthLut[i]);
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

/*
int mapDepthToRgb(int depthU, int depthV, double depthPointZ) {

	static const double fx_d = 1.0 / 570.55841064;
	static const double fy_d = 1.0 / 570.97265625;
	static const double cx_d = 323.78082275;
	static const double cy_d = 268.92361450;

	double depthPointX = (depthU - cx_d) * depthPointZ * fx_d;
	double depthPointY = (depthV - cy_d) * depthPointZ * fy_d;
	// double depthPointZ = pointDepth;



	double Tx = -2.48245621;
	double Ty = -0.01868717;
	double Tz = 0.08752445;

	double rgbPointX =  0.99999845 * depthPointX + 9.63805651e-04  * depthPointY -1.48875453e-03 * depthPointZ + Tx;
	double rgbPointY =  -9.64035571e-04 * depthPointX + 0.99999952 * depthPointY - 1.53721863e-04 * depthPointZ + Ty;
	double rgbPointZ =  1.48860563e-03 * depthPointX + 1.55156828e-04  * depthPointY + 0.99999887 * depthPointZ + Tz;


	static const double fx_rgb = 509.48260498;
	static const double fy_rgb = 511.50158691;
	static const double cx_rgb = 305.60083008;
	static const double cy_rgb = 275.12200928;

	double invZ = 1.0f / rgbPointZ;

	int rgbU = round((rgbPointX * fx_rgb * invZ) + cx_rgb);   // TODO: mettere i bound tra 0 e 639
	int rgbV = round((rgbPointY * fy_rgb * invZ) + cy_rgb);
	int rgbIndex = FREENECT_FRAME_W*rgbV + rgbU;
}
 */


static CvMat *rgbIntrinsic;
static CvMat *rgbDistortion;
static CvMat *irIntrinsic;
static CvMat *irDistortion;
static CvMat *T_mat;
static CvMat *R_mat;

static IplImage* rgbMapX;
static IplImage* rbgMapY;
static IplImage* irMapX;
static IplImage* irMapY;

void initUndistortMaps() {
	rgbIntrinsic = (CvMat*)cvLoad( "../data/Intrinsics_RGB.xml", NULL, NULL, NULL );
	rgbDistortion = (CvMat*)cvLoad( "../data/Distortion_RGB.xml", NULL, NULL, NULL);

	irIntrinsic = (CvMat*)cvLoad( "../data/Intrinsics_IR.xml", NULL, NULL, NULL );
	irDistortion = (CvMat*)cvLoad( "../data/Distortion_IR.xml", NULL, NULL, NULL );

	rgbMapX = cvCreateImage( cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_32F, 1 );
	rbgMapY = cvCreateImage( cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_32F, 1 );
	cvInitUndistortMap(rgbIntrinsic, rgbDistortion, rgbMapX, rbgMapY);

	irMapX = cvCreateImage( cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_32F, 1 );
	irMapY = cvCreateImage( cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_32F, 1 );
	cvInitUndistortMap(irIntrinsic, irDistortion, irMapX, irMapY);

	T_mat = (CvMat*)cvLoad( "../data/kinect_T.xml", NULL, NULL, NULL );
	R_mat = (CvMat*)cvLoad( "../data/kinect_R.xml", NULL, NULL, NULL );

}


int createPclImage(PclImage dest, char *dataRgb, uint16_t *dataDepth) {

	IplImage *rgbImage = cvCreateImageHeader(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_8U, 3);
	IplImage *rgbImageUndist = cvCreateImage(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_8U, 3);
	cvSetData(rgbImage, dataRgb, FREENECT_FRAME_W*3);
   // cvCvtColor(rgbImage, rgbImage, CV_RGB2BGR);

    IplImage *depthImage = cvCreateImageHeader(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_16U, 1);
    IplImage *depthImageUndist = cvCreateImage(cvSize(FREENECT_FRAME_W, FREENECT_FRAME_H ), IPL_DEPTH_16U, 1);
    cvSetData(depthImage, dataDepth, FREENECT_FRAME_W*2);

	cvRemap(rgbImage, rgbImageUndist, rgbMapX, rbgMapY, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(255));
	cvRemap(depthImage, depthImageUndist, irMapX, irMapY, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(255));


	int u, v;
	for(v=0; v<FREENECT_FRAME_H; v++) {
		for(u=0; u<FREENECT_FRAME_W; u++) {
			dest[v][u].z = depthToMts(((uint16_t *)(depthImageUndist->imageData))[v*FREENECT_FRAME_W+u]);
			if (dest[v][u].z == 0) {
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
			int rgbIndex = 3*(FREENECT_FRAME_W*rgbV + rgbU);


			dest[v][u].blue = ((unsigned char *)(rgbImageUndist->imageData))[rgbIndex+2];
			dest[v][u].green = ((unsigned char *)(rgbImageUndist->imageData))[rgbIndex + 1];
			dest[v][u].red = ((unsigned char *)(rgbImageUndist->imageData))[rgbIndex];

		}
	}
	
	cvReleaseImageHeader(&rgbImage);
	cvReleaseImage(&rgbImageUndist);
	cvReleaseImageHeader(&depthImage);
	cvReleaseImage(&depthImageUndist);

	return 1;
}

void rgbImage(PclImage src, IplImage *dst) {
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


void depthImage(PclImage src, IplImage *dst) {
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

float spaceDistance(Point3d pt1, Point3d pt2) {
	return sqrt(pow(pt1.x-pt2.x,2)+pow(pt1.y-pt2.y,2)+pow(pt1.z-pt2.z,2));
}

float colorDistance(Point3d pt1, Point3d pt2) {
	return sqrt(pow(pt1.blue-pt2.blue,2)+pow(pt1.red-pt2.red,2)+pow(pt1.green-pt2.green,2));
}



