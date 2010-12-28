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
		// depth = k3 * tanf(i/k2 + k1);
		depth = 1.0 / (i * -0.0030711016 + 3.3309495161);
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


int mapDepthToRgb(int depthU, int depthV, double depthPointZ) {

	static const double fx_d = 1.0 / 5.9421434211923247e+02;
	static const double fy_d = 1.0 / 5.9104053696870778e+02;
	static const double cx_d = 3.3930780975300314e+02;
	static const double cy_d = 2.4273913761751615e+02;

	double depthPointX = (depthU - cx_d) * depthPointZ * fx_d;
	double depthPointY = (depthV - cy_d) * depthPointZ * fy_d;
	// double depthPointZ = pointDepth;

	double Tx = 1.9985242312092553e-02;
	double Ty = -7.4423738761617583e-04;
	double Tz = -1.0916736334336222e-02;

//	double rgbPointX =  9.9984628826577793e-01 * depthPointX + 1.2635359098409581e-03 * depthPointY - 1.7487233004436643e-02 * depthPointZ - Tx;
//	double rgbPointY = -1.4779096108364480e-03 * depthPointX + 9.9992385683542895e-01 * depthPointY - 1.2251380107679535e-02 * depthPointZ - Ty;
//	double rgbPointZ =  1.7470421412464927e-02 * depthPointX + 1.2275341476520762e-02 * depthPointY + 9.9977202419716948e-01 * depthPointZ - Tz;

	double rgbPointX =  9.9984628826577793e-01 * depthPointX - 1.4779096108364480e-03 * depthPointY - 1.7470421412464927e-02 * depthPointZ - Tx;
	double rgbPointY =  1.2635359098409581e-03 * depthPointX + 9.9992385683542895e-01 * depthPointY - 1.2275341476520762e-02 * depthPointZ - Ty;
	double rgbPointZ =  1.7487233004436643e-02 * depthPointX + 1.2251380107679535e-02 * depthPointY + 9.9977202419716948e-01 * depthPointZ - Tz;


	static const double fx_rgb = 5.2921508098293293e+02;
	static const double fy_rgb = 5.2556393630057437e+02;
	static const double cx_rgb = 3.2894272028759258e+02;
	static const double cy_rgb = 2.6748068171871557e+02;

	double invZ = 1.0f / rgbPointZ;

	int rgbU = round((rgbPointX * fx_rgb * invZ) + cx_rgb);   // TODO: mettere i bound tra 0 e 639
	int rgbV = round((rgbPointY * fy_rgb * invZ) + cy_rgb);
	int rgbIndex = FREENECT_FRAME_W*rgbV + rgbU;
}


int buildPclImage(PclImage dest, char *dataRgb, uint16_t *dataDepth) {

	int u, v;
	for(v=0; v<FREENECT_FRAME_H; v++) {
		for(u=0; u<FREENECT_FRAME_W; u++) {
			dest[v][u].z = depthToMts(dataDepth[v*FREENECT_FRAME_W+u]);
			if (dest[v][u].z == 0) {
				dest[v][u].valid = 0;
				continue;
			}

			dest[v][u].valid = 1;

			static const double fx_d = 1.0 / 5.9421434211923247e+02;
			static const double fy_d = 1.0 / 5.9104053696870778e+02;
			static const double cx_d = 3.3930780975300314e+02;
			static const double cy_d = 2.4273913761751615e+02;

			dest[v][u].x = (u - cx_d) * dest[v][u].z * fx_d;
			dest[v][u].y = (v - cy_d) * dest[v][u].z * fy_d;

			double Tx = 1.9985242312092553e-02;
			double Ty = -7.4423738761617583e-04;
			double Tz = -1.0916736334336222e-02;

		//	double rgbPointX =  9.9984628826577793e-01 * depthPointX + 1.2635359098409581e-03 * depthPointY - 1.7487233004436643e-02 * depthPointZ - Tx;
		//	double rgbPointY = -1.4779096108364480e-03 * depthPointX + 9.9992385683542895e-01 * depthPointY - 1.2251380107679535e-02 * depthPointZ - Ty;
		//	double rgbPointZ =  1.7470421412464927e-02 * depthPointX + 1.2275341476520762e-02 * depthPointY + 9.9977202419716948e-01 * depthPointZ - Tz;

			double rgbPointX =  9.9984628826577793e-01 * dest[v][u].x - 1.4779096108364480e-03 * dest[v][u].y - 1.7470421412464927e-02 * dest[v][u].z - Tx;
			double rgbPointY =  1.2635359098409581e-03 * dest[v][u].x + 9.9992385683542895e-01 * dest[v][u].y - 1.2275341476520762e-02 * dest[v][u].z - Ty;
			double rgbPointZ =  1.7487233004436643e-02 * dest[v][u].x + 1.2251380107679535e-02 * dest[v][u].y + 9.9977202419716948e-01 * dest[v][u].z - Tz;


			static const double fx_rgb = 5.2921508098293293e+02;
			static const double fy_rgb = 5.2556393630057437e+02;
			static const double cx_rgb = 3.2894272028759258e+02;
			static const double cy_rgb = 2.6748068171871557e+02;

			double invZ = 1.0f / rgbPointZ;

			int rgbU = round((rgbPointX * fx_rgb * invZ) + cx_rgb);   // TODO: mettere i bound tra 0 e 639
			int rgbV = round((rgbPointY * fy_rgb * invZ) + cy_rgb);
			int rgbIndex = 3*(FREENECT_FRAME_W*rgbV + rgbU);

			dest[v][u].blue = dataRgb[rgbIndex];
			dest[v][u].green = dataRgb[rgbIndex + 1];
			dest[v][u].red = dataRgb[rgbIndex + 2];
		}
	}

	return 1;
}

void rgbImage(PclImage src, IplImage *dst) {
	int u, v;
	for(v=0; v<FREENECT_FRAME_H; v++) {
		for(u=0; u<FREENECT_FRAME_W; u++) {
			if (src[v][u].valid) {
				((unsigned char*)(dst->imageData))[3*(FREENECT_FRAME_W*v + u)] = src[v][u].red;
				((unsigned char*)(dst->imageData))[3*(FREENECT_FRAME_W*v + u)+1] = src[v][u].green;
				((unsigned char*)(dst->imageData))[3*(FREENECT_FRAME_W*v + u)+2] = src[v][u].blue;
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

