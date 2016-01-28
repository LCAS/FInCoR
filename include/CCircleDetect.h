/*
 * File name: CCircleDetect.h
 * Date:      2010
 * Author:   Tom Krajnik 
 */

#ifndef __CCIRCLEDETECT_H__
#define __CCIRCLEDETECT_H__

#include "CRawImage.h"
#include "CNecklace.h"
#include <math.h>

#define ID_SAMPLES 320
#define ID_BITS 8

typedef struct{
	float x;
	float y;
	float angle,horizontal;
	int size;
	int maxy,maxx,miny,minx;
	int mean;
	int type;
	float roundness;
	float bwRatio;
	bool round;
	bool valid;
	float m0,m1;
	float v0,v1;
	float r0, r1;
	int ID;
}SSegment;

class CCircleDetect
{
	public:

		CCircleDetect(int wi,int he,bool id=false);
		~CCircleDetect();
		void bufferCleanup(SSegment init);
		int adjustDimensions(int wi,int he);
		SSegment findSegment(CRawImage* image, SSegment init);
		bool examineSegment(CRawImage* image,SSegment *segmen,int ii,float areaRatio);
		int identifySegment(SSegment* inner,CRawImage* image);
		SSegment calcSegment(SSegment segment,int size,long int x,long int y,long int cm0,long int cm1,long int cm2);

		bool identify;
		bool changeThreshold();
		bool debug,draw,drawAll;
		int ID;
	private:
		float normalizeAngle(float a);
		CNecklace *decoder;
		bool track;
		int maxFailed;
		int numFailed;
		int threshold; 

		int minSize; 
		int lastThreshold; 
		int thresholdBias; 
		int maxThreshold; 

		int thresholdStep;
		float circularTolerance;
		float circularityTolerance;
		float ratioTolerance;
		float centerDistanceToleranceRatio;
		int centerDistanceToleranceAbs;
		bool enableCorrections;
		SSegment inner;
		SSegment outer;
		bool lastTrackOK;
		float outerAreaRatio,innerAreaRatio,areasRatio;
		int queueStart,queueEnd,queueOldStart,numSegments;
		static int width,height,len;
		int expand[4];
		unsigned char *ptr;
		int sizer,sizerAll;
		float diameterRatio;
		bool ownBuffer;
		int step;
		static int *buffer;
		static int *queue;
};

#endif

/* end of CCircleDetect.h */
