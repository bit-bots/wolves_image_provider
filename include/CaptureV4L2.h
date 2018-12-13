/*
 * CaptureV4L2.h
 *
 *  Created on: 12.11.2012
 *      Author: Stefan Krupop
 */

#ifndef CAPTUREV4L2_H_
#define CAPTUREV4L2_H_

#include <string>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

/**
 * Capture with V4L2
 */
class CaptureV4L2 {
public:

	/**
	 * Constructor for V4L2 capturing
	 * @param devicePath	path to device, e.g. "/dev/video0"
	 */
	CaptureV4L2(string devicePath);


	/**
	 * Destructor
	 */
	~CaptureV4L2();

	/**
	 * update image
	 */
	bool execute(Mat &image);

private:
#ifndef WIN32
#ifndef __APPLE__
	int xioctl(int fd, unsigned long request, void* argp) const;
	bool setCameraParameters(string devicePath);
	bool setControl(uint32_t controlID, double value, string name);
	bool setControlAbsolute(uint32_t controlID, double value, string name);
	bool captureStop();
	bool captureStart();
	bool deviceUninit();
	bool mmapInit();
	bool deviceInit();
	bool deviceClose();
	bool deviceOpen();
	void getRosParameter(double &value,string name);
	void getRosParameter(int &value,string name);
#endif
#endif
    struct buffer {
		void* start;
		size_t length;
    };

	string mDevicePath;
	int mVideoDevice;
	int mWidth;
	int mHeight;
	int mFrameIntervalNumerator;
	int mFrameIntervalDenominator;
	bool mOldWasChanged;
	uint8_t *bufferData;
    struct buffer* mBuffers;
    uint32_t mBufferCount;
};

#endif /* CAPTUREV4L2_H_ */
