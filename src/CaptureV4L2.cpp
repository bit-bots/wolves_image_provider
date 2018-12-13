/*
 * CaptureV4L2.cpp
 *
 *  Created on: 12.11.2012
 *      Author: Stefan Krupop
 */

#include "CaptureV4L2.h"


#include <sys/stat.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <errno.h>
#include <sys/mman.h>
#include <malloc.h>
#include <string.h>
#include "ros/ros.h"
#include <cstdlib>
#include <sys/ioctl.h>

#define CLEAR(x) memset(&(x), 0, sizeof (x))

//lint -e835 -e845

CaptureV4L2::CaptureV4L2(string devicePath)
:	mDevicePath(devicePath),
	mVideoDevice(0),
	mWidth(1280),
	mHeight(800),
	mFrameIntervalNumerator(1),
	mFrameIntervalDenominator(30),
	mBuffers(NULL),
	mBufferCount(0)
{


	ros::param::get("/camera_v4l/resolution/width",mWidth);
	ros::param::get("/camera_v4l/resolution/height",mHeight);
#ifndef WIN32
#ifndef __APPLE__
	// open and initialize device
	if (deviceOpen()) {
		if (deviceInit()) {
			setCameraParameters(devicePath);
			// start capturing
			captureStart();
		}
	}
#endif
#endif
}



CaptureV4L2::~CaptureV4L2() {
#ifndef WIN32
#ifndef __APPLE__
	// stop capturing
	captureStop();

	// close device
	deviceUninit();
	deviceClose();
#endif
#endif
	free(mBuffers);
}

bool CaptureV4L2::execute(Mat &image) {

#ifndef WIN32
#ifndef __APPLE__
	fd_set fds;
	struct timeval tv;
	int r;
	//setCameraParameters();
	double brightness = 0.45;
	//setControl(V4L2_CID_BRIGHTNESS,brightness,"brightness");


	FD_ZERO(&fds);
	//lint --e(713)
	FD_SET((uint32_t)mVideoDevice, &fds);

	/* Timeout. */
	tv.tv_sec = 2;
	tv.tv_usec = 0;

	r = select(mVideoDevice + 1, &fds, NULL, NULL, &tv);

	if (r <= 0) {
		ROS_ERROR("select: %d, %s", errno, strerror(errno));
		return false;
	}

	struct v4l2_buffer buf;
	CLEAR (buf);
	uint8_t *buffer;
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(mVideoDevice, VIDIOC_DQBUF, &buf)) {
		switch (errno) {
			case EAGAIN:
			return false;
			case EIO:
			// Could ignore EIO, see spec
			// fall through
			default:
			ROS_ERROR("VIDIOC_DQBUF: %d, %s", errno, strerror(errno));
			return false;
		}
	}

	//assert (buf.index < n_buffers);

	//bufferData = mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, mVideoDevice, buf.m.offset);
	//image.data=mBuffers->start;
	//image = Mat(480, 640, CV_8UC3,mBuffers->start);


	//imdecode((uchar*) mBuffers[buf.index].start);
	image.data = (uchar*) mBuffers[buf.index].start;
	//getImage().updateImage((char*)mBuffers[buf.index].start, mWidth, mHeight);

	if (-1 == xioctl(mVideoDevice, VIDIOC_QBUF, &buf)) {
		ROS_ERROR("VIDIOC_QBUF: %d, %s", errno, strerror(errno));
		return false;
	}
#endif
#endif
	return true;
}

#ifndef WIN32
#ifndef __APPLE__
int CaptureV4L2::xioctl(int fd, unsigned long request, void* argp) const {
	int r;

	do r = ioctl(fd, request, argp);
	while (-1 == r && EINTR == errno);

	return r;
}

bool CaptureV4L2::setCameraParameters(string devicePath) {

	double brightness = 0.45;
	setControl(V4L2_CID_BRIGHTNESS,brightness,"brightness");
	double contrast = 0.5;
	setControl(V4L2_CID_CONTRAST, contrast,"contrast");

	double saturation = 0.6;
	setControl(V4L2_CID_SATURATION, saturation,"saturation");

	double hue = 0.0;
	setControl(V4L2_CID_HUE, hue,"hue");

	int auto_white_balance = 0;
	setControl(V4L2_CID_AUTO_WHITE_BALANCE, auto_white_balance,"auto_white_balance");

	int red_balance = 0;
	setControl(V4L2_CID_RED_BALANCE, red_balance,"red_balance");

	int blue_balance = 0;
	setControl(V4L2_CID_BLUE_BALANCE, blue_balance,"blue_balance");

	double gamma = 0.4;
	setControl(V4L2_CID_GAMMA, gamma,"gamma");

	double exposure = 0.0;
	setControl(V4L2_CID_EXPOSURE, exposure,"exposure");

	double autogain =0.0;
	setControl(V4L2_CID_AUTOGAIN, autogain,"autogain");

	double gain = 0.61;
	setControl(V4L2_CID_GAIN, gain,"gain");

#ifdef V4L2_CID_POWER_LINE_FREQUENCY
//	setControl(V4L2_CID_POWER_LINE_FREQUENCY, settings.powerlinefreq);
#endif
#ifdef V4L2_CID_WHITE_BALANCE_TEMPERATURE
//	setControl(V4L2_CID_WHITE_BALANCE_TEMPERATURE, settings.whitebalancetemp);
#endif
#ifdef V4L2_CID_SHARPNESS
//	setControl(V4L2_CID_SHARPNESS, settings.sharpness);
#endif
#ifdef V4L2_CID_BACKLIGHT_COMPENSATION
//	setControl(V4L2_CID_BACKLIGHT_COMPENSATION, settings.backlightcompensation);
#endif
	setControlAbsolute(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL,"autoExposure");
	//setControl(V4L2_CID_EXPOSURE_AUTO_PRIORITY, V4L2_EXPOSURE_MANUAL,"autoPriority");
	//setControl(V4L2_CID_AUTO_EXPOSURE_BIAS, V4L2_EXPOSURE_MANUAL,"autoPriority");
	setControl(V4L2_CID_EXPOSURE_ABSOLUTE, 0.3,"absoluteExposure");
	setControl(V4L2_CID_WHITE_BALANCE_TEMPERATURE, 0.4,"whiteBalance");
	setControl(V4L2_CID_AUTO_WHITE_BALANCE, 0,"auto_white_balance");
	setControl(V4L2_CID_FOCUS_ABSOLUTE, 0.0,"focus_absolute");
	setControl(V4L2_CID_FOCUS_AUTO, 0.0,"focus_auto");
	setControl(V4L2_CID_FOCUS_RELATIVE, 0.0,"focus_relative");
	setControl(V4L2_CID_AUTO_FOCUS_STOP,1.0,"auto_focus_stop");
	std::stringstream ss;
	ss <<"uvcdynctrl -v -d ";
	ss << devicePath;
	ss << " --set='Focus, Auto' 0";
	const string s = ss.str();
	system(s.c_str());
	setControl(V4L2_CID_EXPOSURE_AUTO_PRIORITY,0.0,"autoExposurePriority");
	setControlAbsolute(V4L2_CID_EXPOSURE_AUTO,V4L2_EXPOSURE_MANUAL,"autoExposure");
	setControl(V4L2_CID_EXPOSURE_ABSOLUTE, 0.3,"absoluteExposure");
	return true;
}


void CaptureV4L2::getRosParameter(double &value,string name){
	ostringstream os;
	os <<"/camera_v4l/";
	os << name;
	if(name == "") {
		ROS_INFO("Not getting ROS Parameter for empty string");
	} else if(!ros::param::get(os.str(), value)){
		ROS_ERROR("Cannot get Parameter for %s get default",name.c_str());
	}
}

void CaptureV4L2::getRosParameter(int &value,string name){
	ostringstream os;
	os <<"/camera_v4l/";
	os << name;
	if(name == "") {
		ROS_INFO("Not getting ROS Parameter for empty string");
	} else if(!ros::param::get(os.str(), value)){
		ROS_ERROR("Cannot get Parameter for %s get default",name.c_str());
	}
}

bool CaptureV4L2::setControl(uint32_t controlID, double value, string name) {
	getRosParameter(value,name);
	struct v4l2_queryctrl queryctrl;
	struct v4l2_control control;

	CLEAR(queryctrl);
	queryctrl.id = controlID;

	if (-1 == xioctl(mVideoDevice, VIDIOC_QUERYCTRL, &queryctrl)) {
		if (errno != EINVAL) {
			ROS_ERROR( "VIDIOC_QUERYCTRL: %d, %s", errno, strerror(errno));
		} else {
			ROS_WARN( "Control 0x%x is not supported(%s)", controlID,name.c_str());
		}
		return false;
	} else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		ROS_WARN("Control '%s' is not supported", queryctrl.name);
	} else {
		memset(&control, 0, sizeof(control));
		control.id = controlID;
		if (value == -1) {
			control.value = queryctrl.default_value;
		} else {
			control.value = queryctrl.minimum + (signed int)(((double)(queryctrl.maximum - queryctrl.minimum) / 1.0) * value);
		}

		//ROS_INFO( "Setting control '%s' (%li to %li, default %li) to %li", queryctrl.name, queryctrl.minimum, queryctrl.maximum, queryctrl.default_value, control.value);

		if (-1 == xioctl(mVideoDevice, VIDIOC_S_CTRL, &control)) {
			ROS_ERROR( "VIDIOC_S_CTRL: %d, %s", errno, strerror(errno));
			return false;
		}
	}
	return true;
}

bool CaptureV4L2::setControlAbsolute(uint32_t controlID, double value, string name) {
	getRosParameter(value,name);
	struct v4l2_queryctrl queryctrl;
	struct v4l2_control control;

	CLEAR(queryctrl);
	queryctrl.id = controlID;

	if (-1 == xioctl(mVideoDevice, VIDIOC_QUERYCTRL, &queryctrl)) {
		if (errno != EINVAL) {
			ROS_ERROR( "VIDIOC_QUERYCTRL: %d, %s", errno, strerror(errno));
		} else {
			ROS_WARN( "Control 0x%x is not supported(%s)", controlID,name.c_str());
		}
		return false;
	} else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		ROS_WARN("Control '%s' is not supported", queryctrl.name);
	} else {
		memset(&control, 0, sizeof(control));
		control.id = controlID;
		if (value == -1) {
			control.value = queryctrl.default_value;
		} else {
			control.value = value;
		}


		if (-1 == xioctl(mVideoDevice, VIDIOC_S_CTRL, &control)) {
			ROS_ERROR( "VIDIOC_S_CTRL: %d, %s", errno, strerror(errno));
			return false;
		}
	}
	return true;
}


bool CaptureV4L2::captureStop() {
	enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (-1 == xioctl(mVideoDevice, VIDIOC_STREAMOFF, &type)) {
		ROS_ERROR("VIDIOC_STREAMOFF: %d, %s", errno, strerror(errno));
		return false;
	}

	return true;
}

bool CaptureV4L2::captureStart() {
	uint32_t i;
	enum v4l2_buf_type type;

	for (i = 0; i < mBufferCount; ++i) {
		struct v4l2_buffer buf;

		CLEAR (buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;

		if (-1 == xioctl(mVideoDevice, VIDIOC_QBUF, &buf)) {
			ROS_ERROR("VIDIOC_QBUF: %d, %s", errno, strerror(errno));
			return false;
		}
	}

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (-1 == xioctl(mVideoDevice, VIDIOC_STREAMON, &type)) {
		ROS_ERROR( "VIDIOC_STREAMON: %d, %s", errno, strerror(errno));
		return false;
	}
	return true;
}

bool CaptureV4L2::deviceUninit() {
	if (mBuffers != NULL) {
		uint32_t i;

		for (i = 0; i < mBufferCount; ++i) {
			if (-1 == munmap(mBuffers[i].start, mBuffers[i].length)) {
				ROS_ERROR( "munmap: %d, %s", errno, strerror(errno));
				return false;
			}
		}
	}

	free(mBuffers);
	mBuffers = NULL;
	return true;
}

bool CaptureV4L2::mmapInit() {
	struct v4l2_requestbuffers req;

	CLEAR (req);

	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(mVideoDevice, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			ROS_ERROR( "%s does not support memory mapping", mDevicePath.c_str());
			return false;
		} else {
			ROS_ERROR( "VIDIOC_REQBUFS: %d, %s", errno, strerror(errno));
			return false;
		}
	}

	if (req.count < 2) {
		ROS_ERROR( "Insufficient buffer memory on %s", mDevicePath.c_str());
		return false;
	}

	mBuffers = (struct buffer*)calloc((size_t)(req.count), sizeof(*mBuffers));

	if (!mBuffers) {
		ROS_ERROR( "Out of memory");
		return false;
	}

	for (mBufferCount = 0; mBufferCount < req.count; ++mBufferCount) {
		struct v4l2_buffer buf;

		CLEAR (buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = mBufferCount;

		if (-1 == xioctl(mVideoDevice, VIDIOC_QUERYBUF, &buf)) {
			ROS_ERROR( "VIDIOC_QUERYBUF: %d, %s", errno, strerror(errno));
			return false;
		}

		mBuffers[mBufferCount].length = buf.length;
		mBuffers[mBufferCount].start = mmap(NULL /* start anywhere */, (size_t)buf.length, PROT_READ | PROT_WRITE /* required */, MAP_SHARED /* recommended */, mVideoDevice, (size_t)buf.m.offset);

		if (MAP_FAILED == mBuffers[mBufferCount].start) {
			ROS_ERROR( "mmap: %d, %s", errno, strerror(errno));
			return false;
		}
	}
	return true;
}

bool CaptureV4L2::deviceInit() {
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	struct v4l2_streamparm streamparm;
	struct v4l2_fract *tpf;
	uint32_t min_val;

	if (-1 == xioctl(mVideoDevice, VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno) {
			ROS_ERROR("%s is no V4L2 device", mDevicePath.c_str());
			return false;
		} else {
			ROS_ERROR( "VIDIOC_QUERYCAP: %d, %s", errno, strerror(errno));
			return false;
		}
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		ROS_ERROR( "%s is no video capture device", mDevicePath.c_str());
		return false;
	}

	if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
		ROS_ERROR("%s does not support streaming i/o", mDevicePath.c_str());
		return false;
	}

	// Select video input, video standard and tune here.
	CLEAR(cropcap);

	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (0 == xioctl(mVideoDevice, VIDIOC_CROPCAP, &cropcap)) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; // reset to default

		if (-1 == xioctl(mVideoDevice, VIDIOC_S_CROP, &crop)) {
			switch (errno) {
				case EINVAL:
				// Cropping not supported.
				break;
				default:
				// Errors ignored.
				break;
			}
		}
	} else {
		// Errors V4L2_PIX_FMT_YUYV;ignored.
	}

	CLEAR (fmt);

	// v4l2_format
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = (uint32_t)mWidth;
	fmt.fmt.pix.height = (uint32_t)mHeight;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	//fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
	//fmt.fmt.pix.field = V4L2_FIELD_NONE;

	if (-1 == xioctl(mVideoDevice, VIDIOC_S_FMT, &fmt)) {
		ROS_ERROR("VIDIOC_S_FMT: %d, %s", errno, strerror(errno));
		return false;
	}

	// Note VIDIOC_S_FMT may change width and height.
	if (mWidth != (int)fmt.fmt.pix.width) {
		mWidth = (int)fmt.fmt.pix.width;
		ROS_WARN( "Image width set to %i by device %s.", mWidth, mDevicePath.c_str());
	}
	if (mHeight != (int)fmt.fmt.pix.height) {
		mHeight = (int)fmt.fmt.pix.height;
		ROS_WARN( "Image height set to %i by device %s.", mHeight, mDevicePath.c_str());
	}

	/* Buggy driver paranoia. */
	min_val = fmt.fmt.pix.width * 2;
	if (fmt.fmt.pix.bytesperline < min_val)
		fmt.fmt.pix.bytesperline = min_val;
	min_val = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if (fmt.fmt.pix.sizeimage < min_val)
		fmt.fmt.pix.sizeimage = min_val;

	/* set frame rate */
	CLEAR(streamparm);
	streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	tpf = &streamparm.parm.capture.timeperframe;
	tpf->numerator = (uint32_t)mFrameIntervalNumerator;
	tpf->denominator = (uint32_t)mFrameIntervalDenominator;
	if (-1 == xioctl(mVideoDevice, VIDIOC_S_PARM, &streamparm)) {
		ROS_ERROR("VIDIOC_S_PARM: %d, %s", errno, strerror(errno));
		return false;
	}

	if (mFrameIntervalDenominator != (int)tpf->denominator || mFrameIntervalNumerator != (int)tpf->numerator) {
		ROS_WARN( "The V4L2 driver changed the time per frame to %d/%d\n", tpf->numerator, tpf->denominator);
		mFrameIntervalNumerator = (int)tpf->numerator;
		mFrameIntervalDenominator = (int)tpf->denominator;
	}

	return mmapInit();
}

bool CaptureV4L2::deviceClose() {
	if (-1 == close(mVideoDevice)) {
		ROS_ERROR( "close: %d, %s", errno, strerror(errno));
		return false;
	}

	mVideoDevice = -1;
	return true;
}

bool CaptureV4L2::deviceOpen() {
	struct stat st;

	ROS_INFO( "Opening %s...",  mDevicePath.c_str());

	// stat file
	if (-1 == stat(mDevicePath.c_str(), &st)) {
		ROS_ERROR( "Cannot identify '%s': %d, %s", mDevicePath.c_str(), errno, strerror(errno));
		return false;
	}

	// check if its device
	if (!S_ISCHR(st.st_mode)) {
		ROS_ERROR("%s is no device", mDevicePath.c_str());
		return false;
	}

	// open device
	mVideoDevice = open(mDevicePath.c_str(), O_RDWR /* required */| O_NONBLOCK, 0);

	// check if opening was successful
	if (-1 == mVideoDevice) {
		ROS_ERROR( "Cannot open '%s': %d, %s", mDevicePath.c_str(), errno, strerror(errno));
		return false;
	}

	return true;
}
#endif
#endif
