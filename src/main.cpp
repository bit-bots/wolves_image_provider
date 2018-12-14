/*
 * main.cpp
 *
 *  Created on: 16.01.2016
 *      Author: Tobias Bolze
 *      Image Provider Node for the camera Logitech C920.
 */

#include "opencv2/opencv.hpp"
#include "CaptureV4L2.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

using namespace cv;

int main(int argc, char **argv) {
	ros::init(argc, argv, "Image Provider v4l");
	ros::NodeHandle n;
	ros::NodeHandle camera_nh_("camera");
	image_transport::ImageTransport it(n);
    string image_topic = "camera";
	ros::param::get("/ros_stuff/image_pub_msg", image_topic);
	image_transport::CameraPublisher pub = it.advertiseCamera(image_topic, 1);
	image_transport::Publisher pubYUY = it.advertise("rawimageYUV", 1);
	ros::Publisher pubCamInfo = n.advertise<sensor_msgs::CameraInfo>("/camera_info", 1);

	//Inital Path used when no parameter is set
	string devPath = "/dev/video0";
	ros::param::get("/camera_v4l/device", devPath);

	int imgWidth = 0;
	int imgHeight = 0;
	ros::param::get("/camera_v4l/resolution/width",imgWidth);
	ros::param::get("/camera_v4l/resolution/height",imgHeight);
	CaptureV4L2 v4lCap(devPath);
	Mat cvmat = Mat(imgHeight, imgWidth, CV_8UC2);
	Mat rgb(imgHeight, imgWidth, CV_8UC3);
	sensor_msgs::Image image_message;
	sensor_msgs::Image image_message_yuv2;
	sensor_msgs::CameraInfo camera_info_message;

	string camera_name = "camera";
	ros::param::get("ros_stuff/camera_name", camera_name);
	boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_(new camera_info_manager::CameraInfoManager(camera_nh_, camera_name, "package://wolves_image_provider/config/" + camera_name + ".yaml"));

	ros::Rate r(30);
	int seq = 0;
	while (ros::ok()) {
		v4lCap.execute(cvmat);
		//Setting the Time to actual for timestamp in package
		image_message.header.stamp = ros::Time::now();
		image_message.header.frame_id = "camera_optical_frame";
        image_message.header.seq = seq;
        seq++;

		// If subscribed send a Message as RGB (ROS rqt visible)
		if(pub.getNumSubscribers() > 0){
			cvtColor(cvmat, rgb, CV_YUV2BGRA_YUY2);
			cvtColor(rgb, rgb, CV_BGRA2RGB);
			image_message.height = imgHeight;
			image_message.width = imgWidth;
			image_message.encoding = "rgb8";
			image_message.step = imgWidth*3;
			image_message.data.assign(rgb.datastart, rgb.dataend);

            sensor_msgs::CameraInfoPtr
                ciptr(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
            ciptr->header.stamp = image_message.header.stamp;
            ciptr->header.frame_id = image_message.header.frame_id;
            ciptr->header.seq = image_message.header.seq;
            sensor_msgs::CameraInfo ci = *ciptr;
			pub.publish(image_message, ci);

		}

		if(pubYUY.getNumSubscribers() > 0){
			image_message_yuv2.height = imgHeight;
			image_message_yuv2.width = imgWidth;
			image_message_yuv2.encoding = "8UC2";
			image_message_yuv2.step = imgWidth*4;
			image_message_yuv2.data.assign(cvmat.datastart, cvmat.dataend);

			pubYUY.publish(image_message_yuv2);
		}

		r.sleep();
		ros::spinOnce();
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}
