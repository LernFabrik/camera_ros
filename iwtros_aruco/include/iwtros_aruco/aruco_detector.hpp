#ifndef GILBRETH_SUPPORT_IWTROS_ARUCO_DETECT_H
#define GILBRETH_SUPPORT_IWTROS_ARUCO_DETECT_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <sstream>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;
static const string OPENCV_WINDOW = "Image View";

namespace iwtros{
    namespace aruco{
        struct cameraParams{
            Mat cameraMatrix = Mat::eye(3,3,CV_64F);
            Mat distCoeffs;
            std::string Filename;
        };

        class detectAruco{
            ros::NodeHandle nh_;
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub_;
            image_transport::Publisher image_pub_;
            ros::Publisher aruco_pub_;
            Mat position_, prev_position_;
            Mat orientation_, prev_orientation_;
            Mat tfMatrix_;
            cameraParams camera_params_;
            float arucoSquareDimention;
            public:
                detectAruco(const std::string cameraTopic, const std::string paramFilename, float squareDimention);
                void arucoDetection(const sensor_msgs::ImageConstPtr& msg);
                void arucoPoseDetection(const sensor_msgs::ImageConstPtr& msg);
            private:
                void getTramsformMat(int markerId, Mat& tfMat);
                Mat rot2quat(const Mat& rotationMatrix);
                Mat rot2euler(const Mat& rotationMatrix);
                bool loadCameraCalibration(Mat& cameraMatrix, Mat& distCoeffs);
        };
    }
}

#endif