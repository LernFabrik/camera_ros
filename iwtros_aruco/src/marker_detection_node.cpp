#include "iwtros_aruco/aruco_detector.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

string homepath = getenv("HOME");
string currentDir = homepath + "/catkin_camera_ws/src/iwtros_aruco/params/calibration.yaml";
string cameraTopic = "/kinect2/hd/image_color";
const float arucoDimention = 0.129f;

int main(int argc, char** argv){
    ros::init(argc, argv, "aruco_detection");
    iwtros::aruco::detectAruco det(cameraTopic, currentDir, arucoDimention);
    ros::spin();
    return 0;
}