#include "iwtros_aruco/aruco_detector.hpp"

namespace iwtros{
    namespace aruco {
        detectAruco::detectAruco(const std::string cameraTopic,  const std::string paramFilename, float squareDimention) : it_(nh_){
            //image_sub_ = it_.subscribe(cameraTopic, 1, &detectAruco::arucoDetection, this);
            image_sub_ = it_.subscribe(cameraTopic, 1, &detectAruco::arucoPoseDetection, this);
            image_pub_ = it_.advertise("/image_converted/output_video", 10);
            aruco_pub_ = nh_.advertise<nav_msgs::Odometry >("vo", 20);
            namedWindow(OPENCV_WINDOW);

            tfMatrix_ = Mat::zeros(4,4,CV_64F);
            orientation_ = Mat::zeros(4,1,CV_64F);
            orientation_.at<double>(3,0) = 1.0;
            prev_orientation_= Mat::zeros(4,1,CV_64F);
            prev_orientation_.at<double>(3,0);
            position_ = Mat::zeros(3,1,CV_64F);
            prev_position_ = Mat::zeros(3,1,CV_64F);
            camera_params_.Filename = paramFilename;
            arucoSquareDimention = squareDimention;
            ROS_INFO("Hello from the class ----------------------------");
        }

        /*detectAruco::~detectAruco(const std::string cameraTopic,  const std::string paramFilename, float squareDimention){
            destroyWindow(OPENCV_WINDOW);
        }*/

        bool detectAruco::loadCameraCalibration(Mat& cameraMatrix, Mat& distCoeffs){
            FileStorage fs(camera_params_.Filename, FileStorage::READ);
            fs["camera_matrix"] >> cameraMatrix;
            fs["distortion_coefficients"] >> distCoeffs;

            if(cameraMatrix.empty() && distCoeffs.empty()){
                ROS_INFO("Camre Matrix and distorted coeffs in not loaded");
                return false;
            }else{
                //ROS_INFO("Camera matrix and distorted coeffs is successfully loaded");
                return true;
            }
            cout << "Camera Matrix = " << endl << " " << cameraMatrix << endl << endl;
        }

        Mat detectAruco::rot2euler(const Mat& rotationMatrix){
            Mat euler(3,1,CV_64F);

            double a11 = rotationMatrix.at<double>(0,0);
            double a12 = rotationMatrix.at<double>(0,1);
            double a13 = rotationMatrix.at<double>(0,2);

            double a21 = rotationMatrix.at<double>(1,0);
            double a22 = rotationMatrix.at<double>(1,1);
            double a23 = rotationMatrix.at<double>(1,2);

            double a31 = rotationMatrix.at<double>(2,0);
            double a32 = rotationMatrix.at<double>(2,1);
            double a33 = rotationMatrix.at<double>(2,2);

            double qr = 0.5*sqrt(1+a11+a22+a33);
            double qi = (a32-a23)/(4*qr);
            double qj = (a13-a31)/(4*qr);
            double qk = (a21-a12)/(4*qr);

            double x, y, z;

            x = atan2(2*(qr*qi + qj*qk), (1-2*(qi*qi + qj*qj)));
            y = asin(2*(qr*qj - qi*qk));
            z = atan2(2*(qr*qk + qj*qi), (1-2*(qk*qk + qj*qj)));
            
            euler.at<double>(0) = x;
            euler.at<double>(1) = y;
            euler.at<double>(2) = z;

            return euler;
        }

        Mat detectAruco::rot2quat(const Mat& rotationMatrix){

            Mat quat(4, 1, CV_64F);

            double a11 = rotationMatrix.at<double>(0, 0);
            double a12 = rotationMatrix.at<double>(0, 1);
            double a13 = rotationMatrix.at<double>(0, 2);

            double a21 = rotationMatrix.at<double>(1, 0);
            double a22 = rotationMatrix.at<double>(1, 1);
            double a23 = rotationMatrix.at<double>(1, 2);
            
            double a31 = rotationMatrix.at<double>(2, 0);
            double a32 = rotationMatrix.at<double>(2, 1);       
            double a33 = rotationMatrix.at<double>(2, 2);

            double qr = 0.5*sqrt(1+a11+a22+a33);
            quat.at<double>(0) = qr;
            quat.at<double>(1) = (a32-a23)/(4*qr);
            quat.at<double>(2) = (a13-a31)/(4*qr);
            quat.at<double>(3) = (a21-a12)/(4*qr);
            
            return quat;
        }

        void detectAruco::getTramsformMat(int markerId, Mat& tfMat){
            switch (markerId)
            {
                case 0:
                    tfMat.at<double>(0,0) = 0;
                    tfMat.at<double>(0,1) = 0;
                    tfMat.at<double>(0,2) = -1;
                    tfMat.at<double>(0,3) = 8;
                    
                    tfMat.at<double>(1,0) = -1;
                    tfMat.at<double>(1,1) = 0;
                    tfMat.at<double>(1,2) = 0;
                    tfMat.at<double>(1,3) = 0;
                    
                    tfMat.at<double>(2,0) = 0;
                    tfMat.at<double>(2,1) = 1;
                    tfMat.at<double>(2,2) = 0;
                    tfMat.at<double>(2,3) = 1;
                    
                    tfMat.at<double>(3,0) = 0;
                    tfMat.at<double>(3,1) = 0;
                    tfMat.at<double>(3,2) = 0;
                    tfMat.at<double>(3,3) = 1;
                    break;
                
                case 1:
                    tfMat.at<double>(0,0) = 0;
                    tfMat.at<double>(0,1) = 0;
                    tfMat.at<double>(0,2) = -1;
                    tfMat.at<double>(0,3) = 9.88;
                    
                    tfMat.at<double>(1,0) = -1;
                    tfMat.at<double>(1,1) = 0;
                    tfMat.at<double>(1,2) = 0;
                    tfMat.at<double>(1,3) = -5;
                    
                    tfMat.at<double>(2,0) = 0;
                    tfMat.at<double>(2,1) = 1;
                    tfMat.at<double>(2,2) = 0;
                    tfMat.at<double>(2,3) = 1;
                    
                    tfMat.at<double>(3,0) = 0;
                    tfMat.at<double>(3,1) = 0;
                    tfMat.at<double>(3,2) = 0;
                    tfMat.at<double>(3,3) = 1;
                    
                    break;
            
                default:
                    tfMat = Mat::zeros(4,4,CV_64F);
                    break;
            }
        }

        void detectAruco::arucoDetection(const sensor_msgs::ImageConstPtr& msg){
            /* ----- Use the function only to detect the markers in the environment---------*/
            vector<int> markerId;
            vector<vector<Point2f> > markerCorners;
            vector<Vec3d> rvec, tvec;
            Mat rotMatrix = Mat::zeros(3,3,CV_64F);
            Mat rotAxis = Mat::zeros(3,3,CV_64F);
            Mat tempPose = Mat::zeros(4,1,CV_64F);
            Mat tTemp = Mat::zeros(3,1,CV_64F);

            Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);

            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }catch(cv_bridge::Exception& e){
                ROS_ERROR("cv bridge error: %s", e.what());
                return;
            }

            Mat view, viewCopy;
            view = cv_ptr->image;
            //ROS_INFO("Image is successfully loaded");
            cv::aruco::detectMarkers(view, markerDictionary, markerCorners, markerId);
            bool cameraCalibration = detectAruco::loadCameraCalibration(camera_params_.cameraMatrix, camera_params_.distCoeffs);
            if(markerId.size() > 0 && cameraCalibration){
                ROS_INFO("Markers found: %d", (int)markerId.size());
                cv::aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimention,
                                                    camera_params_.cameraMatrix, camera_params_.distCoeffs, rvec, tvec);
            }

            view.copyTo(viewCopy);
            if(markerId.size() > 0){
                cv::aruco::drawDetectedMarkers(viewCopy, markerCorners, markerId);
                if(cameraCalibration){
                    for(unsigned int i = 0; i < markerId.size(); i++){
                        cv::aruco::drawAxis(viewCopy, camera_params_.cameraMatrix, 
                                            camera_params_.distCoeffs, rvec[i], tvec[i], 
                                            arucoSquareDimention * 0.5f);
                    }
                }
            }
            imshow(OPENCV_WINDOW, viewCopy);
            //image_pub_.publish(cv_ptr->toImageMsg);
            char key = (char)waitKey(3);
            if(key == 27) ros::shutdown();
        }

        void detectAruco::arucoPoseDetection(const sensor_msgs::ImageConstPtr& msg){
            vector<int> markerId;
            vector<vector<Point2f> > markerCorners;
            vector<Vec3d> rvec, tvec;
            Mat rotMatrix = Mat::zeros(3,3,CV_64F);
            Mat rotAxis = Mat::zeros(3,3,CV_64F);
            Mat tempPose = Mat::zeros(4,1,CV_64F);
            Mat tTemp = Mat::zeros(3,1,CV_64F);

            Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }catch(cv_bridge::Exception& e){
                ROS_ERROR("cv bridge error: %s", e.what());
                return;
            }

            Mat view, viewCopy;
            view = cv_ptr->image;
            //ROS_INFO("Image is successfully loaded");
            cv::aruco::detectMarkers(view, markerDictionary, markerCorners, markerId);
            bool cameraCalibration = detectAruco::loadCameraCalibration(camera_params_.cameraMatrix, camera_params_.distCoeffs);
            if(markerId.size() > 0 && cameraCalibration){
                ROS_INFO("Markers found: %d", (int)markerId.size());
                cv::aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimention,
                                                    camera_params_.cameraMatrix, camera_params_.distCoeffs, rvec, tvec);
            }

            prev_position_ = position_;
            prev_orientation_ = orientation_;
            ros::Rate r(20.0);
            view.copyTo(viewCopy);
            if(markerId.size() > 0){
                cv::aruco::drawDetectedMarkers(viewCopy, markerCorners, markerId);
                if(cameraCalibration){
                    for(unsigned int i = 0; i < markerId.size(); i++){
                        cv::aruco::drawAxis(viewCopy, camera_params_.cameraMatrix, 
                                            camera_params_.distCoeffs, rvec[i], tvec[i], 
                                             arucoSquareDimention*0.5f);
                        cv::Rodrigues(rvec[i], rotMatrix);
                        //cout << "Rotation Matrix:" << " " << rotMatrix.t() << endl;
                        for(int j = 0; j < 3; j++) tTemp.at<double>(j, 0) = tvec[i][j];
                        position_ = -(rotMatrix.t()) * tTemp; //Pose w.r.t to the camera
                        cout << "Marker Position : " << markerId[i] << " " << position_.t() << endl; // remove other cout except this one

                        /*for(int k = 0; k < 3; k++) tempPose.at<double>(k, 0) = position_.at<double>(k,0);
                        tempPose.at<double>(3,0) = 1;

                        detectAruco::getTramsformMat(markerId[i], tfMatrix_);
                        tempPose = tfMatrix_*tempPose; //camera Pose at the world frame: 1 exatra element

                        for(int k = 0; k<3; k++) position_.at<double>(k,0) = tempPose.at<double>(k,0);  //final camera pose at the world frame
                        cout << "Marker Position after the transformation: " << markerId[i] << " " << position_.t() << endl;

                        //Rotation of each marker w.r.t World frame
                        for(int x = 0; x < 3; x++){
                            for(int y = 0; y < 3; y++){
                                rotAxis.at<double>(x, y) = tfMatrix_.at<double>(x, y);
                            }
                        }*/

                        rotMatrix =  rotMatrix.t();
                        orientation_ = rot2euler(rotMatrix);
                        cout << "Marker Orientation:" << markerId[i] << " " << orientation_.t() << endl;
                        orientation_ = rot2quat(rotMatrix);

                        //geometry_msgs::TransformStamped arucoOdom_trans;  //for tf
                        nav_msgs::Odometry aruco_odom;
                        aruco_odom.header.stamp = ros::Time::now();
                        aruco_odom.header.frame_id = "vo";

                        aruco_odom.pose.pose.position.x = position_.at<double>(0,0);
                        aruco_odom.pose.pose.position.y = position_.at<double>(1,0);
                        aruco_odom.pose.pose.position.z = position_.at<double>(2,0);

                        aruco_odom.pose.pose.orientation.x = orientation_.at<double>(0,0);
                        aruco_odom.pose.pose.orientation.y = orientation_.at<double>(1,0);
                        aruco_odom.pose.pose.orientation.z = orientation_.at<double>(2,0);
                        aruco_odom.pose.pose.orientation.w = orientation_.at<double>(3,0);

                        aruco_pub_.publish(aruco_odom);

                    }
                }
            }
            imshow(OPENCV_WINDOW, viewCopy);
            //image_pub_.publish(cv_ptr->toImageMsg);
            char key = (char)waitKey(3);
            if(key == 27) ros::shutdown();
        }
    }
}