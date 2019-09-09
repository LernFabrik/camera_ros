#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <ctime>
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Raw Image window";
static const std::string OPENCV_WINDOW_1 = "Camera Calibration";

enum {
    DETECTION = 0,
    CAPTURING = 1,
    CALIBRATED = 2
};


class Camera_Calibration_chess{
    ros::NodeHandle nh_;
    //ros::NodeHandle node_handle;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    vector<vector<Point2f> > imagePoints;
    string outputFilename = "/home/vishnu/catkin_camera_ws/src/iwtros_aruco/params/calibration.yaml";
    Size boardSize;                     //the number of inner corners per one of board dimension
    Size imageSize;
    float squareSize = 0.03f;
    float aspectRatio = 1.0f;
    int flags = 0;
    bool writeExtrinsics = true;
    bool writePoints = true;
    Mat cameraMatrix, distCoeffs;
    int mode = CAPTURING;
    clock_t prevTimestamp = 0;
    int delay = 0;                                      //a minimum delay in ms between subsequent attempts to capture a next view ---
    bool undistortImage = false;
    int nframes = 200;                                    // the number of frames to be used for the calibration
    bool blink = false;
    double imageCounter;


    public:
        Camera_Calibration_chess()
        :it_(nh_){
            //std::string camera_topic;
            //node_handle.getParam("/camera_calibration/camera_topic", camera_topic);
            image_sub_ = it_.subscribe("/kinect2/hd/image_color", 1, &Camera_Calibration_chess::calibrationChess, this);
            image_pub_ = it_.advertise("/cameraCalibration/image_raw", 1);
            cv::namedWindow("Image view");
            mode = CAPTURING;
            flags |= CALIB_FIX_ASPECT_RATIO;
            boardSize.height = 7;
            boardSize.width = 5;
            imageCounter = 0;
            /*std::string flags_type;
            node_handle.getParam("/camera_calibration/flags_type", flags_type);
            if(flags_type == "a") flags |= CALIB_FIX_ASPECT_RATIO;
            if(flags_type == "zt") flags |= CALIB_ZERO_TANGENT_DIST;        //assume zero tangential distortion
            if(flags_type == "p") flags |= CALIB_FIX_PRINCIPAL_POINT;      //fix the principal point at the center

            int board_width, board_hight;
            node_handle.getParam("/camera_calibration/board_width", board_width);
            node_handle.getParam("/camera_calibration/board_hight", board_hight);
            boardSize.height = board_hight;
            boardSize.width = board_width;

            float square_size;
            node_handle.getParam("/camera_calibration/square_size", square_size);
            squareSize = square_size;

            float apect_ratio;
            node_handle.getParam("/camera_calibration/apect_ratio", apect_ratio);
            aspectRatio = apect_ratio;

            double n_frames;
            node_handle.getParam("/camera_calibration/n_frames", n_frames);
            nframes = n_frames;

            std::string outputFile;
            node_handle.getParam("/camera_calibration/outputFile", outputFile);
            outputFilename = outputFile;

            bool write_extrinsic;
            node_handle.getParam("/camera_calibration/write_extrinsic", write_extrinsic);
            writeExtrinsics = apect_ratio;

            bool write_points;
            node_handle.getParam("/camera_calibration/write_points", write_points);
            writePoints = write_points;*/
        }

        ~Camera_Calibration_chess(){
            destroyWindow(OPENCV_WINDOW);
            destroyWindow(OPENCV_WINDOW_1);
        }

        void calibrationChess(const sensor_msgs::ImageConstPtr& msg){
            cv_bridge::CvImagePtr cv_ptr;
            namespace enc = sensor_msgs::image_encodings;
            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                ROS_INFO("Loading image from the ROS to OpenCV");
            }catch(cv_bridge::Exception& e){
                ROS_ERROR("cv bridge error: %s", e.what());
            }

            // cv_ptr->image holds the current camera frame. Type is "Mat"
            cameraCalibration(cv_ptr->image);
            image_pub_.publish(cv_ptr->toImageMsg());

        }

        void cameraCalibration(Mat view){
            Mat viewGray;
            //vector<Mat> imageList;
            //imageList[imageCounter] = view;
            //imageCounter ++;
            //imshow("Camera Live", view);
            if(view.empty()){
                if(imagePoints.size() > 0)
                    runAndSave(outputFilename, imagePoints, imageSize,
                                boardSize, squareSize, aspectRatio, 
                                flags, cameraMatrix, distCoeffs,
                                writeExtrinsics, writePoints);
                ros::shutdown();
            }
            ROS_INFO("Image is not empty");
            imageSize = view.size();
            /* write if programme requires to flip the camera*/
            vector<Point2f> pointbuf;
            cvtColor(view, viewGray, COLOR_BGR2GRAY);

            bool found;
            
            /* In the case of chess board */
            found = findChessboardCorners(view, boardSize, pointbuf,
                                            CV_CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
            ROS_INFO("Chess baord found successful");
            //improve the found corners' coordinate accuracy
            if(found) cornerSubPix(viewGray, pointbuf, Size(11,11), 
                                    Size(-1,-1), TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1));
            
            if(mode == CAPTURING && found){ 
                imagePoints.push_back(pointbuf);
                prevTimestamp = clock();
                ROS_INFO("copied the buffer data to image Points: %f", (float)imagePoints.size());
            }

            if(found){
                drawChessboardCorners(view, boardSize, Mat(pointbuf), found);
                ROS_INFO("Chess board drawing the corners");
            }
                
            
            string msgs = mode == CAPTURING ? "100/100" : mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
            int baseLine = 0;
            Size textSize = getTextSize(msgs, 1, 1, 1, &baseLine);
            Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine -10);

            if (mode == CAPTURING){
                if(undistortImage){
                    msgs = format( "%d/%d Undist", (int)imagePoints.size(), nframes );
                    ROS_INFO("Image is distorted");
                }else{
                    msgs = format( "%d/%d", (int)imagePoints.size(), nframes );
                    ROS_INFO("Image is not distorted");
                }
            }

            putText (view, msgs, textOrigin, 1, 1,
                        mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));
            
            /* In the example OpenCV calibration.cpp it is given that boolean variable "blink" will monitor
                the of camera is open or not. If the camera is open then perfor bitwise not for the current 
                image frame -----------------------------------------------------------------------------*/
            bitwise_not(view, view);

            if(mode == CALIBRATED && undistortImage){
                Mat temp = view.clone();
                undistort(temp, view, cameraMatrix, distCoeffs);
                ROS_INFO("Image is not distorted in if case mode == CALIBRATED");
            }

            imshow ("Image view", view);
            char key = (char)waitKey(3);
            if(key == 27) ros::shutdown();

            if(key == 'u' && mode == CALIBRATED)
                undistortImage = !undistortImage;
            
            if(key == 'g'){
                mode = CAPTURING;
                imagePoints.clear();
            }

            if(mode==CAPTURING && imagePoints.size() >= (unsigned)nframes){
                ROS_INFO("Calibration is running please wait.......................");
                if(runAndSave(outputFilename, imagePoints, imageSize,
                                boardSize, squareSize, aspectRatio, 
                                flags, cameraMatrix, distCoeffs, 
                                writeExtrinsics, writePoints)){
                    ROS_INFO("Calibration is successful and saving the the data...! 'Now ROS will shutdown'");
                    mode = CALIBRATED;
                    imageCounter = 0;
                    //showUndistortedImages(imageList);
                    ros::shutdown();
                }else{   
                    mode = CAPTURING;
                }
                
                /* in case if came is not open */
            }
            
        }

        void showUndistortedImages(vector<Mat>& imagesInList){
            ROS_INFO("Showing undistorted Images");
            Mat view, rview, map1, map2;
            initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                                    getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                                    imageSize, CV_16SC2, map1, map2);
            for(int i = 0; i < (int)imagesInList.size(); i++){
                view = imagesInList[i];
                if(view.empty()) continue;
                remap(view, rview, map1, map2, INTER_LINEAR);
                imshow("Image View", rview);
                char c = (char)waitKey();
                if(c==27 || c=='q' || c=='Q') ros::shutdown();
            }
        }


        static void calcChessboardCorners (Size boardSize, float squareEdgeLength, vector<Point3f>& corners){
            for(int i = 0; i < boardSize.height; i++){
                for(int j = 0; j < boardSize.width; j++){
                    corners.push_back(Point3f(float(j*squareEdgeLength), float(i*squareEdgeLength), 0.0f));
                }
            }
        }

        static double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
                                                const vector<vector<Point2f> >& imagePoints,
                                                const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                                const Mat& cameraMatrix, const Mat& distCoeffs,
                                                vector<float>& perViewErrors ){
            vector<Point2f> imagePoints2;
            int i, totalPoints = 0;
            double totalErr = 0, err;
            perViewErrors.resize(objectPoints.size());

            for( i = 0; i < (int)objectPoints.size(); i++ )
            {
                projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                            cameraMatrix, distCoeffs, imagePoints2);
                err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
                int n = (int)objectPoints[i].size();
                perViewErrors[i] = (float)std::sqrt(err*err/n);
                totalErr += err*err;
                totalPoints += n;
            }

            return std::sqrt(totalErr/totalPoints);
        }

        static bool runAndSave(const string& ouutPutFilename,
                                const vector<vector<Point2f> >& imagePoints,
                                Size imageSize, Size boardSize, float squareSize,
                                float aspectRatio, int flags, Mat& cameraMatrix, 
                                Mat& distCoeffs, bool writeExtrinsics, bool writePoints){
            
            vector<Mat> rvecs, tvecs;
            vector<float> reprojErrs;
            double totalAvgErr = 0;

            bool ok = runCalibration(imagePoints, imageSize, boardSize, squareSize,
                                        aspectRatio, flags, cameraMatrix, distCoeffs, 
                                        rvecs, tvecs, reprojErrs, totalAvgErr);
            ROS_INFO("%s. avg reprojection error = %.2f\n", ok ? "Calibration succeeded" : "Calibration failed", totalAvgErr);
            if(ok){
                saveCameraParams(ouutPutFilename, imageSize, 
                                    boardSize, squareSize, aspectRatio,
                                    flags, cameraMatrix, distCoeffs,
                                    writeExtrinsics ? rvecs : vector<Mat>(),
                                    writeExtrinsics ? tvecs : vector<Mat>(),
                                    writeExtrinsics ? reprojErrs : vector<float>(),
                                    writePoints ? imagePoints : vector<vector<Point2f> >(),
                                    totalAvgErr);
                return ok;
            }

        }

        static bool runCalibration( vector<vector<Point2f> > imagePoints,
                    Size imageSize, Size boardSize,
                    float squareSize, float aspectRatio,
                    int flags, Mat& cameraMatrix, Mat& distCoeffs,
                    vector<Mat>& rvecs, vector<Mat>& tvecs,
                    vector<float>& reprojErrs,
                    double& totalAvgErr){
            
            cameraMatrix = Mat::eye(3,3,CV_64F);
            if(flags & CALIB_FIX_ASPECT_RATIO)
                cameraMatrix.at<double>(0,0) = aspectRatio;
            
            distCoeffs = Mat::zeros(8, 1, CV_64F);

            vector<vector<Point3f> > objectPoints(1);
            calcChessboardCorners(boardSize, squareSize, objectPoints[0]);

            objectPoints.resize(imagePoints.size(),objectPoints[0]);

            double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                            distCoeffs, rvecs, tvecs, flags|CALIB_FIX_K4|CALIB_FIX_K5);
                            ///*|CALIB_FIX_K3*/|CALIB_FIX_K4|CALIB_FIX_K5);
            ROS_INFO("RMS error reported by calibrateCamera: %g\n", rms);

            bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

            totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                        rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

            return ok;
        }

        static void saveCameraParams( const string& filename,
                       Size imageSize, Size boardSize,
                       float squareSize, float aspectRatio, int flags,
                       const Mat& cameraMatrix, const Mat& distCoeffs,
                       const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                       const vector<float>& reprojErrs,
                       const vector<vector<Point2f> >& imagePoints,
                       double totalAvgErr ){

                FileStorage fs( filename, FileStorage::WRITE );

                time_t tt;
                time( &tt );
                struct tm *t2 = localtime( &tt );
                char buf[1024];
                strftime( buf, sizeof(buf)-1, "%c", t2 );

                fs << "calibration_time" << buf;

                if( !rvecs.empty() || !reprojErrs.empty() )
                    fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
                fs << "image_width" << imageSize.width;
                fs << "image_height" << imageSize.height;
                fs << "board_width" << boardSize.width;
                fs << "board_height" << boardSize.height;
                fs << "square_size" << squareSize;

                if( flags & CALIB_FIX_ASPECT_RATIO )
                    fs << "aspectRatio" << aspectRatio;

                if( flags != 0 )
                {
                    sprintf( buf, "flags: %s%s%s%s",
                        flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                        flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                        flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                        flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
                    //cvWriteComment( *fs, buf, 0 );
                }

                fs << "flags" << flags;

                fs << "camera_matrix" << cameraMatrix;
                fs << "distortion_coefficients" << distCoeffs;

                fs << "avg_reprojection_error" << totalAvgErr;
                if( !reprojErrs.empty() )
                    fs << "per_view_reprojection_errors" << Mat(reprojErrs);

                if( !rvecs.empty() && !tvecs.empty() )
                {
                    CV_Assert(rvecs[0].type() == tvecs[0].type());
                    Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
                    for( int i = 0; i < (int)rvecs.size(); i++ )
                    {
                        Mat r = bigmat(Range(i, i+1), Range(0,3));
                        Mat t = bigmat(Range(i, i+1), Range(3,6));

                        CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
                        CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
                        //*.t() is MatExpr (not Mat) so we can use assignment operator
                        r = rvecs[i].t();
                        t = tvecs[i].t();
                    }
                    //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
                    fs << "extrinsic_parameters" << bigmat;
                }

                if( !imagePoints.empty() )
                {
                    Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
                    for( int i = 0; i < (int)imagePoints.size(); i++ )
                    {
                        Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
                        Mat imgpti(imagePoints[i]);
                        imgpti.copyTo(r);
                    }
                    fs << "image_points" << imagePtMat;
                }
            }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cameraCalibration");
    Camera_Calibration_chess ic;
    ros::spin();
    return 0;
}