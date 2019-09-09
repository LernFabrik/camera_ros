#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Raw Image window";
static const std::string OPENCV_WINDOW_1 = "Edge Detection";

class Edge_Detection{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    public:
        Edge_Detection()
            : it_(nh_){
                image_sub_ = it_.subscribe("/kinect2/hd/image_color", 1,
                    &Edge_Detection::imageCb, this);
                image_pub_ = it_.advertise("/edge_detection/raw_image", 1);
                cv::namedWindow(OPENCV_WINDOW);
                cv::namedWindow(OPENCV_WINDOW_1);
                ROS_ERROR("Edge detection class call success");
            }

        ~Edge_Detection(){
            cv::destroyWindow(OPENCV_WINDOW);
        }

        void imageCb(const sensor_msgs::ImageConstPtr& msg){
            cv_bridge::CvImagePtr cv_ptr;
            namespace enc = sensor_msgs::image_encodings;
            
            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                ROS_INFO("cv_bridge::toCvCopy success");
            }catch(cv_bridge::Exception& e){
                ROS_ERROR("cv bridge exception: %s", e.what());
                return;
            }

            ROS_INFO("image size success");
                detect_edges(cv_ptr->image);
                image_pub_.publish(cv_ptr->toImageMsg());
            // Draw an example circle on th vedio stream
            //if(cv_ptr->image.cols > 400 && cv_ptr->image.rows > 500){
              //  ROS_INFO("image size success");
            //    detect_edges(cv_ptr->image);
              //  image_pub_.publish(cv_ptr->toImageMsg());
            //}
        }

        void detect_edges (cv::Mat img){
            ROS_INFO("detect_edge function success");
            cv::Mat src, src_gray;
            cv::Mat dst, detect_edge;

            int edgeThresh = 1;
            int lowThreshold = 100;
            int highThreshold = 200;
            int kernel_size = 5;

            img.copyTo(src);
            cv::cvtColor(img, src_gray, CV_BGR2GRAY);
            ROS_INFO("gray success");
            cv::blur(src_gray, detect_edge, cv::Size(5,5));
            cv::Canny(detect_edge, detect_edge, lowThreshold, highThreshold, kernel_size);
            dst = cv::Scalar::all(0);
            img.copyTo(dst, detect_edge);
            dst.copyTo(img);
            ROS_INFO("Canny filter success");

            cv::imshow(OPENCV_WINDOW, src);
            cv::imshow(OPENCV_WINDOW_1, dst);
            cv::waitKey(3);
        }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Edge_Detection");
    Edge_Detection ic;
    ros::spin();
    return 0;
}
