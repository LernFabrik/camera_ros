#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/cvstd.hpp>

using namespace cv;
using namespace std;
string homepath = getenv("HOME");
string current_dir_4x4 = homepath + "/ArUco_markers/4x4_Markers";
string current_dir_5x5 = homepath + "/catkin_camera_ws/src/iwtros_aruco/src/5x5_Markers";
string current_dir_6x6 = homepath + "/catkin_camera_ws/src/iwtros_aruco/src/6x6_Markers";
string current_dir_7x7 = homepath + "/catkin_camera_ws/src/iwtros_aruco/src/7x7_Markers";
string current_dir_Exp = homepath + "/catkin_camera_ws/src/iwtros_aruco/src/Experiamental";

int borderBits = 10;
int margins = 1;
int markersX = 1;
int markersY = 1;
int markerLength = 1;
int markerSeparation = 1;

int main(int argc, char **argv){
    cv::Mat outputMarker, outBoard;
    Ptr<aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    Ptr<aruco::GridBoard> board = cv::aruco::GridBoard::create(markersX,markersY,100, float(markerSeparation), dictionary);
    
    Size ImageSize;
    ImageSize.width = markersX * (markerLength + markerSeparation) - markerSeparation + 2*margins;
    ImageSize.height = markersY * (markerLength + markerSeparation) - markerSeparation + 2*margins;

    //board->draw(ImageSize, outBoard, margins, borderBits);
    //imwrite("board.jpg", outBoard);
    
    for(int i=0;i<50;i++)
        {
            aruco::drawMarker(dictionary,i,500,outputMarker,1);
            ostringstream convert;
            String imageName = current_dir_4x4 + "/4X4Marker_500_";
            convert << imageName << i << ".jpg";
            imwrite(convert.str(), outputMarker);
            
        }
   
    return 0;
}
