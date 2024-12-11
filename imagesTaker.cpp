#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "iostream"
#include <string>


int showWebCameraContent() {
    // open the first webcam plugged in the computer
    cv::VideoCapture camera(0,cv::CAP_DSHOW),camera2(1,cv::CAP_DSHOW);
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    // create a window to display the images from the webcam
    cv::namedWindow("Webcam1");
    cv::namedWindow("Webcam2");
    char saved='a';
    // this will contain the image from the webcam
    cv::Mat frame,frame2;
    // display the frame until you press a key
    while (1) {
        // capture the next frame from the webcam
        camera >> frame;
        camera2 >> frame2;
        // show the image on the window
        cv::imshow("Webcam1", frame);
        cv::imshow("Webcam2", frame2);
        // wait (10ms) for a key to be pressed
        if(cv::waitKey(5)=='s'){
            printf("saving image\n");
            std::string out="./images/a_imgn.png";
            std::string out2="./images/b_imgn.png";
            out[14]=saved;
            out2[14]=saved;
            saved++;
            cv::imwrite(out, frame);
            cv::imwrite(out2, frame2);
        }
        if(cv::waitKey(5)=='q')
            break;
    }
    return 0;
}

int main(int, char**) {
   //showImage();
   showWebCameraContent();
}