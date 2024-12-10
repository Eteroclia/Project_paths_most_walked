#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "iostream"
#include <string>

int showImage() {
    std::string path = "cat.jpg";
    cv::Mat img = cv::imread(path);
    imshow("Portrait", img);
    cv::waitKey(0);
    return 0;
}

int showWebCameraContent() {
    // open the first webcam plugged in the computer
    cv::VideoCapture camera(0,cv::CAP_DSHOW);
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    // create a window to display the images from the webcam
    cv::namedWindow("Webcam");
    int cam=0;
    char saved='a';
    // this will contain the image from the webcam
    cv::Mat frame;
    // display the frame until you press a key
    while (1) {
        // capture the next frame from the webcam
        camera >> frame;
        // show the image on the window
        cv::imshow("Webcam", frame);
        // wait (10ms) for a key to be pressed
        if (cv::waitKey(5) == 'd') {
            cam++;
            
            if(cam>2)
                break;
           camera.open(cam,cv::CAP_DSHOW);
           printf("Now showing %i\n",cam);
        }else if(cv::waitKey(5) == 'a'){
            cam--;
            
            if(cam<0)
                break;
           camera.open(cam,cv::CAP_DSHOW);
           printf("Now showing %i\n",cam);
        }else if(cv::waitKey(5)=='s'){
            printf("saving image\n");
            char out[]="./images/img1.png";
            printf(out);
            printf("\n");
            out[12]=saved;
            printf(out);
            printf("\n");
            saved++;
            cv::imwrite(out, frame);
        }
    }
    return 0;
}

int main(int, char**) {
   //showImage();
   showWebCameraContent();
}