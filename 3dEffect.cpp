#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>


int main()
{
    // Check for left and right camera IDs
    int CamL_id{1}; // Camera ID for left camera
    int CamR_id{0}; // Camera ID for right camera

    cv::VideoCapture camL(CamL_id,cv::CAP_DSHOW), camR(CamR_id,cv::CAP_DSHOW);
    
    cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
    cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;

    cv::FileStorage cv_file = cv::FileStorage("./data/stereo_rectify_maps.xml", cv::FileStorage::READ);
    cv_file["Left_Stereo_Map_x"] >> Left_Stereo_Map1;
    cv_file["Left_Stereo_Map_y"] >> Left_Stereo_Map2;
    cv_file["Right_Stereo_Map_x"] >> Right_Stereo_Map1;
    cv_file["Right_Stereo_Map_y"] >> Right_Stereo_Map2;
    cv_file.release();

    // Check if left camera is attched
    if (!camL.isOpened())
    {
        std::cout << "Could not open camera with index : " << CamL_id << std::endl;
        return -1;
    }

    // Check if right camera is attached
    if (!camL.isOpened())
    {
        std::cout << "Could not open camera with index : " << CamL_id << std::endl;
        return -1;
    }

    cv::Mat frameL, frameR;
    
    for (size_t i{0}; i<100000; i++)
    {
        camL >> frameL;
        camR >> frameR;

        cv::Mat Left_nice, Right_nice;

        cv::remap(frameL,
                Left_nice,
                Left_Stereo_Map1,
                Left_Stereo_Map2,
                cv::INTER_LANCZOS4,
                cv::BORDER_CONSTANT,
                0);

        cv::remap(frameR,
                Right_nice,
                Right_Stereo_Map1,
                Right_Stereo_Map2,
                cv::INTER_LANCZOS4,
                cv::BORDER_CONSTANT,
                0);


        cv::Mat Left_nice_split[3], Right_nice_split[3];

        std::vector<cv::Mat> Anaglyph_channels;

        cv::split(Left_nice, Left_nice_split);
        cv::split(Right_nice, Right_nice_split);

        Anaglyph_channels.push_back(Left_nice_split[0]);
        Anaglyph_channels.push_back(Left_nice_split[1]);
        Anaglyph_channels.push_back(Right_nice_split[2]);

        cv::Mat Anaglyph_img;

        cv::merge(Anaglyph_channels, Anaglyph_img);

        cv::imshow("Anaglyph image", Anaglyph_img);
        cv::waitKey(1);
    }
    
    return 0;
}