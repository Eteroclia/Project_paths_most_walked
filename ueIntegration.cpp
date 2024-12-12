#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "iostream"
#include <string>
#include <tuple>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include "opencv2/imgcodecs.hpp"
#include <iomanip>


class calibration{
    public:
        cv::VideoCapture cameraLeft;
        cv::VideoCapture cameraRight;


    calibration(int leftCamId,int rightCamId){
        cameraLeft.open(leftCamId,cv::CAP_DSHOW);
        cameraRight.open(rightCamId,cv::CAP_DSHOW);
    }
    
    std::tuple<cv::Mat,cv::Mat> getImage() {
        // open the first webcam plugged in the computer
        if (!cameraLeft.isOpened()||!cameraRight.isOpened()) {
            std::cerr << "ERROR: Could not open camera" << std::endl;
            
        }

        // this will contain the image from the webcam
        cv::Mat frameLeft,frameRight;
       
        // capture the next frame from the webcam
        cameraLeft >> frameLeft;
        cameraRight >> frameRight;

        return std::tuple<cv::Mat,cv::Mat>(frameLeft,frameRight);
    }

    std::tuple<cv::Mat,cv::Mat,cv::Mat,cv::Mat> getStereoMatrixes(std::tuple<cv::Mat,cv::Mat> frames){
        // Defining the dimensions of checkerboard
        int CHECKERBOARD[2]{6,9}; 
        // Creating vector to store vectors of 3D points for each checkerboard image
        std::vector<std::vector<cv::Point3f> > objpoints;
        // Creating vector to store vectors of 2D points for each checkerboard image
        std::vector<std::vector<cv::Point2f> > imgpointsL, imgpointsR;
        // Defining the world coordinates for 3D points
        std::vector<cv::Point3f> objp;
        for(int i{0}; i<CHECKERBOARD[1]; i++)
        {
            for(int j{0}; j<CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j,i,0));
        }

        cv::Mat frameL, frameR, grayL, grayR;
        // vector to store the pixel coordinates of detected checker board corners 
        std::vector<cv::Point2f> corner_ptsL, corner_ptsR;
        bool successL, successR;

        frameL = std::get<0>(frames);
        cv::cvtColor(frameL,grayL,cv::COLOR_BGR2GRAY);

        frameR = std::get<1>(frames);
        cv::cvtColor(frameR,grayR,cv::COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true  
        successL = cv::findChessboardCorners(grayL,cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]),corner_ptsL);
        // cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        successR = cv::findChessboardCorners(grayR,cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]),corner_ptsR);
        // cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        /*
        * If desired number of corner are detected,
        * we refine the pixel coordinates and display 
        * them on the images of checker board
        */
        if((successL) && (successR))
        {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(grayL,corner_ptsL,cv::Size(11,11), cv::Size(-1,-1),criteria);
            cv::cornerSubPix(grayR,corner_ptsR,cv::Size(11,11), cv::Size(-1,-1),criteria);

            // Displaying the detected corner points on the checker board
            //cv::drawChessboardCorners(frameL, cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_ptsL,successL);
            //cv::drawChessboardCorners(frameR, cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_ptsR,successR);

            objpoints.push_back(objp);
            imgpointsL.push_back(corner_ptsL);
            imgpointsR.push_back(corner_ptsR);
        }
        cv::Mat mtxL,distL,R_L,T_L;
        cv::Mat mtxR,distR,R_R,T_R;
        /*
            * Performing camera calibration by 
            * passing the value of known 3D points (objpoints)
            * and corresponding pixel coordinates of the 
            * detected corners (imgpoints)
        */
        cv::Mat new_mtxL, new_mtxR;

        // Calibrating left camera
        cv::calibrateCamera(objpoints,imgpointsL,grayL.size(),mtxL,distL,R_L,T_L);

        new_mtxL = cv::getOptimalNewCameraMatrix(mtxL,distL,grayL.size(),1,grayL.size(),0);

        // Calibrating right camera
        cv::calibrateCamera(objpoints,imgpointsR,grayR.size(),mtxR,distR,R_R,T_R);

        new_mtxR = cv::getOptimalNewCameraMatrix(mtxR,distR,grayR.size(),1,grayR.size(),0);
        
        // Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat 
        // are calculated. Hence intrinsic parameters are the same.
        cv::Mat Rot, Trns, Emat, Fmat;

        int flag = 0;
        flag |= cv::CALIB_FIX_INTRINSIC;

        // This step is performed to transform between the two cameras and calculate Essential and 
        // Fundamental matrix
        cv::stereoCalibrate(objpoints,imgpointsL,imgpointsR,new_mtxL,distL,new_mtxR,distR,grayR.size(),Rot,Trns,Emat,Fmat,flag,
                            cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));

        cv::Mat rect_l, rect_r, proj_mat_l, proj_mat_r, Q;

        // Once we know the transformation between the two cameras we can perform 
        // stereo rectification
        cv::stereoRectify(new_mtxL,distL,new_mtxR,distR,grayR.size(),Rot,Trns,rect_l,rect_r,proj_mat_l,proj_mat_r,Q,1);

        // Use the rotation matrixes for stereo rectification and camera intrinsics for undistorting the image
        // Compute the rectification map (mapping between the original image pixels and 
        // their transformed values after applying rectification and undistortion) for left and right camera frames
        cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
        cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;

        cv::initUndistortRectifyMap(new_mtxL,distL,rect_l,proj_mat_l,grayR.size(),CV_16SC2,Left_Stereo_Map1,Left_Stereo_Map2);

        cv::initUndistortRectifyMap(new_mtxR,distR,rect_r,proj_mat_r,grayR.size(),CV_16SC2,Right_Stereo_Map1,Right_Stereo_Map2);

        // Initialize matrix for rectified stereo images
        cv::Mat Left_nice;
        cv::Mat Right_nice;

        // Applying stereo image rectification on the left image
        cv::remap(grayL,Left_nice,Left_Stereo_Map1,Left_Stereo_Map2,cv::INTER_LANCZOS4,cv::BORDER_CONSTANT,0);

        // Applying stereo image rectification on the right image
        cv::remap(grayR,Right_nice,Right_Stereo_Map1,Right_Stereo_Map2,cv::INTER_LANCZOS4,cv::BORDER_CONSTANT,0);
        return std::tuple<cv::Mat,cv::Mat,cv::Mat,cv::Mat>(Left_Stereo_Map1,Left_Stereo_Map2,Right_Stereo_Map1,Right_Stereo_Map2);
    }
};


class depthDetection{
    public:
        cv::VideoCapture cameraLeft;
        cv::VideoCapture cameraRight;
        // initialize values for StereoSGBM parameters
        int numDisparities = 8;
        int blockSize = 5;
        int preFilterType = 1;
        int preFilterSize = 1;
        int preFilterCap = 31;
        int minDisparity = 0;
        int textureThreshold = 10;
        int uniquenessRatio = 15;
        int speckleRange = 0;
        int speckleWindowSize = 0;
        int disp12MaxDiff = -1;
        float M = 4.4075000000000003e+01;

        float imageSizeThresh=0.01;

        // These parameters can vary according to the setup
        float max_depth = 400.0; //maximum distance the setup can measure (in cm)
        float min_depth = 50.0; //minimum distance the setup can measure (in cm)
        float depth_thresh = 150.0; // Threshold for SAFE distance (in cm)

        // Creating an object of StereoBM algorithm
        cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
        

        //Initialize variables to store the maps for stereo rectification
        cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
        cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;


    depthDetection(int leftCamId,int rightCamId){
        cameraLeft.open(leftCamId,cv::CAP_DSHOW);
        cameraRight.open(rightCamId,cv::CAP_DSHOW);

        updateStereo();
    }
    void updateStereo(){
        stereo->setNumDisparities(numDisparities);
        stereo->setBlockSize(blockSize);
        stereo->setPreFilterType(preFilterType);
        stereo->setPreFilterSize(preFilterSize);
        stereo->setPreFilterCap(preFilterCap);
        stereo->setTextureThreshold(textureThreshold);
        stereo->setUniquenessRatio(uniquenessRatio);
        stereo->setSpeckleRange(speckleRange);
        stereo->setSpeckleWindowSize(speckleWindowSize);
        stereo->setDisp12MaxDiff(disp12MaxDiff);
        stereo->setMinDisparity(minDisparity);
    }

    std::vector<std::vector<cv::Point>> getContours(cv::Mat depth_map){
        cv::Mat imgL;
        cv::Mat imgR;
        cv::Mat imgL_gray;
        cv::Mat imgR_gray;
        cv::Mat disp, disparity;
        cv::Mat output_canvas;

        cv::Mat mask;
        cv::Mat mean;
        cv::Mat stddev;
        cv::Mat mask2;

        // Mask to segment regions with depth less than safe distance
        cv::inRange(depth_map, 10, depth_thresh, mask);
        double s = (cv::sum(mask)[0])/255.0;
        double img_area = double(mask.rows * mask.cols);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<std::vector<cv::Point>> relevantContours;
        std::vector<cv::Vec4i> hierarchy;

        // Check if a significantly large obstacle is present and filter out smaller noisy regions
        if (s >imageSizeThresh*img_area){
            // finding conoturs in the generated mask
            cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

            for(int i=0;i<contours.size();i++){
                std::vector<cv::Point> cnt=contours[i];
                // Check if detected contour is significantly large (to avoid multiple tiny regions)
                double cnt_area = fabs(cv::contourArea(cv::Mat(cnt)));
                if (cnt_area >imageSizeThresh*img_area)
                {
                    relevantContours.push_back(cnt);
                }
            }
        }
        return relevantContours;
    }
        
    cv::Mat calcDepthMap(cv::Mat frameL,cv::Mat FrameR){
        cv::Mat imgL_gray;
        cv::Mat imgR_gray;
        cv::Mat disp, disparity;
        // Converting images to grayscale
        cv::cvtColor(frameL, imgL_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(FrameR, imgR_gray, cv::COLOR_BGR2GRAY);

        // Initialize matrix for rectified stereo images
        cv::Mat Left_nice;
        cv::Mat Right_nice;
        // Applying stereo image rectification on the left image
        cv::remap(imgL_gray,
                Left_nice,
                Left_Stereo_Map1,
                Left_Stereo_Map2,
                cv::INTER_LANCZOS4,
                cv::BORDER_CONSTANT,
                0);

        // Applying stereo image rectification on the right image
        cv::remap(imgR_gray,
                Right_nice,
                Right_Stereo_Map1,
                Right_Stereo_Map2,
                cv::INTER_LANCZOS4,
                cv::BORDER_CONSTANT,
                0);

        // Calculating disparith using the StereoBM algorithm
        stereo->compute(Left_nice,Right_nice,disp);
        
        // NOTE: compute returns a 16bit signed single channel image,
            // CV_16S containing a disparity map scaled by 16. Hence it 
        // is essential to convert it to CV_16S and scale it down 16 times.

        // Converting disparity values to CV_32F from CV_16S
        disp.convertTo(disparity,CV_32F, 1.0);

        // Scaling down the disparity values and normalizing them
        disparity = (disparity/16.0f - (float)minDisparity)/((float)numDisparities);

        // Calculating disparity to depth map using the following equation
        // ||    depth = M * (1/disparity)   ||
        cv::Mat depth_map = (float)M/disparity;
        return depth_map;
    }   
    std::vector<cv::Rect> getBoundingBoxes(std::vector<std::vector<cv::Point>> contours){
        std::vector<cv::Rect> boundingBoxes;
        for(int i=0;i<contours.size();i++){
            std::vector<cv::Point> cnt=contours[i];
            cv::Rect box;
            // Finding the bounding rectangle for the largest contour
            box = cv::boundingRect(cnt);
            boundingBoxes.push_back(box);
        }
        return boundingBoxes;
    }
};

int main(){
  return 0;
}
