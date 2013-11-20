//
//  ProjectorCalibration.cpp
//  camera_projector_calibration_CV
//
//  Created by kikko on 18/11/13.
//
//

#include "CustomCalibration.h"

using namespace cv;

namespace ofxCv {

    #pragma mark - CameraCalibration

    void CameraCalibration::resetBoards(){
        objectPoints.clear();
        imagePoints.clear();
        boardRotations.clear();
        boardTranslations.clear();
    }
    
    void CameraCalibration::remove(int i){
        objectPoints.erase(objectPoints.begin() + i);
        imagePoints.erase(imagePoints.begin() + i);
        boardRotations.erase(boardRotations.begin() + i);
        boardTranslations.erase(boardTranslations.begin() + i);
    }

    void CameraCalibration::setupCandidateObjectPoints(){
        candidateObjectPts.clear();
        for(int i = 0; i < patternSize.height; i++) {
            for(int j = 0; j < patternSize.width; j++) {
                candidateObjectPts.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
            }
        }
    }

    void CameraCalibration::computeCandidateBoardPose(const vector<cv::Point2f> & imgPts, cv::Mat& boardRot, cv::Mat& boardTrans){
        cv::solvePnP(candidateObjectPts, imgPts,
                     distortedIntrinsics.getCameraMatrix(),
                     distCoeffs,
                     boardRot, boardTrans);
    }

    // some crazy stuff, no idea what's happening there. So thanks Alvaro & Niklas!
    bool CameraCalibration::backProject(const Mat& boardRot64,
                                        const Mat& boardTrans64,
                                        const vector<Point2f>& imgPt,
                                        vector<Point3f>& worldPt) {
        if( imgPt.size() == 0 ) {
            return false;
        }
        else
        {
            Mat imgPt_h = Mat::zeros(3, imgPt.size(), CV_32F);
            for( int h=0; h<imgPt.size(); ++h ) {
                imgPt_h.at<float>(0,h) = imgPt[h].x;
                imgPt_h.at<float>(1,h) = imgPt[h].y;
                imgPt_h.at<float>(2,h) = 1.0f;
            }
            Mat Kinv64 = getUndistortedIntrinsics().getCameraMatrix().inv();
            Mat Kinv,boardRot,boardTrans;
            Kinv64.convertTo(Kinv, CV_32F);
            boardRot64.convertTo(boardRot, CV_32F);
            boardTrans64.convertTo(boardTrans, CV_32F);
            
            // Transform all image points to world points in camera reference frame
            // and then into the plane reference frame
            Mat worldImgPt = Mat::zeros( 3, imgPt.size(), CV_32F );
            Mat rot3x3;
            Rodrigues(boardRot, rot3x3);
            
            Mat transPlaneToCam = rot3x3.inv()*boardTrans;
            
            for( int i=0; i<imgPt.size(); ++i ) {
                Mat col = imgPt_h.col(i);
                Mat worldPtcam = Kinv*col;
                Mat worldPtPlane = rot3x3.inv()*(worldPtcam);
                
                float scale = transPlaneToCam.at<float>(2)/worldPtPlane.at<float>(2);
                Mat worldPtPlaneReproject = scale*worldPtPlane-transPlaneToCam;
                
                Point3f pt;
                pt.x = worldPtPlaneReproject.at<float>(0);
                pt.y = worldPtPlaneReproject.at<float>(1);
                pt.z = 0;
                worldPt.push_back(pt);
            }
        }
        return true;
    }


    #pragma mark - ProjectorCalibration

    void ProjectorCalibration::setStaticCandidateImagePoints(){
        candidateImagePoints.clear();
        Point2f p;
        for(int i = 0; i < patternSize.height; i++) {
            for(int j = 0; j < patternSize.width; j++) {
                p.x = patternPosition.x + float(((2 * j) + (i % 2)) * squareSize);
                p.y = patternPosition.y + float(i * squareSize);
                candidateImagePoints.push_back(p);
            }
        }
    }
    
    void ProjectorCalibration::setCandidateImagePoints(vector<cv::Point2f> pts){
        candidateImagePoints = pts;
    }

    void ProjectorCalibration::resetBoards(){
        objectPoints.clear();
        imagePoints.clear();
        boardRotations.clear();
        boardTranslations.clear();
    }
    
    void ProjectorCalibration::remove(int i){
        objectPoints.erase(objectPoints.begin() + i);
        imagePoints.erase(imagePoints.begin() + i);
        boardRotations.erase(boardRotations.begin() + i);
        boardTranslations.erase(boardTranslations.begin() + i);
    }

    //--

    void ProjectorCalibration::setImagerSize(int width, int height) {
        imagerSize = cv::Size(width, height);
        distortedIntrinsics.setImageSize(imagerSize);
        undistortedIntrinsics.setImageSize(imagerSize);
        addedImageSize = imagerSize;
    }

    void ProjectorCalibration::setPatternPosition(float px, float py) {
        patternPosition = Point2f(px, py);
    }
}