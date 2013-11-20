//
//  StereoCalibration.h
//  camera_projector_calibration_CV
//
//  Created by kikko on 18/11/13.
//
//

#pragma once

#include "ofMain.h"
#include "ofxCv.h"

namespace ofxCv {

    
    class CameraCalibration : public Calibration {
    public:

        void setupCandidateObjectPoints();
        
        void resetBoards();
        void remove(int index);
        
        void computeCandidateBoardPose(const vector<cv::Point2f> & imgPts, cv::Mat& boardRot, cv::Mat& boardTrans);
        
        bool backProject(const cv::Mat& boardRot64, const cv::Mat& boardTrans64,
                         const vector<cv::Point2f>& imgPt,
                         vector<cv::Point3f>& worldPt);
        
        vector<cv::Mat> & getBoardRotations() { return boardRotations; }
        vector<cv::Mat> & getBoardTranslations() { return boardTranslations; }
        
        vector<cv::Point3f> getCandidateObjectPoints() { return candidateObjectPts; }
        vector<vector<cv::Point3f> > & getObjectPoints() { return objectPoints; }
        
    private:
        
        vector<cv::Point3f> candidateObjectPts;
    };



    class ProjectorCalibration : public Calibration {
        
    public:
        
        void setStaticCandidateImagePoints();
        void setCandidateImagePoints(vector<cv::Point2f> pts);
        
        void resetBoards();
        void remove(int index);
        
        void setImagerSize(int width, int height);
        void setPatternPosition(float x, float y);
        
        cv::Size getPatternSize() { return patternSize; }
        
        vector<cv::Mat> & getBoardRotations() { return boardRotations; }
        vector<cv::Mat> & getBoardTranslations() { return boardTranslations; }
        
        const vector<cv::Point2f> & getCandidateImagePoints() const { return candidateImagePoints; }
        vector<vector<cv::Point3f> > & getObjectPoints() { return objectPoints; }
        
    protected:
        
        void processImageForCircleDetection(cv::Mat inImg, cv::Mat outImg);
        
        ofxCv::Calibration * pair;
        cv::Size imagerSize;
        cv::Point2f patternPosition;
        
    private:
        
        vector<cv::Point2f> candidateImagePoints;
    };
}
