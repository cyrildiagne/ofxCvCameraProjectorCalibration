//
//  StereoCalibration.h
//  camera_projector_calibration_CV
//
//  Created by kikko on 17/11/13.
//
//

#pragma once

#include "ofMain.h"
#include "ofxCv.h"

namespace ofxCv {
    
    class CalibrationPatched : public Calibration {
        
    public:
        void resetBoards() {
            objectPoints.clear();
            imagePoints.clear();
            boardRotations.clear();
            boardTranslations.clear();
        }
        void remove(int index){
            objectPoints.erase(objectPoints.begin() + index);
            imagePoints.erase(imagePoints.begin() + index);
            boardRotations.erase(boardRotations.begin() + index);
            boardTranslations.erase(boardTranslations.begin() + index);
        }
        cv::Size getPatternSize() { return patternSize; }
        vector<cv::Mat> & getBoardRotations() { return boardRotations; }
        vector<cv::Mat> & getBoardTranslations() { return boardTranslations; }
        vector<vector<cv::Point3f> > & getObjectPoints() { return objectPoints; }
    };
    
#pragma mark - CameraCalibration
    
    class CameraCalibration : public CalibrationPatched {
        
    public:
        void computeCandidateBoardPose(const vector<cv::Point2f> & imgPts, cv::Mat& boardRot, cv::Mat& boardTrans);
        bool backProject(const cv::Mat& boardRot64, const cv::Mat& boardTrans64,
                         const vector<cv::Point2f>& imgPt,
                         vector<cv::Point3f>& worldPt);
        void setupCandidateObjectPoints();
        vector<cv::Point3f> getCandidateObjectPoints() { return candidateObjectPts; }
        
    private:
        vector<cv::Point3f> candidateObjectPts;
    };
    
#pragma mark - ProjectorCalibration
    
    class ProjectorCalibration : public CalibrationPatched {
        
    public:
        void setImagerSize(int width, int height);
        void setPatternPosition(float x, float y);
        void setStaticCandidateImagePoints();
        void setCandidateImagePoints(vector<cv::Point2f> pts);
        const vector<cv::Point2f> & getCandidateImagePoints() const { return candidateImagePoints; }
        
    protected:
        ofxCv::Calibration * pair;
        cv::Size imagerSize;
        cv::Point2f patternPosition;
        
    private:
        vector<cv::Point2f> candidateImagePoints;
    };
    
#pragma mark - CameraProjectorCalibration
    
    class CameraProjectorCalibration {
        
    public:
        
        void load(string cameraConfig = "calibrationCamera.yml",
                  string projectorConfig  = "calibrationProjector.yml",
                  string extrinsicsConfig = "CameraProjectorExtrinsics.yml");
        void setup(int projectorWidth, int projectorHeight);
        void update(cv::Mat camMat);
        
        void saveExtrinsics(string filename, bool absolute = false) const;
        void loadExtrinsics(string filename, bool absolute = false);
        
        bool addProjected(cv::Mat img, cv::Mat processedImg);
        
        bool setDynamicProjectorImagePoints(cv::Mat img);
        void stereoCalibrate();
        void resetBoards();
        int cleanStereo(float maxReproj);
        
        vector<Point2f> getProjected(const vector<Point3f> & ptsInWorld,
                                     const cv::Mat & rotObjToCam = Mat::zeros(3, 1, CV_64F),
                                     const cv::Mat & transObjToCam = Mat::zeros(3, 1, CV_64F));
        
        CameraCalibration & getCalibrationCamera() { return calibrationCamera; }
        ProjectorCalibration & getCalibrationProjector() { return calibrationProjector; }
        
        const cv::Mat & getCamToProjRotation() { return rotCamToProj; }
        const cv::Mat & getCamToProjTranslation() { return transCamToProj; }
        
    protected:
        
        CameraCalibration calibrationCamera;
        ProjectorCalibration calibrationProjector;
        
        cv::Mat rotCamToProj;
        cv::Mat transCamToProj;
    };
}