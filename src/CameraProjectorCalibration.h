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
#include "StereoCalibration.h"


enum CalibState {
    CAMERA,
    PROJECTOR_STATIC,
    PROJECTOR_DYNAMIC
};


class CameraProjectorCalibration {
    
public:
    
    void setup(CalibState initState = CAMERA);
	void update(cv::Mat camMat);
    
    void drawLastCameraImagePoints();
    void drawProjectorStatic();
    
    bool addCameraBoard(cv::Mat img);
    void cleanCameraBoards();
    
    bool detectPrintedAndProjectedPatterns(cv::Mat img);
    
    void setState(CalibState state);
    CalibState getCurrentState() { return currState; }
    string getCurrentStateString();
    
    string getLog(int numLines=15);
    
    const ofxCv::Calibration & getCalibrationCamera() { return calibrationCamera; }
    const ofxCv::Calibration & getCalibrationProjector() { return calibrationProjector; }
    
    cv::Mat & getProcessedImg() { return processedImg; }
    
    // params
    ofParameterGroup boardsParams;
    ofParameter<int> numBoardsFinalCamera;
    ofParameter<int> numBoardsFinalProjector;
    ofParameter<int> numBoardsBeforeCleaning;
    ofParameter<int> numBoardsBeforeDynamicProjection;
    ofParameter<float> maxReprojErrorCamera;
    ofParameter<float> maxReprojErrorProjector;
    
    ofParameterGroup imageProcessingParams;
    ofParameter<int> circleDetectionThreshold;
    
private:
    
    void setupDefaultParams();
    
    stringstream log;
    bool bLog;
    
    CalibState currState;
    
    ofRectangle projectorRect;
    ofRectangle screenRect;
    
    CameraCalibration calibrationCamera;
    ProjectorCalibration calibrationProjector;
    
    // --
    cv::Mat processedImg;
};
