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
#include "CustomCalibration.h"

namespace ofxCv {
    
    enum CalibState {
        CAMERA,
        PROJECTOR_STATIC,
        PROJECTOR_DYNAMIC
    };


    class CameraProjectorCalibration {
        
    public:
        
        void setup(CalibState initState, int projectorWidth, int projectorHeight);
        void update(cv::Mat camMat);
        
        void saveExtrinsics(string filename, bool absolute = false) const;
        void loadExtrinsics(string filename, bool absolute = false);
        
        bool calibrateCamera(cv::Mat img);
        bool calibrateProjector(cv::Mat img);
        
        bool setDynamicProjectorImagePoints(cv::Mat img);
        
        // getters / setters
        
        void setState(CalibState state);
        CalibState getCurrentState() { return currState; }
        string getCurrentStateString();
        
        string getLog(int numLines=15);
        
        const CameraCalibration & getCalibrationCamera() { return calibrationCamera; }
        const ProjectorCalibration & getCalibrationProjector() { return calibrationProjector; }
        
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
        
        
    protected:
        
        void setupDefaultParams();
        
        void stereoCalibrate();
        void cleanStereo();
        
        stringstream log_;
        stringstream & log() {
            return log_;
        }
        bool bLog;
        
        template <class T>
        friend stringstream& operator<<(stringstream& os, const T& item);
        
        CalibState currState;
        
        CameraCalibration calibrationCamera;
        ProjectorCalibration calibrationProjector;
        
        cv::Mat rotCamToProj;
        cv::Mat transCamToProj;
        
        // --
        
        cv::Mat processedImg;
    };
}