#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxCv.h"
#include "ofxCvCameraProjectorCalibration.h"

enum CalibState {
    CAMERA,
    PROJECTOR_STATIC,
    PROJECTOR_DYNAMIC
};

class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
    
    ofxCv::CameraProjectorCalibration camProjCalib;
    
    ofVideoGrabber cam;
    
    void setState(CalibState state);
    string getCurrentStateString();
    
    bool calibrateCamera(cv::Mat img);
    bool calibrateProjector(cv::Mat img);
    
private:
    
    CalibState currState;
    
    // circles detection
    
    void processImageForCircleDetection(cv::Mat camMat);
    cv::Mat processedImg;
    
    // board holding movement
    
    ofPixels previous;
	ofPixels diff;
	float diffMean;
	float lastTime;
    bool updateCamDiff(cv::Mat camMat);

    // lock to give enough time for the projection to be seen by the camera
    
    bool bProjectorRefreshLock;
    
    // screen & projector configuration
    
    ofRectangle projectorRect;
    ofRectangle screenRect;
    
    // gui
    
    ofxPanel gui;
    void setupGui();
    
    // draw
    
    void drawReprojErrors(string name, const ofxCv::Calibration & calib, int y);
    void drawReprojLog(const ofxCv::Calibration & calib, int y);
    void drawLastCameraImagePoints();
    void drawProjectorPattern();
    
    // log
    
    stringstream log_;
    stringstream & log() {
        return log_;
    }
    bool bLog;
    string getLog(int numLines=15);
    
    // params
    
    void setupDefaultParams();
    ofParameterGroup appParams;
    ofParameter<float> diffMinBetweenFrames;
    ofParameter<float> timeMinBetweenCaptures;
    ofParameter<string> currStateString;
    
    ofParameterGroup boardsParams;
    ofParameter<int> numBoardsFinalCamera;
    ofParameter<int> numBoardsFinalProjector;
    ofParameter<int> numBoardsBeforeCleaning;
    ofParameter<int> numBoardsBeforeDynamicProjection;
    ofParameter<float> maxReprojErrorCamera;
    ofParameter<float> maxReprojErrorProjector;
    
    ofParameterGroup imageProcessingParams;
    ofParameter<int> circleDetectionThreshold;
};
