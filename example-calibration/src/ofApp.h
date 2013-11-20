#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxCv.h"
#include "CameraProjectorCalibration.h"


class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
    
    ofxCv::CameraProjectorCalibration camProjCalib;
    
private:
    
    ofVideoGrabber cam;
    
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
    
    // params
    void setupDefaultParams();
    ofParameterGroup params;
    ofParameter<float> diffMinBetweenFrames;
    ofParameter<float> timeMinBetweenCaptures;
    ofParameter<string> currState;
};
