#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxCvFeaturesTrackerThreaded.h"
#include "ofxCvCameraProjectorCalibration.h"

#define SCREEN_WIDTH 1280
#define SCREEN_HEIGHT 800

#define PROJECTOR_WIDTH 1280
#define PROJECTOR_HEIGHT 800

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
    void exit();
    void keyPressed(int key);
	
    // draw with OpenCV projection method (slower but useful for debug)
    void drawUsingCV();
    
    // draw using OpenGL projection directly
    void drawUsingGL();
    
private:
    
    void setupCamProj();
    void setupTracker();
    
	ofVideoGrabber cam;
    ofImage trackedImg;
    
    ofxCv::FeaturesTrackerThreaded tracker;
    ofxCv::CameraProjectorCalibration camproj;
    cv::Mat rotObjToCam, transObjToCam;
    cv::Mat rotObjToProj, transObjToProj;
    
    bool bDrawDebug;
    bool bDrawWithCV;
    
    string getRTMatInfos(const cv::Mat rvecs, const cv::Mat tvecs);
    
    // since we're using the threaded tracker, "found" status can change between update() and draw() calls
    // so we store its value here
    bool bFound;
};
