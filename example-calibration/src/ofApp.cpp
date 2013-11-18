#include "ofApp.h"
#include "ofAppGLFWWindow.h"

using namespace ofxCv;
using namespace cv;

#pragma mark Setup

void ofApp::setup(){
    
    ((ofAppGLFWWindow*)ofGetWindowPtr())->setMultiDisplayFullscreen(true);
    ofSetFullscreen(true);
    ofSetVerticalSync(true);
    ofEnableAlphaBlending();
    ofEnableSmoothing();
    ofSetCircleResolution(30);
    ofBackground(0,0,0);
    
	cam.initGrabber(640, 480);
	imitate(previous, cam);
	imitate(diff, cam);
    
    setupDefaultParams();
    setupGui();
    
    camProjCalib.setup(PROJECTOR_STATIC); //CAMERA
    
	lastTime = 0;
    bProjectRefreshLock = true;
}

void ofApp::setupDefaultParams(){
    
    diffMinBetweenFrames.set("Difference min. between frames", 4.0, 0, 10);
    timeMinBetweenCaptures.set("Time min. between captures", 2.0, 0, 10);
}

void ofApp::setupGui(){
    
    gui.setDefaultWidth(270);
    gui.setDefaultBorderColor(ofColor(0,0,0));
    gui.setDefaultBackgroundColor(ofColor(0,0,0));
    gui.setDefaultFillColor(ofColor::royalBlue);
    
    gui.setup("CameraProjectorCalibration", "settings.xml");
    gui.setBorderColor(ofColor(0,0,0,0));
    gui.setBackgroundColor(ofColor(0,0,0,0));
    gui.setPosition(650, 10);
    
    gui.add( currState.set("Current State :", camProjCalib.getCurrentStateString()) );
    
    params.setName("Application");
    params.add( diffMinBetweenFrames );
    params.add( timeMinBetweenCaptures );
    gui.add(params);
    
    gui.loadFromFile("settings.xml");
}

#pragma mark - Update

void ofApp::update(){
    
	cam.update();
    
	if(cam.isFrameNew()) {		
		Mat camMat = toCv(cam);
        
        if( !updateCamDiff(camMat) ){
            return;
        }
        
        switch (camProjCalib.getCurrentState()) {
            case CAMERA:
                if(camProjCalib.addCameraBoard(camMat)){
                    lastTime = ofGetElapsedTimef();
                }
                break;
            case PROJECTOR_STATIC:
                if(camProjCalib.detectPrintedAndProjectedPatterns(camMat)){
                    lastTime = ofGetElapsedTimef();
                }
                break;
            case PROJECTOR_DYNAMIC:
                if(bProjectRefreshLock){
                    bProjectRefreshLock = false;
                } else {
                    bProjectRefreshLock = true;
                }
                break;
            default: break;
        }
    }
}

bool ofApp::updateCamDiff(cv::Mat camMat) {
    
    Mat prevMat = toCv(previous);
    Mat diffMat = toCv(diff);
    absdiff(prevMat, camMat, diffMat);
    diffMean = mean(Mat(mean(diffMat)))[0];
    camMat.copyTo(prevMat);
    
    float timeDiff = ofGetElapsedTimef() - lastTime;
    
    return timeMinBetweenCaptures < timeDiff && diffMinBetweenFrames > diffMean;
}


#pragma mark - Draw

void ofApp::draw(){
    
    cam.draw(0, 0);
    camProjCalib.drawLastCameraImagePoints();
    
    currState = camProjCalib.getCurrentStateString();
    
    gui.draw();
    
    ofDrawBitmapStringHighlight("Movement: "+ofToString(diffMean), 10, 20, ofxCv::cyanPrint);
    
    drawReprojErrors();
    
    switch (camProjCalib.getCurrentState()) {
        case CAMERA:
            drawCameraReprojLog();
            break;
        case PROJECTOR_STATIC:
            if(ofxCv::getAllocated(camProjCalib.getProcessedImg())){
                ofxCv::drawMat(camProjCalib.getProcessedImg(), 640, 240, 320, 240);
            }
            camProjCalib.drawProjectorStatic();
            break;
        case PROJECTOR_DYNAMIC:
            
            break;
            
        default:
            break;
    }
    
    
    ofDrawBitmapString(camProjCalib.getLog(), 10, cam.height+20);
}

void ofApp::drawReprojErrors(){
    
    string buff;
    const ofxCv::Calibration & camCalib = camProjCalib.getCalibrationCamera();
    buff = "Camera Reproj. Error: " + ofToString(camCalib.getReprojectionError(), 2);
    buff += " from " + ofToString(camCalib.size());
    ofDrawBitmapStringHighlight(buff, 10, 40, ofxCv::magentaPrint);
    
    const ofxCv::Calibration & projCalib = camProjCalib.getCalibrationProjector();
    buff = "Projector Reproj. Error: " + ofToString(projCalib.getReprojectionError(), 2);
    buff += " from " + ofToString(projCalib.size());
    ofDrawBitmapStringHighlight(buff, 10, 60, ofxCv::magentaPrint);
}

void ofApp::drawCameraReprojLog(){
    
    const ofxCv::Calibration & camCalib = camProjCalib.getCalibrationCamera();
    
    string buff;
	for(int i = 0; i < camCalib.size(); i++) {
        buff = ofToString(i) + ": " + ofToString(camCalib.getReprojectionError(i));
		ofDrawBitmapStringHighlight(buff, 10, 60 + 16 * i, ofxCv::magentaPrint);
	}
}


#pragma mark - Inputs

void ofApp::keyPressed(int key){
    
    
}
