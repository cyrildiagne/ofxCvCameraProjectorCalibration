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
    
    screenRect.set(0,0,1280,800);
    projectorRect.set(1280,0,1280,800);
    
    camProjCalib.setup(PROJECTOR_STATIC, projectorRect.width, projectorRect.height);
    
    setupDefaultParams();
    setupGui();
    
	lastTime = 0;
    bProjectorRefreshLock = true;
}

void ofApp::setupDefaultParams(){
    
    diffMinBetweenFrames.set("Difference min between frames", 4.0, 0, 10);
    timeMinBetweenCaptures.set("Time min between captures", 2.0, 0, 10);
}

void ofApp::setupGui(){
    
    gui.setDefaultWidth(300);
    gui.setDefaultBorderColor(ofColor(0,0,0));
    gui.setDefaultBackgroundColor(ofColor(0,0,0));
    gui.setDefaultFillColor(ofColor::royalBlue);
    
    gui.setup("", "settings.xml");
    gui.setBorderColor(ofColor(0,0,0,0));
    gui.setBackgroundColor(ofColor(0,0,0,0));
    gui.setPosition(640*1.5 + 10, 10);
    
    gui.add( currState.set("Current State", camProjCalib.getCurrentStateString()) );
    
    params.setName("Application");
    params.add( diffMinBetweenFrames );
    params.add( timeMinBetweenCaptures );
    gui.add(params);
    
    gui.add(camProjCalib.boardsParams);
    gui.add(camProjCalib.imageProcessingParams);
    
    gui.loadFromFile("settings.xml");
}

#pragma mark - Update

void ofApp::update(){
    
	cam.update();
    
	if(cam.isFrameNew()) {		
		Mat camMat = toCv(cam);
        
        switch (camProjCalib.getCurrentState()) {
            case CAMERA:
                if( !updateCamDiff(camMat) ){
                    return;
                }
                if(camProjCalib.calibrateCamera(camMat)){
                    lastTime = ofGetElapsedTimef();
                }
                break;
            case PROJECTOR_STATIC:
                if( !updateCamDiff(camMat) ){
                    return;
                }
                if(camProjCalib.calibrateProjector(camMat)){
                    lastTime = ofGetElapsedTimef();
                }
                break;
            case PROJECTOR_DYNAMIC:
                if(bProjectorRefreshLock){
                    if(camProjCalib.setDynamicProjectorImagePoints(camMat)){
                        if( !updateCamDiff(camMat) ){
                            return;
                        }
                        bProjectorRefreshLock = false;
                    }
                } else {
                    camProjCalib.calibrateProjector(camMat);
                    bProjectorRefreshLock = true;
                    lastTime = ofGetElapsedTimef();
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
    drawLastCameraImagePoints();
    
    currState = camProjCalib.getCurrentStateString();
    
    gui.draw();
    
    ofDrawBitmapStringHighlight("Movement: "+ofToString(diffMean), 10, 20, ofxCv::cyanPrint);
    
    drawReprojErrors("Camera", camProjCalib.getCalibrationCamera(), 40);
    
    switch (camProjCalib.getCurrentState()) {
        case CAMERA:
            drawReprojLog(camProjCalib.getCalibrationCamera(), 60);
            break;
        case PROJECTOR_STATIC:
        case PROJECTOR_DYNAMIC:
            drawReprojErrors("Projector", camProjCalib.getCalibrationProjector(), 60);
            drawReprojLog(camProjCalib.getCalibrationProjector(), 80);
            if(ofxCv::getAllocated(camProjCalib.getProcessedImg())){
                ofxCv::drawMat(camProjCalib.getProcessedImg(), 640, 0, 320, 240);
            }
            drawProjectorPattern();
            break;
            
        default:
            break;
    }
    
    ofDrawBitmapString(camProjCalib.getLog(20), 10, cam.height+20);
}

void ofApp::drawReprojErrors(string name, const ofxCv::Calibration & calib, int y){
    
    string buff;
    buff = name + " Reproj. Error: " + ofToString(calib.getReprojectionError(), 2);
    buff += " from " + ofToString(calib.size());
    ofDrawBitmapStringHighlight(buff, 10, y, ofxCv::magentaPrint);
}

void ofApp::drawReprojLog(const ofxCv::Calibration & calib, int y) {
    
    string buff;
	for(int i = 0; i < calib.size(); i++) {
        buff = ofToString(i) + ": " + ofToString(calib.getReprojectionError(i));
		ofDrawBitmapStringHighlight(buff, 10, y + 16 * i, ofxCv::magentaPrint);
	}
}

void ofApp::drawLastCameraImagePoints(){
    
    const auto & imagePoints = camProjCalib.getCalibrationCamera().imagePoints;
    
    if(imagePoints.size()<=0) return;
    
    ofPushStyle(); ofSetColor(ofColor::blue);
    for(const auto & p : imagePoints.back()) {
        ofCircle(ofxCv::toOf(p), 3);
    }
    ofPopStyle();
}

void ofApp::drawProjectorPattern(){
    
    ofRectangle vp = ofGetCurrentViewport();
    ofViewport(projectorRect);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0,projectorRect.width, projectorRect.height, 0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    {
        ofSetColor(ofColor::white);
        for(const auto & p : camProjCalib.getCalibrationProjector().getCandidateImagePoints()) {
            ofCircle(p.x, p.y, 4);
        }
    }
    ofViewport(vp);
}

#pragma mark - Inputs

void ofApp::keyPressed(int key){
    
    
}
