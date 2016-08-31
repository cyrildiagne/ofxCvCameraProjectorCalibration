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
    
    camProjCalib.setup(projectorRect.width, projectorRect.height);
    
    setupDefaultParams();
    setupGui();
    
	lastTime = 0;
    bProjectorRefreshLock = true;
    
    bLog = true;
    
    setState(PROJECTOR_STATIC);
    
    log() << "Calibration started at step : " << getCurrentStateString() << endl;
}

void ofApp::setupDefaultParams(){
    
    appParams.setName("Application");
    appParams.add( diffMinBetweenFrames.set("Difference min between frames", 4.0, 0, 10) );
    appParams.add( timeMinBetweenCaptures.set("Time min between captures", 2.0, 0, 10) );
    
    boardsParams.setName("Boards Params");
    boardsParams.add( numBoardsFinalCamera.set("Num boards Camera", 20, 10, 30) );
    boardsParams.add( numBoardsFinalProjector.set("Num boards Projector", 12, 6, 15) );
    boardsParams.add( numBoardsBeforeCleaning.set("Num boards before cleaning", 8, 5, 10) );
    boardsParams.add( numBoardsBeforeDynamicProjection.set("Num boards before dynamic proj", 5, 3, 10) );
    boardsParams.add( maxReprojErrorCamera.set("Max reproj error Camera", 0.2, 0.1, 0.5) );
    boardsParams.add( maxReprojErrorProjector.set("Max reproj error Projector", 0.6, 0.1, 1.0) );
    
    imageProcessingParams.setName("Processing Params");
    imageProcessingParams.add( circleDetectionThreshold.set("Circle image threshold", 220, 150, 255) );
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
    
    gui.add( currStateString.set("Current State", getCurrentStateString()) );
    
    gui.add(appParams);
    gui.add(boardsParams);
    gui.add(imageProcessingParams);
    
    gui.loadFromFile("settings.xml");
}

#pragma mark - App states

void ofApp::setState(CalibState state){
    
    CameraCalibration & calibrationCamera = camProjCalib.getCalibrationCamera();
    ProjectorCalibration & calibrationProjector = camProjCalib.getCalibrationProjector();
    
    switch (state) {
        case CAMERA:
            camProjCalib.resetBoards();
            break;
        case PROJECTOR_STATIC:
            calibrationCamera.load("calibrationCamera.yml");
            camProjCalib.resetBoards();
            calibrationCamera.setupCandidateObjectPoints();
            calibrationProjector.setStaticCandidateImagePoints();
            break;
        case PROJECTOR_DYNAMIC:
            break;
        default:
            break;
    }
    currState = state;
    
    currStateString = getCurrentStateString();
    log() << "Set state : " << getCurrentStateString() << endl;
}

string ofApp::getCurrentStateString(){
    
    string name;
    switch (currState) {
        case CAMERA:            name = "CAMERA"; break;
        case PROJECTOR_STATIC:  name = "PROJECTOR_STATIC"; break;
        case PROJECTOR_DYNAMIC: name = "PROJECTOR_DYNAMIC"; break;
        default: break;
    }
    return name;
}

#pragma mark - Update

void ofApp::update(){
    
	cam.update();
    
	if(cam.isFrameNew()) {		
		Mat camMat = toCv(cam);
        
        switch (currState) {
                
            case CAMERA:
                if( !updateCamDiff(camMat) ) return;
                
                if( calibrateCamera(camMat) ){
                    lastTime = ofGetElapsedTimef();
                }
                break;
                
            case PROJECTOR_STATIC:
                if( !updateCamDiff(camMat) ) return;
                
                if( calibrateProjector(camMat) ){
                    lastTime = ofGetElapsedTimef();
                }
                break;
                
            case PROJECTOR_DYNAMIC:
                if(bProjectorRefreshLock){
                    if( camProjCalib.setDynamicProjectorImagePoints(camMat) ){
                        if( !updateCamDiff(camMat) ) return;
                        
                        bProjectorRefreshLock = false;
                    }
                }
                else {
                    if( calibrateProjector(camMat) ) {
                        lastTime = ofGetElapsedTimef();
                    }
                    bProjectorRefreshLock = true;
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

void ofApp::processImageForCircleDetection(cv::Mat img){
    
    if(img.type() != CV_8UC1) {
        cvtColor(img, processedImg, CV_RGB2GRAY);
    } else {
        processedImg = img;
    }
    cv::threshold(processedImg, processedImg, circleDetectionThreshold, 255, cv::THRESH_BINARY_INV);
}

bool ofApp::calibrateCamera(cv::Mat img){
    
    CameraCalibration & calibrationCamera = camProjCalib.getCalibrationCamera();
    
    bool bFound = calibrationCamera.add(img);
    if(bFound){
        
        log() << "Found board!" << endl;
        
        calibrationCamera.calibrate();
        
        if(calibrationCamera.size() >= numBoardsBeforeCleaning) {
            
            log() << "Cleaning" << endl;
            
            calibrationCamera.clean(maxReprojErrorCamera);
            
            if(calibrationCamera.getReprojectionError(calibrationCamera.size()-1) > maxReprojErrorCamera) {
                log() << "Board found, but reproj. error is too high, skipping" << endl;
                return false;
            }
        }
        
        if (calibrationCamera.size()>=numBoardsFinalCamera) {
            
            calibrationCamera.save("calibrationCamera.yml");
            
            log() << "Camera calibration finished & saved to calibrationCamera.yml" << endl;
            
            setState(PROJECTOR_STATIC);
        }
    } else log() << "Could not find board" << endl;
    
    return bFound;
}

bool ofApp::calibrateProjector(cv::Mat img){
    
    CameraCalibration & calibrationCamera = camProjCalib.getCalibrationCamera();
    ProjectorCalibration & calibrationProjector = camProjCalib.getCalibrationProjector();
    
    processImageForCircleDetection(img);
    
    if(camProjCalib.addProjected(img, processedImg)){
        
        log() << "Calibrating projector" << endl;
        
        calibrationProjector.calibrate();
        
        if(calibrationProjector.size() >= numBoardsBeforeCleaning) {
            
            log() << "Cleaning" << endl;
            
            int numBoardRemoved = camProjCalib.cleanStereo(maxReprojErrorProjector);
            
            log() << numBoardRemoved << " boards removed";
            
            if(currState == PROJECTOR_DYNAMIC && calibrationProjector.size() < numBoardsBeforeDynamicProjection) {
                log() << "Too many boards removed, restarting to PROJECTOR_STATIC" << endl;
                setState(PROJECTOR_STATIC);
                return false;
            }
        }
        
        log() << "Performing stereo-calibration" << endl;
        
        camProjCalib.stereoCalibrate();
        
        log() << "Done" << endl;
        
        if(currState == PROJECTOR_STATIC) {
            
            if( calibrationProjector.size() < numBoardsBeforeDynamicProjection) {
                log() << numBoardsBeforeDynamicProjection - calibrationProjector.size() << " boards to go before dynamic projection" << endl;
            } else {
                setState(PROJECTOR_DYNAMIC);
            }
            
        } else {
            
            if( calibrationProjector.size() < numBoardsFinalProjector) {
                log() << numBoardsFinalProjector - calibrationProjector.size() << " boards to go to completion" << endl;
            } else {
                calibrationProjector.save("calibrationProjector.yml");
                log() << "Projector calibration finished & saved to calibrationProjector.yml" << endl;
                
                camProjCalib.saveExtrinsics("CameraProjectorExtrinsics.yml");
                log() << "Stereo Calibration finished & saved to CameraProjectorExtrinsics.yml" << endl;
                
                log() << "Congrats, you made it ;)" << endl;
            }
        }
        return true;
    }
    return false;
}


#pragma mark - Draw

void ofApp::draw(){
    
    cam.draw(0, 0);
    drawLastCameraImagePoints();
    
    gui.draw();
    
    ofDrawBitmapStringHighlight("Movement: "+ofToString(diffMean), 10, 20, ofxCv::cyanPrint);
    
    drawReprojErrors("Camera", camProjCalib.getCalibrationCamera(), 40);
    
    switch (currState) {
        case CAMERA:
            drawReprojLog(camProjCalib.getCalibrationCamera(), 60);
            break;
        case PROJECTOR_STATIC:
        case PROJECTOR_DYNAMIC:
            drawReprojErrors("Projector", camProjCalib.getCalibrationProjector(), 60);
            drawReprojLog(camProjCalib.getCalibrationProjector(), 80);
            if(ofxCv::getAllocated(processedImg)){
                ofxCv::drawMat(processedImg, 640, 0, 320, 240);
            }
            drawProjectorPattern();
            break;
            
        default:
            break;
    }
    
    ofDrawBitmapString(getLog(20), 10, cam.height+20);
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
    glOrtho(0, projectorRect.width, projectorRect.height, 0, -1, 1);
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

#pragma mark - Log

string ofApp::getLog(int numLines){
    
    vector<string> elems;
    stringstream ss(log_.str());
    string buff;
    while (getline(ss, buff)) elems.push_back(buff);
    numLines = fmin(elems.size(), numLines);
    buff = "";
    if(elems.size()>0) {
        for (int i=elems.size()-1; i>elems.size()-numLines; i--) {
            buff = elems[i] + "\n" + buff;
        }
    }
    return buff;
}
