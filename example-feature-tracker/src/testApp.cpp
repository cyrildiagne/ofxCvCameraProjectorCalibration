#include "testApp.h"
#include "ofAppGLFWWindow.h"

using namespace ofxCv;

void testApp::setup() {
	
    ofBackground(0, 0, 0);
    ofSetVerticalSync(false);
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofSetFrameRate(31);
    
    cam.initGrabber(640, 480);
    
    setupCamProj();
    setupTracker();
    
    bDrawDebug = true;
    bDrawWithCV = false;
    bFound = false;
}

void testApp::setupCamProj(){
    rotObjToCam = Mat::zeros(3, 1, CV_64F);
    transObjToCam = Mat::zeros(3, 1, CV_64F);
    ofRectangle screenRect    = ofRectangle(0, 0, SCREEN_WIDTH, ofGetHeight());
    ofRectangle projectorRect = ofRectangle(SCREEN_WIDTH, 0, PROJECTOR_WIDTH, ofGetHeight());
    camproj.setup(screenRect, projectorRect);
}

void testApp::setupTracker(){
    trackedImg.loadImage("ville_crea.png");
    tracker.setup(camproj.getCalibrationCamera());
    tracker.add(trackedImg);
    tracker.startThread();
}

void testApp::exit() {
    ofLog() << "Exiting app";
    //tracker.waitForThread(true);
}

void testApp::update() {
    
    ofSetWindowTitle(ofToString(ofGetFrameRate(),1)+"fps");
    
	cam.update();
    
    if(cam.isFrameNew()) {
        
        tracker.update(cam);
        
        bFound = false;
        
        if(tracker.isFound()){
            
            // get object-to-camera transforms
            cv::Mat rvec, tvec;
            tracker.getRT(rvec, tvec);
            
            // hacks to adjust results
            for (int i=0; i<3; i++) { *tvec.ptr<double>(i) *= 26; } // TODO : find this scaling value in configuration file
            *tvec.ptr<double>(1) += 1.5; // TODO : remove this hack to get Y axis right
            
            // smooth results
            rotObjToCam += (rvec-rotObjToCam) * 0.3;
            transObjToCam += (tvec-transObjToCam) * 0.3;
            
            // update the object-to-projector
            camproj.update(rotObjToCam, transObjToCam);
            
            bFound = true;
        }
	}
}

void testApp::draw() {
    
    ofSetColor(255);
	cam.draw(0, 0);
    
    if(bDrawDebug){
        trackedImg.draw(cam.getWidth(),0);
        ofDrawBitmapString("num features : " + ofToString(tracker.getNumFeatures()), ofPoint(20, 20));
        ofDrawBitmapString("num matches : " + ofToString(tracker.getNumMatches()), ofPoint(20, 40));
        ofDrawBitmapString("update time : " + ofToString(tracker.getUpdateTime()) + "ms", ofPoint(20, 60));
    }
    
	if(bFound) {
        
        if(bDrawDebug){
            tracker.draw();
            ofLog() << "zob";
            ofDrawBitmapString(camproj.toString(), 20, 500);
        }
        
        if(bDrawWithCV) drawUsingCV();
        else drawUsingGL();
	}
}

void testApp::drawUsingCV(){
    
    // set some input points
    int w = 15;
    int h = 24;
    vector<ofPoint> inPts;
    inPts.push_back(ofPoint(-w, -h, 0));
    inPts.push_back(ofPoint(w, -h, 0));
    inPts.push_back(ofPoint(w, h, 0));
    inPts.push_back(ofPoint(-w, h, 0));
    
    // get videoproj's projection of inputPts
    vector<ofPoint> outPts = camproj.getProjected(inPts);
    
    // project the inputPts over the object
    ofPushMatrix();
    ofTranslate(1280, 0);
    ofSetColor(ofColor::red);
    for (int i=0; i<outPts.size(); i++){
        int next = (i+1) % outPts.size();
        ofLine(outPts[i], outPts[next]);
    }
    ofPopMatrix();
    
    // display some infos
    ofSetColor(ofColor::white);
    stringstream ptss;
    ptss << "Points projections : \n";
    for (int i=0; i<4; i++) {
        ptss << "Input : " << inPts[i] << "\t> Output :" << outPts[i] << "\n";
    }
    ofDrawBitmapString(ptss.str(), 350, 500);
}

void testApp::drawUsingGL(){
    
    camproj.beginGL();
    
    ofScale(0.115, 0.115, 0); // todo : find this value in configuration file
    ofTranslate(-15, 0); // todo : remove translation hack
    
    // project some square animation
    float w = trackedImg.width;
    float h = trackedImg.height;
    float millis = ofGetElapsedTimeMillis();
    int numRec = 10;
    ofNoFill();
    for (int i=0; i<numRec; i++) {
        float cw = w / numRec*i;
        float ch = h / numRec*i;
        int cc = (sin(millis/100 + i) * 0.5 + 0.5) * 255;
        ofSetColor(cc);
        ofRect(-cw*0.5, -ch*0.5, cw, ch);
    }
    
    camproj.endGL(); //!\ doesn't work yet
}

void testApp::keyPressed(int key){
    switch (key) {
        case 's':
            bDrawDebug = !bDrawDebug;
            break;
        case ' ':
            bDrawWithCV = !bDrawWithCV;
            break;
        case 'f':
            ofToggleFullscreen();
            break;
        default:
            break;
    }
}
