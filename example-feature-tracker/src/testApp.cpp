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
    
    rotObjToProj = Mat::zeros(3, 1, CV_64F);
    transObjToProj = Mat::zeros(3, 1, CV_64F);
    
    bDrawDebug = true;
    bDrawWithCV = false;
    bFound = false;
}

void testApp::setupCamProj(){
    rotObjToCam = Mat::zeros(3, 1, CV_64F);
    transObjToCam = Mat::zeros(3, 1, CV_64F);
    camproj.load("calibrationCamera.yml", "calibrationProjector.yml", "CameraProjectorExtrinsics.yml");
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
            for (int i=0; i<3; i++) { *tvec.ptr<double>(i) *= 9.3; } // TODO : find this scaling value in configuration files
            *tvec.ptr<double>(1) += 1;
            
            // smooth results
            rotObjToCam += (rvec-rotObjToCam) * 0.3;
            transObjToCam += (tvec-transObjToCam) * 0.3;
            
            // update the object-to-projector
            //camproj.update(rotObjToCam, transObjToCam);
            
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
            ofDrawBitmapString(getRTMatInfos(rotObjToCam, transObjToCam), 20, 500);
        }
        
        if(bDrawWithCV) drawUsingCV();
        else drawUsingGL();
	}
}

void testApp::drawUsingCV(){
    
    // set some input points
    float w = 12.5 / 2;
    int h = 19 / 2;
    vector<Point3f> inPts;
    inPts.push_back(Point3f(-w, -h, 0));
    inPts.push_back(Point3f(w, -h, 0));
    inPts.push_back(Point3f(w, h, 0));
    inPts.push_back(Point3f(-w, h, 0));
    
    // get videoproj's projection of inputPts
    vector<cv::Point2f> outPts = camproj.getProjected(inPts, rotObjToCam, transObjToCam);
    
    // project the inputPts over the object
    ofPushMatrix();
    ofTranslate(1280, 0);
    ofSetColor(ofColor::red);
    for (int i=0; i<outPts.size(); i++){
        int next = (i+1) % outPts.size();
        ofLine(outPts[i].x, outPts[i].y, outPts[next].x, outPts[next].y);
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
    
    ofPushMatrix();
    
    // Set perspective matrix using the projector intrinsics
    ofPoint projectorOffset = ofPoint(1280, 0);
    camproj.getCalibrationProjector().getDistortedIntrinsics().loadProjectionMatrix(1, 10000, projectorOffset);
    
    // apply model to projector transformations
    cv::composeRT(rotObjToCam,  transObjToCam,
                  camproj.getCamToProjRotation(), camproj.getCamToProjTranslation(),
                  rotObjToProj, transObjToProj);
    applyMatrix(makeMatrix(rotObjToProj, transObjToProj));
    
    ofScale(0.04, 0.04, 0); // todo : find this value in configuration files
    
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
    
    ofPopMatrix();
    
    // TODO : restore GL matrices
}

string testApp::getRTMatInfos(const cv::Mat rvecs, const cv::Mat tvecs){
    cv::Mat rot3x3 = cv::Mat::zeros(3, 3, CV_32F);
    Rodrigues(rvecs, rot3x3);
    stringstream imgRTs;
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++)  imgRTs << ofToString(rot3x3.at<double>(i,j),1) << "\t";
        imgRTs << ofToString(tvecs.at<double>(i),1) << endl;
    }
    return imgRTs.str();
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
