//
//  CameraProjectorCalibration.cpp
//  camera_projector_calibration_CV
//
//  Created by kikko on 17/11/13.
//
//

#include "CameraProjectorCalibration.h"


void CameraProjectorCalibration::setup(CalibState initState){
    
    screenRect.set(0,0,1280,800);
    projectorRect.set(1280,0,1280,800);
    
    setupDefaultParams();
    
    calibrationCamera.setPatternSize(8, 5);
    calibrationCamera.setSquareSize(1.25);
    calibrationCamera.setPatternType(ofxCv::CHESSBOARD);
    
    calibrationProjector.setImagerSize(1280, 800);
    calibrationProjector.setPatternSize(4, 5);
    calibrationProjector.setPatternPosition(400, 250);
    calibrationProjector.setSquareSize(40);
    calibrationProjector.setPatternType(ofxCv::ASYMMETRIC_CIRCLES_GRID);
    
    bLog = true;
    
    setState(initState);
    
    if(bLog) log << "Calibration started at step : " << getCurrentStateString() << endl;
}

void CameraProjectorCalibration::setupDefaultParams(){
    
    boardsParams.setName("Boards Params");
    boardsParams.add( numBoardsFinalCamera.set("Num boards Camera", 20, 10, 30) );
    boardsParams.add( numBoardsFinalProjector.set("Num boards Projector", 12, 6, 15) );
    boardsParams.add( numBoardsBeforeCleaning.set("Num boards before cleaning", 8, 5, 10) );
    boardsParams.add( numBoardsBeforeDynamicProjection.set("Num boards before dynamic proj", 5, 3, 10) );
    boardsParams.add( maxReprojErrorCamera.set("Max reproj error Camera", 0.2, 0.1, 0.5) );
    boardsParams.add( maxReprojErrorProjector.set("Max reproj error Projector", 0.45, 0.1, 1.0) );
    
    imageProcessingParams.setName("Processing Params");
    imageProcessingParams.add( circleDetectionThreshold.set("Circle image threshold", 220, 150, 255) );
}

#pragma mark - camera

bool CameraProjectorCalibration::addCameraBoard(cv::Mat img){
    
    bool bFound = calibrationCamera.add(img);
    if(bFound){
        
        if(bLog) log << "Found board!" << endl;
        
        calibrationCamera.calibrate();
        
        if(calibrationCamera.size() >= numBoardsBeforeCleaning) {
        
            if(bLog) log << "Cleaning" << endl;
            
            calibrationCamera.clean(maxReprojErrorCamera);
            
            if(calibrationCamera.getReprojectionError(calibrationCamera.size()-1) > maxReprojErrorCamera) {
                if(bLog) log << "Board found, but reproj. error is too high, skipping" << endl;
                return false;
            }
        }
        
        if (calibrationCamera.size()>=numBoardsFinalCamera) {
        
            if(bLog) log << "Camera calibration finished & saved to calibrationCamera.yml" << endl;
            
            calibrationCamera.save("calibrationCamera.yml");
            
            setState(PROJECTOR_STATIC);
            
            if(bLog) log << "switching mode to PROJECTOR_STATIC" << endl;
        }
    } else if(bLog) log << "Could not find board" << endl;
    
    return bFound;
}

bool CameraProjectorCalibration::detectPrintedAndProjectedPatterns(cv::Mat img){
    
    vector<cv::Point2f> chessImgPts;
    bool bPrintedPatternFound = calibrationCamera.findBoard(img, chessImgPts, true);
    
    if(bPrintedPatternFound) {
        
        if(img.type() != CV_8UC1) {
            cvtColor(img, processedImg, CV_RGB2GRAY);
        } else {
            processedImg = img;
        }
        cv::threshold(processedImg, processedImg, circleDetectionThreshold, 255, cv::THRESH_BINARY_INV);
        
        vector<cv::Point2f> circlesImgPts;
        bool bProjectedPatternFound = cv::findCirclesGrid(processedImg, calibrationProjector.getPatternSize(), circlesImgPts, cv::CALIB_CB_ASYMMETRIC_GRID);
        
        if(bProjectedPatternFound){
            
            if(bLog) log << "Found projected circles! Adding points.." << endl;
            
            vector<cv::Point3f> circlesObjectPts;
            cv::Mat boardRot;
            cv::Mat boardTrans;
            calibrationCamera.computeCandidateBoardPose(chessImgPts, boardRot, boardTrans);
            calibrationCamera.backProject(boardRot, boardTrans, circlesImgPts, circlesObjectPts);
            
            calibrationCamera.imagePoints.push_back(chessImgPts);
            calibrationCamera.getObjectPoints().push_back(calibrationCamera.getCandidateObjectPoints());
            calibrationCamera.getBoardRotations().push_back(boardRot);
            calibrationCamera.getBoardTranslations().push_back(boardTrans);
            
            calibrationProjector.imagePoints.push_back(calibrationProjector.getCandidateImagePoints());
            calibrationProjector.getObjectPoints().push_back(circlesObjectPts);
            
            if(bLog) log << "Calibrating projector.." << endl;
            
            calibrationProjector.calibrate();
            
            if(calibrationProjector.size() >= numBoardsBeforeCleaning) {
                
                if(bLog) log << "Cleaning" << endl;
                
                // TODO : SIMULTANEOUS ClEAN !!
            }
            
            if(bLog) log << "Performing stereo-calibration.." << endl;
            
            calibrationProjector.stereoCalibration(calibrationCamera);
            
            if(bLog) log << "Done. " << numBoardsFinalProjector - calibrationProjector.size() << " boards to go." << endl;
        }
        else {
            /*
            if(bLog) log << "Found chessboard but could'nt find projected circles." << endl
                         << "> Adjust board or settings to see the circles on the processed image!" << endl;
            */
        }
        return bProjectedPatternFound;
    }
    return false;
}

void CameraProjectorCalibration::drawLastCameraImagePoints(){
    
    if(calibrationCamera.imagePoints.size()<=0) return;
    
    ofPushStyle(); ofSetColor(ofColor::blue);
    for(auto & p : calibrationCamera.imagePoints.back()) {
        ofCircle(ofxCv::toOf(p), 3);
    }
    ofPopStyle();
}

#pragma mark - projector

void CameraProjectorCalibration::drawProjectorStatic(){
    
    ofRectangle vp = ofGetCurrentViewport();
    ofViewport(projectorRect);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0,projectorRect.width, projectorRect.height, 0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    {
        const vector<cv::Point2f> pts = calibrationProjector.getCandidateImagePoints();
        ofSetColor(ofColor::white);
        for(int i = 0; i < pts.size(); i++) {
            ofCircle(pts[i].x, pts[i].y, 4);
        }
    }
    ofViewport(vp);
}

#pragma mark - setters / getters

void CameraProjectorCalibration::setState(CalibState state){
    
    switch (state) {
        case CAMERA:
            calibrationProjector.resetBoards();
            calibrationCamera.resetBoards();
            break;
        case PROJECTOR_STATIC:
            calibrationCamera.load("calibrationCamera.yml");
            calibrationCamera.resetBoards();
            calibrationCamera.setupCandidateObjectPoints();
            calibrationProjector.resetBoards();
            calibrationProjector.setupCandidateImagePoints();
            break;
        default:
            break;
    }
    currState = state;
    
    if(bLog) log << "Set state : " << getCurrentStateString() << endl;
}

string CameraProjectorCalibration::getCurrentStateString(){
    
    string name;
    switch (currState) {
        case CAMERA:            name = "CAMERA"; break;
        case PROJECTOR_STATIC:  name = "PROJECTOR_STATIC"; break;
        case PROJECTOR_DYNAMIC: name = "PROJECTOR_DYNAMIC"; break;
        default: break;
    }
    return name;
}

string CameraProjectorCalibration::getLog(int numLines){
    
    vector<string> elems;
    stringstream ss(log.str());
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