//
//  ofxCvCameraProjector.cpp
//
//  Created by kikko_fr on 14/11/13.
//
//

#include "ofxCvCameraProjector.h"

using namespace cv;

namespace ofxCv {

    void CameraProjector::setup(ofRectangle screen, ofRectangle projector){
        
        screenRect = screen;
        projectorRect = projector;
        
        calibrationCamera.load("calibrationCamera.yml");
        calibrationProjector.load("calibrationProjector.yml");
        
        projectorMatrix = calibrationProjector.getDistortedIntrinsics().getCameraMatrix();
        projectorDistCoefs = calibrationProjector.getDistCoeffs();
        
        cameraMatrix = calibrationCamera.getDistortedIntrinsics().getCameraMatrix();
        cameraDistCoefs = calibrationCamera.getDistCoeffs();
        
        rotObjToCam = Mat::zeros(3, 1, CV_64F);
        transObjToCam = Mat::zeros(3, 1, CV_64F);
        
        rotObjToProj = Mat::zeros(3, 1, CV_64F);
        transObjToProj = Mat::zeros(3, 1, CV_64F);
        
        cv::FileStorage fs(ofToDataPath("CameraProjectorExtrinsics.yml", true), cv::FileStorage::READ);
        fs["Rotation_Vector"] >> rotCamToProj;
        fs["Translation_Vector"] >> transCamToProj;
    }

    void CameraProjector::update(cv::Mat rotObjToCam, cv::Mat transObjToCam){
        this->rotObjToCam = rotObjToCam;
        this->transObjToCam = transObjToCam;
        cv::composeRT(rotObjToCam,  transObjToCam,
                      rotCamToProj, transCamToProj,
                      rotObjToProj, transObjToProj);
    }
    
    
    void CameraProjector::beginGL(){
        ofPushMatrix();
        // Set perspective matrix using the projector intrinsics
        calibrationProjector.getDistortedIntrinsics().loadProjectionMatrix(1, 10000, projectorRect.position);
        // apply model to projector transformations
        applyMatrix(makeMatrix(rotObjToProj, transObjToProj));
    }
    
    void CameraProjector::endGL(){
        ofPopMatrix();
    }
    
    vector<ofPoint> CameraProjector::getProjected(const vector<ofPoint> & inputPts){
        vector<Point3f> cvIn(inputPts.size());
        for (int i=0; i<inputPts.size(); i++) {
            cvIn[i] = toCv(inputPts[i]);
        }
        vector<Point2f> cvOut;
        projectPoints(Mat(cvIn), rotObjToProj, transObjToProj, projectorMatrix, projectorDistCoefs, cvOut);
        vector<ofPoint> outputPts(cvOut.size());
        for (int i=0; i<outputPts.size(); i++) {
            outputPts[i] = toOf(cvOut[i]);
        }
        return outputPts;
    }
    
#pragma mark - getters
    
    string CameraProjector::toString(){
        string buff;
        buff += "Input (Object-to-Camera) R/T\n";
        buff += getRTMatInfos(rotObjToCam, transObjToCam) + "\n";
        buff += "Final (Object-to-Projector) R/T\n";
        buff += getRTMatInfos(rotObjToProj, transObjToProj);
        return buff;
    }
    
    string CameraProjector::getRTMatInfos(const cv::Mat rvecs, const cv::Mat tvecs){
        cv::Mat rot3x3 = cv::Mat::zeros(3, 3, CV_32F);
        Rodrigues(rvecs, rot3x3);
        stringstream imgRTs;
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++)  imgRTs << ofToString(rot3x3.at<double>(i,j),1) << "\t";
            imgRTs << ofToString(tvecs.at<double>(i),1) << endl;
        }
        return imgRTs.str();
    }
}