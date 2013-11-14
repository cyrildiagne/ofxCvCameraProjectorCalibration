//
//  ofxCvCameraProjector.h
//
//  Created by kikko_fr on 14/11/13.
//
//

#pragma once

#include "ofMain.h"
#include "ofxCv.h"

namespace ofxCv {

    class CameraProjector {
        
    public:
        /* setup with screen rect & projector rect.
         * Depends on screen position in system configuration */
        void setup(ofRectangle screenRect, ofRectangle projectorRect);
        
        /* update object-to-projector transform
           composing object-to-camera and camera-to-projector matrices */
        void update(cv::Mat rotObjToCam, cv::Mat transObjToCam);
        
        /* setup the openGL projection matrix
         * to draw in object space (2D) and aligned projection */
        void beginGL();
        
        /*  not implemented yet (for now, just call beginGL last in your ofApp::draw method */
        void endGL();
        
        /* use cv::projectPoints to get points projection from projector-space & using projector intrinsics
           slower than using CameraProjector::beginGL but can be useful for debugging */
        
        vector<ofPoint> getProjected(const vector<ofPoint> & inputPts);
        
        /* return a string containing obj-to-cam and obj-to-proj transform vectors */
        string toString();
        
        /* this should be moved to ofxCv::Utils */
        string getRTMatInfos(const cv::Mat rvecs, const cv::Mat tvecs);
        
        /* getters */
        const Calibration & getCalibrationCamera() { return calibrationCamera; }
        const Calibration & getCalibrationProjector() { return calibrationProjector; }
        const cv::Mat & getCameraMatrix() { return cameraMatrix; }
        const cv::Mat & getCameraDistCoefs() { return cameraDistCoefs; }
        const cv::Mat & getProjectorMatrix() { return projectorMatrix; }
        const cv::Mat & getProjectorDistCoefs() { return projectorDistCoefs; }
        
    private:
        
        ofRectangle screenRect, projectorRect;
        
        /* calibration objects */
        Calibration calibrationCamera;
        Calibration calibrationProjector;
        
        /* calibration matrices */
        cv::Mat cameraMatrix, cameraDistCoefs;
        cv::Mat projectorMatrix, projectorDistCoefs;
        
        /* rotation & translation vectors */
        cv::Mat rotObjToCam, transObjToCam;
        cv::Mat rotCamToProj, transCamToProj;
        cv::Mat rotObjToProj, transObjToProj;
    };
    
}