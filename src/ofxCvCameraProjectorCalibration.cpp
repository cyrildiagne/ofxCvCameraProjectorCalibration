//
//  CameraProjectorCalibration.cpp
//  camera_projector_calibration_CV
//
//  Created by kikko on 17/11/13.
//
//

#include "ofxCvCameraProjectorCalibration.h"

namespace ofxCv {
    
    
#pragma mark - CameraCalibration
    
    
    void CameraCalibration::setupCandidateObjectPoints(){
        candidateObjectPts.clear();
        for(int i = 0; i < patternSize.height; i++) {
            for(int j = 0; j < patternSize.width; j++) {
                candidateObjectPts.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
            }
        }
    }
    
    void CameraCalibration::computeCandidateBoardPose(const vector<cv::Point2f> & imgPts, cv::Mat& boardRot, cv::Mat& boardTrans){
        cv::solvePnP(candidateObjectPts, imgPts,
                     distortedIntrinsics.getCameraMatrix(),
                     distCoeffs,
                     boardRot, boardTrans);
    }
    
    // some crazy stuff, no idea what's happening there. So thanks Alvaro & Niklas!
    bool CameraCalibration::backProject(const Mat& boardRot64,
                                        const Mat& boardTrans64,
                                        const vector<Point2f>& imgPt,
                                        vector<Point3f>& worldPt) {
        if( imgPt.size() == 0 ) {
            return false;
        }
        else
        {
            Mat imgPt_h = Mat::zeros(3, imgPt.size(), CV_32F);
            for( int h=0; h<imgPt.size(); ++h ) {
                imgPt_h.at<float>(0,h) = imgPt[h].x;
                imgPt_h.at<float>(1,h) = imgPt[h].y;
                imgPt_h.at<float>(2,h) = 1.0f;
            }
            Mat Kinv64 = getUndistortedIntrinsics().getCameraMatrix().inv();
            Mat Kinv,boardRot,boardTrans;
            Kinv64.convertTo(Kinv, CV_32F);
            boardRot64.convertTo(boardRot, CV_32F);
            boardTrans64.convertTo(boardTrans, CV_32F);
            
            // Transform all image points to world points in camera reference frame
            // and then into the plane reference frame
            Mat worldImgPt = Mat::zeros( 3, imgPt.size(), CV_32F );
            Mat rot3x3;
            Rodrigues(boardRot, rot3x3);
            
            Mat transPlaneToCam = rot3x3.inv()*boardTrans;
            
            for( int i=0; i<imgPt.size(); ++i ) {
                Mat col = imgPt_h.col(i);
                Mat worldPtcam = Kinv*col;
                Mat worldPtPlane = rot3x3.inv()*(worldPtcam);
                
                float scale = transPlaneToCam.at<float>(2)/worldPtPlane.at<float>(2);
                Mat worldPtPlaneReproject = scale*worldPtPlane-transPlaneToCam;
                
                Point3f pt;
                pt.x = worldPtPlaneReproject.at<float>(0);
                pt.y = worldPtPlaneReproject.at<float>(1);
                pt.z = 0;
                worldPt.push_back(pt);
            }
        }
        return true;
    }
    
    
#pragma mark - ProjectorCalibration
    
    
    void ProjectorCalibration::setStaticCandidateImagePoints(){
        candidateImagePoints.clear();
        Point2f p;
        for(int i = 0; i < patternSize.height; i++) {
            for(int j = 0; j < patternSize.width; j++) {
                p.x = patternPosition.x + float(((2 * j) + (i % 2)) * squareSize);
                p.y = patternPosition.y + float(i * squareSize);
                candidateImagePoints.push_back(p);
            }
        }
    }
    
    void ProjectorCalibration::setCandidateImagePoints(vector<cv::Point2f> pts){
        candidateImagePoints = pts;
    }
    
    void ProjectorCalibration::setImagerSize(int width, int height) {
        imagerSize = cv::Size(width, height);
        distortedIntrinsics.setImageSize(imagerSize);
        undistortedIntrinsics.setImageSize(imagerSize);
        addedImageSize = imagerSize;
    }
    
    void ProjectorCalibration::setPatternPosition(float px, float py) {
        patternPosition = Point2f(px, py);
    }
    
    
#pragma mark - CameraProjectorCalibration
    
    void CameraProjectorCalibration::load(string cameraConfig, string projectorConfig, string extrinsicsConfig){
        calibrationCamera.load(cameraConfig);
        calibrationProjector.load(projectorConfig);
        loadExtrinsics(extrinsicsConfig);
    }
    
    void CameraProjectorCalibration::setup(int projectorWidth, int projectorHeight){
        
        calibrationCamera.setPatternSize(8, 5);
        calibrationCamera.setSquareSize(1.25);
        calibrationCamera.setPatternType(CHESSBOARD);
        
        calibrationProjector.setImagerSize(projectorWidth, projectorHeight);
        calibrationProjector.setPatternSize(4, 5);
        calibrationProjector.setPatternPosition(500, 250);
        calibrationProjector.setSquareSize(40);
        calibrationProjector.setPatternType(ASYMMETRIC_CIRCLES_GRID);
    }

    void CameraProjectorCalibration::saveExtrinsics(string filename, bool absolute) const {
        
        cv::FileStorage fs(ofToDataPath(filename, absolute), cv::FileStorage::WRITE);
        fs << "Rotation_Vector" << rotCamToProj;
        fs << "Translation_Vector" << transCamToProj;
    }

    void CameraProjectorCalibration::loadExtrinsics(string filename, bool absolute) {
        
        cv::FileStorage fs(ofToDataPath(filename, absolute), cv::FileStorage::READ);
        fs["Rotation_Vector"] >> rotCamToProj;
        fs["Translation_Vector"] >> transCamToProj;
    }
    
    vector<Point2f> CameraProjectorCalibration::getProjected(const vector<Point3f> & pts,
                                                             const cv::Mat & rotObjToCam,
                                                             const cv::Mat & transObjToCam){
        cv::Mat rotObjToProj, transObjToProj;
        
        cv::composeRT(rotObjToCam,  transObjToCam,
                      rotCamToProj, transCamToProj,
                      rotObjToProj, transObjToProj);
        
        vector<Point2f> out;
        projectPoints(Mat(pts),
                      rotObjToProj, transObjToProj,
                      calibrationProjector.getDistortedIntrinsics().getCameraMatrix(),
                      calibrationProjector.getDistCoeffs(),
                      out);
        return out;
    }
    
    bool CameraProjectorCalibration::addProjected(cv::Mat img, cv::Mat processedImg){
        
        vector<cv::Point2f> chessImgPts;
        
        bool bPrintedPatternFound = calibrationCamera.findBoard(img, chessImgPts, true);
        
        if(bPrintedPatternFound) {
            
            vector<cv::Point2f> circlesImgPts;
            bool bProjectedPatternFound = cv::findCirclesGrid(processedImg, calibrationProjector.getPatternSize(), circlesImgPts, cv::CALIB_CB_ASYMMETRIC_GRID);
            
            if(bProjectedPatternFound){
            
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
                
                return true;
            }
        }
        return false;
    }
    
    bool CameraProjectorCalibration::setDynamicProjectorImagePoints(cv::Mat img){
        
        vector<cv::Point2f> chessImgPts;
        bool bPrintedPatternFound = calibrationCamera.findBoard(img, chessImgPts, true);

        if(bPrintedPatternFound) {
            
            cv::Mat boardRot;
            cv::Mat boardTrans;
            calibrationCamera.computeCandidateBoardPose(chessImgPts, boardRot, boardTrans);
            
            const auto & camCandObjPts = calibrationCamera.getCandidateObjectPoints();
            Point3f axisX = camCandObjPts[1] - camCandObjPts[0];
            Point3f axisY = camCandObjPts[calibrationCamera.getPatternSize().width] - camCandObjPts[0];
            Point3f pos   = camCandObjPts[0] - axisY * (calibrationCamera.getPatternSize().width-2);
            
            vector<Point3f> auxObjectPoints;
            for(int i = 0; i < calibrationProjector.getPatternSize().height; i++) {
                for(int j = 0; j < calibrationProjector.getPatternSize().width; j++) {
                    auxObjectPoints.push_back(pos + axisX * float((2 * j) + (i % 2)) + axisY * i);
                }
            }
            
            Mat Rc1, Tc1, Rc1inv, Tc1inv, Rc2, Tc2, Rp1, Tp1, Rp2, Tp2;
            Rp1 = calibrationProjector.getBoardRotations().back();
            Tp1 = calibrationProjector.getBoardTranslations().back();
            Rc1 = calibrationCamera.getBoardRotations().back();
            Tc1 = calibrationCamera.getBoardTranslations().back();
            Rc2 = boardRot;
            Tc2 = boardTrans;
            
            Mat auxRinv = Mat::eye(3,3,CV_32F);
            Rodrigues(Rc1,auxRinv);
            auxRinv = auxRinv.inv();
            Rodrigues(auxRinv, Rc1inv);
            Tc1inv = -auxRinv*Tc1;
            Mat Raux, Taux;
            composeRT(Rc2, Tc2, Rc1inv, Tc1inv, Raux, Taux);
            composeRT(Raux, Taux, Rp1, Tp1, Rp2, Tp2);
            
            vector<Point2f> followingPatternImagePoints;
            projectPoints(Mat(auxObjectPoints),
                          Rp2, Tp2,
                          calibrationProjector.getDistortedIntrinsics().getCameraMatrix(),
                          calibrationProjector.getDistCoeffs(),
                          followingPatternImagePoints);
            
            calibrationProjector.setCandidateImagePoints(followingPatternImagePoints);
        }
        return bPrintedPatternFound;
    }
    
    void CameraProjectorCalibration::stereoCalibrate(){
        
        const auto & objectPoints = calibrationProjector.getObjectPoints();
        
        vector<vector<cv::Point2f> > auxImagePointsCamera;
        for (int i=0; i<objectPoints.size() ; i++ ) {
            vector<cv::Point2f> auxImagePoints;
            projectPoints(cv::Mat(objectPoints[i]),
                          calibrationCamera.getBoardRotations()[i],
                          calibrationCamera.getBoardTranslations()[i],
                          calibrationCamera.getDistortedIntrinsics().getCameraMatrix(),
                          calibrationCamera.getDistCoeffs(),
                          auxImagePoints);
            
            auxImagePointsCamera.push_back(auxImagePoints);
        }
        
        cv::Mat projectorMatrix     = calibrationProjector.getDistortedIntrinsics().getCameraMatrix();
        cv::Mat projectorDistCoeffs = calibrationProjector.getDistCoeffs();
        cv::Mat cameraMatrix        = calibrationCamera.getDistortedIntrinsics().getCameraMatrix();
        cv::Mat cameraDistCoeffs    = calibrationCamera.getDistCoeffs();
        
        cv::Mat fundamentalMatrix, essentialMatrix;
        cv::Mat rotation3x3;
        
        cv::stereoCalibrate(objectPoints,
                            auxImagePointsCamera,
                            calibrationProjector.imagePoints,
                            cameraMatrix, cameraDistCoeffs,
                            projectorMatrix, projectorDistCoeffs,
                            calibrationCamera.getDistortedIntrinsics().getImageSize(),
                            rotation3x3, transCamToProj,
                            essentialMatrix, fundamentalMatrix);
        
        cv::Rodrigues(rotation3x3, rotCamToProj);
    }

    void CameraProjectorCalibration::resetBoards(){
        calibrationCamera.resetBoards();
        calibrationProjector.resetBoards();
    }
    
    int CameraProjectorCalibration::cleanStereo(float maxReproj){
        int removed = 0;
		for(int i = calibrationProjector.size() - 1; i >= 0; i--) {
			if(calibrationProjector.getReprojectionError(i) > maxReproj) {
				calibrationProjector.remove(i);
				calibrationCamera.remove(i);
                removed++;
			}
		}
        return removed;
    }
    
}