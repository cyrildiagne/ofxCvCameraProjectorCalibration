//
//  CameraProjectorCalibration.cpp
//  camera_projector_calibration_CV
//
//  Created by kikko on 17/11/13.
//
//

#include "CameraProjectorCalibration.h"

namespace ofxCv {

    void CameraProjectorCalibration::setup(CalibState initState, int projectorWidth, int projectorHeight){
        
        setupDefaultParams();
        
        calibrationCamera.setPatternSize(8, 5);
        calibrationCamera.setSquareSize(1.25);
        calibrationCamera.setPatternType(CHESSBOARD);
        
        calibrationProjector.setImagerSize(projectorWidth, projectorHeight);
        calibrationProjector.setPatternSize(4, 5);
        calibrationProjector.setPatternPosition(500, 250);
        calibrationProjector.setSquareSize(40);
        calibrationProjector.setPatternType(ASYMMETRIC_CIRCLES_GRID);
        
        bLog = true;
        
        setState(initState);
        
        log() << "Calibration started at step : " << getCurrentStateString() << endl;
    }

    void CameraProjectorCalibration::setupDefaultParams(){
        
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

    #pragma mark - camera

    bool CameraProjectorCalibration::calibrateCamera(cv::Mat img){
        
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

    bool CameraProjectorCalibration::calibrateProjector(cv::Mat img){
        
        
        vector<cv::Point2f> chessImgPts;
        bool bPrintedPatternFound = calibrationCamera.findBoard(img, chessImgPts, true);
        if(bPrintedPatternFound){
            
            if(img.type() != CV_8UC1) {
                cvtColor(img, processedImg, CV_RGB2GRAY);
            } else {
                processedImg = img;
            }
            cv::threshold(processedImg, processedImg, circleDetectionThreshold, 255, cv::THRESH_BINARY_INV);
            
            vector<cv::Point2f> circlesImgPts;
            bool bProjectedPatternFound = cv::findCirclesGrid(processedImg, calibrationProjector.getPatternSize(), circlesImgPts, cv::CALIB_CB_ASYMMETRIC_GRID);
            
            if(bProjectedPatternFound){
                
                log() << "Found projected circles" << endl;
                
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
                
                log() << "Calibrating projector" << endl;
                
                calibrationProjector.calibrate();
                
                if(calibrationProjector.size() >= numBoardsBeforeCleaning) {
                    
                    log() << "Cleaning" << endl;
                    
                    cleanStereo();
                    
                    if(currState == PROJECTOR_DYNAMIC && calibrationProjector.size() < numBoardsBeforeDynamicProjection) {
                        log() << "Too many boards removed, restarting to PROJECTOR_STATIC" << endl;
                        
                        setState(PROJECTOR_STATIC);
                        return false;
                    }
                }
                
                log() << "Performing stereo-calibration" << endl;
                
                stereoCalibrate();
                
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
                        saveExtrinsics("CameraProjectorExtrinsics.yml");
                        log() << "Stereo Calibration finished & saved to CameraProjectorExtrinsics.yml" << endl;
                        log() << "Congrats, you made it ;)" << endl;
                    }
                }
                
            }
            else {
                /*
                log() << "Found chessboard but could'nt find projected circles." << endl
                             << "> Adjust board or settings to see the circles on the processed image!" << endl;
                */
            }
            return bProjectedPatternFound;
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
            // Previous bord position in projector coordinate frame:
            Rp1 = calibrationProjector.getBoardRotations().back();
            Tp1 = calibrationProjector.getBoardTranslations().back();
            // Previous board position in camera coordinate frame:
            Rc1 = calibrationCamera.getBoardRotations().back();
            Tc1 = calibrationCamera.getBoardTranslations().back();
            // Latest board position in camera coordiante frame (not yet in the vector list!!):
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
        
        cv::stereoCalibrate(objectPoints,           // common "3d" points (on the board)
                            auxImagePointsCamera,   // image points for the camera
                            calibrationProjector.imagePoints,            // image points for the projector
                            cameraMatrix, cameraDistCoeffs,
                            projectorMatrix, projectorDistCoeffs,
                            calibrationCamera.getDistortedIntrinsics().getImageSize(), // <--- only used for initialization
                            rotation3x3, transCamToProj,  // << ----  OUPUT: position of CAMERA in PROJECTOR coordinate frame
                            essentialMatrix, fundamentalMatrix); // << ---- this is also the result (but is not made avaiblable by the method)
        
        cv::Rodrigues(rotation3x3, rotCamToProj);
    }

    void CameraProjectorCalibration::cleanStereo(){
        int removed = 0;
		for(int i = calibrationProjector.size() - 1; i >= 0; i--) {
			if(calibrationProjector.getReprojectionError(i) > maxReprojErrorProjector) {
                log() << "Removing calib " << i << " with reproj error " << calibrationProjector.getReprojectionError(i) << endl;
				calibrationProjector.remove(i);
				calibrationCamera.remove(i);
                removed++;
			}
		}
        if(removed>0) {
            log() << "Removed " << removed << " calibration pairs" << endl;
        }
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
                calibrationProjector.setStaticCandidateImagePoints();
                break;
            case PROJECTOR_DYNAMIC:
                break;
            default:
                break;
        }
        currState = state;
        
        log() << "Set state : " << getCurrentStateString() << endl;
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
}