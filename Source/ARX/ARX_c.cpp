/*
 *  ARX_c.cpp
 *  artoolkitX
 *
 *  This file is part of artoolkitX.
 *
 *  artoolkitX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  artoolkitX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with artoolkitX.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of this library give you
 *  permission to link this library with independent modules to produce an
 *  executable, regardless of the license terms of these independent modules, and to
 *  copy and distribute the resulting executable under terms of your choice,
 *  provided that you also meet, for each linked independent module, the terms and
 *  conditions of the license of that module. An independent module is a module
 *  which is neither derived from nor based on this library. If you modify this
 *  library, you may extend this exception to your version of the library, but you
 *  are not obligated to do so. If you do not wish to do so, delete this exception
 *  statement from your version.
 *
 *  Copyright 2018 Realmax, Inc.
 *  Copyright 2015 Daqri, LLC.
 *  Copyright 2010-2015 ARToolworks, Inc.
 *
 *  Author(s): Philip Lamb, Julian Looser.
 *
 */

// ----------------------------------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------------------------------

#include <ARX/ARX_c.h>
#include <ARX/ARController.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ARX/ARTrackableMultiSquareAuto.h>
#include <ARX/Calibration.hpp>
#include <ARX/calc.hpp>

#ifdef DEBUG
#  ifdef _WIN32
#    define MAXPATHLEN MAX_PATH
#    include <direct.h>               // _getcwd()
#    define getcwd _getcwd
#  else
#    include <unistd.h>
#    include <sys/param.h>
#  endif
#endif
#include <stdio.h>
#if !ARX_TARGET_PLATFORM_WINDOWS && !ARX_TARGET_PLATFORM_WINRT
#  include <pthread.h>
#endif

// ----------------------------------------------------------------------------------------------------

#if defined(_MSC_VER) && !defined(NAN)
#  define __nan_bytes { 0, 0, 0xc0, 0x7f }
static union { unsigned char __c[4]; float __d; } __nan_union = { __nan_bytes };
#  define NAN (__nan_union.__d)
#endif

// ----------------------------------------------------------------------------------------------------

static ARController *gARTK = NULL;
static ARController *gARTKLowRes = NULL;

// Calibration variables:
cv::Size gCalibrationPatternSize;
std::vector<std::vector<cv::Point2f> > foundCorners;
std::vector<cv::Point2f> corners;
std::vector<cv::Point2f> cornersLowRes;
int maxCornersFound;
float cornerSpacing;
int videoWidth;
int videoHeight;
int videoWidthLowRes;
int videoHeightLowRes;

// ----------------------------------------------------------------------------------------------------

void arwRegisterLogCallback(PFN_LOGCALLBACK callback)
{
    arLogSetLogger(callback, 1); // 1 -> only callback on same thread, as required e.g. by C# interop.
}

void arwSetLogLevel(const int logLevel)
{
    if (logLevel >= 0) {
        arLogLevel = logLevel;
    }
}

// ----------------------------------------------------------------------------------------------------
#pragma mark  artoolkitX lifecycle functions
// ----------------------------------------------------------------------------------------------------

bool arwInitialiseAR()
{
    if (!gARTK) gARTK = new ARController;
	return gARTK->initialiseBase();
    
    if (!gARTKLowRes) gARTKLowRes = new ARController;
    //gARTKLowRes->logCallback = log;
    return gARTKLowRes->initialiseBase();
}

bool arwGetARToolKitVersion(char *buffer, int length)
{
	if (!buffer) return false;
    if (!gARTK) return false;
    
	if (const char *version = gARTK->getARToolKitVersion()) {
		strncpy(buffer, version, length - 1); buffer[length - 1] = '\0';
		return true;
	}
	return false;
}

int arwGetError(bool lowRes)
{
    if (!lowRes) {
        if (!gARTK) return ARX_ERROR_NONE;
        return gARTK->getError();
    } else {
        if (!gARTKLowRes) return ARX_ERROR_NONE;
        return gARTKLowRes->getError();
    }
}

bool arwChangeToResourcesDir(const char *resourcesDirectoryPath)
{
    bool ok;
#if ARX_TARGET_PLATFORM_ANDROID
    if (resourcesDirectoryPath) ok = (arUtilChangeToResourcesDirectory(AR_UTIL_RESOURCES_DIRECTORY_BEHAVIOR_USE_SUPPLIED_PATH, resourcesDirectoryPath, NULL) == 0);
	else ok = (arUtilChangeToResourcesDirectory(AR_UTIL_RESOURCES_DIRECTORY_BEHAVIOR_BEST, NULL, NULL) == 0);
#elif ARX_TARGET_PLATFORM_WINRT
	ok = false; // No current working directory in WinRT.
#else
    if (resourcesDirectoryPath) ok = (arUtilChangeToResourcesDirectory(AR_UTIL_RESOURCES_DIRECTORY_BEHAVIOR_USE_SUPPLIED_PATH, resourcesDirectoryPath) == 0);
	else ok = (arUtilChangeToResourcesDirectory(AR_UTIL_RESOURCES_DIRECTORY_BEHAVIOR_BEST, NULL) == 0);
#endif
#ifdef DEBUG
    char buf[MAXPATHLEN];
    ARLOGd("cwd is '%s'.\n", getcwd(buf, sizeof(buf)));
#endif
    return (ok);
}

bool arwStartRunning(const char *vconf, const char *cparaName)
{
    if (!gARTK) return false;
	return gARTK->startRunning(vconf, cparaName, NULL, 0);
}

bool arwStartRunningB(const char *vconf, const char *cparaBuff, const int cparaBuffLen)
{
    if (!gARTK) return false;
	return gARTK->startRunning(vconf, NULL, cparaBuff, cparaBuffLen);
}

bool arwStartRunningStereo(const char *vconfL, const char *cparaNameL, const char *vconfR, const char *cparaNameR, const char *transL2RName)
{
    if (!gARTK) return false;
	return gARTK->startRunningStereo(vconfL, cparaNameL, NULL, 0L, vconfR, cparaNameR, NULL, 0L, transL2RName, NULL, 0L);
}

bool arwStartRunningStereoB(const char *vconfL, const char *cparaBuffL, const int cparaBuffLenL, const char *vconfR, const char *cparaBuffR, const int cparaBuffLenR, const char *transL2RBuff, const int transL2RBuffLen)
{
    if (!gARTK) return false;
	return gARTK->startRunningStereo(vconfL, NULL, cparaBuffL, cparaBuffLenL, vconfR, NULL, cparaBuffR, cparaBuffLenR, NULL, transL2RBuff, transL2RBuffLen);
}

bool arwIsRunning()
{
    if (!gARTK) return false;
	return gARTK->isRunning();
}

bool arwIsInited()
{
    if (!gARTK) return false;
	return gARTK->isInited();
}

bool arwStopRunning()
{
    if (!gARTK) return false;
	return gARTK->stopRunning();
}

bool arwShutdownAR()
{
    if (gARTK) {
        delete gARTK; // Delete the artoolkitX instance to initiate shutdown.
        gARTK = NULL;
    }
    if (gARTKLowRes) {
        delete gARTKLowRes; // Delete the artoolkitX instance to initiate shutdown.
        gARTKLowRes = NULL;
    }

    return (true);
}


// ----------------------------------------------------------------------------------------------------
#pragma mark  Single image functions
// -------------------------------------------------------------------------------------------------

void arwInitARToolKit(const char *vconf, const char *cparaName, const char *vconfLowRes, const char *cparaNameLowRes, const int xSize, const int ySize, const int xSizeLowRes, const int ySizeLowRes)
{
    
#ifdef ARDOUBLE_IS_FLOAT
    ARLOGe("ARDOUBLE_IS_FLOAT is defined\n");
#else
    ARLOGe("ARDOUBLE_IS_FLOAT is NOT defined\n");
#endif
    
    if (!gARTK) gARTK = new ARController;
    //gARTK->logCallback = log;
    gARTK->initialiseBase();
    
    gARTK->startRunning(vconf, cparaName, NULL, 0);
    
    if (!gARTKLowRes) gARTKLowRes = new ARController;
    //gARTKLowRes->logCallback = log;
    gARTKLowRes->initialiseBase();
    
    gARTKLowRes->startRunning(vconfLowRes, cparaNameLowRes, NULL, 0);
    
    return;
}

bool arwUpdateARToolKit(unsigned char *imageBytes, bool lowRes, bool doDatums)
{
    //ARLOGe("UpdateARToolKit called.\n");
    //ARPRINT("UpdateARToolKit called.\n");
    
    if (!lowRes) {
        if (!gARTK) return false;
        return gARTK->updateWithImage(imageBytes, lowRes, doDatums);
    } else {
        if (!gARTKLowRes) return false;
        return gARTKLowRes->updateWithImage(imageBytes, lowRes, doDatums);
    }
}

void arwCleanupARToolKit()
{
    if (gARTK) {
        delete gARTK; // Delete the ARToolKit instance to initiate shutdown.
        gARTK = NULL;
    }
    
    if (gARTKLowRes) {
        delete gARTKLowRes; // Delete the ARToolKit instance to initiate shutdown.
        gARTKLowRes = NULL;
    }
    
    return;
}


// ----------------------------------------------------------------------------------------------------
#pragma mark  Video stream management
// -------------------------------------------------------------------------------------------------


bool arwGetProjectionMatrix(const float nearPlane, const float farPlane, double p[16], bool lowRes)
{
    if (!lowRes) {
        if (!gARTK) return false;
        return gARTK->projectionMatrix(0, nearPlane, farPlane, p);
    } else {
        if (!gARTKLowRes) return false;
        return gARTKLowRes->projectionMatrix(0, nearPlane, farPlane, p);
    }
}

bool arwGetProjectionMatrixStereo(const float nearPlane, const float farPlane, float pL[16], float pR[16])
{
    if (!gARTK) return false;

#ifdef ARDOUBLE_IS_FLOAT
    return (gARTK->projectionMatrix(0, nearPlane, farPlane, pL) && gARTK->projectionMatrix(1, nearPlane, farPlane, pR));
#else
    ARdouble p0L[16];
    ARdouble p0R[16];
    if (!gARTK->projectionMatrix(0, nearPlane, farPlane, p0L) || !gARTK->projectionMatrix(1, nearPlane, farPlane, p0R)) {
        return false;
    }
    for (int i = 0; i < 16; i++) pL[i] = (float)p0L[i];
    for (int i = 0; i < 16; i++) pR[i] = (float)p0R[i];
    return true;
#endif
}


bool arwGetVideoParams(int *width, int *height, int *pixelSize, char *pixelFormatStringBuffer, int pixelFormatStringBufferLen)
{
    AR_PIXEL_FORMAT pf;
    
    if (!gARTK) return false;
	if (!gARTK->videoParameters(0, width, height, &pf)) return false;
    if (pixelSize) *pixelSize = arUtilGetPixelSize(pf);
    if (pixelFormatStringBuffer && pixelFormatStringBufferLen > 0) {
        strncpy(pixelFormatStringBuffer, arUtilGetPixelFormatName(pf), pixelFormatStringBufferLen);
        pixelFormatStringBuffer[pixelFormatStringBufferLen - 1] = '\0'; // guarantee nul termination.
    }
    return true;
}

bool arwGetVideoParamsStereo(int *widthL, int *heightL, int *pixelSizeL, char *pixelFormatStringBufferL, int pixelFormatStringBufferLenL, int *widthR, int *heightR, int *pixelSizeR, char *pixelFormatStringBufferR, int pixelFormatStringBufferLenR)
{
    AR_PIXEL_FORMAT pfL, pfR;
    
    if (!gARTK) return false;
	if (!gARTK->videoParameters(0, widthL, heightL, &pfL)) return false;
	if (!gARTK->videoParameters(1, widthR, heightR, &pfR)) return false;
    if (pixelSizeL) *pixelSizeL = arUtilGetPixelSize(pfL);
    if (pixelSizeR) *pixelSizeR = arUtilGetPixelSize(pfR);
    if (pixelFormatStringBufferL && pixelFormatStringBufferLenL > 0) {
        strncpy(pixelFormatStringBufferL, arUtilGetPixelFormatName(pfL), pixelFormatStringBufferLenL);
        pixelFormatStringBufferL[pixelFormatStringBufferLenL - 1] = '\0'; // guarantee nul termination.
    }
    if (pixelFormatStringBufferR && pixelFormatStringBufferLenR > 0) {
        strncpy(pixelFormatStringBufferR, arUtilGetPixelFormatName(pfR), pixelFormatStringBufferLenR);
        pixelFormatStringBufferR[pixelFormatStringBufferLenR - 1] = '\0'; // guarantee nul termination.
    }
    return true;
}

bool arwCapture()
{
    if (!gARTK) return false;
    return (gARTK->capture());
}

bool arwUpdateAR()
{
    if (!gARTK) return false;
    return gARTK->update();
}

bool arwUpdateTexture32(uint32_t *buffer, bool lowRes)
{
    if (!lowRes) {
        if (!gARTK) return false;
        return gARTK->updateTextureRGBA32(0, buffer);
    } else {
        if (!gARTKLowRes) return false;
        return gARTKLowRes->updateTextureRGBA32(0, buffer);
    }
}

bool arwUpdateTexture32Stereo(uint32_t *bufferL, uint32_t *bufferR)
{
    if (!gARTK) return false;
    return (gARTK->updateTextureRGBA32(0, bufferL) && gARTK->updateTextureRGBA32(1, bufferR));
}

// ----------------------------------------------------------------------------------------------------
#pragma mark  Calibration.
// ----------------------------------------------------------------------------------------------------

bool arwInitChessboardCorners(int nHorizontal, int nVertical, float patternSpacing, int calibImageNum, int xsize, int ysize, int xsizeLowRes, int ysizeLowRes)
{
    gCalibrationPatternSize = cv::Size(nHorizontal, nVertical);
    maxCornersFound = calibImageNum;
    cornerSpacing = patternSpacing;
    foundCorners.clear();
    videoWidth = xsize;
    videoHeight = ysize;
    videoWidthLowRes = xsizeLowRes;
    videoHeightLowRes = ysizeLowRes;

    ARLOGe("Initialised corner calibration OK.\n");
    return true;
}

int arwFindChessboardCorners(float* vertices, int *corner_count, ARUint8 *imageBytes, bool lowRes)
{
    int cornerFoundAllFlag;
    int i;
    
    if (!lowRes) {
        
        cv::Mat calibImage = cv::Mat(videoHeight, videoWidth, CV_8UC1, imageBytes);

        corners.clear();
        cornerFoundAllFlag = cv::findChessboardCorners(calibImage, gCalibrationPatternSize, corners, cv::CALIB_CB_FAST_CHECK|cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_FILTER_QUADS);
        
        *corner_count = (int)corners.size();
        
        if (cornerFoundAllFlag) ARLOGe("Found %d corners\n", (int)corners.size());
        
        if (*corner_count != gCalibrationPatternSize.width * gCalibrationPatternSize.height) cornerFoundAllFlag = 0;
        
        for (i = 0; i < *corner_count; i++) {
            vertices[i*2    ] = corners[i].x ;
            vertices[i*2 + 1] = corners[i].y;
        }
        
    } else {
        
        cv::Mat calibImageLowRes = cv::Mat(videoHeightLowRes, videoWidthLowRes, CV_8UC1, imageBytes);

        cornersLowRes.clear();
        cornerFoundAllFlag = cv::findChessboardCorners(calibImageLowRes, gCalibrationPatternSize, cornersLowRes, cv::CALIB_CB_FAST_CHECK|cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_FILTER_QUADS);
        
        *corner_count = (int)cornersLowRes.size();
        for (i = 0; i < *corner_count; i++) {
            vertices[i*2    ] = cornersLowRes[i].x ;
            vertices[i*2 + 1] = cornersLowRes[i].y;
        }
    }
    
    return cornerFoundAllFlag;
}

int arwCaptureChessboardCorners(ARUint8 *imageBytes, int n)
{
    if (foundCorners.size() >= maxCornersFound & n == -1) return 0;
    
    ARLOGe("CornerSubPix %d corners\n", (int)corners.size());

    cv::Mat calibImage = cv::Mat(videoHeight, videoWidth, CV_8UC1, imageBytes);

    // Refine the corner positions.
    cornerSubPix(calibImage, corners, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 0.1));
    
    ARLOGe("Capturing %d corners\n", (int)corners.size());
    
    // Save the corners.
    if (n == -1 || n >= foundCorners.size()) {
        foundCorners.push_back(corners);
    } else {
        foundCorners[n] = corners;
    }
    
    return (int)foundCorners.size();
}

float arwCalibChessboardCorners(char *file_name, float *results)
{
    ARParam param_out;
    float projectionError;
    
    ARLOGe("About to calibrate corners. foundCorners.Size=%d\n", (int)foundCorners.size());
    for (int i = 0; i < foundCorners.size(); i++) {
        ARLOGe("Corner set %d size=%d\n", i + 1, foundCorners[i].size());
    }
    ARLOGe("Pattern Size=%d,%d\n", gCalibrationPatternSize.width, gCalibrationPatternSize.height);
    ARLOGe("Pattern Spacing=%f\n", cornerSpacing);
    
    projectionError = calc((int)foundCorners.size(), Calibration::CalibrationPatternType::CHESSBOARD, gCalibrationPatternSize, cornerSpacing, foundCorners, videoWidth, videoHeight, AR_DIST_FUNCTION_VERSION_DEFAULT, &param_out, results);
    
    ARLOGe("About to save calibration file...\n");
    
    if (arParamSave(file_name, 1, &param_out) < 0) {
        ARLOGe("Error writing camera_para.dat file.\n");
    } else {
        ARLOGe("Success in saving camera_para.dat file.\n");
    }
    
    return projectionError;
}

void arwCleanupChessboardCorners()
{
    // if (calibImage) cvReleaseImageHeader(&calibImage);
    // free(videoFrame);
    
    // if (calibImageLowRes) cvReleaseImageHeader(&calibImageLowRes);
    // free(videoFrameLowRes);
}


// ----------------------------------------------------------------------------------------------------
#pragma mark  Video stream drawing.
// ----------------------------------------------------------------------------------------------------
bool arwDrawVideoInit(const int videoSourceIndex)
{
    if (!gARTK) return false;
    
    return (gARTK->drawVideoInit(videoSourceIndex));
}

bool arwDrawVideoSettings(int videoSourceIndex, int width, int height, bool rotate90, bool flipH, bool flipV, int hAlign, int vAlign, int scalingMode, int32_t viewport[4])
{
    if (!gARTK)return false;
    
    return (gARTK->drawVideoSettings(videoSourceIndex, width, height, rotate90, flipH, flipV, (ARVideoView::HorizontalAlignment)hAlign, (ARVideoView::VerticalAlignment)vAlign, (ARVideoView::ScalingMode)scalingMode, viewport));
}

bool arwDrawVideo(const int videoSourceIndex)
{
    if (!gARTK)return false;
    
    return (gARTK->drawVideo(videoSourceIndex));
}

bool arwDrawVideoFinal(const int videoSourceIndex)
{
    if (!gARTK) return false;
    
    return (gARTK->drawVideoFinal(videoSourceIndex));
}

// ----------------------------------------------------------------------------------------------------
#pragma mark  Tracking configuration
// ----------------------------------------------------------------------------------------------------

void arwSetTrackerOptionBool(int option, bool value, bool lowRes)
{
    ARController *gARTK2 = NULL;
    
    if (!lowRes) {
        gARTK2 = gARTK;
    } else {
        gARTK2 = gARTKLowRes;
    }
    
    if (!gARTK2) return;
    
    if (option == ARW_TRACKER_OPTION_NFT_MULTIMODE) {
#if HAVE_NFT
        gARTK2->getNFTTracker()->setNFTMultiMode(value);
#else
        return;
#endif
    } else if (option == ARW_TRACKER_OPTION_SQUARE_DEBUG_MODE) {
        gARTK2->getSquareTracker()->setDebugMode(value);
    } else if (option == ARW_TRACKER_OPTION_2D_CORNER_REFINEMENT) {
        gARTK2->getSquareTracker()->setCornerRefinementMode(value);
    }
}

void arwSetTrackerOptionInt(int option, int value, bool lowRes)
{
    ARController *gARTK2 = NULL;
    
    if (!lowRes) {
        gARTK2 = gARTK;
    } else {
        gARTK2 = gARTKLowRes;
    }
    
    if (!gARTK2) return;
    
    if (option == ARW_TRACKER_OPTION_SQUARE_THRESHOLD) {
        if (value < 0 || value > 255) return;
        gARTK2->getSquareTracker()->setThreshold(value);
    } else if (option == ARW_TRACKER_OPTION_SQUARE_THRESHOLD_MODE) {
        gARTK2->getSquareTracker()->setThresholdMode(value);
    } else if (option == ARW_TRACKER_OPTION_SQUARE_LABELING_MODE) {
        gARTK2->getSquareTracker()->setLabelingMode(value);
    } else if (option == ARW_TRACKER_OPTION_SQUARE_PATTERN_DETECTION_MODE) {
        gARTK2->getSquareTracker()->setPatternDetectionMode(value);
    } else if (option == ARW_TRACKER_OPTION_SQUARE_MATRIX_CODE_TYPE) {
        gARTK2->getSquareTracker()->setMatrixCodeType(value);
    } else if (option == ARW_TRACKER_OPTION_SQUARE_IMAGE_PROC_MODE) {
        gARTK2->getSquareTracker()->setImageProcMode(value);
    } else if (option == ARW_TRACKER_OPTION_SQUARE_PATTERN_SIZE) {
        gARTK2->getSquareTracker()->setPatternSize(value);
    } else if (option == ARW_TRACKER_OPTION_SQUARE_PATTERN_COUNT_MAX) {
        gARTK2->getSquareTracker()->setPatternCountMax(value);
    } else if (option == ARW_TRACKER_OPTION_2D_TRACKER_FEATURE_TYPE) {
#if HAVE_2D
        if (value < 0 || value > 3) return;
        gARTK2->get2dTracker()->setDetectorType(value);
#else
        return;
#endif
    } else if (option == ARW_TRACKER_OPTION_SQUARE_PATTERN_COUNT_MAX) {
        gARTK2->getSquareTracker()->setPatternCountMax(value);
    }
}

void arwSetTrackerOptionFloat(int option, float value, bool lowRes)
{
    ARController *gARTK2 = NULL;
    
    if (!lowRes) {
        gARTK2 = gARTK;
    } else {
        gARTK2 = gARTKLowRes;
    }
    
    if (!gARTK2) return;
    
    if (option == ARW_TRACKER_OPTION_SQUARE_BORDER_SIZE) {
        if (value <= 0.0f || value >= 0.5f) return;
        gARTK2->getSquareTracker()->setPattRatio(1.0f - 2.0f*value); // Convert from border size to pattern ratio.
    }
}

bool arwGetTrackerOptionBool(int option, bool lowRes)
{
    ARController *gARTK2 = NULL;
    
    if (!lowRes) {
        gARTK2 = gARTK;
    } else {
        gARTK2 = gARTKLowRes;
    }
    
    if (!gARTK2) return false;
    
    if (option == ARW_TRACKER_OPTION_NFT_MULTIMODE) {
#if HAVE_NFT
        return  gARTK2->getNFTTracker()->NFTMultiMode();
#else
        return false;
#endif
    } else if (option == ARW_TRACKER_OPTION_SQUARE_DEBUG_MODE) {
        return gARTK2->getSquareTracker()->debugMode();
    }
    return false;
}

int arwGetTrackerOptionInt(int option, bool lowRes)
{
    ARController *gARTK2 = NULL;
    
    if (!lowRes) {
        gARTK2 = gARTK;
    } else {
        gARTK2 = gARTKLowRes;
    }
    
    if (!gARTK2) return (INT_MAX);
    
    if (option == ARW_TRACKER_OPTION_SQUARE_THRESHOLD) {
        return gARTK2->getSquareTracker()->threshold();
    } else if (option == ARW_TRACKER_OPTION_SQUARE_THRESHOLD_MODE) {
        return gARTK2->getSquareTracker()->thresholdMode();
    } else if (option == ARW_TRACKER_OPTION_SQUARE_LABELING_MODE) {
        return gARTK2->getSquareTracker()->labelingMode();
    } else if (option == ARW_TRACKER_OPTION_SQUARE_PATTERN_DETECTION_MODE) {
        return gARTK2->getSquareTracker()->patternDetectionMode();
    } else if (option == ARW_TRACKER_OPTION_SQUARE_MATRIX_CODE_TYPE) {
        return gARTK2->getSquareTracker()->matrixCodeType();
    } else if (option == ARW_TRACKER_OPTION_SQUARE_IMAGE_PROC_MODE) {
        return gARTK2->getSquareTracker()->imageProcMode();
    } else if (option == ARW_TRACKER_OPTION_SQUARE_PATTERN_SIZE) {
        return gARTK2->getSquareTracker()->patternSize();
    } else if (option == ARW_TRACKER_OPTION_SQUARE_PATTERN_COUNT_MAX) {
        return gARTK2->getSquareTracker()->patternCountMax();
    }
    return (INT_MAX);
}

float arwGetTrackerOptionFloat(int option, bool lowRes)
{
    ARController *gARTK2 = NULL;
    
    if (!lowRes) {
        gARTK2 = gARTK;
    } else {
        gARTK2 = gARTKLowRes;
    }
    
    if (!gARTK2) return (NAN);
    
    if (option == ARW_TRACKER_OPTION_SQUARE_BORDER_SIZE) {
        float value = gARTK2->getSquareTracker()->pattRatio();
        if (value > 0.0f && value < 1.0f) return (1.0f - value)/2.0f; // Convert from pattern ratio to border size.
    }
    return (NAN);
}

// ----------------------------------------------------------------------------------------------------
#pragma mark  Trackable management
// ---------------------------------------------------------------------------------------------


void AddOldStyleMarkersToARToolKit(int threshold, int thresholdMode, int* myGFMarkerID, int *myStepMarkerID, int* myMarkerIDs, int *myLeftBulkheadMarkerID, int *myRightBulkheadMarkerID, int *myDoorHingeRightMarkerID, int *myDoorFrameRightMarkerID, int *myDoorHingeLeftMarkerID, int *myDoorFrameLeftMarkerID, int *myObstruct1MarkerID, int *myObstruct2MarkerID, int *myObstruct3MarkerID, int *myObstruct4MarkerID, int *myWall1MarkerID, int *myWall2MarkerID, int *myWall3MarkerID, int *myWall4MarkerID) {
    
    //!!!IMPORTANT NOTE:
    //In arConfig.h:
    //#define   AR_LABELING_32_BIT                  1     // 0 = 16 bits per label, 1 = 32 bits per label.
    //#  define AR_LABELING_WORK_SIZE      1024*32*64

    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_PATTERN_DETECTION_MODE, AR_MATRIX_CODE_DETECTION, false);
    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_PATTERN_DETECTION_MODE, AR_MATRIX_CODE_DETECTION, true);

    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_MATRIX_CODE_TYPE, AR_MATRIX_CODE_4x4, false);
    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_MATRIX_CODE_TYPE, AR_MATRIX_CODE_4x4, true);

    //ARToolKitFunctions.Instance.arwSetMarkerExtractionMode(AR_USE_TRACKING_HISTORY_V2); //This doesn't work in ARToolKitX
    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_THRESHOLD, (int)(threshold * 255.0 / 100.0), false);
    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_THRESHOLD, (int)(threshold * 255.0 / 100.0), true);
    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_THRESHOLD_MODE, thresholdMode, false);
    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_THRESHOLD_MODE, thresholdMode, true);
    
    arwSetTrackerOptionBool(ARW_TRACKER_OPTION_2D_CORNER_REFINEMENT, true, false);
    arwSetTrackerOptionBool(ARW_TRACKER_OPTION_2D_CORNER_REFINEMENT, false, true);

    for (int i = 1; i <= 100; i++) {
        std::string number;
        if (i < 100) {
            char buffer[3];
            std::snprintf(buffer, sizeof(buffer), "%02d", i);
            number = buffer;
        } else {
            char buffer[4];
            std::snprintf(buffer, sizeof(buffer), "%03d", i);
            number = buffer;
        }
        std::string code = "multi;data/MarkerLarge" + number + ".dat";
        int markerId = arwAddTrackable(code.c_str(), false, (i == 1) ? 0 : -1);
        myMarkerIDs[i - 1] = markerId;
        arwSetTrackableOptionInt(markerId, ARW_TRACKABLE_OPTION_MULTI_MIN_SUBMARKERS, 2, false);
        arwSetTrackableOptionInt(markerId, ARW_TRACKABLE_OPTION_MULTI_MIN_SUBMARKERS, 2, true);
        arwSetTrackableOptionFloat(markerId, ARW_TRACKABLE_OPTION_MULTI_MIN_CONF_MATRIX, 1.0f, false);
        arwSetTrackableOptionFloat(markerId, ARW_TRACKABLE_OPTION_MULTI_MIN_CONF_MATRIX, 1.0f, true);
        arwSetTrackableOptionBool(markerId, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
        arwSetTrackableOptionBool(markerId, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
        arwSetTrackableOptionFloat(markerId, ARW_TRACKABLE_OPTION_MULTI_MIN_INLIER_PROB, 1.0f, false);
        arwSetTrackableOptionFloat(markerId, ARW_TRACKABLE_OPTION_MULTI_MIN_INLIER_PROB, 1.0f, true);
    }

    *myGFMarkerID = arwAddTrackable("multi;data/GFMarker.dat", false, -1);
    arwSetTrackableOptionInt(*myGFMarkerID, ARW_TRACKABLE_OPTION_MULTI_MIN_SUBMARKERS, 4, false);
    arwSetTrackableOptionInt(*myGFMarkerID, ARW_TRACKABLE_OPTION_MULTI_MIN_SUBMARKERS, 4, true);
    arwSetTrackableOptionFloat(*myGFMarkerID, ARW_TRACKABLE_OPTION_MULTI_MIN_CONF_MATRIX, 1.0f, false);
    arwSetTrackableOptionFloat(*myGFMarkerID, ARW_TRACKABLE_OPTION_MULTI_MIN_CONF_MATRIX, 1.0f, true);
    arwSetTrackableOptionBool(*myGFMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myGFMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    arwSetTrackableOptionFloat(*myGFMarkerID, ARW_TRACKABLE_OPTION_MULTI_MIN_INLIER_PROB, 1.0f, false);
    arwSetTrackableOptionFloat(*myGFMarkerID, ARW_TRACKABLE_OPTION_MULTI_MIN_INLIER_PROB, 1.0f, true);

    *myStepMarkerID = arwAddTrackable("multi;data/StepMarker.dat", false, -1);
    arwSetTrackableOptionInt(*myStepMarkerID, ARW_TRACKABLE_OPTION_MULTI_MIN_SUBMARKERS, 4, false);
    arwSetTrackableOptionInt(*myStepMarkerID, ARW_TRACKABLE_OPTION_MULTI_MIN_SUBMARKERS, 4, true);
    arwSetTrackableOptionFloat(*myStepMarkerID, ARW_TRACKABLE_OPTION_MULTI_MIN_CONF_MATRIX, 1.0f, false);
    arwSetTrackableOptionFloat(*myStepMarkerID, ARW_TRACKABLE_OPTION_MULTI_MIN_CONF_MATRIX, 1.0f, true);
    arwSetTrackableOptionBool(*myStepMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myStepMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    arwSetTrackableOptionFloat(*myStepMarkerID, ARW_TRACKABLE_OPTION_MULTI_MIN_INLIER_PROB, 1.0f, false);
    arwSetTrackableOptionFloat(*myStepMarkerID, ARW_TRACKABLE_OPTION_MULTI_MIN_INLIER_PROB, 1.0f, true);

    *myLeftBulkheadMarkerID = arwAddTrackable("single_barcode;249;80;", false, -1);
    arwSetTrackableOptionBool(*myLeftBulkheadMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myLeftBulkheadMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myRightBulkheadMarkerID = arwAddTrackable("single_barcode;250;80;", false, -1);
    arwSetTrackableOptionBool(*myRightBulkheadMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myRightBulkheadMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);

    *myDoorHingeRightMarkerID = arwAddTrackable("single_barcode;251;80;", false, -1);
    arwSetTrackableOptionBool(*myDoorHingeRightMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myDoorHingeRightMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myDoorFrameRightMarkerID = arwAddTrackable("single_barcode;252;80;", false, -1);
    arwSetTrackableOptionBool(*myDoorHingeRightMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myDoorHingeRightMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myDoorHingeLeftMarkerID = arwAddTrackable("single_barcode;253;80;", false, -1);
    arwSetTrackableOptionBool(*myDoorHingeLeftMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myDoorHingeLeftMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myDoorFrameLeftMarkerID = arwAddTrackable("single_barcode;254;80;", false, -1);
    arwSetTrackableOptionBool(*myDoorFrameLeftMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myDoorFrameLeftMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);

    *myObstruct1MarkerID = arwAddTrackable("single_barcode;255;80;", false, -1);
    arwSetTrackableOptionBool(*myObstruct1MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myObstruct1MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myObstruct2MarkerID = arwAddTrackable("single_barcode;256;80;", false, -1);
    arwSetTrackableOptionBool(*myObstruct2MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myObstruct2MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myObstruct3MarkerID = arwAddTrackable("single_barcode;257;80;", false, -1);
    arwSetTrackableOptionBool(*myObstruct3MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myObstruct3MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myObstruct4MarkerID = arwAddTrackable("single_barcode;258;80;", false, -1);
    arwSetTrackableOptionBool(*myObstruct4MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myObstruct4MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);

    *myWall1MarkerID = arwAddTrackable("single_barcode;259;80;", false, -1);
    arwSetTrackableOptionBool(*myWall1MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myWall1MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myWall2MarkerID = arwAddTrackable("single_barcode;260;80;", false, -1);
    arwSetTrackableOptionBool(*myWall2MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myWall2MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myWall3MarkerID = arwAddTrackable("single_barcode;261;80;", false, -1);
    arwSetTrackableOptionBool(*myWall3MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myWall3MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myWall4MarkerID = arwAddTrackable("single_barcode;262;80;", false, -1);
    arwSetTrackableOptionBool(*myWall4MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myWall4MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
}

void AddDatumMarkersToARToolKit(int threshold, int thresholdMode, int* myGFMarkerID, int *myStepMarkerID, int* myMarkerIDs, int *myLeftBulkheadMarkerID, int *myRightBulkheadMarkerID, int *myDoorHingeRightMarkerID, int *myDoorFrameRightMarkerID, int *myDoorHingeLeftMarkerID, int *myDoorFrameLeftMarkerID, int *myObstruct1MarkerID, int *myObstruct2MarkerID, int *myObstruct3MarkerID, int *myObstruct4MarkerID, int *myWall1MarkerID, int *myWall2MarkerID, int *myWall3MarkerID, int *myWall4MarkerID)
{

    //!!!IMPORTANT NOTE:
    //In arConfig.h:
    //#define   AR_LABELING_32_BIT                  1     // 0 = 16 bits per label, 1 = 32 bits per label.
    //#  define AR_LABELING_WORK_SIZE      1024*32*64

    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_PATTERN_DETECTION_MODE, AR_MATRIX_CODE_DETECTION, false);
    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_PATTERN_DETECTION_MODE, AR_MATRIX_CODE_DETECTION, true);

    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_MATRIX_CODE_TYPE, AR_MATRIX_CODE_5x5_BCH_22_12_5, false);
    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_MATRIX_CODE_TYPE, AR_MATRIX_CODE_5x5_BCH_22_12_5, true);

    //ARToolKitFunctions.Instance.arwSetMarkerExtractionMode(AR_USE_TRACKING_HISTORY_V2); //This doesn't work in ARToolKitX
    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_THRESHOLD, (int)(threshold * 255.0 / 100.0), false);
    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_THRESHOLD, (int)(threshold * 255.0 / 100.0), true);
    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_THRESHOLD_MODE, thresholdMode, false);
    arwSetTrackerOptionInt(ARW_TRACKER_OPTION_SQUARE_THRESHOLD_MODE, thresholdMode, true);
    
    arwSetTrackerOptionBool(ARW_TRACKER_OPTION_2D_CORNER_REFINEMENT, true, false);
    arwSetTrackerOptionBool(ARW_TRACKER_OPTION_2D_CORNER_REFINEMENT, false, true);

    *myGFMarkerID = arwAddTrackable("single_barcode;0;80;", false, 0);
    arwSetTrackableOptionBool(*myGFMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myGFMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myStepMarkerID = arwAddTrackable("single_barcode;1;80;", false, -1);
    arwSetTrackableOptionBool(*myStepMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myStepMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);

    for (int i = 1; i <= 100; i++) {
        std::string code = "single_barcode;" + std::to_string(i + 1) + ";80";
        int markerId = arwAddTrackable(code.c_str(), false, -1);
        myMarkerIDs[i - 1] = markerId;
        arwSetTrackableOptionBool(markerId, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
        arwSetTrackableOptionBool(markerId, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    }

    *myLeftBulkheadMarkerID = arwAddTrackable("single_barcode;102;80;", false, -1);
    arwSetTrackableOptionBool(*myLeftBulkheadMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myLeftBulkheadMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myRightBulkheadMarkerID = arwAddTrackable("single_barcode;103;80;", false, -1);
    arwSetTrackableOptionBool(*myRightBulkheadMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myRightBulkheadMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);

    *myDoorHingeRightMarkerID = arwAddTrackable("single_barcode;104;80;", false, -1);
    arwSetTrackableOptionBool(*myDoorHingeRightMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myDoorHingeRightMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myDoorFrameRightMarkerID = arwAddTrackable("single_barcode;105;80;", false, -1);
    arwSetTrackableOptionBool(*myDoorFrameRightMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myDoorHingeRightMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myDoorHingeLeftMarkerID = arwAddTrackable("single_barcode;106;80;", false, -1);
    arwSetTrackableOptionBool(*myDoorHingeLeftMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myDoorHingeLeftMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myDoorFrameLeftMarkerID = arwAddTrackable("single_barcode;107;80;", false, -1);
    arwSetTrackableOptionBool(*myDoorFrameLeftMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myDoorFrameLeftMarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);

    *myObstruct1MarkerID = arwAddTrackable("single_barcode;108;80;", false, -1);
    arwSetTrackableOptionBool(*myObstruct1MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myObstruct1MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myObstruct2MarkerID = arwAddTrackable("single_barcode;109;80;", false, -1);
    arwSetTrackableOptionBool(*myObstruct2MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myObstruct2MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myObstruct3MarkerID = arwAddTrackable("single_barcode;110;80;", false, -1);
    arwSetTrackableOptionBool(*myObstruct3MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myObstruct3MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myObstruct4MarkerID = arwAddTrackable("single_barcode;111;80;", false, -1);
    arwSetTrackableOptionBool(*myObstruct4MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myObstruct4MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    
    *myWall1MarkerID = arwAddTrackable("single_barcode;112;80;", false, -1);
    arwSetTrackableOptionBool(*myWall1MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myWall1MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myWall2MarkerID = arwAddTrackable("single_barcode;113;80;", false, -1);
    arwSetTrackableOptionBool(*myWall2MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myWall2MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myWall3MarkerID = arwAddTrackable("single_barcode;114;80;", false, -1);
    arwSetTrackableOptionBool(*myWall3MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myWall3MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
    *myWall4MarkerID = arwAddTrackable("single_barcode;115;80;", false, -1);
    arwSetTrackableOptionBool(*myWall4MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, false);
    arwSetTrackableOptionBool(*myWall4MarkerID, ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION, false, true);
}

int arwAddTrackable(const char *cfg, bool avoidLowRes, int overrideUID)
{
    int n = -1;
    
    if (gARTK) {
        n = gARTK->addTrackable(cfg, overrideUID);
    }
    if (gARTKLowRes) {
        if (!avoidLowRes) gARTKLowRes->addTrackable(cfg, n);
    }
    return n;
}

bool arwGetTrackables(int *count_p, ARWTrackableStatus **statuses_p)
{
    if (!gARTK) return false;
    if (!count_p) return false;
    
    unsigned int trackableCount = gARTK->countTrackables();
    *count_p = (int)trackableCount;
    if (statuses_p) {
        if (trackableCount == 0) *statuses_p = NULL;
        else {
            ARWTrackableStatus *st = (ARWTrackableStatus *)calloc(trackableCount, sizeof(ARWTrackableStatus));
            for (unsigned int i = 0; i < trackableCount; i++) {
                ARTrackable *t = gARTK->getTrackableAtIndex(i);
                if (!t) {
                    st[i].uid = -1;
                } else {
                    st[i].uid = t->UID;
                    st[i].visible = t->visible;
#ifdef ARDOUBLE_IS_FLOAT
                    memcpy(st[i].matrix, t->transformationMatrix, 16*sizeof(float));
                    memcpy(st[i].matrixR, t->transformationMatrixR, 16*sizeof(float));
#else
                    for (int j = 0; j < 16; j++) st[i].matrix[j] = (float)t->transformationMatrix[j];
                    for (int j = 0; j < 16; j++) st[i].matrixR[j] = (float)t->transformationMatrixR[j];
#endif
                }
            }
            *statuses_p = st;
        }
    }
    
    return true;
}

bool arwRemoveTrackable(int trackableUID)
{
    if (!gARTK) return false;
	return gARTK->removeTrackable(trackableUID);
}

int arwRemoveAllTrackables()
{
    if (!gARTK) return 0;
	return gARTK->removeAllTrackables();
}

#if HAVE_2D
bool arwLoad2dTrackableDatabase(const char *databaseFileName)
{
    if (!gARTK) return false;
    return gARTK->load2DTrackerImageDatabase(databaseFileName);
}

bool arwSave2dTrackableDatabase(const char *databaseFileName)
{
    if (!gARTK) return false;
    return gARTK->save2DTrackerImageDatabase(databaseFileName);
}
#endif // HAVE_2D

bool arwQueryTrackableVisibilityAndTransformation(int trackableUID, double matrix[16], double corners[32], int *numCorners, bool lowRes)
{
    ARController *gARTK2 = NULL;
    
    if (!lowRes) {
        gARTK2 = gARTK;
    } else {
        gARTK2 = gARTKLowRes;
    }
    
    ARTrackable *trackable;
    
    if (!gARTK2) return false;
    if (!(trackable = gARTK2->findTrackable(trackableUID))) {
        ARLOGe("arwQueryTrackableVisibilityAndTransformation(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return false;
    }
    for (int i = 0; i < 16; i++) matrix[i] = (double)trackable->transformationMatrix[i];
    if (trackable->visible && (trackable->type == ARTrackable::SINGLE || trackable->type == ARTrackable::MULTI || trackable->type == ARTrackable::MULTI_AUTO)) {
        *numCorners = (int)trackable->imagePoints.size();
        if (*numCorners > 16) *numCorners = 16;
        for (int i = 0; i < *numCorners; i++) {
            corners[i * 2] = trackable->imagePoints.at(i).x;
            corners[i * 2 + 1] = trackable->imagePoints.at(i).y;
        }
    }
    return trackable->visible;
}

bool arwQueryTrackableMapperTransformation(int gMapUID, int trackableUID, double *matrix) {
    ARTrackableMultiSquareAuto *t = reinterpret_cast<ARTrackableMultiSquareAuto *>(gARTK->findTrackable(gMapUID));
    if (t) {
        ARMultiMarkerInfoT *map = t->copyMultiConfig();
        if (map) {
            for (int n = 0; n < map->marker_num; n++) {
                if (map->marker[n].patt_id == trackableUID) {
                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 4; j++) {
                            matrix[i + j * 4] = (double)map->marker[n].trans[i][j];
                            matrix[3 + j * 4] = 0;
                        }
                    }
                    return true;
                }
            }
        }
    }
    return false;
}

void arwListTrackables(int gMapUID) {
    ARTrackableMultiSquareAuto *t = reinterpret_cast<ARTrackableMultiSquareAuto *>(gARTK->findTrackable(gMapUID));
    if (t) {
        ARMultiMarkerInfoT *map = t->copyMultiConfig();
        if (map) {
            for (int n = 0; n < map->marker_num; n++) {
                ARLOGd("Found trackable with UID %d\n", map->marker[n].patt_id);
            }
        }
    }
}

int arwResetMapperTrackable(int gMapUID, const char* cfg) {
    gARTK->removeTrackable(gMapUID);
    return gARTK->addTrackable(cfg);
}

void arwSetMappedMarkersVisible(int nMarkers, double* markerTrans, int* uids, double* corners) {
    std::vector<arx_mapper::Marker> markers;
    for (int n = 0; n < nMarkers; n++) {
        arx_mapper::Marker marker;
        marker.uid = uids[n];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                marker.trans[i][j] = (ARdouble)(markerTrans[n * 12 + i * 4 + j]);
            }
        }
        for (int i = 0; i < 8; i++) {
            marker.corners[i] = (ARdouble)(corners[n * 8 + i]);
        }
        markers.push_back(marker);
    }

    //Set markers visible or invisible based on the uids given
    for (int i = 0; i < gARTK->countTrackables(); i++) {
        ARTrackable* it = gARTK->getTrackableAtIndex(i);

        it->visible = false;
        for (std::vector<arx_mapper::Marker>::iterator mt = markers.begin(); mt != markers.end(); ++mt) {
            arx_mapper::Marker m = (arx_mapper::Marker)(*mt);

            if (it->type == ARTrackable::SINGLE) {
                ARTrackableSquare* marker = reinterpret_cast<ARTrackableSquare*>(it);
                if (marker->patt_id == m.uid) {
                    it->visible = true;
                }
            } else if (it->type == ARTrackable::MULTI) {
                ARTrackableMultiSquare* marker = reinterpret_cast<ARTrackableMultiSquare*>(it);
                if (marker->config->marker[0].patt_id == m.uid) {
                    it->visible = true;
                }
            }

            if (it->visible) {
                for (int j = 0; j < 3; j++) {
                    for (int k = 0; k < 4; k++) {
                        it->SetTrans(j, k, m.trans[j][k]);
                    }
                }
                it->update();
                it->imagePoints.clear();
                for (int j = 0; j < 8; j = j + 2) {
                    it->imagePoints.push_back(cv::Point2f(m.corners[j], m.corners[j + 1]));
                }
                break;
            }
        }
    }

}

void arwAddMappedMarkers(int gMapUID, int GFMarkerID, int nMarkers, double* markerTrans, int* uids, double* corners) {

    std::vector<arx_mapper::Marker> markers;
    for (int n = 0; n < nMarkers; n++) {
        arx_mapper::Marker marker;
        marker.uid = uids[n];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                marker.trans[i][j] = (ARdouble)(markerTrans[n * 12 + i * 4 + j]);
            }
        }
        for (int i = 0; i < 8; i++) {
            marker.corners[i] = (ARdouble)(corners[n * 8 + i]);
        }
        markers.push_back(marker);
    }

    ARTrackableMultiSquare* GFMarker = reinterpret_cast<ARTrackableMultiSquare*>(gARTK->findTrackable(GFMarkerID));

    ARTrackableMultiSquareAuto* t = reinterpret_cast<ARTrackableMultiSquareAuto*>(gARTK->findTrackable(gMapUID));
    if (t) {

        if (t->m_MultiConfig->marker_num == 0) {
            GFMarker->visible = false;
            for (std::vector<arx_mapper::Marker>::iterator mt = markers.begin(); mt != markers.end(); ++mt) {
                arx_mapper::Marker m = (arx_mapper::Marker)(*mt);
                if (m.uid == t->m_OriginMarkerUid) {
                    GFMarker->visible = true;
                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 4; j++) {
                            GFMarker->SetTrans(i, j, m.trans[i][j]);
                        }
                    }
                    break;
                }
            }
            if (!GFMarker->visible) return;

            t->initialiseWithMultiSquareTrackable(GFMarker);

            //Only add markers which belong to the ground floor board
            std::vector<arx_mapper::Marker> newmarkers;
            for (std::vector<arx_mapper::Marker>::iterator mt = markers.begin(); mt != markers.end(); ++mt) {
                arx_mapper::Marker m = (arx_mapper::Marker)(*mt);
                for (int i = 0; i < GFMarker->config->marker_num; i++) {
                    if (GFMarker->config->marker[i].patt_id == m.uid) {
                        newmarkers.push_back(m);
                        break;
                    }
                }
            }
            markers.clear();
            for (std::vector<arx_mapper::Marker>::iterator mt = newmarkers.begin(); mt != newmarkers.end(); ++mt) {
                arx_mapper::Marker m = (arx_mapper::Marker)(*mt);
                markers.push_back(m);
            }
        }

        bool success = t->updateWithDetectedMarkers2(markers, gARTK->getAR3DHandle());
        if (success && t->visible) {
            success = t->updateMapperWithMarkers(markers);
        }
        else {
            // Mapper not visible
            t->visible = false;
        }

    }
}

bool arwQueryTrackableVisibilityAndTransformationStereo(int trackableUID, float matrixL[16], float matrixR[16])
{
    ARTrackable *trackable;
    
    if (!gARTK) return false;
	if (!(trackable = gARTK->findTrackable(trackableUID))) {
        ARLOGe("arwQueryTrackableVisibilityAndTransformationStereo(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return false;
    }
    for (int i = 0; i < 16; i++) matrixL[i] = (float)trackable->transformationMatrix[i];
    for (int i = 0; i < 16; i++) matrixR[i] = (float)trackable->transformationMatrixR[i];
    return trackable->visible;
}

// ----------------------------------------------------------------------------------------------------
#pragma mark  Trackable patterns
// ---------------------------------------------------------------------------------------------

int arwGetTrackablePatternCount(int trackableUID)
{
    ARTrackable *trackable;
    
    if (!gARTK) return 0;
	if (!(trackable = gARTK->findTrackable(trackableUID))) {
        ARLOGe("arwGetTrackablePatternCount(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return 0;
    }
    return trackable->patternCount;
}

bool arwGetTrackablePatternConfig(int trackableUID, int patternID, double matrix[16], double *width, double *height, int *imageSizeX, int *imageSizeY, int *barcodeID)
{
    ARTrackable *trackable;
    ARPattern *p;
    
    if (!gARTK) return false;
	if (!(trackable = gARTK->findTrackable(trackableUID))) {
        ARLOGe("arwGetTrackablePatternConfig(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return false;
    }
    
    if (!(p = trackable->getPattern(patternID))) {
        ARLOGe("arwGetTrackablePatternConfig(): Trackable with UID %d has no pattern with ID %d.\n", trackableUID, patternID);
        return false;
    }

    if (matrix) {
        for (int i = 0; i < 16; i++) matrix[i] = (double)p->m_matrix[i];
    }
    if (width) *width = (double)p->m_width;
    if (height) *height = (double)p->m_height;
    if (imageSizeX) *imageSizeX = p->m_imageSizeX;
    if (imageSizeY) *imageSizeY = p->m_imageSizeY;
    
    if (trackable->type == ARTrackable::MULTI) *barcodeID = ((ARTrackableMultiSquare *)trackable)->config->marker[patternID].patt_id;
    
    return true;
}

bool arwGetTrackablePatternImage(int trackableUID, int patternID, uint32_t *buffer)
{
    ARTrackable *trackable;
    ARPattern *p;
    
    if (!gARTK) return false;
	if (!(trackable = gARTK->findTrackable(trackableUID))) {
        ARLOGe("arwGetTrackablePatternImage(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return false;
    }
    
    if (!(p = trackable->getPattern(patternID))) {
        ARLOGe("arwGetTrackablePatternImage(): Trackable with UID %d has no pattern with ID %d.\n", trackableUID, patternID);
        return false;
    }

    if (!p->m_image) {
        return false;
    }
    
    memcpy(buffer, p->m_image, sizeof(uint32_t) * p->m_imageSizeX * p->m_imageSizeY);
    return true;

}

// ----------------------------------------------------------------------------------------------------
#pragma mark  Trackable options
// ---------------------------------------------------------------------------------------------

bool arwGetTrackableOptionBool(int trackableUID, int option, bool lowRes)
{
    ARController *gARTK2 = NULL;
    
    if (!lowRes) {
        gARTK2 = gARTK;
    } else {
        gARTK2 = gARTKLowRes;
    }
    
    ARTrackable *trackable;
    
    if (!gARTK2) return false;
	if (!(trackable = gARTK2->findTrackable(trackableUID))) {
        ARLOGe("arwGetTrackableOptionBool(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return false;
    }
    
    switch (option) {
        case ARW_TRACKABLE_OPTION_FILTERED:
            return(trackable->isFiltered());
            break;
        case ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION:
            if (trackable->type == ARTrackable::SINGLE) return (((ARTrackableSquare *)trackable)->useContPoseEstimation);
            break;
        default:
            ARLOGe("arwGetTrackableOptionBool(): Unrecognised option %d.\n", option);
            break;
    }
    return(false);
}

void arwSetTrackableOptionBool(int trackableUID, int option, bool value, bool lowRes)
{
    ARController *gARTK2 = NULL;
    
    if (!lowRes) {
        gARTK2 = gARTK;
    } else {
        gARTK2 = gARTKLowRes;
    }
    
    ARTrackable *trackable;
    
    if (!gARTK2) return;
	if (!(trackable = gARTK2->findTrackable(trackableUID))) {
        ARLOGe("arwSetTrackableOptionBool(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return;
    }

    switch (option) {
        case ARW_TRACKABLE_OPTION_FILTERED:
            trackable->setFiltered(value);
            break;
        case ARW_TRACKABLE_OPTION_SQUARE_USE_CONT_POSE_ESTIMATION:
            if (trackable->type == ARTrackable::SINGLE) ((ARTrackableSquare *)trackable)->useContPoseEstimation = value;
            break;
        default:
            ARLOGe("arwSetTrackableOptionBool(): Unrecognised option %d.\n", option);
            break;
    }
}

int arwGetTrackableOptionInt(int trackableUID, int option, bool lowRes)
{
    ARController *gARTK2 = NULL;
    
    if (!lowRes) {
        gARTK2 = gARTK;
    } else {
        gARTK2 = gARTKLowRes;
    }
    
    ARTrackable *trackable;
    
    if (!gARTK2) return INT_MIN;
	if (!(trackable = gARTK2->findTrackable(trackableUID))) {
        ARLOGe("arwGetTrackableOptionBool(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return (INT_MIN);
    }
    
    switch (option) {
        case ARW_TRACKABLE_OPTION_MULTI_MIN_SUBMARKERS:
            if (trackable->type == ARTrackable::MULTI) return ((ARTrackableMultiSquare *)trackable)->config->min_submarker;
            break;
        default:
            ARLOGe("arwGetTrackableOptionInt(): Unrecognised option %d.\n", option);
            break;
    }
    return (INT_MIN);
}

void arwSetTrackableOptionInt(int trackableUID, int option, int value, bool lowRes)
{
    ARController *gARTK2 = NULL;
    
    if (!lowRes) {
        gARTK2 = gARTK;
    } else {
        gARTK2 = gARTKLowRes;
    }
    
    ARTrackable *trackable;
    
    if (!gARTK2) return;
	if (!(trackable = gARTK2->findTrackable(trackableUID))) {
        ARLOGe("arwSetTrackableOptionInt(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return;
    }

    switch (option) {
        case ARW_TRACKABLE_OPTION_MULTI_MIN_SUBMARKERS:
            if (trackable->type == ARTrackable::MULTI) ((ARTrackableMultiSquare *)trackable)->config->min_submarker = value;
            break;
        default:
            ARLOGe("arwSetTrackableOptionInt(): Unrecognised option %d.\n", option);
            break;
    }
}

float arwGetTrackableOptionFloat(int trackableUID, int option, bool lowRes)
{
    ARController *gARTK2 = NULL;
    
    if (!lowRes) {
        gARTK2 = gARTK;
    } else {
        gARTK2 = gARTKLowRes;
    }
    
    ARTrackable *trackable;
    
    if (!gARTK2) return (NAN);
	if (!(trackable = gARTK2->findTrackable(trackableUID))) {
        ARLOGe("arwGetTrackableOptionBool(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return (NAN);
    }
    
    switch (option) {
        case ARW_TRACKABLE_OPTION_FILTER_SAMPLE_RATE:
            return ((float)trackable->filterSampleRate());
            break;
        case ARW_TRACKABLE_OPTION_FILTER_CUTOFF_FREQ:
            return ((float)trackable->filterCutoffFrequency());
            break;
        case ARW_TRACKABLE_OPTION_SQUARE_CONFIDENCE:
            if (trackable->type == ARTrackable::SINGLE) return ((float)((ARTrackableSquare *)trackable)->getConfidence());
            else return (NAN);
            break;
        case ARW_TRACKABLE_OPTION_SQUARE_CONFIDENCE_CUTOFF:
            if (trackable->type == ARTrackable::SINGLE) return ((float)((ARTrackableSquare *)trackable)->getConfidenceCutoff());
            else return (NAN);
            break;
        case ARW_TRACKABLE_OPTION_NFT_SCALE:
#if HAVE_NFT
            if (trackable->type == ARTrackable::NFT) return ((float)((ARTrackableNFT *)trackable)->NFTScale());
            else return (NAN);
#else
            return (NAN);
#endif
            break;
        case ARW_TRACKABLE_OPTION_MULTI_MIN_CONF_MATRIX:
            if (trackable->type == ARTrackable::MULTI) return (float)((ARTrackableMultiSquare *)trackable)->config->cfMatrixCutoff;
            else return (NAN);
            break;
        case ARW_TRACKABLE_OPTION_MULTI_MIN_CONF_PATTERN:
            if (trackable->type == ARTrackable::MULTI) return (float)((ARTrackableMultiSquare *)trackable)->config->cfPattCutoff;
            else return (NAN);
            break;
        case ARW_TRACKABLE_OPTION_MULTI_MIN_INLIER_PROB:
            if (trackable->type == ARTrackable::MULTI) return (float)((ARTrackableMultiSquare *)trackable)->config->minInlierProb;
            else return (NAN);
            break;
        default:
            ARLOGe("arwGetTrackableOptionFloat(): Unrecognised option %d.\n", option);
            break;
    }
    return (NAN);
}

void arwSetTrackableOptionFloat(int trackableUID, int option, float value, bool lowRes)
{
    ARController *gARTK2 = NULL;
    
    if (!lowRes) {
        gARTK2 = gARTK;
    } else {
        gARTK2 = gARTKLowRes;
    }
    
    ARTrackable *trackable;
    
    if (!gARTK2) return;
	if (!(trackable = gARTK2->findTrackable(trackableUID))) {
        ARLOGe("arwSetTrackableOptionFloat(): Couldn't locate trackable with UID %d.\n", trackableUID);
        return;
    }

    switch (option) {
        case ARW_TRACKABLE_OPTION_FILTER_SAMPLE_RATE:
            trackable->setFilterSampleRate(value);
            break;
        case ARW_TRACKABLE_OPTION_FILTER_CUTOFF_FREQ:
            trackable->setFilterCutoffFrequency(value);
            break;
        case ARW_TRACKABLE_OPTION_SQUARE_CONFIDENCE_CUTOFF:
            if (trackable->type == ARTrackable::SINGLE) ((ARTrackableSquare *)trackable)->setConfidenceCutoff(value);
            break;
        case ARW_TRACKABLE_OPTION_NFT_SCALE:
#if HAVE_NFT
            if (trackable->type == ARTrackable::NFT) ((ARTrackableNFT *)trackable)->setNFTScale(value);
#endif
            break;
        case ARW_TRACKABLE_OPTION_MULTI_MIN_CONF_MATRIX:
            if (trackable->type == ARTrackable::MULTI) ((ARTrackableMultiSquare *)trackable)->config->cfMatrixCutoff = value;
            break;
        case ARW_TRACKABLE_OPTION_MULTI_MIN_CONF_PATTERN:
            if (trackable->type == ARTrackable::MULTI) ((ARTrackableMultiSquare *)trackable)->config->cfPattCutoff = value;
            break;
        case ARW_TRACKABLE_OPTION_MULTI_MIN_INLIER_PROB:
        if (trackable->type == ARTrackable::MULTI) ((ARTrackableMultiSquare*)trackable)->config->minInlierProb = value;
        if (trackable->type == ARTrackable::MULTI_AUTO) ((ARTrackableMultiSquareAuto*)trackable)->m_MultiConfig->minInlierProb = value;
            break;
        default:
            ARLOGe("arwSetTrackableOptionFloat(): Unrecognised option %d.\n", option);
            break;
    }
}

// ----------------------------------------------------------------------------------------------------
#pragma mark  Utility
// ----------------------------------------------------------------------------------------------------
bool arwLoadOpticalParams(const char *optical_param_name, const char *optical_param_buff, const int optical_param_buffLen, const float projectionNearPlane, const float projectionFarPlane, float *fovy_p, float *aspect_p, float m[16], float p[16])
{
    if (!gARTK) return false;
    
#ifdef ARDOUBLE_IS_FLOAT
    return gARTK->loadOpticalParams(optical_param_name, optical_param_buff, optical_param_buffLen, projectionNearPlane, projectionFarPlane, fovy_p, aspect_p, m, p);
#else
    ARdouble fovy, aspect, m0[16], p0[16];
	if (!gARTK->loadOpticalParams(optical_param_name, optical_param_buff, optical_param_buffLen, projectionNearPlane, projectionFarPlane, &fovy, &aspect, m0, (p ? p0 : NULL))) {
        return false;
    }
    *fovy_p = (float)fovy;
    *aspect_p = (float)aspect;
    for (int i = 0; i < 16; i++) m[i] = (float)m0[i];
    if (p) for (int i = 0; i < 16; i++) p[i] = (float)p0[i];
    return true;
#endif
}

// ----------------------------------------------------------------------------------------------------
#pragma mark  Java API
// ----------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
// Java API
// 
// The following functions provide a JNI compatible wrapper around the first set of 
// exported functions.
// --------------------------------------------------------------------------------------
#if ARX_TARGET_PLATFORM_ANDROID

// Utility function to create a Java float array from a C float array
jfloatArray glArrayToJava(JNIEnv *env, ARdouble *arr, int len) {
	jfloatArray result = NULL;
	if ((result = env->NewFloatArray(len))) env->SetFloatArrayRegion(result, 0, len, arr);
	return result;
}

extern "C" {
    JNIEXPORT void JNICALL JNIFUNCTION(arwSetLogLevel(JNIEnv * env, jobject obj, jint logLevel));
	JNIEXPORT jstring JNICALL JNIFUNCTION(arwGetARToolKitVersion(JNIEnv *env, jobject obj));
    JNIEXPORT jint JNICALL JNIFUNCTION(arwGetError(JNIEnv *env, jobject obj));
	JNIEXPORT jboolean JNICALL JNIFUNCTION(arwInitialiseAR(JNIEnv *env, jobject obj));
	JNIEXPORT jboolean JNICALL JNIFUNCTION(arwChangeToResourcesDir(JNIEnv *env, jobject obj, jstring resourcesDirectoryPath));
	JNIEXPORT jboolean JNICALL JNIFUNCTION(arwShutdownAR(JNIEnv *env, jobject obj));
	JNIEXPORT jboolean JNICALL JNIFUNCTION(arwStartRunning(JNIEnv *env, jobject obj, jstring vconf, jstring cparaName));
	JNIEXPORT jboolean JNICALL JNIFUNCTION(arwStartRunningStereo(JNIEnv *env, jobject obj, jstring vconfL, jstring cparaNameL, jstring vconfR, jstring cparaNameR, jstring transL2RName));
	JNIEXPORT jboolean JNICALL JNIFUNCTION(arwIsRunning(JNIEnv *env, jobject obj));
    JNIEXPORT jboolean JNICALL JNIFUNCTION(arwIsInited(JNIEnv *env, jobject obj));
	JNIEXPORT jboolean JNICALL JNIFUNCTION(arwStopRunning(JNIEnv *env, jobject obj));
	JNIEXPORT jfloatArray JNICALL JNIFUNCTION(arwGetProjectionMatrix(JNIEnv *env, jobject obj, jfloat nearPlane, jfloat farPlane));
	JNIEXPORT jboolean JNICALL JNIFUNCTION(arwGetProjectionMatrixStereo(JNIEnv *env, jobject obj, jfloat nearPlane, jfloat farPlane, jfloatArray projL, jfloatArray projR));
    JNIEXPORT jboolean JNICALL JNIFUNCTION(arwGetVideoParams(JNIEnv *env, jobject obj, jintArray width, jintArray height, jintArray pixelSize, jobjectArray pixelFormatStringBuffer));
    JNIEXPORT jboolean JNICALL JNIFUNCTION(arwGetVideoParamsStereo(JNIEnv *env, jobject obj, jintArray widthL, jintArray heightL, jintArray pixelSizeL, jobjectArray pixelFormatStringL, jintArray widthR, jintArray heightR, jintArray pixelSizeR, jobjectArray  pixelFormatStringR));
	JNIEXPORT jboolean JNICALL JNIFUNCTION(arwCapture(JNIEnv *env, jobject obj));
	JNIEXPORT jboolean JNICALL JNIFUNCTION(arwUpdateAR(JNIEnv *env, jobject obj));
    JNIEXPORT jboolean JNICALL JNIFUNCTION(arwUpdateTexture32(JNIEnv *env, jobject obj, jbyteArray pinArray));
    JNIEXPORT jboolean JNICALL JNIFUNCTION(arwUpdateTextureStereo32(JNIEnv *env, jobject obj, jbyteArray pinArrayL, jbyteArray pinArrayR));
    JNIEXPORT jboolean JNICALL JNIFUNCTION(arwDrawVideoInit(JNIEnv *env, jobject obj, jint videoSourceIndex));
    JNIEXPORT jboolean JNICALL JNIFUNCTION(arwDrawVideoSettings(JNIEnv *env, jobject obj, jint videoSourceIndex, jint width, jint height, jboolean rotate90, jboolean flipH, jboolean flipV, jint hAlign, jint vAlign, jint scalingMode, jintArray viewport));
    JNIEXPORT jboolean JNICALL JNIFUNCTION(arwDrawVideo(JNIEnv *env, jobject obj, jint videoSourceIndex));
    JNIEXPORT jboolean JNICALL JNIFUNCTION(arwDrawVideoFinal(JNIEnv *env, jobject obj, jint videoSourceIndex));
	JNIEXPORT jint JNICALL JNIFUNCTION(arwAddTrackable(JNIEnv *env, jobject obj, jstring cfg));
	JNIEXPORT jboolean JNICALL JNIFUNCTION(arwRemoveTrackable(JNIEnv *env, jobject obj, jint trackableUID));
	JNIEXPORT jint JNICALL JNIFUNCTION(arwRemoveAllTrackables(JNIEnv *env, jobject obj));
    JNIEXPORT jboolean JNICALL JNIFUNCTION(arwQueryTrackableVisibilityAndTransformation(JNIEnv *env, jobject obj, jint trackableUID, jfloatArray matrix));
    JNIEXPORT jboolean JNICALL JNIFUNCTION(arwQueryTrackableVisibilityAndTransformationStereo(JNIEnv *env, jobject obj, jint trackableUID, jfloatArray matrixL, jfloatArray matrixR));
	JNIEXPORT jint JNICALL JNIFUNCTION(arwGetTrackablePatternCount(JNIEnv *env, jobject obj, int trackableUID));
    JNIEXPORT void JNICALL JNIFUNCTION(arwSetTrackerOptionBool(JNIEnv *env, jobject obj, jint option, jboolean value));
    JNIEXPORT void JNICALL JNIFUNCTION(arwSetTrackerOptionInt(JNIEnv *env, jobject obj, jint option, jint value));
    JNIEXPORT void JNICALL JNIFUNCTION(arwSetTrackerOptionFloat(JNIEnv *env, jobject obj, jint option, jfloat value));
    JNIEXPORT jboolean JNICALL JNIFUNCTION(arwGetTrackerOptionBool(JNIEnv *env, jobject obj, jint option));
    JNIEXPORT jint JNICALL JNIFUNCTION(arwGetTrackerOptionInt(JNIEnv *env, jobject obj, jint option));
    JNIEXPORT jfloat JNICALL JNIFUNCTION(arwGetTrackerOptionFloat(JNIEnv *env, jobject obj, jint option));
    JNIEXPORT void JNICALL JNIFUNCTION(arwSetTrackableOptionBool(JNIEnv *env, jobject obj, jint trackableUID, jint option, jboolean value));
    JNIEXPORT void JNICALL JNIFUNCTION(arwSetTrackableOptionInt(JNIEnv *env, jobject obj, jint trackableUID, jint option, jint value));
    JNIEXPORT void JNICALL JNIFUNCTION(arwSetTrackableOptionFloat(JNIEnv *env, jobject obj, jint trackableUID, jint option, jfloat value));
    JNIEXPORT jboolean JNICALL JNIFUNCTION(arwGetTrackableOptionBool(JNIEnv *env, jobject obj, jint trackableUID, jint option));
    JNIEXPORT jint JNICALL JNIFUNCTION(arwGetTrackableOptionInt(JNIEnv *env, jobject obj, jint trackableUID, jint option));
    JNIEXPORT jfloat JNICALL JNIFUNCTION(arwGetTrackableOptionFloat(JNIEnv *env, jobject obj, jint trackableUID, jint option));

	// Additional Java-specific function not found in the C-API
    JNIEXPORT jint JNICALL JNIFUNCTION(arwAndroidVideoPushInit(JNIEnv *env, jobject obj, jint videoSourceIndex, jint width, jint height, jstring pixelFormat, jint camera_index, jint camera_face));
    JNIEXPORT jint JNICALL JNIFUNCTION(arwAndroidVideoPush1(JNIEnv *env, jobject obj, jint videoSourceIndex, jbyteArray buf, jint bufSize));
    JNIEXPORT jint JNICALL JNIFUNCTION(arwAndroidVideoPush2(JNIEnv *env, jobject obj, jint videoSourceIndex,
                                       jobject buf0, jint buf0PixelStride, jint buf0RowStride,
                                       jobject buf1, jint buf1PixelStride, jint buf1RowStride,
                                       jobject buf2, jint buf2PixelStride, jint buf2RowStride,
                                       jobject buf3, jint buf3PixelStride, jint buf3RowStride));
    JNIEXPORT jint JNICALL JNIFUNCTION(arwAndroidVideoPushFinal(JNIEnv *env, jobject obj, jint videoSourceIndex));

	// ------------------------------------------------------------------------------------
	// JNI Functions Not Yet Implemented
    //bool arwStartRunningB(const char *vconf, const char *cparaBuff, const int cparaBuffLen, const float nearPlane, const float farPlane);
    //bool arwStartRunningStereoB(const char *vconfL, const char *cparaBuffL, const int cparaBuffLenL, const char *vconfR, const char *cparaBuffR, const int cparaBuffLenR, const char *transL2RBuff, const int transL2RBuffLen, const float nearPlane, const float farPlane);
    //bool arwGetTrackables(int *count_p, ARWTrackableStatus **statuses_p);
	//bool arwGetTrackablePatternConfig(int trackableUID, int patternID, float matrix[16], float *width, float *height);
	//bool arwGetTrackablePatternImage(int trackableUID, int patternID, Color *buffer);

    //bool arwLoadOpticalParams(const char *optical_param_name, const char *optical_param_buff, const int optical_param_buffLen, float *fovy_p, float *aspect_p, float m[16], float p[16]);
	// ------------------------------------------------------------------------------------
}

JNIEXPORT void JNICALL JNIFUNCTION(arwSetLogLevel(JNIEnv * env, jobject
                                           obj, jint
                                           logLevel)) {
    arwSetLogLevel(logLevel);
}


JNIEXPORT jstring JNICALL JNIFUNCTION(arwGetARToolKitVersion(JNIEnv *env, jobject obj)) 
{
	char versionString[1024];
    
	if (arwGetARToolKitVersion(versionString, 1024)) return env->NewStringUTF(versionString);		
	return env->NewStringUTF("unknown version");
}

JNIEXPORT jint JNICALL JNIFUNCTION(arwGetError(JNIEnv *env, jobject obj))
{
    return arwGetError();
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwInitialiseAR(JNIEnv *env, jobject obj)) 
{
	return arwInitialiseAR();
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwChangeToResourcesDir(JNIEnv *env, jobject obj, jstring resourcesDirectoryPath)) 
{
    bool ok;
    
    if (resourcesDirectoryPath != NULL) {
        const char *resourcesDirectoryPathC = env->GetStringUTFChars(resourcesDirectoryPath, NULL);
        ok = arwChangeToResourcesDir(resourcesDirectoryPathC);
        env->ReleaseStringUTFChars(resourcesDirectoryPath, resourcesDirectoryPathC);
    } else ok = arwChangeToResourcesDir(NULL);
    
    return ok;
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwShutdownAR(JNIEnv *env, jobject obj)) 
{
	return arwShutdownAR();
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwStartRunning(JNIEnv *env, jobject obj, jstring vconf, jstring cparaName))
{
    const char *vconfC = (env->IsSameObject(vconf, NULL) ? NULL : env->GetStringUTFChars(vconf, NULL));
	const char *cparaNameC = (env->IsSameObject(cparaName, NULL) ? NULL : env->GetStringUTFChars(cparaName, NULL));

	bool running = arwStartRunning(vconfC, cparaNameC);

	if (vconfC) env->ReleaseStringUTFChars(vconf, vconfC);
	if (cparaNameC) env->ReleaseStringUTFChars(cparaName, cparaNameC);

	return running;
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwStartRunningStereo(JNIEnv *env, jobject obj, jstring vconfL, jstring cparaNameL, jstring vconfR, jstring cparaNameR, jstring transL2RName))
{
	const char *vconfLC = env->GetStringUTFChars(vconfL, NULL);
	const char *cparaNameLC = env->GetStringUTFChars(cparaNameL, NULL);
	const char *vconfRC = env->GetStringUTFChars(vconfR, NULL);
	const char *cparaNameRC = env->GetStringUTFChars(cparaNameR, NULL);
	const char *transL2RNameC = env->GetStringUTFChars(transL2RName, NULL);
    
	bool running = arwStartRunningStereo(vconfLC, cparaNameLC, vconfRC, cparaNameRC, transL2RNameC);
    
	env->ReleaseStringUTFChars(vconfL, vconfLC);
	env->ReleaseStringUTFChars(cparaNameL, cparaNameLC);
	env->ReleaseStringUTFChars(vconfR, vconfRC);
	env->ReleaseStringUTFChars(cparaNameR, cparaNameRC);
	env->ReleaseStringUTFChars(transL2RName, transL2RNameC);
    
	return running;
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwIsRunning(JNIEnv *env, jobject obj)) 
{
	return arwIsRunning();
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwIsInited(JNIEnv *env, jobject obj)) 
{
	return arwIsInited();
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwStopRunning(JNIEnv *env, jobject obj)) 
{
	return arwStopRunning();
}

#define PIXEL_FORMAT_BUFFER_SIZE 1024

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwGetVideoParams(JNIEnv *env, jobject obj, jintArray width, jintArray height, jintArray pixelSize, jobjectArray pixelFormatString))
{
    int w, h, ps;
    char pf[PIXEL_FORMAT_BUFFER_SIZE];
    
    if (!arwGetVideoParams(&w, &h, &ps, pf, PIXEL_FORMAT_BUFFER_SIZE)) return false;
    if (width) env->SetIntArrayRegion(width, 0, 1, &w);
    if (height) env->SetIntArrayRegion(height, 0, 1, &h);
    if (pixelSize) env->SetIntArrayRegion(pixelSize, 0, 1, &ps);
    if (pixelFormatString) env->SetObjectArrayElement(pixelFormatString, 0, env->NewStringUTF(pf));
    return true;
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwGetVideoParamsStereo(JNIEnv *env, jobject obj, jintArray widthL, jintArray heightL, jintArray pixelSizeL, jobjectArray pixelFormatStringL, jintArray widthR, jintArray heightR, jintArray pixelSizeR, jobjectArray  pixelFormatStringR))
{
    int wL, hL, psL, wR, hR, psR;
    char pfL[PIXEL_FORMAT_BUFFER_SIZE], pfR[PIXEL_FORMAT_BUFFER_SIZE];

    if (!arwGetVideoParamsStereo(&wL, &hL, &psL, pfL, PIXEL_FORMAT_BUFFER_SIZE, &wR, &hR, &psR, pfR, PIXEL_FORMAT_BUFFER_SIZE)) return false;
    if (widthL) env->SetIntArrayRegion(widthL, 0, 1, &wL);
    if (heightL) env->SetIntArrayRegion(heightL, 0, 1, &hL);
    if (pixelSizeL) env->SetIntArrayRegion(pixelSizeL, 0, 1, &psL);
    if (pixelFormatStringL) env->SetObjectArrayElement(pixelFormatStringL, 0, env->NewStringUTF(pfL));
    if (widthR) env->SetIntArrayRegion(widthR, 0, 1, &wR);
    if (heightR) env->SetIntArrayRegion(heightR, 0, 1, &hR);
    if (pixelSizeR) env->SetIntArrayRegion(pixelSizeR, 0, 1, &psR);
    if (pixelFormatStringR) env->SetObjectArrayElement(pixelFormatStringR, 0, env->NewStringUTF(pfR));
    return true;
}

JNIEXPORT jfloatArray JNICALL JNIFUNCTION(arwGetProjectionMatrix(JNIEnv *env, jobject obj, jfloat nearPlane, jfloat farPlane))
{
	float proj[16];
    
	if (arwGetProjectionMatrix(nearPlane, farPlane, proj)) return glArrayToJava(env, proj, 16);
	return NULL;
}
	
JNIEXPORT jboolean JNICALL JNIFUNCTION(arwGetProjectionMatrixStereo(JNIEnv *env, jobject obj, jfloat nearPlane, jfloat farPlane, jfloatArray projL, jfloatArray projR))
{
	float pL[16];
	float pR[16];
    
	if (!arwGetProjectionMatrixStereo(nearPlane, farPlane, pL, pR)) return false;
    env->SetFloatArrayRegion(projL, 0, 16, pL);
    env->SetFloatArrayRegion(projR, 0, 16, pR);
	return true;
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwCapture(JNIEnv *env, jobject obj))
{
	return arwCapture();
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwUpdateAR(JNIEnv *env, jobject obj)) 
{
	return arwUpdateAR();
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwUpdateTexture32(JNIEnv *env, jobject obj, jbyteArray pinArray))
{
    bool updated = false;

    if (jbyte *inArray = env->GetByteArrayElements(pinArray, NULL)) {
        updated = arwUpdateTexture32((uint32_t *)inArray);
        env->ReleaseByteArrayElements(pinArray, inArray, 0); // 0 -> copy back the changes on the native side to the Java side.
    }

    return updated;
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwUpdateTextureStereo32(JNIEnv *env, jobject obj, jbyteArray pinArrayL, jbyteArray pinArrayR))
{
    bool updated = false;

    if (jbyte *inArrayL = env->GetByteArrayElements(pinArrayL, NULL)) {
        if (jbyte *inArrayR = env->GetByteArrayElements(pinArrayR, NULL)) {
            updated = arwUpdateTexture32Stereo((uint32_t *)inArrayL, (uint32_t *)inArrayR);
            env->ReleaseByteArrayElements(pinArrayR, inArrayR, 0); // 0 -> copy back the changes on the native side to the Java side.
        }
        env->ReleaseByteArrayElements(pinArrayL, inArrayL, 0); // 0 -> copy back the changes on the native side to the Java side.
    }

    return updated;
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwDrawVideoInit(JNIEnv *env, jobject obj, jint videoSourceIndex))
{
    return arwDrawVideoInit(videoSourceIndex);
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwDrawVideoSettings(JNIEnv *env, jobject obj, jint videoSourceIndex, jint width, jint height, jboolean rotate90, jboolean flipH, jboolean flipV, jint hAlign, jint vAlign, jint scalingMode, jintArray viewport))
{
    int32_t vp[4];
    if (!arwDrawVideoSettings(videoSourceIndex, width, height, rotate90, flipH, flipV, hAlign, vAlign, scalingMode, (viewport ? vp : NULL))) return false;
    if (viewport) {
        env->SetIntArrayRegion(viewport, 0, 4, vp);
    }
    return true;
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwDrawVideo(JNIEnv *env, jobject obj, jint videoSourceIndex))
{
    return arwDrawVideo(videoSourceIndex);
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwDrawVideoFinal(JNIEnv *env, jobject obj, jint videoSourceIndex))
{
    return arwDrawVideoFinal(videoSourceIndex);
}

JNIEXPORT jint JNICALL JNIFUNCTION(arwAddTrackable(JNIEnv *env, jobject obj, jstring cfg))
{
	jboolean isCopy;

	const char *cfgC = env->GetStringUTFChars(cfg, &isCopy);
	int trackableUID = arwAddTrackable(cfgC);
	env->ReleaseStringUTFChars(cfg, cfgC);
	return trackableUID;
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwRemoveTrackable(JNIEnv *env, jobject obj, jint trackableUID)) 
{
	return arwRemoveTrackable(trackableUID);
}

JNIEXPORT jint JNICALL JNIFUNCTION(arwRemoveAllTrackables(JNIEnv *env, jobject obj)) 
{
	return arwRemoveAllTrackables();
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwQueryTrackableVisibilityAndTransformation(JNIEnv *env, jobject obj, jint trackableUID, jfloatArray matrix))
{
    float m[16];
    
    if (!arwQueryTrackableVisibilityAndTransformation(trackableUID, m)) return false;
    env->SetFloatArrayRegion(matrix, 0, 16, m);
    return true;
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwQueryTrackableVisibilityAndTransformationStereo(JNIEnv *env, jobject obj, jint trackableUID, jfloatArray matrixL, jfloatArray matrixR))
{
	float mL[16];
	float mR[16];
    
	if (!arwQueryTrackableVisibilityAndTransformationStereo(trackableUID, mL, mR)) return false;
    env->SetFloatArrayRegion(matrixL, 0, 16, mL);
    env->SetFloatArrayRegion(matrixR, 0, 16, mR);
	return true;
}

JNIEXPORT jint JNICALL JNIFUNCTION(arwGetTrackablePatternCount(JNIEnv *env, jobject obj, int trackableUID)) 
{
	return arwGetTrackablePatternCount(trackableUID);
}

JNIEXPORT void JNICALL JNIFUNCTION(arwSetTrackerOptionBool(JNIEnv *env, jobject obj, jint option, jboolean value))
{
    return arwSetTrackerOptionBool(option, value);
}

JNIEXPORT void JNICALL JNIFUNCTION(arwSetTrackerOptionInt(JNIEnv *env, jobject obj, jint option, jint value))
{
    return arwSetTrackerOptionInt(option, value);
}

JNIEXPORT void JNICALL JNIFUNCTION(arwSetTrackerOptionFloat(JNIEnv *env, jobject obj, jint option, jfloat value))
{
    return arwSetTrackerOptionFloat(option, value);
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwGetTrackerOptionBool(JNIEnv *env, jobject obj, jint option))
{
    return arwGetTrackerOptionBool(option);
}

JNIEXPORT jint JNICALL JNIFUNCTION(arwGetTrackerOptionInt(JNIEnv *env, jobject obj, jint option))
{
    return arwGetTrackerOptionInt(option);
}

JNIEXPORT jfloat JNICALL JNIFUNCTION(arwGetTrackerOptionFloat(JNIEnv *env, jobject obj, jint option))
{
    return arwGetTrackerOptionFloat(option);
}

JNIEXPORT void JNICALL JNIFUNCTION(arwSetTrackableOptionInt(JNIEnv *env, jobject obj, jint trackableUID, jint option, jint value))
{
    return arwSetTrackableOptionInt(trackableUID, option, value);
}

JNIEXPORT void JNICALL JNIFUNCTION(arwSetTrackableOptionFloat(JNIEnv *env, jobject obj, jint trackableUID, jint option, jfloat value)) 
{
    return arwSetTrackableOptionFloat(trackableUID, option, value);
}

JNIEXPORT jboolean JNICALL JNIFUNCTION(arwGetTrackableOptionBool(JNIEnv *env, jobject obj, jint trackableUID, jint option)) 
{
    return arwGetTrackableOptionBool(trackableUID, option);
}

JNIEXPORT jint JNICALL JNIFUNCTION(arwGetTrackableOptionInt(JNIEnv *env, jobject obj, jint trackableUID, jint option)) 
{
    return arwGetTrackableOptionInt(trackableUID, option);
}

JNIEXPORT jfloat JNICALL JNIFUNCTION(arwGetTrackableOptionFloat(JNIEnv *env, jobject obj, jint trackableUID, jint option)) 
{
    return arwGetTrackableOptionFloat(trackableUID, option);
}

// Additional JNI functions not found in the C API.

JNIEXPORT jint JNICALL JNIFUNCTION(arwAndroidVideoPushInit(JNIEnv *env, jobject obj, jint videoSourceIndex, jint width, jint height, jstring pixelFormat, jint camera_index, jint camera_face))
{
    if (!gARTK) {
        return -1;
    }

    jboolean isCopy;
    jint ret;

    const char *pixelFormatC = env->GetStringUTFChars(pixelFormat, &isCopy);
    ret = gARTK->androidVideoPushInit(env, obj, videoSourceIndex, width, height, pixelFormatC, camera_index, camera_face);
    env->ReleaseStringUTFChars(pixelFormat, pixelFormatC);
    return ret;
}

JNIEXPORT jint JNICALL JNIFUNCTION(arwAndroidVideoPush1(JNIEnv *env, jobject obj, jint videoSourceIndex, jbyteArray buf, jint bufSize))
{
    if (!gARTK) {
        return -1;
    }

    return gARTK->androidVideoPush1(env, obj, videoSourceIndex, buf, bufSize);
}

JNIEXPORT jint JNICALL JNIFUNCTION(arwAndroidVideoPush2(JNIEnv *env, jobject obj, jint videoSourceIndex,
                                                        jobject buf0, jint buf0PixelStride, jint buf0RowStride,
                                                        jobject buf1, jint buf1PixelStride, jint buf1RowStride,
                                                        jobject buf2, jint buf2PixelStride, jint buf2RowStride,
                                                        jobject buf3, jint buf3PixelStride, jint buf3RowStride))
{
    if (!gARTK) {
        return -1;
    }

    return gARTK->androidVideoPush2(env, obj, videoSourceIndex, buf0, buf0PixelStride, buf0RowStride, buf1, buf1PixelStride, buf1RowStride, buf2, buf2PixelStride, buf2RowStride, buf3, buf3PixelStride, buf3RowStride);
}

JNIEXPORT jint JNICALL JNIFUNCTION(arwAndroidVideoPushFinal(JNIEnv *env, jobject obj, jint videoSourceIndex))
{
    if (!gARTK) {
        return -1;
    }

    return gARTK->androidVideoPushFinal(env, obj, videoSourceIndex);
}

#endif // ARX_TARGET_PLATFORM_ANDROID
