/*
 *  Calibration.cpp
 *  artoolkitX Camera Calibration Utility
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
 *  Copyright 2015-2017 Daqri, LLC.
 *  Copyright 2002-2015 ARToolworks, Inc.
 *
 *  Author(s): Hirokazu Kato, Philip Lamb
 *
 */

#include <ARX/Calibration.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ARX/calc.hpp>

//
// A class to encapsulate the inputs and outputs of a corner-finding run, and to allow for copying of the results
// of a completed run.
//

Calibration::CalibrationCornerFinderData::CalibrationCornerFinderData(const Calibration::CalibrationPatternType patternType_in, const cv::Size patternSize_in, const int videoWidth_in, const int videoHeight_in) :
    patternType(patternType_in),
    patternSize(patternSize_in),
    videoWidth(videoWidth_in),
    videoHeight(videoHeight_in),
    cornerFoundAllFlag(0),
    corners()
{
    init();
}

// copy constructor.
Calibration::CalibrationCornerFinderData::CalibrationCornerFinderData(const Calibration::CalibrationCornerFinderData& orig) :
    patternType(orig.patternType),
    patternSize(orig.patternSize),
    videoWidth(orig.videoWidth),
    videoHeight(orig.videoHeight),
    cornerFoundAllFlag(orig.cornerFoundAllFlag),
    corners(orig.corners)
{
    init();
    copy(orig);
}

// copy assignement.
const Calibration::CalibrationCornerFinderData& Calibration::CalibrationCornerFinderData::operator=(const Calibration::CalibrationCornerFinderData& orig)
{
    if (this != &orig) {
        dealloc();
        patternType = orig.patternType;
        patternSize = orig.patternSize;
        videoWidth = orig.videoWidth;
        videoHeight = orig.videoHeight;
        cornerFoundAllFlag = orig.cornerFoundAllFlag;
        corners = orig.corners;
        init();
        copy(orig);
    }
    return *this;
}

Calibration::CalibrationCornerFinderData::~CalibrationCornerFinderData()
{
    dealloc();
}

void Calibration::CalibrationCornerFinderData::init()
{
    if (videoWidth > 0 && videoHeight > 0) {
        arMalloc(videoFrame, uint8_t, videoWidth * videoHeight);
        calibImage = cvCreateImageHeader(cvSize(videoWidth, videoHeight), IPL_DEPTH_8U, 1);
        cvSetData(calibImage, videoFrame, videoWidth); // Last parameter is rowBytes.
    } else {
        videoFrame = nullptr;
        calibImage = nullptr;
    }
}

void Calibration::CalibrationCornerFinderData::copy(const CalibrationCornerFinderData& orig)
{
    memcpy(videoFrame, orig.videoFrame, sizeof(uint8_t) * videoWidth * videoHeight);
}

void Calibration::CalibrationCornerFinderData::dealloc()
{
    if (calibImage) cvReleaseImageHeader(&calibImage);
    free(videoFrame);
}


//
// User-facing calibration functions.
//

Calibration::Calibration(const CalibrationPatternType patternType, const int calibImageCountMax, const cv::Size patternSize, const int chessboardSquareWidth, const int videoWidth, const int videoHeight) :
    m_cornerFinderData(patternType, patternSize, videoWidth, videoHeight),
    m_cornerFinderResultData(patternType, patternSize, 0, 0),
    m_calibImageCountMax(calibImageCountMax),
    m_patternType(patternType),
    m_patternSize(patternSize),
    m_chessboardSquareWidth(chessboardSquareWidth),
    m_videoWidth(videoWidth),
    m_videoHeight(videoHeight),
    m_corners()
{
    
}

bool Calibration::cornerFinderResults(int *cornerFoundAllFlag, std::vector<cv::Point2f>& corners)
{
    *cornerFoundAllFlag = m_cornerFinderResultData.cornerFoundAllFlag;
    corners = m_cornerFinderResultData.corners;
    return true;
}

bool Calibration::findCorners(ARUint8 *imageBytes)
{
    
    memcpy(m_cornerFinderData.videoFrame, imageBytes, m_cornerFinderData.videoWidth * m_cornerFinderData.videoHeight);

    m_cornerFinderData.cornerFoundAllFlag = cv::findChessboardCorners(cv::cvarrToMat(m_cornerFinderData.calibImage), m_cornerFinderData.patternSize, m_cornerFinderData.corners, CV_CALIB_CB_FAST_CHECK|CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS);
    
    return true;
}

bool Calibration::capture()
{
    if (m_corners.size() >= m_calibImageCountMax) return false;
   
    bool saved = false;
    
    if (m_cornerFinderResultData.cornerFoundAllFlag) {
        // Refine the corner positions.
        cornerSubPix(cv::cvarrToMat(m_cornerFinderResultData.calibImage), m_cornerFinderResultData.corners, cv::Size(5,5), cvSize(-1,-1), cv::TermCriteria(CV_TERMCRIT_ITER, 100, 0.1));
        
        // Save the corners.
        m_corners.push_back(m_cornerFinderResultData.corners);
        saved = true;
    }
    
    if (saved) {
        ARPRINT("---------- %2d/%2d -----------\n", (int)m_corners.size(), m_calibImageCountMax);
        const std::vector<cv::Point2f>& corners = m_corners.back();
        for (std::vector<cv::Point2f>::const_iterator it = corners.begin(); it < corners.end(); it++) {
            ARPRINT("  %f, %f\n", it->x, it->y);
        }
        ARPRINT("---------- %2d/%2d -----------\n", (int)m_corners.size(), m_calibImageCountMax);
    }
    
    return (saved);
}

bool Calibration::uncapture(void)
{
    if (m_corners.size() <= 0) return false;
    m_corners.pop_back();
    return true;
}

bool Calibration::uncaptureAll(void)
{
    if (m_corners.size() <= 0) return false;
    m_corners.clear();
    return true;
}

void Calibration::calib(ARParam *param_out, ARdouble *err_min_out, ARdouble *err_avg_out, ARdouble *err_max_out)
{
    float resultsArray[10];
    float *results;
    results = resultsArray;
    
    calc((int)m_corners.size(), m_patternType, m_patternSize, m_chessboardSquareWidth, m_corners, m_videoWidth, m_videoHeight, AR_DIST_FUNCTION_VERSION_DEFAULT, param_out, results);
}

void Calibration::SaveParam(ARParam *param, const char *paramPathname) {
    if (arParamSave(paramPathname, 1, param) < 0) {
        ARLOGe("Error writing camera_para.dat file.\n");
    } else {
        
    }
}

Calibration::~Calibration()
{
    
}


