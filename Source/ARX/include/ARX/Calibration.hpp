/*
 *  Calibration.hpp
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

#pragma once

#include <ARX/AR/ar.h>
#include <ARX/AR/param.h>
#include <opencv2/core/core.hpp>
#include <ARX/ARVideoSource.h>
#include <map>
//#include <CoreGraphics/CoreGraphics.h>

class Calibration
{
public:
    
    enum class CalibrationPatternType {
        CHESSBOARD,
        CIRCLES_GRID,
        ASYMMETRIC_CIRCLES_GRID
    };
    
    Calibration(const CalibrationPatternType patternType, const int calibImageCountMax, const cv::Size patternSize, const int chessboardSquareWidth, const int videoWidth, const int videoHeight);
    int calibImageCount() const {return (int)m_corners.size(); }
    int calibImageCountMax() const {return m_calibImageCountMax; }
    bool cornerFinderResults(int *cornerFoundAllFlag, std::vector<cv::Point2f>& corners);
    bool findCorners(ARUint8 *imageBytes);
    bool capture();
    void SaveParam(ARParam *param, const char *paramPathname);
    bool uncapture();
    bool uncaptureAll();
    void calib(ARParam *param_out, ARdouble *err_min_out, ARdouble *err_avg_out, ARdouble *err_max_out);
    ~Calibration();
    
private:
    
    Calibration(const Calibration&) = delete; // No copy construction.
    Calibration& operator=(const Calibration&) = delete; // No copy assignment.
    
    // A class to encapsulate the inputs and outputs of a corner-finding run, and to allow for copying of the results
    // of a completed run.
    class CalibrationCornerFinderData {
    public:
        CalibrationCornerFinderData(const CalibrationPatternType patternType_in, const cv::Size patternSize_in, const int videoWidth_in, const int videoHeight_in);
        CalibrationCornerFinderData(const CalibrationCornerFinderData& orig);
        const CalibrationCornerFinderData& operator=(const CalibrationCornerFinderData& orig);
        ~CalibrationCornerFinderData();
        CalibrationPatternType patternType;
        cv::Size             patternSize;
        int                  videoWidth;
        int                  videoHeight;
        uint8_t             *videoFrame;
        cv::Mat            calibImage;
        int                  cornerFoundAllFlag;
        std::vector<cv::Point2f> corners;
    private:
        void init();
        void copy(const CalibrationCornerFinderData& orig);
        void dealloc();
    };
    
    CalibrationCornerFinderData m_cornerFinderData; // Corner finder input and output.
    CalibrationCornerFinderData m_cornerFinderResultData; // Corner finder results copy, for display to user.
    
    std::vector<std::vector<cv::Point2f> > m_corners; // Collected corner information which gets passed to the OpenCV calibration function.
    int                  m_calibImageCountMax;
    CalibrationPatternType m_patternType;
    cv::Size             m_patternSize;
    int                  m_chessboardSquareWidth;
    int                  m_videoWidth;
    int                  m_videoHeight;
};
