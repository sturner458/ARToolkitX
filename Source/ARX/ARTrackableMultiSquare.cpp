/*
 *  ARTrackableMultiSquare.cpp
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
 *  Author(s): Philip Lamb.
 *
 */

#include <ARX/ARTrackableMultiSquare.h>
#include <ARX/ARController.h>

#ifdef ARDOUBLE_IS_FLOAT
#  define _0_0 0.0f
#  define _1_0 1.0f
#else
#  define _0_0 0.0
#  define _1_0 1.0
#endif

ARTrackableMultiSquare::ARTrackableMultiSquare(int setUID) : ARTrackable(MULTI, setUID),
    m_loaded(false),
    config(NULL),
    robustFlag(false)
{
}

ARTrackableMultiSquare::~ARTrackableMultiSquare()
{
    if (m_loaded) unload();
}

bool ARTrackableMultiSquare::load(const char *multiConfig, ARPattHandle *arPattHandle)
{
    if (m_loaded) unload();
    
    config = arMultiReadConfigFile(multiConfig, arPattHandle);
    
    if (!config) {
        ARLOGe("Error loading multimarker config %s\n", multiConfig);
        return false;
    }
    
    visible = visiblePrev = false;
    
    // ARPatterns to hold images and positions of the patterns for display to the user.
    allocatePatterns(config->marker_num);
    for (int i = 0; i < patternCount; i++) {
        if (config->marker[i].patt_type == AR_MULTI_PATTERN_TYPE_TEMPLATE) {
            patterns[i]->loadTemplate(config->marker[i].patt_id, arPattHandle, (float)config->marker[i].width);
        } else {
            patterns[i]->loadMatrix(config->marker[i].patt_id, AR_MATRIX_CODE_3x3, (float)config->marker[i].width); // TODO: Determine actual matrix code type in use.
        }
        patterns[i]->m_matrix[ 0] = config->marker[i].trans[0][0];
        patterns[i]->m_matrix[ 1] = config->marker[i].trans[1][0];
        patterns[i]->m_matrix[ 2] = config->marker[i].trans[2][0];
        patterns[i]->m_matrix[ 3] = _0_0;
        patterns[i]->m_matrix[ 4] = config->marker[i].trans[0][1];
        patterns[i]->m_matrix[ 5] = config->marker[i].trans[1][1];
        patterns[i]->m_matrix[ 6] = config->marker[i].trans[2][1];
        patterns[i]->m_matrix[ 7] = _0_0;
        patterns[i]->m_matrix[ 8] = config->marker[i].trans[0][2];
        patterns[i]->m_matrix[ 9] = config->marker[i].trans[1][2];
        patterns[i]->m_matrix[10] = config->marker[i].trans[2][2];
        patterns[i]->m_matrix[11] = _0_0;
        patterns[i]->m_matrix[12] = config->marker[i].trans[0][3];
        patterns[i]->m_matrix[13] = config->marker[i].trans[1][3];
        patterns[i]->m_matrix[14] = config->marker[i].trans[2][3];
        patterns[i]->m_matrix[15] = _1_0;
    }
    config->min_submarker = 0;
    m_loaded = true;
    return true;
}

bool ARTrackableMultiSquare::unload()
{
    if (m_loaded) {
        freePatterns();
        if (config) {
            arMultiFreeConfig(config);
            config = NULL;
        }
        m_loaded = false;
    }
    
    return true;
}

bool ARTrackableMultiSquare::updateWithDetectedMarkers(ARMarkerInfo* markerInfo, int markerNum, AR3DHandle *ar3DHandle)
{
    if (!m_loaded || !config) return false;            // Can't update without multimarker config

    visiblePrev = visible;

    imagePoints.clear();
    if (markerInfo) {
    
        ARdouble err;

        if (robustFlag) {
            err = arGetTransMatMultiSquareRobust(ar3DHandle, markerInfo, markerNum, config);
        } else {
            err = arGetTransMatMultiSquare(ar3DHandle, markerInfo, markerNum, config);
        }
        
        // Marker is visible if a match was found.
        if (config->prevF != 0) {
            visible = true;
            for (int j = 0; j < 3; j++) for (int k = 0; k < 4; k++) trans[j][k] = config->trans[j][k];

            //Record the locations of all the corners
            for (int i = 0; i < config->marker_num; i++) {
                for (int j = 0; j < markerNum; j++) {
                    if (markerInfo[j].idMatrix == config->marker[i].patt_id) {
                        int dir;
                        if (markerInfo[j].idMatrix < 0)
                            dir = markerInfo[j].dirPatt;
                        else if (markerInfo[j].idPatt < 0)
                            dir = markerInfo[j].dirMatrix;
                        else
                            dir = markerInfo[j].dir;

                        imagePoints.push_back(cv::Point2f(markerInfo[j].vertex[(4 - dir) % 4][0], markerInfo[j].vertex[(4 - dir) % 4][1]));
                        imagePoints.push_back(cv::Point2f(markerInfo[j].vertex[(5 - dir) % 4][0], markerInfo[j].vertex[(5 - dir) % 4][1]));
                        imagePoints.push_back(cv::Point2f(markerInfo[j].vertex[(6 - dir) % 4][0], markerInfo[j].vertex[(6 - dir) % 4][1]));
                        imagePoints.push_back(cv::Point2f(markerInfo[j].vertex[(7 - dir) % 4][0], markerInfo[j].vertex[(7 - dir) % 4][1]));
                    }
                }
            }

        } else visible = false;
    } else visible = false;

    return (ARTrackable::update()); // Parent class will finish update.
}

bool ARTrackableMultiSquare::updateWithDetectedDatums2(ARParam arParams, ARUint8* buffLuma, int imageWidth, int imageHeight, AR3DHandle* ar3DHandle, bool largeBoard, int numberOfDatums)
{
    ARdouble* datumCoords2D;
    ARdouble* datumCoords;

    cv::Mat grayImage = cv::Mat(imageHeight, imageWidth, CV_8UC1, (void*)buffLuma, imageWidth);

    // Known coordinates for circle centres.
    std::vector<cv::Point2f> circleCentres;

    //ARdouble errMax = 15.0;

    if (numberOfDatums == 4)
    {
        circleCentres.push_back(cv::Point2f(-115, 25));
        circleCentres.push_back(cv::Point2f(-115, -25));
        circleCentres.push_back(cv::Point2f(45, 25));
        circleCentres.push_back(cv::Point2f(45, -25));
    }
    else if (numberOfDatums == 6)
    {
        circleCentres.push_back(cv::Point2f(-115, 25));
        circleCentres.push_back(cv::Point2f(-115, -25));
        circleCentres.push_back(cv::Point2f(-115, 0));
        circleCentres.push_back(cv::Point2f(45, 0));
        circleCentres.push_back(cv::Point2f(45, 25));
        circleCentres.push_back(cv::Point2f(45, -25));
    }

    ARdouble ox, oy;
    std::vector<cv::Point2f> circles;
    std::vector<cv::Point3f> circlePoints;

    double datumCircleDiameter = 15.0;

    for (int i = 0; i < (int)circleCentres.size(); i++)
    {
        cv::Point2f pt = circleCentres.at(i);
        if (GetCenterPointForDatum2(datumCircleDiameter, pt.x, pt.y, arParams, trans, grayImage, imageWidth, imageHeight, &ox, &oy))
        {
            circles.push_back(cv::Point2f(ox, oy));
            circlePoints.push_back(cv::Point3f(pt.x, pt.y, 0));
        }
    }
    
    //ARLOGd("Found %i datums", (int)corners.size());
    
    // Known coordinates for circle centres.
    std::vector<cv::Point2f> cornerCentres;    // Add the corners of the marker square 1
    
    cornerCentres.push_back(cv::Point2f(-32.5, 32.5));
    cornerCentres.push_back(cv::Point2f(-32.5, -32.5));
    cornerCentres.push_back(cv::Point2f(32.5, -32.5));
    cornerCentres.push_back(cv::Point2f(32.5, 32.5));
    // Add the corners of the marker square 2
    cornerCentres.push_back(cv::Point2f(-102.5, 32.5));
    cornerCentres.push_back(cv::Point2f(-102.5, -32.5));
    cornerCentres.push_back(cv::Point2f(-37.5, -32.5));
    cornerCentres.push_back(cv::Point2f(-37.5, 32.5));
    
    std::vector<cv::Point2f> corners;
    std::vector<cv::Point3f> cornerPoints;
    for (int i = 0; i < (int)cornerCentres.size(); i++)
    {
        cv::Point2f pt = cornerCentres.at(i);
        ModelToImageSpace(arParams, trans, pt.x, pt.y, &ox, &oy);
        corners.push_back(cv::Point2f(ox, oy));
        cornerPoints.push_back(cv::Point3f(pt.x, pt.y, 0));
    }

    // Populate datum coords with whatever we have in objectPoints.
    datumCoords = new ARdouble[(circlePoints.size() + cornerPoints.size()) * 3];
    for (int i = 0; i < (int)cornerPoints.size(); i++)
    {
        cv::Point3f pt = cornerPoints.at(i);
        datumCoords[i * 3] = pt.x;
        datumCoords[i * 3 + 1] = pt.y;
        datumCoords[i * 3 + 2] = 0;
    }

    std::vector<cv::Point2f> cornersCopy;
    for (int i = 0; i < corners.size(); i++)
    {
        cornersCopy.push_back(cv::Point2f(corners.at(i).x, corners.at(i).y));
    }

    datumCoords2D = new ARdouble[(circles.size() + corners.size()) * 2];

    cv::cornerSubPix(grayImage, corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 0.1));
    for (int i = 0; i < (int)corners.size(); i++)
    {
        ARdouble ix, iy;
        ARdouble ox, oy;
        double d = sqrt((cornersCopy.at(i).x - corners.at(i).x) * (cornersCopy.at(i).x - corners.at(i).x) + (cornersCopy.at(i).y - corners.at(i).y) * (cornersCopy.at(i).y - corners.at(i).y));
        if (d < 3.0)
        {
            // refined
            ix = corners.at(i).x;
            iy = corners.at(i).y;
        }
        else
        {
            // non-refined
            ix = cornersCopy.at(i).x;
            iy = cornersCopy.at(i).y;
        }

        arParamObserv2Ideal(arParams.dist_factor, ix, iy, &ox, &oy, arParams.dist_function_version);
        datumCoords2D[i * 2] = ox;
        datumCoords2D[i * 2 + 1] = oy;
    }
    
    int n = (int)corners.size();
    for (int i = 0; i < circlePoints.size(); i++) {
        ARdouble ix, iy;
        ARdouble ox, oy;
        ix = circles.at(i).x;
        iy = circles.at(i).y;
        arParamObserv2Ideal(arParams.dist_factor, ix, iy, &ox, &oy, arParams.dist_function_version);
        datumCoords2D[n * 2] = ox;
        datumCoords2D[n * 2 + 1] = oy;
        cv::Point3f pt = circlePoints.at(i);
        datumCoords[n * 3] = pt.x;
        datumCoords[n * 3 + 1] = pt.y;
        datumCoords[n * 3 + 2] = 0;
        n = n + 1;
    }

    ARdouble err;
    err = arGetTransMatDatum(ar3DHandle, datumCoords2D, datumCoords, (int)(circles.size() + corners.size()), trans);
    if (err > 10.0f) visible = false;

    imageDatums.clear();
    imagePoints.clear();

    for (int i = 0; i < cornerCentres.size(); i++)
    {
        cv::Point2f pt = cornerCentres.at(i);
        ModelToImageSpace(arParams, trans, pt.x, pt.y, &ox, &oy);
        imageDatums.push_back(cv::Point2f(ox, oy));
    }
    
    for (int i = 0; i < circlePoints.size(); i++)
    {
        cv::Point3f pt = circlePoints.at(i);
        ModelToImageSpace(arParams, trans, pt.x, pt.y, &ox, &oy);
        imageDatums.push_back(cv::Point2f(ox, oy));
    }

    delete[] datumCoords2D;
    delete[] datumCoords;

    if (visible) return (ARTrackable::update()); // Parent class will finish update.
    return false;
}

bool ARTrackableMultiSquare::updateWithDetectedMarkersOpenCV(ARMarkerInfo* markerInfo, int markerNum, AR3DHandle* ar3DHandle, ARHandle* arHandle)
{
    if (!m_loaded || !config) return false;            // Can't update without multimarker config

    visiblePrev = visible;
    visible = false;

    imagePoints.clear();
    if (markerInfo) {

        ARdouble err;

        std::vector<cv::Point3f> cornerPoints;
        for (int i = 0; i < config->marker_num; i++) {
            std::vector<cv::Point3f> cornerPoints2;
            cornerPoints2.push_back(cv::Point3f(-config->marker[i].width / 2.0f, config->marker[i].width / 2.0f, 0.0f));
            cornerPoints2.push_back(cv::Point3f(config->marker[i].width / 2.0f, config->marker[i].width / 2.0f, 0.0f));
            cornerPoints2.push_back(cv::Point3f(config->marker[i].width / 2.0f, -config->marker[i].width / 2.0f, 0.0f));
            cornerPoints2.push_back(cv::Point3f(-config->marker[i].width / 2.0f, -config->marker[i].width / 2.0f, 0.0f));

            for (int j = 0; j < 4; j++) {
                cv::Point3f pt = cornerPoints2.at(j);
                float cx = config->marker[i].trans[0][0] * pt.x + config->marker[i].trans[0][1] * pt.y + config->marker[i].trans[0][3];
                float cy = config->marker[i].trans[1][0] * pt.x + config->marker[i].trans[1][1] * pt.y + config->marker[i].trans[1][3];
                float cz = config->marker[i].trans[2][0] * pt.x + config->marker[i].trans[2][1] * pt.y + config->marker[i].trans[2][3];

                cornerPoints.push_back(cv::Point3f(cx, cy, 0));
            }

            float ox, oy;
            for (int j = 0; j < markerNum; j++) {
                if (markerInfo[j].idMatrix == config->marker[i].patt_id) {
                    int dir;
                    if (markerInfo[j].idMatrix < 0)
                        dir = markerInfo[j].dirPatt;
                    else if (markerInfo[j].idPatt < 0)
                        dir = markerInfo[j].dirMatrix;
                    else
                        dir = markerInfo[j].dir;

                    arParamIdeal2ObservLTf(&arHandle->arParamLT->paramLTf, markerInfo[j].vertex[(4 - dir) % 4][0], markerInfo[j].vertex[(4 - dir) % 4][1], &ox, &oy);
                    imagePoints.push_back(cv::Point2f(ox, oy));
                    arParamIdeal2ObservLTf(&arHandle->arParamLT->paramLTf, markerInfo[j].vertex[(5 - dir) % 4][0], markerInfo[j].vertex[(5 - dir) % 4][1], &ox, &oy);
                    imagePoints.push_back(cv::Point2f(ox, oy));
                    arParamIdeal2ObservLTf(&arHandle->arParamLT->paramLTf, markerInfo[j].vertex[(6 - dir) % 4][0], markerInfo[j].vertex[(6 - dir) % 4][1], &ox, &oy);
                    imagePoints.push_back(cv::Point2f(ox, oy));
                    arParamIdeal2ObservLTf(&arHandle->arParamLT->paramLTf, markerInfo[j].vertex[(7 - dir) % 4][0], markerInfo[j].vertex[(7 - dir) % 4][1], &ox, &oy);
                    imagePoints.push_back(cv::Point2f(ox, oy));

                    break;
                }
            }
        }

        if (imagePoints.size() == cornerPoints.size()) {
            cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
            cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output translation vector
            cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64FC1);
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    cameraMatrix.at<double>(i, j) = (double)(arHandle->arParamLT->param.mat[i][j]);
                }
            }

            double s = (double)(arHandle->arParamLT->param.dist_factor[16]);
            cameraMatrix.at<double>(0, 0) *= s;
            cameraMatrix.at<double>(0, 1) *= s;
            cameraMatrix.at<double>(1, 0) *= s;
            cameraMatrix.at<double>(1, 1) *= s;

            cv::Mat distortionCoeffs = cv::Mat(8, 1, CV_64FC1);
            for (int i = 0; i < 8; i++) {
                distortionCoeffs.at<double>(i) = (double)(arHandle->arParamLT->param.dist_factor[i]);
            }
            cv::solvePnP(cornerPoints, imagePoints, cameraMatrix, distortionCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);
            cv::Mat rotationMatrix = cv::Mat(3, 3, CV_64FC1);
            Rodrigues(rvec, rotationMatrix);

            for (int j = 0; j < 3; j++) {
                for (int i = 0; i < 3; i++) {
                    trans[j][i] = (float)rotationMatrix.at<double>(j, i);
                }
                trans[j][3] = (float)tvec.at<double>(j);
            }

            std::vector<cv::Point2f> reprojectPoints;
            cv::projectPoints(cornerPoints, rvec, tvec, cameraMatrix, distortionCoeffs, reprojectPoints);
            err = cv::norm(reprojectPoints, imagePoints);

            imagePoints.clear();
            for (int i = 0; i < reprojectPoints.size(); i++) {
                imagePoints.push_back(cv::Point2f(reprojectPoints.at(i).x, reprojectPoints.at(i).y));
            }

            if (err < 10.0f) visible = true;
        }
    }

    return (ARTrackable::update()); // Parent class will finish update.
}

bool ARTrackableMultiSquare::updateWithDetectedMarkersStereo(ARMarkerInfo* markerInfoL, int markerNumL, ARMarkerInfo* markerInfoR, int markerNumR, AR3DStereoHandle *handle, ARdouble transL2R[3][4])
{
    if (!m_loaded || !config) return false;            // Can't update without multimarker config
    
    visiblePrev = visible;
    
    if (markerInfoL && markerInfoR) {
        
        ARdouble err;
        
        if (robustFlag) {
            err = arGetTransMatMultiSquareStereoRobust(handle, markerInfoL, markerNumL, markerInfoR, markerNumR, config);
        } else {
            err = arGetTransMatMultiSquareStereo(handle, markerInfoL, markerNumL, markerInfoR, markerNumR, config);
        }
        
        // Marker is visible if a match was found.
        if (config->prevF != 0) {
            visible = true;
            for (int j = 0; j < 3; j++) for (int k = 0; k < 4; k++) trans[j][k] = config->trans[j][k];
        } else visible = false;
        
    } else visible = false;
    
    return (ARTrackable::update(transL2R)); // Parent class will finish update.
}

