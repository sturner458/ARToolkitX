/*
 *  ARTrackableMultiSquareAuto.cpp
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
 *  Copyright 2018 Eden Networks Ltd.
 *
 *  Author(s): Philip Lamb.
 *
 */

#include <ARX/ARTrackableMultiSquare.h>
#include <ARX/ARTrackableMultiSquareAuto.h>

#if HAVE_GTSAM
#  include "mapper.hpp"
#endif

// This struct contains member variables we don't want to appear in the header.
struct ARTrackableMapPrivateMembers {
    ARTrackableMapPrivateMembers()
#if HAVE_GTSAM
    : m_mapper(0.04, 1)
#endif
    {
    }
#if HAVE_GTSAM
    arx_mapper::Mapper m_mapper;
#endif
};

ARTrackableMultiSquareAuto::ARTrackableMultiSquareAuto(int setUID) : ARTrackable(MULTI_AUTO, setUID),
    m_OriginMarkerUid(0),
    m_markerWidth(65.0f),
    m_MultiConfig(NULL),
    m_pm(new struct ARTrackableMapPrivateMembers)
{
    m_MultiConfig = arMultiAllocConfig();
}

ARTrackableMultiSquareAuto::~ARTrackableMultiSquareAuto()
{
    if (m_MultiConfig) {
        arMultiFreeConfig(m_MultiConfig);
        m_MultiConfig = NULL;
    }
}

bool ARTrackableMultiSquareAuto::initWithOriginMarkerUID(int originMarkerUID, ARdouble markerWidth)
{
    //ARLOGd("Initialising Multisquare Marker width = %f\n", markerWidth);
    m_OriginMarkerUid = originMarkerUID;
    m_markerWidth = markerWidth;
    return true;
}

bool ARTrackableMultiSquareAuto::updateMapper(ARMarkerInfo* markerInfo, int markerNum, int videoWidth, int videoHeight, AR3DHandle *ar3DHandle, std::vector<ARTrackable*>& trackables) {
    
    //ARLOGd("ARTrackableMultiSquareAuto::updateWithDetectedMarkers(...)\n");
    
    // Need at least one detected marker.
    if (markerInfo && markerNum > 0) {
        
        // Shallow copy the markerInfo array, so that we can filter it.
        ARMarkerInfo *markerInfoCopy = (ARMarkerInfo *)malloc(sizeof(ARMarkerInfo) * markerNum);
        memcpy(markerInfoCopy, markerInfo, sizeof(ARMarkerInfo) * markerNum);
        
        int goodMarkers = 0;
        int discarded = 0;
        ARdouble borderX = (ARdouble)videoWidth*m_ImageBorderZone;
        ARdouble borderY = (ARdouble)videoHeight*m_ImageBorderZone;
        for (int i = 0; i < markerNum; i++) {
            if (markerInfoCopy[i].idMatrix == -1) continue;

            // Discard markers towards the edge of the image.
            if (m_ImageBorderZone > 0.0f) {
                if (markerInfoCopy[i].pos[0] < borderX || markerInfoCopy[i].pos[0] > ((ARdouble)videoWidth - borderX)
                    || markerInfoCopy[i].pos[1] < borderY || markerInfoCopy[i].pos[1] > ((ARdouble)videoHeight - borderY)) {
                    markerInfoCopy[i].id = markerInfoCopy[i].idPatt = markerInfoCopy[i].idMatrix = -1;
                    discarded++;
                    continue;
                }
            }
            
            // Marker is OK.
            goodMarkers++;
        }
        //if (discarded > 0) ARLOGd("Discarded %d markers not in image centre.\n", discarded);
        
        if (goodMarkers > 0) {
            // If map is empty, see if we've found the first marker.
            if (m_MultiConfig->marker_num == 0) {
                
                for (int i = 0; i < markerNum; i++) {
                    if (markerInfoCopy[i].idMatrix != -1 && markerInfoCopy[i].idMatrix == m_OriginMarkerUid) {
                        ARLOGi("Initing marker map with marker %d.\n", m_OriginMarkerUid);
                        ARdouble origin[3][4] = {{1.0, 0.0, 0.0, 0.0},  {0.0, 1.0, 0.0, 0.0},  {0.0, 0.0, 1.0, 0.0}};
                        arMultiAddOrUpdateSubmarker(m_MultiConfig, m_OriginMarkerUid, AR_MULTI_PATTERN_TYPE_MATRIX, m_markerWidth, origin, 0);
                    }
                }
            }
            
            // If map is not empty, calculate the pose of the multimarker in camera frame, i.e. trans_M_c.
            if (m_MultiConfig->marker_num > 0) {
                
                ARdouble multiErr;
                if (m_robustFlag) multiErr = arGetTransMatMultiSquareRobust(ar3DHandle, markerInfoCopy, markerNum, m_MultiConfig);
                else multiErr = arGetTransMatMultiSquare(ar3DHandle, markerInfoCopy, markerNum, m_MultiConfig, 1);
                if (m_MultiConfig->prevF != 0) {
                    
                    //ARLOGi("Got multimarker pose with err=%0.3f\n", multiErr);
                    //arUtilPrintTransMat(m_MultiConfig->trans);
                    memcpy(trans, m_MultiConfig->trans, sizeof(trans));

#if !HAVE_GTSAM
                    // Construct map by simple inter-marker pose estimation.
                    // This approach will result in accumulation of pose errors as estimates are chained from
                    // previously estimated markers. Also, absolute pose error increases with distance from the origin.
                    
                    // Get the pose of the camera in the multimarker frame, i.e. trans_c_M.
                    ARdouble trans_c_M[3][4];
                    arUtilMatInv(m_MultiConfig->trans, trans_c_M);
                    
                    // Now add or update all markers (except never update the origin marker).
                    // Get pose in camera frame of individual marker, i.e. trans_m_c, compose with trans_c_M
                    // to get pose in multimarker local coordinate system, i.e. trans_m_M.
                    
                    for (int i = 0; i < markerNum; i++) {
                        if (markerInfoCopy[i].idMatrix != -1 && markerInfoCopy[i].idMatrix != m_OriginMarkerUid) {
                            ARdouble trans_m_c[3][4];
                            ARdouble err = arGetTransMatSquare(ar3DHandle, &(markerInfoCopy[i]), m_markerWidth, trans_m_c);
                            if (err < m_maxErr) {
                                ARdouble trans_m_M[3][4];
                                arUtilMatMul(trans_c_M, trans_m_c, trans_m_M);
                                
                                int multi_marker_count_prev = m_MultiConfig->marker_num;
                                arMultiAddOrUpdateSubmarker(m_MultiConfig, markerInfoCopy[i].idMatrix, AR_MULTI_PATTERN_TYPE_MATRIX, m_markerWidth, trans_m_M, 0, 0);
                                if (m_MultiConfig->marker_num > multi_marker_count_prev) {
                                    ARLOGi("Added marker %d to map (now %d markers in map) with pose:\n", markerInfoCopy[i].idMatrix, m_MultiConfig->marker_num);
                                    arUtilPrintTransMat(trans_m_M);
                                }

                                //Save for later
                                arx_mapper::Marker marker;
                                marker.uid = markerInfoCopy[i].idMatrix;
                                memcpy(marker.trans, trans_m_M, sizeof(marker.trans));
                                lastMarkers.push_back(marker);
                            }
                        }
                    }
#else
                    // Construct map using GTSAM's iSAM2 (incremental smoothing and mapping v2).
                    // This results in pose error values which are minimised over the entire map.
                    
                    // Now add factors for our marker observations to the graph.
                    std::vector<arx_mapper::Marker> squareMarkers;
                    for (int i = 0; i < markerNum; i++) {
                        if (markerInfoCopy[i].idMatrix != -1) {
                            arx_mapper::Marker marker;
                            ARdouble err = arGetTransMatSquare(ar3DHandle, &(markerInfoCopy[i]), m_markerWidth, marker.trans);
                            if (err < m_maxErr) {
                                marker.uid = markerInfoCopy[i].idMatrix;
                                squareMarkers.push_back(marker);
                            }
                        }
                    }

                    //Now filter the multi-markers to make sure that all barcodes are visible
                    std::vector<arx_mapper::Marker> newMarkers;
                    for (int i = 0; i < squareMarkers.size(); i++) {
                        bool isOK = true;
                        bool isMulti = false;
                        int nIndex = -1;

                        std::vector<ARTrackable*>::iterator it;
                        for (it = trackables.begin(); it != trackables.end(); ++it) {
                            if ((*it)->type == ARTrackable::MULTI) {
                                for (int j = 0; j < ((ARTrackableMultiSquare*)(*it))->config->marker_num; j++) {
                                    int barcodeID = ((ARTrackableMultiSquare*)(*it))->config->marker[j].patt_id;
                                    if (barcodeID == squareMarkers.at(i).uid) {
                                        isMulti = true;
                                        nIndex = j;
                                        break;
                                    }
                                }
                                if (isMulti) {
                                    if (nIndex > 0) isOK = false;
                                    break;
                                }
                            }
                        }

                        /*if (isOK && isMulti) { //Make sure that all barcodes in this multi-marker are visible
                            bool allBarcodesVisible = true;
                            for (int j = 0; j < ((ARTrackableMultiSquare *)(*it))->config->marker_num; j++) {
                                int barcodeID = ((ARTrackableMultiSquare *)(*it))->config->marker[j].patt_id;
                                bool barcodeVisible = false;
                                for (int k = 0; k < markers.size(); k++) {
                                    if (markers.at(k).uid == barcodeID) {
                                        barcodeVisible = true;
                                        break;
                                    }
                                }
                                if (!barcodeVisible) {
                                    allBarcodesVisible = false;
                                    break;
                                }
                            }
                            if (!allBarcodesVisible) isOK = false;
                        }*/

                        if (isOK) {
                            newMarkers.push_back(squareMarkers.at(i));
                        }
                    }

                    if (newMarkers.size() > 0) {
                        // Add a pose estimate to the graph.
                        m_pm->m_mapper.AddPose(m_MultiConfig->trans);

                        m_pm->m_mapper.AddFactors(newMarkers);

                        // Do mapping.
                        if (!m_pm->m_mapper.inited()) {
                            // Add a landmark for the origin marker.
                            // We fix this in the map at the origin and thus fix the scale for first pose and first landmark.
                            m_pm->m_mapper.Initialize(m_OriginMarkerUid, m_markerWidth);
                        }
                        else {
                            // This will add new landmarks for each marker not previously seen, with the
                            // initial pose estimate calculated from the marker pose in the camera frame
                            // composed with the camera pose in the map frame.
                            m_pm->m_mapper.AddLandmarks(squareMarkers);
                            m_pm->m_mapper.Optimize();
                            // Get latest estimates from mapper and put into map.
                            m_pm->m_mapper.Update(m_MultiConfig);
                            // Prepare for next iteration.
                            m_pm->m_mapper.Clear();
                        }
                    } //if (newMarkers.size() > 0)
#endif // HAVE_GTSAM
                    
                } // m_MultiConfig->prevF != 0
            } // m_MultiConfig->marker_num > 0

        } // goodMarkers > 0
        
        free(markerInfoCopy);
    } // markerInfo && markerNum > 0
    
    return (ARTrackable::update()); // Parent class will finish update.
}

bool ARTrackableMultiSquareAuto::updateWithDetectedMarkers(ARMarkerInfo* markerInfo, int markerNum, AR3DHandle* ar3DHandle, int lowRes)
{ // lowRes included purely for consistency with the other trackables. This method is not called when in lowRes mode.
    visiblePrev = visible;

    if (markerInfo) {
        ARdouble err;
        if (m_MultiConfig->marker_num < 2) {
            m_MultiConfig->min_submarker = 1;
        }
        else if (m_MultiConfig->marker_num < 3) {
            m_MultiConfig->min_submarker = 2;
        }
        else if (m_MultiConfig->marker_num < 4) {
            m_MultiConfig->min_submarker = 2;
        }
        else {
            m_MultiConfig->min_submarker = 2;
        }
        if (m_robustFlag) {
            err = arGetTransMatMultiSquareRobust(ar3DHandle, markerInfo, markerNum, m_MultiConfig);
            //ARLOGe("arGetTransMatMultiSquareRobust %f", err);
        }
        else {
            err = arGetTransMatMultiSquare(ar3DHandle, markerInfo, markerNum, m_MultiConfig, 0);
        }
        // Marker is visible if a match was found.
        if (m_MultiConfig->prevF != 0) {
            visible = true;

            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 4; k++)
                    trans[j][k] = m_MultiConfig->trans[j][k];
        }
        else visible = false;

    }
    else visible = false;
    //ARLOGd("updateWithDetectedMarkers: [visible: %d]\n", visible);
    return (ARTrackable::update()); // Parent class will finish update.
}

#if HAVE_GTSAM
bool ARTrackableMultiSquareAuto::updateWithDetectedMarkers2(std::vector<arx_mapper::Marker> markers, AR3DHandle* ar3DHandle)
{
    visiblePrev = visible;

    ARdouble err;
    if (m_MultiConfig->marker_num < 2) {
        m_MultiConfig->min_submarker = 1;
    }
    else if (m_MultiConfig->marker_num < 3) {
        m_MultiConfig->min_submarker = 2;
    }
    else if (m_MultiConfig->marker_num < 4) {
        m_MultiConfig->min_submarker = 2;
    }
    else {
        m_MultiConfig->min_submarker = 2;
    }
    //ARLOGd("updateWithDetectedMarkers2 m_MultiConfig->prevF = %d \n", m_MultiConfig->prevF);
    err = GetTransMatMultiSquare(markers, ar3DHandle);
    //ARLOGd("updateWithDetectedMarkers2 m_MultiConfig->prevF = %d \n", m_MultiConfig->prevF);

    // Marker is visible if a match was found.
    if (m_MultiConfig->prevF != 0) {
        visible = true;

        for (int j = 0; j < 3; j++) for (int k = 0; k < 4; k++) trans[j][k] = m_MultiConfig->trans[j][k];
    }
    else visible = false;


    return (ARTrackable::update()); // Parent class will finish update.
}

bool ARTrackableMultiSquareAuto::isVisible(std::vector<arx_mapper::Marker> markers, AR3DHandle* ar3DHandle)
{
    bool visibleRes = false;

    ARdouble err;
    if (m_MultiConfig->marker_num < 2) {
        m_MultiConfig->min_submarker = 1;
    }
    else if (m_MultiConfig->marker_num < 3) {
        m_MultiConfig->min_submarker = 2;
    }
    else if (m_MultiConfig->marker_num < 4) {
        m_MultiConfig->min_submarker = 2;
    }
    else {
        m_MultiConfig->min_submarker = 2;
    }
    err = GetTransMatMultiSquare(markers, ar3DHandle);

    if (m_MultiConfig->prevF != 0) {
        visibleRes = true;
    }

    return visibleRes;
}

ARdouble ARTrackableMultiSquareAuto::GetTransMatMultiSquare(std::vector<arx_mapper::Marker> markers, AR3DHandle* ar3DHandle)
{
    ARdouble* pos2d, * pos3d;
    ARdouble              trans1[3][4], trans2[3][4];
    ARdouble              maxLength;
    int                   max;
    int                   vnum;
    int                   i, j, k, k1;
    //char  mes[12];
    //ARLOGd("-- Pass2--\n");
    vnum = 0;
    for (i = 0; i < m_MultiConfig->marker_num; i++) {
        m_MultiConfig->marker[i].visible = -1;
        k = -1;
        for (const arx_mapper::Marker& marker : markers) {
            k = k + 1;
            if (marker.uid == m_MultiConfig->marker[i].patt_id) {
                m_MultiConfig->marker[i].visible = k;
                for (j = 0; j < 3; j++) {
                    for (int k1 = 0; k1 < 4; k1++) trans2[j][k1] = marker.trans[j][k1];
                }
                break;
            }
        }
        if (m_MultiConfig->marker[i].visible == -1) continue;

        ARdouble x1, y1, x2, y2, x3, y3, x4, y4;
        x1 = markers.at(k).corners[0];
        y1 = markers.at(k).corners[1];
        x2 = markers.at(k).corners[2];
        y2 = markers.at(k).corners[3];
        x3 = markers.at(k).corners[4];
        y3 = markers.at(k).corners[5];
        x4 = markers.at(k).corners[6];
        y4 = markers.at(k).corners[7];

        ARdouble length = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)) + sqrt((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2)) + sqrt((x4 - x3) * (x4 - x3) + (y4 - y3) * (y4 - y3)) + sqrt((x1 - x4) * (x1 - x4) + (y1 - y4) * (y1 - y4));

        // Use the largest (in terms of 2D coordinates) marker's pose estimate as the
        // input for the initial estimate for the pose estimator.
        //if (vnum == 0 || maxArea < marker_info[j].area || (marker_info[j].area > maxArea / 2 && marker_info[j].area / err > maxArea / maxErr)) {
        if (vnum == 0 || maxLength < length) {
            maxLength = length;
            max = i;
            for (j = 0; j < 3; j++) {
                for (k1 = 0; k1 < 4; k1++) trans1[j][k1] = trans2[j][k1];
            }
        }
        vnum++;
    }
    if (vnum == 0 || vnum < m_MultiConfig->min_submarker) {
        m_MultiConfig->prevF = 0;
        return -1;
    }
    arUtilMatMul((const ARdouble(*)[4])trans1, (const ARdouble(*)[4])m_MultiConfig->marker[max].itrans, trans2);

    arMalloc(pos2d, ARdouble, vnum * 4 * 2);
    arMalloc(pos3d, ARdouble, vnum * 4 * 3);

    j = 0;
    int n1 = 0;
    int n2 = 0;
    for (i = 0; i < m_MultiConfig->marker_num; i++) {
        if ((k = m_MultiConfig->marker[i].visible) < 0) continue;

        pos2d[n1 + 0] = markers.at(k).corners[0];
        pos2d[n1 + 1] = markers.at(k).corners[1];
        pos2d[n1 + 2] = markers.at(k).corners[2];
        pos2d[n1 + 3] = markers.at(k).corners[3];
        pos2d[n1 + 4] = markers.at(k).corners[4];
        pos2d[n1 + 5] = markers.at(k).corners[5];
        pos2d[n1 + 6] = markers.at(k).corners[6];
        pos2d[n1 + 7] = markers.at(k).corners[7];
        //ARLOGd("{Marker Num: %d 2D Data}:\n[x: %f y: %f]\n[x: %f y: %f]\n[x: %f y: %f]\n[x: %f y: %f]\n",
        //       i, pos2d[n1 + 0], pos2d[n1 + 1], pos2d[n1 + 2], pos2d[n1 + 3], pos2d[n1 + 4], pos2d[n1 + 5], pos2d[n1 + 6], pos2d[n1 + 7]);
        n1 = n1 + 8;
        pos3d[n2 + 0] = m_MultiConfig->marker[i].pos3d[0][0];
        pos3d[n2 + 1] = m_MultiConfig->marker[i].pos3d[0][1];
        pos3d[n2 + 2] = m_MultiConfig->marker[i].pos3d[0][2];
        pos3d[n2 + 3] = m_MultiConfig->marker[i].pos3d[1][0];
        pos3d[n2 + 4] = m_MultiConfig->marker[i].pos3d[1][1];
        pos3d[n2 + 5] = m_MultiConfig->marker[i].pos3d[1][2];
        pos3d[n2 + 6] = m_MultiConfig->marker[i].pos3d[2][0];
        pos3d[n2 + 7] = m_MultiConfig->marker[i].pos3d[2][1];
        pos3d[n2 + 8] = m_MultiConfig->marker[i].pos3d[2][2];
        pos3d[n2 + 9] = m_MultiConfig->marker[i].pos3d[3][0];
        pos3d[n2 + 10] = m_MultiConfig->marker[i].pos3d[3][1];
        pos3d[n2 + 11] = m_MultiConfig->marker[i].pos3d[3][2];
        //ARLOGd("{Marker Num: %d 3D Data}:\n[x: %f y: %f z: %f]\n[x: %f y: %f z: %f]\n[x: %f y: %f z: %f]\n[x: %f y: %f z: %f]\n", i, pos3d[n2 + 0], pos3d[n2 + 1], pos3d[n2 + 2], pos3d[n2 + 3], pos3d[n2 + 4], pos3d[n2 + 5], pos3d[n2 + 6], pos3d[n2 + 7], pos3d[n2 + 8], pos3d[n2 + 9], pos3d[n2 + 10], pos3d[n2 + 11]);
        
        n2 = n2 + 12;
        
//        ARLOGd("Marker %d trans:\n", m_MultiConfig->marker[i].patt_id);
//        for (int z = 0; z < 3; z++) {
//            ARLOGd("%f %f %f %f\n", m_MultiConfig->marker[i].trans[z][0], m_MultiConfig->marker[i].trans[z][1], m_MultiConfig->marker[i].trans[z][2], m_MultiConfig->marker[i].trans[z][3]);
//        }
        j++;
    }
    
    //ARLOGd("");
    
    ARdouble maxDeviation = AR_MULTI_POSE_ERROR_CUTOFF_COMBINED_DEFAULT;

    if (vnum <= 2) {
        maxDeviation = 5.0;
    }
    else if (vnum <= 4) {
        maxDeviation = 15.0;
    }
    else {
        //maxDeviation = 120.0;
        maxDeviation = 500.0; //I'm finding that a bad measurement on the image before this one can cause problems.
    }
//    ARLOGd("trans2\n");
//    for (j = 0; j < 3; j++) {
//        ARLOGd("%f %f %f %f\n", trans2[j][0], trans2[j][1], trans2[j][2], trans2[j][3]);
//    }
//    ARLOGd("m_MultiConfig->trans\n");
//    for (j = 0; j < 3; j++) {
//        ARLOGd("%f %f %f %f\n", m_MultiConfig->trans[j][0], m_MultiConfig->trans[j][1], m_MultiConfig->trans[j][2], m_MultiConfig->trans[j][3]);
//    }
    
    ARdouble err = arGetTransMat(ar3DHandle, trans2, (ARdouble(*)[2])pos2d, (ARdouble(*)[3])pos3d, vnum * 4, m_MultiConfig->trans);
    free(pos3d);
    free(pos2d);
    //ARLOGd("ARTrackableMultiSquareAuto::GetTransMatMultiSquare called: err: %f, maxDeviation: %f \n", err, maxDeviation);
    if (err < maxDeviation) {
        m_MultiConfig->prevF = 1;
    }
    else {
        m_MultiConfig->prevF = 0;
    }

    return err;
}

#endif

void ARTrackableMultiSquareAuto::initialiseWithSquareTrackable(ARTrackableSquare* trackable) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            m_MultiConfig->trans[i][j] = trackable->GetTrans(i, j);
            trans[i][j] = trackable->GetTrans(i, j);
        }
    }

    ARdouble origin[3][4] = { {1.0, 0.0, 0.0, 0.0},  {0.0, 1.0, 0.0, 0.0},  {0.0, 0.0, 1.0, 0.0} };
    arMultiAddOrUpdateSubmarker(m_MultiConfig, trackable->patt_id, AR_MULTI_PATTERN_TYPE_MATRIX, m_markerWidth, origin, 0);

    visible = true;
}

void ARTrackableMultiSquareAuto::initialiseWithMultiSquareTrackable(ARTrackableMultiSquare *trackable) {
    ARMultiMarkerInfoT* map = trackable->config;
    m_pm->m_mapper.pose_cnt = 0;
    for (int i = 0; i < map->marker_num; i++) {
        ARdouble origin[3][4];
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 4; k++) {
                origin[j][k] = map->marker[i].trans[j][k];
            }
        }
        ARLOGd("initialiseWithMultiSquareTrackable, m_markerWidth = %f\n", m_markerWidth);
        arMultiAddOrUpdateSubmarker(m_MultiConfig, map->marker[i].patt_id, AR_MULTI_PATTERN_TYPE_MATRIX, m_markerWidth, origin, 0);

        if (map->marker[i].patt_id == m_OriginMarkerUid) {
            ARdouble trans2[3][4];
            ARdouble trans3[3][4];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 4; j++) {
                    trans2[i][j] = trackable->GetTrans(i, j);
                }
            }
            arUtilMatMul(trans2, map->marker[i].trans, trans3);

            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 4; k++) {
                    m_MultiConfig->trans[j][k] = trans3[j][k];
                    trans[j][k] = trans3[j][k];
                }
            }

        }
    }
    visible = true;
}

bool ARTrackableMultiSquareAuto::updateMapperWithMarkers(std::vector<arx_mapper::Marker> markers) {

    // If map is not empty, calculate the pose of the multimarker in camera frame, i.e. trans_M_c.
    if (m_MultiConfig->marker_num > 0) {

#if !HAVE_GTSAM
        // Construct map by simple inter-marker pose estimation.
        // This approach will result in accumulation of pose errors as estimates are chained from
        // previously estimated markers. Also, absolute pose error increases with distance from the origin.

        // Get the pose of the camera in the multimarker frame, i.e. trans_c_M.
        ARdouble trans_c_M[3][4];
        arUtilMatInv(m_MultiConfig->trans, trans_c_M);

        // Now add or update all markers (except never update the origin marker).
        // Get pose in camera frame of individual marker, i.e. trans_m_c, compose with trans_c_M
        // to get pose in multimarker local coordinate system, i.e. trans_m_M.

        for (int i = 0; i < markers.size(); i++) {
            if (markers.at(i).uid != m_OriginMarkerUid) {
                ARdouble trans_m_c[3][4];
                for (int i1 = 0; i1 < 3; i1++) {
                    for (int j = 0; j < 4; j++) {
                        trans_m_c[i1][j] = (ARdouble)markers.at(i).trans[i1][j];
                    }
                }

                ARdouble trans_m_M[3][4];
                arUtilMatMul(trans_c_M, trans_m_c, trans_m_M);

                int multi_marker_count_prev = m_MultiConfig->marker_num;
                arMultiAddOrUpdateSubmarker(m_MultiConfig, markers.at(i).uid, AR_MULTI_PATTERN_TYPE_MATRIX, m_markerWidth, trans_m_M, 0, 0);
                if (m_MultiConfig->marker_num > multi_marker_count_prev) {
                    ARLOGi("Added marker %d to map (now %d markers in map) with pose:\n", markers.at(i).uid, m_MultiConfig->marker_num);
                }
            }
        }
#else
        // Construct map using GTSAM's iSAM2 (incremental smoothing and mapping v2).
        // This results in pose error values which are minimised over the entire map.

        // Add a pose estimate to the graph.
        m_pm->m_mapper.AddPose(m_MultiConfig->trans);

        // Now add factors for our marker observations to the graph.
        m_pm->m_mapper.AddFactors(markers);

        if (!m_pm->m_mapper.inited()) {
            // Add a landmark for the origin marker.
            // We fix this in the map at the origin and thus fix the scale for first pose and first landmark.
            m_pm->m_mapper.Initialize(m_OriginMarkerUid, m_markerWidth);
        }
        else {
            // This will add new landmarks for each marker not previously seen, with the
            // initial pose estimate calculated from the marker pose in the camera frame
            // composed with the camera pose in the map frame.
            m_pm->m_mapper.AddLandmarks(markers);
            m_pm->m_mapper.Optimize();
            // Get latest estimates from mapper and put into map.
            m_pm->m_mapper.Update(m_MultiConfig);
            // Prepare for next iteration.
            m_pm->m_mapper.Clear();
        }

#endif // HAVE_GTSAM

    } // markerInfo && markerNum > 0
    return (ARTrackable::update()); // Parent class will finish update.
}

bool ARTrackableMultiSquareAuto::updateWithDetectedMarkersStereo(ARMarkerInfo* markerInfoL, int markerNumL, int videoWidthL, int videoHeightL, ARMarkerInfo* markerInfoR, int markerNumR, int videoWidthR, int videoHeightR, AR3DStereoHandle *handle, ARdouble transL2R[3][4]) {
    
    ARLOGd("ARTrackableMultiSquareAuto::updateWithDetectedMarkersStereo(...)\n");
    
    visiblePrev = visible;
    visible = false;
    
    if (markerInfoL && markerNumL > 0 && markerInfoR && markerNumR > 0) {
        // TODO: stereo implementation.
    } // markerInfoL && markerNumL > 0 && markerInfoR && markerNumR > 0
    
    return (ARTrackable::update(transL2R)); // Parent class will finish update.
}

ARMultiMarkerInfoT *ARTrackableMultiSquareAuto::copyMultiConfig()
{
    return arMultiCopyConfig(m_MultiConfig);
}


bool ARTrackableMultiSquareAuto::updateWithDetectedDatums(ARParam arParams, ARUint8* buffLuma, int imageWidth, int imageHeight, AR3DHandle* ar3DHandle) {

    ARdouble* datumCoords2D;
    ARdouble* datumCoords;
    ARdouble trans2[3][4];

    cv::Mat grayImage = cv::Mat(imageHeight, imageWidth, CV_8UC1, (void*)buffLuma, imageWidth);

    int vnum = 0;
    std::vector<cv::Point2f> corners;
    std::vector<cv::Point3f> datumCoords3D;
    for (int i = 0; i < m_MultiConfig->marker_num; i++) {
        if (m_MultiConfig->marker[i].visible < 0) continue;

        std::vector<cv::Point2f> datumCentres;
        if (m_MultiConfig->marker[i].patt_id == 0 || m_MultiConfig->marker[i].patt_id == 1) {
            datumCentres.push_back(cv::Point2f(-128.5, 85));
            datumCentres.push_back(cv::Point2f(-128.5, -85));
            datumCentres.push_back(cv::Point2f(128.5, 85));
            datumCentres.push_back(cv::Point2f(128.5, -85));
        }
        else {
            datumCentres.push_back(cv::Point2f(-55, 30));
            datumCentres.push_back(cv::Point2f(-55, -30));
            datumCentres.push_back(cv::Point2f(55, 30));
            datumCentres.push_back(cv::Point2f(55, -30));
        }

        arUtilMatMul((const ARdouble(*)[4])trans, (const ARdouble(*)[4])m_MultiConfig->marker[i].trans, trans2);

        ARdouble ox, oy;
        for (int j = 0; j < (int)datumCentres.size(); j++) {
            cv::Point2f pt = datumCentres.at(j);

            if (GetCenterPointForDatum(pt.x, pt.y, arParams, trans2, grayImage, imageWidth, imageHeight, &ox, &oy)) {
                corners.push_back(cv::Point2f(ox, oy));
                float cx = m_MultiConfig->marker[i].trans[0][0] * pt.x + m_MultiConfig->marker[i].trans[0][1] * pt.y + m_MultiConfig->marker[i].trans[0][3];
                float cy = m_MultiConfig->marker[i].trans[1][0] * pt.x + m_MultiConfig->marker[i].trans[1][1] * pt.y + m_MultiConfig->marker[i].trans[1][3];
                float cz = m_MultiConfig->marker[i].trans[2][0] * pt.x + m_MultiConfig->marker[i].trans[2][1] * pt.y + m_MultiConfig->marker[i].trans[2][3];
                datumCoords3D.push_back(cv::Point3f(cx, cy, cz));
            }
        }

        datumCentres.clear();
        datumCentres.push_back(cv::Point2f(-40, 40));
        datumCentres.push_back(cv::Point2f(-40, -40));
        datumCentres.push_back(cv::Point2f(40, -40));
        datumCentres.push_back(cv::Point2f(40, 40));
        for (int j = 0; j < 4; j++) {
            cv::Point2f pt = datumCentres.at(j);
            ModelToImageSpace(arParams, trans2, pt.x, pt.y, &ox, &oy);
            corners.push_back(cv::Point2f(ox, oy));
            float cx = m_MultiConfig->marker[i].trans[0][0] * pt.x + m_MultiConfig->marker[i].trans[0][1] * pt.y + m_MultiConfig->marker[i].trans[0][3];
            float cy = m_MultiConfig->marker[i].trans[1][0] * pt.x + m_MultiConfig->marker[i].trans[1][1] * pt.y + m_MultiConfig->marker[i].trans[1][3];
            float cz = m_MultiConfig->marker[i].trans[2][0] * pt.x + m_MultiConfig->marker[i].trans[2][1] * pt.y + m_MultiConfig->marker[i].trans[2][3];
            datumCoords3D.push_back(cv::Point3f(cx, cy, cz));
        }

        vnum++;
    }

    qrMarkerCornerPointsInPixels.clear();
    if (vnum >= m_MultiConfig->min_submarker) {
        int nCorners = corners.size();
        arMalloc(datumCoords2D, ARdouble, nCorners * 2);
        arMalloc(datumCoords, ARdouble, nCorners * 3);

        std::vector<cv::Point2f> cornersCopy;
        for (int i = 0; i < nCorners; i++) {
            cornersCopy.push_back(cv::Point2f(corners.at(i).x, corners.at(i).y));
        }
        cv::cornerSubPix(grayImage, corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 0.1));
        for (int i = 0; i < nCorners; i++) {
            ARdouble ix, iy;
            ARdouble ox, oy;
            double d = sqrt((cornersCopy.at(i).x - corners.at(i).x) * (cornersCopy.at(i).x - corners.at(i).x) + (cornersCopy.at(i).y - corners.at(i).y) * (cornersCopy.at(i).y - corners.at(i).y));
            if (d < 3.0) {
                ix = corners.at(i).x;
                iy = corners.at(i).y;
            }
            else {
                ix = cornersCopy.at(i).x;
                iy = cornersCopy.at(i).y;
            }
            qrMarkerCornerPointsInPixels.push_back(cv::Point2f(ix, iy));
            arParamObserv2Ideal(arParams.dist_factor, ix, iy, &ox, &oy, arParams.dist_function_version);
            datumCoords2D[i * 2] = ox;
            datumCoords2D[i * 2 + 1] = oy;
            datumCoords[i * 3] = datumCoords3D[i].x;
            datumCoords[i * 3 + 1] = datumCoords3D[i].y;
            datumCoords[i * 3 + 2] = datumCoords3D[i].z;
        }

        ARdouble err;
        ARdouble maxDeviation = AR_MULTI_POSE_ERROR_CUTOFF_COMBINED_DEFAULT;
        if (vnum < 2) {
            maxDeviation = 20.0;
        }
        else if (vnum < 3) {
            maxDeviation = 40.0;
        }
        else if (vnum < 4) {
            maxDeviation = 60.0;
        }
        else {
            maxDeviation = 120.0;
        }
        if (vnum == 1) { //Planar
            err = arGetTransMatDatum(ar3DHandle, datumCoords2D, datumCoords, nCorners, m_MultiConfig->trans);
        }
        else {
            err = arGetTransMat(ar3DHandle, trans, (ARdouble(*)[2])datumCoords2D, (ARdouble(*)[3]) datumCoords, nCorners, m_MultiConfig->trans);
        }
        if (err > maxDeviation) visible = false;

        for (int j = 0; j < 3; j++) for (int k = 0; k < 4; k++) trans[j][k] = m_MultiConfig->trans[j][k];
        free(datumCoords2D);
        free(datumCoords);
    }
    else {
        visible = false;
    }

    if (visible) return (ARTrackable::update()); // Parent class will finish update.
    return false;
}
