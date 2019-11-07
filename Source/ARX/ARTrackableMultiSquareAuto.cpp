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

ARTrackableMultiSquareAuto::ARTrackableMultiSquareAuto() : ARTrackable(MULTI_AUTO),
    m_OriginMarkerUid(0),
    m_markerWidth(80.0f),
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
    m_OriginMarkerUid = originMarkerUID;
    m_markerWidth = markerWidth;
    return true;
}

bool ARTrackableMultiSquareAuto::updateMapper(ARMarkerInfo* markerInfo, int markerNum, int videoWidth, int videoHeight, AR3DHandle *ar3DHandle, std::vector<ARTrackable*>& trackables) {
    
    ARLOGd("ARTrackableMultiSquareAuto::updateWithDetectedMarkers(...)\n");
    
    visiblePrev = visible;
    visible = false;;
	lastUpdateSuccessful = false;
	lastMarkers.clear();
    
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
                else multiErr = arGetTransMatMultiSquare(ar3DHandle, markerInfoCopy, markerNum, m_MultiConfig);
                if (m_MultiConfig->prevF != 0) {
                    
                    //ARLOGi("Got multimarker pose with err=%0.3f\n", multiErr);
                    //arUtilPrintTransMat(m_MultiConfig->trans);
                    memcpy(trans, m_MultiConfig->trans, sizeof(trans));
                    visible = true;
					memcpy(lastTrans, m_MultiConfig->trans, sizeof(lastTrans));
					lastUpdateSuccessful = true;
					numSuccessfulUpdates = numSuccessfulUpdates + 1;

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
                                arMultiAddOrUpdateSubmarker(m_MultiConfig, markerInfoCopy[i].idMatrix, AR_MULTI_PATTERN_TYPE_MATRIX, m_markerWidth, trans_m_M, 0);
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
							lastMarkers.push_back(squareMarkers.at(i));
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

bool ARTrackableMultiSquareAuto::updateWithDetectedMarkers(ARMarkerInfo* markerInfo, int markerNum, AR3DHandle* ar3DHandle)
{
	visiblePrev = visible;

	if (markerInfo) {

		ARdouble err;

		if (m_robustFlag) {
			err = arGetTransMatMultiSquareRobust(ar3DHandle, markerInfo, markerNum, m_MultiConfig);
		}
		else {
			err = arGetTransMatMultiSquare(ar3DHandle, markerInfo, markerNum, m_MultiConfig);
		}

		// Marker is visible if a match was found.
		if (m_MultiConfig->prevF != 0) {
			visible = true;

			for (int j = 0; j < 3; j++) for (int k = 0; k < 4; k++) trans[j][k] = m_MultiConfig->trans[j][k];
		}
		else visible = false;

	}
	else visible = false;

	return (ARTrackable::update()); // Parent class will finish update.
}

bool ARTrackableMultiSquareAuto::updateMapperWithMarkers(std::vector<arx_mapper::Marker> markers) {

	visiblePrev = visible;
	visible = true;
	lastUpdateSuccessful = false;
	lastMarkers.clear();

	// If map is empty, see if we've found the first marker.
	if (m_MultiConfig->marker_num == 0) {
		for (int i = 0; i < (int)markers.size(); i++) {
			if (markers.at(i).uid == m_OriginMarkerUid) {
				ARdouble origin[3][4] = { {1.0, 0.0, 0.0, 0.0},  {0.0, 1.0, 0.0, 0.0},  {0.0, 0.0, 1.0, 0.0} };
				arMultiAddOrUpdateSubmarker(m_MultiConfig, m_OriginMarkerUid, AR_MULTI_PATTERN_TYPE_MATRIX, m_markerWidth, origin, 0);
				for (int j = 0; j < 3; j++) {
					for (int k = 0; k < 4; k++) {
						m_MultiConfig->trans[j][k] = markers.at(i).trans[j][k];
					}
				}
			}
		}
	}

	// If map is not empty, calculate the pose of the multimarker in camera frame, i.e. trans_M_c.
	if (m_MultiConfig->marker_num > 0) {

		lastUpdateSuccessful = true;
		numSuccessfulUpdates = numSuccessfulUpdates + 1;

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
				arMultiAddOrUpdateSubmarker(m_MultiConfig, markers.at(i).uid, AR_MULTI_PATTERN_TYPE_MATRIX, m_markerWidth, trans_m_M, 0);
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
		std::vector<arx_mapper::Marker> newMarkers;
		for (int i = 0; i < (int)markers.size(); i++) {
			newMarkers.push_back(markers.at(i));
		}

		// Now add factors for our marker observations to the graph.
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

void ARTrackableMultiSquareAuto::addStoredMarkers(float thisTrans[12], std::vector<arx_mapper::Marker> markers) {

	visiblePrev = visible;
	visible = true;
	lastUpdateSuccessful = false;
	lastMarkers.clear();

	// If map is empty, see if we've found the first marker.
	if (m_MultiConfig->marker_num == 0) {
		for (int i = 0; i < (int)markers.size(); i++) {
			if (markers.at(i).uid == m_OriginMarkerUid) {
				ARdouble origin[3][4] = { {1.0, 0.0, 0.0, 0.0},  {0.0, 1.0, 0.0, 0.0},  {0.0, 0.0, 1.0, 0.0} };
				arMultiAddOrUpdateSubmarker(m_MultiConfig, m_OriginMarkerUid, AR_MULTI_PATTERN_TYPE_MATRIX, m_markerWidth, origin, 0);
			}
		}
	}

	// If map is not empty, calculate the pose of the multimarker in camera frame, i.e. trans_M_c.
	if (m_MultiConfig->marker_num > 0) {

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				trans[i][j] = (ARdouble)thisTrans[i * 4 + j];
				m_MultiConfig->trans[i][j] = (ARdouble)thisTrans[i * 4 + j];
			}
		}

		lastUpdateSuccessful = true;
		numSuccessfulUpdates = numSuccessfulUpdates + 1;

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
				arMultiAddOrUpdateSubmarker(m_MultiConfig, markers.at(i).uid, AR_MULTI_PATTERN_TYPE_MATRIX, m_markerWidth, trans_m_M, 0);
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
		std::vector<arx_mapper::Marker> newMarkers;
		for (int i = 0; i < (int)markers.size(); i++) {
			newMarkers.push_back(markers.at(i));
		}

		// Now add factors for our marker observations to the graph.
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
			m_pm->m_mapper.AddLandmarks(markers);
			m_pm->m_mapper.Optimize();
			// Get latest estimates from mapper and put into map.
			m_pm->m_mapper.Update(m_MultiConfig);
			// Prepare for next iteration.
			m_pm->m_mapper.Clear();
		}

#endif // HAVE_GTSAM

	} // markerInfo && markerNum > 0
	ARTrackable::update();
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

	for (int i = 0; i < m_MultiConfig->marker_num; i++) {
		if (m_MultiConfig->marker[i].visible < 0) continue;
	}

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
				vnum++;
			}
		}
	}

	if (vnum >= 4) {

		arMalloc(datumCoords2D, ARdouble, vnum * 4 * 2);
		arMalloc(datumCoords, ARdouble, vnum * 4 * 3);

		cv::cornerSubPix(grayImage, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER, 100, 0.1));
		for (int i = 0; i < vnum; i = i + 1) {
			datumCoords2D[i * 2] = corners[i].x;
			datumCoords2D[i * 2 + 1] = corners[i].y;
			datumCoords[i * 3] = datumCoords3D[i].x;
			datumCoords[i * 3 + 1] = datumCoords3D[i].y;
			datumCoords[i * 3 + 2] = datumCoords3D[i].z;
		}

		ARdouble err;
		err = arGetTransMatDatumSquare(ar3DHandle, datumCoords2D, datumCoords, vnum, trans);
		if (err > 10.0f) visible = false;

		free(datumCoords2D);
		free(datumCoords);
	}
	else {
		visible = false;
	}

	if (visible) return (ARTrackable::update()); // Parent class will finish update.
	return false;
}

bool ARTrackableMultiSquareAuto::GetCenterPointForDatum(ARdouble x, ARdouble y, ARParam arParams, ARdouble trans[3][4], cv::Mat grayImage, int imageWidth, int imageHeight, ARdouble* ox, ARdouble* oy) {
	ModelToImageSpace(arParams, trans, x, y, ox, oy);
	int halfSquare = GetSquareForDatum(x, y, arParams, trans);
	if (halfSquare < 10) return false;
	if (*ox - halfSquare < 0 || *ox + halfSquare > imageWidth || *oy - halfSquare < 0 || *oy + halfSquare > imageHeight) return false;

	cv::Rect rect = cv::Rect((int)*ox - halfSquare, (int)*oy - halfSquare, 2 * halfSquare, 2 * halfSquare);
	cv::Mat region = cv::Mat(grayImage, rect);
	cv::Mat binaryRegion = region.clone();
	double otsuThreshold = cv::threshold(region, binaryRegion, 0.0, 255.0, CV_THRESH_OTSU);
	int nonzero = cv::countNonZero(binaryRegion);
	int square = 4 * halfSquare * halfSquare;
	return (nonzero > square * 0.333f && nonzero < square * 0.666f);
}

void ARTrackableMultiSquareAuto::ModelToImageSpace(ARParam param, ARdouble trans[3][4], ARdouble ix, ARdouble iy, ARdouble* ox, ARdouble* oy) {
	ARdouble        cx, cy, cz, hx, hy, h, sx, sy;

	*ox = ix;
	*oy = iy;

	cx = trans[0][0] * ix + trans[0][1] * iy + trans[0][3];
	cy = trans[1][0] * ix + trans[1][1] * iy + trans[1][3];
	cz = trans[2][0] * ix + trans[2][1] * iy + trans[2][3];
	hx = param.mat[0][0] * cx + param.mat[0][1] * cy + param.mat[0][2] * cz + param.mat[0][3];
	hy = param.mat[1][0] * cx + param.mat[1][1] * cy + param.mat[1][2] * cz + param.mat[1][3];
	h = param.mat[2][0] * cx + param.mat[2][1] * cy + param.mat[2][2] * cz + param.mat[2][3];
	if (h == 0.0) return;
	sx = hx / h;
	sy = hy / h;
	arParamIdeal2Observ(param.dist_factor, sx, sy, ox, oy, param.dist_function_version);
}

int ARTrackableMultiSquareAuto::GetSquareForDatum(ARdouble x, ARdouble y, ARParam arParams, ARdouble trans[3][4]) {
	ARdouble ox, oy, ox1, oy1, ox2, oy2, ox3, oy3, ox4, oy4;
	ModelToImageSpace(arParams, trans, x, y, &ox, &oy);
	ModelToImageSpace(arParams, trans, x - 8, y - 8, &ox1, &oy1);
	ModelToImageSpace(arParams, trans, x + 8, y - 8, &ox2, &oy2);
	ModelToImageSpace(arParams, trans, x + 8, y + 8, &ox3, &oy3);
	ModelToImageSpace(arParams, trans, x - 8, y + 8, &ox4, &oy4);
	ox1 = ox1 - ox;
	oy1 = oy1 - oy;
	ox2 = ox2 - ox;
	oy2 = oy2 - oy;
	ox3 = ox3 - ox;
	oy3 = oy3 - oy;
	ox4 = ox4 - ox;
	oy4 = oy4 - oy;

	ARdouble maxD = 100;

	ARdouble nx = oy1 - oy2;
	ARdouble ny = ox2 - ox1;
	ARdouble d = sqrt(nx * nx + ny * ny);
	if (d > 0) {
		nx = nx / d;
		ny = ny / d;
	}
	d = ox1 * nx + oy1 * ny;
	if (d > 10 && d < maxD) maxD = d;

	nx = oy2 - oy3;
	ny = ox3 - ox2;
	d = sqrt(nx * nx + ny * ny);
	if (d > 10) {
		nx = nx / d;
		ny = ny / d;
	}
	d = ox2 * nx + oy2 * ny;
	if (d > 10 && d < maxD) maxD = d;

	nx = oy3 - oy4;
	ny = ox4 - ox3;
	d = sqrt(nx * nx + ny * ny);
	if (d > 0) {
		nx = nx / d;
		ny = ny / d;
	}
	d = ox3 * nx + oy3 * ny;
	if (d > 10 && d < maxD) maxD = d;

	nx = oy4 - oy1;
	ny = ox1 - ox4;
	d = sqrt(nx * nx + ny * ny);
	if (d > 0) {
		nx = nx / d;
		ny = ny / d;
	}
	d = ox4 * nx + oy4 * ny;
	if (d > 10 && d < maxD) maxD = d;

	return (int)(maxD / sqrt(2.0));
}

ARdouble ARTrackableMultiSquareAuto::arGetTransMatDatumSquare(AR3DHandle* handle, ARdouble* datumCoords2D, ARdouble* datumCoords, const int numDatums, ARdouble conv[3][4])
{
	const int numCoords = 2 * numDatums;
	ICP2DCoordT* screenCoord = new ICP2DCoordT[numDatums];
	ICP3DCoordT* worldCoord = new ICP3DCoordT[numDatums];
	ICPDataT       data;
	ARdouble         initMatXw2Xc[3][4];
	ARdouble         err;

	for (int i = 0; i < numDatums; i++) {
		screenCoord[i].x = datumCoords2D[i * 2];
		screenCoord[i].y = datumCoords2D[i * 2 + 1];
		worldCoord[i].x = datumCoords[i * 3];
		worldCoord[i].y = datumCoords[i * 3 + 1];
		worldCoord[i].z = datumCoords[i * 3 + 2];
	}
	data.screenCoord = screenCoord;
	data.worldCoord = worldCoord;
	data.num = numDatums;

	if (icpGetInitXw2Xc_from_PlanarData(handle->icpHandle->matXc2U, data.screenCoord, data.worldCoord, data.num, initMatXw2Xc) < 0) return 100000000.0;
	if (icpPoint(handle->icpHandle, &data, initMatXw2Xc, conv, &err) < 0) return 100000000.0;

	delete[] screenCoord;
	delete[] worldCoord;

	return err;
}

