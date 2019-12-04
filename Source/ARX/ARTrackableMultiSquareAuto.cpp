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
			ARLOGe("arGetTransMatMultiSquareRobust %f", err);
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

bool ARTrackableMultiSquareAuto::updateWithDetectedMarkersOpenCV(ARMarkerInfo* markerInfo, int markerNum, AR3DHandle* ar3DHandle, ARHandle* arHandle)
{
	visiblePrev = visible;
	visible = false;
	int numVisible = 0;
	int numReqd = 1;
	if (m_MultiConfig->marker_num > 1) numReqd = 2;

	imagePoints.clear();
	if (markerInfo && m_MultiConfig->marker_num > 0) {

		ARdouble err;

		std::vector<cv::Point3f> cornerPoints;
		for (int i = 0; i < m_MultiConfig->marker_num; i++) {

			float ox, oy;
			for (int j = 0; j < markerNum; j++) {
				if (markerInfo[j].idMatrix == m_MultiConfig->marker[i].patt_id) {

					std::vector<cv::Point3f> cornerPoints2;
					cornerPoints2.push_back(cv::Point3f(-m_MultiConfig->marker[i].width / 2.0f, m_MultiConfig->marker[i].width / 2.0f, 0.0f));
					cornerPoints2.push_back(cv::Point3f(m_MultiConfig->marker[i].width / 2.0f, m_MultiConfig->marker[i].width / 2.0f, 0.0f));
					cornerPoints2.push_back(cv::Point3f(m_MultiConfig->marker[i].width / 2.0f, -m_MultiConfig->marker[i].width / 2.0f, 0.0f));
					cornerPoints2.push_back(cv::Point3f(-m_MultiConfig->marker[i].width / 2.0f, -m_MultiConfig->marker[i].width / 2.0f, 0.0f));

					for (int k = 0; k < 4; k++) {
						cv::Point3f pt = cornerPoints2.at(k);
						float cx = m_MultiConfig->marker[i].trans[0][0] * pt.x + m_MultiConfig->marker[i].trans[0][1] * pt.y + m_MultiConfig->marker[i].trans[0][3];
						float cy = m_MultiConfig->marker[i].trans[1][0] * pt.x + m_MultiConfig->marker[i].trans[1][1] * pt.y + m_MultiConfig->marker[i].trans[1][3];
						float cz = m_MultiConfig->marker[i].trans[2][0] * pt.x + m_MultiConfig->marker[i].trans[2][1] * pt.y + m_MultiConfig->marker[i].trans[2][3];

						cornerPoints.push_back(cv::Point3f(cx, cy, cz));
					}

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
					numVisible++;

					break;
				}
			}
		}

		if (numVisible >= numReqd) {
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
			cv::solvePnP(cornerPoints, imagePoints, cameraMatrix, distortionCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
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
			err = cv::norm(reprojectPoints, imagePoints) / sqrt((double)numVisible * 4.0);

			imagePoints.clear();
			for (int i = 0; i < reprojectPoints.size(); i++) {
				imagePoints.push_back(cv::Point2f(reprojectPoints.at(i).x, reprojectPoints.at(i).y));
			}

			if (err < 20.0f) visible = true;
		}
	}

	return (ARTrackable::update()); // Parent class will finish update.
}

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
	for (int i = 0; i < map->marker_num; i++) {
		ARdouble origin[3][4];
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 4; k++) {
				origin[j][k] = map->marker[i].trans[j][k];
			}
		}

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

void ARTrackableMultiSquareAuto::addStoredMarkers(float thisTrans[12], std::vector<arx_mapper::Marker> markers) {
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

	imagePoints.clear();
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
			imagePoints.push_back(cv::Point2f(ix, iy));
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
		ARdouble init[3][4];
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
