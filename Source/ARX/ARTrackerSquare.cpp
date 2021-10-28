/*
 *  ARTracker.cpp
 *  artoolkitX
 *
 *  A C++ class implementing the artoolkitX square fiducial marker tracker.
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

#include <ARX/ARTrackerSquare.h>
#include <ARX/ARTrackableSquare.h>
#include <ARX/ARTrackableMultiSquare.h>
#include <ARX/ARTrackableMultiSquareAuto.h>
#include <ARX/AR/ar.h>

ARTrackerSquare::ARTrackerSquare() :
    m_threshold(AR_DEFAULT_LABELING_THRESH),
    m_thresholdMode(AR_LABELING_THRESH_MODE_DEFAULT),
    m_imageProcMode(AR_DEFAULT_IMAGE_PROC_MODE),
    m_labelingMode(AR_DEFAULT_LABELING_MODE),
    m_pattRatio(AR_PATT_RATIO),
    m_patternDetectionMode(AR_DEFAULT_PATTERN_DETECTION_MODE),
    m_matrixCodeType(AR_MATRIX_CODE_TYPE_DEFAULT),
    m_debugMode(FALSE),
    m_cornerRefinementMode(FALSE),
    m_patternSize(AR_PATT_SIZE1),
    m_patternCountMax(AR_PATT_NUM_MAX),
    m_arHandle0(NULL),
    m_arHandle1(NULL),
    m_arPattHandle(NULL),
    m_ar3DHandle(NULL),
    m_ar3DStereoHandle(NULL)
{
    
}

ARTrackerSquare::~ARTrackerSquare()
{
    terminate();
}

bool ARTrackerSquare::initialize()
{
    // Create pattern handle so square template trackables can begin to be added.
    if ((m_arPattHandle = arPattCreateHandle2(m_patternSize, m_patternCountMax)) == NULL) {
        ARLOGe("Error: arPattCreateHandle2.\n");
        return false;
    }
    
    return true;
}

AR3DHandle* ARTrackerSquare::getAR3DHandle() {
    return m_ar3DHandle;
}

bool ARTrackerSquare::isRunning()
{
    return ((bool)m_arHandle0 && (!(bool)m_ar3DStereoHandle || (bool)m_arHandle1));
}

// ----------------------------------------------------------------------------------------------------
#pragma mark  Configuration functions
// If an arHandle is current, it will be updated. If no arHandle is current, the value will be
// set on initialisation (in initAR()). The value can be queried at any time.
// ----------------------------------------------------------------------------------------------------
void ARTrackerSquare::setDebugMode(bool debug)
{
    m_debugMode = debug;
    if (m_arHandle0) {
        arSetDebugMode(m_arHandle0, debug ? AR_DEBUG_ENABLE : AR_DEBUG_DISABLE);
        ARLOGi("Debug mode set to %s.\n", debug ? "on." : "off.");
    }
    if (m_arHandle1) {
        arSetDebugMode(m_arHandle1, debug ? AR_DEBUG_ENABLE : AR_DEBUG_DISABLE);
        ARLOGi("Debug mode set to %s.\n", debug ? "on." : "off.");
    }
}

bool ARTrackerSquare::debugMode() const
{
    return m_debugMode;
}

void ARTrackerSquare::setCornerRefinementMode(bool mode)
{
    m_cornerRefinementMode = mode;
    if (m_arHandle0) {
        arSetCornerRefinementMode(m_arHandle0, mode ? AR_CORNER_REFINEMENT_ENABLE : AR_CORNER_REFINEMENT_DISABLE);
    }
    if (m_arHandle1) {
        arSetCornerRefinementMode(m_arHandle1, mode ? AR_CORNER_REFINEMENT_ENABLE : AR_CORNER_REFINEMENT_DISABLE);
    }
    ARLOGi("Corner Refinement mode set to %d.\n", mode ? AR_CORNER_REFINEMENT_ENABLE : AR_CORNER_REFINEMENT_DISABLE);
    
}

bool ARTrackerSquare::cornerRefinementMode() const
{
    return m_cornerRefinementMode;
}

void ARTrackerSquare::setImageProcMode(int mode)
{
    m_imageProcMode = mode;
    
    if (m_arHandle0) {
        arSetImageProcMode(m_arHandle0, mode);
    }
    if (m_arHandle1) {
        arSetImageProcMode(m_arHandle1, mode);
     }
    ARLOGi("Image proc. mode set to %d.\n", m_imageProcMode);
}

int ARTrackerSquare::imageProcMode() const
{
    return m_imageProcMode;
}

void ARTrackerSquare::setThreshold(int thresh)
{
    if (thresh < 0 || thresh > 255) return;
    m_threshold = thresh;
    if (m_arHandle0) {
        arSetLabelingThresh(m_arHandle0, m_threshold);
    }
    if (m_arHandle1) {
        arSetLabelingThresh(m_arHandle1, m_threshold);
    }
    ARLOGi("Threshold set to %d\n", m_threshold);
}

int ARTrackerSquare::threshold() const
{
    return m_threshold;
}

void ARTrackerSquare::setThresholdMode(int mode)
{
    m_thresholdMode = (AR_LABELING_THRESH_MODE)mode;
    if (m_arHandle0) {
        arSetLabelingThreshMode(m_arHandle0, m_thresholdMode);
    }
    if (m_arHandle1) {
        arSetLabelingThreshMode(m_arHandle1, m_thresholdMode);
    }
    ARLOGi("Threshold mode set to %d\n", (int)m_thresholdMode);
}

int ARTrackerSquare::thresholdMode() const
{
    return (int)m_thresholdMode;
}

void ARTrackerSquare::setLabelingMode(int mode)
{
    m_labelingMode = mode;
    if (m_arHandle0) {
        arSetLabelingMode(m_arHandle0, m_labelingMode);
    }
    if (m_arHandle1) {
        arSetLabelingMode(m_arHandle1, m_labelingMode);
    }
    ARLOGi("Labeling mode set to %d\n", m_labelingMode);
}

int ARTrackerSquare::labelingMode() const
{
    return m_labelingMode;
}

void ARTrackerSquare::setPatternDetectionMode(int mode)
{
    m_patternDetectionMode = mode;
    if (m_arHandle0) {
        arSetPatternDetectionMode(m_arHandle0, m_patternDetectionMode);
        ARLOGi("Pattern detection mode set to %d.\n", m_patternDetectionMode);
    }
    if (m_arHandle1) {
        arSetPatternDetectionMode(m_arHandle1, m_patternDetectionMode);
        ARLOGi("Pattern detection mode set to %d.\n", m_patternDetectionMode);
    }
}

int ARTrackerSquare::patternDetectionMode() const
{
    return m_patternDetectionMode;
}

void ARTrackerSquare::setPattRatio(float ratio)
{
    if (ratio <= 0.0f || ratio >= 1.0f) return;
    m_pattRatio = (ARdouble)ratio;
    if (m_arHandle0) {
        arSetPattRatio(m_arHandle0, m_pattRatio);
        ARLOGi("Pattern ratio size set to %d.\n", m_pattRatio);
    }
    if (m_arHandle1) {
        arSetPattRatio(m_arHandle1, m_pattRatio);
        ARLOGi("Pattern ratio size set to %d.\n", m_pattRatio);
    }
}

float ARTrackerSquare::pattRatio() const
{
    return (float)m_pattRatio;
}

void ARTrackerSquare::setMatrixCodeType(int type)
{
    m_matrixCodeType = (AR_MATRIX_CODE_TYPE)type;
    if (m_arHandle0) {
        arSetMatrixCodeType(m_arHandle0, m_matrixCodeType);
        ARLOGi("Matrix code type set to %d.\n", m_matrixCodeType);
    }
    if (m_arHandle1) {
        arSetMatrixCodeType(m_arHandle1, m_matrixCodeType);
        ARLOGi("Matrix code type set to %d.\n", m_matrixCodeType);
    }
}

int ARTrackerSquare::matrixCodeType() const
{
    return (int)m_matrixCodeType;
}

void ARTrackerSquare::setPatternSize(int patternSize)
{
    if (!m_arPattHandle) return;
    if (m_arPattHandle->patt_num > 0) {
        ARLOGe("Attempt to set pattern size but patterns already loaded. Unload first and then retry.\n");
    }
    if (patternSize < 16 || patternSize > AR_PATT_SIZE1_MAX) {
        return;
        ARLOGe("Attempt to set pattern size to invalid value %d.\n", patternSize);
    }
    
    arPattDeleteHandle(m_arPattHandle);
    m_patternSize = patternSize;
    m_arPattHandle = arPattCreateHandle2(m_patternSize, m_patternCountMax);
}

int ARTrackerSquare::patternSize() const
{
    return m_patternSize;
}

void ARTrackerSquare::setPatternCountMax(int patternCountMax)
{
    if (!m_arPattHandle) return;
    if (m_arPattHandle->patt_num > 0) {
        ARLOGe("Attempt to set pattern count max but patterns already loaded. Unload first and then retry.\n");
    }
    if (patternCountMax > AR_PATT_NUM_MAX || patternCountMax <= 0) {
        ARLOGe("Attempt to set pattern count max to invalid value %d.\n", patternCountMax);
        return;
    }
    
    arPattDeleteHandle(m_arPattHandle);
    m_patternCountMax = patternCountMax;
    m_arPattHandle = arPattCreateHandle2(m_patternSize, m_patternCountMax);
}

int ARTrackerSquare::patternCountMax() const
{
    return m_patternCountMax;
}

bool ARTrackerSquare::start(ARParamLT *paramLT, AR_PIXEL_FORMAT pixelFormat)
{
    return start(paramLT, pixelFormat, NULL, AR_PIXEL_FORMAT_INVALID, NULL);
}

bool ARTrackerSquare::start(ARParamLT *paramLT0, AR_PIXEL_FORMAT pixelFormat0, ARParamLT *paramLT1, AR_PIXEL_FORMAT pixelFormat1, const ARdouble transL2R[3][4])
{
    if (!paramLT0 || pixelFormat0 == AR_PIXEL_FORMAT_INVALID || (paramLT1 && (pixelFormat0 == AR_PIXEL_FORMAT_INVALID || !transL2R))) return false;
    
    // Create AR handle
    if ((m_arHandle0 = arCreateHandle(paramLT0)) == NULL) {
        ARLOGe("arCreateHandle\n");
        goto bail;
    }
    
    // Set the pixel format
    arSetPixelFormat(m_arHandle0, pixelFormat0);
    
    arPattAttach(m_arHandle0, m_arPattHandle);
    
    // Set initial configuration. One call for each configuration option.
    arSetLabelingThresh(m_arHandle0, m_threshold);
    arSetLabelingThreshMode(m_arHandle0, m_thresholdMode);
    arSetImageProcMode(m_arHandle0, m_imageProcMode);
    arSetDebugMode(m_arHandle0, m_debugMode ? AR_DEBUG_ENABLE : AR_DEBUG_DISABLE);
    arSetLabelingMode(m_arHandle0, m_labelingMode);
    arSetPattRatio(m_arHandle0, m_pattRatio);
    arSetPatternDetectionMode(m_arHandle0, m_patternDetectionMode);
    arSetMatrixCodeType(m_arHandle0, m_matrixCodeType);
    arSetCornerRefinementMode(m_arHandle0, m_cornerRefinementMode ? AR_CORNER_REFINEMENT_ENABLE : AR_CORNER_REFINEMENT_DISABLE);
    
    if (paramLT1) {
        // Create AR handle
        if ((m_arHandle1 = arCreateHandle(paramLT1)) == NULL) {
            ARLOGe("arCreateHandle\n");
            goto bail1;
        }
        
        // Set the pixel format
        arSetPixelFormat(m_arHandle1, pixelFormat1);
        
        arPattAttach(m_arHandle1, m_arPattHandle);
        
        // Set initial configuration. One call for each configuration option.
        arSetLabelingThresh(m_arHandle1, m_threshold);
        arSetLabelingThreshMode(m_arHandle1, m_thresholdMode);
        arSetImageProcMode(m_arHandle1, m_imageProcMode);
        arSetDebugMode(m_arHandle1, m_debugMode ? AR_DEBUG_ENABLE : AR_DEBUG_DISABLE);
        arSetLabelingMode(m_arHandle1, m_labelingMode);
        arSetPattRatio(m_arHandle1, m_pattRatio);
        arSetPatternDetectionMode(m_arHandle1, m_patternDetectionMode);
        arSetMatrixCodeType(m_arHandle1, m_matrixCodeType);
        arSetCornerRefinementMode(m_arHandle1, m_cornerRefinementMode ? AR_CORNER_REFINEMENT_ENABLE : AR_CORNER_REFINEMENT_DISABLE);
    }
    
    if (!paramLT1) {
        // Create 3D handle
        if ((m_ar3DHandle = ar3DCreateHandle(&paramLT0->param)) == NULL) {
            ARLOGe("ar3DCreateHandle\n");
            goto bail2;
        }
    } else{
        memcpy(m_transL2R, transL2R, sizeof(ARdouble)*12);
        m_ar3DStereoHandle = ar3DStereoCreateHandle(&paramLT0->param, &paramLT1->param, AR_TRANS_MAT_IDENTITY, m_transL2R);
        if (!m_ar3DStereoHandle) {
            ARLOGe("ar3DStereoCreateHandle\n");
            goto bail2;
        }
    }
    
    ARLOGd("ARTrackerSquare::start() done.\n");
    return true;
    
bail2:
    arDeleteHandle(m_arHandle1);
    m_arHandle1 = NULL;
bail1:
    arDeleteHandle(m_arHandle0);
    m_arHandle0 = NULL;
bail:
    return false;
}

bool ARTrackerSquare::update(AR2VideoBufferT *buff, std::vector<ARTrackable *>& trackables, bool lowRes, bool doDatums, bool doMapper, int markerType, int numberOfDatums)
{
    return update(buff, NULL, trackables, lowRes, doDatums, doMapper, markerType, numberOfDatums);
}

bool ARTrackerSquare::update(AR2VideoBufferT *buff0, AR2VideoBufferT *buff1, std::vector<ARTrackable *>& trackables, bool lowRes, bool doDatums, bool doMapper, int markerType, int numberOfDatums)
{
    ARMarkerInfo *markerInfo0 = NULL;
    ARMarkerInfo *markerInfo1 = NULL;
    int markerNum0 = 0;
    int markerNum1 = 0;

//    if(!lowRes) {
//        ARLOGd("ARX::ARTrackerSquare::update() highRes\n");
//    } else {
//        ARLOGd("-");
//    }

    if (!m_arHandle0 || (buff1 && !m_arHandle1)) return false;

    if (arDetectMarker(m_arHandle0, buff0, lowRes ? 1 : 0) < 0) {
        ARLOGe("arDetectMarker().\n");
        return false;
    }
    markerInfo0 = arGetMarker(m_arHandle0);
    markerNum0 = arGetMarkerNum(m_arHandle0);
    
//    ARPRINT("ARX::ARTrackerSquare::update() num markers = %d.\n", markerNum0);
//    for (int i = 0; i < markerNum0; i++) {
//        ARPRINT("Marker found : %d %d.\n", (int)markerInfo0->globalID, markerInfo0->idMatrix);
//    }
    
    if (buff1) {
        if (arDetectMarker(m_arHandle1, buff1, lowRes ? 1 : 0) < 0) {
            ARLOGe("arDetectMarker().\n");
            return false;
        }
        markerInfo1 = arGetMarker(m_arHandle1);
        markerNum1 = arGetMarkerNum(m_arHandle1);
    }
    
    // Update square markers.
    bool success = true;
    if (!buff1) {
        if (lowRes) {
            for (std::vector<ARTrackable *>::iterator it = trackables.begin(); it != trackables.end(); ++it) {
                if ((*it)->type == ARTrackable::SINGLE) {
                    success &= ((ARTrackableSquare *)(*it))->updateWithDetectedMarkers(markerInfo0, markerNum0, m_ar3DHandle, m_arHandle0->arParamLT->param);
                } else if ((*it)->type == ARTrackable::MULTI) {
                    success &= ((ARTrackableMultiSquare *)(*it))->updateWithDetectedMarkers(markerInfo0, markerNum0, m_ar3DHandle, 1);
                }
            }
        } else {
            //Find the mapper
            int m_OriginUid = -1;
            ARTrackable* originTrackable = 0;
    #if HAVE_GTSAM
            for (std::vector<ARTrackable*>::iterator it = trackables.begin(); it != trackables.end(); ++it) {
                if ((*it)->type == ARTrackable::MULTI_AUTO) {
                    ARTrackableMultiSquareAuto* marker = (ARTrackableMultiSquareAuto*)(*it);
                    m_OriginUid = marker->m_OriginMarkerUid;
                    break;
                }
            }

            // Do all the square and multi-markers before the multi_auto one
            std::vector<arx_mapper::Marker> markers;
    #endif
            
            for (std::vector<ARTrackable *>::iterator it = trackables.begin(); it != trackables.end(); ++it) {
                if ((*it)->type == ARTrackable::SINGLE) {
                    ARTrackableSquare* target = ((ARTrackableSquare*)(*it));
                    bool success2 = target->updateWithDetectedMarkers(markerInfo0, markerNum0, m_ar3DHandle, m_arHandle0->arParamLT->param);
                    success &= success2;
                    // Note: markerType == 1 means it's RevC7 (5x5 single square).
                    // We do not want to compute datums for large boards.
                    if (success2 && doDatums && markerType == 1) {
                        if (target->visible && target->UID < 100) {
                            bool largeBoard = false;
                            //if (target->UID < 2) largeBoard = true;
                            //ARLOGd("Attempting to use datum circles. For RevC7_ %i \n", numberOfDatums);
                            //ARLOGd("Target UID= %i .\n", target->UID );
                            success2 = target->updateWithDetectedDatums2(m_arHandle0->arParamLT->param, buff0->buffLuma, m_arHandle0->xsize, m_arHandle0->ysize, m_ar3DHandle, largeBoard, numberOfDatums);
                            
                            //ARLOGe("Image Width: %i Height: %i.\n", m_arHandle0->xsize, m_arHandle0->ysize );
                            success &= success2;
                            if (!target->visible) {
                                for (int j = 0; j < markerNum0; j++) {
                                    if (markerInfo0[j].idMatrix == target->patt_id) {
                                        markerInfo0[j].idMatrix = -1;
                                    }
                                }
                            }
                        }
                    }
    #if HAVE_GTSAM
                    if (success2 && target->visible) {
                        arx_mapper::Marker marker;
                        marker.uid = target->patt_id;
                        if (marker.uid == m_OriginUid) originTrackable = target;
                        for (int i = 0; i < 3; i++) {
                            for (int j = 0; j < 4; j++) {
                                marker.trans[i][j] = target->GetTrans(i, j);
                            }
                        }
                        markers.push_back(marker);
                    }
    #endif
                    
                } else if ((*it)->type == ARTrackable::MULTI) {
                    ARTrackableMultiSquare* target = ((ARTrackableMultiSquare*)(*it));
                    bool success2 = target->updateWithDetectedMarkers(markerInfo0, markerNum0, m_ar3DHandle, 1);
                    success &= success2;
    #if HAVE_GTSAM
                    ARMultiMarkerInfoT* map = target->config;                    
                    
                    if (success2 && doDatums && markerType == 0)
                    {
                        if (target->visible && target->UID < 100)
                        {
                            bool largeBoard = false;
                            //ARLOGd("Attempting to use datum circles. For RevC1_ %i \n", numberOfDatums);
                            //ARLOGd("Target UID= %i .\n", target->UID );
                            
                            //const char * hiresLowres = lowRes ? "LowRes": "HiRes";
                            //ARLOGd("%s \n", hiresLowres);
                            //ARPRINT("%s \n", hiresLowres);
                            
                            success2 = target->updateWithDetectedDatums2(m_arHandle0->arParamLT->param, buff0->buffLuma, m_arHandle0->xsize, m_arHandle0->ysize, markerInfo0, m_ar3DHandle, largeBoard, numberOfDatums);
                            //ARLOGe("Image Width: %i Height: %i.\n", m_arHandle0->xsize, m_arHandle0->ysize );
                            success &= success2;
                            if (!target->visible){
                                for (int i = 0; i < map->marker_num; i++){
                                    for (int j = 0; j < markerNum0; j++){
                                        if (markerInfo0[j].idMatrix == map->marker[i].patt_id){
                                            markerInfo0[j].idMatrix = -1;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    if (success2 && target->visible) {
                        for (int i = 0; i < map->marker_num; i++) {
                            arx_mapper::Marker marker;
                            marker.uid = map->marker[i].patt_id;
                            if (marker.uid == m_OriginUid) originTrackable = target;
                            ARdouble trans[3][4];
                            for (int i = 0; i < 3; i++) {
                                for (int j = 0; j < 4; j++) {
                                    trans[i][j] = target->GetTrans(i, j);
                                }
                            }
                            arUtilMatMul(trans, map->marker[i].trans, marker.trans);
                            markers.push_back(marker);
                        }
                    }
                    else {
                        for (int i = 0; i < map->marker_num; i++) {
                            for (int j = 0; j < markerNum0; j++) {
                                if (markerInfo0[j].idMatrix == map->marker[i].patt_id) {
                                    markerInfo0[j].idMatrix = -1;
                                }
                            }
                        }
                    }
    #endif
                }
            }
            
            //ARLOGd("ARTrackerSquare::update:L:558 DOMAPPER:%d\n", doMapper);
            
            // Now do the multi_auto marker
            for (std::vector<ARTrackable*>::iterator it = trackables.begin(); it != trackables.end(); ++it) {
                
                if (doMapper && (*it)->type == ARTrackable::MULTI_AUTO) {
    #if HAVE_GTSAM
                    ARTrackableMultiSquareAuto* marker = (ARTrackableMultiSquareAuto*)(*it);
                    if (m_OriginUid > -1 && originTrackable != 0 && originTrackable->type == ARTrackable::MULTI && marker->m_MultiConfig->marker_num == 0) {
                        marker->initialiseWithMultiSquareTrackable((ARTrackableMultiSquare*)originTrackable);

                        //Only add markers which belong to the ground floor board
                        std::vector<arx_mapper::Marker> newmarkers;
                        ARTrackableMultiSquare* target = ((ARTrackableMultiSquare*)(originTrackable));
                        for (std::vector<arx_mapper::Marker>::iterator mt = markers.begin(); mt != markers.end(); ++mt) {
                            arx_mapper::Marker m = (arx_mapper::Marker)(*mt);
                            for (int i = 0; i < target->config->marker_num; i++) {
                                if (target->config->marker[i].patt_id == m.uid) {
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

                    } else if (m_OriginUid > -1 && originTrackable != 0 && originTrackable->type == ARTrackable::SINGLE && marker->m_MultiConfig->marker_num == 0) {
                        marker->initialiseWithSquareTrackable((ARTrackableSquare*)originTrackable);

                        //Only add markers which belong to the ground floor board
                        std::vector<arx_mapper::Marker> newmarkers;
                        ARTrackableSquare* target = ((ARTrackableSquare*)(originTrackable));
                        for (std::vector<arx_mapper::Marker>::iterator mt = markers.begin(); mt != markers.end(); ++mt) {
                            arx_mapper::Marker m = (arx_mapper::Marker)(*mt);
                            if (target->patt_id == m.uid) {
                                newmarkers.push_back(m);
                                break;
                            }
                        }
                        markers.clear();
                        for (std::vector<arx_mapper::Marker>::iterator mt = newmarkers.begin(); mt != newmarkers.end(); ++mt) {
                            arx_mapper::Marker m = (arx_mapper::Marker)(*mt);
                            markers.push_back(m);
                        }
                    }
                    
                    success = marker->updateWithDetectedMarkers(markerInfo0, markerNum0, m_ar3DHandle, 0);
//                    if (success && marker->visible && doDatums) {
//                        success = marker->updateWithDetectedDatums(m_arHandle0->arParamLT->param, buff0->buffLuma, m_arHandle0->xsize, m_arHandle0->ysize, m_ar3DHandle);
//                    }
                    if (success && marker->visible) success = marker->updateMapperWithMarkers(markers);
    #endif
                }
            }
        } // lowRes
    } else {
        for (std::vector<ARTrackable *>::iterator it = trackables.begin(); it != trackables.end(); ++it) {
            if ((*it)->type == ARTrackable::SINGLE) {
                success &= ((ARTrackableSquare *)(*it))->updateWithDetectedMarkersStereo(markerInfo0, markerNum0, markerInfo1, markerNum1, m_ar3DStereoHandle, m_transL2R);
            } else if ((*it)->type == ARTrackable::MULTI) {
                success &= ((ARTrackableMultiSquare *)(*it))->updateWithDetectedMarkersStereo(markerInfo0, markerNum0, markerInfo1, markerNum1, m_ar3DStereoHandle, m_transL2R);
            } else if ((*it)->type == ARTrackable::MULTI_AUTO) {
                success &= ((ARTrackableMultiSquareAuto *)(*it))->updateWithDetectedMarkersStereo(markerInfo0, markerNum0, m_arHandle0->xsize, m_arHandle0->ysize, markerInfo1, markerNum1, m_arHandle1->xsize, m_arHandle1->ysize, m_ar3DStereoHandle, m_transL2R);
            }
        }
    }

    return true;
}

bool ARTrackerSquare::stop()
{
    //ARLOGd("Cleaning up artoolkitX handles.\n");
    if (m_ar3DHandle) {
        ar3DDeleteHandle(&m_ar3DHandle); // Sets ar3DHandle0 to NULL.
    }
    if (m_ar3DStereoHandle) {
        ar3DStereoDeleteHandle(&m_ar3DStereoHandle); // Sets ar3DStereoHandle to NULL.
    }
    
    if (m_arHandle0) {
        arPattDetach(m_arHandle0);
        arDeleteHandle(m_arHandle0);
        m_arHandle0 = NULL;
    }
    
    if (m_arHandle1) {
        arPattDetach(m_arHandle1);
        arDeleteHandle(m_arHandle1);
        m_arHandle1 = NULL;
    }

    return true;
}
void ARTrackerSquare::terminate()
{
    if (m_arPattHandle) {
        arPattDeleteHandle(m_arPattHandle);
        m_arPattHandle = NULL;
    }
}

ARTrackable *ARTrackerSquare::newTrackable(std::vector<std::string> config, int setUID)
{
    // First token is trackable type.
    if (config.at(0).compare("single") == 0) {
        
        // Token 2 is path to pattern.
        if (config.size() < 2) {
            ARLOGe("Pattern marker config. requires path to pattern.\n");
            return nullptr;
        }
        
        // Token 3 is marker width.
        if (config.size() < 3) {
            ARLOGe("Pattern marker config. requires marker width.\n");
            return nullptr;
        }
        ARdouble width;
#ifdef ARDOUBLE_IS_FLOAT
        width = strtof(config.at(2).c_str(), NULL);
#else
        width = strtod(config.at(2).c_str(), NULL);
#endif
        if (width == 0.0f) {
            ARLOGe("Pattern marker config. specified with invalid width parameter ('%s').\n", config.at(2).c_str());
            return nullptr;
        }
        
        ARLOGi("Creating ARTrackableSquare with pattern='%s', width=%f.\n", config.at(1).c_str(), width);
        ARTrackableSquare *ret = new ARTrackableSquare(setUID);
        if (!ret->initWithPatternFile(config.at(1).c_str(), width, m_arPattHandle)) {
            // Marker failed to load, or was not added.
            delete ret;
            ret = NULL;
        }
        return ret;
        
    } else if (config.at(0).compare("single_buffer") == 0) {
        
        // Token 2 is marker width.
        if (config.size() < 2) {
            ARLOGe("Pattern marker from buffer config. requires marker width.\n");
            return nullptr;
        }
        ARdouble width;
#ifdef ARDOUBLE_IS_FLOAT
        width = strtof(config.at(1).c_str(), NULL);
#else
        width = strtod(config.at(1).c_str(), NULL);
#endif
        if (width == 0.0f) {
            ARLOGe("Pattern marker from buffer config. specified with invalid height parameter ('%s').\n", config.at(1).c_str());
            return nullptr;
        }
        
        // Token 3 is buffer.
        if (config.size() < 3) {
            ARLOGe("Pattern marker from buffer config. requires buffer in config.\n");
            return nullptr;
        }
        if (config.at(2).compare(0, 7, "buffer=") != 0) {
            ARLOGe("Pattern marker from buffer config. specified with invalid buffer parameter.\n");
            return nullptr;
        }
        const char *bufferStart = config.at(2).c_str() + 7;
        
        ARTrackableSquare *ret = new ARTrackableSquare(setUID);
        if (!ret->initWithPatternFromBuffer(bufferStart, width, m_arPattHandle)) {
            // Marker failed to load, or was not added
            delete ret;
            ret = NULL;
        }
        return ret;
        
    } else if (config.at(0).compare("single_barcode") == 0) {
        
        // Token 2 is barcode ID.
        errno = 0;
        if (config.size() < 2) {
            ARLOGe("Barcode marker config. requires barcode ID.\n");
            return nullptr;
        }
        long barcodeID = strtol(config.at(1).c_str(), NULL, 0);
        if (barcodeID < 0 || (barcodeID == 0 && (errno == EINVAL || errno == ERANGE))) {
            ARLOGe("Barcode marker config. specified with invalid ID parameter ('%s').\n", config.at(1).c_str());
            return nullptr;
        }
        
        // Token 3 is marker width.
        if (config.size() < 3) {
            ARLOGe("Barcode marker config. requires marker width.\n");
            return nullptr;
        }
        ARdouble width;
#ifdef ARDOUBLE_IS_FLOAT
        width = strtof(config.at(2).c_str(), NULL);
#else
        width = strtod(config.at(2).c_str(), NULL);
#endif
        if (width == 0.0f) {
            ARLOGe("Barcode marker config. specified with invalid width parameter ('%s').\n", config.at(2).c_str());
            return nullptr;
        }
        
        ARTrackableSquare *ret = new ARTrackableSquare(setUID);
        if (!ret->initWithBarcode((int)barcodeID, width)) {
            // Marker failed to load, or was not added
            delete ret;
            ret = NULL;
        }
        return ret;
        
    } else if (config.at(0).compare("multi") == 0) {
        
        // Token 2 is path to config.
        if (config.size() < 2) {
            ARLOGe("Multimarker config. requires path to multi config file.\n");
            return nullptr;
        }
        
        ARTrackableMultiSquare *ret = new ARTrackableMultiSquare(setUID);
        if (!ret->load(config.at(1).c_str(), m_arPattHandle)) {
            // Marker failed to load, or was not added
            delete ret;
            ret = NULL;
        }
        return ret;
        
    } else if (config.at(0).compare("multi_auto") == 0) {
        
        // Token 2 is origin marker barcode ID.
        if (config.size() < 2) {
            ARLOGe("Multimarker auto config. requires base marker ID.\n");
            return nullptr;
        }
        long originMarkerUID = strtol(config.at(1).c_str(), NULL, 0);
        if (originMarkerUID < 0 || (originMarkerUID == 0 && (errno == EINVAL || errno == ERANGE))) {
            ARLOGe("Multimarker auto config. specified with invalid origin marker UID parameter ('%s').\n", config.at(1).c_str());
            return nullptr;
        }

        // Token 3 is marker width.
        if (config.size() < 3) {
            ARLOGe("Multimarker auto config. requires marker width.\n");
            return nullptr;
        }
        ARdouble width;
#ifdef ARDOUBLE_IS_FLOAT
        width = strtof(config.at(2).c_str(), NULL);
#else
        width = strtod(config.at(2).c_str(), NULL);
#endif
        if (width == 0.0f) {
            ARLOGe("Multimarker auto config. specified with invalid width parameter ('%s').\n", config.at(2).c_str());
            return nullptr;
        }
        
        ARTrackableMultiSquareAuto *ret = new ARTrackableMultiSquareAuto();
        if (!ret->initWithOriginMarkerUID(originMarkerUID, width)) {
            // Marker failed to load, or was not added
            delete ret;
            ret = NULL;
        }
        return ret;
    } else {
        return nullptr;
    }

}

void ARTrackerSquare::deleteTrackable(ARTrackable **trackable_p)
{
    if (!trackable_p || !(*trackable_p)) return;
    if ((*trackable_p)->type != ARTrackable::SINGLE && (*trackable_p)->type != ARTrackable::MULTI && (*trackable_p)->type != ARTrackable::MULTI_AUTO) return;
    
    delete (*trackable_p);
    (*trackable_p) = NULL;
}

// ----------------------------------------------------------------------------------------------------
#pragma mark Square tracker debug texture
// ----------------------------------------------------------------------------------------------------

bool ARTrackerSquare::updateDebugTextureRGBA32(const int videoSourceIndex, uint32_t* buffer)
{
#ifdef AR_DISABLE_LABELING_DEBUG_MODE
    ARLOGe("Debug texture not supported.");
    return false;
#else
    // Check everything is valid.
    if (!buffer) return false;
    ARHandle *arHandle = (videoSourceIndex == 1 ? m_arHandle1 : m_arHandle0);
    if (!arHandle) return false;
    if (!arHandle->labelInfo.bwImage) return false;
    
    uint8_t *src;
    uint32_t* dest = buffer;
    int h = arHandle->ysize;
    if (arGetImageProcMode(arHandle) == AR_IMAGE_PROC_FIELD_IMAGE) {
        int wdiv2 = arHandle->xsize >> 1;
        for (int y = 0; y < h; y++) {
            src = &(arHandle->labelInfo.bwImage[(h >> 1) * wdiv2]);
            for (int x = 0; x < wdiv2; x++) {
                *dest = ((*src) << 24) + ((*src) << 16) + ((*src) << 8) + 255;
                dest++;
                *dest = ((*src) << 24) + ((*src) << 16) + ((*src) << 8) + 255;
                dest++;
                src++;
            }
        }
    } else {
        src = arHandle->labelInfo.bwImage;
        int w = arHandle->xsize;
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                *dest = ((*src) << 24) + ((*src) << 16) + ((*src) << 8) + 255;
                src++;
                dest++;
            }
        }
        
    }
    return true;
#endif
}

