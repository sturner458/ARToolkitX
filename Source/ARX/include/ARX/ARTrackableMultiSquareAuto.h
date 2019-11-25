/*
 *  ARTrackableMultiSquareAuto.h
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
 *  Author(s): Philip Lamb
 *
 */

#ifndef ARTRACKABLEMAP_H
#define ARTRACKABLEMAP_H

#include <ARX/ARTrackable.h>
#include <ARX/AR/arMulti.h>
#include <ARX/ARTrackableSquare.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#if HAVE_GTSAM
#include "../../mapper.hpp"
#endif

/**
 * An ARTrackable that auto-builds a map of matrix (2D barcode) markers.
 */
class ARTrackableMultiSquareAuto : public ARTrackable {

private:
    ARdouble m_markerWidth;

    // A holder for a struct which holds member variables which we don't want to appear to the header.
    std::unique_ptr<struct ARTrackableMapPrivateMembers> m_pm;


protected:
    
public:
	
	ARTrackableMultiSquareAuto();
	~ARTrackableMultiSquareAuto();
    
    bool initWithOriginMarkerUID(int originMarkerUID, ARdouble markerWidth);
	int m_OriginMarkerUid; // The UID of the barcode marker which defines the origin of the world coordinate system.
    
    // Tracking parameters.
    ARMultiMarkerInfoT *m_MultiConfig;
    ARdouble m_ImageBorderZone = 0.2f; ///< The proportion of the image width/height to consider as an "border" zone in which markers are not to be detected. Set to 0.0f to allow markers to appear anywhere in the image.
    bool m_robustFlag = false; ///< Flag specifying which pose estimation approach to use
    ARdouble m_maxErr = 4.0f; ///< The maximum allowable pose estimate error.

	/**
	 * Updates the marker with new tracking info.
     * Then calls ARTrackable::update()
     * @param markerInfo		Array containing detected marker information
     * @param markerNum			Number of items in the array
     * @param ar3DHandle        AR3DHandle used to extract marker pose.
     */
	bool updateWithDetectedMarkers(ARMarkerInfo* markerInfo, int markerNum, AR3DHandle* ar3DHandle);
	bool updateWithDetectedMarkersOpenCV(ARMarkerInfo* markerInfo, int markerNum, AR3DHandle* ar3DHandle, ARHandle* arHandle);
	bool updateMapper(ARMarkerInfo* markerInfo, int markerNum, int videoWidth, int videoHeight, AR3DHandle* ar3DHandle, std::vector<ARTrackable*>& trackables);
	void ARTrackableMultiSquareAuto::initialiseWithSquareTrackable(ARTrackableSquare* trackable);
	void ARTrackableMultiSquareAuto::initialiseWithMultiSquareTrackable(ARTrackableMultiSquare* trackable);
	bool updateMapperWithMarkers(std::vector<arx_mapper::Marker> markers);
	void addStoredMarkers(float thisTrans[12], std::vector<arx_mapper::Marker> markers);

	bool updateWithDetectedDatums(ARParam arParams, ARUint8* buffLuma, int imageWidth, int imageHeight, AR3DHandle* ar3DHandle);

    bool updateWithDetectedMarkersStereo(ARMarkerInfo* markerInfoL, int markerNumL, int videoWidthL, int videoHeightL, ARMarkerInfo* markerInfoR, int markerNumR, int videoWidthR, int videoHeightR, AR3DStereoHandle *handle, ARdouble transL2R[3][4]);
    
    /**
     * Make a copy of the multi config.
     * Caller must call arMultiFreeConfig() on the returned value when done.
     */
    ARMultiMarkerInfoT *copyMultiConfig();
};


#endif // !ARTRACKABLEMAP_H
