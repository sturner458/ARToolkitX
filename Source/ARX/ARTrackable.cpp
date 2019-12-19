/*
 *  ARTrackable.cpp
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
 *  Author(s): Julian Looser, Philip Lamb.
 *
 */

#include <ARX/ARTrackable.h>
#include <ARX/ARTrackableSquare.h>
#include <ARX/ARTrackableMultiSquare.h>
#if HAVE_NFT
#  include <ARX/ARTrackableNFT.h>
#endif
#if HAVE_2D
#  include <ARX/ARTrackable2d.h>
#endif
#include <ARX/ARController.h>
#include <ARX/AR/paramGL.h>

#ifdef _WIN32
#  define MAXPATHLEN MAX_PATH
#else
#  include <sys/param.h>
#endif

ARTrackable::ARTrackable(TrackableType type) :
    m_ftmi(NULL),
    m_filterCutoffFrequency(AR_FILTER_TRANS_MAT_CUTOFF_FREQ_DEFAULT),
    m_filterSampleRate(AR_FILTER_TRANS_MAT_SAMPLE_RATE_DEFAULT),
#ifdef ARDOUBLE_IS_FLOAT
    m_positionScaleFactor(1.0f),
#else
    m_positionScaleFactor(1.0),
#endif
    type(type),
    visiblePrev(false),
	visible(false),
    patternCount(0),
    patterns(NULL)
{
	static int nextUID = 0;
	UID = nextUID++;
}

ARTrackable::~ARTrackable()
{
    freePatterns();

    if (m_ftmi) arFilterTransMatFinal(m_ftmi);
}

void ARTrackable::allocatePatterns(int count)
{
	freePatterns();

    if (count) {
        patternCount = count;
        //ARLOGd("Allocating %d patterns on trackable %d.\n", patternCount, UID);
        patterns = new ARPattern*[patternCount];
        for (int i = 0; i < patternCount; i++) {
            patterns[i] = new ARPattern();
        }        
    }
}

void ARTrackable::freePatterns()
{
	if (patternCount) ARLOGd("Freeing %d patterns on trackable %d.\n", patternCount, UID);

	for (int i = 0; i < patternCount; i++) {
        if (patterns[i]) {
            delete patterns[i];
            patterns[i] = NULL;
        }
	}
	if (patterns) {
        delete[] patterns;
        patterns = NULL;
    }

	patternCount = 0;
}

ARPattern* ARTrackable::getPattern(int n)
{
	// Check n is in acceptable range
	if (!patterns || n < 0 || n >= patternCount) return NULL;

	return patterns[n];
}

void ARTrackable::setPositionScalefactor(ARdouble scale)
{
    m_positionScaleFactor = scale;
}

ARdouble ARTrackable::positionScalefactor()
{
    return m_positionScaleFactor;
}

ARdouble ARTrackable::GetTrans(int i, int j) {
	return trans[i][j];
}

void ARTrackable::SetTrans(int i, int j, ARdouble value) {
    trans[i][j] = value;
}

bool ARTrackable::update(const ARdouble transL2R[3][4])
{
    // Subclasses will have already determined visibility and set/cleared 'visible' and 'visiblePrev',
    // as well as setting 'trans'.
    if (visible) {
        
        // Filter the pose estimate.
        if (m_ftmi) {
            if (arFilterTransMat(m_ftmi, trans, !visiblePrev) < 0) {
                ARLOGe("arFilterTransMat error with trackable %d.\n", UID);
            }
        }
        
        if (!visiblePrev) {
            //ARLOGi("trackable %d now visible.\n", UID);
        }
        
        // Convert to GL matrix.
#ifdef ARDOUBLE_IS_FLOAT
        arglCameraViewRHf(trans, transformationMatrix, m_positionScaleFactor);
#else
        arglCameraViewRH(trans, transformationMatrix, m_positionScaleFactor);
#endif

        // Do stereo if required.
        if (transL2R) {
            ARdouble transR[3][4];
            
            arUtilMatMul(transL2R, trans, transR);
#ifdef ARDOUBLE_IS_FLOAT
            arglCameraViewRHf(transR, transformationMatrixR, m_positionScaleFactor);
#else
            arglCameraViewRH(transR, transformationMatrixR, m_positionScaleFactor);
#endif
            
        }
    } else {
        
        if (visiblePrev) {
            //ARLOGi("Trackable %d no longer visible.\n", UID);
        }
        
    }
    
    return true;
}

void ARTrackable::setFiltered(bool flag)
{
    if (flag && !m_ftmi) {
        m_ftmi = arFilterTransMatInit(m_filterSampleRate, m_filterCutoffFrequency);
    } else if (!flag && m_ftmi) {
        arFilterTransMatFinal(m_ftmi);
        m_ftmi = NULL;
    }
}

bool ARTrackable::isFiltered()
{
    return (m_ftmi != NULL);
}

ARdouble ARTrackable::filterSampleRate()
{
    return m_filterSampleRate;
}

void ARTrackable::setFilterSampleRate(ARdouble rate)
{
    m_filterSampleRate = rate;
    if (m_ftmi) arFilterTransMatSetParams(m_ftmi, m_filterSampleRate, m_filterCutoffFrequency);
}

ARdouble ARTrackable::filterCutoffFrequency()
{
    return m_filterCutoffFrequency;
}

void ARTrackable::setFilterCutoffFrequency(ARdouble freq)
{
    m_filterCutoffFrequency = freq;
    if (m_ftmi) arFilterTransMatSetParams(m_ftmi, m_filterSampleRate, m_filterCutoffFrequency);
}

bool ARTrackable::GetCenterPointForDatum(ARdouble x, ARdouble y, ARParam arParams, ARdouble trans[3][4], cv::Mat grayImage, int imageWidth, int imageHeight, ARdouble* ox, ARdouble* oy) {
	ModelToImageSpace(arParams, trans, x, y, ox, oy);
	int halfSquare = GetSquareForDatum(x, y, arParams, trans);
	if (halfSquare < 5) return false;
	if (*ox - halfSquare < 0 || *ox + halfSquare > imageWidth || *oy - halfSquare < 0 || *oy + halfSquare > imageHeight) return false;

	cv::Rect rect = cv::Rect((int)*ox - halfSquare, (int)*oy - halfSquare, 2 * halfSquare, 2 * halfSquare);
	cv::Mat region = cv::Mat(grayImage, rect);
	cv::Mat binaryRegion = region.clone();
	double otsuThreshold = cv::threshold(region, binaryRegion, 0.0, 255.0, cv::THRESH_OTSU);
	int nonzero = cv::countNonZero(binaryRegion);
	int square = 4 * halfSquare * halfSquare;
	return (nonzero > square * 0.2f && nonzero < square * 0.8f);
}

void ARTrackable::ModelToImageSpace(ARParam param, ARdouble trans[3][4], ARdouble ix, ARdouble iy, ARdouble* ox, ARdouble* oy) {
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

int ARTrackable::GetSquareForDatum(ARdouble x, ARdouble y, ARParam arParams, ARdouble trans[3][4]) {
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

	return (int)(maxD / sqrt(2.0) + 0.499);
}

ARdouble ARTrackable::arGetTransMatDatum(AR3DHandle* handle, ARdouble* datumCoords2D, ARdouble* datumCoords, const int numDatums, ARdouble conv[3][4])
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


