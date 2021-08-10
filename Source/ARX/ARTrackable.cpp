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

ARTrackable::ARTrackable(TrackableType type, int setUID) :
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
    if (setUID == -1) {
        UID = nextUID++;
    } else {
        UID = setUID;
        nextUID = setUID + 1;
    }
    //ARLOGe("ARTrackable called with setUID %d. nextUID now %d\n", setUID, nextUID);
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

bool ARTrackable::GetCenterPointForDatum2(double datumCircleDiameter, ARdouble x, ARdouble y, ARParam arParams, ARdouble trans[3][4], cv::Mat grayImage, int imageWidth, int imageHeight, ARdouble* ox, ARdouble* oy)
{
    // Convert world coords to image coords for a given point.
    ModelToImageSpace(arParams, trans, x, y, ox, oy);

    // Find the rectangular area of where to search for datum circle.
    cv::Rect rect = GetRectForDatum(datumCircleDiameter, x, y, arParams, trans);
    if (rect.width < 5 || rect.height < 5) return false;
    if (rect.x < 0 || rect.x + rect.width > imageWidth || rect.y < 0 || rect.y + rect.height > imageHeight) return false;

    cv::Mat region = cv::Mat(grayImage, rect);
    cv::Mat scaledRegion = cv::Mat();
    cv::resize(region, scaledRegion, cv::Size(), 10.0, 10.0, cv::InterpolationFlags::INTER_CUBIC);
    cv::Mat scaledBinaryRegion = scaledRegion.clone();
    
    cv::threshold(scaledRegion, scaledBinaryRegion, 0.0, 255.0, cv::THRESH_OTSU);

    std::vector<cv::Point2d> centerPoints;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(scaledBinaryRegion, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    if (contours.size() > 0)
    {
        for (int i = 0; i < contours.size(); i++)
        {
            std::vector<cv::Point> contour = contours[i];
            if (contour.size() > 4)
            {
                auto rotRect = cv::fitEllipse(contour);
//                auto area = rotRect.size.width * rotRect.size.height;
//                auto width = rotRect.size.width > rotRect.size.height ? rotRect.size.width : rotRect.size.height;
//                auto height = rotRect.size.width > rotRect.size.height ? rotRect.size.height : rotRect.size.width;
                
                centerPoints.push_back(cv::Point2d((rotRect.center.x / 10.0) + rect.x, (rotRect.center.y /10.0) + rect.y));
            }
        }
    }

    if (centerPoints.size() > 0)
    {
        cv::Point2d idealPoint(*ox, *oy);
        // Find center that is closest to the original point.
        cv::Point2d closestPt(-100000.0, -100000.0);
        for (int i = 0; i < centerPoints.size(); i++)
        {
            if (DistanceBetweenTwoPoints(centerPoints[i], idealPoint) < DistanceBetweenTwoPoints(closestPt, idealPoint))
            {
                closestPt = centerPoints[i];
            }
        }
        // Ensure this point is within specified tolerance.
        // We assume it's within half the average size of the rect.
        if (DistanceBetweenTwoPoints(closestPt, idealPoint) < (rect.height + rect.width) * 0.5 * 0.5)
        {
            *ox = closestPt.x;
            *oy = closestPt.y;
            //ARLOGi("Found datum point at x:%f y:%f!\n", *ox, *oy);
            return true;
        }
    }
    return false;
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

cv::Rect ARTrackable::GetRectForDatum(double datumCircleDiameter, ARdouble x, ARdouble y, ARParam arParams, ARdouble trans[3][4])
{
    double rectHalfWidth = (datumCircleDiameter / 2) + 2.5;

    ARdouble ox, oy, ox1, oy1, ox2, oy2, ox3, oy3, ox4, oy4;
    ModelToImageSpace(arParams, trans, x, y, &ox, &oy);
    ModelToImageSpace(arParams, trans, x - rectHalfWidth, y - rectHalfWidth, &ox1, &oy1);
    ModelToImageSpace(arParams, trans, x + rectHalfWidth, y - rectHalfWidth, &ox2, &oy2);
    ModelToImageSpace(arParams, trans, x + rectHalfWidth, y + rectHalfWidth, &ox3, &oy3);
    ModelToImageSpace(arParams, trans, x - rectHalfWidth, y + rectHalfWidth, &ox4, &oy4);

    // Find the min and max values of rect.
    ARdouble minX = ox1;
    if (ox2 < minX) minX = ox2;
    if (ox3 < minX) minX = ox3;
    if (ox4 < minX) minX = ox4;

    ARdouble maxX = ox1;
    if (ox2 > maxX) maxX = ox2;
    if (ox3 > maxX) maxX = ox3;
    if (ox4 > maxX) maxX = ox4;

    ARdouble minY = oy1;
    if (oy2 < minY) minY = oy2;
    if (oy3 < minY) minY = oy3;
    if (oy4 < minY) minY = oy4;

    ARdouble maxY = oy1;
    if (oy2 > maxY) maxY = oy2;
    if (oy3 > maxY) maxY = oy3;
    if (oy4 > maxY) maxY = oy4;

    return cv::Rect(cv::Point2i((int)minX, (int)minY), cv::Point2i((int)maxX, (int)maxY));
    
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

double ARTrackable::AverageDistanceToEllipse(std::vector<cv::Point> contour, cv::RotatedRect rect)
{
    int n = (int)contour.size();
    double dist = 0.0;

    for (int i = 0; i < n; i++)
    {
        dist = dist + DistanceToEllipse(cv::Point2d((double)contour[i].x, (double)contour[i].y), rect);
    }

    return dist / n;
}

double ARTrackable::FurthestDistanceToEllipse(std::vector<cv::Point> contour, cv::RotatedRect rect)
{
    int n = (int)contour.size();
    double dist = 0.0;

    for (int i = 0; i < n; i++)
    {
        double d = DistanceToEllipse(cv::Point2d((double)contour[i].x, (double)contour[i].y), rect);
        if (d > dist) dist = d;
    }
    return dist;
}

double ARTrackable::DistanceToEllipse(cv::Point2d pt, cv::RotatedRect rect)
{
    // Translate the point, so that rect centre is at zero.
    auto c = (cv::Point2d)rect.center;
    pt = pt - c;
    // Rotate the point by the rect angle.
    cv::Point3d pt3d(pt.x, pt.y, 1);
    auto rot2d = cv::getRotationMatrix2D(cv::Point2d(0, 0), -rect.angle, 1);
    cv::Mat rPtMat = (rot2d * cv::Mat(pt3d)).t();
    pt = cv::Point2d(rPtMat.at<double>(0,0), rPtMat.at<double>(0,1));
    // Compute the nearest point to ellipse.
    cv::Point2d pt2(NearestPointOnEllipse(pt, rect.size.width / 2.0, rect.size.height / 2.0));
    // Calculate distance between the points.
    return DistanceBetweenTwoPoints(pt, pt2);
}

double ARTrackable::DistanceBetweenTwoPoints(cv::Point2d pt1, cv::Point2d pt2)
{
    return std::sqrt(std::pow((pt1.x - pt2.x), 2) + std::pow((pt1.y - pt2.y), 2));
}

cv::Point2d ARTrackable::NearestPointOnEllipse(cv::Point2d point, double semiMajor, double semiMinor)
{
    double px = std::abs(point.x);
    double py = std::abs(point.y);

    double a = semiMajor;
    double b = semiMinor;

    double tx = 0.70710678118;
    double ty = 0.70710678118;

    double x, y, ex, ey, rx, ry, qx, qy, r, q, t = 0;

    for (int i = 0; i < 3; ++i)
    {
        x = a * tx;
        y = b * ty;

        ex = (a * a - b * b) * (tx * tx * tx) / a;
        ey = (b * b - a * a) * (ty * ty * ty) / b;

        rx = x - ex;
        ry = y - ey;

        qx = px - ex;
        qy = py - ey;

        r = std::sqrt(rx * rx + ry * ry);
        q = std::sqrt(qy * qy + qx * qx);

        tx = std::min(1.0, std::max(0.0, (qx * r / q + ex) / a));
        ty = std::min(1.0, std::max(0.0, (qy * r / q + ey) / b));

        t = std::sqrt(tx * tx + ty * ty);

        tx /= t;
        ty /= t;
    }

    return cv::Point2d(
        x = (float)(a * (point.x < 0 ? -tx : tx)),
        y = (float)(b * (point.y < 0 ? -ty : ty))
    );
}

ARdouble ARTrackable::arGetTransMatDatum(AR3DHandle* handle, ARdouble* datumCoords2D, ARdouble* datumCoords, const int numDatums, ARdouble conv[3][4])
{
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


