/*
 *  arMultiEditConfig.c
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
 *  Copyright 2018 Philip Lamb
 *  Copyright 2015 Daqri, LLC.
 *  Copyright 2002-2015 ARToolworks, Inc.
 *
 *  Author(s): Hirokazu Kato, Philip Lamb
 *
 */

#include <ARX/AR/ar.h>
#include <ARX/AR/arMulti.h>
#include <string.h> // memset()

ARMultiMarkerInfoT *arMultiAllocConfig(void)
{
    ARMultiMarkerInfoT *marker_info = (ARMultiMarkerInfoT *)malloc(sizeof(ARMultiMarkerInfoT));
    if (!marker_info) {
        ARLOGe("arMultiAllocConfig out of memory!!\n");
        return NULL;
    }
    
    marker_info->marker = NULL;
    marker_info->marker_num = 0;
    marker_info->prevF = 0;
    marker_info->patt_type = AR_MULTI_PATTERN_DETECTION_MODE_NONE;
    marker_info->cfPattCutoff = AR_MULTI_CONFIDENCE_PATTERN_CUTOFF_DEFAULT;
    marker_info->cfMatrixCutoff = AR_MULTI_CONFIDENCE_MATRIX_CUTOFF_DEFAULT;
    marker_info->min_submarker = 0;
    
    return (marker_info);
}

// patt_type: Either AR_MULTI_PATTERN_TYPE_TEMPLATE or AR_MULTI_PATTERN_TYPE_MATRIX.
int arMultiAddOrUpdateSubmarker(ARMultiMarkerInfoT *marker_info, int patt_id, int patt_type, ARdouble width, const ARdouble trans[3][4], uint64_t globalID)
{
    int i;
    
    //ARLOGi("arMultiAddOrUpdateSubmarker called with %d markers. Adding id %d\n", marker_info->marker_num, patt_id);
    
    // Look for matching marker already in set.
    for (i = 0; i < marker_info->marker_num; i++) {
        if (marker_info->marker[i].patt_type == patt_type && marker_info->marker[i].patt_id == patt_id) {
            if (patt_type == AR_MULTI_PATTERN_TYPE_TEMPLATE || (patt_type == AR_MULTI_PATTERN_TYPE_MATRIX && marker_info->marker[i].globalID == globalID)) break;
        }
    }
    
    if (i == marker_info->marker_num) { // Not found, need to add to it.
        
        // Increase the array size.
        ARMultiEachMarkerInfoT *emi;
        if (!marker_info->marker) emi = (ARMultiEachMarkerInfoT *)malloc(sizeof(ARMultiEachMarkerInfoT));
        else emi = (ARMultiEachMarkerInfoT *)realloc(marker_info->marker, sizeof(ARMultiEachMarkerInfoT) * (marker_info->marker_num + 1));
        if (!emi) {
            ARLOGe("arMultiAddOrUpdateSubmarker out of memory!!\n");
            return (-1);
        }
        marker_info->marker = emi;
        marker_info->marker_num++;

        // As we've enlarged the array, i is now a valid index.
        memset(&marker_info->marker[i], 0, sizeof(ARMultiEachMarkerInfoT));
        marker_info->marker[i].patt_id = patt_id;
        marker_info->marker[i].patt_type = patt_type;
        marker_info->marker[i].width = width;
        if (patt_type == AR_MULTI_PATTERN_TYPE_MATRIX) {
            marker_info->marker[i].globalID = globalID;
        }
    }
    
    arMultiUpdateSubmarkerPose(&marker_info->marker[i], trans);
    
    if (patt_type == AR_MULTI_PATTERN_TYPE_MATRIX) {
        if (marker_info->patt_type == AR_MULTI_PATTERN_DETECTION_MODE_TEMPLATE) {
            marker_info->patt_type = AR_MULTI_PATTERN_DETECTION_MODE_TEMPLATE_AND_MATRIX;
        } else {
            marker_info->patt_type = AR_MULTI_PATTERN_DETECTION_MODE_MATRIX;
        }
    } else { // patt_type == AR_MULTI_PATTERN_TYPE_TEMPLATE
        if (marker_info->patt_type == AR_MULTI_PATTERN_DETECTION_MODE_MATRIX) {
            marker_info->patt_type = AR_MULTI_PATTERN_DETECTION_MODE_TEMPLATE_AND_MATRIX;
        } else {
            marker_info->patt_type = AR_MULTI_PATTERN_DETECTION_MODE_TEMPLATE;
        }
    }
    
    return 0;
}

int arMultiAddOrUpdateSubmarker2(ARMultiMarkerInfoT *marker_info, ARMultiMarkerInfoT *marker_info2, ARMultiMarkerInfoT *marker_info3, int patt_id, int patt_type, ARdouble width, const ARdouble trans[3][4], uint64_t globalID)
{
    int i;
    
    // Look for matching marker already in set.
    for (i = 0; i < marker_info->marker_num; i++) {
        if (marker_info->marker[i].patt_id == patt_id) break;
    }
    
    if (i == marker_info->marker_num) { // Not found, need to add to it.
        
        ARLOGi("arMultiAddOrUpdateSubmarker2 called with %d markers. Adding id %d\n", marker_info->marker_num, patt_id);
        
        // Increase the array size.
        ARMultiEachMarkerInfoT *emi;
        if (!marker_info->marker) emi = (ARMultiEachMarkerInfoT *)malloc(sizeof(ARMultiEachMarkerInfoT));
        else emi = (ARMultiEachMarkerInfoT *)realloc(marker_info->marker, sizeof(ARMultiEachMarkerInfoT) * (marker_info->marker_num + 1));
        if (!emi) {
            ARLOGe("arMultiAddOrUpdateSubmarker out of memory!!\n");
            return (-1);
        }
        marker_info->marker = emi;
        marker_info->marker_num++;
        
        // As we've enlarged the array, i is now a valid index.
        memset(&marker_info->marker[i], 0, sizeof(ARMultiEachMarkerInfoT));
        marker_info->marker[i].patt_id = patt_id;
        marker_info->marker[i].patt_type = patt_type;
        marker_info->marker[i].width = width;
    }
    
    arMultiUpdateSubmarkerPose(&marker_info->marker[i], trans);
    
    for (int j = 0; j < marker_info3->marker_num; j++) {
        
        ARMat *dest;
        ARMat *a;
        a = arMatrixAlloc( 4, 4 );
        a->m[12] = a->m[13] = a->m[14] = 0.0; a->m[15] = 1.0;
        for(int k = 0; k < 3; k++ ) {
            for(int k2 = 0; k2 < 4; k2++ ) {
                a->m[k*4+k2] = trans[k][k2];
            }
        }
        
        ARMat * b;
        b = arMatrixAlloc( 4, 4 );
        b->m[12] = b->m[13] = b->m[14] = 0.0; b->m[15] = 1.0;
        for(int k = 0; k < 3; k++ ) {
            for(int k2 = 0; k2 < 4; k2++ ) {
                b->m[k*4+k2] = marker_info3->marker[j].trans[k][k2];
            }
        }
        
        dest = arMatrixAllocMul(a, b);
        ARdouble trans2[3][4];
        for(int k = 0; k < 3; k++ ) {
            for(int k2 = 0; k2 < 4; k2++ ) {
                trans2[k][k2] = dest->m[k*4+k2];
            }
        }
        
        arMultiAddOrUpdateSubmarker(marker_info2, marker_info3->marker[j].patt_id, patt_type, marker_info3->marker[j].width, trans2, globalID);
    }
    
    return 0;
}

void arMultiUpdateSubmarkerPose(ARMultiEachMarkerInfoT *submarker, const ARdouble trans[3][4])
{
    int i, j;
    ARdouble wpos3d[4][2];
    
    for (j = 0; j < 3; j++) {
        for (i = 0; i < 4; i++) {
            submarker->trans[j][i] = trans[j][i];
        }
    }
    
    arUtilMatInv((const ARdouble (*)[4])submarker->trans, submarker->itrans);
    
    wpos3d[0][0] =  -submarker->width/2.0;
    wpos3d[0][1] =   submarker->width/2.0;
    wpos3d[1][0] =   submarker->width/2.0;
    wpos3d[1][1] =   submarker->width/2.0;
    wpos3d[2][0] =   submarker->width/2.0;
    wpos3d[2][1] =  -submarker->width/2.0;
    wpos3d[3][0] =  -submarker->width/2.0;
    wpos3d[3][1] =  -submarker->width/2.0;
    for (j = 0; j < 4; j++) {
        submarker->pos3d[j][0] = submarker->trans[0][0] * wpos3d[j][0]
                               + submarker->trans[0][1] * wpos3d[j][1]
                               + submarker->trans[0][3];
        submarker->pos3d[j][1] = submarker->trans[1][0] * wpos3d[j][0]
                               + submarker->trans[1][1] * wpos3d[j][1]
                               + submarker->trans[1][3];
        submarker->pos3d[j][2] = submarker->trans[2][0] * wpos3d[j][0]
                               + submarker->trans[2][1] * wpos3d[j][1]
                               + submarker->trans[2][3];
    }
}

int arMultiRemoveSubmarker(ARMultiMarkerInfoT *marker_info, int patt_id, int patt_type, uint64_t globalID)
{
    int i;
    
    // Look for matching marker already in set.
    for (i = 0; i < marker_info->marker_num; i++) {
        if (marker_info->marker[i].patt_type == patt_type && marker_info->marker[i].patt_id == patt_id) {
            if (patt_type == AR_MULTI_PATTERN_TYPE_TEMPLATE || (patt_type == AR_MULTI_PATTERN_TYPE_MATRIX && marker_info->marker[i].globalID == globalID)) break;
        }
    }

    if (i == marker_info->marker_num) return -1; // Not found.
    
    // Shuffle the rest down.
    for (i++; i < marker_info->marker_num; i++) {
        marker_info->marker[i - 1] = marker_info->marker[i];
    }
    
    // Reduce the array size. Because we're shrinking, realloc failure isn't fatal.
    ARMultiEachMarkerInfoT *emi = (ARMultiEachMarkerInfoT *)realloc(marker_info->marker, sizeof(ARMultiEachMarkerInfoT) * (marker_info->marker_num - 1));
    if (!emi) {
        ARLOGw("arMultiAddOrUpdateEachMarker out of memory!!\n.");
    } else {
        marker_info->marker = emi;
    }
    marker_info->marker_num--;

    return 0;
}
