#include "regmap.h"

vector<vector<StwayGPS> >  GenRegWaypoint(std::vector<struct wayPointGPS> gridPlygon, double gridSpacing, bool hoverAndCaptureEnabled, double hoverAndCaptureDelaySeconds, double gridAngle, 
                                bool gridTriggerCamera, double gridTriggerCameraDist, double turnaroundDist, uint8_t gridMode, uint8_t gridRefly) 
{  
    _gridPlygon = gridPlygon;
    _gridSpacing = gridSpacing;
    _hoverAndCaptureEnabled = hoverAndCaptureEnabled;
    _hoverAndCaptureDelaySeconds = hoverAndCaptureDelaySeconds;
    _gridAngle = gridAngle;
    _gridTriggerCamera = gridTriggerCamera;
    _gridTriggerCameraDist = gridTriggerCameraDist;
    _turnaroundDist = turnaroundDist;
    _gridMode = gridMode;
    _gridRefly = gridRefly;

    generateGrid();

    FILE *fp,*fp1;
    int i;
    if( (fp = fopen("dataman.txt","w+")) == NULL){
        if((fp = fopen("dataman.txt","w")) == NULL ){
            return _transectSegments; 
        } 
    }

    if( (fp1 = fopen("dataman1.txt","w+")) == NULL){
        if((fp1 = fopen("dataman1.txt","w")) == NULL ){
            return _transectSegments; 
        } 
    }
    // for(i = 0; i < _gridPlygon.size(); i ++){
    //    fprintf(fp,"%lf %lf\n",gridPlygon[i].lat,gridPlygon[i].lon); 
    // }
    for(i = 0; i < _transectSegments.size(); i ++){
        for(int j = 0; j < _transectSegments[i].size(); j ++){
            fprintf(fp,"%0.7f\n",_transectSegments[i][j].lat); 
            fprintf(fp1,"%0.7f\n",_transectSegments[i][j].lon); 
        }
    }
    fclose(fp);
    fclose(fp1);

    return _transectSegments;
}  

void generateGrid(void)
{
    // if (_ignoreRecalc) {
    //     return;
    // }

    if (_gridPlygon.size() < 3 || _gridSpacing <= 0) {
        return;
    }

    _simpleGridPoints.clear();
    _transectSegments.clear();
    _reflyTransectSegments.clear();
    _additionalFlightDelaySeconds = 0;

    std::vector<StwayXY>           polygonPoints_tmp;
    vector<vector<StwayXY> >   transectSegments;

    // Convert polygon to NED
    StwayGPS tangentOrigin = _gridPlygon[0];
    #ifdef DEBUG
    printf("Convert polygon to NED - tangentOrigin %lf %lf %d\n",tangentOrigin.lat,tangentOrigin.lon,(int)_gridPlygon.size());
    #endif
    for (int i=0; i<_gridPlygon.size(); i++) {
        StwayXY tmp;
        StwayGPS vertex = _gridPlygon[i];
        if (i == 0) {
            // This avoids a nan calculation that comes out of convertGeoToNed
            tmp.x = tmp.y = 0;
        } else {
            convertGeoToNed(vertex, tangentOrigin, &tmp.y, &tmp.x);
        }
        polygonPoints_tmp.push_back(tmp);
        #ifdef DEBUG
        printf("vertex:x:y %f %f\n", polygonPoints_tmp.back().x,polygonPoints_tmp.back().y);
        #endif
    }

    // std::vector<StwayXY> polygonPoints = ConvexHull(polygonPoints_tmp);//convexPolygon(polygonPoints_tmp);ConvexHull
    std::vector<StwayXY> polygonPoints = polygonPoints_tmp;
    // vector<StwayXY> polygonPoints(convexPolygon(polygonPoints_tmp));
    //vector<StwayXY> polygonPoints = polygonPoints_tmp;

    #ifdef DEBUG
    printf("convexPolygon vertex:x:y %d\n", (int)polygonPoints.size());
    #endif

    for (int i=0; i<polygonPoints.size(); i++) {
        #ifdef DEBUG
        printf("convexPolygon vertex:x:y %f %f\n", polygonPoints[i].x,polygonPoints[i].y);
        #endif
    }

    // double coveredArea = 0.0;
    // for (int i=0; i<polygonPoints.count(); i++) {
    //     if (i != 0) {
    //         coveredArea += polygonPoints[i - 1].x() * polygonPoints[i].y() - polygonPoints[i].x() * polygonPoints[i -1].y();
    //     } else {
    //         coveredArea += polygonPoints.last().x() * polygonPoints[i].y() - polygonPoints[i].x() * polygonPoints.last().y();
    //     }
    // }
    // _setCoveredArea(0.5 * fabs(coveredArea));

    // Generate grid
    int cameraShots = 0;
    cameraShots += gridGenerator(polygonPoints, transectSegments, false /* refly */);
    convertTransectToGeo(transectSegments, tangentOrigin, _transectSegments);
    // _adjustTransectsToEntryPointLocation(_transectSegments);
    appendGridPointsFromTransects(_transectSegments);
    // if (_refly90Degrees) {
    //     QVariantList reflyPointsGeo;

    //     transectSegments.clear();
    //     cameraShots += _gridGenerator(polygonPoints, transectSegments, true /* refly */);
    //     _convertTransectToGeo(transectSegments, tangentOrigin, _reflyTransectSegments);
    //     _optimizeTransectsForShortestDistance(_transectSegments.last().last(), _reflyTransectSegments);
    //     _appendGridPointsFromTransects(_reflyTransectSegments);
    // }

    // Calc survey distance
    double surveyDistance = 0.0;
    for (int i=1; i<_simpleGridPoints.size(); i++) {
        StwayGPS coord1 = _simpleGridPoints[i-1];
        StwayGPS coord2 = _simpleGridPoints[i];
        surveyDistance += get_distance_to_next_waypoint(coord1.lat,coord1.lon,coord2.lat,coord2.lon);
    }
    // _setSurveyDistance(surveyDistance);
    _surveyDistance = surveyDistance;

    if (cameraShots == 0 && _gridTriggerCamera) {
        cameraShots = (int)floor(surveyDistance / _gridTriggerCameraDist);
        // Take into account immediate camera trigger at waypoint entry
        cameraShots++;
    }
    // _setCameraShots(cameraShots);
    _cameraShots = cameraShots;

    if (_hoverAndCaptureEnabled) {
        _additionalFlightDelaySeconds = cameraShots * _hoverAndCaptureDelaySeconds;
        #ifdef DEBUG
            printf("_additionalFlightDelaySeconds %f\n", _additionalFlightDelaySeconds);
        #endif
    }
    // emit additionalTimeDelayChanged(_additionalFlightDelaySeconds);

    // emit gridPointsChanged();

    // Determine command count for lastSequenceNumber

    _missionCommandCount= 0;
    // _hoverAndCaptureEnabled = true;
    for (int i=0; i<_transectSegments.size(); i++) {
        const vector<StwayGPS>& transectSegment = _transectSegments[i];

        _missionCommandCount += transectSegment.size();    // This accounts for all waypoints
        if (_hoverAndCaptureEnabled) {
            // Internal camera trigger points are entry point, plus all points before exit point
            _missionCommandCount += transectSegment.size() - (_turnaroundDist > 0 ? 2 : 0) - 1;
        } else if (_gridTriggerCamera) {
            _missionCommandCount += 2;                          // Camera on/off at entry/exit
        }
    }
    // emit lastSequenceNumberChanged(lastSequenceNumber());

    // Set exit coordinate
    if (_simpleGridPoints.size()) {
        StwayGPS coordinate = _simpleGridPoints.front();
        // coordinate.setAltitude(_gridAltitudeFact.rawValue().toDouble());
        // setCoordinate(coordinate);
        _coordinate = coordinate;
        StwayGPS exitCoordinate = _simpleGridPoints.back();
        // _setExitCoordinate(exitCoordinate);
        _exitCoordinate = exitCoordinate;
    }

    // setDirty(true);
}

int gridGenerator(const vector<StwayXY>& polygonPoints,  vector<vector<StwayXY> >& transectSegments, bool refly)
{
    int cameraShots = 0;

    double gridAngle = _gridAngle;
    double gridSpacing = _gridSpacing;

    gridAngle = clampGridAngle90(gridAngle);
    gridAngle += refly ? 90 : 0;
    // qCDebug(SurveyMissionItemLog) << "Clamped grid angle" << gridAngle;
#ifdef DEBUG
    printf("Clamped grid angle %f\n",gridAngle);
#endif

    // qCDebug(SurveyMissionItemLog) << "SurveyMissionItem::_gridGenerator gridSpacing:gridAngle:refly" << gridSpacing << gridAngle << refly;

    transectSegments.clear();

    // Convert polygon to bounding rect

    // qCDebug(SurveyMissionItemLog) << "Polygon";
#ifdef DEBUG
    printf("Polygon\n");
#endif
    vector<StwayXY> polygon;
    for (int i=0; i<polygonPoints.size(); i++) {
        // qCDebug(SurveyMissionItemLog) << polygonPoints[i];
        polygon.push_back(polygonPoints[i]);
    }
    polygon.push_back(polygonPoints[0]);
    // QRectF smallBoundRect = polygon.boundingRect();
    RectXY smallBoundRect = boundingRect(polygon);
#ifdef DEBUG
    printf("smallBoundRect %d %f %f %f %f\n",(int)polygon.size(),smallBoundRect.x,smallBoundRect.y,smallBoundRect.width,smallBoundRect.lenth);
#endif
    StwayXY boundingCenter = center(smallBoundRect);
#ifdef DEBUG    
    printf("Bounding rect center %f %f\n",boundingCenter.x,boundingCenter.y);
    // qCDebug(SurveyMissionItemLog) << "Bounding rect" << smallBoundRect.topLeft().x() << smallBoundRect.topLeft().y() << smallBoundRect.bottomRight().x() << smallBoundRect.bottomRight().y();
    printf("Bounding rect\n");
#endif
    // Rotate the bounding rect around it's center to generate the larger bounding rect
    vector<StwayXY> boundPolygon;
    StwayXY rotatePoint_temp;
    rotatePoint_temp.x = smallBoundRect.x;
    rotatePoint_temp.y = smallBoundRect.y;
    boundPolygon.push_back(rotatePoint(rotatePoint_temp, boundingCenter, gridAngle));
    rotatePoint_temp.x = smallBoundRect.x + smallBoundRect.width;
    rotatePoint_temp.y = smallBoundRect.y;
    boundPolygon.push_back(rotatePoint(rotatePoint_temp, boundingCenter, gridAngle));
    rotatePoint_temp.x = smallBoundRect.x + smallBoundRect.width;
    rotatePoint_temp.y = smallBoundRect.y + smallBoundRect.lenth;
    boundPolygon.push_back(rotatePoint(rotatePoint_temp, boundingCenter, gridAngle));
    rotatePoint_temp.x = smallBoundRect.x;
    rotatePoint_temp.y = smallBoundRect.y + smallBoundRect.lenth;
    boundPolygon.push_back(rotatePoint(rotatePoint_temp, boundingCenter, gridAngle));
    boundPolygon.push_back(boundPolygon[0]);

    for (int i=0; i<boundPolygon.size(); i++) {
        #ifdef DEBUG
        printf("boundPolygon:x:y %f %f\n", boundPolygon[i].x,boundPolygon[i].y);
        #endif
    }

    RectXY largeBoundRect = boundingRect(boundPolygon);
#ifdef DEBUG
    printf("smallBoundRect %d %f %f %f %f\n",(int)boundPolygon.size(),largeBoundRect.x,largeBoundRect.y,largeBoundRect.width,largeBoundRect.lenth);
#endif
//     qCDebug(SurveyMissionItemLog) << "Rotated bounding rect" << largeBoundRect.topLeft().x() << largeBoundRect.topLeft().y() << largeBoundRect.bottomRight().x() << largeBoundRect.bottomRight().y();
#ifdef DEBUG
    printf("Rotated bounding rect\n");
#endif
    // Create set of rotated parallel lines within the expanded bounding rect. Make the lines larger than the
    // bounding box to guarantee intersection.

    vector<LineXY> lineList;
    bool northSouthTransects = gridAngleIsNorthSouthTransects();
#ifdef DEBUG
    printf("%d\n",northSouthTransects);
#endif
    int entryLocation = _gridEntryLocation;

    if (northSouthTransects) {
        // qCDebug(SurveyMissionItemLog) << "Clamped grid angle" << gridAngle;
        if (entryLocation == EntryLocationTopLeft || entryLocation == EntryLocationBottomLeft) {
            // Generate transects from left to right
            // qCDebug(SurveyMissionItemLog) << "Generate left to right";
            float x = largeBoundRect.x;// - (gridSpacing / 2);
            while (x < largeBoundRect.x + largeBoundRect.width) {
                float yTop =    largeBoundRect.y;// - 10000.0;
                float yBottom = largeBoundRect.y + largeBoundRect.lenth;// + 10000.0;

                LineXY lineList_temp = getLine(x, yTop, x, yBottom, boundingCenter, gridAngle);
                lineList.push_back(lineList_temp);
                // qCDebug(SurveyMissionItemLog) << "line(" << lineList.last().x1() << ", " << lineList.last().y1() << ")-(" << lineList.last().x2() <<", " << lineList.last().y2() << ")";
#ifdef DEBUG
    printf("SurveyMissionItemLogs %f %f %f %f\n",lineList.back().start.x,lineList.back().start.y,lineList.back().stop.x,lineList.back().stop.y);
#endif
                x += gridSpacing;  
            }
        } else {
            // Generate transects from right to left
            // qCDebug(SurveyMissionItemLog) << "Generate right to left";
            float x = largeBoundRect.x + largeBoundRect.width;// + (gridSpacing / 2);
            while (x > largeBoundRect.x) {
                float yTop =    largeBoundRect.y;// - 10000.0;
                float yBottom = largeBoundRect.y- largeBoundRect.lenth;//+ 10000.0;

                LineXY lineList_temp = getLine(x, yTop, x, yBottom, boundingCenter, gridAngle);
                lineList.push_back(lineList_temp);

                // lineList += QLineF(_rotatePoint(QPointF(x, yTop), boundingCenter, gridAngle), _rotatePoint(QPointF(x, yBottom), boundingCenter, gridAngle));
                // qCDebug(SurveyMissionItemLog) << "line(" << lineList.last().x1() << ", " << lineList.last().y1() << ")-(" << lineList.last().x2() <<", " << lineList.last().y2() << ")";
#ifdef DEBUG
    printf("SurveyMissionItemLogs %f %f %f %f\n",lineList.back().start.x,lineList.back().start.y,lineList.back().stop.x,lineList.back().stop.y);
#endif
                x -= gridSpacing;
            }
        }
    } else {
        gridAngle = clampGridAngle90(gridAngle - 90.0);
        // qCDebug(SurveyMissionItemLog) << "Clamped grid angle" << gridAngle;
        if (entryLocation == EntryLocationTopLeft || entryLocation == EntryLocationTopRight) {
            // Generate transects from top to bottom
            // qCDebug(SurveyMissionItemLog) << "Generate top to bottom";
            float y = largeBoundRect.y;// + (gridSpacing / 2);
            while (y > largeBoundRect.y) {
                float xLeft =   largeBoundRect.x;// - 10000.0;
                float xRight =  largeBoundRect.x + largeBoundRect.width;// + 10000.0;

                LineXY lineList_temp = getLine(xLeft, y, xRight, y, boundingCenter, gridAngle);
                lineList.push_back(lineList_temp);
                // lineList += QLineF(_rotatePoint(QPointF(xLeft, y), boundingCenter, gridAngle), _rotatePoint(QPointF(xRight, y), boundingCenter, gridAngle));
                // qCDebug(SurveyMissionItemLog) << "y:xLeft:xRight" << y << xLeft << xRight << "line(" << lineList.last().x1() << ", " << lineList.last().y1() << ")-(" << lineList.last().x2() <<", " << lineList.last().y2() << ")";

                y -= gridSpacing;
            }
        } else {
            // Generate transects from bottom to top
            // qCDebug(SurveyMissionItemLog) << "Generate bottom to top";
            float y = largeBoundRect.y - (gridSpacing / 2);
            while (y < largeBoundRect.y-largeBoundRect.lenth) {
                float xLeft =   largeBoundRect.x - 10000.0;
                float xRight =  largeBoundRect.x+largeBoundRect.width + 10000.0;

                LineXY lineList_temp = getLine(xLeft, y, xRight, y, boundingCenter, gridAngle);
                lineList.push_back(lineList_temp);
                // lineList += QLineF(_rotatePoint(QPointF(xLeft, y), boundingCenter, gridAngle), _rotatePoint(QPointF(xRight, y), boundingCenter, gridAngle));
                // qCDebug(SurveyMissionItemLog) << "y:xLeft:xRight" << y << xLeft << xRight << "line(" << lineList.last().x1() << ", " << lineList.last().y1() << ")-(" << lineList.last().x2() <<", " << lineList.last().y2() << ")";

                y += gridSpacing;
            }
        }
    }
#ifdef DEBUG
    printf("intersectLinesWithPolygon %d\n",(int)lineList.size());
#endif
    // Now intersect the lines with the polygon
    vector<LineXY> intersectLines;
#if 1
    intersectLinesWithPolygon(lineList, polygon, intersectLines);
#else
    // This is handy for debugging grid problems, not for release
    intersectLines = lineList;
#endif

    // Less than two transects intersected with the polygon:
    //      Create a single transect which goes through the center of the polygon
    //      Intersect it with the polygon
    if (intersectLines.size() < 2) {
        // _gridPlygon.center();
        LineXY firstLine = lineList.front();
        StwayXY lineCenter = pointAt(firstLine,0.5);
        StwayXY centerOffset;
        centerOffset.x = boundingCenter.x - lineCenter.x;
        centerOffset.y = boundingCenter.y - lineCenter.y;
        firstLine = getTranslate(firstLine,centerOffset);
        lineList.clear();
        lineList.push_back(firstLine);
        intersectLines = lineList;
        intersectLinesWithPolygon(lineList, polygon, intersectLines);
    }

//     // Make sure all lines are going to same direction. Polygon intersection leads to line which
//     // can be in varied directions depending on the order of the intesecting sides.
    vector<LineXY> resultLines;
    adjustLineDirection(intersectLines, resultLines);
#ifdef DEBUG
printf(" Calc camera shots %d\n",(int)intersectLines.size());
#endif
    // Calc camera shots here if there are no images in turnaround
    if (_gridTriggerCamera) {
        for (int i=0; i<resultLines.size(); i++) {
            cameraShots += (int)floor(lineLenth(resultLines[i]) / _gridTriggerCameraDist);
            // Take into account immediate camera trigger at waypoint entry
            printf("cameraShots %d\n",(int)floor(lineLenth(resultLines[i]) / _gridTriggerCameraDist));
            cameraShots++;
        }
    }
#ifdef DEBUG
printf(" Turn into a path \n");
#endif
    // Turn into a path
    for (int i=0; i<resultLines.size(); i++) {
        LineXY           transectLine;
        vector<StwayXY>  transectPoints;
        const LineXY&    line = resultLines[i];
#ifdef DEBUG
printf(" Polygon entry point 1\n");
#endif
        float turnaroundPosition = _turnaroundDist / lineLenth(line);

        if (i & 1) {
            // transectLine = QLineF(line.p2(), line.p1());
            transectLine.start = line.stop;
            transectLine.stop = line.start;
        } else {
            transectLine.start = line.start;
            transectLine.stop = line.stop;
            // transectLine = QLineF(line.p1(), line.p2());
        }

        // Build the points along the transect
// #ifdef DEBUG
// printf(" Polygon entry point 1\n");
// #endif
        if (_turnaroundDist > 0 ) {
            // transectPoints.append(transectLine.pointAt(-turnaroundPosition));
            transectPoints.push_back(pointAt(transectLine, -turnaroundPosition));
        }

        // Polygon entry point
        transectPoints.push_back(transectLine.start);
// #ifdef DEBUG
// printf(" Polygon entry point 2\n");
// #endif
        // For hover and capture we need points for each camera location
        // _hoverAndCaptureEnabled = true;
        if (_gridTriggerCamera && _hoverAndCaptureEnabled) {
            if (_gridTriggerCameraDist < lineLenth(transectLine)) {
                int innerPoints = floor(lineLenth(transectLine) / _gridTriggerCameraDist);
                // qCDebug(SurveyMissionItemLog) << "innerPoints" << innerPoints;
                float transectPositionIncrement = _gridTriggerCameraDist / lineLenth(transectLine);
                for (int i=0; i<innerPoints; i++) {
                    transectPoints.push_back(pointAt(transectLine,transectPositionIncrement * (i + 1)));
                }
            }
        }
// #ifdef DEBUG
// printf(" Polygon exit point \n");
// #endif
        // Polygon exit point
        transectPoints.push_back(transectLine.stop);

        if (_turnaroundDist > 0 ) {
            transectPoints.push_back(pointAt(transectLine,1 + turnaroundPosition));
        }

        transectSegments.push_back(transectPoints);
    }

    return cameraShots;
}

// double ccw(StwayXY pt1, StwayXY pt2, StwayXY pt3)
// {
//     return (pt2.x-pt1.x)*(pt3.y-pt1.y) - (pt2.y-pt1.y)*(pt3.x-pt1.x);
// }

// double dp(StwayXY pt1, StwayXY pt2)
// {
//     double a1;
//     if((int)pt2.x == (int)pt1.x){
//         a1 = 3;
//         // printf("222222222222\n");
//         // b1 = line1.start.x;
//     }else if((int)pt2.y == (int)pt1.y){
//         a1 = 0;
//         // printf("33333333333333\n");
//         // b1 = line1.start.y;
//     }else{
//         a1 = (pt2.y - pt1.y) / (pt2.x - pt1.x);
//         // printf("1111111111111 %f %f %f %f %f\n",a1,(pt2.y - pt1.y),(pt2.x - pt1.x), pt2.x, pt1.x);
//         if(abs(int(a1)) > 2)
//             a1 = int(a1) > 0? 3 : -3;
//         // b1 = pt1.y - a1 * (pt1.x);
//     }
//     return a1;
//     // return (pt2.x-pt1.x)/sqrt((pt2.x-pt1.x)*(pt2.x-pt1.x) + (pt2.y-pt1.y)*(pt2.y-pt1.y));
// }

// void swapPoints(vector<StwayXY>& points, int index1, int index2)
// {
//     StwayXY temp = points[index1];
//     points[index1] = points[index2];
//     points[index2] = temp;
// }

// std::vector<StwayXY> convexPolygon(vector<StwayXY>  polygon)
// {
//     // We use the Graham scan algorithem to convert the possibly concave polygon to a convex polygon
//     // https://en.wikipedia.org/wiki/Graham_scan

//     vector<StwayXY> workPolygon = polygon;
//     StwayXY tmp;
//     #ifdef DEBUG
//     printf(" convexPolygon \n");
//     #endif

//     // First point must be lowest y-coordinate point
//     for (int i=1; i<workPolygon.size(); i++) {
//         if (workPolygon[i].y < workPolygon[0].y) {
//             swapPoints(workPolygon, i, 0);
//             // tmp = workPolygon[i];
//             // workPolygon[i] = workPolygon[0];
//             // workPolygon[0] = tmp;
//         }
//     }

//     for (int i=0; i<workPolygon.size(); i++) {
//         #ifdef DEBUG
//         printf(" convexPolygon2 x:y %f %f \n",workPolygon[i].x,workPolygon[i].y);
//         #endif
//     }

//      #ifdef DEBUG
//     printf(" convexPolygon 2 \n");
//     #endif

//     // Sort the points by angle with first point
//     // for (int i=1; i<workPolygon.size(); i++) {
//     //     double angle = dp(workPolygon[0], workPolygon[i]);
//     //     for (int j=i+1; j<workPolygon.size(); j++) {

//     //         printf("%d %d %f %f %f %f\n",i,j,dp(workPolygon[0], workPolygon[j]),angle,workPolygon[j].x,workPolygon[0].x);
//     //         if (dp(workPolygon[0], workPolygon[j]) > angle) {
//     //             swapPoints(workPolygon, i, j);
//     //             printf("33333333333333\n");
//     //             for (int i=0; i<workPolygon.size(); i++) {
//     //                 #ifdef DEBUG
//     //                 printf(" convexPolygon3 x:y %f %f \n",workPolygon[i].x,workPolygon[i].y);
//     //                 #endif
//     //             }
//     //             // tmp = workPolygon[i];
//     //             // workPolygon[i] = workPolygon[j];
//     //             // workPolygon[j] = tmp;
//     //             // angle = dp(workPolygon[0], workPolygon[j]);
//     //         }
//     //     }
//     // }

//     for (int i=0; i<workPolygon.size(); i++) {
//         #ifdef DEBUG
//         printf(" convexPolygon1 x:y %f %f \n",workPolygon[i].x,workPolygon[i].y);
//         #endif
//     }
//      #ifdef DEBUG
//     printf(" convexPolygon 1\n");
//     #endif

//     // Perform the the Graham scan

//     workPolygon[0] = workPolygon.back();  // Sentinel for algo stop
//     int convexCount = 1;                        // Number of points on the convex hull.

//     for (int i=2; i<polygon.size(); i++) {
//         while (ccw(workPolygon[convexCount-1], workPolygon[convexCount], workPolygon[i]) <= 0) {
//             if (convexCount > 1) {
//                 convexCount -= 1;
//             } else if (i == polygon.size() - 1) {
//                 break;
//             } else {
//                 i++;
//             }
//         }
//         convexCount++;
//         printf("%d %d\n",convexCount,i);
//         swapPoints(workPolygon, convexCount, i);
//         // tmp = workPolygon[convexCount];
//         // workPolygon[convexCount] = workPolygon[i];
//         // workPolygon[i] = tmp;
//     }
//     for (int i=0; i<workPolygon.size(); i++) {
//         #ifdef DEBUG
//         printf(" convexPolygon3 x:y %f %f \n",workPolygon[i].x,workPolygon[i].y);
//         #endif
//     }
//      #ifdef DEBUG
//     printf(" convexPolygon 3 %d %d\n",(int)workPolygon.size(),convexCount);
//     #endif
//     vector<StwayXY> res(workPolygon.begin(),workPolygon.begin()+convexCount);
//     // polygonPoints =  (vector<StwayXY>)malloc(convexCount*sizeof(StwayXY));
//     // for(int i = 0; i < convexCount; i ++)
//     //     res.push_back(workPolygon[i]);

//     for (int i=0; i<res.size(); i++) {
//         #ifdef DEBUG
//         printf(" res x:y %f %f \n",res[i].x,res[i].y);
//         #endif
//     }
//     // polygonPoints.swap(res);
//     //  #ifdef DEBUG
//     // printf(" convexPolygon 4\n");
//     // #endif
//     return res;
// }

// //向量（x1,y1）,(x2,y2)的叉积
// double CrossMul(StwayXY x1,StwayXY x2)
// {
//     // return x1*y2-x2*y1;
//     return x1.x*x2.y-x2.x*x1.y;
// }
// //向量（x1,y1）,(x2,y2)的点积
// double DotMul(StwayXY x1,StwayXY x2)
// {
//     // return x1*x2+y1*y2;
//     return x1.x*x2.x+x1.y*x2.y;
// }
//向量（x1,y1）,(x2,y2)的叉积
double CrossMul(double x1,double y1,double x2,double y2)
{
    return x1*y2-x2*y1;
}
//向量（x1,y1）,(x2,y2)的点积
double DotMul(double x1,double y1,double x2,double y2)
{
    return x1*x2+y1*y2;
}
//跨立判断
//判断点c是在向量ab的逆时针方向还是顺时针方向，大于零逆时针，等于0则共线
double CrossMul(StwayXY a,StwayXY b,StwayXY c)
{
    return CrossMul(b.x-a.x,b.y-a.y,c.x-a.x,c.y-a.y);
}
//计算向量ab和ac点积
double DotMul(StwayXY a,StwayXY b,StwayXY c)
{
    return DotMul(b.x-a.x,b.y-a.y,c.x-a.x,c.y-a.y);
}
//判断浮点数符号
int doublecmp(double d)
{
    if(fabs(d)<10e-6)
        return 0;
    return d>0?1:-1;
}
//判断同一直线上的三个点位置，点c是否在点ab之间
bool betweenCmp(StwayXY a,StwayXY b,StwayXY c)
{
    if(doublecmp(DotMul(c,a,b))<=0)
        return true;
    return false;
}
//判断j是否在base->i向量的左边或当共线时j是否位于它们的线段之间
bool isLeftorNearer(StwayXY base,StwayXY i,StwayXY j)
{
    if(CrossMul(base,i,j)>0)
        return true;
    if(CrossMul(base,i,j)==0 && betweenCmp(base,i,j))
        return true;
    return false;
}
void swap(StwayXY& a,StwayXY& b)
{
    StwayXY temp = b;
    b=a;
    a=temp;
}
//以s中的最低点为参考点，对其他所有点进行极角排序（逆时针）
//共线时离参考点较远的点排在前面，凸包的起始边共线点从近到远排列
void sortpoint(std::vector<StwayXY> s)
{
    //找最低点
    for(int i=1;i<s.size();i++)
    {
        if(s[i].y<s[0].y || (s[i].y==s[0].y && s[i].x<s[0].x))
            swap(s[0],s[i]);
    }
    #ifdef DEBUG
        for(int i=0;i<s.size();i++)
        {
            printf(" sortpoint %f %f\n",s[i].x,s[i].y);
        }
    #endif
    qsortpoint(s,1,(int)s.size());
    #ifdef DEBUG
        for(int i=0;i<s.size();i++)
        {
            printf(" qsortpoint %f %f\n",s[i].x,s[i].y);
        }
    #endif
    // 将起始边上的共线点重新排列
    sortstartedge(s);
    #ifdef DEBUG
        for(int i=0;i<s.size();i++)
        {
            printf(" sortstartedge %f %f\n",s[i].x,s[i].y);
        }
    #endif
    // return;
}
void sortstartedge(std::vector<StwayXY> s)
{
    int i,j;
    for(i=2;i<s.size();i++)
    {
        if(CrossMul(s[0],s[1],s[i])!=0)
            break;
    }
    for(j=1;j<(i+1)/2;j++)
        swap(s[j],s[i-j]);
}

//将点按极角逆时针排序
void qsortpoint(std::vector<StwayXY> s,int start,int end)
{
    if(start>=end)
        return;
    StwayXY partition = s[end-1];
    int i=start-1,j=start-1;
    while(++j<end-1)
    {
        if(isLeftorNearer(s[0],s[j],partition))
        {
            swap(s[++i],s[j]);
        }
    }
    swap(s[++i],s[end-1]);
    qsortpoint(s,start,i);
    qsortpoint(s,i+1,end);
}
// std::vector<StwayXY> convexPolygon(vector<StwayXY>  polygon)
std::vector<StwayXY> ConvexHull(vector<StwayXY>  s)
{
    vector<StwayXY> result;
    sortpoint(s);

    if(s.size()<=3)
        return s;
    #ifdef DEBUG
    printf("CrossMul11111 \n");
    #endif
    int top=2;
    int i;
    for(i=0;i<2;i++)
        result.push_back(s[i]);
    while(i<s.size())
    {
        //用<号判断则包含凸包边上的共线点，<=号判断则只包含凸包顶点
        if(top >= 2 && CrossMul(result[top-2],result[top-1],s[i])<=0)
        {
            
            #ifdef DEBUG
            printf("CrossMul11111 %d %f %f %f %f %f %f\n",top,result[top-2].x,result[top-2].y,result[top-1].x,result[top-1].y,s[i].x,s[i].y);
            #endif
            top--;
        }
        else
        {
             #ifdef DEBUG
            printf("CrossMul222222 %d %f %f %f %f %f %f\n",top,result[top-2].x,result[top-2].y,result[top-1].x,result[top-1].y,s[i].x,s[i].y);
            #endif
            result.push_back(s[i++]);
            top ++;
        }
    }
    #ifdef DEBUG
    printf("top1 %d\n",top);
    #endif
    //最后加入起点形成闭包
    while(top > 2 && CrossMul(result[top-2],result[top-1],s[0])<=0 )
    {
        top--;
    }
    #ifdef DEBUG
    printf("top2 %d\n",top);
    #endif
    vector<StwayXY> res(result.begin(),result.begin()+top);
    return res;
}

void adjustLineDirection(const vector<LineXY>& lineList, vector<LineXY>& resultLines)
{
    // qreal firstAngle = 0;
    // for (int i=0; i<lineList.count(); i++) {
    //     const QLineF& line = lineList[i];
    //     QLineF adjustedLine;

    //     if (i == 0) {
    //         firstAngle = line.angle();
    //     }

    //     if (qAbs(line.angle() - firstAngle) > 1.0) {
    //         adjustedLine.setP1(line.p2());
    //         adjustedLine.setP2(line.p1());
    //     } else {
    //         adjustedLine = line;
    //     }

    //     resultLines += adjustedLine;
    // }
    resultLines = lineList;
}

double lineLenth(LineXY line){
    return sqrt(pow(line.stop.x - line.start.x, 2)+pow(line.stop.y - line.start.y, 2));
}

LineXY getTranslate(LineXY line, StwayXY center){
    LineXY temp;
    temp.start.x = line.start.x+center.x;
    temp.start.y = line.start.y+center.y;
    temp.stop.x = line.stop.x+center.x;
    temp.stop.y = line.stop.y+center.y;
    return temp;
}

StwayXY pointAt(LineXY line, double proportion){
    StwayXY res;
    double s = sqrt(pow(line.stop.x - line.start.x, 2)+pow(line.stop.y - line.start.y, 2));
    res.x = (line.stop.x - line.start.x)*proportion + line.start.x;
    res.y = (line.stop.y - line.start.y)*proportion+line.start.y;
    return res;
}


void intersectLinesWithPolygon(const vector<LineXY>& lineList, const vector<StwayXY>& polygon, vector<LineXY>& resultLines)
{
    resultLines.clear();
    #ifdef DEBUG
    printf(" intersectLinesWithPolygon  %d\n",(int)lineList.size());
    #endif
    for (int i=0; i<lineList.size(); i++) {
        int foundCount = 0;
        LineXY intersectLine;
        const LineXY& line = lineList[i];


        for (int j=0; j<polygon.size()-1; j++) {
            StwayXY intersectPoint;
            // StwayXY polygonLine_temp_1;
            // StwayXY polygonLine_temp_2;
            // polygonLine_temp_1 = polygon[j];
            // polygonLine_temp_2 = polygon[j+1];

            LineXY polygonLine;
            polygonLine.start = polygon[j];
            polygonLine.stop = polygon[j+1];

            intersectPoint = getCross(line, polygonLine);
            #ifdef DEBUG
                printf(" polygonLine %f %f %f %f \n",polygonLine.start.x,polygonLine.start.y,polygonLine.stop.x,polygonLine.stop.y);
                printf(" line %f %f %f %f \n",line.start.x,line.start.y,line.stop.x,line.stop.y);
                if(intersectPoint.x != DBL_MAX)
                    printf(" intersectPoint %f %f \n ",intersectPoint.x,intersectPoint.y);
                // printf(" intersectPoint %f %f \n ",intersectPoint.x,intersectPoint.y);
            #endif
            
            if(!(intersectPoint.x == DBL_MAX && intersectPoint.y == DBL_MAX)){
                // #ifdef DEBUG
                //     printf(" polygonLine %f %f %f %f \n",polygonLine.start.x,polygonLine.start.y,polygonLine.stop.x,polygonLine.stop.y);
                //     printf(" line %f %f %f %f \n",line.start.x,line.start.y,line.stop.x,line.stop.y);
                //     printf(" intersectPoint %f %f \n ",intersectPoint.x,intersectPoint.y);
                // #endif
                if (foundCount == 0) {
                    foundCount++;
                    intersectLine.start = intersectPoint;
                } else {
                    foundCount++;
                    intersectLine.stop = intersectPoint;
                    break;
                }
            }

        }

        if (foundCount == 2) {
            resultLines.push_back(intersectLine);
        }
    }
}

StwayXY getCross(LineXY line1, LineXY line2)
{
    StwayXY CrossP;
    //y = a * x + b;


    double a1,b1,a2,b2;
    if((int)line1.start.x == (int)line1.stop.x){
        a1 = 3;
        b1 = line1.start.x;
    }else if((int)line1.start.y == (int)line1.stop.y){
        a1 = 0;
        b1 = line1.start.y;
    }else{
        a1 = (line1.start.y - line1.stop.y) / (line1.start.x - line1.stop.x);
        if(abs(int(a1)) > 2)
            a1 = 3;
        b1 = line1.start.y - a1 * (line1.start.x);
    }

    if((int)line2.start.x == (int)line2.stop.x){
        a2 = 3;
        b2 = line2.start.x;
    }else if((int)line2.start.y == (int)line2.stop.y){
        a2 = 0;
        b2 = line2.start.y;
    }else{
        a2 = (line2.start.y - line2.stop.y) / (line2.start.x - line2.stop.x);
        if(abs(int(a2)) > 2)
            a1 = 3;
        b2 = line2.start.y - a2 * (line2.start.x);
    }

    if((int)a2 == (int)a1){
        CrossP.x = DBL_MAX;
        CrossP.y = DBL_MAX;
        #ifdef DEBUG
            printf("22222222222222222 %d %d\n",int(a1),int(a2));
            // if(CrossP.x != DBL_MAX)
            // printf(" getCross444 %d %f %d\n ",abs(a1),fabs(a1),abs(a2));
        #endif
        if(int(a1) == 3 && int(a2) == 3 ){
            CrossP.x = DBL_MAX;
            CrossP.y = DBL_MAX;
            #ifdef DEBUG
                printf("11111111111111111 %d %d\n",int(a1),int(a2));
                // if(CrossP.x != DBL_MAX)
                // printf(" 11111111111111111 %d %d\n ",abs(a1-DBL_MAX),abs(a1-DBL_MAX));
            #endif
        }
        
    }else{
        if((abs(int(a2)) == 3 && abs(int(a1)) == 0)){
            CrossP.x = b2;
            CrossP.y = b1;
        }else if((abs(int(a1)) == 3 && abs(int(a2)) == 0)){
            CrossP.x = b1;
            CrossP.y = b2;
        }else if(abs(int(a1)) == 3){
            CrossP.x = b1;
            CrossP.y = a1 * CrossP.x + b1;
        }else if(abs(int(a2)) == 3){
            CrossP.x = b2;
            CrossP.y = a1 * CrossP.x + b1;
        }else{
            CrossP.x = (b1 - b2) / (a2 - a1);
            CrossP.y = a1 * CrossP.x + b1;
        }
        

        //if(line1.start.x == line1.stop.x || line1.start.y == line1.stop.y || line2.start.x == line2.stop.x || line2.start.y == line2.stop.y)

        // if((line1.start.x <= line1.stop.x && CrossP.x >= line1.start.x && CrossP.x <= line1.stop.x)){
        //     CrossP.x = -0.0;
        //     CrossP.y = 0.0;
        // }else
        #ifdef DEBUG
            printf(" getCross %f %f %f %f \n",a1,a2,b1,b2);
            // if(CrossP.x != DBL_MAX)
            printf(" getCross333 %f %f \n ",CrossP.x,CrossP.y);
        #endif

        if(((int)line1.start.x == (int)line1.stop.x && (int)CrossP.x != (int)line1.start.x) || ((int)line1.start.y == (int)line1.stop.y && (int)CrossP.y != (int)line1.start.y)){
            #ifdef DEBUG
                printf(" getCross222 %f %f\n",CrossP.x ,CrossP.y);
                // printf(" intersectPoint %f %f \n ",intersectPoint.x,intersectPoint.y);
            #endif
            CrossP.x = DBL_MAX;
            CrossP.y = DBL_MAX;
        }else if(((int)line2.start.x == (int)line2.stop.x && (int)CrossP.x != (int)line2.start.x) || ((int)line2.start.y == (int)line2.stop.y && (int)CrossP.y != (int)line2.start.y)){
            #ifdef DEBUG
                printf(" getCross444 %f %f\n",CrossP.x ,CrossP.y);
                // printf(" intersectPoint %f %f \n ",intersectPoint.x,intersectPoint.y);
            #endif
            CrossP.x = DBL_MAX;
            CrossP.y = DBL_MAX;
         }else if((int)((int)CrossP.x - (int)line2.start.x)*((int)CrossP.x - (int)line2.stop.x) > 0 || (int)((int)CrossP.y - (int)line2.start.y)*((int)CrossP.y - (int)line2.stop.y) > 0){
            //else if((int)line1.start.x > (int)line1.stop.x){
                #ifdef DEBUG
                    printf(" getCross111 %f %f\n",CrossP.x ,CrossP.y);
                    // printf(" intersectPoint %f %f \n ",intersectPoint.x,intersectPoint.y);
                #endif
                CrossP.x = DBL_MAX;
                CrossP.y = DBL_MAX;
            // }
        }
        // else if((int)line1.stop.x > (int)line1.start.x){
        //     if((int)((int)CrossP.x - (int)line1.start.x)*((int)CrossP.x - (int)line1.stop.x) > 0){
        //         #ifdef DEBUG
        //             printf(" getCross111 %f %f\n",CrossP.x ,CrossP.y);
        //             // printf(" intersectPoint %f %f \n ",intersectPoint.x,intersectPoint.y);
        //         #endif
        //         CrossP.x = DBL_MAX;
        //         CrossP.y = DBL_MAX;
        //     }
        // }
        // }else if( (int)((int)CrossP.x - (int)line1.start.x)*((int)line1.stop.x - (int)CrossP.x) > 0 || (int)((int)CrossP.y - (int)line1.start.y)*((int)line1.stop.y - (int)CrossP.y) > 0){
        //     #ifdef DEBUG
        //         printf(" getCross111 %f %f\n",CrossP.x ,CrossP.y);
        //         // printf(" intersectPoint %f %f \n ",intersectPoint.x,intersectPoint.y);
        //     #endif
        //     CrossP.x = DBL_MAX;
        //     CrossP.y = DBL_MAX;
        // }

    }
    
    #ifdef DEBUG
        // printf(" getCross %f %f %f %f \n",a1,a2,b1,b2);
        // if(CrossP.x != DBL_MAX)
        // printf(" intersectPoint %f %f \n ",CrossP.x,CrossP.y);
    #endif
    return CrossP;
}

LineXY getLine(double x1, double y1, double x2, double y2, const StwayXY& origin, double angle){
    LineXY lineList_temp;
    StwayXY lineList_start_temp_point = {x1,y1};
    StwayXY lineList_start_temp = rotatePoint(lineList_start_temp_point, origin, angle);
    StwayXY lineList_end_temp_point = {x2, y2};
    StwayXY lineList_end_temp = rotatePoint(lineList_end_temp_point, origin, angle);
    lineList_temp.start = lineList_start_temp;
    lineList_temp.stop = lineList_end_temp;
    return lineList_temp;
}

bool gridAngleIsNorthSouthTransects()
{
    // Grid angle ranges from -360<->360
    double gridAngle = fabs(_gridAngle);
    return gridAngle < 45.0 || (gridAngle > 360.0 - 45.0) || (gridAngle > 90.0 + 45.0 && gridAngle < 270.0 - 45.0);
}

StwayXY rotatePoint(const StwayXY& point, const StwayXY& origin, double angle)
{
    StwayXY rotated;
    double radians = (M_PI / 180.0) * -angle;

    rotated.x = ((point.x - origin.x) * cos(radians)) - ((point.y - origin.y) * sin(radians)) + origin.x;
    rotated.y = ((point.x - origin.x) * sin(radians)) + ((point.y - origin.y) * cos(radians)) + origin.y;

    return rotated;
}

StwayXY center(RectXY smallBoundRect)
{
    StwayXY res;
    double d_left = smallBoundRect.x;
    double d_right = smallBoundRect.x + smallBoundRect.width;
    double d_top = smallBoundRect.y;
    double d_bottom = smallBoundRect.y + smallBoundRect.lenth;

    res.x = d_left + (d_right - d_left) / 2.0;
    res.y = d_top + (d_bottom - d_top) / 2.0;
    return res;
}

RectXY boundingRect(vector<StwayXY> polygonPoints)
{
    const size_t sz = polygonPoints.size();
    RectXY res;
    if ( sz <= 0 ){
        res.x = 1.0;
        res.y = 1.0;
        res.width = -2.0;
        res.lenth = -2.0;
        return res; // invalid
    }

    double minX, maxX, minY, maxY;
    minX = maxX = polygonPoints[0].x;
    minY = maxY = polygonPoints[0].y;

    for ( size_t i = 1; i < sz; i++ ) {
        const double xv = polygonPoints[i].x;
        if ( xv < minX )
            minX = xv;
        if ( xv > maxX )
            maxX = xv;

        const double yv = polygonPoints[i].y;
        if ( yv < minY )
            minY = yv;
        if ( yv > maxY )
            maxY = yv;
    }
    res.x = minX;
    res.y = minY;
    res.width = maxX - minX;
    res.lenth = maxY - minY;
    return res;
}

double clampGridAngle90(double gridAngle)
{
    // Clamp grid angle to -90<->90. This prevents transects from being rotated to a reversed order.
    if (gridAngle > 90.0) {
        gridAngle -= 180.0;
    } else if (gridAngle < -90.0) {
        gridAngle += 180;
    }
    return gridAngle;
}

void appendGridPointsFromTransects(vector<vector<StwayGPS> >& rgTransectSegments)
{
    // qCDebug(SurveyMissionItemLog) << "Entry point _appendGridPointsFromTransects" << rgTransectSegments.first().first();
    // printf("Entry point appendGridPointsFromTransects %f %f",rgTransectSegments[0][0].lat,rgTransectSegments[0][0].lon);

    for (int i=0; i<rgTransectSegments.size(); i++) {
        _simpleGridPoints.push_back(rgTransectSegments[i].front());
        _simpleGridPoints.push_back(rgTransectSegments[i].back());
    }
}

void convertTransectToGeo(const vector<vector<StwayXY> >& transectSegmentsNED, const StwayGPS& tangentOrigin, vector<vector<StwayGPS> >& transectSegmentsGeo)
{
    transectSegmentsGeo.clear();

    for (int i=0; i<transectSegmentsNED.size(); i++) {
        vector<StwayGPS>   transectCoords;
        const vector<StwayXY>&   transectPoints = transectSegmentsNED[i];

        for (int j=0; j<transectPoints.size(); j++) {
            StwayGPS coord;
            const StwayXY& point = transectPoints[j];
            convertNedToGeo(point.y, point.x, tangentOrigin, &coord);
            transectCoords.push_back(coord);
        }
        transectSegmentsGeo.push_back(transectCoords);
    }
}

double get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
    double lat_now_rad = lat_now / (double)180.0 * M_PI;
    double lon_now_rad = lon_now / (double)180.0 * M_PI;
    double lat_next_rad = lat_next / (double)180.0 * M_PI;
    double lon_next_rad = lon_next / (double)180.0 * M_PI;


    double d_lat = lat_next_rad - lat_now_rad;
    double d_lon = lon_next_rad - lon_now_rad;

    double a = sin(d_lat / (double)2.0) * sin(d_lat / (double)2.0) + sin(d_lon / (double)2.0) * sin(d_lon /
            (double)2.0) * cos(lat_now_rad) * cos(lat_next_rad);
    double c = (double)2.0 * atan2(sqrt(a), sqrt((double)1.0 - a));

    return CONSTANTS_RADIUS_OF_EARTH * c;
}


void convertGeoToNed(StwayGPS coord, StwayGPS origin, double* x, double* y) {

    double lat_rad = coord.lat * M_DEG_TO_RAD;
    double lon_rad = coord.lon * M_DEG_TO_RAD;

    double ref_lon_rad = origin.lon * M_DEG_TO_RAD;
    double ref_lat_rad = origin.lat * M_DEG_TO_RAD;

    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double cos_d_lon = cos(lon_rad - ref_lon_rad);

    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double c = acos(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon);
    double k = (fabs(c) < epsilon) ? 1.0 : (c / sin(c));

    *x = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
    *y = k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;

    // if(coord.lat == origin.lat)
    //     *x = 0.0;
    // *z = -(coord.altitude() - origin.altitude());
}

void convertNedToGeo(double x, double y, StwayGPS origin, StwayGPS *coord) {
    double x_rad = x / CONSTANTS_RADIUS_OF_EARTH;
    double y_rad = y / CONSTANTS_RADIUS_OF_EARTH;
    double c = sqrtf(x_rad * x_rad + y_rad * y_rad);
    double sin_c = sin(c);
    double cos_c = cos(c);

    double ref_lon_rad = origin.lon * M_DEG_TO_RAD;
    double ref_lat_rad = origin.lat * M_DEG_TO_RAD;

    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double lat_rad;
    double lon_rad;

    if (fabs(c) > epsilon) {
        lat_rad = asin(cos_c * ref_sin_lat + (x_rad * sin_c * ref_cos_lat) / c);
        lon_rad = (ref_lon_rad + atan2(y_rad * sin_c, c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c));

    } else {
        lat_rad = ref_lat_rad;
        lon_rad = ref_lon_rad;
    }

    coord->lat = lat_rad * M_RAD_TO_DEG;
    coord->lon = lon_rad * M_RAD_TO_DEG;
}


