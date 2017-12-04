#include "regmap.h"

void GenRegWaypoint(std::vector<struct wayPointGPS> gridPlygon, double gridSpacing, double gridAngle, 
                                bool gridTriggerCamera, double gridTriggerCameraDist, double _turnaroundDist, uint8_t gridMode, uint8_t gridRefly) 
{  
    _gridPlygon = gridPlygon;
    _gridSpacing = gridSpacing;
    _gridAngle = gridAngle;
    _gridTriggerCamera = gridTriggerCamera;
    _gridTriggerCameraDist = gridTriggerCameraDist;
    _gridMode = gridMode;
    _gridRefly = gridRefly;

    generateGrid();

    FILE *fp;
    int i;
    if( (fp = fopen("dataman.txt","w+")) == NULL){
        if((fp = fopen("dataman.txt","w")) == NULL ){
            return ; 
        } 
    }
    // for(i = 0; i < _gridPlygon.size(); i ++){
    //    fprintf(fp,"%lf %lf\n",gridPlygon[i].lat,gridPlygon[i].lon); 
    // }
    for(i = 0; i < _transectSegments.size(); i ++){
        for(int j = 0; j < _transectSegments[i].size(); j ++)
            fprintf(fp,"%f %f\n",_transectSegments[i][j].lat,_transectSegments[i][j].lon); 
    }
    fclose(fp);
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

    vector<StwayXY>           polygonPoints;
    vector<vector<StwayXY> >   transectSegments;

    // Convert polygon to NED
    StwayGPS tangentOrigin = _gridPlygon[0];
    printf("Convert polygon to NED - tangentOrigin %lf %lf\n",tangentOrigin.lat,tangentOrigin.lon);
    for (int i=0; i<_gridPlygon.size(); i++) {
        StwayXY tmp;
        StwayGPS vertex = _gridPlygon[i];
        if (i == 0) {
            // This avoids a nan calculation that comes out of convertGeoToNed
            tmp.x = tmp.y = 0;
        } else {
            convertGeoToNed(vertex, tangentOrigin, &tmp.y, &tmp.x);
        }
        polygonPoints.push_back(tmp);
        printf("vertex:x:y %f %f\n", polygonPoints.back().x,polygonPoints.back().y);
    }

    // polygonPoints = _convexPolygon(polygonPoints);

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

    // if (_hoverAndCaptureEnabled()) {
    //     _additionalFlightDelaySeconds = cameraShots * _hoverAndCaptureDelaySeconds;
    // }
    // emit additionalTimeDelayChanged(_additionalFlightDelaySeconds);

    // emit gridPointsChanged();

    // Determine command count for lastSequenceNumber

    _missionCommandCount= 0;
    _hoverAndCaptureEnabled = false;
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
    printf("Clamped grid angle %f\n",gridAngle);

    // qCDebug(SurveyMissionItemLog) << "SurveyMissionItem::_gridGenerator gridSpacing:gridAngle:refly" << gridSpacing << gridAngle << refly;

    transectSegments.clear();

    // Convert polygon to bounding rect

    // qCDebug(SurveyMissionItemLog) << "Polygon";
    printf("Polygon\n");
    vector<StwayXY> polygon;
    for (int i=0; i<polygonPoints.size(); i++) {
        // qCDebug(SurveyMissionItemLog) << polygonPoints[i];
        polygon.push_back(polygonPoints[i]);
    }
    polygon.push_back(polygonPoints[0]);
    // QRectF smallBoundRect = polygon.boundingRect();
    RectXY smallBoundRect = boundingRect(polygon);
    StwayXY boundingCenter = center(smallBoundRect);
    // qCDebug(SurveyMissionItemLog) << "Bounding rect" << smallBoundRect.topLeft().x() << smallBoundRect.topLeft().y() << smallBoundRect.bottomRight().x() << smallBoundRect.bottomRight().y();
    printf("Bounding rect\n");
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
    RectXY largeBoundRect = boundingRect(boundPolygon);
//     qCDebug(SurveyMissionItemLog) << "Rotated bounding rect" << largeBoundRect.topLeft().x() << largeBoundRect.topLeft().y() << largeBoundRect.bottomRight().x() << largeBoundRect.bottomRight().y();
    printf("Rotated bounding rect\n");
    // Create set of rotated parallel lines within the expanded bounding rect. Make the lines larger than the
    // bounding box to guarantee intersection.

    vector<LineXY> lineList;
    bool northSouthTransects = gridAngleIsNorthSouthTransects();
    int entryLocation = _gridEntryLocation;

    if (northSouthTransects) {
        // qCDebug(SurveyMissionItemLog) << "Clamped grid angle" << gridAngle;
        if (entryLocation == EntryLocationTopLeft || entryLocation == EntryLocationBottomLeft) {
            // Generate transects from left to right
            // qCDebug(SurveyMissionItemLog) << "Generate left to right";
            float x = largeBoundRect.x - (gridSpacing / 2);
            while (x < largeBoundRect.x + largeBoundRect.width) {
                float yTop =    largeBoundRect.y - 10000.0;
                float yBottom = largeBoundRect.y - largeBoundRect.lenth+ 10000.0;

                LineXY lineList_temp = getLine(x, yTop, x, yBottom, boundingCenter, gridAngle);
                lineList.push_back(lineList_temp);
                // qCDebug(SurveyMissionItemLog) << "line(" << lineList.last().x1() << ", " << lineList.last().y1() << ")-(" << lineList.last().x2() <<", " << lineList.last().y2() << ")";

                x += gridSpacing;
            }
        } else {
            // Generate transects from right to left
            // qCDebug(SurveyMissionItemLog) << "Generate right to left";
            float x = largeBoundRect.x + largeBoundRect.width + (gridSpacing / 2);
            while (x > largeBoundRect.x) {
                float yTop =    largeBoundRect.y - 10000.0;
                float yBottom = largeBoundRect.y- largeBoundRect.lenth+ 10000.0;

                LineXY lineList_temp = getLine(x, yTop, x, yBottom, boundingCenter, gridAngle);
                lineList.push_back(lineList_temp);

                // lineList += QLineF(_rotatePoint(QPointF(x, yTop), boundingCenter, gridAngle), _rotatePoint(QPointF(x, yBottom), boundingCenter, gridAngle));
                // qCDebug(SurveyMissionItemLog) << "line(" << lineList.last().x1() << ", " << lineList.last().y1() << ")-(" << lineList.last().x2() <<", " << lineList.last().y2() << ")";

                x -= gridSpacing;
            }
        }
    } else {
        gridAngle = clampGridAngle90(gridAngle - 90.0);
        // qCDebug(SurveyMissionItemLog) << "Clamped grid angle" << gridAngle;
        if (entryLocation == EntryLocationTopLeft || entryLocation == EntryLocationTopRight) {
            // Generate transects from top to bottom
            // qCDebug(SurveyMissionItemLog) << "Generate top to bottom";
            float y = largeBoundRect.y + (gridSpacing / 2);
            while (y > largeBoundRect.y) {
                float xLeft =   largeBoundRect.x - 10000.0;
                float xRight =  largeBoundRect.x + largeBoundRect.width + 10000.0;

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
    printf("intersectLinesWithPolygon\n");
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
//     _adjustLineDirection(intersectLines, resultLines);
printf(" Calc camera shots\n");
    // Calc camera shots here if there are no images in turnaround
    if (_gridTriggerCamera) {
        for (int i=0; i<resultLines.size(); i++) {
            cameraShots += (int)floor(lineLenth(resultLines[i]) / _gridTriggerCameraDist);
            // Take into account immediate camera trigger at waypoint entry
            cameraShots++;
        }
    }
printf(" Turn into a path \n");
    // Turn into a path
    for (int i=0; i<resultLines.size(); i++) {
        LineXY           transectLine;
        vector<StwayXY>  transectPoints;
        const LineXY&    line = resultLines[i];
printf(" Polygon entry point 1\n");
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
printf(" Polygon entry point 1\n");
        if (_turnaroundDist > 0 ) {
            // transectPoints.append(transectLine.pointAt(-turnaroundPosition));
            transectPoints.push_back(pointAt(transectLine, -turnaroundPosition));
        }

        // Polygon entry point
        transectPoints.push_back(transectLine.start);
printf(" Polygon entry point 2\n");
        // For hover and capture we need points for each camera location
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
printf(" Polygon exit point \n");
        // Polygon exit point
        transectPoints.push_back(transectLine.stop);

        if (_turnaroundDist > 0 ) {
            transectPoints.push_back(pointAt(transectLine,1 + turnaroundPosition));
        }

        transectSegments.push_back(transectPoints);
    }

    return cameraShots;
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
            if(!(intersectPoint.x == -0.000000 && intersectPoint.y == 0.000000)){
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
    double a1 = (line1.start.y - line1.stop.y) / (line1.start.x - line1.stop.x);
    double b1 = line1.start.y - a1 * (line1.start.x);

    double a2 = (line2.start.y - line2.stop.y) / (line2.start.x - line2.stop.x);
    double b2 = line2.start.y - a1 * (line2.start.x);

    CrossP.x = (b1 - b2) / (a2 - a1);
    CrossP.y = a1 * CrossP.x + b1;
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


