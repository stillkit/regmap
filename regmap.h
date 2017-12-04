#ifndef _REG_MAP_H
#define _REG_MAP_H

#include <iostream>   
#include <vector>    
#include <algorithm>   
#include <cmath>
#include <cstdio>
#include <limits>
// #include <stdbool.h>

using namespace std;   

#ifndef sincos
#define sincos(th,x,y) { (*(x))=sin(th); (*(y))=cos(th); }
#endif

typedef unsigned char uint8_t;
// These defines are private
#define M_DEG_TO_RAD (M_PI / 180.0)

#define M_RAD_TO_DEG (180.0 / M_PI)

#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			/* kg/m^3		*/
#define CONSTANTS_AIR_GAS_CONST				287.1f 			/* J/(kg * K)		*/
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			-273.15f		/* °C			*/
#define CONSTANTS_RADIUS_OF_EARTH			6371000			/* meters (m)		*/

static const float epsilon = std::numeric_limits<double>::epsilon();

typedef struct wayPointGPS{
  double lat;
  double lon;
}StwayGPS;

typedef struct wayPointXy{
  double x;
  double y;
}StwayXY;

typedef struct lineXy{
  StwayXY start;
  StwayXY stop;
}LineXY;

typedef struct rectXy{
  double x;
  double y;
  double width;
  double lenth;
}RectXY;

enum EntryLocation {
    EntryLocationTopLeft,
    EntryLocationTopRight,
    EntryLocationBottomLeft,
    EntryLocationBottomRight,
};

std::vector<struct wayPointGPS> _gridPlygon;
double 							_gridSpacing;
double 							_gridAngle;
bool 							 _gridTriggerCamera;
bool 							  _hoverAndCaptureEnabled;
double							_gridTriggerCameraDist;
double 							_turnaroundDist;
uint8_t 						_gridMode;
uint8_t 						_gridRefly;


// int                             _sequenceNumber;
// bool                            _dirty;
// std::vector<struct wayPointGPS> _mapPolygon;
std::vector<StwayGPS>		      _simpleGridPoints;      ///< Grid points for drawing simple grid visuals
vector<vector<StwayGPS> >    	_transectSegments;      ///< Internal transect segments including grid exit, turnaround and internal camera points
vector<vector<StwayGPS> >    	_reflyTransectSegments; ///< Refly segments
StwayGPS                  		_coordinate;
StwayGPS                  		_exitCoordinate;
double                        _additionalFlightDelaySeconds;

int             _cameraShots;
int             _missionCommandCount;
double          _surveyDistance;
int             _gridEntryLocation = EntryLocationTopLeft;

void GenRegWaypoint(std::vector<struct wayPointGPS> gridPlygon, double gridSpacing, double gridAngle, 
                    bool gridTriggerCamera, double gridTriggerCameraDist, double _turnaroundDist, uint8_t gridMode, uint8_t gridRefly);
void convertGeoToNed(StwayGPS coord, StwayGPS origin, double* x, double* y);
void convertNedToGeo(double x, double y, StwayGPS origin, StwayGPS *coord);
void generateGrid(void);
int gridGenerator(const vector<StwayXY>& polygonPoints,  vector<vector<StwayXY> >& transectSegments, bool refly);
void convertTransectToGeo(const vector<vector<StwayXY> >& transectSegmentsNED, const StwayGPS& tangentOrigin, vector<vector<StwayGPS> >& transectSegmentsGeo);
void appendGridPointsFromTransects(vector<vector<StwayGPS> >& rgTransectSegments);
double get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);
double clampGridAngle90(double gridAngle);
RectXY boundingRect(vector<StwayXY> polygonPoints);
StwayXY center(RectXY smallBoundRect);
StwayXY rotatePoint(const StwayXY& point, const StwayXY& origin, double angle);
bool gridAngleIsNorthSouthTransects();
LineXY getLine(double x1, double y1, double x2, double y2, const StwayXY& origin, double angle);
StwayXY getCross(LineXY line1, LineXY line2);
void intersectLinesWithPolygon(const vector<LineXY>& lineList, const vector<StwayXY>& polygon, vector<LineXY>& resultLines);
StwayXY pointAt(LineXY line, double proportion);
LineXY getTranslate(LineXY line, StwayXY center);
double lineLenth(LineXY line);

#endif 