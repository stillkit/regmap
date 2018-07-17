#ifndef _REG_MAP_H
#define _REG_MAP_H

#include <iostream>   
#include <vector>    
#include <algorithm>   
#include <cmath>
#include <cstdio>
#include <limits>
#include <cfloat>
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
#define MIN_POINT_SEG_DIST         10
#define MIN_POINT_POINT_DIST         10

#define DEBUG

static const float epsilon = std::numeric_limits<double>::epsilon();

typedef struct wayPointGPS{
  double lat;
  double lon;
}StwayGPS;

typedef struct wayPointXy{
  double x;
  double y;
  bool operator==(const wayPointXy b) const  
  {  
      return this->x == b.x && this->y == b.y;  
  } 
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
double              _hoverAndCaptureDelaySeconds;
double 							_gridAngle;
bool 							  _gridTriggerCamera;
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

// std::vector<int>              _polygonConcavePointsSer;
StwayXY         _okDegree;
// std::vector<StwayXY>          _polygonConcavePoints;
// vector<StwayXY> polygonPoints;

int             _cameraShots;
int             _missionCommandCount;
double          _surveyDistance;
int             _gridEntryLocation = EntryLocationTopLeft;
double          _coveredArea;

/*
 *@author: kit
 *@date: 20180716
 *@description: main function
 *input parameter:
 *  1,std::vector<struct wayPointGPS> gridPlygon:             Mapping polygon vertices
 *  2,double gridSpacing:                                     Mapping spacing
 *  3,bool hoverAndCaptureEnabled(TODO):                      Whether to stop taking photos at the waypoint
 *  4,double hoverAndCaptureDelaySeconds(TODO):               Delay time at the camera point
 *  5,double gridAngle:                                       Angle of the survey line
 *  6,bool gridTriggerCamera(TODO):                           Trigger photo point sign
 *  7,double gridTriggerCameraDist(TODO):                     Distance between photo points
 *  8,double _turnaroundDist:                                 The distance from the turning point to the boundary line
 *  9,uint8_t gridMode:                                       Mapping mode,0:normal,1:Recommended scanning angle,2:Convert to convex polygon scan,6:Equal interval scanning,7:(TODO),8:Convert to graph polygons for equally spaced scans
 *  10,bool gridRefly(TOD):                                   Whether to mark the second scan        
 *return:
 *  vector<vector<StwayGPS> > :Surveyed waypoint collection
 */
vector<vector<StwayGPS> >   GenRegWaypoint(std::vector<struct wayPointGPS> gridPlygon, double gridSpacing, bool hoverAndCaptureEnabled, double hoverAndCaptureDelaySeconds, double gridAngle, 
                    bool gridTriggerCamera, double gridTriggerCameraDist, double _turnaroundDist, uint8_t gridMode, bool gridRefly);
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
// StwayXY getCross(LineXY line1, LineXY line2);
bool getCross(LineXY line1, LineXY line2 ,StwayXY &CrossP);
void intersectLinesWithPolygon(const vector<LineXY>& lineList, const vector<StwayXY>& polygon, vector<LineXY>& resultLines);
StwayXY pointAt(LineXY line, double proportion);
LineXY getTranslate(LineXY line, StwayXY center);
double lineLenth(LineXY line);
void adjustLineDirection(const vector<LineXY>& lineList, vector<LineXY>& resultLines);
// double ccw(StwayXY pt1, StwayXY pt2, StwayXY pt3);
// double dp(StwayXY pt1, StwayXY pt2);
// void swapPoints(vector<StwayXY>& points, int index1, int index2);
// std::vector<StwayXY> convexPolygon(vector<StwayXY> polygon);
std::vector<StwayXY> ConvexHull(vector<StwayXY>  polygon);
void qsortpoint(std::vector<StwayXY> &s,int start,int end);
void sortstartedge(std::vector<StwayXY> &s);
void sortpoint(std::vector<StwayXY> &s);
void swap(StwayXY& a,StwayXY& b);
bool isLeftorNearer(StwayXY base,StwayXY i,StwayXY j);
bool betweenCmp(StwayXY a,StwayXY b,StwayXY c);
int doublecmp(double d);
double CrossMul(double x1,double y1,double x2,double y2);
//向量（x1,y1）,(x2,y2)的点积
double DotMul(double x1,double y1,double x2,double y2);
//跨立判断
//判断点c是在向量ab的逆时针方向还是顺时针方向，大于零逆时针，等于0则共线
double CrossMul(StwayXY a,StwayXY b,StwayXY c);
//计算向量ab和ac点积
double DotMul(StwayXY a,StwayXY b,StwayXY c);
bool getOkDegree(vector<StwayXY> polygonPoints, vector<int> polygonConcavePointsSer);
double getSlope(StwayXY p1, StwayXY p2);
double transformQuadrant(double angle);
void transformQuadrantPlus(StwayXY &degree_res);
std::vector<StwayXY> zoomPolygon(std::vector<StwayXY> polygonPoints, double gridSpacing);
wayPointXy addWayPointXy(wayPointXy a, wayPointXy b);
wayPointXy minusWayPointXy(wayPointXy a, wayPointXy b);
double multyWayPointXy(wayPointXy a, wayPointXy b);
wayPointXy multyWayPointXy(wayPointXy a, double value);
double xlJi(wayPointXy a, wayPointXy b);
int zoomPolygonGrid(const vector<StwayXY>& polygonPoints,  vector<vector<StwayXY> >& transectSegments, bool refly);
// std::vector<StwayXY> zoomPolygon(std::vector<StwayXY> polygonPoints, double gridSpacing);
std::vector<StwayXY> zoomPolygon(std::vector<StwayXY> polygonPoints, double gridSpacing, vector<int> relativePosition, vector<int> &relativePositionInner);
double pointToSegDist(wayPointXy s, wayPointXy x1, wayPointXy x2);
/*
 *@author: kit
 *@date: 20180716
 *@description: Get minimum point-to-edge distance
 *input parameter:
 *               Vertex of a polygon
 *return: 
 *               Minimum point-to-edge distance
 */
double getMinPoiintToSegDist(const vector<StwayXY>& polygonPoints);
bool isCanGenerateGrid(std::vector<StwayXY> polygonPoints);
// bool isCloseToPointAndSeg(const vector<StwayXY>& polygonPoints, int i);
bool isCloseToPointAndSeg(const vector<StwayXY>& polygonPoints, int i, StwayXY point);
double lineLenth(StwayXY startPoint, StwayXY endPoint);
bool isOkRelativePosition(StwayXY p1, StwayXY p2,int compId);
int getRelativePosition(StwayXY p1, StwayXY p2);
bool isPolygonCross(const vector<StwayXY>& polygonPointsStart, const vector<StwayXY>& polygonPointsEnd);
int getTriggercameraShots(const vector<StwayXY>& intersectLines,  vector<vector<StwayXY> >& transectSegments);
/*
 *@author: kit
 *@date: 20180716
 *@description: Get all the unit vector of a polygon
 *input parameter:
 *               Vertex of a polygon
 *return: 
 *               a line vector
 */
std::vector<StwayXY> unitizedVector(std::vector<StwayXY> polygonPoints);
/*
 *@author: kit
 *@date: 20180716
 *@description: Get the unit vector of two points
 *input parameter:
 *               tow points
 *return: 
 *               a line
 */
StwayXY unitizedVector(StwayXY p1, StwayXY p2);
bool getConcavePoint(std::vector<StwayXY> polygonPoints,vector<int> &polygonConcavePointsSer);
// bool isOkRelativePosition(StwayXY p1, StwayXY p2,StwayXY p3,int compIdP1P2,int compIdP2P3);

/*
 *@author: kit
 *@date: 20180716
 *@description: Get the area of a polygon
 *input parameter:
 *               Vertex of a polygon
 *return: 
 *               the area of a polygon
 */
double polygonArea(std::vector<StwayXY> polygonPoints);
#endif 
