#include <cstdio>  
#include <vector>

typedef unsigned char uint8_t;
typedef struct wayPointGPS{
  double lat;
  double lon;
}StawayGPS;
extern std::vector<std::vector<struct wayPointGPS> > GenRegWaypoint(std::vector<struct wayPointGPS> gridPlygon, double gridSpacing, bool hoverAndCaptureEnabled, double hoverAndCaptureDelaySeconds, double gridAngle, 
                    bool gridTriggerCamera, double gridTriggerCameraDist, double _turnaroundDist, uint8_t gridMode, uint8_t gridRefly);
int main()   
{  
    StawayGPS ways[7];
    std::vector<StawayGPS> way(ways,ways+7);
    // int i;
    // for(i = 0; i < 4; i ++){

      way[0].lat = 40.0351124;
      way[0].lon = 116.3420277;

      way[1].lat = 40.0351124;
      way[1].lon = 116.3470277;

      // way[2].lat = 40.0351124;
      // way[2].lon = 116.3480277;

      way[2].lat = 40.0371124;
      way[2].lon = 116.3450277;

      way[3].lat = 40.0391124;
      way[3].lon = 116.3470277;

      // way[4].lat = 40.0381124;
      // way[4].lon = 116.3460277;

      way[4].lat = 40.0391124;
      way[4].lon = 116.3420277;

      way[5].lat = 40.0371124;
      way[5].lon = 116.3320277;

      way[6].lat = 40.0371124;
      way[6].lon = 116.3370277;

      // way[7].lat = 40.0361124;
      // way[7].lon = 116.3450277;

    // }
    int i;
    for(i = 0; i < 7; i ++){
        printf("%0.8f %0.8f \n",way[i].lat,way[i].lon);
    }
    
    std::vector<std::vector<StawayGPS> > result = GenRegWaypoint(way,20,true,1,110,true,10,10,1,false);
    // for(i = 0; i < result.size(); i ++)
    //     for(int j = 0; j < result[i].size(); j ++)
    //         printf("%0.7f %0.7f\n",result[i][j].lat,result[i][j].lon);

    return 0;  
}  
