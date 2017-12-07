#include <cstdio>  
#include <vector>

typedef unsigned char uint8_t;
typedef struct wayPointGPS{
  double lat;
  double lon;
}StawayGPS;
extern std::vector<std::vector<struct wayPointGPS> > GenRegWaypoint(std::vector<struct wayPointGPS> gridPlygon, double gridSpacing, double gridAngle, 
                    bool gridTriggerCamera, double gridTriggerCameraDist, double _turnaroundDist, uint8_t gridMode, uint8_t gridRefly);
int main()   
{  
    StawayGPS ways[4];
    std::vector<StawayGPS> way(ways,ways+4);
    // int i;
    // for(i = 0; i < 4; i ++){

      way[0].lat = 40.0351124;
      way[0].lon = 116.3420277;

      way[1].lat = 40.0351124;
      way[1].lon = 116.3470277;

      way[2].lat = 40.0391124;
      way[2].lon = 116.3470277;

      way[3].lat = 40.0391124;
      way[3].lon = 116.3420277;

    // }
    int i;
    for(i = 0; i < 4; i ++){
        printf("%0.8f %0.8f \n",way[i].lat,way[i].lon);
    }
    
    std::vector<std::vector<StawayGPS> > result = GenRegWaypoint(way,10,30,true,10,10,0,false);
    for(i = 0; i < result.size(); i ++)
        for(int j = 0; j < result[i].size(); j ++)
            printf("%0.7f %0.7f\n",result[i][j].lat,result[i][j].lon);

    return 0;  
}  
