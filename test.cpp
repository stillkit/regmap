#include <cstdio>  
#include <vector>

typedef unsigned char uint8_t;
typedef struct wayPointGPS{
  double lat;
  double lon;
}StawayGPS;
extern std::vector<std::vector<struct wayPointGPS> > GenRegWaypoint(std::vector<struct wayPointGPS> gridPlygon, double gridSpacing, bool hoverAndCaptureEnabled, double hoverAndCaptureDelaySeconds, double gridAngle, 
                    bool gridTriggerCamera, double gridTriggerCameraDist, double _turnaroundDist, uint8_t gridMode, bool gridRefly);
int main()   
{  
    StawayGPS ways[3];
    std::vector<StawayGPS> way(ways,ways+3);
    // int i;
    // for(i = 0; i < 4; i ++){

      // way[0].lat = 40.0351124;
      // way[0].lon = 116.3420277;

      // way[1].lat = 40.0351124;
      // way[1].lon = 116.3470277;

      // way[2].lat = 40.0351124;
      // way[2].lon = 116.3480277;

      // way[2].lat = 40.0371124;
      // way[2].lon = 116.3450277;

      // way[3].lat = 40.0391124;
      // way[3].lon = 116.3470277;

      // way[4].lat = 40.0381124;
      // way[4].lon = 116.3460277;

      // way[4].lat = 40.0391124;
      // way[4].lon = 116.3420277;

      // way[5].lat = 40.0371124;
      // way[5].lon = 116.3320277;

      // way[6].lat = 40.0371124;
      // way[6].lon = 116.3370277;

      // way[7].lat = 40.0361124;
      // way[7].lon = 116.3450277;

      // way[0].lat = 40.0351124;
      // way[0].lon = 116.3420277;

      // way[1].lat = 40.0351124;
      // way[1].lon = 116.3470277;

      // way[2].lat = 40.0351124;
      // way[2].lon = 116.3480277;

      // // way[2].lat = 40.0371124;
      // // way[2].lon = 116.3450277;

      // way[3].lat = 40.0391124;
      // way[3].lon = 116.3470277;

      // // way[4].lat = 40.0381124;
      // // way[4].lon = 116.3460277;

      // way[4].lat = 40.0391124;
      // way[4].lon = 116.3420277;

      // way[5].lat = 40.0371124;
      // way[5].lon = 116.3320277;

      // way[6].lat = 40.0371124;
      // way[6].lon = 116.3370277;

      // way[7].lat = 40.0361124;
      // way[7].lon = 116.3450277;

    // }


    // way[0].lat = 40.0351124;
    // way[0].lon = 116.3520277;

    // way[1].lat = 40.0381124;
    // way[1].lon = 116.3620277;

    // way[2].lat = 40.0361124;
    // way[2].lon = 116.3720277;

    // way[3].lat = 40.0371124;
    // way[3].lon = 116.3820277;

    // way[4].lat = 40.0366124;
    // way[4].lon = 116.3920277;

    // way[5].lat = 40.0391124;
    // way[5].lon = 116.3920277;

    // way[6].lat = 40.0391124;
    // way[6].lon = 116.3520277;

    // way[1].lat =  40.24388969343385;
    // way[1].lon = 116.24197600734988;

    // way[0].lat =  40.243938246843008;
    // way[0].lon =  116.31313479381264;

    // way[2].lat = 40.151242760635853;
    // way[2].lon = 116.24201261649188;

    // way[3].lat = 40.151292130713294;
    // way[3].lon = 116.31317127969599;


// way[0].lat =  40.199533023729899;
//       way[0].lon =  116.28217000765871;

//       way[1].lat =  40.19953586581677;
//       way[1].lon =  116.28260688650128;

//       way[2].lat = 40.199384233986308;
//       way[2].lon =  116.28260705458629;

//       way[3].lat =40.199381391844703;
//       way[3].lon =  116.2821701757606;


      // way[0].lat =    40.198328809826577;
      // way[0].lon =    116.28175165775558;

      // way[1].lat =  40.198333725478093;
      // way[1].lon =    116.28253222285689;

      // way[2].lat = 40.197041899942228;
      // way[2].lon =    116.28253365362495;

      // way[3].lat =40.19703698343961;
      // way[3].lon =    116.28175308874729;


      // way[0].lat =    40.200703779681049;
      // way[0].lon =      116.28065962724504;

      // way[1].lat =    40.200722348947906;
      // way[1].lon =        116.28361351355854;

      // way[2].lat =  40.197820353689536;
      // way[2].lon =      116.28361672795837;

      // way[3].lat =  40.197801777295759;
      // way[3].lon =    116.28066284382282;

    // way[0].lat =    40.200287606590564;
    // way[0].lon =    116.28049501139863;

    // way[1].lat =    40.200307439266346;
    // way[1].lon =    116.28367291843655;

    // way[2].lat =   40.198872752655376;
    // way[2].lon =   116.28367450772042;

    // way[3].lat =   40.198852916195541;
    // way[3].lon =    116.28049660187398;

    // way[0].lat =      40.207661187383522;
    // way[0].lon =      116.27612677859391;

    // way[1].lat =    40.207845249677163;
    // way[1].lon =      116.2964037183301;

    // way[2].lat =   40.185201496950313;
    // way[2].lon =   116.29642738238257;

    // way[3].lat =   40.185017122307741;
    // way[3].lon =        116.27615048220035;

    // way[0].lat =      40.21104094518541;
    // way[0].lon =      116.27121126011399;

    // way[1].lat =      40.211168068641278;
    // way[1].lon =       116.29293667777743;

    // way[2].lat =    40.185424704405868;
    // way[2].lon =    116.29296331572712;

    // way[3].lat =    40.185297168502871;
    // way[3].lon =          116.27123800982724;

    // way[0].lat =      40.2110409;
    // way[0].lon =      116.2712112;

    // way[1].lat =      40.2111680;
    // way[1].lon =       116.2929366;

    // way[2].lat =    40.1854247;
    // way[2].lon =    116.2929633;

    // way[3].lat =    40.1852971;
    // way[3].lon =    116.2712380;


    // way[0].lat =      40.208329327761291;
    // way[0].lon =      116.27133986955815;

    // way[1].lat =      40.208574344407197;
    // way[1].lon =       116.30063758199735;

    // way[2].lat =     40.178827367231541;
    // way[2].lon =    116.3006667764041;

    // way[3].lat =     40.178581802295184;
    // way[3].lon =          116.27136906819931;   

    way[0].lat =       40.207217557036863;
    way[0].lon =      116.29515469818109;

    way[1].lat =       40.206563261696736;
    way[1].lon =        116.29466110619987;

    way[2].lat =     40.206563261696736;
    way[2].lon =    116.29564699944409;

    // way[3].lat =     40.178581802295184;
    // way[3].lon =          116.27136906819931;   

    int i;
    for(i = 0; i < 3; i ++){
        printf("%0.8f %0.8f \n",way[i].lat,way[i].lon);
    }
    
    std::vector<std::vector<StawayGPS> > result = GenRegWaypoint(way,6,true,1,110,true,10,0,0,false);
    // for(i = 0; i < result.size(); i ++)
    //     for(int j = 0; j < result[i].size(); j ++)
    //         printf("%0.7f %0.7f\n",result[i][j].lat,result[i][j].lon);

    return 0;  
}  
