#include <cstdio>
typedef struct waypoint{
  double lat;
  double lon;
}STWAY;
void GenRegWaypoint(STWAY * poly, int count) 
{  
    FILE *fp;
    int i;
    if( (fp = fopen("dataman.txt","w+")) == NULL){
        printf("111111111111111");
        if((fp = fopen("dataman.txt","w")) == NULL ){
            return ; 
        } 
    }
    for(i = 0; i < count; i ++){
       fprintf(fp,"%lf %lf\n",poly[i].lat,poly[i].lon); 
    }
    fclose(fp);
}  

