#include <math.h>
#include <string.h> 
#define MAX_VILLAGE_NAME_LENGTH 15
#define DEGREES_TO_METERS_LATITUDE 78710 
#define DEGREES_TO_METERS_LONGITUDE 78605 

struct Localisation {
    float longitude;
    float latitude;
    char villageName[MAX_VILLAGE_NAME_LENGTH];
};


struct Journey {
    float duration;
    float distance;
    char direction[3];
};

void calc_journey(Journey *road, Localisation gps, Localisation destination){
  
  float ref_latitude = destination.latitude - gps.latitude;
  float ref_longitude = destination.longitude - gps.longitude;
  road->distance = hypot(ref_latitude*DEGREES_TO_METERS_LATITUDE, ref_longitude*DEGREES_TO_METERS_LATITUDE)/1000;
  road->duration = road->distance/5.0;


  if (ref_latitude == 0) {
    // Handle cases when ref_latitude is 0
    if (ref_longitude > 0) road->direction[0] = 'E';
    else if (ref_longitude < 0) road->direction[0] = 'W';
  } 
  else if (ref_longitude == 0) {
    // Handle cases when ref_latitude is 0
    if (ref_latitude > 0) road->direction[0] = 'N';
    else if (ref_latitude < 0) road->direction[0] = 'S';
  } 
  else if (ref_latitude > 0){
    (ref_longitude > 0) ? strcpy(road->direction, "NE") : strcpy(road->direction, "NW");
  }
  
  else{
    (ref_longitude > 0) ? strcpy(road->direction, "SE") : strcpy(road->direction, "SW");
  }
}


//funkcja zwraca dystans czas i kierunek
int main() {
  // Example values for gps and destination objects
  Localisation gps = {40.7128, -74.0060, "New York City"};
  Localisation destination = {34.0522, -118.2437, "Los Angeles"};
  Journey road;     //travel, route
  calc_journey(&road, gps, destination);
;
// Serial.print(); // Print voltage with 2 decimal places    
  delay(1000); // Delay for 1 second
  return 0;
}