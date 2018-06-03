// ---------------------------------------------------------------------------
// Example NewPing library sketch that pings 3 sensors 20 times a second.
// ---------------------------------------------------------------------------

#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 250 // Maximum distance (in cm) to ping.

float dist = 0;

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(3, 2, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(11, 4, MAX_DISTANCE), 
  NewPing(13, 12, MAX_DISTANCE)
};

void setup() {
  Serial.begin(250000); // Open serial monitor at 115200 baud to see ping results.
}

void loop() { 
  Serial.println();
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
    delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    Serial.print(i);
    Serial.print("= ");
    dist = sonar[i].ping_cm();
    while(dist == 0){
      delay(40);
      dist = sonar[i].ping_cm();
    }
    Serial.print(dist);
    Serial.print(" cm || ");
  }
  
}
