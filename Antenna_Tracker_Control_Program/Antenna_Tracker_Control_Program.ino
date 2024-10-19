/*
* Author: Pablo Islas
* Date: July 16th, 2024
* Purpose: To allow the antenna tracker to autonomously follow any UAV
*/

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <math.h>

// Define pins for RX and TX on GPS module
#define RX_PIN 3
#define TX_PIN 2

// Define stepper motor connections and settings
#define DIR_PIN_H 6    // Direction pin for horizontal stepper
#define STEP_PIN_H 7   // Step pin for horizontal stepper
#define DIR_PIN_V 4    // Direction pin for vertical stepper
#define STEP_PIN_V 5   // Step pin for vertical stepper
#define STEPS_PER_REV 400.0 // Steps per revolution (adjust according stepper settings
#define DRIVE_TRAIN_FACTOR 3.0 // Gear drive ratio

// Define value used to average current gps coords
#define AVG_COORD 1000

// Define actual coordinates read from iphone
// NOTE: This is a temporary fix until the gps module is made accurate (might best to make permanent and just change before use)
#define ACTUAL_LAT 49.2645884 // Temporary
#define ACTUAL_LON -123.2465638 // Temporary
#define ACTUAL_ASL 115
#define EARTH_RADIUS 6371 // Earths radius in km

TinyGPSPlus gps;
SoftwareSerial ss(RX_PIN, TX_PIN);

// Define stepper motor objects
AccelStepper Hstepper(AccelStepper::DRIVER, STEP_PIN_H, DIR_PIN_H);
AccelStepper Vstepper(AccelStepper::DRIVER, STEP_PIN_V, DIR_PIN_V);

// Define global varibales for storing current GPS module coordinates of Tracker
float trackerLat = 0; // Tracker latitude
float trackerLon = 0; // Tracker longitude

// Define global variables for storing target GPS coordinates
float targetLat;
float targetLon;
float targetAlt;

// Define global variables for storing bearing and elevation angles to reach target
float bearing;
float elevation;


/*
* Function Name: toRadians
* Purpose: Convert a value in degrees to radians
* Inputs: float degrees
* Output: value in radians
*/
float toRadians(float degrees) {
  return degrees * (PI / 180.0);
}

/*
* Function Name: toDegrees
* Purpose: Convert a value in radians to degrees
* Inputs: float radians
* Output: value in degrees
*/
float toDegrees(float radians) {
  return radians * (180.0 / PI);
}

/*
* Function Name: calculate_Bearing_and_Elevation
* Purpose: Compute the bearing and elevation angles between the tracker's position and the UAV's location
* Inputs (Global Variables): Latitude and longitude coordinates of the tracker (float lat1, float lon1) and UAV (float lat2, float lon2),
*                            UAV's altitude (float alt),
*                            Two pointers to return the bearing (float* bearing) and elevation (float* elevation) results
* Output (Global Variables): Bearing and elevation angles (returned through pointers)
*/
void calculate_Bearing_and_Elevation(){
  // Convert latitude and longitude from degrees to radians
  float lat1Rad = toRadians(trackerLat);
  float lon1Rad = toRadians(trackerLon);
  float lat2Rad = toRadians(targetLat);
  float lon2Rad = toRadians(targetLon);

  // Calculate the difference in latitudes and longitudes
  float dLat = lat2Rad - lat1Rad;
  float dLon = lon2Rad - lon1Rad;

  // Perform bearing calculation using bearing formulas on https://www.movable-type.co.uk/scripts/latlong.html
  float y = sin(dLon) * cos(lat2Rad);
  float x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);
  float bearingRad = atan2(y, x);
  // Convert bearing from radians to degrees and normalize range from 0 to 360 degrees
  float bearingDeg = round(fmod((toDegrees(bearingRad) + 360.0), 360.0));

  // Perform elevation calculation using distance formulas on https://www.movable-type.co.uk/scripts/latlong.html
  float a = (sin(dLat / 2.0) * sin(dLat / 2.0)) + (cos(lat1Rad) * cos(lat2Rad) * sin(dLon / 2.0) * sin(dLon / 2.0));
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  float distance = EARTH_RADIUS * c; // Distance results in km
  float elevationAngle = round(toDegrees(atan2(targetAlt, distance * 1000)));

  // Return the values using pointers
  bearing = bearingDeg;
  elevation = elevationAngle;
}

/*
* Function Name: moveStepper
* Purpose: Set the target position for the corresponding stepper motor
* Inputs: Elevation or bearing value (float angle),
*         Stepper motor to be positioned (AccelStepper &stepper),
*         Stepper motor identifier (const char* stepperName)
* Output: void
*/
void moveStepper(float angle, AccelStepper &stepper, const char* stepperName){
  // Constrain elevation angle to a range of 0 to 90 degrees
  if (stepperName == "Vstepper"){
    angle = constrain(angle, 0, 90);
  }

  // Define variable to store and get desired stepper position
  float desiredPosition = round((angle / 360.0) * STEPS_PER_REV * DRIVE_TRAIN_FACTOR);

  // Calculate the current position in steps
  long currentPosition = stepper.currentPosition();

  // Calculate shortance distance travel path from current to desired position
  long delta = desiredPosition - currentPosition;
  if (delta > STEPS_PER_REV * DRIVE_TRAIN_FACTOR/ 2) {
    delta -= STEPS_PER_REV * DRIVE_TRAIN_FACTOR;
  } else if (delta < -STEPS_PER_REV * DRIVE_TRAIN_FACTOR / 2) {
    delta += STEPS_PER_REV * DRIVE_TRAIN_FACTOR;
  }

  // Set new target position for the stepper motor
  stepper.move(delta);
}

/*
* Function Name: calibrate_tracker
* Purpose: Get current coordinates of tracker and move to initial target postion
* Inputs: Void
* Output: Void
*/
void calibrate_tracker(){
  // Average multiple data points to get precise coordinates of tracker
  for (int i = 0; i < AVG_COORD; i++){
    trackerLat += gps.location.lat();
    trackerLon += gps.location.lng();
  }
  trackerLat = trackerLat / AVG_COORD; 
  trackerLon = trackerLon / AVG_COORD;

  // Calculate and account for error in gps reading (temporary fix until gps module's accuracy is fixed)
  float errorLat = ACTUAL_LAT - trackerLat;
  float errorLon = ACTUAL_LON - trackerLon;
  trackerLat = trackerLat + errorLat;
  trackerLon = trackerLon + errorLon;
  Serial.println("DEBUG: Tracker's Position: " + String(trackerLat, 7) + "(lat), " + String(trackerLon, 7) + "(lon)");

  // Calibrate initial bearing and elevation angles (before takeoff)
  targetLat = 49.3409915; // Temporary value, will be replaced with data collected from drone
  targetLon = -123.1264555; // Temporary value, will be replaced with data collected from drone
  float target_ASL_elevation = 114; // Temporary value, will be replaced with data collected from drone
  float initial_displacement = ACTUAL_ASL - target_ASL_elevation; // Initial vertical distance between tracker and drone
  targetAlt = initial_displacement;
  // Account for scenario where tracker is below drone before computing elevation angle
  if (targetAlt < 0){
    targetAlt = -targetAlt;
  }

  calculate_Bearing_and_Elevation();

  // Two scenarios possible if tracker is above drone
  if (initial_displacement > 0) {
    // Scenario 1: drone is out of trackers vertical range (less than -13 degrees)
    if (elevation > 13) {
      Serial.println("ERROR: DRONE OUT OF RANGE. LOWER TRACKER OR RAISE DRONE AND RESTART PROGAM");
      while(true){
        ;
      }
    }
    // Scenario 2: Drone is between 0 and -13 degrees vertically from trackers horizontal position
    else {
      elevation = 13 - elevation;
    }
  }
  // One scenario possible if tracker is below drone
  // Scenario 3: Tracker below drone
  else {
    elevation = elevation + 13;
  }

  // Set target position
  moveStepper(bearing, Hstepper, "Hstepper");
  moveStepper(elevation, Vstepper, "Vstepper");

  // Do not move to target position unless the user is ready
  Serial.println("Ready to calibrate and move to starting position: " + String(targetLat, 7) + "(lat), " + String(targetLon, 7) + "(lon)?");
  Serial.println("NOTE: ENSURE TRACKER IS POINTING NORTH BEFORE PROCEEDING");
  Serial.println("y if yes: (USER/GCOM RESPONSE)");
  while (true) {
    if (Serial.available() > 0) {
      char userInput = Serial.read();
      if (userInput == 'y' || userInput == 'Y') {
      break; // Exit the loop after calibration
      }
    }
  }

  // Move to target position
  Serial.println("Moving...");
  Hstepper.runToPosition();
  Vstepper.runToPosition();
  Serial.println("Tracker calibrated.");
}





void setup() {
  // put your setup code here, to run once:
  // Initialize serial communication
  Serial.begin(9600); // Communication with computer
  ss.begin(110500); // Communication with gps module

  /* NO GPS SIGNAL INDOORS
  // Wait for a gps signal from tracker's gps module before proceeding
  Serial.println("Waiting for GPS signal...");
  while (!gps.location.isValid()) {
    while (ss.available() > 0) {
      gps.encode(ss.read());
    }
  }
  Serial.println("GPS signal acquired.");
  */

  // Set up the motor properties
  Hstepper.setMaxSpeed(75);       // Adjust maximum speed as needed
  Hstepper.setAcceleration(2000);    // Adjust acceleration as needed
  Vstepper.setMaxSpeed(75);       // Adjust maximum speed as needed
  Vstepper.setAcceleration(2000);    // Adjust acceleration as needed

  // Wait for serial connection to be established
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  Serial.println("Serial connection established.");

  calibrate_tracker();

  Serial.println("Please enter the coordinates in the format: latitude,longitude (USER/GCOM RESPONSE)");
  String userInput = Serial.readStringUntil('\n');
  
}


// 49.2679805, -123.2448525 (North)
// 49.2646732, -123.2415124 (East)
// 49.2598840, -123.2464730 (South)
// 49.2639150, -123.2552316 (West)


void loop() {
  // put your main code here, to run repeatedly:
  // Retreive data from serial monitor input
  while (Serial.available() > 0) {
    Serial.println("Please enter the coordinates in the format: latitude,longitude (USER/GCOM RESPONSE)");
    String input = Serial.readStringUntil('\n');
    input.trim();

    int commaIndex = input.indexOf(',');
    if (commaIndex == -1) {
      Serial.println("Invalid input. Please enter the coordinates in the format: latitude,longitude (ARDUINO RESPONSE)");
      continue;
    }

    // Get target coordinate values
    targetLat = input.substring(0, commaIndex).toFloat();
    targetLon = input.substring(commaIndex + 1).toFloat();
    targetAlt = 100;

    // Calculate bearing and elevation
    calculate_Bearing_and_Elevation();

    moveStepper(bearing, Hstepper, "Hstepper");
    moveStepper(elevation, Vstepper, "Vstepper");
  }

  // Run the stepper motors to the desired position
  Hstepper.run();
  Vstepper.run();
}





