/*
   Caution: the code calculates servo angles, likely from -90 to +90, where 0 is at the horizontal.
    -->>>   how does the bot configure the servos??
*/

/*
   see https://www.robotshop.com/en/arduino-sensor-shield-shd012.html?gclid=Cj0KCQjwtOLVBRCZARIsADPLtJ08Ib3VOUE-uZ5oDILBzgPGTzQgLwduga0wNy5vcjuYhYkuAUX8p0AaAsy4EALw_wcB
   also http://yourduino.com/sunshop//index.php?l=product_detail&p=195
   for more info on the servo shield
*/

#include <Servo.h>

#define DEBUG

#include "DebugUtils.h"

// delta arm dimensions and leg locations (in degrees)
double thighLength = 24;    // length of the thigh segment in cm
double shinLength = 51.1;     // length of the shin segment in cm
double effectorLength = 7.5;  // distance from center of effector to pivot point of shin attachment, in cm
double baseLength = 9;      // distnace from center of base to pivot point of thigh attachment, in cm
double gripperLength = 17;   // distance from center of effector to the "gripping point"
// angles for servos 1, 2, and 3, respectively
double phi[] = {270, 150, 30}; // rotational location of each leg, in degrees.

// servo pins on arduino
Servo myServos[3];
// arduino mega digital pins for servos 1, 2, and 3, respectively
int servoPins[] = {38, 42, 34};

// this array will hold the required angles for the three servos that control the arm
int servoAngle[] = {0, 0, 0};
int servoBias[] = {0, 0, 0};




void setup() {
#ifdef DEBUG
  // start serial port at 9600 bps:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif
  // convert phi to radians, simplifying the math later on
  for (int i = 0; i < 3; i++) {
    phi[i] = convertToRadians(phi[i]);
  }
  // set up the servo motors
  for (int i = 0; i < 3; i++) {
    myServos[i].attach(servoPins[i]);
  }
}

void loop() {
  bool valid = true;  // is the target location bogus?

  // locations to move the arm to, one after another
  // these will hold arrays of desired locations of the effector/gripper/hand;
  // note that "xPrime" is the distance to the right
  //           "zPrime" is the distance to the front
  //           "yPrime" is the distance down below the base (should be negative, but it doesn't really matter)
  double xPrime[] = {0, 0, 0, 10};
  double yPrime[] = { -61.285, -61.285, -61.285, -61.285};
  double zPrime[] = {0, 0, 0, 0};

  // loop through all of the required gripper positions, pausing between
  for (int j = 0; j < 4; j++) {
    valid = calculateServoAngles(xPrime[j], yPrime[j], zPrime[j]);


    // set the servos to the appropriate angles
    // keep this loop separate from the others to ensure all servos are set is rapid order
    // (keep the legs from going whacko!)
    if (valid) {
      for (int i = 0; i < 3; i++) {
        myServos[i].write(servoAngle[i]);
        DEBUG_PRINT(String(servoAngle[0]) + " " + String(servoAngle[1]) + " " + String(servoAngle[2]));
      }
      delay(2000);  // wait 2 seconds before moving to another position
    }
  }
}

double convertToRadians(double angle) {
  return angle / 180.*PI;
}

double convertToDegrees(double angle) {
  return angle / PI * 180. ;
}

/*
  Function calculateServoAngles
  Purpose: calculate required angles for 3 servos to position the gripper at the specified location (xPrime,yPrime,ZPrime)
  Arguments:
	double xPrime - gripper location to the right of the arm centerline
	double yPrime - gripper location down below the base (should be negative, but it doesn't really matter)
	double zPrime - gripper location to the front of the arm centerline
  Returns:
	boolean - True if all servo angles are valid, False otherwise
*/
double x, y, z, c, tempData, a, alpha, beta;
bool calculateServoAngles(double xPrime, double yPrime, double zPrime) {
  // calculate angle of each of three servos for the current gripper position
  for (int i = 0; i < 3; i++) {
    x = xPrime * cos(phi[i]) - zPrime * sin(phi[i]);
    // deal with possible sign confusion with yPrime and adjust for gripper length
    y = -abs(yPrime) + gripperLength;
    z = xPrime * sin(phi[i]) + zPrime * cos(phi[i]);
    tempData = x + effectorLength - baseLength;
    c = sqrt(tempData * tempData + y * y);
    // check for valid position data (no square roots of negative numbers)
    tempData = shinLength * shinLength - z * z;
    if (tempData < 0.0)
      return false;
    a = sqrt(tempData);
    // check for valid calculation of the angle "alpha"
    tempData = (-(a * a) + thighLength * thighLength + c * c) / (2 * thighLength * c);
    if (abs(tempData) > 1.0)
      return false;
    alpha = acos( tempData );
    beta = atan2(y, x + effectorLength - baseLength);

    servoAngle[i] = 90 + servoBias[i] + round(convertToDegrees(alpha - abs(beta))); // servo angles are in degrees, and are integers
    if (servoAngle[i]<0 or servoAngle[i]>180)
      return false;
  }
  return true;
}

