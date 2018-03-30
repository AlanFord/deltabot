
/*
 * listen.ino
 * 
 * Direct the delta-arm to move to coordinates specified over a serial connection from the Arduino Serial Monitor.
 * 
 * see https://www.robotshop.com/en/arduino-sensor-shield-shd012.html?gclid=Cj0KCQjwtOLVBRCZARIsADPLtJ08Ib3VOUE-uZ5oDILBzgPGTzQgLwduga0wNy5vcjuYhYkuAUX8p0AaAsy4EALw_wcB
 * also http://yourduino.com/sunshop//index.php?l=product_detail&p=195
 * for more info on the servo shield
*/

#include <Servo.h>

// uncomment the following line to enable debug print statements
// #define DEBUG

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
int servoPins[] = {38, 30, 34};

// this array will hold the required angles for the three servos that control the arm
int servoAngle[] = {0, 0, 0};
int servoBias[] = {-15, -12, -20};

int time1, time2;


void setup() {
  // start serial port at 9600 bps:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
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
  double xPrime, yPrime, zPrime;

  // if there's any serial available, read it:
  while (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    int xPrime = Serial.parseFloat();
    // do it again:
    int yPrime = Serial.parseFloat();
    // do it again:
    int zPrime = Serial.parseFloat();

    // look for the newline. That's the end of your
    // sentence:
    if (Serial.read() == '\n') {
      time1 = micros();
      valid = calculateServoAngles(xPrime, yPrime, zPrime);
      time2 = micros();
      // set the servos to the appropriate angles
      // keep this loop separate from the others to ensure all servos are set is rapid order
      // (keep the legs from going whacko!)
      if (valid) {
        Serial.print("Calc time is: ");
        Serial.print(time2-time1);
        Serial.println(" microseconds");
        Serial.print("For coordinates ");
        Serial.print(String(xPrime) + " " + String(yPrime) + " " + String(zPrime));
        Serial.print("-> Servo angles are: ");
        Serial.println(String(servoAngle[0]) + " " + String(servoAngle[1]) + " " + String(servoAngle[2]));
        for (int i = 0; i < 3; i++) {
          myServos[i].write(servoAngle[i]);
        }
      }
      else {
        Serial.print("Coordinates ");
        Serial.print(String(xPrime) + " " + String(yPrime) + " " + String(zPrime));
        Serial.println(" are invalid");
      }
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

