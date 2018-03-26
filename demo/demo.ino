/*
   Caution: the code calculates servo angles, likely from -90 to +90, where 0 is at the horizontal.
    -->>>   how does the bot configure the servos??
*/

#include <Servo.h>

// delta arm dimensions and leg locations (in degrees)
double thighLength = 25;    // length of the thigh segment in cm
double shinLength = 25;     // length of the shin segment in cm
double effectorLength = 2;  // distance from center of effector to pivot point of shin attachment, in cm
double baseLength = 5;      // distnace from center of base to pivot point of thigh attachment, in cm
double phi[] = {0, 120, 240}; // rotational location of each leg, in degrees.

Servo myServos[3];
int servoPins[] = {8, 9, 10};




void setup() {
  // convert phi to radians, simplifying the math later on
  for (int i = 0; i < 3; i++) {
    phi[i] = phi[i] / 180. * PI;
  }
  // set up the servo motors
  for (int i = 0; i < 3; i++) {
    myServos[i].attach(servoPins[i]);
  }
}

void loop() {
  // these will hold arrays of desired locations of the effector/gripper/hand;
  // note that "xPrime" is the distance to the right
  //           "zPrime" is the distance to the front
  //           "yPrime" is the distance down below the base (should be negative, but it doesn't really matter)
  double xPrime[] = {0, 0, 0, 0};
  double yPrime[] = {0, 0, 0, 0};
  double zPrime[] = {0, 0, 0, 0};


  int servoAngle[] = {0, 0, 0};

  // loop through all of the required gripper positions, pausing between
  for (int j = 0; j < 4; j++) {
    // calculate angle of each of three servos for the current gripper position
    for (int i = 0; i < 3; i++) {
      double x = xPrime[j] * cos(phi[i]) - zPrime[j] * sin(phi[i]);
      double y = yPrime[j];
      double z = xPrime[j] * sin(phi[i]) + zPrime[j] * cos(phi[i]);
      double c = sqrt(pow(x + effectorLength - baseLength, 2) + y * y);
      double a = sqrt(shinLength * shinLength - z * z);
      double alpha = acos( (-(a * a) + thighLength * thighLength + c * c) / (2 * thighLength * c) );
      double beta = atan2(y, x + effectorLength - baseLength);
      servoAngle[i] = round((alpha - abs(beta)) / PI * 180); // servo angles are in degrees, and are integers
    }
    // sanity check of the servo angles
    bool bogus = false;
    for (int i = 0; i < 3; i++) {
      if (servoAngle[i]<0 or servoAngle[i]>180) {
        bogus = true;
      }
    }
    // set the servos to the appropriate angles
    // keep this loop separate from the others to ensure all servos are set is rapid order
    // (keep the legs from going whacko!)
    if (bogus == false) {
      for (int i = 0; i < 3; i++) {
        myServos[i].write(servoAngle[i]);
      }
    }
    delay(2000);
  }
}


