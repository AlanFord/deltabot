# deltabot
Demonstration program for driving a servo-based delta arm.

The software is written for the Arduino to drive the delta-arm described in
the book "Arduino and Kinect Projects" by E. Melgar and C Diez (2012).
The software follows the modeling described in the reverse kinematics document
found in the kinematics folder, a modification of the modeling given in the book.

The dance program is a demonstration Arduino program that cycles the arm through a series of
preset positions, separated by a second pause between positions.
The listen program reads from the serial connecton a position (a set of three real numbers x, y, and z)
and them moves the arm to that position, if a valid position.  The Arduino then echos the time required to perform
the reverse kinematics calculations and the resulting servo angles for the three servo motors.

