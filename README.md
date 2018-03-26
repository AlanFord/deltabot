# deltabot
Demonstration program for driving a servo-based delta arm

The kinematics folder contains a derivation of the formulas used here.
The demo folder contains the demonstration Arduino program.  The
program will run the delta arm through four different positions, with a
2 second pause between positions.  A fair amount of trig is required, so a
loop to set all three servo motor angles is used to keep the all of the
servo manipulations in a short time interval. 
