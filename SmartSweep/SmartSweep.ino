/* SmartSweep
    Athor:        ROBERT HÜLSMANN
    Version:      1.2.1
    Modified:     20.10.2016
    Description:  Sweep programm for Servo with motion profile calculation first.
                  Open serial monitor to see more information.
                  Wait for calculation after plugging in.
*/





/* -------- DEFINE THE PROGRAMM PARAMETERS HERE -------- */
#define SERPIN  3     // <-- Set here servo pin
#define POSLEN  400   // <-- Set here length of position array
#define RESLTN  10    // <-- Set here resolution of controll in milliseconds
#define MAXROT  160   // <-- Set here maximal rotation in degree between 0° and 160°
#define MAXJRK  30    // <-- Set here the jerk of movement in kilodegree per cubicseconds (k°/s^3). MAXJRK is depending on RESLTN!
/* -------- DEFINE THE PROGRAMM PARAMETERS HERE -------- */





#include <Servo.h>
Servo myservo;                                                      // create servo object to control a servo
int potpin = 0;                                                     // analog pin used to connect the potentiometer

unsigned int positions[POSLEN];                                     // array for positions in centi degree
int counter = 0;                                                    // count time steps
int steps = 0;                                                      // maximum step number
bool upDown = false;                                                // true, if moving from 0° -> max. rotation and false for max. rotation -> 0°



void setup() {
  myservo.attach(SERPIN);                                           // attaches the servo on pin to the servo object
  Serial.begin(9600);                                               // Open serial connection. Take a look in

  steps = getPositions(RESLTN, MAXJRK, 90);                         // to drive to 0° smoothly fill position array and get max. steps in once while using getPosition function
  counter = steps*2;                                                // beacause the position array includes the position of the half of rotation, set counter to steps*2 for get count value of end position
  while(counter > 0){
    counter = goToPosition(false, counter, steps, 90, RESLTN);
  }

  steps = getPositions(RESLTN, MAXJRK, MAXROT);                     //fill position array and get max. steps in once while using getPosition function
}



void loop() {
  
  if(counter <= 0){
    upDown = true;                                                  // Set variable to increase counter, if position reaches 0°
  }else if(counter >= (steps-1) * 2){
    upDown = false;                                                 // Set variable to decrease counter, if position reaches 160°
  }
  
  counter = goToPosition(upDown, counter, steps, MAXROT, RESLTN);   // set Position with function
}



int getPositions(int resolution, int maxJerk, int maxRot) {
  int jerk = 0;
  float acc = 0;                                                    // Start acceration
  float vel = 0;                                                    // Start velocity
  float pos = 0;                                                    // Start position

  bool wasNegJerk = false;                                          // is true if previouse step has a jerk != 0
  
  for(unsigned int i = 0; i < POSLEN; i++){                         // for each item in position array
    if(pos == 0 ){                                                  // if position 0 start movement with maximal jerk
      jerk = maxJerk;
    }else if(pos >= (float)maxRot / 4.0 && wasNegJerk == false){    // if position greater than the first quarter of max. rotation and the there was no negative jerk before set negative jerk
      jerk = maxJerk * -1;
      wasNegJerk = true;
    }else{                                                          // in all other cases, set jerk to 0
      jerk = 0;
    }
    
    acc = (float)jerk*1000 * ((float)resolution / 1000.0) + acc;    // get current acceleration by integration
    vel = acc * ((float)resolution / 1000.0) + vel;                 // get current velocity by integration
    pos = vel * ((float)resolution / 1000.0) + pos;                 // get current position by integration
    
    positions[i] = (unsigned int)(pos*100);                         // write current position in array

    Serial.println(String( (float)i*(float)resolution / 1000.0) + "s:\tJ:" + String(jerk) + (jerk==0?"":"k") + "\tA:" + String(acc) + "   \tV:" + String(vel)  + "   \tP:" + String(pos) + "   \t-> " + String(positions[i]));
    
    if (pos >= (float)maxRot / 2.0) {                               // if code arrive half of max rotation, than exit function
      Serial.println("Time: " + String(steps * 2.0 / 100.0) + " seconds");
      return i;                                                     // return current step -> thats is hightest step
    }
  }

  // If programm jumps out of for loop, than step number is higher as array length -> programm can't work correctly
  Serial.println("You are too slow! Rise jerk or change length of positions array and try again");
  return POSLEN;
}



int goToPosition(bool upDown, int counter, int steps, int maxRotation, int delayTime){
  float curPos = 0;                                                 // current position of servo
  
  if(counter < steps - 1){                                          // If position in first half of maximal rotation
    curPos = (float)positions[counter] / 100;                       // Get current position from positions array via function
    Serial.println(String(counter) + ":\t " + curPos);
  }else{                                                            // If position in second half of maximal rotation
    curPos = (float)maxRotation - ((float)positions[2*steps - counter - 2] / 100); //Get current position by inverse position of first half of maximal rotation
    Serial.println(String(2*steps - counter - 2) + ":\t " + curPos);
  }

  myservo.write(curPos);                                            // Write position

  counter += (upDown?1:-1);                                         // Increase of decrease counter
  
  delay(delayTime);                                                 // Wait delayTime (= resolution of controll) */
  return counter;                                                   // return new current counter
}


