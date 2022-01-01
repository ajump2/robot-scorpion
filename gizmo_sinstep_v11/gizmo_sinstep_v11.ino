#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "gizmo.h"

int trigPin = TRIGPIN;
int echoPin = ECHOPIN;
int servoMin = SERVOMIN;
int servoMax = SERVOMAX;
int servoFreq = SERVO_FREQ;
long distance;
long duration;



Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
uint8_t servonum = SERVO_NUM;
int startingAngle = STARTING_ANGLE;
const int numberServos = NUMBER_SERVOS; 
const int numberLegs = NUMBER_LEGS;
const int z = Z;
const int y = Y;

const int buttonG = BUTTON_G; 
const int buttonBL = BUTTON_BL; 
const int buttonBR = BUTTON_BR; 
const int buttonR = BUTTON_R;

const int stinger = STINGER; 
const int pincerL = PINCER_L; 
const int pincerR = PINCER_R;
const int mouth = MOUTH; 

const float pi = PI;

int posZ = POS_Z;
int posY = POS_Y;
const int amplitude = AMPLITUDE;
float phaseZ = PHASE_Z; 


//int step_size = STEP_SIZE; //set angle size of the step

int legServos[6][2] = LEG_SERVOS;

int currentAngle[6][2] = CURRENT_ANGLE;

int sideOffset[6] = SIDE_OFFSET; 

int legOffsetNewYear[6] = LEG_OFFSET_NEW_YEAR;

//FOR NOW: no leg offset for v2. See if all move in tandem. Then, play with offset values to alter leg motions
float legOffsetV1[6] = LEG_OFFSET_V1; //use to created a delay between the start of each leg motion while walking

//FOR NOW: no leg offset for v2. See if all move in tandem. Then, switch 2, 4, 6 to negative
int legOffsetV2[6] = LEG_OFFSET_V2; //used to switch leg 2, 4, 6 y motion to be reversed, so robot walks instead of all servos moving in parallel
float phaseZV2[6] = PHASE_Z_V2; // shift z sine wave for legs 2, 4, 6 so the step occurs at the right time for legs moving in opposite sync
int legCounterOffsetZV2[6][2] = LEG_COUNTER_OFFSET_Z_V2; //offset counter to shift the z curve for walk v2

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Direction: increasing angles lifts legs R1-3 and moves them backwards. Lowers L1-3 and moves them forward

void setup() {
 Serial.begin(115200);
 pwm.begin();
 pwm.setOscillatorFrequency(27000000);
 pwm.setPWMFreq(servoFreq);  // Analog servos run at ~50 Hz updates
 pinMode(buttonG, INPUT);
 pinMode(buttonBL, INPUT);
 pinMode(buttonBR, INPUT);
 pinMode(buttonR, INPUT);

 //forward_step_initialization_v2();
 //delay(1000);


 for (int i = 0; i < numberServos; i++) { //initialize all servos to 90˚starting position
     for (int j = 85; j < 90; j++) {
     pwm.setPWM(i, 0, angleToPulse(j));
     delay(10);
     }
 }

 for (int i = 12; i < 16; i++) { //initialize the stinger, pincers, and mouth to start at 0
     pwm.setPWM(i, 0, angleToPulse(0));
     delay(10);
 }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

 int Button_state_G = digitalRead(buttonG);
 int Button_state_R = digitalRead(buttonR);
 int Button_state_BL = digitalRead(buttonBL);
 int Button_state_BR = digitalRead(buttonBR);

happy_new_year();
//step_leg_forward_v2();
//////////////////////////////////////////////////////////////////////////////////////////////////////
 while (Button_state_G == HIGH) {
   Serial.println("Green button = high");
   step_leg_forward_v2();
   //delay(50);
   Button_state_G = digitalRead(buttonG);
   //Serial.print("Push button = "); Serial.println(Button_state_G);
 }

//////////////////////////////////////////////////////////////////////////////////////////////////////
 // If the L pincer button is pressed, move pincer L
//  if (Button_state_BL == HIGH) {
//    Serial.println("blue-left button is HIGH");
//    for (int i = 0; i < 30; i++) {
//      pwm.setPWM(pincerL, 0, angleToPulse(i)); //move pincer L to 30 degrees
//      Button_state_BL = digitalRead(buttonBR);
//      delay(5);
//    }
//  }
//    else { //if not pressed, move pincer to 0
//      Serial.println("blue-left button is LOW");
//      for (int i = 0; i < 30; i++) {
//        pwm.setPWM(pincerL, 0, angleToPulse(0)); //move pincer L to start position
//        Button_state_BL = digitalRead(buttonBL);
//        delay(5);
//      }
//    }
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  // If the right pincer button is pressed, move pincer
//  if (Button_state_BR == HIGH) {
//    Serial.println("blue-right button is HIGH");
//    for (int i = 0; i < 30; i++) {
//      pwm.setPWM(pincerR, 0, angleToPulse(i)); //move pincer R to 30 degrees
//      Button_state_BR = digitalRead(buttonR);
//      delay(5);
//    }
//  }
//  else { //if not pressed, move pincer to 0
//      Serial.println("blue-right button is LOW");
//      pwm.setPWM(pincerR, 0, angleToPulse(0)); //move pincer R to start position
//      Button_state_BR = digitalRead(buttonR);
//      delay(5);
//      }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//  if (Button_state_R == HIGH) { //if red button is pressed, start sting or kiss funtion
//    Serial.println("Red button is HIGH");
//    sting_or_kiss();
//  }
//}
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
//void sting_or_kiss() { //uses ultrasonic sensor to either sting or kiss someone
//  
//}

void happy_new_year() {
  for (int leg = 0; leg < numberLegs; leg++) //iterate over all legs
     {
      int leg_servo_y = legServos[leg][y];
      posY = 90; //neutral position z
      pwm.setPWM(leg_servo_y, 0, angleToPulse(posY)); //move to 90. stay there
      currentAngle[leg][y] = posY;
     }
     
  for (int counter = -20; counter <= 20; counter++) 
   {
    for (int leg = 0; leg < numberLegs; leg++) //iterate over all legs
     {
       int leg_servo_z = legServos[leg][z];

       if (leg == 1) { //move second and fifth legs to wave in air
        posZ =  90 + -(legOffsetV2[leg]*sideOffset[leg]*abs(counter) + 30*legOffsetNewYear[leg]);
        pwm.setPWM(leg_servo_z, 0, angleToPulse(posZ));
        currentAngle[leg][z] = posZ;
       }

       else if (leg == 4) {
        posZ =  90 + -(legOffsetV2[leg]*sideOffset[leg]*abs(counter) + 30*legOffsetNewYear[leg]);
        pwm.setPWM(leg_servo_z, 0, angleToPulse(posZ));
        currentAngle[leg][z] = posZ;
        Serial.println(posZ);
       }

       else {
       //Set z
       posZ = legOffsetV2[leg]*sideOffset[leg]*abs(counter) + 90;
       //for legs 2, 5: alternate: move from 135 to 155 and back
       pwm.setPWM(leg_servo_z, 0, angleToPulse(posZ));
       currentAngle[leg][z] = posZ;
       }
    }
    delay(8);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////     step_leg_forward_v2()    ///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void step_leg_forward_v2() { //type 2 walk motion
 /////legs all have same motion, but legs 2, 4, 6 are doing the opposite timing to legs 1, 3, 5. 

 Serial.println("step forward v2");

   for (int counter = 0; counter <= 360; counter++) 
   {
     for (int leg = 0; leg < numberLegs; leg++) //iterate over all legs
     {
       int leg_servo_y = legServos[leg][y];
       int leg_servo_z = legServos[leg][z];

         //Set y 
         posY = legOffsetV2[leg]*sideOffset[leg]*amplitude*sin(counter*pi/180) + 90; //add legOffsetV2[leg] to switch sine from +/- for legs 2, 4, 6
         pwm.setPWM(leg_servo_y, 0, angleToPulse(posY));
         Serial.print(posY); Serial.print(" -- "); Serial.println(posZ);
         currentAngle[leg][y] = posY;

         //Set z walk --> the legs that are moving in opposite directions will have z moving in same direction, but the part of the curve that gets chopped shifts by pi. 
         //Since the sine curve is automatically flipped upside down, the slope method works to shift z.
         posZ = sideOffset[leg]*amplitude*sin(phaseZV2[leg] + phaseZ + (-32+counter)*pi/180) + 90; //include "-32" as offset to fix phase lag issue. phaseZV2 is for legs 2, 4, 6 since their motion is opposite sync
         if (sideOffset[leg]*posZ < 90)
         {
           pwm.setPWM(leg_servo_z, 0, angleToPulse(posZ));
           currentAngle[leg][z] = posZ;
           //Serial.print(posY); Serial.print(" -- "); Serial.println(posZ);
           //Serial.print("z_leg1 is "); Serial.println(currentAngle[0][z]);
           //delay(5);
         }

         else {
           int posZ_2 = 90; //if slope is negative. leg is on ground --> flattened curve
           pwm.setPWM(leg_servo_z, 0, angleToPulse(posZ_2)); 
           currentAngle[leg][z] = posZ_2;
           //Serial.print(posY); Serial.print(" -- "); Serial.println(posZ);
           Serial.print("z_leg1 is "); Serial.println(currentAngle[0][z]);
           //delay(5);
         }
     }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int angleToPulse(int ang) {
 int pulse = map(ang, 0, 180, servoMin, servoMax);
 //Serial.print("Angle: "); Serial.print(ang);
 //Serial.print(" pulse: "); Serial.println(pulse);
 return pulse;
}
