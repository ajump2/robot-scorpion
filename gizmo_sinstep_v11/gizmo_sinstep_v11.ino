#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

int trigPin = 9;
int echoPin = 8;
long distance;
long duration;

#define SERVOMIN 120
#define SERVOMAX 630
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
uint8_t servonum = 0;
int starting_angle = 90;
const int number_servos = 12; //number of servos in legs
const int number_legs = 6;
const int z = 0; //to reference the correct column in leg_servos (z motion)
const int y = 1;

const int Button_G = 33; //set button pin for walk forward (green)
const int Button_BL = 35; //set button pin for left pincer
const int Button_BR = 32; //set button pin for right pincer
const int Button_R = 34; //set button pin for left sting

const int stinger = 12; //set stinger servo to #13
const int pincer_L = 13; //set pincer_L = servo #14
const int pincer_R = 14; //set pincer_R = servo #15
const int mouth = 15; //set mouth servo to #16

const float pi = 3.141592653589793238462643383279502884197169399375105820974944592; //define pi

int pos_z = 0;
int pos_y = 0;
const int amplitude = 15;
float phase_z = -pi/2; // shift z sine wave by 1/4 of a cycle + an offset to fix some sort of persisting lag


//int step_size = 5; //set angle size of the step

int leg_servos[6][2] = { {0, 6}, //right legs: legs 1, 2, 3 {y, z} servos
                        {1, 7},
                        {2, 8},

                        {3, 9}, //left legs: legs 4, 5, 6
                        {4, 10},
                        {5, 11}
                        };

int current_angle[6][2] = { {90, 90}, //right legs: legs 1, 2, 3 {y, z} servos. Set current angle
                           {90, 90},
                           {90, 90},

                           {90, 90}, //left legs: legs 4, 5, 6
                           {90, 90},
                           {90, 90}
                          };

int side_offset[6] = {-1, -1, -1, 1, 1, 1}; //use to created mirrored motion for left-side legs

int leg_offset_newYear[6] = {1, 1, 1, 1, -1, 1};

//FOR NOW: no leg offset for v2. See if all move in tandem. Then, play with offset values to alter leg motions
float leg_offset_v1[6] = {0, 0, 0, 0, 0, 0}; //use to created a delay between the start of each leg motion while walking

//FOR NOW: no leg offset for v2. See if all move in tandem. Then, switch 2, 4, 6 to negative
int leg_offset_v2[6] = {1, 1, 1, 1, 1, 1}; //used to switch leg 2, 4, 6 y motion to be reversed, so robot walks instead of all servos moving in parallel
float phase_z_v2[6] = {0, 0, 0, 0, 0, 0};; // shift z sine wave for legs 2, 4, 6 so the step occurs at the right time for legs moving in opposite sync
int leg_counter_offset__z_v2[6][2] = {0, 180, 0, 180, 0, 180}; //offset counter to shift the z curve for walk v2

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Direction: increasing angles lifts legs R1-3 and moves them backwards. Lowers L1-3 and moves them forward

void setup() {
 Serial.begin(115200);
 pwm.begin();
 pwm.setOscillatorFrequency(27000000);
 pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
 pinMode(Button_G, INPUT);
 pinMode(Button_BL, INPUT);
 pinMode(Button_BR, INPUT);
 pinMode(Button_R, INPUT);

 //forward_step_initialization_v2();
 //delay(1000);


 for (int i = 0; i < number_servos; i++) { //initialize all servos to 90˚starting position
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

 int Button_state_G = digitalRead(Button_G);
 int Button_state_R = digitalRead(Button_R);
 int Button_state_BL = digitalRead(Button_BL);
 int Button_state_BR = digitalRead(Button_BR);

happy_new_year();
//step_leg_forward_v2();
//////////////////////////////////////////////////////////////////////////////////////////////////////
 while (Button_state_G == HIGH) {
   Serial.println("Green button = high");
   step_leg_forward_v2();
   //delay(50);
   Button_state_G = digitalRead(Button_G);
   //Serial.print("Push button = "); Serial.println(Button_state_G);
 }

//////////////////////////////////////////////////////////////////////////////////////////////////////
 // If the L pincer button is pressed, move pincer L
//  if (Button_state_BL == HIGH) {
//    Serial.println("blue-left button is HIGH");
//    for (int i = 0; i < 30; i++) {
//      pwm.setPWM(pincer_L, 0, angleToPulse(i)); //move pincer L to 30 degrees
//      Button_state_BL = digitalRead(Button_BR);
//      delay(5);
//    }
//  }
//    else { //if not pressed, move pincer to 0
//      Serial.println("blue-left button is LOW");
//      for (int i = 0; i < 30; i++) {
//        pwm.setPWM(pincer_L, 0, angleToPulse(0)); //move pincer L to start position
//        Button_state_BL = digitalRead(Button_BL);
//        delay(5);
//      }
//    }
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  // If the right pincer button is pressed, move pincer
//  if (Button_state_BR == HIGH) {
//    Serial.println("blue-right button is HIGH");
//    for (int i = 0; i < 30; i++) {
//      pwm.setPWM(pincer_R, 0, angleToPulse(i)); //move pincer R to 30 degrees
//      Button_state_BR = digitalRead(Button_R);
//      delay(5);
//    }
//  }
//  else { //if not pressed, move pincer to 0
//      Serial.println("blue-right button is LOW");
//      pwm.setPWM(pincer_R, 0, angleToPulse(0)); //move pincer R to start position
//      Button_state_BR = digitalRead(Button_R);
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
  for (int leg = 0; leg < number_legs; leg++) //iterate over all legs
     {
      int leg_servo_y = leg_servos[leg][y];
      pos_y = 90; //neutral position z
      pwm.setPWM(leg_servo_y, 0, angleToPulse(pos_y)); //move to 90. stay there
      current_angle[leg][y] = pos_y;
     }
     
  for (int counter = -20; counter <= 20; counter++) 
   {
    for (int leg = 0; leg < number_legs; leg++) //iterate over all legs
     {
       int leg_servo_z = leg_servos[leg][z];

       if (leg == 1) { //move second and fifth legs to wave in air
        pos_z =  90 + -(leg_offset_v2[leg]*side_offset[leg]*abs(counter) + 30*leg_offset_newYear[leg]);
        pwm.setPWM(leg_servo_z, 0, angleToPulse(pos_z));
        current_angle[leg][z] = pos_z;
       }

       else if (leg == 4) {
        pos_z =  90 + -(leg_offset_v2[leg]*side_offset[leg]*abs(counter) + 30*leg_offset_newYear[leg]);
        pwm.setPWM(leg_servo_z, 0, angleToPulse(pos_z));
        current_angle[leg][z] = pos_z;
        Serial.println(pos_z);
       }

       else {
       //Set z
       pos_z = leg_offset_v2[leg]*side_offset[leg]*abs(counter) + 90;
       //for legs 2, 5: alternate: move from 135 to 155 and back
       pwm.setPWM(leg_servo_z, 0, angleToPulse(pos_z));
       current_angle[leg][z] = pos_z;
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
     for (int leg = 0; leg < number_legs; leg++) //iterate over all legs
     {
       int leg_servo_y = leg_servos[leg][y];
       int leg_servo_z = leg_servos[leg][z];

         //Set y 
         pos_y = leg_offset_v2[leg]*side_offset[leg]*amplitude*sin(counter*pi/180) + 90; //add leg_offset_v2[leg] to switch sine from +/- for legs 2, 4, 6
         pwm.setPWM(leg_servo_y, 0, angleToPulse(pos_y));
         Serial.print(pos_y); Serial.print(" -- "); Serial.println(pos_z);
         current_angle[leg][y] = pos_y;

         //Set z walk --> the legs that are moving in opposite directions will have z moving in same direction, but the part of the curve that gets chopped shifts by pi. 
         //Since the sine curve is automatically flipped upside down, the slope method works to shift z.
         pos_z = side_offset[leg]*amplitude*sin(phase_z_v2[leg] + phase_z + (-32+counter)*pi/180) + 90; //include "-32" as offset to fix phase lag issue. phase_z_v2 is for legs 2, 4, 6 since their motion is opposite sync
         if (side_offset[leg]*pos_z < 90)
         {
           pwm.setPWM(leg_servo_z, 0, angleToPulse(pos_z));
           current_angle[leg][z] = pos_z;
           //Serial.print(pos_y); Serial.print(" -- "); Serial.println(pos_z);
           //Serial.print("z_leg1 is "); Serial.println(current_angle[0][z]);
           //delay(5);
         }

         else {
           int pos_z_2 = 90; //if slope is negative. leg is on ground --> flattened curve
           pwm.setPWM(leg_servo_z, 0, angleToPulse(pos_z_2)); 
           current_angle[leg][z] = pos_z_2;
           //Serial.print(pos_y); Serial.print(" -- "); Serial.println(pos_z);
           Serial.print("z_leg1 is "); Serial.println(current_angle[0][z]);
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
 int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
 //Serial.print("Angle: "); Serial.print(ang);
 //Serial.print(" pulse: "); Serial.println(pulse);
 return pulse;
}
