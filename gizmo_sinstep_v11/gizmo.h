#define TRIGPIN 9
#define ECHOPIN 8
#define SERVOMIN 120
#define SERVOMAX 630
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVO_NUM 0
#define STARTING_ANGLE 90
#define NUMBER_SERVOS 12 //number of servos in legs
#define NUMBER_LEGS 6
#define Z 0 //to reference the correct column in leg_servos (z motion)
#define Y 1
#define BUTTON_G 33 //set button pin for walk forward (green)
#define BUTTON_BL 35 //set button pin for left pincer
#define BUTTON_BR 32 //set button pin for right pincer
#define BUTTON_R 34 //set button pin for left sting
#define STINGER 12 //set stinger servo to #13
#define PINCER_L 13 //set pincerL = servo #14
#define PINCER_R  14 //set pincerR = servo #15
#define MOUTH 15 //set mouth servo to #16
#define PI 3.141592653589793238462643383279502884197169399375105820974944592 //define pi
#define POS_Z 0
#define POS_Y 0
#define AMPLITUDE 15
#define PHASE_Z -PI/2 // shift z sine wave by 1/4 of a cycle + an offset to fix some sort of persisting lag
#define STEP_SIZE 5
#define LEG_SERVOS {{0,6},{1,7},{2,8},{3,9},{4,10},{5,11}}
#define CURRENT_ANGLE {{90,90},{90,90},{90,90},{90,90},{90,90}}
#define SIDE_OFFSET {-1, -1, -1, 1, 1, 1} //use to created mirrored motion for left-side legs
#define LEG_OFFSET_NEW_YEAR {1,1,1,1,-1,1}
#define LEG_OFFSET_V1 {0,0,0,0,0,0}
#define LEG_OFFSET_V2 {1, 1, 1, 1, 1, 1}
#define PHASE_Z_V2 {0,0,0,0,0,0}
#define LEG_COUNTER_OFFSET_Z_V2 {0,180,0,180,0,180}