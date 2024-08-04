#include <SoftwareSerial.h>

SoftwareSerial HC06(A0, A1);  // RX, TX
char BT_value;
String BT_value_temp;
volatile int Front_Distance;
volatile boolean Flag = true;

const int Trig = A3;
const int Echo = A2;
const int PWM2A = 11;      //M1 motor
const int PWM2B = 3;       //M2 motor
const int PWM0A = 6;       //M3 motor
const int PWM0B = 5;       //M4 motor
const int DIR_CLK = 4;     // Data input clock line
const int DIR_EN = 7;      //Equip the L293D enabling pins
const int DATA = 8;        // USB cable
const int DIR_LATCH = 12;  // Output memory latch clock
//Define the pin of ultrasonic obstacle avoidance sensor
//Define motion state
const int Move_Forward = 39;       //Move Forward
const int Move_Backward = 216;     //Move Backward
const int Left_Move = 116;         //Left translation
const int Right_Move = 139;        //Right translation
const int Right_Rotate = 149;      //Rotate Left
const int Left_Rotate = 106;       //Rotate Left
const int Stop = 0;                //Parking variable
const int Upper_Left_Move = 36;    //Upper Left Move
const int Upper_Right_Move = 3;    //Upper Right Move
const int Lower_Left_Move = 80;    //Lower Left Move
const int Lower_Right_Move = 136;  //Lower Right Move
const int Drift_Left = 20;         //Drift on Left
const int Drift_Right = 10;        //Drift on Right
//Set the default speed between 1 and 255

int Speed1 = 255;
int Speed2 = 255;
int Speed3 = 255;
int Speed4 = 255;

void Motor(int Dir, int Speed1, int Speed2, int Speed3, int Speed4) {
  analogWrite(PWM2A, Speed1);  //Motor PWM speed regulation
  analogWrite(PWM2B, Speed2);  //Motor PWM speed regulation
  analogWrite(PWM0A, Speed3);  //Motor PWM speed regulation
  analogWrite(PWM0B, Speed4);  //Motor PWM speed regulation

  digitalWrite(DIR_LATCH, LOW);            //DIR_LATCH sets the low level and writes the direction of motion in preparation
  shiftOut(DATA, DIR_CLK, MSBFIRST, Dir);  //Write Dir motion direction value
  digitalWrite(DIR_LATCH, HIGH);           //DIR_LATCH sets the high level and outputs the direction of motion
}

float checkdistance() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  float distance = pulseIn(Echo, HIGH) / 58.00;
  delay(10);
  return distance;
}

void Ultrasonic_Avoidance() {
  int Front_Distance = 0;

  Front_Distance = checkdistance();
  if (0 < Front_Distance && Front_Distance <= 45) {
    if (Front_Distance <= 20) {
      Flag = !Flag;
      Motor(Stop, 0, 0, 0, 0);
      delay(250);
      Motor(Move_Backward, Speed1, Speed2, Speed3, Speed4);
      delay(200);
      Motor(Stop, 0, 0, 0, 0);
      delay(250);
      if (Flag) {
        Motor(Left_Rotate, Speed1, Speed2, Speed3, Speed4);
      } else {
        Motor(Right_Rotate, Speed1, Speed2, Speed3, Speed4);
      }
      delay(100);
      Motor(Stop, 0, 0, 0, 0);
      delay(250);

    } else {
      Motor(Stop, 0, 0, 0, 0);
      delay(250);
      if (Flag) {
        Motor(Left_Rotate, Speed1, Speed2, Speed3, Speed4);
      } else {
        Motor(Right_Rotate, Speed1, Speed2, Speed3, Speed4);
      }
      delay(100);
      Motor(Stop, 0, 0, 0, 0);
      delay(250);
    }

  } else {
    Motor(Move_Forward, 100, 100, 100, 100);
  }
}

void Ultrasonic_Follow() {
  Front_Distance = checkdistance();
  if ((Front_Distance >= 0) && (Front_Distance <= 10)) {
    Motor(Move_Backward, Speed1, Speed2, Speed3, Speed4);
    delay(20);
  } else if ((Front_Distance > 10) && (Front_Distance <= 15)) {
    Motor(Stop, 0, 0, 0, 0);
    delay(20);
  } else {
    Motor(Move_Forward, 170, 170, 170, 170);
    delay(20);
  }
}

void setup() {

  BT_value = 'S';
  BT_value_temp = "";
  Front_Distance = 0;

  HC06.begin(9600);    //Set the myserial port baud rate 9600
  Serial.begin(9600);  //Set the serial port baud rate 9600
  //Configure as output mode
  pinMode(DIR_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(DIR_EN, OUTPUT);
  pinMode(DIR_LATCH, OUTPUT);
  pinMode(PWM0B, OUTPUT);
  pinMode(PWM0A, OUTPUT);
  pinMode(PWM2A, OUTPUT);
  pinMode(PWM2B, OUTPUT);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
}

void loop() {
  while (true) {
    while (HC06.available() > 0) {
      char receive = HC06.read();
      if (receive == 'T') {
        BT_value = receive;
      }
      if (receive == 'S') {
        BT_value = receive;
      }
    }
    if(BT_value == 'T'){
       Ultrasonic_Avoidance();
    }
    if(BT_value == 'S'){
       Motor(Stop, 0, 0, 0, 0);
    }
  }
}