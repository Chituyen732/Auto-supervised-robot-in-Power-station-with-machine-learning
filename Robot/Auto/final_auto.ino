#include "Wire.h"
#include <MPU6050_light.h>
MPU6050 mpu(Wire);
#include <Servo.h>
Servo myservo1, myservo2;

#define bSpeed 150
#define maxSpeed 230

//L298 kết nối arduino
const int ENA  = 5;  // kết nối chân ENA với chân 5 arduino
const int INA1 = 3;  // kết nối chân IN1 với chân 3 arduino
const int INA2 = 4;  // kết nối chân IN2 với chân 4 arduino

const int ENB  = 6;  // kết nối chân ENA với chân 5 arduino
const int INB1 = 7; // kết nối chân IN3 với chân 7 arduino
const int INB2 = 8; // kết nối chân IN4 với chân 8 arduino

const int HN = A3; // kết nối chân OUT Hồng ngoại với chân A3 ARDUINO

const int trigPin = 1; // kết nối chân trig cảm biến vật cản 
const int echoPin = A0; // kết nối chân echo cảm biến vật cản 

int nho = 0;


float Angle, angleQuay;

float timer;

/////////////////PID CONSTANTS/////////////////
float Kp = 25;//25
float Ki = 0.01; 
float Kd = 0.15; ///0.1 ,0.4, 1.4, 2.5

float PIDv=0, pErr=0, motorSpeedA, motorSpeedB;
float pid_p, pid_i = 0, pid_d;
bool nowHN = 0, lastHN = 0;
float a = 0;
int re = 0, T = 0;
float elapsedTime, time_pid, timePrev;

  //bien cho phan re trai
  float goctrai_after, goctrai_before;
  int dem_quaytrai = 0;
  
  //bien cho phan re phai
  float gocphai_after, gocphai_before;
  int dem_quayphai = 0;
  
  //bien cho phan di thang
  float Setpoint = 0.0;
  float Offset;

bool demQuay;

void setup() {
myservo1.attach(11);
myservo2.attach(12);
myservo1.write(90);
myservo2.write(90);
  pinMode(ENA, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(HN, INPUT); 

  //Cảm biến siêu âm xuất và nhận tín hiệu
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Wire.begin();  
  byte status = mpu.begin();
  while(status!=0){ } // stop everything if could not connect to MPU6050
  delay(2000);
  mpu.calcOffsets(); // gyro and accelero
}

void loop() {
  
          Thuc_hien();    

}

void readAngle(void){
  mpu.update();
  Angle = mpu.getAngleZ();
}

void readHN(void){
  nowHN = digitalRead(HN);
  if (nowHN > lastHN) {
    T = T + 1;
    timer = millis();    
    re = 0;
    };
  lastHN = nowHN;
}

void goLeft(float angleTurn_L){
  dem_quaytrai += 1; 
  if(dem_quaytrai == 1){
    readAngle();
    goctrai_before = Angle;
    goctrai_after = goctrai_before - angleTurn_L;
    };
  analogWrite(ENA,bSpeed+0);   //Left Motor Speed
  analogWrite(ENB,bSpeed+75);  //Right Motor Speed
  digitalWrite(INA1,LOW);
  digitalWrite(INA2,HIGH);
  digitalWrite(INB1,HIGH);
  digitalWrite(INB2,LOW);
  while(true) {
      readAngle();
      if (Angle <= goctrai_after) {
          dem_quaytrai = 0;
          break;
        };
  };
  
  a = a - angleTurn_L;
}

void goRight(float angleTurn_R){
  dem_quayphai += 1; 
  if(dem_quayphai == 1){
        readAngle();
        gocphai_before = Angle;
        gocphai_after = gocphai_before + angleTurn_R;
   }; 
  analogWrite(ENA,bSpeed+50);   //Left Motor Speed
  analogWrite(ENB,bSpeed+0);  //Right Motor Speed
  digitalWrite(INA1,HIGH);
  digitalWrite(INA2,LOW);
  digitalWrite(INB1,LOW);
  digitalWrite(INB2,HIGH);
  while(true) {
      readAngle();
      if (Angle >= gocphai_after) {
          dem_quayphai = 0;
          break;
        };
  };
  
  a = a + angleTurn_R;
}

void Stop(void){
  analogWrite(ENA,0);   //Left Motor Speed
  analogWrite(ENB,0);  //Right Motor Speed
  digitalWrite(INA1,LOW);
  digitalWrite(INA2,LOW);
  digitalWrite(INB1,LOW);
  digitalWrite(INB2,LOW);
}

void moveForward() {
  analogWrite(ENA, motorSpeedA); //Left Motor Speed
  analogWrite(ENB, motorSpeedB); //Right Motor Speed
  digitalWrite (INA1, HIGH);
  digitalWrite(INA2, LOW);
  digitalWrite (INB1, HIGH);
  digitalWrite(INB2, LOW);
}

void Calculate_Offset(float angleSet){
  
  readAngle();
  
  if( ( angleSet >= -10) && ( angleSet <= 10 ) ){
        Setpoint = 0;
  };
  
  if( (angleSet - (-90) >= -10) && ( angleSet - (-90) <= 10 ) ){
        Setpoint = -90;
  };
  
  if( ( angleSet - (-180) >= -10) && ( angleSet - (-180) <= 10 ) ){
        Setpoint = -180;
  };

  if( ( angleSet - (-270) >= -10) && ( angleSet - (-270) <= 10 ) ){
        Setpoint = -270;
  };

  if( (angleSet - 90 >= -10 ) && (angleSet - 90 <= 10) ){
        Setpoint = 90;
  };
      
      Offset = Setpoint - Angle;  
  
}

void goPID(float angleSet){

  readAngle();
  if( ( angleSet >= -10) && ( angleSet <= 10 ) ){
        Setpoint = 0;
  };
  
  if( (angleSet - (-90) >= -10) && ( angleSet - (-90) <= 10 ) ){
        Setpoint = -90;
  };
  
  if( ( angleSet - (-180) >= -10) && ( angleSet - (-180) <= 10 ) ){
        Setpoint = -180;
  };

  if( ( angleSet - (-270) >= -10) && ( angleSet - (-270) <= 10 ) ){
        Setpoint = -270;
  };
  
  if( (angleSet - 90 >= -10 ) && (angleSet - 90 <= 10) ){
        Setpoint = 90;
  };
  
  timePrev = time_pid;  // the previous time is stored before the actual time read
  time_pid = millis();  // actual time read
  elapsedTime = (time_pid - timePrev) / 1000; 
  
  Offset = Setpoint - Angle;  
  pid_p = Kp * Offset;
  pid_i = constrain( Ki * ( pid_i + (Offset*elapsedTime) ) ,-200,+200);
  pid_d = Kd * ( (Offset - pErr) / elapsedTime );
  PIDv = pid_p + pid_i + pid_d;
  
  motorSpeedA = bSpeed + PIDv;
  motorSpeedB = bSpeed - PIDv;
  motorSpeedA = constrain(motorSpeedA, 0, maxSpeed);
  motorSpeedB = constrain(motorSpeedB, 0, maxSpeed);
  moveForward(); 

  pErr = Offset;

}

void Thuc_hien(void){
    readHN();
    switch (T) {
        case 0:
              goPID(a);
              break;
        case 1:
              goPID(a);
              break;
        case 2:
              if ( re == 0){goLeft(90); re = 1;} else {goPID(a);};
              break;     
        case 3:
              if( nho  == 0 ){
                    if( (millis()-timer) <= 2000 ){ 
                          Stop();
                          myservo1.write(180);
                          myservo2.write(73);  
                    }else{
                          nho += 1;
                          goPID(a);
                    };
              }
              break;  
        case 4:
              if( nho  == 1 ){
                    if( (millis()-timer) <= 2000 ){ 
                          Stop();
                          myservo1.write(180);
                          myservo2.write(73);  
                    }else{
                          nho += 1;
                          goPID(a);
                    };
              }
              break;  
        case 5:
              if( nho  == 2 ){
                    if( (millis()-timer) <= 2000 ){ 
                          Stop();
                          myservo1.write(180);
                          myservo2.write(73);  
                    }else{
                          nho += 1;
                          goPID(a);
                    };
              }
              break;   
        case 6:
              if ( re == 0){goLeft(90); re = 1;} else {goPID(a);};
              break;
        case 7:
              if ( re == 0){goLeft(90); re = 1;} else {goPID(a);};
              break; 
        case 8:
              if( nho  == 3 ){
                    if( (millis()-timer) <= 2000 ){ 
                          Stop();
                          myservo1.write(0);
                          myservo2.write(73);  
                    }else{
                          nho += 1;
                          goPID(a);
                    };
              }
              break;   
        case 9:
              if( nho  == 4 ){
                    if( (millis()-timer) <= 2000 ){ 
                          Stop();
                          myservo1.write(0);
                          myservo2.write(73);  
                    }else{
                          nho = 0;
                          goPID(a);
                    };
              } 
              break; 
        case 10:
              if ( re == 0){goRight(90); re = 1;} else {goPID(a);};
              break;
        case 11:
              if ( re == 0){goRight(180); re = 1;} else {
                   Calculate_Offset(a);
                   if( (Offset > 1) || ( Offset < -1)){
                         if(Offset > 1){
                         goRight(Offset);
                         };
                         
                         if( Offset < -1){
                         goLeft(-Offset);
                         };
                   }else{
                          Stop();
                   };

              };
              break;
                              
     }
  
}

long microsecondsToCentimeters(long microseconds) {
    return microseconds / 29 / 2;
}

bool vatCan(void) {
   long duration, distance;
   pinMode(trigPin, OUTPUT);
   digitalWrite(trigPin, LOW);
   delayMicroseconds(2);
   digitalWrite(trigPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(trigPin, LOW);
   pinMode(echoPin, INPUT);
   duration = pulseIn(echoPin, HIGH,4000);
   distance = microsecondsToCentimeters(duration);   
   if ((distance > 0)&&(distance <40)) {
       return 1;}
   else{
       return 0;}   
}
