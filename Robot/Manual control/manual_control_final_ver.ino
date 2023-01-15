#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPLdVjnaMPh"
#define BLYNK_DEVICE_NAME "đk bang tay"
#define BLYNK_AUTH_TOKEN "q-7UD2kDD5sEWxUwcMgDGNis9VNjCufs"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>

bool forward = 0;
bool backward = 0;
bool left = 0;
bool right =0;
bool Mode = 0;
int Speed;

char auth[]="q-7UD2kDD5sEWxUwcMgDGNis9VNjCufs"; 
char ssid[]="Haha";
char pass []="12345678";


String v2arduino; // values to arduino
String myString; // complete message from arduino, which consistors of snesors data
char rdata; // received charactors

int forward_Val, backward_Val, left_Val, right_Val, Mode_Val, Speed_Val; // sensors value
// This function sends Arduino's up time every second to Virtual Pin (1).
// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.



void setup(){
      Serial.begin(9600);
      Blynk.begin(auth, ssid, pass);
}

void loop () {
      if (Serial.available() == 0 ) 
      {
            Blynk.run();
            toarduino();
      }
      
      if (Serial.available() > 0 ) 
      {
          rdata = Serial.read(); 
          myString = myString + rdata; 
          // Serial.print(rdata);
          if( rdata == '\n')
          {
      
                // new code
                String r_for = getValue(myString, ',', 0);
                String r_back = getValue(myString, ',', 1);
                String r_left = getValue(myString, ',', 2); 
                String r_right = getValue(myString, ',', 3);
                String r_mode = getValue(myString, ',', 4);        
                String r_speed = getValue(myString, ',', 5);    
                      
                forward_Val = r_for.toInt();
                backward_Val = r_back.toInt();
                left_Val = r_left.toInt();
                right_Val = r_right.toInt();
                Mode_Val = r_mode.toInt();
                Speed_Val = r_speed.toInt();
                
                myString = "";
      // end new code
          } 
      }

     
}

BLYNK_WRITE (V0) {
 forward = param.asInt ();
 int sdata = forward_Val;
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V6, sdata);
}

BLYNK_WRITE (V1) {
 backward = param.asInt ();
 int sdata = backward_Val;
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V7, sdata);
}

BLYNK_WRITE (V2) {
 left = param.asInt ();
 int sdata = left_Val;
// You can send any value at any time.
// Please don't send more that 10 values per second.
Blynk.virtualWrite(V8, sdata);
}

BLYNK_WRITE (V3) {
 right = param.asInt ();
 int sdata = right_Val;
// You can send any value at any time.
// Please don't send more that 10 values per second.
Blynk.virtualWrite(V9, sdata);
}

BLYNK_WRITE (V4) {
 Mode = param.asInt ();
 int sdata = Mode_Val;
// You can send any value at any time.
// Please don't send more that 10 values per second.
Blynk.virtualWrite(V10, sdata);
}

BLYNK_WRITE (V5) {
 Speed = param.asInt ();
 int sdata = Speed_Val;
// You can send any value at any time.
// Please don't send more that 10 values per second.
Blynk.virtualWrite(V11, sdata);
}

void toarduino()
{
      v2arduino = v2arduino + forward + "," + backward + "," + left + "," + right + "," + Mode + "," + Speed; 
      Serial.println(v2arduino); 
      delay(100); 
      v2arduino = ""; 
}

String getValue(String data, char separator, int index)
{
      int found = 0;
      int strIndex[] = { 0, -1 };
      int maxIndex = data.length() - 1;
  
      for (int i = 0; i <= maxIndex && found <= index; i++) {
            if (data.charAt(i) == separator || i == maxIndex) {
                  found++;
                  strIndex[0] = strIndex[1] + 1;
                  strIndex[1] = (i == maxIndex) ? i+1 : i;
            }
      }
      return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
