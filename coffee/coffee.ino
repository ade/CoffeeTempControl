#include <PID_v1.h>


#define COLD_THRESHOLD 70
#define COFFEE_TEMPERATURE 97
#define OVERHEAT_TEMPERATURE 103
#define STEAM_READY 125         

#define RGB_R 7
#define RGB_G 8
#define RGB_B 9

// which analog pin to connect
#define THERMISTORPIN A0         
// resistance at 25 degrees C
// temp. for nominal resistance (almost always 25 C)

#define THERMISTORNOMINAL 61370
#define TEMPERATURENOMINAL 36.7

//#define THERMISTORNOMINAL 100000
//#define TEMPERATURENOMINAL 25

// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 4
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 110000    
 
int samples[NUMSAMPLES];


//PID SETUP
//Define Variables we'll be connecting to
double PIDSetpoint, PIDInput, PIDOutput;

//Specify the links and initial tuning parameters
//PID myPID(&PIDInput, &PIDOutput, &PIDSetpoint,1.5,0.05,11.5, DIRECT);

int PIDMax = 3000;
float lastTemp = 0;
 
void setup(void) {
  //PID
  
  /*
  PIDSetpoint = COFFEE_TEMPERATURE;
  myPID.SetOutputLimits(0, PIDMax);
  myPID.SetMode(AUTOMATIC);
  */

  //Other
  Serial.begin(9600);

  //Relay Ctrl
  pinMode(A1, OUTPUT);

  //Relay VCC
  pinMode(A2, OUTPUT);
  analogWrite(A2, 255);
  
  //Thermistor GND
  pinMode(A3, OUTPUT);
  analogWrite(A3, 0);

  pinMode(RGB_R, OUTPUT);
  digitalWrite(RGB_R, 1);

  pinMode(RGB_G, OUTPUT);
  digitalWrite(RGB_G, 0);

  pinMode(RGB_B, OUTPUT);
  digitalWrite(RGB_B, 0);
}

float getTemperature() {
  uint8_t i;
  float average;
  
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTORPIN);
   delay(10);
  }
 
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
 
  //Serial.print("Average analog reading "); 
  //Serial.println(average);
 
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
  //Serial.print("Thermistor resistance "); 
  //Serial.println(average);
 
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C

  return steinhart;
}
 
void loop(void) {
  float steinhart = getTemperature();
  Serial.print("Temperature "); 
  Serial.print(steinhart);
  Serial.println(" *C");

  analogWrite(1, 255);
  bool heat = false;
  bool ledblink = false;
  bool red = false;
  bool blue = false;
  bool green = false;
  
  if(steinhart < COLD_THRESHOLD) {
    blue = true;
    heat = true;
  } else if(steinhart >= COLD_THRESHOLD && steinhart < COFFEE_TEMPERATURE) {
    red = true;
    heat = true;
  } else if(steinhart >= COFFEE_TEMPERATURE && steinhart < OVERHEAT_TEMPERATURE) {  
    green = true;
    heat = false;
  } else if(steinhart >= OVERHEAT_TEMPERATURE && steinhart < STEAM_READY) {  
    green = true;
    ledblink = true;
    heat = false;
  } else if(steinhart >= STEAM_READY) {
    heat = false;
    green = true;
    blue = true;
  }

  /*
  //PID
  PIDInput = steinhart;
  myPID.Compute();

  int pidTime = (int)PIDOutput;
  int sleepTime = (int)(PIDMax - PIDOutput);

  Serial.print("On for ");
  Serial.print(PIDOutput);
  Serial.print("ms. Sleep for ");
  Serial.print(sleepTime);
  Serial.println("ms.");

  if(PIDOutput > 0) {
    analogWrite(A1, 0);
    delay(pidTime);
  }

  if(sleepTime > 0) {
    analogWrite(A1, 255);
    delay(sleepTime);
  }
  */


  /************************************************
  * turn the output pin on/off based on pid output
  ************************************************/
  /*
  unsigned long now = millis();
  if(now - windowStartTime > WindowSize) { 
    //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(PIDOutput > now - windowStartTime) {
    analogWrite(A1, 0);
  } else { 
    analogWrite(A1, 255);
  }
  */

 
  if(heat) {
    analogWrite(A1, 0);

    float diff = steinhart - lastTemp;
    lastTemp = steinhart;
    bool gaining = false;

    if(diff > 2) {
      gaining = true;
    }
    
    if(steinhart < 80 || (steinhart < 92 && !gaining)) {
      delay(2000);
    } else if(steinhart < 92) {
      delay(1000);
      analogWrite(A1, 255);
      delay(1000);
    } else if(steinhart < COFFEE_TEMPERATURE-1) {
      delay(500);
      analogWrite(A1, 255);
      delay(1000);
    } else {
      delay(500);
      analogWrite(A1, 255);
      delay(1500);
    }
  } else {
    analogWrite(A1, 255);
    delay(1000);
  }
  

  if(!red) {
    digitalWrite(RGB_R, 1);
  }
  if(!green) {
    digitalWrite(RGB_G, 1);
  }
  if(!blue) {
    digitalWrite(RGB_B, 1);
  }

  if(ledblink) {   
    if(red) digitalWrite(RGB_R,0);
    if(green) digitalWrite(RGB_G,0);
    if(blue) digitalWrite(RGB_B,0);
    delay(500);
    digitalWrite(RGB_R,1);
    digitalWrite(RGB_G,1);
    digitalWrite(RGB_B,1);
    delay(500);
  } else {
    if(red) digitalWrite(RGB_R,0);
    if(green) digitalWrite(RGB_G,0);
    if(blue) digitalWrite(RGB_B,0);
    delay(1000);
  }
}
