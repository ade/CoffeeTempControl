
#define COFFEE_TEMPERATURE 95         

// which analog pin to connect
#define THERMISTORPIN A0         
// resistance at 25 degrees C
#define THERMISTORNOMINAL 61370
//#define THERMISTORNOMINAL 327755
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 36.7
//#define TEMPERATURENOMINAL 0
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 2
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 110000    
 
int samples[NUMSAMPLES];
 
void setup(void) {
  Serial.begin(9600);

  //Relay Ctrl
  pinMode(A1, OUTPUT);

  //Relay VCC
  pinMode(A2, OUTPUT);
  analogWrite(A2, 255);
  
  //Thermistor GND
  pinMode(A3, OUTPUT);
  analogWrite(A3, 0);
}
 
void loop(void) {
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
 
  Serial.print("Average analog reading "); 
  Serial.println(average);
 
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
  Serial.print("Thermistor resistance "); 
  Serial.println(average);
 
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
 
  Serial.print("Temperature "); 
  Serial.print(steinhart);
  Serial.println(" *C");

  analogWrite(1, 255);
  
  if(steinhart > COFFEE_TEMPERATURE) {
    analogWrite(A1, 255);
  } else {
    analogWrite(A1, 0);
  }
  
 
  delay(1000);
}
