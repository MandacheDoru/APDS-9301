#include "Wire.h"
#include <Sparkfun_APDS9301_Library.h>

APDS9301 apds;

#define INT_PIN 2 

bool lightIntHappened = false; 
                               
void setup() {
  Serial.begin(115200);
  Wire.begin();

  apds.begin(0x39);                                     
                                                       
  apds.setGain(APDS9301::LOW_GAIN);                    
  apds.setIntegrationTime(APDS9301::INT_TIME_13_7_MS); 
  apds.setLowThreshold(0); 
  apds.setHighThreshold(50);
  apds.setCyclesForInterrupt(1);
  apds.enableInterrupt(APDS9301::INT_ON); 
  apds.clearIntFlag();

  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), lightInt, FALLING);
}

void loop() {
  static unsigned long outLoopTimer = 0;
  apds.clearIntFlag();                          

  if (millis() - outLoopTimer >= 1000)
  {
    outLoopTimer = millis();

    Serial.print("Luminous flux: ");
    Serial.println(apds.readLuxLevel(),6);

    if (lightIntHappened)
    {
      Serial.println("Interrupt");
      lightIntHappened = false;
    }
  }
}

void lightInt()
{
  lightIntHappened = true;
}
