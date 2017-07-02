//#include <bluefruit.h>
#include <stdio.h>

#define STRING_BUF_SIZE     80

static char TmpBuf [STRING_BUF_SIZE];

void setup()
{
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println ("Hello World");
}

void loop()
{
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
  sprintf (TmpBuf, "Millis since start: %ul msec", millis());
}
