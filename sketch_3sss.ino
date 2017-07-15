// *****************************************************************************
// INCLUDES Section
// *****************************************************************************
#include <Q2HX711.h>
#include <stdio.h>

// *****************************************************************************
// DEFINITIONS Section
// *****************************************************************************
#define STRING_BUF_SIZE     80
#define PIEZO_BUF_SIZE      20
#define HX711_BUF_SIZE      20
#define ANA_RES_BITS        14
#define ANA_INP_RANGE       16384
#define ANA_VOLT_REF        (float) 3.6

// *****************************************************************************
// GLOBAL Variables Section
// *****************************************************************************
static char TmpBuf [STRING_BUF_SIZE];
static byte MainCount = 0;
//static bool LedState = false;

static word PiezoBuf [PIEZO_BUF_SIZE];
static byte PiezoSensor = A0;
static byte PiezoIndex = 0;

// Loadcell, Strain Gauge Amplifier variables
const byte  hx711_data_pin = (7);
const byte  hx711_clock_pin = (11);
static long Hx711Buf [HX711_BUF_SIZE];
static byte Hx711Index = 0;
// Construct the Loadcell Amplifier
Q2HX711 hx711(hx711_data_pin, hx711_clock_pin);


// *****************************************************************************
// Initialization and Board setup routines
// *****************************************************************************
void setup()
{
  // Put setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  Serial.begin(115200);
  Serial.println ("Hello 3sss");
  
  // Set the AD resolution to 14 bits
  analogReadResolution (ANA_RES_BITS);
  // Create readSensors task using Scheduler to run in 'parallel' with main loop()
  Scheduler.startLoop(readSensorsTask);
}


// *****************************************************************************
// Main process loop task
// *****************************************************************************
void loop()
{
byte  i;
long  piezoVal;
long  piezoMax;
float piezoV;

  // Main loop for serial output and debugging tasks:

  // Toggle Run Led every 500 msec
  if (MainCount % 5 == 0)
    digitalWrite(LED_BUILTIN, !digitalRead (LED_BUILTIN));
  
  // Calculate moving average of Piezo Sensor values (20 samples @ 20 Hz)
  piezoVal = 0;
  piezoMax = 0;
  for (i = 0; i < PIEZO_BUF_SIZE; i++)
  {
    piezoVal += PiezoBuf [i];
    if (PiezoBuf [i] > piezoMax)
      piezoMax = PiezoBuf [i];
  }
  piezoVal = piezoVal / PIEZO_BUF_SIZE;
  piezoV = (ANA_VOLT_REF * (float) piezoVal) / (float) ANA_INP_RANGE;
  
  // Output serial data for debugging
  //sprintf (TmpBuf, "Millis since start: %ul msec", millis());
  sprintf (TmpBuf, "Piezo: %u cnt; ", (word) piezoVal);
  Serial.print (TmpBuf);
  Serial.println (piezoV);
  
  // Run this task at 10 Hz
  delay (100);
  MainCount++;
}


// *****************************************************************************
// Sensors Input Task samples at 50 msec
// *****************************************************************************
void readSensorsTask()
{
  // Sensors read task:
   
  // Toggle the LED
  digitalWrite(LED_BLUE, !digitalRead (LED_BLUE));

  // Read the value from the Piezo sensor into the sample buffer
  PiezoBuf [PiezoIndex] = analogRead(PiezoSensor);
  PiezoIndex++;
  if (PiezoIndex >= PIEZO_BUF_SIZE)
    PiezoIndex = 0;

  // Read the value from the HX711 sensor into the sample buffer
  if (hx711.readyToSend())
  {
    Hx711Buf [Hx711Index] = hx711.read ();
    Hx711Index++;
    if (Hx711Index >= HX711_BUF_SIZE)
      Hx711Index = 0;
  }
     
  // Run this task at 20 Hz
  delay (50);
}

