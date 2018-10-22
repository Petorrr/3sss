// *****************************************************************************
// INCLUDES Section
// *****************************************************************************
#include "src\hx711.h"
//#include <FreqPeriodCounter.h>
#include <SPI.h>
#include <SD.h>

// *****************************************************************************
// DEFINITIONS Section
// *****************************************************************************
// Main Loop timing definitions
#define MAIN_LOOP_MILLIS    250
#define HX711_PWRUP_MILLIS  (MAIN_LOOP_MILLIS-60)
#define MAIN_TICKS_PER_SEC  (1000/MAIN_LOOP_MILLIS)
#define MSEC_TO_TICKS(x)    (uint16_t) (((uint32_t) (x)*(uint32_t) MAIN_TICKS_PER_SEC)/(uint32_t) 1000)

// Set the pins used
#define CARD_SELECT     (4)
#define CARD_DETECT     (7)
#define CLOSE_FILE_PIN  (A5)
#define VBAT_PIN        (A7)
#define RED_LED_PIN     (13)
#define GREEN_LED_PIN   (8)
#define HX711_DATA_PIN  (14)
#define HX711_CLOCK_PIN (15)
#define SPEED_INPUT_PIN (10)

// Other string and buf definitions
#define LOG_BUF_SIZE    80
#define SPEED_SAMPLES   4

// Fatal error defintions
typedef enum
{
  NO_ERROR,
  LOG_FILE_CLOSED,
  ERR_NO_CARD,
  ERR_MAX_LOGS,
  ERR_CREATE_FILE,
  NR_OF_ERRORS
};

// *****************************************************************************
// GLOBAL Variables Section
// *****************************************************************************
// Logdata variables
static File Logfile;
static char LogBuf[LOG_BUF_SIZE];

// Main loop variables
static uint16_t MainCount = 0;
static uint32_t TotalSeconds = 0;

// Loadcell, Strain Gauge Amplifier variables
const uint8_t  hx711_data_pin = HX711_DATA_PIN;
const uint8_t  hx711_clock_pin = HX711_CLOCK_PIN;
static int32_t Hx711SensorValA = 0;
static int32_t Hx711SensorValB = 0;
static int32_t Hx711SensorFilt = 0;
static int32_t Hx711SensorOffset = 0;
// Construct the Loadcell Amplifier object
static HX711 hx711 = HX711(hx711_data_pin, hx711_clock_pin, 128);

// Speed caluclation variables
static uint32_t CurrentPulseTime = 0;          // In us
static uint32_t CurrentFreq = 0;               // In Hertz
static uint32_t CurrentSpeed = 0;              // In #.## m/s
static uint32_t CurrentKmPerH = 0;             // In km/h
static uint32_t SpeedPulseBuf[SPEED_SAMPLES];

// *****************************************************************************
// Local Function prototypes 
// *****************************************************************************
static void determineSpeed(void);
static void error(uint8_t errno);

// *****************************************************************************
// Initialization and Board setup routines
// *****************************************************************************
void setup()
{
char    filename[15];
uint8_t i;

  Serial.begin(115200);
  // Wait for native USB to be ready, max 5 seconds timeout
  while (!Serial && millis() < 5000)
    ;
  Serial.println("\nFeather Logger test");

  // Initialize digital LED pins as an output
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);

  pinMode(CLOSE_FILE_PIN, INPUT_PULLUP);
  pinMode(SPEED_INPUT_PIN, INPUT_PULLUP);

  // Test PWM output, use wire to pin 10 to test determineSpeed function
  analogWrite(5, 128);
  
  // See if the card is present and can be initialized:
  Serial.print("Initializing SD card...");
  if (!SD.begin(CARD_SELECT))
  {
    Serial.println("Card failed, or not present");
    // Throw an error exception
    error(ERR_NO_CARD);
  }
  Serial.println("Card initialized.");

  // Find next 'free' Logfile
  strcpy(filename, "LOG000.TXT");
  for (i = 0; i < 100; i++)
  {
    filename[4] = '0' + i/10;
    filename[5] = '0' + i%10;
    // Create if does not exist, do not open existing, write, sync after write
    if (!SD.exists(filename))
      break;
  }
  if (i==100)
    error(ERR_MAX_LOGS);

  Logfile = SD.open(filename, FILE_WRITE);
  if(!Logfile)
  {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    error(ERR_CREATE_FILE);
  }
  Serial.print("Writing to "); 
  Serial.println(filename);

  Serial.println("Setup routine is Ready");
}

 
// *****************************************************************************
// Main process loop task
// *****************************************************************************
void loop()
{
uint16_t vbat;
uint32_t startMillis;
uint32_t execMillis;
int      rxChar=-1;

  // Setup and maintain loop timing and counter, blinky
  startMillis = millis();
  MainCount++;
  // Maintain Total Seconds counter, Logfile flushing and blinky 
  if ((MainCount % MSEC_TO_TICKS(1000)) == 0)
  {
    digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));
    TotalSeconds++;
    
    // Flush data every 10 seconds to SD card, indicate Green LED
     if ((TotalSeconds % 10) == 0)
     {
       digitalWrite(GREEN_LED_PIN, HIGH);
       Logfile.flush();
       digitalWrite(GREEN_LED_PIN, LOW);
     }
  }

  // Read sensors data
  vbat = analogRead(VBAT_PIN);
  vbat = (vbat * 2 * 330) / 1024;        // divided by 2, multiply by Ref and divide by 1024
  // =====================================
  // Read the value from the HX711 sensors
  // =====================================
  if (hx711.is_ready())
  {
      Hx711SensorValA = hx711.read();
 
#if 0
    if ((MainCount % 1) == 0)
    {
      hx711.set_gain(32);
      Hx711SensorValA = hx711.read();
    }
    else
    {
      hx711.set_gain(128);
      Hx711SensorValB = hx711.read();
    }
#endif
  }

  // Measure and determine the Speed
  determineSpeed();
  
  // Create the logfile string
  sprintf(LogBuf, "%04u.%1u, %3u, %6d, %4d", startMillis / 1000, (startMillis % 1000) / 100, vbat, Hx711SensorValA, CurrentFreq);
  Logfile.println(LogBuf);

  // Handle any input character
  if (Serial.available() > 0)
  {
    // Read and echo the incoming byte
    rxChar = Serial.read();
    Serial.print("rxChar: ");
    Serial.println(rxChar, DEC);
  }
  // Check for stop logging pin activated or Space Character
  if (digitalRead(CLOSE_FILE_PIN) == LOW || rxChar == ' ')
  {
    Logfile.flush();
    Logfile.close();
    // Not really an error, but quit logging
    error(LOG_FILE_CLOSED);
  }
  Serial.println(LogBuf);

  // Determine execution time, and delay for the remainder
  execMillis = (uint16_t) (millis() - startMillis);
  delay(MAIN_LOOP_MILLIS - execMillis);

}

// *****************************************************************************
// Speed determination with CDM324 Doppler sensor
// *****************************************************************************
static void determineSpeed(void)
{
uint32_t pulse_length = 0;
uint8_t  i;
boolean  check;

  // Sample the input frequency and average, interrupts off
  noInterrupts();
  pulseIn(SPEED_INPUT_PIN, HIGH);
  for (i = 0; i < SPEED_SAMPLES; i++)
  {
    pulse_length = pulseIn(SPEED_INPUT_PIN, HIGH); 
    pulse_length += pulseIn(SPEED_INPUT_PIN, LOW);    
    SpeedPulseBuf[i] = pulse_length;
  }
  interrupts();

  check = true;
  pulse_length = SpeedPulseBuf[0];
  for (i = 1; i < SPEED_SAMPLES; i++)
  {
    pulse_length += SpeedPulseBuf[i];
    // Check validity of samples for outliers
    if ((SpeedPulseBuf[i] > SpeedPulseBuf[0] * 2) || (SpeedPulseBuf[i] < SpeedPulseBuf[0] / 2))
      check = false;
  }

  if (check)
  {
    CurrentPulseTime = pulse_length / SPEED_SAMPLES;
    CurrentFreq = 1000000 / CurrentPulseTime;
    CurrentKmPerH = CurrentFreq / 44;
    CurrentSpeed = (1000 * CurrentFreq) / (44 *36);
  }
}

// *****************************************************************************
// Error Handler Function: will block further action
// *****************************************************************************
static void error(uint8_t errno)
{
uint8_t i;

  while(1)
  {
    if (errno == LOG_FILE_CLOSED)
      digitalWrite(GREEN_LED_PIN, HIGH);
 
    for (i=0; i<errno; i++)
    {
      digitalWrite(RED_LED_PIN, HIGH);
      delay(100);
      digitalWrite(RED_LED_PIN, LOW);
      delay(100);
      Serial.print("In error function, code = ");
      switch (errno)
      {
        case NO_ERROR:        Serial.println("NO_ERROR"); break;
        case LOG_FILE_CLOSED: Serial.println("LOG_FILE_CLOSED"); break;
        case ERR_NO_CARD:     Serial.println("ERR_NO_CARD"); break;
        case ERR_MAX_LOGS:    Serial.println("ERR_MAX_LOGS"); break;
        case ERR_CREATE_FILE: Serial.println("ERR_CREATE_FILE"); break;
        default:              Serial.println("IMPOSSIBLE"); break;
      }
    }
    for (i=errno; i<10; i++)
      delay(200);
  }
}

