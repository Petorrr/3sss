// INCLUDES Section
// *****************************************************************************
#include <stdio.h>
#include "src\hx711.h"
#include "src\adxl362.h"

#include <EEPROM.h>
#include <IntervalTimer.h>
#include <SPI.h>

// *****************************************************************************
// DEFINITIONS Section
// *****************************************************************************
// Main Loop timing definitions
#define MAIN_LOOP_MILLIS    50
#define MAIN_TICKS_PER_SEC  (1000/MAIN_LOOP_MILLIS)
#define MSEC_TO_TICKS(x)    (word) (((uint32_t) (x)*(uint32_t) MAIN_TICKS_PER_SEC)/(uint32_t) 1000)
// Sensor tick timing definitions
#define SENSOR_TICK_MILLIS      10
#define SENSOR_TICKS_PER_SEC    (1000/SENSOR_TICK_MILLIS)
#define MSEC_TO_SENSOR_TICKS(x) (word) (((uint32_t) (x)*(uint32_t) SENSOR_TICKS_PER_SEC)/(uint32_t) 1000)

// Debug settings definitions
#define DEBUG               1
#if DEBUG == 1
#define DEBUG_MAIN_PROCESS  1
#define DEBUG_ANT_TX        0
#define DEBUG_ANT_RX        0
#endif
#define ANT_SIMULATION      1
#define STRING_BUF_SIZE     80

// Hardware and Product definitions
#define HARDWARE_REVISION   ((byte) 0x01)
#define MANUFACTURER_ID     ((word) 0x00FF)
#define MODEL_NUMBER        ((word) 3555)
#define SW_MAIN_REVISION    1
#define SW_SUB_REVISION     2

// Sensor and input definitions
// Strain Gauge interface
#define HX711_BUF_SIZE      20
#define HX711_DATA_PIN      (3)
#define HX711_CLOCK_PIN     (2)
#define FORCE_SAMPLES_AVG   5
#define FORCE_SCALE_FACTOR  ((float) 500 /(float) 100000)
#define DEFAULT_FORCE_DIST  100     // in mm

// ADXL362 interface
#define DEFAULT_TEMP_OFFSET -77     // in counts
#define ACTIVITY_G          500     // in milli G
#define ACTIVITY_TIME       50      // in 100 Hz ticks: 0.5 sec
#define INACTIVITY_G        50      // in milli G
#define INACTIVITY_TIME     1000    // in 100 Hz ticks: 10 sec

#define ADXL362_CS_PIN      (15)
#define ADXL362_SCK_PIN     (14)

// Analog iputs interface/sensors
#define ANA_RES_BITS        12
#define ANA_INP_RANGE       4096
#define ANA_VOLT_REF        330
#define ANA_BATT_VOLT_CH    (A7)
#define BATTERY_NEW         1
#define BATTERY_GOOD        2
#define BATTERY_OK          3
#define BATTERY_LOW         4
#define BATTERY_CRITICAL    5

// EEPROM address definitions
#define EE_START_ADDR       2
#define EE_STRAIN_OFFSET    EE_START_ADDR + 0
#define EE_TEMP_OFFSET      EE_START_ADDR + 4

// Process state definitions
#define CRANK_UP            0
#define CRANK_FORWARD       1
#define CRANK_DOWN          2
#define CRANK_BACK          3
#define CRANK_UNKNOWN       0xFF

// ANT AP2 Interface definitions
#define AP2_TX_PIN          (10)
#define AP2_RX_PIN          (9)
#define AP2_RST_PIN         (8)
#define AP2_SUSPEND_PIN     (7)
#define AP2_SLEEP_PIN       (6)
#define AP2_RTS_PIN         (16)
#if 0
#define AP2_BR1_PIN         (17)
#define AP2_BR2_PIN         (18)
#define AP2_BR3_PIN         (19)
#endif

// ANT Bike Power Profile definitions
#define ANT_PUBLIC_NETWORK  0x00
#define ANT_CHANNEL_NR      0x00
#define ANT_DEVICE_NR       3555      //3SSS :)

#define BIKE_POWER_PROFILE  0x0B
#define BIKE_POWER_DEVICE   0x0001
#define BIKE_POWER_RF       57
#define BIKE_POWER_TT       0x05
#define BIKE_POWER_CP       8182

// ANT Protocol Message definitions
#define ANT_RXBUF_SIZE                40
#define ANT_RX_MAX_TIMEOUT            50 // In milliseconds (was 100 on Feather?)
// ANT Transmit Message & Msg ID definitions
#define MSG_TX_SYNC                   ((byte) 0xA4)
#define MSG_SYSTEM_RESET_ID           ((byte) 0x4A)
#define MSG_NETWORK_KEY_ID            ((byte) 0x46)
#define MSG_ASSIGN_CHANNEL_ID         ((byte) 0x42)
#define MSG_CHANNEL_ID_ID             ((byte) 0x51)
#define MSG_CHANNEL_RADIO_FREQ_ID     ((byte) 0x45)
#define MSG_CHANNEL_MESG_PERIOD_ID    ((byte) 0x43)
#define MSG_RADIO_TX_POWER_ID         ((byte) 0x47)
#define MSG_CHANNEL_SEARCH_TIMEOUT_ID ((byte) 0x44)
#define MSG_OPEN_CHANNEL_ID           ((byte) 0x4B)
#define MSG_BROADCAST_DATA_ID         ((byte) 0x4E)
#define MSG_ACKNOWLEDGE_DATA_ID       ((byte) 0x4F)

#define MSG_CHANNEL_RESPONSE          ((byte) 0x40)
#define MSG_TX_EVENT                  ((byte) 0x03)
#define MSG_CALIBRATION_REQUEST       ((byte) 0xAA)

// Define Data Pages
#define DP_CALIBRATION_REQ            ((byte) 0x01)
#define DP_STANDARD_POWER_ONLY        ((byte) 0x10)
#define DP_MANUFACTURER_INFO          ((byte) 0x50)
#define DP_PRODUCT_INFO               ((byte) 0x51)
#define DP_BATTERY_INFO               ((byte) 0x52)

// ANT Channel Type Codes
#define CHANNEL_TYPE_BIDIRECTIONAL_RECEIVE    0x00
#define CHANNEL_TYPE_BIDIRECTIONAL_TRANSMIT   0x10
#define CHANNEL_TYPE_UNIDIRECTIONAL_RECEIVE   0x50
#define CHANNEL_TYPE_UNIDIRECTIONAL_TRANSMIT  0x40
#define CHANNEL_TYPE_SHARED_RECEIVE           0x20
#define CHANNEL_TYPE_SHARED_TRANSMIT          0x30

// *****************************************************************************
// GLOBAL Variables Section
// *****************************************************************************
static char TmpBuf [STRING_BUF_SIZE];
static word MainCount = 0;
static word BatteryVoltage = 0;
static byte BatteryStatus;
static long TotalSeconds;
static IntervalTimer SensorTimer;

// Bike Power processing variables
static byte  CrankState = CRANK_UNKNOWN;
static byte  PrevCrankState = CRANK_UNKNOWN;
static word  CrankTicks = 0;
static word  CrankTime = 0;
static word  Cadence = 0;
static float Force;
static float Torque;
static float Power;
static word  ForceDistance = DEFAULT_FORCE_DIST;

// Loadcell, Strain Gauge Amplifier variables
const byte  hx711_data_pin = HX711_DATA_PIN;
const byte  hx711_clock_pin = HX711_CLOCK_PIN;
static long Hx711Buf [HX711_BUF_SIZE];
static byte Hx711Index = 0;
static long Hx711SensorVal = 0;
// Construct the Loadcell Amplifier object
static HX711 hx711 = HX711(hx711_data_pin, hx711_clock_pin, 128);

// ADXL362 interface variables
static ADXL362 Adxl;
static int16_t XValue;
static int16_t YValue;
static int16_t ZValue;
static int16_t Temperature;
static long    RealTemperature;
static word    TempOffset = DEFAULT_TEMP_OFFSET;

// ANT communication data
#define AntSerial Serial1

// ANT Developer key, if you want to connect to ANT+, you must get the key from thisisant.com
static const byte NETWORK_KEY[] = {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45};

// ANT TX messages processor variables
static byte       AntPedalPower;
static byte       AntCadence;
static word       AntAccuPower;
static word       AntPower;
static byte       SendCount = 0;
static byte       UpdateEventCount = 0;

// ANT RX messages processor variables
static boolean    MsgSync = false;
static byte       MsgIndex = 0;
static byte       MsgLength = 0;
static byte       MsgBuf [ANT_RXBUF_SIZE];
static boolean    AntCalibration = false;

// *****************************************************************************
// Local Function prototypes
// *****************************************************************************
static void readSensorsTask();

static void eepromGetConfig();
static void eepromSetConfig();

static void broadcastBikePower (void);
static void broadcastCalibration (boolean success, word calibrationVal);
static void broadcastMfg (void);
static void broadcastProduct (void);
static void broadcastBatteryInfo (void);

static void antSetup();

static void AP2reset (void);
static bool AP2waitCts (void);

static boolean ANTreceive (word timeout);
static void    ANTrxProcess (void);

static void ANTsend (uint8_t buf[], int length);
static byte checkSum (byte *dataPtr, int length);

static void ANTreset (void);
static void ANTsetNetwork (void);
static void ANTassignChannel (byte chNr, byte chType);
static void ANTsetChannelId (byte chNr, word deviceNr, byte deviceId, byte deviceTt);
static void ANTsetFrequency (byte chNr, byte freq);
static void ANTsetPeriod (byte chNr, word period);
static void ANTopenChannel (byte chNr);

// *****************************************************************************
// Initialization and Board setup routines
// *****************************************************************************
void setup()
{
byte temp;

  // Put setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("Hello 3sss");

  // Read the Configuration for the EPROM storage
  eepromGetConfig();
  TempOffset = DEFAULT_TEMP_OFFSET;
  
  // Initialize the SPI and ADXL interfaces
  pinMode(ADXL362_CS_PIN, OUTPUT);
  digitalWrite(ADXL362_CS_PIN, 1);
  SPI.setSCK(ADXL362_SCK_PIN);
  Adxl.begin(ADXL362_CS_PIN);
  // Setup thresholds
  Adxl.setupACInactivityInterrupt(INACTIVITY_G, INACTIVITY_TIME);
  Adxl.setupACActivityInterrupt(ACTIVITY_G, ACTIVITY_TIME);
  // Turn on Loop Mode
  temp = Adxl.SPIreadOneRegister(0x27); // Read current reg value
  temp = temp | (0x30);                 // turn on Loop bits  
  Adxl.SPIwriteOneRegister(0x27, temp); // Write new reg value 
  // Map INT2 to AWAKE bit
  temp = Adxl.SPIreadOneRegister(0x2B); // Read current reg value
  temp = temp | (0x40);                 // Map INT to AWAKE
  Adxl.SPIwriteOneRegister(0x2B, temp); // Write new reg value 
  
  // After setting thresholds, start the measurement mode
  Adxl.beginMeasure();
  
  // Initialize the AP2 interface pins
#ifdef AP2_BR1_PIN
  pinMode(AP2_BR1_PIN, OUTPUT);
  pinMode(AP2_BR2_PIN, OUTPUT);
  pinMode(AP2_BR3_PIN, OUTPUT);
#endif
  digitalWrite(AP2_RST_PIN, 1);
  pinMode(AP2_RST_PIN, OUTPUT);
  pinMode(AP2_SUSPEND_PIN, OUTPUT);
  pinMode(AP2_SLEEP_PIN, OUTPUT);
  pinMode(AP2_RTS_PIN, INPUT);

  // Baudrate selection to 38400 baud
#ifdef AP2_BR1_PIN
  digitalWrite(AP2_BR1_PIN, 1);
  digitalWrite(AP2_BR2_PIN, 0);
  digitalWrite(AP2_BR3_PIN, 0);
#endif

  digitalWrite(AP2_RST_PIN, 1);
  digitalWrite(AP2_SLEEP_PIN, 0);
  digitalWrite(AP2_SUSPEND_PIN, 1);

  // Initalize Serial1 or SoftwareSerial
  AntSerial.begin(38400);
  // Stabilize for a while
  delay(10);
  // Issue an AP2 hard reset
  AP2reset();
  // Setup and initialize the ANT network
  antSetup();

  // Set the AD resolution to 14 bits
  analogReadResolution(ANA_RES_BITS);

  // Create readSensors task using Scheduler to run in 'parallel' with main loop()
  SensorTimer.begin(readSensorsTask, (SENSOR_TICK_MILLIS * 1000));
}


// *****************************************************************************
// Main process loop task
// *****************************************************************************
void loop()
{
byte  i;
long  forceBufAvg;
long  forceBufMax;
uint32_t startMillis;
word     execMillis;
long tempX, tempY, tempZ;

   
  // Main loop for serial output and debugging tasks:

  // Setup and maintain loop timing and counter, blinky
  startMillis = millis();
  MainCount++;
  // Toggle Run Led every 500 msec on, will be turned off by sensorstask
  if ((MainCount % MSEC_TO_TICKS(500)) == 0)
    digitalWrite(LED_BUILTIN, HIGH);

  // Maintain Total Seconds counter
  if ((MainCount % MSEC_TO_TICKS(1000)) == 0)
    TotalSeconds++;

  // Calculate moving average of Force buffer sensor values
  forceBufAvg = 0;
  forceBufMax = 0;
  for (i = 0; i < HX711_BUF_SIZE; i++)
  {
    forceBufAvg += Hx711Buf [i];
    if (Hx711Buf [i] > forceBufMax)
      forceBufMax = Hx711Buf [i];
  }
  forceBufAvg = forceBufAvg / HX711_BUF_SIZE;
  // Calculate Force, Torque and eventually Power
  Force = ((float) forceBufAvg - (float) hx711.get_offset()) * (float) hx711.get_scale();
  Torque = (Force * (float) ForceDistance) / (float) 1000;
  Power = ((float) 0.105 * (float) Cadence * Torque) / (float) 10;
  
  // Output serial data for debugging
#if DEBUG_MAIN_PROCESS == 1
  //sprintf(TmpBuf, "Millis since start: %ul msec", millis());
  //sprintf(TmpBuf, "LC: %ul", forceBufAvg);
  //Serial.println(TmpBuf);
  //Serial.println(Hx711SensorVal & 0x007FFFFF);
  //noInterrupts();
  //tempX = XValue; tempY = YValue; tempZ = ZValue;
  //interrupts();
  switch (CrankState)
  {
    case CRANK_UP: Serial.print ("CRANK_UP: "); break;
    case CRANK_FORWARD: Serial.print ("CRANK_FWD: "); break;
    case CRANK_DOWN: Serial.print ("CRANK_DN: "); break;
    case CRANK_BACK: Serial.print ("CRANK_BWD: "); break;
    case CRANK_UNKNOWN: Serial.print ("CRANK_UNKNOWN: "); break;
  }
  //Serial.print(CrankState);
  //Serial.print(' ');
  //Serial.print(PrevCrankState);
  Serial.print(' ');
  Serial.println(Cadence);
  pinMode (20, INPUT);
  Serial.println (digitalRead(20));
#endif

  // Broadcast ANT data messages @ 4 Hz update intervals
  if ((MainCount % MSEC_TO_TICKS(250)) == 1)
  {
    // Check for an ANT Calibration request
    if (AntCalibration)
    {
#if ANT_SIMULATION == 1
      broadcastCalibration(true, Hx711SensorVal + random (0, 100));
#else
      // Tare offset the strain gauge values with averaged value
      hx711.set_offset(forceBufAvg);
      hx711.set_scale((float) FORCE_SCALE_FACTOR);
      broadcastCalibration(true, (word) (hx711.get_offset() >> 4));
      
      // Finally save the configuration to the EEPROM
      eepromSetConfig();
#endif
      AntCalibration = false;
    }
    // Interleave Power broadcast every 4 messages
    else if ((SendCount % 5) != 4)
    {
      AntPedalPower = 50;
      AntCadence = (byte) Cadence;
#if ANT_SIMULATION == 1
      AntPower = 300 + random (-25, 100);
#else
      AntPower = (word) (((float) forceBufAvg - (float) hx711.get_offset()) * (float) hx711.get_scale());
#endif
      AntAccuPower += AntPower;
      broadcastBikePower();
      UpdateEventCount++;
#if DEBUG_MAIN_PROCESS == 1
      //sprintf(TmpBuf, "Pdl %u; Cdn %u; TotPwr %u Pwr: %u", AntPedalPower, AntCadence, AntAccuPower, AntPower);
      //Serial.println(TmpBuf);
#endif
    }
    // Interleave every 5th message with either Manufacturer Info and Product Info
    // And Battery Info every 15 seconds
    else
    {
      // Battery Info every 15 seconds
      if ((TotalSeconds % 15) == 0)
        broadcastBatteryInfo();
      // Every other 4 interleaves Manufacturer and Product Info
      else if ((SendCount % 2) == 0)
        broadcastMfg();
      else
        broadcastProduct();
    }
    // Update the TX send/interleave counter
    SendCount++;
  }

  // Run ANT RX parser check with no timeout for incoming requests
  ANTreceive(0);

  // Run this task at 20 Hz
  execMillis = (word) (millis() - startMillis);
  if (execMillis < MAIN_LOOP_MILLIS)
    delay (MAIN_LOOP_MILLIS - execMillis);
}


// *****************************************************************************
// Sensors Input Task samples at 10 msec
// *****************************************************************************
static void readSensorsTask()
{
  // Led off, to keep current usage low
  digitalWrite(LED_BUILTIN, LOW);

  // Read the value from the HX711 sensor into the sample buffer
  if (hx711.is_ready())
  {
    Hx711SensorVal = hx711.read();
    Hx711Buf [Hx711Index] = Hx711SensorVal & 0x007FFFFF;
    Hx711Index++;
    if (Hx711Index >= HX711_BUF_SIZE)
      Hx711Index = 0;
  }

  // Read all three acceleration axes in burst to ensure all measurements correspond to same sample time
  Adxl.readXYZTData(XValue, YValue, ZValue, Temperature);  
  // Calculate the Temperature in XX.XXX
  RealTemperature = ((long) Temperature - (long) TempOffset) * (long) 65;
  
  // Perform the X/Y state machine to determine Crank Speed/Cadence
  if (XValue > 900 && XValue < 1100 && YValue > -100 && YValue < 100)
    CrankState = CRANK_UP;
  else if (XValue > -100 && XValue < 100 && YValue > 900 && YValue < 1100)
    CrankState = CRANK_FORWARD;
  else if (XValue > -1100 && XValue < -900 && YValue > -100 && YValue < 100)
    CrankState = CRANK_DOWN;
  else if (XValue > -100 && XValue < 100 && YValue > -1100 && YValue < -900)
    CrankState = CRANK_BACK;
  
  // Maintain Crank timing
  CrankTicks++;
  // Next check on state change to UP  
  // Or reverse pedaling....
  if ((PrevCrankState == CRANK_BACK && CrankState == CRANK_UP) ||
      (PrevCrankState == CRANK_FORWARD && CrankState == CRANK_UP))
  {
    CrankTime = CrankTicks;
    CrankTicks = 0;
    // Calculate Cadence in ###.#
    Cadence = (word) 60000 / CrankTime;   
  }
  // Store last CrankState for next time
  PrevCrankState = CrankState;
  
  // Battery monitoring
  BatteryVoltage = (word) (((uint32_t) ANA_VOLT_REF * (uint32_t) analogRead (ANA_BATT_VOLT_CH)) / (uint32_t) ANA_INP_RANGE);
  if (BatteryVoltage < 280)
    BatteryStatus = BATTERY_CRITICAL;
  else if (BatteryVoltage < 290)
    BatteryStatus = BATTERY_LOW;
  else if (BatteryVoltage < 305)
    BatteryStatus = BATTERY_OK;
  else if (BatteryVoltage < 310)
    BatteryStatus = BATTERY_GOOD;
  else
    BatteryStatus = BATTERY_NEW;
}


// *****************************************************************************
// EEPROM routines to read/write the configuration to/from non volatile memory
// *****************************************************************************
static void eepromGetConfig()
{
int  i;
long templ;

  // Check for EEPROM valid data configuration
  if (EEPROM[0] == 0x55 && EEPROM[1] == 0xAA)
  {
    // Read configuration variables out of EEPROM
    templ = ((long) EEPROM[EE_STRAIN_OFFSET + 0] << 24 |
             (long) EEPROM[EE_STRAIN_OFFSET + 1] << 16 |
             (long) EEPROM[EE_STRAIN_OFFSET + 2] << 8  |
             (long) EEPROM[EE_STRAIN_OFFSET + 3]);
    hx711.set_offset(templ);
    TempOffset = ((long) EEPROM[EE_TEMP_OFFSET + 0] << 8  |
                  (long) EEPROM[EE_TEMP_OFFSET + 1]);

  } 
  else
  {
    // EEPROM first time initialization
    EEPROM[0] = 0x55;
    EEPROM[1] = 0xAA;
    // Clear all contents to 0
    for (i = EE_START_ADDR; i < EEPROM.length(); i++)
      EEPROM.write(i, 0x00);
  }
}

// *****************************************************************************
// EEPROM routines to read/write the configuration to/from non volatile memory
// *****************************************************************************
static void eepromSetConfig()
{
long temp;

  // Write configuration variables to the EEPROM
  temp = hx711.get_offset();
  EEPROM[EE_STRAIN_OFFSET + 0] = (byte) ((temp >> 24) & 0x000000FF);
  EEPROM[EE_STRAIN_OFFSET + 1] = (byte) ((temp >> 16) & 0x000000FF);
  EEPROM[EE_STRAIN_OFFSET + 2] = (byte) ((temp >> 8) & 0x000000FF);
  EEPROM[EE_STRAIN_OFFSET + 3] = (byte) (temp & 0x000000FF);

  EEPROM[EE_TEMP_OFFSET + 0] = (byte) ((TempOffset >> 8) & 0x000000FF);
  EEPROM[EE_TEMP_OFFSET + 1] = (byte) (TempOffset & 0x000000FF);
}


// *****************************************************************************
// ANT stack initialization and network setup
// *****************************************************************************
static void antSetup()
{
  // Flush any spurious received characters
  AntSerial.flush();

  // Send reset to ANT Network
  ANTreset();
  // Delay after resetting the radio (spec: 500 msec)
  delay (750);
  ANTreceive(ANT_RX_MAX_TIMEOUT);

  // Set the Network Key (currently to DEVELOPER)
  ANTsetNetwork();
  ANTreceive(ANT_RX_MAX_TIMEOUT);

  // Assign ANT Channel as bi-directional transmitter (Master)
  ANTassignChannel(0, CHANNEL_TYPE_BIDIRECTIONAL_TRANSMIT);
  ANTreceive(ANT_RX_MAX_TIMEOUT);

  // Set Channel ID to Bike Power profile
  ANTsetChannelId(ANT_CHANNEL_NR, (word) ANT_DEVICE_NR, BIKE_POWER_PROFILE, BIKE_POWER_TT);
  ANTreceive(ANT_RX_MAX_TIMEOUT);

  // Set frequency for profile to 4Hz updates
  ANTsetFrequency(ANT_CHANNEL_NR, BIKE_POWER_RF);
  ANTreceive(ANT_RX_MAX_TIMEOUT);

  // Set ANT period
  ANTsetPeriod(ANT_CHANNEL_NR, BIKE_POWER_CP);
  ANTreceive(ANT_RX_MAX_TIMEOUT);

  // Open the ANT channel for communication broadcasting
  ANTopenChannel(ANT_CHANNEL_NR);
  ANTreceive(ANT_RX_MAX_TIMEOUT);
}

// *****************************************************************************
// ANT Broadcast messages; Bike Power (Required)
// *****************************************************************************
static void broadcastBikePower (void)
{
uint8_t buf[13];

  // Fill ANT Bike Power data buffer with data
  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x09;
  buf[2] = MSG_BROADCAST_DATA_ID;
  buf[3] = ANT_CHANNEL_NR;
  buf[4] = DP_STANDARD_POWER_ONLY;
  buf[5] = UpdateEventCount;
  buf[6] = AntPedalPower;
  buf[7] = AntCadence;
  buf[8] = lowByte (AntAccuPower);
  buf[9] = highByte (AntAccuPower);
  buf[10] = lowByte (AntPower);
  buf[11] = highByte (AntPower);
  buf[12] = checkSum(buf, 12);
  ANTsend(buf, 13);

  ANTreceive(ANT_RX_MAX_TIMEOUT);
}

// *****************************************************************************
// ANT Broadcast messages; Calibration
// *****************************************************************************
static void broadcastCalibration (boolean success, word calibrationVal)
{
uint8_t buf[13];

  // Fill ANT Calibration data buffer with data
  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x09;
  buf[2] = MSG_BROADCAST_DATA_ID;
  buf[3] = ANT_CHANNEL_NR;
  buf[4] = DP_CALIBRATION_REQ;
  if (success)
    buf[5] = 0xAC;
  else
    buf[5] = 0xAF;
  buf[6] = 0xFF;
  buf[7] = 0xFF;
  buf[8] = 0xFF;
  buf[9] = 0xFF;
  buf[10] = lowByte (calibrationVal);
  buf[11] = highByte (calibrationVal);
  buf[12] = checkSum(buf, 12);
  ANTsend(buf, 13);

  ANTreceive(ANT_RX_MAX_TIMEOUT);
}

// *****************************************************************************
// ANT Broadcast messages; Manufacturer (Required)
// *****************************************************************************
static void broadcastMfg (void)
{
uint8_t buf[13];

  // Fill ANT Manufacturer data buffer with data
  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x09;
  buf[2] = MSG_BROADCAST_DATA_ID;
  buf[3] = ANT_CHANNEL_NR;
  buf[4] = DP_MANUFACTURER_INFO;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = HARDWARE_REVISION;
  buf[8] = lowByte (MANUFACTURER_ID);
  buf[9] = highByte (MANUFACTURER_ID);
  buf[10] = lowByte (MODEL_NUMBER);
  buf[11] = highByte (MODEL_NUMBER);
  buf[12] = checkSum(buf, 12);
  ANTsend(buf, 13);

  ANTreceive(ANT_RX_MAX_TIMEOUT);
}

// *****************************************************************************
// ANT Broadcast messages; Product Info (Required)
// *****************************************************************************
static void broadcastProduct (void)
{
uint8_t buf[13];

  // Fill ANT Product info data buffer with data
  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x09;
  buf[2] = MSG_BROADCAST_DATA_ID;
  buf[3] = ANT_CHANNEL_NR;
  buf[4] = DP_PRODUCT_INFO;
  buf[5] = 0xFF;
  buf[6] = SW_SUB_REVISION;
  buf[7] = SW_MAIN_REVISION;
  buf[8] = 0xFF;
  buf[9] = 0xFF;
  buf[10] = 0xFF;
  buf[11] = 0xFF;
  buf[12] = checkSum(buf, 12);
  ANTsend(buf, 13);

  ANTreceive(ANT_RX_MAX_TIMEOUT);
}

// *****************************************************************************
// ANT Broadcast messages; Battery Info (Optional)
// *****************************************************************************
static void broadcastBatteryInfo (void)
{
uint8_t buf[13];
long ticks;

  // Fill ANT Battery info data buffer with data
  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x09;
  buf[2] = MSG_BROADCAST_DATA_ID;
  buf[3] = ANT_CHANNEL_NR;
  buf[4] = DP_BATTERY_INFO;
  buf[5] = 0xFF;
  buf[6] = 0xFF;                  // Battery identifier not used
  ticks = TotalSeconds / 2;       // Runtime in 2 seconds units
  buf[7] = (byte) (ticks & 0x000000FF);
  ticks = ticks >> 8;
  buf[8] = (byte) (ticks & 0x000000FF);
  ticks = ticks >> 8;
  buf[9] = (byte) (ticks & 0x000000FF);
  buf[10] = BatteryVoltage % 100;
  buf[11] = 0x80 | (BatteryStatus << 4) | (BatteryVoltage / 100);
  buf[12] = checkSum(buf, 12);
  ANTsend (buf, 13);

  ANTreceive (ANT_RX_MAX_TIMEOUT);
}

// *****************************************************************************
// ANT AP2 hard reset function
// *****************************************************************************
static void AP2reset (void)
{
  // Issue and AP2 hard reset
  digitalWrite(AP2_RST_PIN, 0);
  delayMicroseconds(100);
  digitalWrite(AP2_RST_PIN, 1);
}

// *****************************************************************************
// ANT AP2 check/wait for RTS line low
// *****************************************************************************
static bool AP2waitCts (void)
{
byte retry = 0;

  // Next send is only allowed when RTS pin is low
  while (digitalRead(AP2_RTS_PIN) && retry < 5)
  {
    delayMicroseconds(50);
    retry++;
  }
  return (retry < 5);
}

// *****************************************************************************
// ANT AP2 receive routine(s); ANTreceive
// *****************************************************************************
static boolean ANTreceive (word timeout)
{
int sbuflength = 0;
uint8_t msg = 0;
uint32_t retry = 0;

  // Wait for start of message reception, timeout in milliseconds
  if (timeout == 0)
    timeout = 1;
  retry = (uint32_t) (timeout * 10);
  while (sbuflength == 0 && retry > 0)
  {
    sbuflength = AntSerial.available();
    if (sbuflength == 0)
      delayMicroseconds (100);
    retry--;
  }

  while (sbuflength > 0 && MsgIndex < ANT_RXBUF_SIZE)
  {
#if DEBUG_ANT_RX == 1
    //Serial.print("sbuf: ");
    //Serial.print(sbuflength);
    //Serial.print(" msg: ");
    //Serial.print (AntSerial.peek(), HEX);
#endif
    if (!MsgSync)
    {
      msg = AntSerial.read();
      if (msg == MSG_TX_SYNC)
      {
        MsgIndex = 0;
        MsgBuf[MsgIndex] = MSG_TX_SYNC;
        MsgSync = true;
      }
    }
    else if (MsgSync && MsgIndex == 0)
    {
      // Read length of message and add 3 for total (sync, length, id, checksum)
      MsgLength = AntSerial.read();
      MsgIndex++;
      MsgBuf[MsgIndex] = MsgLength;
      MsgLength += 4;
    }
    else if (MsgSync && MsgIndex > 0 && MsgIndex < MsgLength)
    {
      MsgIndex++;
      MsgBuf[MsgIndex] = AntSerial.read();
      if (MsgIndex == MsgLength - 1)
      {
#if DEBUG_ANT_RX == 1
        for (int i = 0 ; i < MsgLength ; i++)
        {
          Serial.print(MsgBuf[i], HEX);
          Serial.print(" ");
        }
        Serial.println(" ");
#endif
        // Call the message parser for further action
        ANTrxProcess ();
        MsgSync = false;
      }
    }

    // Wait for next character to parse (can be too fast), max 2 ms (slowest ~4800 baud)
    if (AntSerial.available() == 0)
      delayMicroseconds(2000);

    sbuflength = AntSerial.available();
  }


  // Reset other used variables for next message
  MsgSync = false;
  MsgIndex = 0;
  MsgLength = 0;

  return (retry > 0);
}

// *****************************************************************************
// ANT AP2 receive routine(s); ANTrxProcess
/// *****************************************************************************
static void ANTrxProcess (void)
{
  // Parse the Channel response messages
  if (MsgBuf[2] == MSG_CHANNEL_RESPONSE)
  {
    // An RF Event is code 1
    if (MsgBuf[4] == 0x01)
    {
      if (MsgBuf[5] == MSG_TX_EVENT)
      {
        // Toggle the communication LED on active transmissions
        digitalWrite(LED_BUILTIN, !digitalRead (LED_BUILTIN));
#if DEBUG_ANT_RX == 1
        Serial.println("TX success");
#endif
      }
    }
    else
    {
#if DEBUG_ANT_RX == 1
      switch (MsgBuf[4])
      {
        case MSG_NETWORK_KEY_ID:
          Serial.print("Set Network Key: ");
          break;

        case MSG_ASSIGN_CHANNEL_ID:
          Serial.print("Assign Channel: ");
          break;

        case MSG_CHANNEL_ID_ID:
          Serial.print("Set Channel ID: ");
          break;

        case MSG_CHANNEL_RADIO_FREQ_ID:
          Serial.print("Set Frequency: ");
          break;

        case MSG_CHANNEL_MESG_PERIOD_ID:
          Serial.print("Set Period: ");
          break;

        case MSG_RADIO_TX_POWER_ID:
          Serial.print("Set Tx Power: ");
          break;

        case MSG_CHANNEL_SEARCH_TIMEOUT_ID:
          Serial.print("Set Timout: ");
          break;

        case MSG_OPEN_CHANNEL_ID:
          Serial.print("Open Channel: ");
          break;

        default:
          Serial.print("Unknown Msg: ");
          break;
      }

      if (MsgBuf[5] == 0)
        Serial.println("No Error");
      else
      {
        Serial.print("ERR: ");
        Serial.println(MsgBuf[5], HEX);
      }
#endif
    }
  }
  else if (MsgBuf[2] == MSG_ACKNOWLEDGE_DATA_ID)
  {
    // Check for Calibration Message requests
    if (MsgBuf[5] == MSG_CALIBRATION_REQUEST)
    {
      AntCalibration = true;
#if DEBUG_ANT_RX == 1
      Serial.println("Calibration Request");
#endif
    }
  }
}

// *****************************************************************************
// ANT AP2 send routine(s)
// *****************************************************************************
static void ANTsend (uint8_t buf[], int length)
{
int i;

#if DEBUG_ANT_TX == 1
  Serial.print("ANTsend:");
#endif
  // Send length buffer bytes to AntSerial
  for (i = 0 ; i < length ; i++)
  {
#if DEBUG_ANT_TX == 1
    Serial.print(buf[i], HEX);
    Serial.print(" ");
#endif
    AntSerial.write(buf[i]);
  }
  // Wait for Clear to Send signal
  AP2waitCts ();
#if DEBUG_ANT_TX == 1
  Serial.println("");
#endif
}

// *****************************************************************************
// ANT message checksum calculation
// *****************************************************************************
static byte checkSum (byte *dataPtr, int length)
{
int i;
byte chksum;

  chksum = dataPtr[0];
  for (i = 1; i < length; i++)
    chksum ^= dataPtr[i];     // +1 since skip prefix sync code, we already counted it
  return (chksum);
}

// *****************************************************************************
// ANT setup routine(s); ANTreset
// *****************************************************************************
static void ANTreset (void)
{
uint8_t buf[5];

  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x01;
  buf[2] = MSG_SYSTEM_RESET_ID;
  buf[3] = 0x00;
  buf[4] = checkSum (buf, 4);
  ANTsend(buf, 5);
}

// *****************************************************************************
// ANT setup routine(s); ANTsetNetwork
// *****************************************************************************
static void ANTsetNetwork (void)
{
uint8_t buf[13];
int i;

  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x09;
  buf[2] = MSG_NETWORK_KEY_ID;
  buf[3] = 0x00;
  for (i = 0; i < 8; i++)
    buf [4 + i] = NETWORK_KEY[i];
  buf[12] = checkSum(buf, 12);
  ANTsend(buf, 13);
}

// *****************************************************************************
// ANT setup routine(s); ANTassignChannel
// *****************************************************************************
static void ANTassignChannel (byte chNr, byte chType)
{
uint8_t buf[7];

  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x03;
  buf[2] = MSG_ASSIGN_CHANNEL_ID;
  buf[3] = chNr;
  buf[4] = chType;
  buf[5] = 0;
  buf[6] = checkSum(buf, 6);
  ANTsend(buf, 7);
}

// *****************************************************************************
// ANT setup routine(s); ANTsetChannelId
// *****************************************************************************
static void ANTsetChannelId (byte chNr, word deviceNr, byte deviceId, byte deviceTt)
{
uint8_t buf[9];

  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x05;
  buf[2] = MSG_CHANNEL_ID_ID;
  buf[3] = chNr;
  buf[4] = lowByte (deviceNr);
  buf[5] = highByte (deviceNr);
  buf[6] = deviceId;
  buf[7] = deviceTt;
  buf[8] = checkSum(buf, 8);
  ANTsend(buf, 9);
}

// *****************************************************************************
// ANT setup routine(s); ANTsetFrequency
// *****************************************************************************
static void ANTsetFrequency (byte chNr, byte freq)
{
uint8_t buf[6];

  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x02;
  buf[2] = MSG_CHANNEL_RADIO_FREQ_ID;
  buf[3] = chNr;
  buf[4] = freq;
  buf[5] = checkSum(buf, 5);
  ANTsend(buf, 6);
}

// *****************************************************************************
// ANT setup routine(s); ANTsetPeriod
// *****************************************************************************
static void ANTsetPeriod (byte chNr, word period)
{
uint8_t buf[7];

  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x03;
  buf[2] = MSG_CHANNEL_MESG_PERIOD_ID;
  buf[3] = chNr;
  buf[4] = lowByte (period);
  buf[5] = highByte (period);
  buf[6] = checkSum(buf, 6);
  ANTsend(buf, 7);
}

// *****************************************************************************
// ANT setup routine(s); ANTopenChannel
// *****************************************************************************
static void ANTopenChannel (byte chNr)
{
uint8_t buf[5];

  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x01;
  buf[2] = MSG_OPEN_CHANNEL_ID;
  buf[3] = chNr;
  buf[4] = checkSum(buf, 4);
  ANTsend(buf, 5);
}
