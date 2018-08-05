// *****************************************************************************
// INCLUDES Section
// *****************************************************************************
#include <stdio.h>
#include "src\hx711.h"
#include "src\adxl362.h"

#include <EEPROM.h>
#include <IntervalTimer.h>
#include <Snooze.h>
#include <SPI.h>

// *****************************************************************************
// DEFINITIONS Section
// *****************************************************************************
// Main Loop timing definitions
#define MAIN_LOOP_MILLIS    250
#define HX711_POWER_UP      (250-60)
#define MAIN_TICKS_PER_SEC  (1000/MAIN_LOOP_MILLIS)
#define MSEC_TO_TICKS(x)    (uint16_t) (((uint32_t) (x)*(uint32_t) MAIN_TICKS_PER_SEC)/(uint32_t) 1000)

#define MAIN_LOOP_DELAY     0
#define MAIN_LOOP_WFI       1
#define MAIN_LOOP_SLEEP     2
#define MAIN_LOOP_DEEPSLEEP 3
#define MAIN_LOOP_HIBERNATE 4
#define MAIN_LOOP_VLPR      5
#define MAIN_LOOP_SETPOWER  MAIN_LOOP_WFI

// Sensors Task timing definitions
#define SENSOR_TASK_MILLIS  250
#define SENS_TICKS_PER_SEC  (1000/SENSOR_TASK_MILLIS)
#define MSEC_TO_SENSTICKS(x)(uint16_t) (((uint32_t) (x)*(uint32_t) SENS_TICKS_PER_SEC)/(uint32_t) 1000)

// Cadence definitions
#define CADENCE_ADXL        0
#define CADENCE_HALL        1
#define CADENCE_CALC        CADENCE_HALL
// Cadence timing in milliseconds: Between 12 and 254
#define CADENCE_TIMEOUT     5000
#define MIN_CADENCE_TIME    236

// Debug settings definitions
#define DEBUG               1
#if DEBUG == 1
#define DEBUG_MAIN_PROCESS  1
#define DEBUG_ANT_TX        0
#define DEBUG_ANT_RX        0
#endif
#define ANT_SIMULATION      0

// Hardware and Product definitions
#define HARDWARE_REVISION   ((uint8_t)  0x01)
#define MANUFACTURER_ID     ((uint16_t) 0x00FF)
#define MODEL_NUMBER        ((uint16_t) 3555)
#define SW_MAIN_REVISION    1
#define SW_SUB_REVISION     2

// Unused controller pins
#define UNUSED1_PIN         (4)
#define UNUSED2_PIN         (5)
#define UNUSED3_PIN         (6)
#define UNUSED4_PIN         (7)
#define UNUSED5_PIN         (8)
#define UNUSED6_PIN         (20)
#define UNUSED7_PIN         (21)
#define UNUSED8_PIN         (22)
//#define UNUSED9_PIN         (23)

// Sensor and input definitions
// Strain Gauge interface
#define HX711_FILTER        100     // new value for 50 %
#define HX711_DATA_PIN      (3)
#define HX711_CLOCK_PIN     (2)
#define FORCE_SCALE_FACTOR  600     // In HX711 counts
#define DEFAULT_FORCE_DIST  100     // in mm
#define POWER_BUF_SIZE      2*SENS_TICKS_PER_SEC

// ADXL362 interface
#define DEFAULT_TEMP_OFFSET -77     // in counts
#define ACTIVITY_G          500     // in milli G
#define ACTIVITY_TIME       50      // in 100 Hz ticks: 0.5 sec
#define INACTIVITY_G        50      // in milli G
#define INACTIVITY_TIME     1000    // in 100 Hz ticks: 10 sec
#define XYAXIS_FILTER       100     // new value for XX %
#define XYZERO_FILTER       1       // new value for XX %
#define MAX_CADENCE         254
#define CDN_BUF_SIZE        (2*3)   // 2 x 0-crossings @ 3 revs
#define USE_ADXL_AXIS       0       // 0=X, 1=Y, 2=Z
#define ADXL_BUF_SIZE       2*SENS_TICKS_PER_SEC

#define ADXL362_CS_PIN      (15)
#define ADXL362_SCK_PIN     (14)

// Hall Sensor definitions
#define CADENCE_INT_PIN     (23)

// Analog inputs interface/sensors
#define ANA_RES_BITS        12
#define ANA_INP_RANGE       4096
#define ANA_BATT_VOLT_CH    (39)    // Use internal ref bandgap 'trick'
#define BATTERY_NEW         1
#define BATTERY_GOOD        2
#define BATTERY_OK          3
#define BATTERY_LOW         4
#define BATTERY_CRITICAL    5

// EEPROM address definitions
#define EE_START_ADDR       2
#define EE_STRAIN_OFFSET    EE_START_ADDR + 0
#define EE_TEMP_OFFSET      EE_START_ADDR + 4

// ANT AP2 Interface definitions
#define AP2_TX_PIN          (10)
#define AP2_RX_PIN          (9)
#define AP2_RST_PIN         (17)
#define AP2_SUSPEND_PIN     (18)
#define AP2_SLEEP_PIN       (19)
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

#define ANT_TX_POWER_HIGH   3
#define ANT_TX_POWER_MEDIUM 2
#define ANT_TX_POWER_LOW    1
#define ANT_TX_POWER_LOWEST 0

// ANT Protocol Message definitions
#define ANT_RXBUF_SIZE                40
#define ANT_RX_SETUP_TIMEOUT          10 // In milliseconds (was 50 initially)
#define ANT_RX_WAIT_TIME              0

// ANT Transmit Message & Msg ID definitions
#define MSG_TX_SYNC                   ((uint8_t) 0xA4)
#define MSG_SYSTEM_RESET_ID           ((uint8_t) 0x4A)
#define MSG_NETWORK_KEY_ID            ((uint8_t) 0x46)
#define MSG_ASSIGN_CHANNEL_ID         ((uint8_t) 0x42)
#define MSG_CHANNEL_ID_ID             ((uint8_t) 0x51)
#define MSG_CHANNEL_RADIO_FREQ_ID     ((uint8_t) 0x45)
#define MSG_CHANNEL_MESG_PERIOD_ID    ((uint8_t) 0x43)
#define MSG_RADIO_TX_POWER_ID         ((uint8_t) 0x47)
#define MSG_CHANNEL_SEARCH_TIMEOUT_ID ((uint8_t) 0x44)
#define MSG_OPEN_CHANNEL_ID           ((uint8_t) 0x4B)
#define MSG_BROADCAST_DATA_ID         ((uint8_t) 0x4E)
#define MSG_ACKNOWLEDGE_DATA_ID       ((uint8_t) 0x4F)

#define MSG_CHANNEL_RESPONSE          ((uint8_t) 0x40)
#define MSG_TX_EVENT                  ((uint8_t) 0x03)
#define MSG_CALIBRATION_REQUEST       ((uint8_t) 0xAA)

// Define Data Pages
#define DP_CALIBRATION_REQ            ((uint8_t) 0x01)
#define DP_STANDARD_POWER_ONLY        ((uint8_t) 0x10)
#define DP_MANUFACTURER_INFO          ((uint8_t) 0x50)
#define DP_PRODUCT_INFO               ((uint8_t) 0x51)
#define DP_BATTERY_INFO               ((uint8_t) 0x52)

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
static uint16_t MainCount = 0;
static uint16_t BatteryVoltage = 0;
static uint8_t  BatteryStatus;
static uint32_t TotalSeconds = 0;

static IntervalTimer SensorsTimer;

SnoozeUSBSerial  Usb;
SnoozeTimer      TimerS;
// configures the lc's 5v data buffer (OUTPUT, LOW) for low power
Snoozelc5vBuffer LC5vBuffer;
SnoozeBlock      Config_teensyLC(TimerS, Usb, LC5vBuffer);
SnoozeBlock      DummyConfig;

// Bike Power processing variables
static uint16_t CrankDeg = 0;
static volatile uint16_t CrankTicks = 0;
static volatile uint16_t CrankTime = 0;
static volatile uint8_t Cadence = 0;
static volatile uint8_t PrevCadence = 0;
static uint16_t CadenceTotal = 0;
static uint8_t  CadenceBuf [CDN_BUF_SIZE];
static uint8_t  CadenceBufIndex = 0;

static volatile uint16_t CadenceCount = 0;
static volatile uint32_t CadenceMillis = 0;
static volatile uint32_t CadenceMillisPrev = 0;
static volatile boolean  CadenceReady = false;

static uint16_t Force;
static uint16_t Torque;
static uint16_t Power;
static uint16_t ForceDistance = DEFAULT_FORCE_DIST;
static uint16_t PowerBuf [POWER_BUF_SIZE];
static uint8_t  PowerBufIndex = 0;

// Loadcell, Strain Gauge Amplifier variables
const uint8_t  hx711_data_pin = HX711_DATA_PIN;
const uint8_t  hx711_clock_pin = HX711_CLOCK_PIN;
static int32_t Hx711SensorVal = 0;
static int32_t Hx711SensorFilt = 0;
static int32_t Hx711SensorOffset = 0;
// Construct the Loadcell Amplifier object
static HX711 hx711 = HX711(hx711_data_pin, hx711_clock_pin, 128);

// ADXL362 interface variables
static ADXL362  Adxl;
static int16_t  AdxlValue;
static int16_t  AdxlValueFilt;
static int16_t  AdxlValuePrev;
static int16_t  AdxlBuf [ADXL_BUF_SIZE];
static int16_t  AdxlBufIndex = 0;
static int16_t  AdxlValueZero = 0;
static int32_t  AdxlTotal;
static int16_t  AdxlTemperature;
static int32_t  AdxlRealTemperature;
static uint16_t AdxlTempOffset = DEFAULT_TEMP_OFFSET;

// ANT communication data
#define AntSerial Serial2

// ANT Developer key, if you want to connect to ANT+, you must get the key from thisisant.com
static const uint8_t NETWORK_KEY[] = {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45};

// ANT TX messages processor variables
static uint8_t  AntPedalPower;
static uint8_t  AntCadence;
static uint16_t AntAccuPower;
static uint16_t AntPower;
static uint8_t  SendCount = 0;
static uint8_t  UpdateEventCount = 0;

// ANT RX messages processor variables
static boolean  MsgSync = false;
static uint8_t  MsgIndex = 0;
static uint8_t  MsgLength = 0;
static uint8_t  MsgBuf [ANT_RXBUF_SIZE];
static boolean  AntCalibration = false;
static boolean  AntTxSuccess = false;
static boolean  AntTxSetupOK = false;

// *****************************************************************************
// Local Function prototypes
// *****************************************************************************
static void handleDelayScenario(uint32_t startMillis);
static void readSensorsTask();
static void cadenceTimerTask();
static void cadenceInterrupt();

static uint32_t getInputVoltage();
static void determineBatteryStatus();

static void eepromGetConfig();
static void eepromSetConfig();

static void broadcastBikePower (void);
static void broadcastCalibration (boolean success, uint16_t calibrationVal);
static void broadcastMfg (void);
static void broadcastProduct (void);
static void broadcastBatteryInfo (void);

static void antSetup();

static void AP2reset (void);
static bool AP2waitCts (void);

static boolean ANTreceive (uint16_t timeout);
static void    ANTrxProcess (void);

static void ANTsend (uint8_t buf[], int16_t length);
static uint8_t checkSum (uint8_t *dataPtr, int16_t length);

static void ANTreset (void);
static void ANTsetNetwork (void);
static void ANTassignChannel (uint8_t chNr, uint8_t chType);
static void ANTsetChannelId (uint8_t chNr, uint16_t deviceNr, uint8_t deviceId, uint8_t deviceTt);
static void ANTsetFrequency (uint8_t chNr, uint8_t freq);
static void ANTsetPeriod (uint8_t chNr, uint16_t period);
static void ANTsetTxPower (uint8_t power);
static void ANTopenChannel (uint8_t chNr);

// *****************************************************************************
// Initialization and Board setup routines
// *****************************************************************************
void setup()
{
uint8_t temp;

  // Put setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

#if DEBUG_MAIN_PROCESS == 1
  Serial.begin(115200);
  Serial.println("Hello 3sss");
#endif

  // Read the Configuration for the EPROM storage
  eepromGetConfig();
  //*** Only for testing
  AdxlTempOffset = DEFAULT_TEMP_OFFSET;

#if 0
  // All unused pins to output mode (saves power)
  pinMode(UNUSED1_PIN, OUTPUT);
  pinMode(UNUSED2_PIN, OUTPUT);
  pinMode(UNUSED3_PIN, OUTPUT);
  pinMode(UNUSED4_PIN, OUTPUT);
  pinMode(UNUSED5_PIN, OUTPUT);
  pinMode(UNUSED6_PIN, OUTPUT);
  pinMode(UNUSED7_PIN, OUTPUT);
  pinMode(UNUSED8_PIN, OUTPUT);
  pinMode(UNUSED9_PIN, OUTPUT);
  digitalWrite(UNUSED1_PIN, 0);
  digitalWrite(UNUSED2_PIN, 0);
  digitalWrite(UNUSED3_PIN, 0);
  digitalWrite(UNUSED4_PIN, 0);
  digitalWrite(UNUSED5_PIN, 0);
  digitalWrite(UNUSED6_PIN, 0);
  digitalWrite(UNUSED7_PIN, 0);
  digitalWrite(UNUSED8_PIN, 0);
  digitalWrite(UNUSED9_PIN, 0);
#endif

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
  temp = temp | (0x40);                 // Map INT2 to AWAKE
  Adxl.SPIwriteOneRegister(0x2B, temp); // Write new reg value 
  // After setting thresholds, start the measurement mode
  Adxl.beginMeasure();
  // *** Only for testing
  pinMode (20, INPUT);                  // INT2 AWAKE status

  // Configure the Reed sensor to input
  pinMode(CADENCE_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CADENCE_INT_PIN), cadenceInterrupt, FALLING);
  
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

#if DEBUG_MAIN_PROCESS == 1
  Serial.println("Starting ANT");
#endif
  // Initalize ANT HW Serial or SoftwareSerial
  AntSerial.begin(38400);
  // Stabilize for a while
  delay(10);
  // Issue an AP2 hard reset
  AP2reset();
  // Setup and initialize the ANT network
  antSetup();

  // Set the AD resolution as defined
  analogReadResolution(ANA_RES_BITS);
  // 39=bandgap ref (PMC_REGSC |= PMC_REGSC_BGBE), used for Batt Voltage
  PMC_REGSC |= PMC_REGSC_BGBE; 

#if DEBUG_MAIN_PROCESS == 1
  Serial.println("Setup ready");
#endif

  // Start the sensors task @ 50 msec
  //SensorsTimer.begin(readSensorsTask, SENSOR_TASK_MILLIS*1000);
  //SensorsTimer.begin(cadenceTimerTask, 1000);
}

// *****************************************************************************
// Main process loop task
// *****************************************************************************
void loop()
{
int16_t  i;
uint16_t powerBufAvg;
uint16_t powerBufMax;
uint32_t startMillis;

  // Setup and maintain loop timing and counter, blinky
  startMillis = millis();
  MainCount++;
  // Maintain Total Seconds counter and blinky 
  if ((MainCount % MSEC_TO_TICKS(1000)) == 0)
  {
    if (AntTxSuccess)
    {
      // Ant Tx is active; blink Led only every second shortly
      digitalWrite(LED_BUILTIN, HIGH);
      AntTxSuccess = false;
      delayMicroseconds(25);
      // Turn off LED indicator
      digitalWrite(LED_BUILTIN, LOW);
   }
    TotalSeconds++;
  }

  // Read Sensor data and calculations
  readSensorsTask();
  
  // Calculate moving average of Power buffer values
  powerBufAvg = 0;
  powerBufMax = 0;
  for (i = 0; i < POWER_BUF_SIZE; i++)
  {
    powerBufAvg += PowerBuf [i];
    if (PowerBuf [i] > powerBufMax)
      powerBufMax = PowerBuf [i];
  }
  powerBufAvg = powerBufAvg / POWER_BUF_SIZE;

  // Output serial data for debugging
#if DEBUG_MAIN_PROCESS == 1
  //Serial.println(Hx711SensorFilt);
  //Serial.printf("%4u %3u %d %4u ", Force, Torque, Cadence, Power);
#endif
 
  // Broadcast ANT data messages @ 4 Hz update intervals
  if ((MainCount % MSEC_TO_TICKS(250)) == 0)
  {
    // Check for an ANT Calibration request
    if (AntCalibration)
    {
#if ANT_SIMULATION == 1
      broadcastCalibration(true, (uint16_t) (Hx711SensorVal + random (0, 100)));
#else
      // Tare offset the strain gauge values with averaged value
      Hx711SensorOffset = Hx711SensorFilt;
      broadcastCalibration(true, (uint16_t) (Hx711SensorOffset >> 4));
      
      // Finally save the configuration to the EEPROM
      eepromSetConfig();
#endif
      AntCalibration = false;
    }
    // Interleave Power broadcast every 4 messages
    else if ((SendCount % 5) != 4)
    {
      AntPedalPower = 0xFF;
      AntCadence = Cadence;
#if ANT_SIMULATION == 1
      AntPower = 300 + random (-25, 100);
#else
      AntPower = powerBufMax;
#endif
      AntAccuPower += AntPower;
      broadcastBikePower();
      UpdateEventCount++;
    }
    // Interleave every 5th message with either Manufacturer Info and Product Info
    // And Battery Info every 15 seconds
    else
    {
      // Battery Info every 15 seconds
      if ((SendCount % 60) == 4)
      {
        determineBatteryStatus();
        broadcastBatteryInfo();
      }
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
  ANTreceive(ANT_RX_WAIT_TIME);

  // Put ANT into sleep mode
  digitalWrite(AP2_SLEEP_PIN, 1);
  // At last handle the 'sleep' until next main cycle needed
  handleDelayScenario(startMillis);
  // Resume the ANT communication from sleep
  digitalWrite(AP2_SLEEP_PIN, 0);
}

// *****************************************************************************
// Handle best delay scenario for Power setting optimization
// *****************************************************************************
static void handleDelayScenario(uint32_t startMillis)
{
uint16_t execMillis;

  // Determine execution time, and delay for the remainder
  execMillis = (uint16_t) (millis() - startMillis);
  
#if MAIN_LOOP_SETPOWER == MAIN_LOOP_DELAY
  if (execMillis < MAIN_LOOP_MILLIS)
    delay (MAIN_LOOP_MILLIS - execMillis);
#endif
#if MAIN_LOOP_SETPOWER == MAIN_LOOP_WFI
  while ((millis() - startMillis) < MAIN_LOOP_MILLIS)
  {
    __asm__("wfi");
  }
#endif
#if MAIN_LOOP_SETPOWER == MAIN_LOOP_SLEEP
  if (execMillis < MAIN_LOOP_MILLIS)
  {
    TimerS.setTimer(MAIN_LOOP_MILLIS - execMillis);
    Snooze.sleep(Config_teensyLC);
  }
#endif
#if MAIN_LOOP_SETPOWER == MAIN_LOOP_DEEPSLEEP
  if (execMillis < MAIN_LOOP_MILLIS)
  {
    TimerS.setTimer(MAIN_LOOP_MILLIS - execMillis);
    Snooze.deepSleep(Config_teensyLC, LLS);
  }
#endif
#if MAIN_LOOP_SETPOWER == MAIN_LOOP_HIBERNATE
  if (execMillis < MAIN_LOOP_MILLIS)
  {
    TimerS.setTimer(MAIN_LOOP_MILLIS - execMillis);
    Snooze.hibernate(Config_teensyLC, LLS);
  }
#endif
#if MAIN_LOOP_SETPOWER == MAIN_LOOP_VLPR
  // Runs inside macro at 2 Mhz --> Factor 12 slower
  REDUCED_CPU_BLOCK(DummyConfig)
  {
    // Run MAIN_LOOP_MILLIS-10 msec inside slow clock
    while ((millis() - startMillis) < MAIN_LOOP_MILLIS/2)
      // And snooze idle (WFI instruction)
      __asm__("wfi");
  }
  
  execMillis = millis();
  while ((millis() - execMillis) < MAIN_LOOP_MILLIS/2)
    // And snooze idle (WFI instruction)
    __asm__("wfi");
#endif
}

// *****************************************************************************
// Sensors Input Task calculations
// *****************************************************************************
static void readSensorsTask()
{
uint8_t  cdn, i;
uint32_t forceCnts;
int16_t  diffDeg;

  // Read the value from the HX711 sensor
  // ====================================
  if (hx711.is_ready())
  {
    Hx711SensorVal = hx711.read();
    // Filter the Force sensor Values
    //Hx711SensorFilt = (((Hx711SensorVal * (int32_t) HX711_FILTER) + (Hx711SensorFilt * (int32_t) (100 - HX711_FILTER))) / (int32_t) 100);
    Hx711SensorFilt = Hx711SensorVal;
  }
  // And save power
  //hx711.power_down();

  // Read the values from the ADXL sensor
  // ====================================
#if USE_ADXL_AXIS==0
  AdxlValue = Adxl.readXData();
#elif USE_ADXL_AXIS==1
  AdxlValue = Adxl.readYData();
#elif USE_ADXL_AXIS==2
  AdxlValue = Adxl.readZData();
#endif  

  // Filter the axis values if configured
#if XYAXIS_FILTER == 100
  AdxlValueFilt = AdxlValue;
#else
  AdxlValueFilt = (int16_t) ((((int32_t) AdxlValue * (int32_t) XYAXIS_FILTER) + ((int32_t) AdxlValueFilt * (int32_t) (100 - XYAXIS_FILTER))) / (int32_t) 100);
#endif

  AdxlValueZero = (int16_t) ((((int32_t) AdxlValueFilt * (int32_t) XYZERO_FILTER) + ((int32_t) AdxlValueZero * (int32_t) (100 - XYZERO_FILTER))) / (int32_t) 100);
#if 0
  AdxlTotal = AdxlTotal - (int32_t) AdxlBuf[AdxlBufIndex] + (int32_t) AdxlValueFilt;
  AdxlValueZero = (int16_t) (AdxlTotal / ADXL_BUF_SIZE);
  AdxlBuf[AdxlBufIndex] = AdxlValueFilt;
  AdxlBufIndex++;
  if (AdxlBufIndex >= ADXL_BUF_SIZE)
    AdxlBufIndex = 0;
#endif
  //Serial.println(AdxlValue);

  // Maintain Crank timing
  CrankTicks++;
  CrankTime = CrankTicks * SENSOR_TASK_MILLIS;
#if CADENCE_CALC == CADENCE_HALL
  // Cadence calculation based on Hall Sensor done in interrupt
  if (CrankTime >= CADENCE_TIMEOUT)
  {
    // Lower than Cadence 12 --> Drop to 0
    Cadence = 0;
    CadenceCount = 0;
    CrankTicks = 0;
  }

#else
  // Used to determine a Cadence Trigger
  cdn = 255;
  
  // Next check on Axis state change through 0 (every 180 deg)
  if ((AdxlValueFilt < AdxlValueZero && AdxlValuePrev > AdxlValueZero) || (AdxlValuePrev < AdxlValueZero && AdxlValueFilt > AdxlValueZero))
    CrankDeg = CrankDeg + 180;

  // Calculate the time done for 1 revolution
  if (CrankDeg >= 360)
  {
    // Calculate Cadence only when above a certain time to prevent too high values
    // Max Cadence is 60000/(236) = 254
    if (CrankTime > 236)
      cdn = (uint8_t) ((uint16_t) 60000 / CrankTime);
    else
      cdn = MAX_CADENCE;

    CrankDeg = 0;
    CrankTicks = 0;
  }
  else if (PrevCadence > 0)
  {
    // Based on previous Cadence, expect on time to calculate again for filtering down
    if (CrankTime > ((uint16_t) 60000 / (uint16_t) PrevCadence))
      cdn = (uint8_t) ((uint16_t) 60000 / CrankTime);
  }

  // Reset to 0 when longer than 5 seconds no movement
  if (CrankTicks > MSEC_TO_SENSTICKS(5000))
  {
    CrankDeg = 0;
    CrankTicks = 0;
    AdxlValueZero = 0;
    
    CadenceTotal = 0;
    cdn = 0;
    // Flush buffer contents
    for (i = 0; i < ADXL_BUF_SIZE; i++)
      AdxlBuf[i] = 0;
    for (i = 0; i < CDN_BUF_SIZE; i++)
      CadenceBuf[i] = 0;
  }

  // Store previous axis readings for next loop
  AdxlValuePrev = AdxlValueFilt;

  // Put Cadence calculated in buffer when new value available
  if (cdn != 255)
  {
    CadenceTotal = CadenceTotal - CadenceBuf[CadenceBufIndex] + (uint16_t) cdn;
    Cadence = (uint8_t) (CadenceTotal / CDN_BUF_SIZE);

    CadenceBuf[CadenceBufIndex] = cdn;
    CadenceBufIndex++;
    if (CadenceBufIndex >= CDN_BUF_SIZE)
      CadenceBufIndex = 0;
    // Store this cadence for next sample calculation to compare on time expectation
    PrevCadence = cdn;
  }
#endif
  
  // Calculate Force, Torque and Power
  // =================================
  if (Hx711SensorFilt >= Hx711SensorOffset) 
    forceCnts = (uint32_t) (Hx711SensorFilt - Hx711SensorOffset);
  else
    forceCnts = (uint32_t) (Hx711SensorOffset - Hx711SensorFilt);
  Force  = (uint16_t) (forceCnts / (uint32_t) FORCE_SCALE_FACTOR);
  Torque = (uint16_t) (((uint32_t) Force * (uint32_t) ForceDistance) / (uint32_t) 1000);
  Power  = (uint16_t) (((uint32_t) 105 * (uint32_t) Cadence * (uint32_t) Torque) / (uint32_t) 1000);
  // Store calculated Power in buffer
  PowerBuf[PowerBufIndex] = Power;
  PowerBufIndex++;
  if (PowerBufIndex >= POWER_BUF_SIZE)
    PowerBufIndex = 0;
}

// *****************************************************************************
// Cadence Interrupt handler
// *****************************************************************************
static void cadenceTimerTask()
{
  CadenceCount++;
}

// *****************************************************************************
// Cadence Interrupt handler
// *****************************************************************************
static void cadenceInterrupt()
{
  // Determine the cycle time in milliseconds
  CadenceMillis = millis();
  CadenceCount = (uint16_t) (CadenceMillis - CadenceMillisPrev);
  // Save for next passing
  CadenceMillisPrev = CadenceMillis;
  // Calculate Cadence only when above a certain time to prevent too high values
  // Max Cadence is 60000/(236) = 254
  //noInterrupts();
  if (CadenceCount > MIN_CADENCE_TIME && CadenceCount < CADENCE_TIMEOUT)
  {
    Cadence = (uint8_t) ((uint16_t) 60000 / (uint16_t) CadenceCount);
    //CadenceCount = 0;
    // Keep timeout counter reset
    CrankTicks = 0;
  }
  //interrupts();
}

// *****************************************************************************
// Teensy LC Battery voltage determination using the 3.3V reference as basis
// for Teensy LC, 3v3 input, only valid between 1.65V and 3.5V. Returns in millivolts.
// *****************************************************************************
uint32_t getInputVoltage()
{ 
uint32_t x;

    x = analogRead(ANA_BATT_VOLT_CH);
    return ((266*x*x + 2496026531 - 1431005 * x) / 342991) + 42;
}

// *****************************************************************************
// Determine Battery Voltage and Status level
// *****************************************************************************
static void determineBatteryStatus()
{
  // Battery monitoring
  BatteryVoltage = (uint16_t) getInputVoltage() / 10;
  if (BatteryVoltage < 220)
    BatteryStatus = BATTERY_CRITICAL;
  else if (BatteryVoltage < 240)
    BatteryStatus = BATTERY_LOW;
  else if (BatteryVoltage < 260)
    BatteryStatus = BATTERY_OK;
  else if (BatteryVoltage < 280)
    BatteryStatus = BATTERY_GOOD;
  else
    BatteryStatus = BATTERY_NEW;
}

// *****************************************************************************
// EEPROM routines to read/write the configuration to/from non volatile memory
// *****************************************************************************
static void eepromGetConfig()
{
int16_t i;
int32_t templ;

  // Check for EEPROM valid data configuration
  if (EEPROM[0] == 0x55 && EEPROM[1] == 0xAA)
  {
    // Read configuration variables out of EEPROM
    templ = ((int32_t) EEPROM[EE_STRAIN_OFFSET + 0] << 24 |
             (int32_t) EEPROM[EE_STRAIN_OFFSET + 1] << 16 |
             (int32_t) EEPROM[EE_STRAIN_OFFSET + 2] << 8  |
             (int32_t) EEPROM[EE_STRAIN_OFFSET + 3]);
    Hx711SensorOffset = templ;
    
    AdxlTempOffset = ((int32_t) EEPROM[EE_TEMP_OFFSET + 0] << 8  |
                      (int32_t) EEPROM[EE_TEMP_OFFSET + 1]);

#if DEBUG_MAIN_PROCESS == 1
    Serial.println ("EEprom restored");
#endif  
  } 
  else
  {
    // EEPROM first time initialization
    EEPROM[0] = 0x55;
    EEPROM[1] = 0xAA;
    // Clear all contents to 0
    for (i = EE_START_ADDR; i < EEPROM.length(); i++)
      EEPROM.write(i, 0x00);

#if DEBUG_MAIN_PROCESS == 1
    Serial.println ("EEprom initialized");
#endif  
  }
}

// *****************************************************************************
// EEPROM routines to read/write the configuration to/from non volatile memory
// *****************************************************************************
static void eepromSetConfig()
{
  // Write configuration variables to the EEPROM
  EEPROM[EE_STRAIN_OFFSET + 0] = (uint8_t) ((Hx711SensorOffset >> 24) & 0x000000FF);
  EEPROM[EE_STRAIN_OFFSET + 1] = (uint8_t) ((Hx711SensorOffset >> 16) & 0x000000FF);
  EEPROM[EE_STRAIN_OFFSET + 2] = (uint8_t) ((Hx711SensorOffset >> 8) & 0x000000FF);
  EEPROM[EE_STRAIN_OFFSET + 3] = (uint8_t) (Hx711SensorOffset & 0x000000FF);

  EEPROM[EE_TEMP_OFFSET + 0] = (uint8_t) ((AdxlTempOffset >> 8) & 0x000000FF);
  EEPROM[EE_TEMP_OFFSET + 1] = (uint8_t) (AdxlTempOffset & 0x000000FF);
}


// *****************************************************************************
// ANT stack initialization and network setup
// *****************************************************************************
static void antSetup()
{
  // Send reset to ANT Network
  ANTreset();
  // Delay after resetting the radio (spec: max 2 msec)
  delay (10);
  ANTreceive(ANT_RX_SETUP_TIMEOUT);

  // Set the Network Key (currently to DEVELOPER)
  ANTsetNetwork();
  ANTreceive(ANT_RX_SETUP_TIMEOUT);

  // Assign ANT Channel as bi-directional transmitter (Master)
  ANTassignChannel(0, CHANNEL_TYPE_BIDIRECTIONAL_TRANSMIT);
  ANTreceive(ANT_RX_SETUP_TIMEOUT);

  // Set Channel ID to Bike Power profile
  ANTsetChannelId(ANT_CHANNEL_NR, (uint16_t) ANT_DEVICE_NR, BIKE_POWER_PROFILE, BIKE_POWER_TT);
  ANTreceive(ANT_RX_SETUP_TIMEOUT);

  // Set frequency for profile to 4Hz updates
  ANTsetFrequency(ANT_CHANNEL_NR, BIKE_POWER_RF);
  ANTreceive(ANT_RX_SETUP_TIMEOUT);

  // Set ANT period
  ANTsetPeriod(ANT_CHANNEL_NR, BIKE_POWER_CP);
  ANTreceive(ANT_RX_SETUP_TIMEOUT);

  // Set ANT Tx Power to medium power for optimum power management
  ANTsetTxPower(ANT_TX_POWER_MEDIUM);
  ANTreceive(ANT_RX_SETUP_TIMEOUT);

  // Open the ANT channel for communication broadcasting
  ANTopenChannel(ANT_CHANNEL_NR);
  ANTreceive(ANT_RX_SETUP_TIMEOUT);
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
}

// *****************************************************************************
// ANT Broadcast messages; Calibration
// *****************************************************************************
static void broadcastCalibration (boolean success, uint16_t calibrationVal)
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
}

// *****************************************************************************
// ANT Broadcast messages; Battery Info (Optional)
// *****************************************************************************
static void broadcastBatteryInfo (void)
{
uint8_t  buf[13];
uint32_t ticks;

  // Fill ANT Battery info data buffer with data
  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x09;
  buf[2] = MSG_BROADCAST_DATA_ID;
  buf[3] = ANT_CHANNEL_NR;
  buf[4] = DP_BATTERY_INFO;
  buf[5] = 0xFF;
  buf[6] = 0xFF;                  // Battery identifier not used
  ticks = TotalSeconds / 2;       // Runtime in 2 seconds units
  buf[7] = (uint8_t) (ticks & 0x000000FF);
  ticks = ticks >> 8;
  buf[8] = (uint8_t) (ticks & 0x000000FF);
  ticks = ticks >> 8;
  buf[9] = (uint8_t) (ticks & 0x000000FF);
  buf[10] = (uint8_t) (256 * (BatteryVoltage % 100) / 100);
  buf[11] = 0x80 | (BatteryStatus << 4) | (uint8_t) (BatteryVoltage / 100);
  buf[12] = checkSum(buf, 12);
  ANTsend (buf, 13);
}

// *****************************************************************************
// ANT AP2 hard reset function
// *****************************************************************************
static void AP2reset (void)
{
  // Issue an AP2 hard reset
  digitalWrite(AP2_RST_PIN, 0);
  delayMicroseconds(100);
  digitalWrite(AP2_RST_PIN, 1);
}

// *****************************************************************************
// ANT AP2 check/wait for RTS line low
// *****************************************************************************
static bool AP2waitCts (void)
{
uint8_t retry = 0;

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
static boolean ANTreceive (uint16_t timeout)
{
int16_t sbuflength = 0;
uint8_t msg = 0;
uint16_t retry = 0;

  // Wait for start of message reception (if timeout in milliseconds)
  if (timeout > 0)
  {
    retry = (uint16_t) (timeout * 10);
    while (sbuflength == 0 && retry > 0)
    {
      sbuflength = AntSerial.available();
      if (sbuflength == 0)
        delayMicroseconds (100);
      retry--;
    }
  }
  // No timeout given, just polling mode in the loop
  else
    sbuflength = AntSerial.available();
 
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
        for (int16_t i = 0 ; i < MsgLength ; i++)
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
    retry = (uint16_t) (20);
    while (AntSerial.available() == 0 && retry > 0)
    {
      delayMicroseconds (100);
      retry--;
    }

    sbuflength = AntSerial.available();
  }

  // Status indication ANT setup procedure
  if (AntTxSetupOK)
  {
    AntTxSetupOK = false;
    digitalWrite(LED_BUILTIN, HIGH);
    delayMicroseconds (50);
    digitalWrite(LED_BUILTIN, LOW);
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
        AntTxSuccess = true;
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
      if (MsgBuf[5] == 0)
        AntTxSetupOK = true;
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
static void ANTsend (uint8_t buf[], int16_t length)
{
int16_t i;

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
    AntSerial.flush();
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
static uint8_t checkSum (uint8_t *dataPtr, int16_t length)
{
int16_t i;
uint8_t chksum;

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
int16_t i;

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
static void ANTassignChannel (uint8_t chNr, uint8_t chType)
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
static void ANTsetChannelId (uint8_t chNr, uint16_t deviceNr, uint8_t deviceId, uint8_t deviceTt)
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
static void ANTsetFrequency (uint8_t chNr, uint8_t freq)
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
static void ANTsetPeriod (uint8_t chNr, uint16_t period)
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
// ANT setup routine(s); ANTsetTxPower
// *****************************************************************************
static void ANTsetTxPower (uint8_t power)
{
uint8_t buf[6];

  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x02;
  buf[2] = MSG_RADIO_TX_POWER_ID;
  buf[3] = 0;
  buf[4] = power;
  buf[5] = checkSum(buf, 5);
  ANTsend(buf, 6);
}

// *****************************************************************************
// ANT setup routine(s); ANTopenChannel
// *****************************************************************************
static void ANTopenChannel (uint8_t chNr)
{
uint8_t buf[5];

  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x01;
  buf[2] = MSG_OPEN_CHANNEL_ID;
  buf[3] = chNr;
  buf[4] = checkSum(buf, 4);
  ANTsend(buf, 5);
}
