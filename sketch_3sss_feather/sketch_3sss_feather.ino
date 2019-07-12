// *****************************************************************************
// INCLUDES Section
// *****************************************************************************
#include <stdio.h>
#include <SPI.h>
//#include <ArduinoLowPower.h>
#include "src\hx711.h"
#include "src\bmg250.h"
#include "src\flashaseeprom.h"

// *****************************************************************************
// DEFINITIONS Section
// *****************************************************************************
// Main Loop timing definitions
#define MAIN_LOOP_MILLIS    250
#define HX711_PWRUP_MILLIS  (MAIN_LOOP_MILLIS-60)
#define MAIN_TICKS_PER_SEC  (1000/MAIN_LOOP_MILLIS)
#define MSEC_TO_TICKS(x)    (uint16_t) (((uint32_t) (x)*(uint32_t) MAIN_TICKS_PER_SEC)/(uint32_t) 1000)
#define INACTIVE_TIMEOUT    (5 * 60)

#define MAIN_LOOP_DELAY     0
#define MAIN_LOOP_WFI       1
#define MAIN_LOOP_SETPOWER  MAIN_LOOP_WFI

// Debug settings definitions
#define DEBUG               1
#if DEBUG == 1
#define DEBUG_MAIN_PROCESS  1
#define DEBUG_ANT_TX        0
#define DEBUG_ANT_RX        0
#endif
#define ANT_SIMULATION      0

// Set the pins used
#define VBAT_PIN        (A7)
#define RED_LED_PIN     (13)
#define HX711_DATA_PIN  (14)
#define HX711_CLOCK_PIN (15)
#define BMG250_SELECT   (A5)

// EEPROM address definitions
#define EE_START_ADDR       2
#define EE_STRAIN_OFFSET    EE_START_ADDR + 0
#define EE_NEXT_SETTING     EE_START_ADDR + 4

// Strain Gauge interface
#define HX711_FILTER        50      // new value for 50 %
#define HX711_DATA_PIN      (A0)
#define HX711_CLOCK_PIN     (A1)
#define FORCE_SCALE_FACTOR  320     // In HX711 counts
#define DEFAULT_FORCE_DIST  88      // in mm
#define POWER_BUF_SIZE      2*MAIN_TICKS_PER_SEC
#define CADENCE_FILTER      20      // New sensor value for 20 %

// BMG250 definitions
#define BMG250_MAXDEG   1000

// Battery status definitions
#define BATTERY_FULL        1
#define BATTERY_GOOD        2
#define BATTERY_OK          3
#define BATTERY_LOW         4
#define BATTERY_CRITICAL    5

// ANT Hardware and Product definitions
#define HARDWARE_REVISION   ((uint8_t)  0x01)
#define MANUFACTURER_ID     ((uint16_t) 0x00FF)
#define MODEL_NUMBER        ((uint16_t) 3555)
#define SW_MAIN_REVISION    1
#define SW_SUB_REVISION     4

// ANT AP2 Interface definitions
#define AP2_TX_PIN          (1)
#define AP2_RX_PIN          (0)
#define AP2_RST_PIN         (5)
#define AP2_SUSPEND_PIN     (6)
#define AP2_SLEEP_PIN       (10)
#define AP2_RTS_PIN         (11)
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
// Main loop variables
static uint16_t MainCount = 0;
static uint16_t BatteryVoltage = 0;
static uint8_t  BatteryStatus;
static uint32_t TotalSeconds = 0;
static uint32_t InactiveSeconds = 0;

// Loadcell, Strain Gauge Amplifier variables
const uint8_t  hx711_data_pin = HX711_DATA_PIN;
const uint8_t  hx711_clock_pin = HX711_CLOCK_PIN;
static int32_t Hx711SensorVal = 0;
static int32_t Hx711SensorFilt = 0;
static int32_t Hx711SensorOffset = 0;
// Construct the Loadcell Amplifier object
static HX711 hx711 = HX711(hx711_data_pin, hx711_clock_pin, 128);

// Gyro interface and calculation variables
static struct bmg250_dev         Gyro_dev;
static struct bmg250_cfg         Gyro_cfg;
static struct bmg250_sensor_data Gyro_data;
static int8_t                    GyroSts = BMG250_OK;

// Buffer varables
static char PrintBuf [160];

// Processing variables
static int16_t  CadenceX;
static int16_t  CadenceY;
static int16_t  CadenceZ;
static int16_t  CadenceFilt = 0;
static uint16_t Force;
static uint16_t Torque;
static uint16_t Power;
static uint16_t ForceDistance = DEFAULT_FORCE_DIST;
static uint16_t PowerBuf [POWER_BUF_SIZE];
static uint8_t  PowerBufIndex = 0;

// ANT communication data
#define AntSerial Serial1

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

static int8_t SPIread(uint8_t id, uint8_t reg, uint8_t *dataPtr, uint16_t len);
static int8_t SPIwrite(uint8_t id, uint8_t reg, uint8_t *dataPtr, uint16_t len);
static void SPIdelay(uint32_t ms);

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
int8_t rslt = BMG250_OK;

  Serial.begin(115200);
#if DEBUG_MAIN_PROCESS == 1
  // Wait for native USB to be ready, max 5 seconds timeout
  while (!Serial && millis() < 5000)
    ;
  Serial.println("\nFeather Basic Setup start");
#endif

  // Initialize digital LED pin as an output
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Read the Configuration for the EEPROM storage
  eepromGetConfig();

  // Initalize the chip select pin
  pinMode(BMG250_SELECT, OUTPUT);
  digitalWrite(BMG250_SELECT, HIGH);
  // Start the SPI library:
  SPI.begin();
  
#if DEBUG_MAIN_PROCESS == 1
  Serial.println("BMG250 Init");
#endif

  // Setup the Gyro Sensor interface over SPI with native chip select line
  Gyro_dev.dev_id = 0;
  Gyro_dev.interface = BMG250_SPI_INTF;
  Gyro_dev.read = SPIread;
  Gyro_dev.write = SPIwrite;
  Gyro_dev.delay_ms = SPIdelay;
  rslt = bmg250_init(&Gyro_dev);  
#if DEBUG_MAIN_PROCESS == 1
  sprintf(PrintBuf, "BMG250 Init: sts=%02X id=%02X\n\r", rslt, Gyro_dev.chip_id);
  Serial.println(PrintBuf);
#endif

  // Setting the power mode as normal
  Gyro_dev.power_mode = BMG250_GYRO_NORMAL_MODE;
  rslt = bmg250_set_power_mode(&Gyro_dev);
#if DEBUG_MAIN_PROCESS == 1
  sprintf(PrintBuf,"BMG250 set_power_mode: sts=%02X", rslt);
  Serial.println(PrintBuf);
#endif

  // Read the set configuration from the sensor
  rslt = bmg250_get_sensor_settings(&Gyro_cfg, &Gyro_dev);
#if DEBUG_MAIN_PROCESS == 1
  sprintf(PrintBuf,"BMG250 get_sensor_settings: sts=%02X; odr=%02X, rng=%02X, bw=%02X", rslt, Gyro_cfg.odr, Gyro_cfg.range, Gyro_cfg.bw);
  Serial.println(PrintBuf);
#endif
  if (rslt == BMG250_OK)
  {
    // Selecting the ODR as 25Hz
    Gyro_cfg.odr = BMG250_ODR_25HZ;
    // Selecting the bw as Normal mode
    Gyro_cfg.bw = BMG250_BW_NORMAL_MODE;
    // Modify Range into 1000 DPS
    Gyro_cfg.range = BMG250_RANGE_1000_DPS;
  }
  rslt = bmg250_set_sensor_settings(&Gyro_cfg, &Gyro_dev); 
#if DEBUG_MAIN_PROCESS == 1
  sprintf(PrintBuf,"BMG250 set_sensor_settings: sts=%02X; odr=%02X, rng=%02X, bw=%02X", rslt, Gyro_cfg.odr, Gyro_cfg.range, Gyro_cfg.bw);
  Serial.println(PrintBuf);
#endif

  // Initialize the AP2 interface pins
  digitalWrite(AP2_RST_PIN, 1);
  pinMode(AP2_RST_PIN, OUTPUT);
  pinMode(AP2_SUSPEND_PIN, OUTPUT);
  pinMode(AP2_SLEEP_PIN, OUTPUT);
  pinMode(AP2_RTS_PIN, INPUT);

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
  
#if DEBUG_MAIN_PROCESS == 1
  Serial.println("Setup routine is Ready");
#endif
}

 
// *****************************************************************************
// Main process loop task
// *****************************************************************************
void loop()
{
int16_t  i;
uint16_t powerBufAvg;
uint32_t startMillis;

  // Setup and maintain loop timing and counter, blinky
  startMillis = millis();
  MainCount++;
  // Maintain Total Seconds counter, blink LED
  if ((MainCount % MSEC_TO_TICKS(1000)) == 0)
  {
    if (AntTxSuccess)
    {
      // Ant Tx is active; blink Led only every second shortly
      digitalWrite(LED_BUILTIN, HIGH);
      AntTxSuccess = false;
      delayMicroseconds(50);
      // Turn off LED indicator
      digitalWrite(LED_BUILTIN, LOW);
    }
    // Increment Seconds counters
    TotalSeconds++;
    InactiveSeconds++;
  }

  // Read Sensor data and calculations
  readSensorsTask();
  // Whenever there is some Cadence, keep Inactive timer reset
  if (CadenceFilt > 5)
    InactiveSeconds = 0;
    
  // Calculate average of Power buffer values
  powerBufAvg = 0;
  for (i = 0; i < POWER_BUF_SIZE; i++)
    powerBufAvg = powerBufAvg + PowerBuf[i];
  powerBufAvg = powerBufAvg / POWER_BUF_SIZE;
  
  // Output serial data for debugging
#if DEBUG_MAIN_PROCESS == 1
  // Log Strain Gauges only for Serial Plotter
  //sprintf(PrintBuf, "%8d", Hx711SensorVal);
  //Serial.println(PrintBuf);
  
  // Log all sensor values
  //sprintf(PrintBuf, "Cx: %8d, Cy: %8d, Cz: %8d, T: %6d, HX711: %8d, bat: %3d", CadenceX, CadenceY, CadenceZ, Gyro_data.sensortime, Hx711SensorVal, BatteryVoltage);
  //Serial.println(PrintBuf);

  // Log Force, Torqu, Cadence and Power
  sprintf(PrintBuf, "F:%4u T:%3u C:%3d P:%3u", Force, Torque, CadenceFilt, Power);
  Serial.println(PrintBuf);
#endif

  // Broadcast ANT data messages @ 4 Hz update intervals
  if ((MainCount % MSEC_TO_TICKS(250)) == 0)
  {
    // Check for an ANT Calibration request
    if (AntCalibration)
    {
      // Tare offset the strain gauge values with averaged value
#if ANT_SIMULATION == 1
      Hx711SensorOffset = 142364ul;
#else
      Hx711SensorOffset = Hx711SensorFilt;
#endif
      broadcastCalibration(true, (uint16_t) (Hx711SensorOffset >> 4));
      // Finally save the configuration to the EEPROM
      eepromSetConfig();
      AntCalibration = false;
    }
    // Interleave Power broadcast every 4 messages
    else if ((SendCount % 5) != 4)
    {
      AntPedalPower = 0xFF;
      AntCadence = CadenceFilt;
#if ANT_SIMULATION == 1
      AntPower = 300 + random (-25, 100);
#else
      AntPower = powerBufAvg;
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
        // Blink short every 15 seconds, another run indcator
        digitalWrite(LED_BUILTIN, HIGH);
        delayMicroseconds(100);
        digitalWrite(LED_BUILTIN, LOW);
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
  if (InactiveSeconds >= INACTIVE_TIMEOUT)
  {
    // Suspend ANT, stops communication
    digitalWrite(AP2_SUSPEND_PIN, 0);
    // Activate Deep Sleep mode, only to recover with a RESET
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __WFI();
    //LowPower.deepSleep();
  }
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
  
#if MAIN_LOOP_SETPOWER == MAIN_LOOP_DELAY
  // Determine execution time, and delay for the remainder
  execMillis = (uint16_t) (millis() - startMillis);
  delay(MAIN_LOOP_MILLIS - execMillis - HX711_PWRUP_MILLIS);
  hx711.power_up();
  delay(HX711_PWRUP_MILLIS);
#endif
#if MAIN_LOOP_SETPOWER == MAIN_LOOP_WFI
  do
  {
    // Idle mode, Wait for Interrupt instruction
    __asm__("wfi");
    // Determine execution time, and delay for the remainder
    execMillis = (uint16_t) (millis() - startMillis);
    // Power up the HX711 'startup time' before Main Loop needs to run again
    if (execMillis > HX711_PWRUP_MILLIS)
      hx711.power_up();
  }
  while (execMillis < MAIN_LOOP_MILLIS);
#endif
}

// *****************************************************************************
// Sensors Input Task calculations
// *****************************************************************************
static void readSensorsTask()
{
uint32_t forceCnts;
int16_t  xDeg, yDeg, zDeg;

  // ====================================
  // Read the value from the HX711 sensor
  // ====================================
  if (hx711.is_ready())
    Hx711SensorVal = hx711.read();
  // And save power by powering down the HX711 and Strain Gauge measurement
  hx711.power_down();
  // Save a filtered value for Calibration purposes
  Hx711SensorFilt = (((Hx711SensorVal * (int32_t) HX711_FILTER) + (Hx711SensorFilt * (int32_t) (100 - HX711_FILTER))) / (int32_t) 100);
  
  // =====================================
  // Read the value from the Gyro sensors
  // =====================================
  GyroSts = bmg250_get_sensor_data(BMG250_DATA_TIME_SEL, &Gyro_data, &Gyro_dev);
#if 0
  xDeg = (int16_t) (((int32_t) BMG250_MAXDEG * (int32_t) Gyro_data.x) / (int32_t) 32767);
  CadenceX = (int16_t) ((int32_t) 60 * (int32_t) xDeg / (int32_t) 360);
  yDeg = (int16_t) (((int32_t) BMG250_MAXDEG * (int32_t) Gyro_data.y) / (int32_t) 32767);
  CadenceY = (int16_t) ((int32_t) 60 * (int32_t) yDeg / (int32_t) 360);
#endif
  zDeg = (int16_t) (((int32_t) BMG250_MAXDEG * (int32_t) Gyro_data.z) / (int32_t) 32767);
  CadenceZ = (int16_t) ((int32_t) 60 * (int32_t) zDeg / (int32_t) 360);
  if (CadenceZ < 0)
    CadenceZ = -CadenceZ;
  CadenceFilt = (((CadenceZ * (int16_t) CADENCE_FILTER) + (CadenceFilt * (int16_t) (100 - CADENCE_FILTER))) / (int16_t) 100);
  
  // =================================
  // Calculate Force, Torque and Power
  // =================================
  if (Hx711SensorVal >= Hx711SensorOffset) 
    forceCnts = (uint32_t) (Hx711SensorVal - Hx711SensorOffset);
  else
    forceCnts = (uint32_t) (Hx711SensorOffset - Hx711SensorVal);
  Force  = (uint16_t) (forceCnts / (uint32_t) FORCE_SCALE_FACTOR);
  Torque = (uint16_t) (((uint32_t) Force * (uint32_t) ForceDistance) / (uint32_t) 1000);
  Power  = (uint16_t) (((uint32_t) 105 * (uint32_t) CadenceFilt * (uint32_t) Torque) / (uint32_t) 1000);
  // Store calculated Power in buffer
  PowerBuf[PowerBufIndex] = Power;
  PowerBufIndex++;
  if (PowerBufIndex >= POWER_BUF_SIZE)
    PowerBufIndex = 0;
}

// *****************************************************************************
// Determine Battery Voltage and Status level
// *****************************************************************************
static void determineBatteryStatus()
{
uint16_t vbat;

  // Battery monitoring
  vbat = analogRead(VBAT_PIN);
  BatteryVoltage = (vbat * 2 * 330) / 1024;        // divided by 2, multiply by Ref and divide by 1024
  if (BatteryVoltage < 330)
    BatteryStatus = BATTERY_CRITICAL;
  else if (BatteryVoltage < 340)
    BatteryStatus = BATTERY_LOW;
  else if (BatteryVoltage < 350)
    BatteryStatus = BATTERY_OK;
  else if (BatteryVoltage < 360)
    BatteryStatus = BATTERY_GOOD;
  else
    BatteryStatus = BATTERY_FULL;
}

// *****************************************************************************
// EEPROM routines to read/write the configuration to/from non volatile memory
// *****************************************************************************
static void eepromGetConfig()
{
int16_t i;
int32_t templ;

  // Check for EEPROM valid data configuration
  if (EEPROM.isValid())
  {
    // Read configuration variables out of EEPROM
    templ = ((int32_t) EEPROM.read(EE_STRAIN_OFFSET + 0) << 24 |
             (int32_t) EEPROM.read(EE_STRAIN_OFFSET + 1) << 16 |
             (int32_t) EEPROM.read(EE_STRAIN_OFFSET + 2) << 8  |
             (int32_t) EEPROM.read(EE_STRAIN_OFFSET + 3));
    Hx711SensorOffset = templ;

#if DEBUG_MAIN_PROCESS == 1
    Serial.println(templ);
    Serial.println("EEprom restored");
#endif  
  } 
  else
  {
    // EEPROM first time initialization
    EEPROM.write(0, 0x55);
    EEPROM.write(1, 0xAA);
    // Clear all contents to 0
    for (i = EE_START_ADDR; i < EEPROM.length(); i++)
      EEPROM.write(i, 0x00);
    
    EEPROM.commit();
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
  EEPROM.write(EE_STRAIN_OFFSET + 0, (uint8_t) ((Hx711SensorOffset >> 24) & 0x000000FF));
  EEPROM.write(EE_STRAIN_OFFSET + 1, (uint8_t) ((Hx711SensorOffset >> 16) & 0x000000FF));
  EEPROM.write(EE_STRAIN_OFFSET + 2, (uint8_t) ((Hx711SensorOffset >> 8) & 0x000000FF));
  EEPROM.write(EE_STRAIN_OFFSET + 3, (uint8_t) (Hx711SensorOffset & 0x000000FF));

  EEPROM.commit();
}

// *****************************************************************************
// Local SPI Read function implementation
// *****************************************************************************
static int8_t SPIread(uint8_t id, uint8_t reg, uint8_t *dataPtr, uint16_t len)
{
uint16_t i;
uint8_t  inByte = 0;           // incoming byte from the SPI
int8_t   rslt = BMG250_OK;     // result to return

  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  // take the chip select low to select the device:
  digitalWrite(BMG250_SELECT, LOW);
  // send the device the register you want to read:
  SPI.transfer(reg);
  for (i = 0; i < len; i++)
  {
    dataPtr[i] = SPI.transfer(0);
  }
  // take the chip select high to de-select:
  digitalWrite(BMG250_SELECT, HIGH);
  SPI.endTransaction();
  // return the result:
  return (rslt);
}

// *****************************************************************************
// Local SPI Write function implementation
// *****************************************************************************
static int8_t SPIwrite(uint8_t id, uint8_t reg, uint8_t *dataPtr, uint16_t len)
{
uint16_t i;
uint8_t  inByte = 0;           // incoming byte from the SPI
int8_t   rslt = BMG250_OK;     // result to return

  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  // take the chip select low to select the device:
  digitalWrite(BMG250_SELECT, LOW);
  // send the device the register you want to read:
  SPI.transfer(reg);
  for (i = 0; i < len; i++)
  {
    SPI.transfer(dataPtr[i]);
  }
  // take the chip select high to de-select:
  digitalWrite(BMG250_SELECT, HIGH);
  SPI.endTransaction();
  // return the result:
  return (rslt);
}

// *****************************************************************************
// Local SPI Delay function implementation
// *****************************************************************************
static void SPIdelay (uint32_t ms)
{
  delay(ms);
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

