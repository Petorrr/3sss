// *****************************************************************************
// INCLUDES Section
// *****************************************************************************
#include <ArduinoBLE.h>
#include <mbed.h>
#include <KVStore.h>
#include <kvstore_global_api.h>

// Customized the driver for the LSM9DS1
#include "src\LSM9DS1.h"

// *****************************************************************************
// DEFINITIONS Section
// *****************************************************************************
// Versioning
#define SW_MAIN_REVISION    0
#define SW_SUB_REVISION     1

// Main Loop timing definitions
#define MAIN_LOOP_MILLIS    50
#define MAIN_TICKS_PER_SEC  (1000/MAIN_LOOP_MILLIS)
#define MSEC_TO_TICKS(x)    (uint16_t) (((uint32_t) (x)*(uint32_t) MAIN_TICKS_PER_SEC)/(uint32_t) 1000)
#define INACTIVE_TIMEOUT    (2*60) //(5 * 60)

#define MAIN_LOOP_DELAY     0
#define MAIN_LOOP_WFI       1
#define MAIN_LOOP_SETPOWER  MAIN_LOOP_WFI

// Battery lifetime defines
#define BATT_CAPACITY       220   // mAh of a CR2032
#define BATT_CPS_CURRENT    4     // mA
#define BATT_EFFECTIVITY    60    // in %
#define BATT_MAX_LIFETIME   (BATT_EFFECTIVITY*(BATT_CAPACITY / BATT_CPS_CURRENT)*36) // in seconds

// Debug settings definitions
#define DEBUG               1
//#define BLE_SIMULATION      1
#define PIN_INPUT_SHUTDOWN  (2)

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -1.67 // Declination (degrees) in Hoofddorp

// Cadence definitions
#define SENSOR_GYRO         1
#define SENSOR_ACCELERO     2
#define SENSOR_USE          SENSOR_GYRO   // Use Gyro for cadence determination

#define ACC_FILTER          (1)
#define UP_FORWARD          1
#define DOWN_FORWARD        2
#define DOWN_BACKWARD       3
#define UP_BACKWARD         4
// Cadence timing definitions (in milliseconds)
#define CADENCE_TIMEOUT     5000

// *****************************************************************************
// GLOBAL Variables Section
// *****************************************************************************
// Main loop variables
static uint16_t MainCount = 0;
static uint32_t BatteryLife = 0;
static uint32_t TotalSeconds = 0;
static uint32_t InactiveSeconds = 0;

// Loadcell, Strain Gauge Amplifier variables
//const uint8_t  hx711_data_pin = HX711_DATA_PIN;
//const uint8_t  hx711_clock_pin = HX711_CLOCK_PIN;
static int32_t Hx711SensorVal = 0;
static int32_t Hx711SensorFilt = 0;
static int32_t Hx711SensorOffset = 0;
// Construct the Loadcell Amplifier object
//static HX711 hx711 = HX711(hx711_data_pin, hx711_clock_pin, 128);

// BLE Variables
static uint8_t   MfgData[5] = {0x33, 0x53, 0x53, 0x53, 0x00};  //3SSS
static char      DeviceData[] = "3SSS PWR 0.01";
//static BLEDevice Central;
static boolean   CentralConnected = false;

// BLE Battery Service
static BLEService BatteryService("180F");
// BLE Battery Level Characteristic
static BLEUnsignedCharCharacteristic BatteryLevelChar("2A19", BLERead | BLENotify);

// BLE Cycling Power Service
static BLEService CyclingPower("1818");
// BLE Cycling Power Characteristics
static BLECharacteristic CPMC("2A63", BLENotify, 8, true);
static BLECharacteristic CPFC("2A65", BLERead, 4 , true);
static BLEUnsignedCharCharacteristic SLC("2A5D", BLERead);

// Set Cycling Power Measurement data to Cadence present, and initial values for power and cadence to 0
static uint8_t CPMCdata[8] = { 0b00101000, 0b00000000, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// Set Cycling Power Feature data; Revolution and Power data, torque based, non-distributed
static uint8_t CPFCdata[4] = { 0b00001001, 0b00000000, 0b00010001, 0b00000000 };

// Sensor task routine variables
static int16_t  OldBatteryLevel = 0;
static uint16_t Power = 0;
static uint16_t CadenceCount = 0;
static uint32_t CadenceTime = 0;
static uint8_t  CrankState = 0;
static uint8_t  NewCrankState = 0;
static uint32_t CrankTime = 0;
static uint32_t CrankPrev = 0;
static uint8_t  RawCadence = 0;
static float    GyroZ = 0.0;
static float    CadenceDeg = 0.0;

static int16_t  RollInt;
static int16_t  PitchInt;
static float    Roll;
static float    Pitch;
static float    Yaw;
static float    AccXFilt;
static float    AccYFilt;
static float    AccZFilt;

// Buffer varables
static char PrintBuf [80];
static char KvBuf [40];

// NVMEM keys and strings
static char CheckValidStr[] = "55AA";

// *****************************************************************************
// Local Function prototypes 
// *****************************************************************************
static void handleDelayScenario(uint32_t startMillis);
static void readSensorsTask(void);
static void calcRawCadence(void);
static void systemOff(void);
static void updateBatteryLevel(void);
static void BLEPeripheralConnectHandler(BLEDevice central);
static void BLEPeripheralDisconnectHandler(BLEDevice central);
static void nvmemGetConfig(void);
static void nvmemSetConfig(void);

void calculateAttitude(float ax, float ay, float az, float mx, float my, float mz);

// *****************************************************************************
// Initialization and Board setup routines
// *****************************************************************************
void setup()
{
  // Initialize LEDs
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  // Turn off Blue Led
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDB, HIGH);
  // Turn off Power Led
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR, LOW);
  pinMode(PIN_INPUT_SHUTDOWN, INPUT_PULLUP);
  
  Serial.begin(115200);    // initialize serial communication
  // Wait for native USB to be ready, max 2 seconds timeout
  while (!Serial && millis() < 2000)
    ;
  sprintf(PrintBuf,"3SSS PWR %1d.%02d", SW_MAIN_REVISION, SW_SUB_REVISION);
  Serial.println(PrintBuf);

  // Initialize on-board IMU
  if (!IMU.begin())
  {
#if DEBUG == 1
    Serial.println("Failed to initialize IMU!");
#endif
    digitalWrite(LEDR, LOW);
  }
  //IMU.powerOffGyro();

  // Begin BLE initialization
  if (!BLE.begin())
  {
#if DEBUG == 1
    Serial.println("Starting BLE failed!");
#endif
    digitalWrite(LEDR, LOW);
  }

  // Read the Configuration for the Parameters storage
  nvmemGetConfig();

  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setManufacturerData(MfgData, 5);
  BLE.setDeviceName(DeviceData);

  BLE.setLocalName("BatteryMonitor");
  BLE.setAdvertisedService(BatteryService);
  BatteryService.addCharacteristic(BatteryLevelChar);
  BLE.addService(BatteryService);
  // Initial value for this characteristic
  BatteryLevelChar.writeValue(OldBatteryLevel);

  BLE.setLocalName("Cycling Power");
  BLE.setAdvertisedService(CyclingPower);
  CyclingPower.addCharacteristic(CPMC);
  CyclingPower.addCharacteristic(CPFC);
  CyclingPower.addCharacteristic(SLC);
  BLE.addService(CyclingPower);
  // Initial values for these characteristics
  CPMC.writeValue(CPMCdata, 8);
  CPFC.writeValue(CPFCdata, 4);
  // Sensor location = 5 (Left Crank)
  SLC.writeValue(5);

  // Assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, BLEPeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, BLEPeripheralDisconnectHandler);

  // Start advertising BLE. It will start continuously transmitting BLE
  // advertising packets and will be visible to remote BLE central devices
  // until it receives a new connection
  // Advertsising interval 250 msec
  BLE.setAdvertisingInterval(400);
  BLE.advertise();

#if DEBUG == 1
  Serial.println("Bluetooth device active, waiting for connections...");
#endif
}

// *****************************************************************************
// Main process loop task
// *****************************************************************************
void loop()
{
uint32_t  startMillis;
uint16_t  cadenceVal;

  // Setup and maintain loop timing and main counter
  startMillis = millis();
  MainCount++;
  // Read and calculate all sensor values
  readSensorsTask();
  
  // Maintain Total Seconds counter, blink BLE LED shortly
  if ((MainCount % MSEC_TO_TICKS(1000)) == 0)
  {
    // Increment Seconds counters
    TotalSeconds++;
    BatteryLife++;
    // Whenever there is some Cadence, keeo inactive 0
    if (RawCadence > 5)
      InactiveSeconds = 0;
    else
      InactiveSeconds++;
#if DEBUG == 1
    // Toggle PWR LED when in DEBUG mode
    digitalWrite(LED_PWR, HIGH);
    delayMicroseconds(200);
    digitalWrite(LED_PWR, LOW);

    // Check if BLE is active; blink Blue Led only every second shortly
   if (CentralConnected)
    {
      digitalWrite(LEDB, LOW);
      delayMicroseconds(200);
      digitalWrite(LEDB, HIGH);
    }
#endif    
  }

  // Send Power and Cadence data every 500 msec
  if ((MainCount % MSEC_TO_TICKS(500)) == 1)
  {
#if BLE_SIMULATION==1
    // Simulate Power and Cadence values
    Power = 200 + random (-25, 100);
    CadenceTime = CadenceTime+2000;
    CadenceCount += random(2,5);
#endif
    Power = 200 + random (-25, 100);
    // Convert to 1/1024 part
    cadenceVal = (uint16_t) ((uint32_t) 1024 * CadenceTime / (uint32_t) 1000);

    // Query for the BLE central
    //central = BLE.central();
    if (CentralConnected)
    {
      // Construct the Cycling Power Measurement Characteristic values
      CPMCdata[2] = lowByte(Power);
      CPMCdata[3] = highByte(Power);
      CPMCdata[4] = lowByte(CadenceCount);
      CPMCdata[5] = highByte(CadenceCount);
      CPMCdata[6] = lowByte(cadenceVal);
      CPMCdata[7] = highByte(cadenceVal);
      CPMC.writeValue(CPMCdata, 8);
    }
#if DEBUG==1
    sprintf(PrintBuf, "state=%1d, raw = %3d, cnt=%4d, tim=%d", CrankState, RawCadence, CadenceCount, CadenceTime);
    //sprintf(PrintBuf, "%4d, %4d, %4d", (int16_t) Roll, (int16_t) Pitch, (int16_t) Yaw);
    //sprintf(PrintBuf, "%4d, %4d", (int16_t) Roll, (int16_t) Pitch);
    //sprintf(PrintBuf, "%4d, %4d, %4d", (int16_t) (AccXFilt*1000), (int16_t) (AccYFilt*1000), (int16_t) (AccZFilt*1000));
    Serial.println(PrintBuf);
#endif
  }

  // Check change in Battery Level every 10 seconds
  if ((MainCount % MSEC_TO_TICKS(10000)) == 2)
  {
    // Calculate and update the Battery Characteristic onlu when changed
    updateBatteryLevel();
  }

  // Check if have to go to system off mode
  if (InactiveSeconds >= INACTIVE_TIMEOUT || digitalRead(PIN_INPUT_SHUTDOWN) == 0)
    systemOff();

  // Check every minute parameter saving
  if ((TotalSeconds % 60) == 0)
     nvmemSetConfig();
     
  // At last handle the 'sleep' until next main cycle needed
  handleDelayScenario(startMillis);
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
  delay (MAIN_LOOP_MILLIS - execMillis);
#endif

#if MAIN_LOOP_SETPOWER == MAIN_LOOP_WFI
  do
  {
    // Idle mode, Wait for Interrupt instruction
    __asm__("wfi");
    // Determine execution time, and delay for the remainder
    execMillis = (uint16_t) (millis() - startMillis);
#if MAIN_LOOP_MILLIS >= 125
    // Power up the HX711 'startup time' before Main Loop needs to run again
    if (execMillis > HX711_PWRUP_MILLIS)
      hx711.power_up();
#endif
  }
  while (execMillis < MAIN_LOOP_MILLIS);
#endif
}

// *****************************************************************************
// Sensors Input Task calculations
// *****************************************************************************
static void readSensorsTask(void)
{
float ax, ay, az;
float mx, my, mz;
uint32_t simCrankTime;

#if BLE_SIMULATION
  return;
#endif

#if SENSOR_USE == SENSOR_GYRO
  if (IMU.gyroscopeAvailable())
  {
    IMU.readGyroscope(ax, ay, GyroZ);
    RawCadence = (uint8_t) ((60.0/360.0) * GyroZ);
  }
  CadenceDeg += ((GyroZ * MAIN_LOOP_MILLIS)/1000);
  if (CadenceDeg >= 360.0)
  {
    CadenceDeg = CadenceDeg - 360.0;
    // One full turn done, update CadenceCount for BLE
    CadenceCount++;
    CadenceTime = CadenceTime + millis();
  }

#else  
  if (IMU.accelerationAvailable() && IMU.magneticFieldAvailable())
  {
    IMU.readAcceleration(ax, ay, az);
    IMU.readMagneticField(mx, my, mz);
#if ACC_FILTER == 1
    // Magneto x and y are opposite of the accelero x and y (on LSM9DS1)
    calculateAttitude(ax, ay, az, -my, -mx, mz);
#else
    AccXFilt = (ACC_FILTER * accX) + ((1-ACC_FILTER) * AccXFilt);
    AccYFilt = (ACC_FILTER * accY) + ((1-ACC_FILTER) * AccYFilt);
    AccZFilt = (ACC_FILTER * accZ) + ((1-ACC_FILTER) * AccZFilt);
    calculateAttitude(AccXFilt, AccYFilt, AccZFilt, -my, -mx, mz);
#endif
  }
  
  // Determine quadrant position based on Roll and Pitch
  if (RollInt < 0 && PitchInt < 0)
    NewCrankState = UP_FORWARD;
  else if (RollInt < 0 && PitchInt > 0)
    NewCrankState = DOWN_FORWARD;
  else if (RollInt > 0 && PitchInt > 0)
    NewCrankState = DOWN_BACKWARD;
  else if (RollInt > 0 && PitchInt < 0)
    NewCrankState = UP_BACKWARD;

  // Filter down the Cadence when timing takes longer than expected
  if (NewCrankState == CrankState)
  {
    simCrankTime = 4 * (millis() - CrankPrev);
    if (RawCadence > 0 && (simCrankTime > ((uint16_t) 60000 / (uint16_t) RawCadence)))
      RawCadence = (uint8_t) ((uint16_t) 60000 / (uint16_t) simCrankTime);
    // Check on timeout value
    if (simCrankTime >= CADENCE_TIMEOUT)
    {
      // Lower than Cadence timeout --> Drop to 0
      RawCadence = 0;
    }
  }

  switch (CrankState)
  {
    case UP_FORWARD:
      if (NewCrankState == DOWN_FORWARD)
      {
        calcRawCadence();
        CrankState = DOWN_FORWARD;
      }
      break;
    case DOWN_FORWARD:
      if (NewCrankState == DOWN_BACKWARD)
      {
        calcRawCadence();
        CrankState = DOWN_BACKWARD;
      }
      break;
    case DOWN_BACKWARD:
      if (NewCrankState == UP_BACKWARD)
      {
        calcRawCadence();
        CrankState = UP_BACKWARD;
      }
      break;
    case UP_BACKWARD:
      if (NewCrankState == UP_FORWARD)
      {
        calcRawCadence();
        // One full turn done, update CadenceCount for BLE
        CadenceCount++;
        CadenceTime = CadenceTime + CrankTime;
        CrankTime = 0;
        CrankState = UP_FORWARD;
      }
      break;
    default:
      // Undefined, e.g. after init, any state next
      CrankState = NewCrankState;
      break;
  }
#endif
}

// *****************************************************************************
// Calculate the raw cadence in a quadrant change event
// *****************************************************************************
static void calcRawCadence(void)
{
uint32_t crankMillis;
uint32_t crankTime;

  // Quadrant change, capture timing
  crankMillis = millis();
  crankTime = crankMillis - CrankPrev;
  CrankPrev = crankMillis;
  // 90 deg * 4 = one turn, convert to seconds
  RawCadence = (uint8_t) (60000 / (crankTime * 4));
  CrankTime = CrankTime + crankTime;
}

// *****************************************************************************
// Turn system off, only to be recovered by a reset, or dedicated event
// *****************************************************************************
static void systemOff(void)
{
#if DEBUG == 1
  Serial.println("Entering power down mode");
#endif

  // Close IMU and BLE modules
  IMU.end();
  BLE.end();
  
  // Leds off
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_PWR, LOW);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);

  // And save power by powering down the HX711 and Strain Gauge measurement
  //hx711.power_down();

  // Enter system OFF, only to be recovered by a reset 
  NRF_POWER->SYSTEMOFF = 1;
}

// *****************************************************************************
// Calculate and update the Battery Level
// *****************************************************************************
static void updateBatteryLevel(void)
{
int32_t batteryLevel;

  batteryLevel = map(BatteryLife, 0, BATT_MAX_LIFETIME, 100, 0);
  // if the battery level has changed
  if (CentralConnected && (batteryLevel != OldBatteryLevel))
  {
#if DEBUG==1
    Serial.print("Battery Level % is now: ");
    Serial.println(batteryLevel);
#endif
    // Update the battery level characteristic value
    BatteryLevelChar.writeValue(batteryLevel);
    OldBatteryLevel = batteryLevel;
  }
}

// *****************************************************************************
// BLE connect handler
// *****************************************************************************
static void BLEPeripheralConnectHandler(BLEDevice central)
{
  // Central connected event handler
  CentralConnected = true;
#if DEBUG == 1
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
#endif
}

// *****************************************************************************
// BLE disconnect handler
// *****************************************************************************
static void BLEPeripheralDisconnectHandler(BLEDevice central)
{
  // Central disconnected event handler
  CentralConnected = false;
#if DEBUG == 1
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
#endif
}

// *****************************************************************************
// NVMEM routines to read/write the configuration to/from non volatile memory
// *****************************************************************************
static void nvmemGetConfig(void)
{
int16_t res;
int16_t i;
int32_t templ;
size_t  actualSize;

  res = kv_get("/kv/check_valid", KvBuf, sizeof(KvBuf), &actualSize);
#if DEBUG == 1
  Serial.println (KvBuf);
#endif
  if (res != MBED_SUCCESS)
  {
    // Reset the nvmem storage when not yet initialized
    res = kv_reset("/kv/");
#if DEBUG == 1
    Serial.println("KV: Could not get check_valid");
#endif
    res = kv_set("/kv/check_valid", CheckValidStr, strlen(CheckValidStr), 0);
    res = kv_set("/kv/cal_offset", "0", 2, 0);
    res = kv_set("/kv/bat_life", "0", 2, 0);
  }
  else if (strcmp(KvBuf, CheckValidStr) == 0)
  {
    // Read calibration offset
    res = kv_get("/kv/cal_offset", KvBuf, sizeof(KvBuf), &actualSize);
    Hx711SensorOffset = atol(KvBuf);
#if DEBUG == 1
    Serial.println("KV: store is valid");
    sprintf(PrintBuf, "Hx711SensorOffset = %d", Hx711SensorOffset);
    Serial.println(PrintBuf);
#endif
    // Read battery life
    res = kv_get("/kv/bat_life", KvBuf, sizeof(KvBuf), &actualSize);
    BatteryLife = atol(KvBuf);
#if DEBUG == 1
    sprintf(PrintBuf, "BatteryLife = %d", BatteryLife);
    Serial.println(PrintBuf);
#endif
  }
}

// *****************************************************************************
// EEPROM routines to read/write the configuration to/from non volatile memory
// *****************************************************************************
static void nvmemSetConfig(void)
{
int16_t res;

  sprintf(PrintBuf, "%ld", Hx711SensorOffset);
  res = kv_set("/kv/cal_offset", PrintBuf, strlen(PrintBuf), 0);
  sprintf(PrintBuf, "%ld", BatteryLife);
  res = kv_set("/kv/bat_life", PrintBuf, strlen(PrintBuf), 0);
}

// *****************************************************************************
// Calculate Roll, Pitch and Yaw
// *****************************************************************************
void calculateAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
  Roll = atan2(ay, az);
  Pitch = atan2(-ax, sqrt(ay * ay + az * az));

 if (my == 0)
    Yaw = (mx < 0) ? PI : 0;
  else
    Yaw = atan2(mx, my);

  Yaw -= DECLINATION * PI / 180;

  if (Yaw > PI) Yaw -= (2 * PI);
  else if (Yaw < -PI) Yaw += (2 * PI);

  // Convert everything from radians to degrees:
  Yaw *= 180.0 / PI;
  Pitch *= 180.0 / PI;
  Roll  *= 180.0 / PI;
  RollInt = (int16_t) Roll;
  PitchInt = (int16_t) Pitch;
}
