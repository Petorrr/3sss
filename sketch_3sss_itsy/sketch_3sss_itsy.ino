// *****************************************************************************
// INCLUDES Section
// *****************************************************************************
#include <Adafruit_DotStar.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#include <bluefruit.h>
#include "src\adxl362.h"

// *****************************************************************************
// DEFINITIONS Section
// *****************************************************************************
// Versioning
#define SW_MAIN_REVISION    0
#define SW_SUB_REVISION     1

// Main Loop timing definitions
#define MAIN_LOOP_MILLIS    10
#define MAIN_TICKS_PER_SEC  (1000/MAIN_LOOP_MILLIS)
#define MSEC_TO_TICKS(x)    (uint16_t) (((uint32_t) (x)*(uint32_t) MAIN_TICKS_PER_SEC)/(uint32_t) 1000)
#define INACTIVE_TIMEOUT    (5 * 60)

#define MAIN_LOOP_DELAY     0
#define MAIN_LOOP_WFI       1
#define MAIN_LOOP_SETPOWER  MAIN_LOOP_WFI

// Debug settings definitions
#define DEBUG               1
//#define BLE_SIMULATION      1

// There is only one DotStar pixel on the board
#define NUMPIXELS           1 

// Digital inputs
#define POWER_DOWN_BUTTON   (4)
//#define AWAKE_INPUT         (PIN_A5)

// ADXL362 interface
#define ACTIVITY_G          500     // in milli G
#define ACTIVITY_TIME       150     // in 100 Hz ticks: 1.5 sec
#define INACTIVITY_G        50      // in milli G
#define INACTIVITY_TIME     50      // in 100 Hz ticks: 0.5 sec
#define ADXL362_CS_PIN      (2)

// Attitude calculations
#define R_XYZ_SEQ           0
#define R_YXZ_SEQ           1
#define R_SEQUENCE          R_XYZ_SEQ

// Cadence defintions
#define ACC_FILTER          (1)
#define UP_FORWARD          1
#define DOWN_FORWARD        2
#define DOWN_BACKWARD       3
#define UP_BACKWARD         4

// *****************************************************************************
// GLOBAL Variables Section
// *****************************************************************************
// Main loop variables
static uint16_t MainCount = 0;
static uint16_t BatteryVoltage = 0;
static uint8_t  BatteryStatus;
static uint32_t TotalSeconds = 0;
static uint32_t InactiveSeconds = 0;

/* Power Meter Service Definitions
 * Cycling Power Service:  0x1818
 * Cycling Power Measurement Char: 0x2A63
 * Cycling Power Feature Char:   0x2A65
 */
BLEService        cps  = BLEService(UUID16_SVC_CYCLING_POWER);
BLECharacteristic cpmc = BLECharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
BLECharacteristic cpfc = BLECharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE);
BLECharacteristic slc  = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

static uint8_t CpmcData[8];

// DotStar LED
static Adafruit_DotStar DotStar(NUMPIXELS, PIN_DOTSTAR_DATA, PIN_DOTSTAR_CLOCK, DOTSTAR_BRG);
static uint32_t LedColor = 0xFF0000;

// ADXL362 interface / IMU variables
static ADXL362  Adxl;
static int16_t  Roll;
static int16_t  Pitch;
static int16_t  Yaw;
static float    AccXFilt;
static float    AccYFilt;
static float    AccZFilt;

// Flash file system variables
static Adafruit_FlashTransport_QSPI FlashTransport;
static Adafruit_SPIFlash            Flash(&FlashTransport);
static FatFileSystem                Fatfs;

// Helper routine variables
static uint16_t  Power = 0;
static uint16_t  CadenceCount = 0;
static uint32_t  CadenceTime = 0;
static uint32_t  CadencePrev = 0;
static uint8_t   CrankState = 0;
static uint8_t   NewCrankState = 0;
static uint32_t  CrankTime = 0;
static uint32_t  CrankPrev = 0;
static uint8_t   RawCadence = 0;

// Buffer varables
static char PrintBuf [80];

// *****************************************************************************
// Local Function prototypes 
// *****************************************************************************
static void handleDelayScenario(uint32_t startMillis);
static void readSensorsTask();
static void calcRawCadence(void);
static void systemOff(void);
static void calculateAttitude(float ax, float ay, float az, float mx, float my, float mz);

static void setupCPS(void);
static void startAdv(void);
static void connect_callback(uint16_t conn_handle);
static void disconnect_callback(uint16_t conn_handle, uint8_t reason);
static void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value);

// *****************************************************************************
// Initialization and Board setup routines
// *****************************************************************************
void setup()
{
uint8_t temp;

  // Initialize the SPI and ADXL interfaces first
  pinMode(ADXL362_CS_PIN, OUTPUT);
  digitalWrite(ADXL362_CS_PIN, 1);
  Adxl.begin(ADXL362_CS_PIN);
  // Map INT1 to inactive to prevent further RESET
  Adxl.SPIwriteOneRegister(0x2A, 0x00); // Write new reg value 

  Serial.begin(115200);
  // Wait for native USB to be ready, max 3 seconds timeout
  while (!Serial && millis() < 3000)
    ;
#if DEBUG
  sprintf(PrintBuf,"3SSS PWR %1d.%02d", SW_MAIN_REVISION, SW_SUB_REVISION);
  Serial.println(PrintBuf);
#endif

  // Initialize DotStar RGB LED
  DotStar.begin();
  DotStar.setBrightness(5);
  // Turn all LEDs off ASAP
  DotStar.show();

  // Initialize Digital inputs
  pinMode(POWER_DOWN_BUTTON, INPUT_PULLUP);
  //pinMode(AWAKE_INPUT, INPUT_PULLUP);
  
#if DEBUG
  Serial.print("ADXL Dev Id1: 0x"); Serial.println(Adxl.SPIreadOneRegister(0x00), HEX);
  Serial.print("ADXL Dev Id2: 0x"); Serial.println(Adxl.SPIreadOneRegister(0x01), HEX);
  Serial.print("ADXL Part Id: 0x"); Serial.println(Adxl.SPIreadOneRegister(0x02), HEX);
  Serial.print("ADXL SRev Id: 0x"); Serial.println(Adxl.SPIreadOneRegister(0x03), HEX);
#endif
  // After setting thresholds, start the measurement mode
  Adxl.beginMeasure();

  // Initialize the Flash File system
  if (Flash.begin())
    Serial.println("Initialize flash chip OK");
  Serial.print("Flash chip JEDEC ID: 0x"); Serial.println(Flash.getJEDECID(), HEX);
  Serial.print("Flash size: "); Serial.println(Flash.size());

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (Fatfs.begin(&Flash))
    Serial.println("Mounted filesystem!");
  
  // Check if a info_uf2 exists and print it out.
  if (Fatfs.exists("boot_out.txt"))
  {
    File info = Fatfs.open("boot_out.txt", FILE_READ);
    Serial.println("Printing config file...");
    while (info.available())
    {
      char c = info.read();
      Serial.print(c);
    }
    Serial.println();
  }
  else
  {
    Serial.println("No config file found...");
  }

  // Initialise the Bluefruit module
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();

  // Set the advertised device name (keep it short!)
  Serial.println("Setting Device Name to '3SSS CPS'");
  Bluefruit.setName("3SSS CPS");

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("3SSS");
  bledis.setModel("3SSS Power");
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  Serial.println("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

  // Setup the Cycling Power service using
  // BLEService and BLECharacteristic classes
  Serial.println("Configuring the Cycling Power Service");
  setupCPS();

  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising payload(s)");
  startAdv();

  Serial.println("\nAdvertising");
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
  readSensorsTask();
  
  // Maintain Total Seconds counter, blink BLE LED shortly
  if ((MainCount % MSEC_TO_TICKS(1000)) == 0)
  {
    // Increment Seconds counters
    TotalSeconds++;
    InactiveSeconds++;
#if DEBUG
    if (Bluefruit.connected())
    {
      // BLE is active; blink Blue Led only every second shortly
      //digitalWrite(LED_BUILTIN, LOW);
      //delayMicroseconds(200);
      // Turn off LED indicator
      //digitalWrite(LED_BUILTIN, HIGH);
    }
    // Rainbow LED when in Debug
    //DotStar.setPixelColor(0, LedColor);
    LedColor = LedColor >> 1;
    if (LedColor == 0)
      LedColor = 0xFF0000;
    //DotStar.show();
#endif    
  }
  
#if BLE_SIMULATION
  // Simulate Power and Cadence values
  Power = 200 + random (-25, 100);
  CadenceTime = CadenceTime+2000;
  CadenceCount += random(2,5);
#endif
  Power = 200 + random (-25, 100);
 
  if ((MainCount % MSEC_TO_TICKS(250)) == 0)
  {
#if DEBUG
    sprintf(PrintBuf, "state=%1d, raw = %3d, cnt=%4d, tim=%d", CrankState, RawCadence, CadenceCount, CadenceTime);
    //sprintf(PrintBuf, "%4d, %4d, %4d", (int16_t) Roll, (int16_t) Pitch, (int16_t) Yaw);
    //sprintf(PrintBuf, "%4d, %4d", (int16_t) Roll, (int16_t) Pitch);
    //sprintf(PrintBuf, "%4d, %4d, %4d", Adxl.readXData(), Adxl.readYData(), Adxl.readZData());
    //sprintf(PrintBuf, "%4d, %4d, %4d", (int16_t) (AccXFilt*1000), (int16_t) (AccYFilt*1000), (int16_t) (AccZFilt*1000));
    //sprintf(PrintBuf, "rs=%4d, ps=%4d", RollSpeed, PitchSpeed);
    Serial.println(PrintBuf);
#endif
    if (Bluefruit.connected())
    {
      CpmcData[0] = 0b00101000;
      CpmcData[1] = 0b00000000;
      CpmcData[2] = lowByte(Power);
      CpmcData[3] = highByte(Power);
      CpmcData[4] = lowByte(CadenceCount);
      CpmcData[5] = highByte(CadenceCount);
      cadenceVal = (uint16_t) ((uint32_t) 1024 * CadenceTime / (uint32_t) 1000);
      CpmcData[6] = lowByte(cadenceVal);
      CpmcData[7] = highByte(cadenceVal);
      
      // Note: We use .notify instead of .write!
      // If it is connected but CCCD is not enabled
      // The characteristic's value is still updated although notification is not sent
      if (cpmc.notify(CpmcData, sizeof(CpmcData)))
      {
        Serial.print("Power updated to: "); Serial.println(Power);
        Serial.print("Cadence Count updated to: "); Serial.println (CadenceCount); 
        Serial.print("Cadence Time updated to: "); Serial.println(cadenceVal);
      }
      else
      {
        Serial.println("ERROR: Notify not set in the CCCD or not connected!");
      }
    }
  }

    // Check for SystemOff event
  if (digitalRead(POWER_DOWN_BUTTON) == 0)
    systemOff();
    
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
static void readSensorsTask()
{
float accX, accY, accZ;

#if BLE_SIMULATION
  return;
#endif

  // Get acceleration data
  accX = (float) Adxl.readXData() / (float) 1000;
  accY = (float) Adxl.readYData() / (float) 1000;
  accZ = (float) Adxl.readZData() / (float) 1000;
#if ACC_FILTER == 1
  calculateAttitude(accX, accY, accZ, 0, 0, 0);
#else
  AccXFilt = (ACC_FILTER * accX) + ((1-ACC_FILTER) * AccXFilt);
  AccYFilt = (ACC_FILTER * accY) + ((1-ACC_FILTER) * AccYFilt);
  AccZFilt = (ACC_FILTER * accZ) + ((1-ACC_FILTER) * AccZFilt);
  calculateAttitude(AccXFilt, AccYFilt, AccZFilt, 0, 0, 0);
#endif

  // Determine quadrant position based on Roll and Pitch
  if (Roll < 0 && Pitch < 0)
    NewCrankState = UP_FORWARD;
  else if (Roll < 0 && Pitch > 0)
    NewCrankState = DOWN_FORWARD;
  else if (Roll > 0 && Pitch > 0)
    NewCrankState = DOWN_BACKWARD;
  else if (Roll > 0 && Pitch < 0)
    NewCrankState = UP_BACKWARD;

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
        CrankState = UP_FORWARD;
      }
      break;
    default:
      // Undefined, e.g. after init, any state next
      CrankState = NewCrankState;
      break;
  }
}

// *****************************************************************************
// Calculate the raw cadence in a quadrant change event
// *****************************************************************************
static void calcRawCadence(void)
{
uint32_t crankMillis;

  // Quadrant change, capture timing
  crankMillis = millis();
  CrankTime = crankMillis - CrankPrev;
  CrankPrev = crankMillis;
  // 90 deg * 4 = one turn, convert to seconds
  RawCadence = (uint8_t) (60000 / (CrankTime * 4));
  // Add this quadrant time to the Cadence Cumulative time
  CadenceTime = CadenceTime + CrankTime;
}

// *****************************************************************************
// Turn system off, only to be recovered by a reset, or dedicated event
// *****************************************************************************
static void systemOff(void)
{
uint8_t temp;

#if DEBUG
  Serial.println("Entering power down mode");
#endif

  // Inactive mode now, safe to setup INT1 mapping
  // Setup ADXL thresholds for a movement RESET; ACTIVITY_TIME active low, ACTIVITY_TIME sec high
  // Should remain high after reset, so immediately disable INT1
  Adxl.setupACInactivityInterrupt(INACTIVITY_G, INACTIVITY_TIME);
  Adxl.setupACActivityInterrupt(ACTIVITY_G, ACTIVITY_TIME);
  // Turn on Loop Mode
  temp = Adxl.SPIreadOneRegister(0x27); // Read current reg value
  temp = temp | (0x30);                 // turn on Loop bits  
  Adxl.SPIwriteOneRegister(0x27, temp); // Write new reg value 

  // First read status register and wait for !AWAKE bit
  do
  {
    temp = Adxl.SPIreadOneRegister(0x0B);
    Serial.print("Sts=0x"); Serial.println(temp, HEX);
    temp = temp & 0x40;
    delay(10);
  }
  while (temp != 0);
    
  // Map INT1 to AWAKE bit, which is now High (inverted to Reset)
  temp = Adxl.SPIreadOneRegister(0x2A); // Read current reg value
  temp = temp | (0xC0);                 // Map INT1 to AWAKE, Inverted Low active
  Adxl.SPIwriteOneRegister(0x2A, temp); // Write new reg value 

  // Leds off
  digitalWrite(LED_BUILTIN, LOW);
  DotStar.setPixelColor(0, 0x00);
  DotStar.show();
  DotStar.setBrightness(0);

  for (temp = 0; temp < PINS_COUNT; temp++)
  {
    pinMode(temp, OUTPUT);
    digitalWrite(temp, LOW);
  }
  FlashTransport.runCommand(0xB9);
  
  // And save power by powering down the HX711 and Strain Gauge measurement
  //hx711.power_down();

  // Enter system OFF, only to be recovered by a reset 
  NRF_POWER->SYSTEMOFF = 1;
}

// *****************************************************************************
// Calculate Roll, Pitch and Yaw
// *****************************************************************************

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -1.67 // Declination (degrees) in Hoofddorp

static void calculateAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
float roll, pitch, yaw;

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf

#if R_SEQUENCE == R_XYZ_SEQ
  roll  = atan2(ay, az);
  pitch = atan2(-ax, sqrt(ay * ay + az * az));
#else
  pitch = atan2(-ax, az);
  roll  = atan2(ay, sqrt(ax * ax + az * az));
#endif

 if (my == 0)
    yaw = (mx < 0) ? PI : 0;
  else
    yaw = atan2(mx, my);

  yaw -= DECLINATION * PI / 180;

  if (yaw > PI) yaw -= (2 * PI);
  else if (yaw < -PI) yaw += (2 * PI);

  // Convert everything from radians to degrees:
  Yaw = (int16_t) (yaw * 180.0 / PI);
  Pitch = (int16_t) (pitch * 180.0 / PI);
  Roll  = (int16_t) (roll * 180.0 / PI);
}

// *****************************************************************************
// Setup BLE services
// *****************************************************************************
static void setupCPS(void)
{
  // Configure the Cycling Power service
  // See: https://www.bluetooth.com/xml-viewer/?src=https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Services/org.bluetooth.service.cycling_power.xml
  // Supported Characteristics:
  // Name                         UUID    Requirement Properties
  // ---------------------------- ------  ----------- ----------
  // Cycling Power Feature        0x2A65  Mandatory   Read
  // Cycling Power Measurement    0x2A63  Mandatory   Notify
  // Sensor Location              0x2A5D  Mandatory   Read
  
  cps.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  // Configure the Cycling Power Feature characteristic
  // See: https://www.bluetooth.com/xml-viewer/?src=https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.cycling_power_feature.xml
  // Properties = Read
  // Min Len    = 4
  // Max Len    = 4
  //    B0      = UINT32  - Flags 0-7 (MANDATORY)
  //      b7    = Accumulated Energy Supported (0 = false, 1 = true)
  //      b6    = Top and Bottom Dead Spot Angles Supported (0 = false, 1 = true)
  //      b5    = Extreme Angles Supported (0 = false, 1 = true)
  //      b4    = Extreme Magnitudes Supported (0 = false, 1 = true)
  //      b3    = Crank Revolution Data Supported (0 = false, 1 = true)
  //      b2    = Wheel Revolution Data Supported (0 = false, 1 = true)
  //      b1    = Accumulated Torque Supported (0 = false, 1 = true)
  //      b0    = Pedal Power support (0 = false, 1 = true)
  //
  //    B1      = UINT32  - Flags 8-15 (MANDATORY)
  //      b7    = Span Length Adjustment Supported (0 = false, 1 = true)
  //      b6    = Chain Weight Adjustment Supported (0 = false, 1 = true)
  //      b5    = Chain Length Adjustment Supported (0 = false, 1 = true)
  //      b4    = Crank Length Adjustment Supported (0 = false, 1 = true)
  //      b3    = Multiple Sensor Locations Supported (0 = false, 1 = true)
  //      b2    = Cycling Power Measurement Characteristic Content Masking Supported (0 = false, 1 = true)
  //      b1    = Offset Compensation Supported (0 = false, 1 = true)
  //      b0    = Offset Compensation Indicator Supported (0 = false, 1 = true)
  //
  //    B2      = UINT16  - Flags 16-23 (MANDATORY)
  //      b7    = RFU (0 = false, 1 = true)
  //      b6    = RFU (0 = false, 1 = true)
  //      b4:5  = Distribute System Support (0 = unspecified (legacy), 1 = not supported, 2 = supported, 3 = RFU)
  //      b3    = Enhanced Offset Compensation Supported (0 = false, 1 = true)
  //      b2    = Factory Calibration Date Supported (0 = false, 1 = true)
  //      b1    = Instantaneous Measurement Direction Supported (0 = false, 1 = true)
  //      b0    = Sensor Measurement Context (0 = Force based, 1 = Torque based)
  //
  //    B3      = UINT16  - Flags 24-31 (MANDATORY)
  //      b7    = RFU (0 = false, 1 = true)
  //      b6    = RFU (0 = false, 1 = true)
  //      b5    = RFU (0 = false, 1 = true)
  //      b4    = RFU (0 = false, 1 = true)
  //      b3    = RFU (0 = false, 1 = true)
  //      b2    = RFU (0 = false, 1 = true)
  //      b1    = RFU (0 = false, 1 = true)
  //      b0    = RFU (0 = false, 1 = true)
  
  cpfc.setProperties(CHR_PROPS_READ);
  cpfc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  cpfc.setFixedLen(4);
  //cpfc.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  cpfc.begin();
  uint8_t cpfcdata[4] = { 0b00001001, 0b00000000, 0b00010001, 0b00000000 }; // Set the characteristic to use 4x 8-bit values
  cpfc.write(cpfcdata, 4);

  // Configure the Cycling Power Measurement characteristic
  // See: https://www.bluetooth.com/xml-viewer/?src=https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.cycling_power_measurement.xml
  // Properties = Notify
  // Min Len    = 4
  // Max Len    = 35
  //    B0      = UINT16  - Flags Low (MANDATORY)
  //      b7    = Extreme Torque Magnitudes Present (0 = false, 1 = true)
  //      b6    = Extreme Force Magnitudes Present (0 = false, 1 = true)
  //      b5    = Crank Revolution Data Present (0 = false, 1 = true)
  //      b4    = Wheel Revolution Data Present (0 = false, 1 = true)
  //      b3    = Accumulated Torque Source (0 = Wheel based, 1 = Crank based)
  //      b2    = Accumulated Torque Present (0 = false, 1 = true)
  //      b1    = Pedal Power Balance Reference (0 = unknown, 1 = true)
  //      b0    = Pedal Power Balance present (0 = false, 1 = true)
  //    B1      = UINT16  - Flags High (MANDATORY)
  //      b7    = RFU
  //      b6    = RFU
  //      b5    = RFU
  //      b4    = Offset Compensation Indicator (0 = false, 1 = true)
  //      b3    = Accumulated Energy Present (0 = false, 1 = true)
  //      b2    = Bottom Dead Spot Angle Present (0 = false, 1 = true)
  //      b1    = Top Dead Spot Angle Present (0 = false, 1 = true)
  //      b0    = Extreme Angles Present (0 = false, 1 = true)
  //    B1      = SINT16 - 16-bit Instantaneous Power in Watt(lower 8 bits)
  //    B2      = SINT16 - 16-bit Instantaneous Power in Watt(higher 8 bits)
  //    B3..B16 = NOT USED, Optional data
  
  cpmc.setProperties(CHR_PROPS_NOTIFY);
  cpmc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  cpmc.setFixedLen(8);
  cpmc.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  cpmc.begin();
  uint8_t cpmcdata[8] = { 0b00101000, 0b00000000, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // Set the characteristic values
  cpmc.write(cpmcdata, 8);

  // Configure the Sensor Location characteristic
  // See: https://www.bluetooth.com/xml-viewer/?src=https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.sensor_location.xml
  // Properties = Read
  // Min Len    = 1
  // Max Len    = 1
  //    B0      = UINT8 -  Sensor Location
  //      0     = Other
  //      1     = Top of Shoe
  //      2     = In Shoe
  //      3     = Hip
  //      4     = Front Wheel
  //      5     = Left Crank
  //      6     = Right Crank
  //      7     = Left Pedal
  //      8     = Right Pedal
  //      9     = Front Hub
  //      10    = Rear dropout
  //      11    = Chain stay
  //      12    = Rear wheel
  //      13    = Rear Hub
  //      14    = Chest
  //      15    = Spider
  //      16    = Chain ring
  //      17:255 = Reserved
  slc.setProperties(CHR_PROPS_READ);
  slc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  slc.setFixedLen(1);
  slc.begin();
  slc.write8(5);    // Set the characteristic to 'Left crank'
}

// *****************************************************************************
// Start BLE Advertising
// *****************************************************************************
static void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include CPS Service UUID
  Bluefruit.Advertising.addService(cps);

  // Include Name
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

/**
 * Callback invoked when a connection is established
 * @param conn_handle connection where this event happens
 */
static void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
static void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  Serial.println("Advertising!");
}

static void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value)
{
    // Display the raw request packet
    Serial.print("CCCD Updated: ");
    //Serial.printBuffer(request->data, request->len);
    Serial.print(cccd_value);
    Serial.println("");

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == cpmc.uuid) {
        if (chr->notifyEnabled(conn_hdl)) {
            Serial.println("CPM 'Notify' enabled");
        } else {
            Serial.println("CPM 'Notify' disabled");
        }
    }
}
