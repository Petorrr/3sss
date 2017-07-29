// *****************************************************************************
// INCLUDES Section
// *****************************************************************************
//#include <Q2HX711.h>
#include <HX711.h>
#include <stdio.h>
#include "src\SoftwareSerial.h"
#include <ANT.h>

// *****************************************************************************
// DEFINITIONS Section
// *****************************************************************************
#define STRING_BUF_SIZE     80
#define PIEZO_BUF_SIZE      20
#define HX711_BUF_SIZE      20
#define HX711_DATA_PIN      (7)
#define HX711_CLOCK_PIN     (11)
#define ANA_RES_BITS        14
#define ANA_INP_RANGE       16384
#define ANA_VOLT_REF        (float) 3.6

// ANT AP2 Interface definitions
#define AP2_TX_PIN          (4)
#define AP2_RX_PIN          (5)
#define AP2_BR1_PIN         (28)
#define AP2_BR2_PIN         (29)
#define AP2_BR3_PIN         (12)
#define AP2_RST_PIN         (13)
#define AP2_SUSPEND_PIN     (14)
#define AP2_SLEEP_PIN       (16)
#define AP2_RTS_PIN         (15)

// ANT Bike Power Profile definitions
#define BIKE_POWER_PROFILE  0x0B
#define BIKE_POWER_DEVICE   0x0001
#define BIKE_POWER_RF       57
#define BIKE_POWER_TT       0x05
#define BIKE_POWER_CP       8182


// *****************************************************************************
// GLOBAL Variables Section
// *****************************************************************************
static char TmpBuf [STRING_BUF_SIZE];
static byte MainCount = 0;

static word PiezoBuf [PIEZO_BUF_SIZE];
static byte PiezoSensorCh = A0;
static byte PiezoIndex = 0;
static word PiezoSensorVal = 0;

// Loadcell, Strain Gauge Amplifier variables
const byte  hx711_data_pin = HX711_DATA_PIN;
const byte  hx711_clock_pin = HX711_CLOCK_PIN;
static long Hx711Buf [HX711_BUF_SIZE];
static byte Hx711Index = 0;
static long Hx711SensorVal = 0;
// Construct the Loadcell Amplifier object
//Q2HX711 hx711(hx711_data_pin, hx711_clock_pin);
static HX711 hx711(hx711_data_pin, hx711_clock_pin, 128);

// ANT communication data
static SoftwareSerial AntSerial (AP2_RX_PIN, AP2_TX_PIN, false);
static Ant            ant = Ant();
// ANT Developer key, if you want to connect to ANT+, you must get the key from thisisant.com
static const byte     NETWORK_KEY[] = {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45};
static byte           AntBuf [8] = {0, 0, 0, 0, 0, 0, 0, 0};
static byte           AntPedalPower;
static byte           AntCadence;
static word           AntAccuPower;
static word           AntPower;

// *****************************************************************************
// Local Function prototypes 
// *****************************************************************************
static void antSetup();
static void readSensorsTask();
static void parseAntMessage() ;
static void parseEventMessage(uint8_t code);
static void AP2reset (void);

// *****************************************************************************
// Initialization and Board setup routines
// *****************************************************************************
void setup()
{
byte i;

  // Put setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  Serial.begin(115200);
  Serial.println ("Hello 3sss");

  // Initialize the AP2 interface pins
  pinMode (AP2_BR1_PIN, OUTPUT);
  pinMode (AP2_BR2_PIN, OUTPUT);
  pinMode (AP2_BR3_PIN, OUTPUT);
  digitalWrite(AP2_RST_PIN, 1);
  pinMode (AP2_RST_PIN, OUTPUT);
  pinMode (AP2_SUSPEND_PIN, OUTPUT);
  pinMode (AP2_SLEEP_PIN, OUTPUT);
  pinMode (AP2_RTS_PIN, INPUT);
  
  // Baudrate selection to 38400 baud
  digitalWrite(AP2_BR1_PIN, 1);
  digitalWrite(AP2_BR2_PIN, 0);
  digitalWrite(AP2_BR3_PIN, 0);
  digitalWrite(AP2_RST_PIN, 1);
  digitalWrite(AP2_SLEEP_PIN, 0);
  digitalWrite(AP2_SUSPEND_PIN, 1);
  // Initalize SoftwareSerial
  AntSerial.begin(38400);

  // Issue an AP2 hard reset
  AP2reset ();
  // Wait for 3 seconds to AP2 startup
  Serial.println ("Waiting for reset");
  for (i=0; i < 30; i++)
  {
    delay (100);
    Serial.print (".");
  }
  
  Serial.println ("");
  Serial.println ("Starting antSetup");
  antSetup ();
  Serial.println ("End of Setup");
  if (digitalRead (AP2_RTS_PIN))
    Serial.println ("RTS=1");
   else
    Serial.println ("RTS=0");

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

BroadcastMsg bm;

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
  piezoV = (ANA_VOLT_REF * (float) piezoMax) / (float) ANA_INP_RANGE;
  
  // Output serial data for debugging
  //sprintf (TmpBuf, "Millis since start: %ul msec", millis());
  //sprintf (TmpBuf, "Piezo: %u; hx711 %u; ", (word) piezoMax, (word) Hx711SensorVal);
  //sprintf (TmpBuf, "LC: %ul", Hx711SensorVal & 0x007FFFFF);
  //Serial.println (TmpBuf);
  //Serial.println (Hx711SensorVal & 0x007FFFFF);
  //Serial.println (piezoV);
  //Serial.println (piezoMax);

  // Simulate ANT Power
  AntPedalPower = 50;
  AntCadence = 0xFF;
  AntPower = 300 + random (-25, 100);
  AntAccuPower += AntPower;
  // Fill ANT Bike Power data buffer with data
  AntBuf[0] = 0x10;
  AntBuf[1] = MainCount;
  AntBuf[2] = AntPedalPower;
  AntBuf[3] = AntCadence;
  AntBuf[4] = lowByte (AntAccuPower);
  AntBuf[5] = highByte (AntAccuPower);
  AntBuf[6] = lowByte (AntPower);
  AntBuf[7] = highByte (AntPower);
  // Finally broadcast the ANT Bike Power Data message
  bm = BroadcastMsg();
  bm.setData(AntBuf);
  bm.setChannel(0);
  ant.send(bm);
  sprintf (TmpBuf, "Pdl %u; Cdn %u; TotPwr %u Pwr: %u", AntPedalPower, AntCadence, AntAccuPower, AntPower);
  //Serial.println (TmpBuf);

  // Parse any incoming ANT messages
  parseAntMessage();

  // Run this task at 10 Hz
  delay (100);
  MainCount++;
}


// *****************************************************************************
// Sensors Input Task samples at 50 msec
// *****************************************************************************
static void readSensorsTask()
{
  // Toggle the sensor task LED
  digitalWrite(LED_BLUE, !digitalRead (LED_BLUE));

  // Read the value from the Piezo sensor into the sample buffer
  PiezoSensorVal = analogRead (PiezoSensorCh);
  PiezoBuf [PiezoIndex] = PiezoSensorVal;
  PiezoIndex++;
  if (PiezoIndex >= PIEZO_BUF_SIZE)
    PiezoIndex = 0;

  // Read the value from the HX711 sensor into the sample buffer
  if (hx711.is_ready())
  {
    Hx711SensorVal = hx711.read ();
    Hx711Buf [Hx711Index] = Hx711SensorVal;
    Hx711Index++;
    if (Hx711Index >= HX711_BUF_SIZE)
      Hx711Index = 0;
  }

  // Run this task at 100 Hz
  delay (10);
}

// *****************************************************************************
// ANT stack initialization
// *****************************************************************************
static void antSetup()
{
AssignChannel      ac;
ResetSystem        rs;
SetNetworkKey      snk;
ChannelId          ci;
ChannelPeriod      cp;
ChannelRfFrequency crf;
OpenChannel        oc;

  ant.setSerial(AntSerial);
  //ant.setSerial(Serial);
  
  // Send reset to ANT Network
  ant.send(rs);
  // Delay after resetting the radio to give the user time to connect on serial
  delay(1000);
  Serial.println("ANT Reset");
  parseAntMessage();

  // Set the Network Key (currently to DEVELOPER)
  snk = SetNetworkKey();
  snk.setNetwork(0);
  snk.setKey((uint8_t*)NETWORK_KEY);
  ant.send(snk);
  parseAntMessage();

  // Assign ANT Channel as bi-directional transmitter (Master)
  ac = AssignChannel();
  ac.setChannel(0);
  ac.setChannelType(CHANNEL_TYPE_BIDIRECTIONAL_TRANSMIT); //can't wildcard this
  ac.setChannelNetwork(0);
  ant.send(ac);
  parseAntMessage();

  ci = ChannelId();
  ci.setChannel(0);
  ci.setDeviceNumber(0x0001);
  ci.setDeviceType(BIKE_POWER_PROFILE);
  ci.setTransmissionType(BIKE_POWER_TT);
  ant.send(ci);
  parseAntMessage();

  crf = ChannelRfFrequency();
  crf.setChannel(0);
  crf.setRfFrequency(BIKE_POWER_RF); //can't wildcard this
  ant.send(crf);
  parseAntMessage();

  cp = ChannelPeriod();
  cp.setChannel(0);
  cp.setPeriod(BIKE_POWER_CP); //can't wildcard this
  ant.send(cp);
  parseAntMessage();

  oc = OpenChannel();
  oc.setChannel(0);
  ant.send(oc);
  parseAntMessage();
}

// *****************************************************************************
// ANT stack message parsing
// *****************************************************************************
static void parseAntMessage() 
{
byte msgId;
byte *framePtr;

  ant.readPacket();
  if(ant.getResponse().isAvailable())
  {
    msgId = ant.getResponse().getMsgId();
    switch (msgId) {
      case CHANNEL_EVENT:
      {
        ChannelEventResponse cer = ChannelEventResponse();
        ant.getResponse().getChannelEventResponseMsg(cer);
        Serial.println("Received Msg: ChannelEventResponse");
        Serial.print("Channel: ");
        Serial.println(cer.getChannelNumber());
        parseEventMessage(cer.getCode());
        break;
      }

      case START_UP_MESSAGE:
      {
        StartUpMessage sum = StartUpMessage();
        ant.getResponse().getStartUpMsg(sum);
        Serial.println("Received Msg: StartupMessage");
        Serial.print("Message: ");
        Serial.println(sum.getMessage());
        break;
      }

      default:
        Serial.print("Ant Message: ");
        Serial.println(msgId, HEX);
        framePtr = ant.getResponse().getFrameData();
        sprintf (TmpBuf, "Ant msg: %02X %02X %02X %02X", framePtr[0], framePtr[1], framePtr[2], framePtr[3]);
        Serial.println (TmpBuf);
        
        break;
    }
  }
  else if (ant.getResponse().isError())
  {
    Serial.print("ANT MSG ERROR: ");
    Serial.println(ant.getResponse().getErrorCode());
  }

  delay (10);
}

// *****************************************************************************
// ANT stack event message parsing
// *****************************************************************************
void parseEventMessage(uint8_t code)
{
BroadcastMsg bm;

  Serial.print("Code: ");
  switch (code)
  {
    case RESPONSE_NO_ERROR:
      Serial.println("RESPONSE_NO_ERROR");
      break;

    case EVENT_CHANNEL_CLOSED:
      Serial.println("EVENT_CHANNEL_CLOSED");
      break;

    case EVENT_TX:
      Serial.println("EVENT_TX");
#if 0
      buffer[0]++;
      bm = BroadcastMsg();
      bm.setData(buffer);
      bm.setChannel(0);
      ant.send(bm);
#endif
      break;

    default:
      Serial.println(code);
      break;
  }
}

// *****************************************************************************
// ANT AP2 hard reset function
// *****************************************************************************
static void AP2reset (void)
{
  // Issue and AP2 hard reset
  digitalWrite(AP2_RST_PIN, 0);
  delay (10);
  digitalWrite(AP2_RST_PIN, 1);
}
