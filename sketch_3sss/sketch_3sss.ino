// *****************************************************************************
// INCLUDES Section
// *****************************************************************************
//#include <Q2HX711.h>
#include <HX711.h>
#include <stdio.h>
#include "src\SoftwareSerial.h"

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
#define ANT_RX_MAX_TIMEOUT            100 // In milliseconds
// ANT Transmit Message definitions
#define MSG_TX_SYNC                   ((byte)0xA4)
#define MSG_SYSTEM_RESET_ID           ((byte)0x4A)
#define MSG_NETWORK_KEY_ID            ((byte)0x46)
#define MSG_ASSIGN_CHANNEL_ID         ((byte)0x42)
#define MSG_CHANNEL_ID_ID             ((byte)0x51)
#define MSG_CHANNEL_RADIO_FREQ_ID     ((byte)0x45)
#define MSG_CHANNEL_MESG_PERIOD_ID    ((byte)0x43)
#define MSG_RADIO_TX_POWER_ID         ((byte)0x47)
#define MSG_CHANNEL_SEARCH_TIMEOUT_ID ((byte)0x44)
#define MSG_OPEN_CHANNEL_ID           ((byte)0x4B)
#define MSG_BROADCAST_DATA_ID         ((byte)0x4E)
#define MSG_CHANNEL_RESPONSE          ((byte)0x40)
#define MSG_TX_EVENT                  ((byte)0x03)

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
static byte MainCount = 0;

// Piezo sensor variables
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
static HX711 hx711 = HX711(hx711_data_pin, hx711_clock_pin, 128);

// ANT communication data
static SoftwareSerial AntSerial = SoftwareSerial(AP2_RX_PIN, AP2_TX_PIN, false);
// ANT Developer key, if you want to connect to ANT+, you must get the key from thisisant.com
static const byte     NETWORK_KEY[] = {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45};
static byte           AntPedalPower;
static byte           AntCadence;
static word           AntAccuPower;
static word           AntPower;

// ANT message processor
static boolean        MsgSync = false;
static byte           MsgIndex = 0;
static byte           MsgLength = 0;
static byte           MsgBuf [ANT_RXBUF_SIZE];
static byte           SendCount = 0;

// *****************************************************************************
// Local Function prototypes 
// *****************************************************************************
static void readSensorsTask();

static void broadcastBikePower (void);
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
  // Stabilize for a while
  delay (10);
  // Issue an AP2 hard reset
  AP2reset ();
  // Setup and initialize the ANT network
  antSetup ();

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
uint32_t startMillis;
word     execMillis;

  // Main loop for serial output and debugging tasks:
  startMillis = millis();
  MainCount++;
  // Toggle Run Led every 500 msec
  if (MainCount % 10 == 0)
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
  AntCadence = 93;
  AntPower = 300 + random (-25, 100);
  AntAccuPower += AntPower;
  //sprintf (TmpBuf, "Pdl %u; Cdn %u; TotPwr %u Pwr: %u", AntPedalPower, AntCadence, AntAccuPower, AntPower);
  //Serial.println (TmpBuf);
  if ((MainCount % 5) == 0)
    broadcastBikePower();

  // Run ANT RX parser check with no timeout for incoming requests
  ANTreceive (0);

  // Run this task at 20 Hz
  execMillis = millis() - startMillis;
  if (execMillis < 50)
    delay (50-execMillis);
}


// *****************************************************************************
// Sensors Input Task samples at 10 msec
// *****************************************************************************
static void readSensorsTask()
{
uint32_t startMillis;
word     execMillis;

  startMillis = millis();

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
  execMillis = millis() - startMillis;
  if (execMillis < 10)
    delay (10-execMillis);
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
  ANTsetNetwork ();
  ANTreceive (ANT_RX_MAX_TIMEOUT);

  // Assign ANT Channel as bi-directional transmitter (Master)
  ANTassignChannel (0, CHANNEL_TYPE_BIDIRECTIONAL_TRANSMIT);
  ANTreceive (ANT_RX_MAX_TIMEOUT);

  // Set Channel ID to Bike Power profile
  ANTsetChannelId (ANT_CHANNEL_NR, (word) ANT_DEVICE_NR, BIKE_POWER_PROFILE, BIKE_POWER_TT);
  ANTreceive (ANT_RX_MAX_TIMEOUT);

  // Set frequency for profile to 4Hz updates
  ANTsetFrequency (ANT_CHANNEL_NR, BIKE_POWER_RF);
  ANTreceive (ANT_RX_MAX_TIMEOUT);

  // Set ANT period
  ANTsetPeriod (ANT_CHANNEL_NR, BIKE_POWER_CP);
  ANTreceive (ANT_RX_MAX_TIMEOUT);

  // Open the ANT channel for communication broadcasting
  ANTopenChannel (ANT_CHANNEL_NR);
  ANTreceive (ANT_RX_MAX_TIMEOUT);
}

// *****************************************************************************
// ANT stack message parsing
// *****************************************************************************
static void broadcastBikePower (void)
{
uint8_t buf[13];

  // Fill ANT Bike Power data buffer with data
  buf[0] = MSG_TX_SYNC;
  buf[1] = 0x09;
  buf[2] = MSG_BROADCAST_DATA_ID;
  buf[3] = ANT_CHANNEL_NR;
  buf[4] = 0x10;
  buf[5] = MainCount;
  buf[6] = AntPedalPower;
  buf[7] = AntCadence;
  buf[8] = lowByte (AntAccuPower);
  buf[9] = highByte (AntAccuPower);
  buf[10] = lowByte (AntPower);
  buf[11] = highByte (AntPower);
  buf[12] = checkSum(buf, 12);
  ANTsend (buf,13);

  ANTreceive (ANT_RX_MAX_TIMEOUT);
}

// *****************************************************************************
// ANT AP2 hard reset function
// *****************************************************************************
static void AP2reset (void)
{
  // Issue and AP2 hard reset
  digitalWrite(AP2_RST_PIN, 0);
  delayMicroseconds (100);
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
    delayMicroseconds (50); 
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
  retry = (uint32_t) (timeout*10);
  while (sbuflength == 0 && retry > 0)
  {
    sbuflength = AntSerial.available();
    if (sbuflength == 0)
      delayMicroseconds (100);
    retry--;
  }
    
  while (sbuflength > 0 && MsgIndex < ANT_RXBUF_SIZE)
  {
    //Serial.print("sbuf: ");
    //Serial.print(sbuflength);
    //Serial.print(" msg: ");
    //Serial.print (AntSerial.peek(), HEX);
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
        for (int i = 0 ; i < MsgLength ; i++)
        {
          Serial.print(MsgBuf[i], HEX);
          Serial.print(" ");
        }
        Serial.println(" ");
        // Call the message parser for further action
        ANTrxProcess ();
        MsgSync = false;
      }
    }
 
    // Wait for next character to parse (can be too fast), max 2 ms (slowest ~4800 baud)
    if (AntSerial.available() == 0)
      delayMicroseconds (2000);
  
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
        digitalWrite(LED_BLUE, !digitalRead (LED_BLUE));
        Serial.println("TX success");
#if 0
        if (SendCount < 4)
        {
          //broadcastBikePower();
          SendCount++;
          //Serial.println("Crank Torque");
        }
        else if (SendCount == 4)
        {
          //broadcastBikePower();
          //cranktorque();
          SendCount = 0;
          //Serial.println("Basic Power");
        }
#endif        
      }
    }
    else
    {
      switch (MsgBuf[4])
      {
        case MSG_NETWORK_KEY_ID:
          Serial.print ("Set Network Key: ");
        break;
  
        case MSG_ASSIGN_CHANNEL_ID:
          Serial.print ("Assign Channel: ");
        break;
        
        case MSG_CHANNEL_ID_ID:
          Serial.print ("Set Channel ID: ");
        break;
        
        case MSG_CHANNEL_RADIO_FREQ_ID:
          Serial.print ("Set Frequency: ");
        break;
       
        case MSG_CHANNEL_MESG_PERIOD_ID:
          Serial.print ("Set Period: ");
        break;
       
        case MSG_RADIO_TX_POWER_ID:
          Serial.print ("Set Tx Power: ");
        break;
       
        case MSG_CHANNEL_SEARCH_TIMEOUT_ID:
          Serial.print ("Set Timout: ");
        break;
      
        case MSG_OPEN_CHANNEL_ID:
          Serial.print ("Open Channel: ");
        break;
        
        default:
          Serial.print ("Unknown Msg: ");
        break;
      }
      
      if (MsgBuf[5] == 0)
        Serial.println("No Error");
      else
      {
        Serial.print ("ERR: ");
        Serial.println(MsgBuf[5], HEX);
      }  
    }
  }
}

// *****************************************************************************
// ANT AP2 send routine(s)
// *****************************************************************************
static void ANTsend (uint8_t buf[], int length)
{
 int i;
 
   Serial.print ("ANTsend:");
   // Send length buffer bytes to AntSerial
   for (i = 0 ; i < length ; i++)
   {
     Serial.print(buf[i], HEX);
     Serial.print(" ");
     AntSerial.write (buf[i]);
   }
   // Wait for Clear to Send signal
   AP2waitCts ();
   Serial.println("");
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
   buf[4] = checkSum (buf,4);
   ANTsend (buf,5);
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
      buf [4+i] = NETWORK_KEY[i];
    buf[12] = checkSum(buf, 12);
    ANTsend (buf,13);
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
    ANTsend (buf,7);
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
    ANTsend (buf,9);
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
    ANTsend (buf,6);
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
    ANTsend (buf,7);
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
    ANTsend (buf,5);
}
