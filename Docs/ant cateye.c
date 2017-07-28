#define UCHAR unsigned char
#define MESG_TX_SYNC ((UCHAR)0xA4)
#define MESG_SYSTEM_RESET_ID ((UCHAR)0x4A)
#define MESG_NETWORK_KEY_ID ((UCHAR)0x46)
#define MESG_ASSIGN_CHANNEL_ID ((UCHAR)0x42)
#define MESG_CHANNEL_ID_ID ((UCHAR)0x51)
#define MESG_CHANNEL_RADIO_FREQ_ID ((UCHAR)0x45)
#define MESG_CHANNEL_MESG_PERIOD_ID ((UCHAR)0x43) // Set channel period 0x43
#define MESG_RADIO_TX_POWER_ID ((UCHAR)0x47) // Set Tx Power 0x47
#define MESG_CHANNEL_SEARCH_TIMEOUT_ID ((UCHAR)0x44) // Set Channel Search Timeout 0x44
#define MESG_OPEN_CHANNEL_ID ((UCHAR)0x4B) // ID Byte 0x4B
#define MESG_BROADCAST_DATA_ID ((UCHAR)0x4E)
#include <SoftwareSerial.h>

SoftwareSerial mySerial(8,9); //RX on Pin 8, TX on Pin 9

const int Mag_pickup = 3; // the number of the pushbutton pin
//int pin = 13;
volatile int state = LOW;
unsigned long time1;
unsigned long time2;
unsigned long period;

double omega;
//double RPM;
//double velocity;
double torque_kgm;
double torque_Nm;

byte ANT_event = 0;
uint16_t ANT_INST_power = 0;
uint16_t ANT_power = 0;
uint8_t ANT_icada = 0; // Instant Cadence
uint8_t ANT_icad = 0; // Corrected Cadence
double powerconst;

boolean recalc = 0;

long lastDebounceTime = 0; // the last time the output pin was toggled
long debounceDelay = 30; // the debounce time; increase if the output flickers
int buttonState; // the current reading from the input pin
int lastButtonState = LOW; // the previous reading from the input pin
boolean sent = 0;

void setup()
{
Serial.begin(4800);
mySerial.begin(4800);
//pinMode(pin, OUTPUT);
//attachInterrupt(0, blink, RISING);
delay(5000);
mySerial.flush();
delay(50);
initiate();
delay(20);
}

void loop()
{

Crank_Pickup();

if (recalc == 1)
{
ANT_event++;
period = time1 - time2; // Time to complete one revolution in microseconds
omega = 6283185.3072/period; // Angular velocity rad/s
ANT_icada = uint8_t(omega*9.549296586); // RPM
ANT_icad = uint8_t(ANT_icada - 1.5); //RPM Corrected by subtraction of 1.5
int sensorValue = analogRead(A0); // Read Analog Voltage on A0
float volt= sensorValue * (5.0 / 1023.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
torque_kgm=0.3413*volt*volt+0.2852*volt-0.0238; // Curve fit equation relating voltage to torque kg-m
torque_Nm=9.80665*torque_kgm; // Torque kg-m converted to Torque N-m
ANT_INST_power = uint16_t(torque_Nm*omega); // Instant power calculated
ANT_power += ANT_INST_power; // Incremental power calcualted
//Serial.print("period: ");
//Serial.print(period);
//Serial.print("omega: ");
//Serial.print(omega);
Serial.print("RPM: "); //REM
Serial.print(ANT_icad); //REM
Serial.print("\t VOLT: "); //REM
Serial.print(volt); //REM
Serial.print("\t Torque(kg-m): "); //REM
Serial.print(torque_kgm); //REM
Serial.print("\t Torque(N-m): "); //REM
Serial.print(torque_Nm); //REM
Serial.print("\t Power: "); //REM
Serial.print(ANT_INST_power); //REM
Serial.print("\t ANT Event: "); //REM
Serial.print(ANT_event); //REM
Serial.print("\t ACC Power: "); //REM
Serial.println(ANT_power); //REM
basicpower(); // Main ANT tranmission of Basic Power Data and RPM
mySerial.flush();
recalc = 0;
}
}

void Crank_Pickup()
{
int reading = digitalRead(Mag_pickup);
if (reading != lastButtonState) {
// reset the debouncing timer
lastDebounceTime = millis();
sent = 0;
} 

if ((millis() - lastDebounceTime) > debounceDelay) {
buttonState = reading;
if (buttonState == 1)
{
if (sent == 0)
{
sent = 1;
blink(); 
}
}
}
lastButtonState = reading; 
}

void blink()
{
recalc = 1;
state = !state;
time2 = time1;
time1 = micros();
}

UCHAR checkSum(UCHAR *data, int length)
{
int i;
UCHAR chksum = data[0];
for (i = 1; i < length; i++)
chksum ^= data[i]; // +1 since skip prefix sync code, we already counted it
return chksum;
}

void reset ()
{
uint8_t buf[5];
buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
buf[1] = 0x01; // LENGTH Byte
buf[2] = MESG_SYSTEM_RESET_ID; // 0x4A
buf[3] = 0x00; // Data Byte N (N=LENGTH)
buf[4] = checkSum(buf,4);
ANTsend(buf,5);
}


void SetNetwork() //thisisANT.com and become an ANT+ Adopter
{
uint8_t buf[13];
buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
buf[1] = 0x09; // LENGTH Byte
buf[2] = MESG_NETWORK_KEY_ID; // ID Byte 0x46
buf[3] = 0x00; // Data Byte N (Network Number)
buf[4] = 0xXX; // Data Byte N (Public Network Key)
buf[5] = 0xXX; // Data Byte N (Public Network Key)
buf[6] = 0xXX; // Data Byte N (Public Network Key)
buf[7] = 0xXX; // Data Byte N (Public Network Key)
buf[8] = 0xXX; // Data Byte N (Public Network Key)
buf[9] = 0xXX; // Data Byte N (Public Network Key)
buf[10] = 0xXX; // Data Byte N (Public Network Key)
buf[11] = 0xXX; // Data Byte N (Public Network Key)
buf[12] = checkSum(buf, 12);
ANTsend(buf,13);
}
void assignch()
{
uint8_t buf[7];
buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
buf[1] = 0x03; // LENGTH Byte
buf[2] = MESG_ASSIGN_CHANNEL_ID; // 0x42
buf[3] = 0x00; // Channel Number
buf[4] = 0x10; // Channel Type
buf[5] = 0x00; // Network Number
buf[6] = checkSum(buf,6);
ANTsend(buf,7);
}

void SetChID()
{
uint8_t buf[9];
buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
buf[1] = 0x05; // LENGTH Byte
buf[2] = MESG_CHANNEL_ID_ID; // Assign Channel ID 0x51
buf[3] = 0x00; // channel number
buf[4] = 0x05; // Device number
buf[5] = 0x00; // Device number
buf[6] = 0x0B; //Device type ID
buf[7] = 0x00; //Transmission type -CHANGED
buf[8] = checkSum(buf, 8);
ANTsend(buf,9);
}

void ANTsend(uint8_t buf[], int length){
//Serial.print("ANTsend TX: ");
for(int i = 0 ; i <= length ; i++)
{
//Serial.print(buf[i], HEX);
//Serial.print(" ");
mySerial.write(buf[i]);
}
//Serial.println("");
}
void SetFreq()
{
uint8_t buf[6];
buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
buf[1] = 0x02; // LENGTH Byte
buf[2] = MESG_CHANNEL_RADIO_FREQ_ID; // Set Channel RF Freq 0x45
buf[3] = 0x00; // Channel number
buf[4] = 0x39; // Frequency
buf[5] = checkSum(buf, 5);
ANTsend(buf,6);
}

void SetPeriod()
{
uint8_t buf[7];
buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
buf[1] = 0x03; // LENGTH Byte
buf[2] = MESG_CHANNEL_MESG_PERIOD_ID; // Set channel period 0x43
buf[3] = 0x00; // Channel number
buf[4] = 0xF6; // Messaging Period byte1
buf[5] = 0x1f; // Messaging period byte2
buf[6] = checkSum(buf, 6);
ANTsend(buf,7);
}

void SetPower()
{
uint8_t buf[6];
buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
buf[1] = 0x02; // LENGTH Byte
buf[2] = MESG_RADIO_TX_POWER_ID; // Set Tx Power 0x47
buf[3] = 0x00; // Channel Number
buf[4] = 0x03; // Tx power
buf[5] = checkSum(buf, 5);
ANTsend(buf,6);
}

void SetTimeout()
{
uint8_t buf[6];
buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
buf[1] = 0x02; // LENGTH Byte
buf[2] = MESG_CHANNEL_SEARCH_TIMEOUT_ID; // Set Channel Search Timeout 0x44
buf[3] = 0x00; // Channel number
buf[4] = 0x1E; // Set timeout
buf[5] = checkSum(buf, 5);
ANTsend(buf,6);
}

void OpenChannel()
{
uint8_t buf[5];
buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
buf[1] = 0x01; // LENGTH Byte
buf[2] = MESG_OPEN_CHANNEL_ID; // ID Byte 0x4B
buf[3] = 0x00;
buf[4] = checkSum(buf, 4);
ANTsend(buf,5);
}

void initiate()
{
SetNetwork();
delay(100);
assignch();
delay(100);
SetChID();
delay(100);
SetFreq();
delay(100);
SetPeriod();
delay(100);
SetPower();
delay(100);
SetTimeout();
delay(100);
OpenChannel();
delay(100);
}

void basicpower()
{
uint8_t buf[13];
buf[0] = MESG_TX_SYNC; // SYNC Byte 0xA4
buf[1] = 0x09; // LENGTH Byte
buf[2] = MESG_BROADCAST_DATA_ID; // 0x4E
buf[3] = 0x00; // Channel number
buf[4] = 0x10; // Standard Power-Only Message, Byte 0
buf[5] = ANT_event; // Power Event Count, Byte 1
buf[6] = 0xFF; // Pedal Power, Byte 2, 0xFF, Pedal Power Not Used
buf[7] = ANT_icad; // Instant Cadence, RPM, Byte 3
buf[8] = byte(ANT_power & 0xFF); // Accumulated power LSB, Byte 4
buf[9] = byte((ANT_power >> 8) & 0xFF); // Accumulated power MSB, Byte 5
buf[10] = byte(ANT_INST_power & 0xFF);; // Instant power LSB, Byte 6
buf[11] = byte((ANT_INST_power >> 8) & 0xFF); // Instant power MSB, Byte 7
buf[12] = checkSum(buf, 12);
ANTsend(buf, 13);
}
