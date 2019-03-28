#include <Wire.h>
#include <XBee.h>
#include <Adafruit_BMP085.h>
#include <SPI.h>
#include "SdFat.h"
SdFat SD;


//init global data values
float werewolf1alt, werewolf1lat, werewolf1lon;
int RH; int RH_bits; //humidity
long press085;//bmp085 data : temp085 = rtp * 10

String temp085 = "Temp085: ", alt085 = "Altitude: ", bearing_x, bearing_y, bearing_z, bearing_d, humidity;
Adafruit_BMP085 bmp;


bool tx_success, rx_success, contact_made = false;
bool sendNewData = true;

//SDCARD
int readIndex = 0, writeIndex = 0;
File myFile;
float temperature;
int logNumber = 0;
int txmode = 0; // 1 = Transfer old 0 = normal mode

// Calibration values
int compx_bits, compy_bits, compz_bits; //compass
const unsigned char OSS = 0;  // Oversampling Setting
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5;

//XBEE
XBee xbee = XBee();
uint8_t payload[] = { 0, 0};
//SH + SL address of receiving XBee
XBeeAddress64 addr64 = XBeeAddress64(0x0013A200, 0x4098DA02);
XBeeResponse response   = XBeeResponse();
ZBRxResponse        rx  = ZBRxResponse();
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
ModemStatusResponse msr = ModemStatusResponse();
int statusLed = 10; // green
int errorLed  = 11; // red
int dataLed   = 12; // yellow
int rxled = 13;

// end XBEE
void setup() {
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed,  OUTPUT);
  pinMode(dataLed,   OUTPUT);
  pinMode(rxled,       OUTPUT);
  
  Serial.begin(9600);
  Wire.begin();
  bmp085Calibration();
  
  // start xbee in serial 2
  Serial2.begin(9600);
  xbee.setSerial(Serial2);
  while (!Serial) {
    ;
  }
  while (!SD.begin(SS)) {
    Serial.println("init for sd failed");
  }
}
//startup : always: read sensor data, werewolf coordinates and store, while checking for grd station data
// mode 0: read sensor data, send to base station, camera on auto swivel.
// mode 1: camera on free roam, controlled by joystick from base station
// mode 2: camera lock on werewolf, werewolf number decided by keypad input
void loop() {
  // put your main code here, to run repeatedly:
  readBearing(); //update compx, compy, compz with bearing values
  readHumidity(); //update RH_bits(10 bits), print RH for percent
  readTemp085(); //update temp085
  readAltitude(); //update alt085
  receiveData(); //hopefully read werewolf coordinates and grd station input
  receiveData();
  humidity += " %      "; 
  temp085 += " deg C";
  alt085 += " m   ";
  Serial.println(humidity);
  writeSD(humidity);
  writeSD(temp085);
  writeSD(alt085);
  writeSD(bearing_d);

  if (txmode == 0) {
    Serial.println("  ");
    Serial.println("Sending New Data from System 2");
    transmitData(humidity);
    Serial.println(humidity);
    transmitData(temp085);
    Serial.println(temp085);
    transmitData(alt085);
    Serial.println(alt085);
    transmitData(bearing_d);
    Serial.println(bearing_d);
  } else {
    Serial.println("  ");
    Serial.println ("Sending SD Data");
    readSD();
    readSD();
    readSD();
    readSD();
  }
}




void transmitData(String data) {
  tx_success = false;
  int limit = 5;
  while (tx_success == false & limit ) {
    sendData(data);
    limit -= 1;
    delay(500);
  }
}
void sendData(String data) {
  uint8_t payload[30] = {0}; // payload has been changed from 20 to 30
  for (int j = 0; j < data.length(); j += 1) {
    payload[j] = data[j];
  }
  XBeeAddress64 addr64 = XBeeAddress64(0x0013A200, 0x4098DA02); // address of the receiver XBee
  ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
  xbee.send(zbTx);
  // flash TX indicator
  flashLed(dataLed, 1, 10);
  // after sending a tx request, we expect a status response
  // wait up to half a second for the status response
  if (xbee.readPacket(500))
  {
    // should be a znet tx status
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE)
    {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      // get the delivery status, the fifth byte
      if (txStatus.getDeliveryStatus() == SUCCESS)
      {
        // success.  time to celebrate
        flashLed(statusLed, 1, 10);
        tx_success = true;
      }
      else
      {
        // the remote XBee did not receive our packet. is it powered on?
        flashLed(errorLed, 1, 50);
      }
    }
  }
  else
  {
    // local XBee did not provide a timely TX Status Response -- should not happen
    flashLed(errorLed, 5, 50);
  }
}
//function to interpret data received from Xbee
void interpretData(String xbeeInput){
  String identity;
  identity = char(xbeeInput[0]) + char(xbeeInput[1]);
  String weredatatype;
  float weredata;
  int colonidx = xbeeInput.indexOf(':');
  weredatatype = xbeeInput.substring(2,colonidx);
  weredata = xbeeInput.substring(colonidx + 1, xbeeInput.length() - 1).toFloat();
  if (identity == "W1"){ //writing values for werewolf 1
      if (weredatatype == "Altitude"){
        werewolf1alt = weredata;
      } else if (weredatatype == "Lat"){
        werewolf1lat = weredata;
      } else if (weredatatype == "Lon"){
        werewolf1lon = weredata;
      } else {
        Serial.print("cannot differentiate weredatatype: "); 
        Serial.println(weredatatype);        
      }
  } else if (identity == "GS") { //writing values for ground staiton
      if (weredatatype == "Key Input"){
        int keypress = xbeeInput.substring(colonidx + 1, xbeeInput.length() - 1).toInt();
        interpretKeypress(keypress);
      } else {
        Serial.print("unable to interpret weredatatype: ");
        Serial.println(weredatatype);
      }
  } else {
    Serial.print("unable to interpret identity ");
    Serial.println(identity);
  }
  return 0;
}
void interpretKeypress(int keypress){ //fn to interpret keypress. keypress input in form of ascii 
  switch (keypress){
    case 49: //ascii 1
      Serial.println("camera lock on werewolf 1");
      break;
    case 50: //ascii 2
      Serial.println("camera lock on werewolf 2");
      break;
    case 42:
      Serial.println("disengage lock on. free roam mode");
      break;
    Serial.print("keyinput received: "); //for testing, can comment out later
    Serial.println(keypress);
  }
}
void receiveData() {
  rx_success = false;
  int limit = 50;
  while (rx_success == false & limit > 0 ) {
    getData();
    limit -= 1;
    delay(5);
  }
}
void getData() {
  digitalWrite(rxled, HIGH); // turn the led pin on
  xbee.readPacket(500);
  flashLed(dataLed, 1, 10);
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) { // got something
      rx_success = true;
      if (contact_made == false) {
        contact_made = true;
      } else {
        String xbeeInput = ""; 
        for (int i = 0; i < rx.getDataLength(); i += 1) {
//          if ( char(rx.getData(i)) == '&')
//          {
//            txmode = 1 - txmode;
//            Serial.println(" ");
//            Serial.println("Mode Change");
//            i += 1;
//          }
//          Serial.print(char(rx.getData(i))); // print Key press
            xbeeInput += char(rx.getData(i));
        }
        interpretData(xbeeInput);
        //Serial.println();
      }
      xbee.getResponse().getZBRxResponse(rx);// got a zb rx packet,now fill our zb rx class
      if (rx.getOption() == 65) {
        flashLed(statusLed, 1, 10); // the sender got an ACK
        rx_success = true;
      } else {
        flashLed(statusLed, 2, 10); // we got it (obviously) but sender didn't get an ACK
      }
      digitalWrite(rxled, LOW); // turn the led pin off
    } else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
      xbee.getResponse().getModemStatusResponse(msr);
      if (msr.getStatus() == ASSOCIATED) {
        flashLed(statusLed, 5, 100);// yay this is great.  flash led
        rx_success = true;
      } else if (msr.getStatus() == DISASSOCIATED) {
        flashLed(errorLed, 10, 50); // this is awful.. flash led to show our discontent
      } else {
        // another statusLed
        flashLed(statusLed, 10, 100);
      }
    } else {
      flashLed(errorLed, 3, 100);// not something we were expecting
    }
  }
}
void flashLed(int pin, int times, int wait)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);
    if (i + 1 < times) delay(wait);
  }
}

//----------------READ ALTITUDE------------------//

void readAltitude() {
  alt085 = "Altitude: ";
  float alt = (float)44330 * (1 - pow(((float) press085 / 101325), 0.190295));
  alt085 += alt;
}

//------------- READ TEMP---------//

void readTemp085() {
  temperature = bmp085GetTemperature(bmp085ReadUT()) / 10;
  temp085 = "Temp085: ";
  temp085  += temperature;
  press085 = bmp085GetPressure(bmp085ReadUP());
}

//-----------------------BMP085 FUNCTIONS----------------------

void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;

  x1 = (((long)ut - (long)ac6) * (long)ac5) >> 15;
  x2 = ((long)mc << 11) / (x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8) >> 4);
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6) >> 12) >> 11;
  x2 = (ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((long)ac1) * 4 + x3) << OSS) + 2) >> 2;

  // Calculate B4
  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;

  b7 = ((unsigned long)(up - b3) * (50000 >> OSS));
  if (b7 < 0x80000000)
    p = (b7 << 1) / b4;
  else
    p = (b7 / b4) << 1;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p += (x1 + x2 + 3791) >> 4;

  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(0x77);
  Wire.write((byte)address);
  Wire.endTransmission();

  Wire.requestFrom(0x77, 1);
  while (!Wire.available());

  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(0x77);
  Wire.write((byte)address);
  Wire.endTransmission();

  Wire.requestFrom(0x77, 2);
  while (Wire.available() < 2);
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb << 8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(0x77);
  Wire.write((byte)0xF4);
  Wire.write((byte)0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(0x77);
  Wire.write((byte)0xF4);
  Wire.write((byte)(0x34 + (OSS << 6)));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3 << OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(0x77);
  Wire.write((byte)0xF6);
  Wire.endTransmission();
  Wire.requestFrom(0x77, 3);

  // Wait for data to become available
  while (Wire.available() < 3);
  msb  = Wire.read();
  lsb  = Wire.read();
  xlsb = Wire.read();
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8 - OSS);

  return up;
}

//-------------------READ HUMDIDIDTY-----------------//

void readHumidity() {
  humidity = "Humidity: ";
  int value = analogRead(A0);
  RH_bits = (value);
  //  Serial.print("RH_bits: "); Serial.println(RH_bits, BIN);
  float outVolt = value * 5.0 / 1024.0;
  float sensorRH =  161.0 * outVolt / 5.0 - 25.8;
  RH = sensorRH / (1.0546 - 0.0026 * temperature);
  humidity += RH;
  humidity += " %";
}

//-------------------READ BEARING FUNCTION----------//

void readBearing() { //0x1E, compAddress
  Wire.beginTransmission(0x1E);
  Wire.write((byte)0x02);
  Wire.write((byte)0x01);
  Wire.endTransmission();
  Wire.requestFrom(0x1E, 6);
  if (6 <= Wire.available()) {
    compx_bits = Wire.read() << 8;
    compx_bits |= Wire.read();
    compz_bits = Wire.read() << 8;
    compz_bits |= Wire.read();
    compy_bits = Wire.read() << 8;
    compy_bits |= Wire.read();
  }
//  bearing_x = "x: ";
//  bearing_x += compx_bits;
//  bearing_x += "              ";
//  bearing_y = "y: ";
//  bearing_y += compy_bits;
//  bearing_y += "               ";
//  bearing_z = "z: ";
//  bearing_z += compz_bits;
  //bearing_z += "                ";
  int bearing_deg = atan2(-compy_bits, compx_bits) / M_PI * 180;
  if (bearing_deg < 0)
  {
    bearing_deg = bearing_deg + 360;
  }
  bearing_d = "Bearing: ";
  bearing_d += bearing_deg;
  bearing_d +=  " degrees ";
}

//----------------SD FUNCTION----------//
//writeSD
void writeSD(String sensorWrite) {
  String tempSensorWrite;
  tempSensorWrite += sensorWrite + '!';
  myFile = SD.open("Sensor17.txt", FILE_WRITE);
  if (myFile) {
    myFile.println(logNumber / 4);
    logNumber += 1;
    myFile.println(tempSensorWrite);
    writeIndex = myFile.position();
    myFile.close();
  } else {
    Serial.println("error writing to SD card");
  }
}

//readSD
void readSD() {
  String readSet;
  myFile = SD.open("Sensor17.txt", FILE_READ);
  if (myFile) {
    myFile.seek(readIndex);
    char currChar = myFile.read();
    while (currChar != '!') {
      readSet += currChar;
      currChar = myFile.read();
    }
    //    Serial.println(readSet);
    readIndex = myFile.position();
    myFile.close();
    transmitData(readSet);
    Serial.println(readSet);
  } else {
    Serial.println("error opening SensorData.txt");
  }
}
