#include <Wire.h>
#include <XBee.h>
#include <LCDi2cR.h>
#define button 2

LCDi2cR lcd = LCDi2cR(4, 20, 0x63, 0);

//int temp102_bits, keypad;float convertedTemp102;
XBee xbee               = XBee();
// create reusable response objects for responses we expect to handle
XBeeResponse response   = XBeeResponse();
ZBRxResponse        rx  = ZBRxResponse();
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
ModemStatusResponse msr = ModemStatusResponse();
int statusLed = 10; // green
int errorLed  = 11; // red
int dataLed   = 12; // yellow
int rxled       = 13;
int txmode = 0; // 0 = normal mode 1 = call data
String temp102, keypad;
bool rx_success, tx_success, contact_made = false;

//button interrupt
int upper_threshold = 1050;
int lower_threshold = 50;
unsigned long long first_press_time = 0;
unsigned long long previous_press_time = 0;
int no_of_presses = 0;

void setup() {
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed,  OUTPUT);
  pinMode(dataLed,   OUTPUT);
  pinMode(rxled,       OUTPUT);
  pinMode ( button, INPUT);

  //button interrupt
  cli();
  attachInterrupt(digitalPinToInterrupt(2), switchISR, RISING);
  sei();
  
  Serial2.begin(9600); // start xbee in serial 2
  xbee.setSerial(Serial2); // xbee.begin(9600);
  Serial.begin(9600);
  
  flashLed(statusLed, 1, 10);
  lcd.init();
  Wire.begin();
  Serial.begin(9600);
}
//possible future changes: use button interrupt for any changes(swap btwn werewolf?)
//start up : send sensor readings to base station
//upon button press: use joystick to control camera freely
// upon keypad number press: tell satellite to lock on werewolf of choice
// upon '*' press, disengage lock on for satellite, back to start up state
void loop() {
  int keyInput;
  keyInput = getKeypad();
  getTemp102(); //store

}

//--------------XBEE TRANSMIT to 0x4098DA08 FUNCTION-----------------
void transmitData(String data) {
  int limit = 50;
  tx_success = false;
  while (tx_success == false & limit > 0) {
    sendData(data);
    limit -= 1;
    delay(5);
  }
}

void sendData(String data) {
  data = "GS" + data;
  uint8_t payload[20] = {0};
  for (int j = 0; j < data.length(); j += 1) {
    payload[j] = data[j];
  }
  // address of the receiver XBee
  XBeeAddress64 addr64 = XBeeAddress64(0x0013A200, 0x4098DA08); 
  ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
  xbee.send(zbTx);
  // flash TX indicator
  flashLed(dataLed, 1, 10);
  // after sending a tx request, we expect a status response
  // wait up to half a second for the status response
  if (xbee.readPacket(1000))
  {
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE)
    {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      // get the delivery status, the fifth byte
      if (txStatus.getDeliveryStatus() == SUCCESS)
      {
        tx_success = true;
        flashLed(statusLed, 1, 10);
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
    Serial.println("tx timeout");
  }
}

//-------------XBEE RECEIVE FUNCTION--------------------
void receiveData() {
  rx_success = false;
  int limit = 5;
  while (rx_success == false && limit > 0) {
    getData();
    limit -= 1;
    delay(500);
  }
}
void getData() {
  int i , dataLength;
  digitalWrite(rxled, HIGH); // turn the led pin on
  xbee.readPacket();
  flashLed(dataLed, 1, 10);
  if (xbee.getResponse().isAvailable())
  {
    // got something
    // get data, print data
    if (contact_made == false) {
      contact_made = true;
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      dataLength = rx.getDataLength();
      for ( i = 0; i < dataLength ; i += 1) { //print data on LCD
        char curr_char = char(rx.getData(i));
        Serial.print(curr_char);
        lcd.print(curr_char);
      }
      if ( txmode == 1 )
      {
        lcd.setCursor(1, 0);
        lcd.print("SD Data No:");
      }
      else
      {
        lcd.setCursor(1, 0);
        lcd.print("From System 2 Data");
      }
    }
    Serial.println("");
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE)
    {
      // got a zb rx packet
      // now fill our zb rx class
      xbee.getResponse().getZBRxResponse(rx);
      if (rx.getOption() == 65)
      {
        // the sender got an ACK
        flashLed(statusLed, 1, 10);
        rx_success = true;
      }
      else
      {
        // we got it (obviously) but sender didn't get an ACK
        flashLed(statusLed, 2, 10);
      }
    }
    else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE)
    {
      xbee.getResponse().getModemStatusResponse(msr);
      // the local XBee sends this response on certain events,
      // like association/dissociation
      if (msr.getStatus() == ASSOCIATED)
      {
        // yay this is great.  flash led
        flashLed(statusLed, 5, 100);
        //rx_success = true;
        Serial.print("associated");
      }
      else if (msr.getStatus() == DISASSOCIATED)
      {
        // this is awful.. flash led to show our discontent
        flashLed(errorLed, 10, 50);
        Serial.print("disassiciated");
      }
      else
      {
        // another status
        flashLed(statusLed, 10, 100);
        Serial.print("sth else");
      }
    }
    else
    {
      // not something we were expecting
      flashLed(errorLed, 3, 100);
      Serial.println("not expected");
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

//-------NON XBEE FUNCTIONS----------------------
int getKeypad() {
  keypad = "Key Input:";
  int keyInput;
  keyInput = lcd.keypad();
  return keyInput;
//  keyInput = lcd.keypad() - 48;
//  if (keyInput == -6) {
//    keypad += '*';
//  } else if (keyInput == -13) {
//    keypad += '#';
//  } else if (keyInput == -16) {
//    keypad += "N.A";
//  } else {
//    keypad += (keyInput);
//  }
//  return keyInput;
}
void getTemp102() {
  temp102 = "Temp102:";
  // bytes used to store value from the TMP102 temperature registers
  byte firstByte, secondByte;
  // an int is capable of storing two bytes
  int value;
  float convertedTemp;

  Wire.beginTransmission(0x48);
  Wire.write((byte)0x00);  // point to temperature register
  Wire.endTransmission();
  Wire.requestFrom(0x48, 2);
  if ( Wire.available() >= 2 )
  {
    // read the TMP102 temperature register
    firstByte  = (Wire.read());
    secondByte = (Wire.read());
  }
  value = ((firstByte) << 4);
  value |= (secondByte >> 4);
  convertedTemp = value * 0.0625; // need a proper number

  temp102 += convertedTemp;
}

void switchISR() {
  //Serial.print("button pressed");
  if (millis() - first_press_time > upper_threshold) {
    no_of_presses = 1;
    first_press_time = millis();
    previous_press_time = first_press_time;
    //Serial.print("press one");
    return;
  } else if (millis() - previous_press_time < lower_threshold) {
    //Serial.print("bouncing");
    return; //debounce
  } else { //millis() - previous_press_time < upper_threshold
    //Serial.print(no_of_presses);
    no_of_presses += 1;
    previous_press_time = millis();
    if (no_of_presses == 3) {
      txmode = 1 - txmode; //change txmode
      no_of_presses = 0;
      first_press_time = 0;
      previous_press_time = 0;
      Serial.println("mode changed");
    }
  }
}
