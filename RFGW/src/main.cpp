#include "Manchester.h"
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Ds1307.h>

/*
 * 433Mhz based gateway for my RF nodes.
 *
 * Sensores send data through RF transmission over the 433Mhz ISM band.
 * The gateway uses the RXB8  receiver to decode and send data through the serial port to a node red flow.
*/
#define RX_PIN  4
#define LED_PIN 13
#define DS1307_I2C_ADDRESS 0x68

uint8_t moo = 1 , res , ddata;

long msg_count = 0 , msg_errors = 0;   // Let's count the number of messages received and messages with errors.

#define BUFFER_SIZE 22
uint8_t buffer[BUFFER_SIZE];
uint8_t data[BUFFER_SIZE];

// Input command and data buffer:
String inputData = "";
boolean inputDataReady = false;

LiquidCrystal_I2C lcd( 0x27, 16, 2); // I2C address, collums and rows
// DS1307 RTC instance
Ds1307 rtc(DS1307_I2C_ADDRESS);
static uint8_t last_second = 0;

const static char* WeekDays[] =
{
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
    "Sunday"
};

void printArray( uint8_t *data , uint8_t dlen )
{
    for(uint8_t i = 0; i < dlen ; i++) {
      if (data[i]<0x10) {Serial.print("0");}
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
}

void processCommand() {
    Serial.println("Processing Command: ");
    inputData.toUpperCase();
    inputData.replace("\n","");
    inputData.replace("\r","");

    Serial.println( inputData);

    if ( inputData.equals("H0") ) {
      digitalWrite(LED_PIN, 0);
      Serial.println("Processing H0");
    }

    if ( inputData.equals("H1") ) {
      Serial.println("Processing H1");
      digitalWrite(LED_PIN, 1);
    }
}

void i2cScanner() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void getTime() {
  Ds1307::DateTime now;
  rtc.getDateTime(&now);

  if (last_second != now.second)
  {
      last_second = now.second;

      lcd.setCursor(0 , 0) ;

      Serial.print("20");
      lcd.print("20");
      Serial.print(now.year);    // 00-99
      lcd.print(now.year);

      Serial.print('-');
      lcd.print("-");

      if (now.month < 10) {
        Serial.print('0');
        lcd.print("0");
      }
      Serial.print(now.month);   // 01-12
      lcd.print(now.month);
      Serial.print('-');
      lcd.print("-");

      if (now.day < 10) {
        Serial.print('0');
        lcd.print("0");
      }
      Serial.print(now.day);     // 01-31
      lcd.print(now.day);
      Serial.print(' ');
      lcd.print(" ");
      Serial.print(WeekDays[now.dow - 1]); // 1-7

      Serial.print(' ');
      lcd.setCursor(0 , 1);
      if (now.hour < 10) {
         Serial.print('0');
         lcd.print("0");
       }
      Serial.print(now.hour);    // 00-23
      lcd.print(now.hour);
      Serial.print(':');
      lcd.print(":");
      if (now.minute < 10){
        Serial.print('0');
        lcd.print("0");
      }
      Serial.print(now.minute);  // 00-59
      lcd.print(now.minute);
      Serial.print(':');
      lcd.print(":");
      if (now.second < 10) {
       Serial.print('0');
       lcd.print("0");
      }
      Serial.print(now.second);  // 00-59
      lcd.print(now.second);
      Serial.println();
  }
}

void setup()
{
  StaticJsonBuffer<200> jsonBuffer;

  // Setup serial port
  Serial.begin(115200);
  lcd.init();                       // Initiates the HD44780 display with the I2C PF8475 adapter
  lcd.backlight();                  // Turns on the display
  lcd.begin(16,2);
  lcd.setCursor(0,0); //Start at character 4 on line 0
  lcd.print("Starting...");
  // Setup RF RX receiver.
  // I've choosen a low bit rate to improve range.
  man.setupReceive(RX_PIN, MAN_600);
  man.beginReceiveArray(BUFFER_SIZE, buffer); // This initiates the listening process

  // Prepare the input buffer for the serial port
  // We only expect at maximum around 32 bytes.
  inputData.reserve(32);

  // Let's use first the Pin 13 LED for heartbeat from node-red
  pinMode(LED_PIN, OUTPUT);

  JsonObject& root = jsonBuffer.createObject();
  root["status"] = "OK";
  root["type"] = "RFBOOT";
  root.printTo(Serial);
  Serial.println("");

  lcd.setCursor(0,0); //Start at character 4 on line 0
  lcd.print("Ready!     ");

  i2cScanner();
  // initialize the RTC
  rtc.init();

  // test if clock is halted and set a date-time (see example 2) to start it
  if (rtc.isHalted())
  {
      Serial.println("RTC is halted. Setting time...");

      Ds1307::DateTime dt = {
          .year = 17,
          .month = Ds1307::MONTH_OCT,
          .day = 30,
          .hour = 17,
          .minute = 41,
          .second = 53,
          .dow = Ds1307::DOW_MON
      };

      rtc.setDateTime(&dt);
  }

}

uint8_t getECSize(uint8_t size) {
  uint8_t s = 0;

  s = size + size / 2;   // One byte for every two bytes;
  if ( size % 2 ) {
    s += 2;    // Add two more bytes for the pad
  }

  return s;
}


void loop()
{

  // Process RF received data
  if (man.receiveComplete())
  {
    StaticJsonBuffer<200> jsonBuffer;

    uint8_t receivedSize = 0;
    //uint8_t sizeMsg = 0;
    long vbat;

    JsonObject& root = jsonBuffer.createObject();

    // Increment the message count
    msg_count++;

    //do something with the data in 'buffer' here before you start receiving to the same buffer again
    receivedSize = buffer[0];       // This is the data size without EC bytes
    // receivedSize = getECSize( sizeMsg);
    // receivedSize = sizeMsg;
    // Serial.print("Received data from RF: ");
    // Serial.print( receivedSize);
    // Serial.println(" bytes received." );
    // printArray( buffer , receivedSize );

    // Now let's apply EC
    res = man.EC_decodeMessage( receivedSize , buffer, &ddata, data );
    // Serial.println("EC message: ");
    // printArray( data, ddata );

    if ( data[1] != 'B') {
      vbat = data[4] + data[5]*256;
      vbat = 1.1 * 1024 * 1000 / vbat;

    } else vbat=0;


    if ( res != NO_ERROR ) {  // Message was received with uncorrectable errors
      msg_errors ++;

      root["status"] = "NOK";
      root["type"] = String((char)data[1]);
      root["vbat"] = vbat;


    } else {                  // Message was received fine. Let's send it to the backend
      root["status"] = "OK";
      root["type"] = String((char)data[1]);
      root["vbat"] = vbat;

    }
    root["msgtotal"] = msg_count;
    root["msgerror"] = msg_errors;
    root["deviceid"] = data[2];
    root["msgseqn"] = data[3];    // For the boot message it will read the 0xFF value

    // Let's send info to node-red
    root.printTo(Serial);
    Serial.println("");

    // And print the resulting EC message.
    // Serial.println("EC message: ");
    // printArray( data, ddata );

    man.beginReceiveArray(BUFFER_SIZE, buffer);
    moo++;
    moo = moo % 2;
    digitalWrite(LED_PIN, moo);

    // Shows data on the display
    lcd.setCursor(0 , 0) ;
    lcd.print("M: ");
    lcd.print( msg_count );
    lcd.print(" E: ");
    lcd.print( msg_errors);
    lcd.setCursor(0,1);
    lcd.print("Vbat: ");
    lcd.print(vbat);
  }

  // Process Serial received data
  if (inputDataReady) {
    Serial.println(inputData);

    // Process the command:
    processCommand();

    // clear the string:
    inputData = "";
    inputDataReady = false;
  }

  getTime();

}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputData += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      inputDataReady = true;
    }
  }
}
