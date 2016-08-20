#include "Manchester.h"
#include <ArduinoJson.h>

/*
 * 433Mhz based gateway for my RF nodes.
 *
 * Sensores send data through RF transmission over the 433Mhz ISM band.
 * The gateway uses the RXB8  receiver to decode and send data through the serial port to a node red flow.
*/
#define RX_PIN  4
#define LED_PIN 13

uint8_t moo = 1 , res , ddata;

long msg_count = 0 , msg_errors = 0;   // Let's count the number of messages received and messages with errors.

#define BUFFER_SIZE 22
uint8_t buffer[BUFFER_SIZE];
uint8_t data[BUFFER_SIZE];

// Input command and data buffer:
String inputData = "";
boolean inputDataReady = false;

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

void setup()
{
  StaticJsonBuffer<200> jsonBuffer;

  // Setup serial port
  Serial.begin(9600);

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
    uint8_t sizeMsg = 0;

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

    if ( res != NO_ERROR ) {  // Message was received with uncorrectable errors
      msg_errors ++;

      root["status"] = "NOK";
      root["type"] = String((char)data[1]);


    } else {                  // Message was received fine. Let's send it to the backend
      root["status"] = "OK";
      root["type"] = String((char)data[1]);

    }
    root["msgtotal"] = msg_count;
    root["msgerror"] = msg_errors;
    root["deviceid"] = data[2];

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
