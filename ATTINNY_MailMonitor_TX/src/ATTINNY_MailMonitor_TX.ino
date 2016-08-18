#include <Arduino.h>


#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <Manchester.h>

#define DEVICE_ID  0x01

#define TX_PIN  1  //pin where your transmitter is connected
//#define LED_PIN 1  //pin for blinking LED

uint8_t ledstat  = 1; //last led status

// Message to be transmitted

// First byte is message lenght without the byte itself.
// 2nd byte is message type identifier.
// 3rd byte is device id
// 4th byte and beyond is message payload ended by 0xFF to easy debug at the receiver end.

uint8_t status_msg[9]  = {8, 'S', DEVICE_ID, 0, 0, 0, 0, 0, 0xFF};

uint8_t data_msg[11]   = {10, 'M', DEVICE_ID, 0, 0, 0, 0, 0, 0, 0, 0xFF};

uint8_t ecdata[16];
uint8_t eclen;

uint8_t msgseq = 0;   // Message sequence number

void sleep() {

    GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
    PCMSK |= _BV(PCINT0);                   // Use PB0 as interrupt pin
    ADCSRA &= ~_BV(ADEN);                   // ADC off
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement

    sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei();                                  // Enable interrupts
    sleep_cpu();                            // sleep

    cli();                                  // Disable interrupts
    PCMSK &= ~_BV(PCINT3);                  // Turn off PB3 as interrupt pin
    sleep_disable();                        // Clear SE bit
    ADCSRA |= _BV(ADEN);                    // ADC on

    sei();                                  // Enable interrupts
    } // sleep

ISR(PCINT0_vect) {
    // This is called when the interrupt occurs, but I don't need to do anything in it
    }

void setup()
{
//  pinMode(LED_PIN, OUTPUT);
//  digitalWrite(LED_PIN, ledstat);
  //man.workAround1MhzTinyCore(); //add this in order for transmitter to work with 1Mhz Attiny85/84
  man.setupTransmit(TX_PIN, MAN_600);
}

void send_status_msg(long vcc ) {

  // Store the voltage:
  status_msg[3] = (byte) 4; //vcc
  status_msg[4] = (byte) 15; //vcc >> 8;
  status_msg[5] = (byte) 83; //vcc >> 16;
  status_msg[6] = (byte) 7; //vcc >> 24;
  status_msg[7] = msgseq;
  status_msg[8] = 0xff;

  msgseq = msgseq + 4;

  // Given the data, add Hamming EC data
  eclen = man.EC_encodeMessage( 9, status_msg, ecdata );
  status_msg[0] = (byte) eclen;
  eclen = man.EC_encodeMessage( 9, status_msg, ecdata );

  //Serial.println( eclen );
  man.transmitArray( eclen, ecdata);
  //man.transmitArray( 8, status_msg);

}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void loop()
{
  long VccBat = 0;

  VccBat = readVcc();

  send_status_msg(VccBat);

  //ledstat = ++ledstat % 2;
  //digitalWrite(LED_PIN, ledstat);


  //delay(1000);
  sleep();

}
