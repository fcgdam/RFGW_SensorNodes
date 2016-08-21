#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <Manchester.h>

#define DEVICE_ID  0x01

#define TX_PIN  1  // Pin where your transmitter is connected

#define FLAP_IRQ 0 // Pin used for the interrupt request of FLAP_IRQ
#define STAT_IRQ 2 // Pin used fof the interrupt request of status

// Message to be transmitted
// First byte is message lenght
// 2nd byte is message type identifier.
// 3rd byte is device id
// 4th byte and beyond is message payload ended by 0xFF to easy debug at the receiver end.

// This message is only sent at boot time. It allows to pin point boots, if any.
uint8_t boot_msg[4]  = {0, 'B', DEVICE_ID, 0xFF};

// The message array. Second byte is S for Status message, or M for data.
uint8_t data_msg[9]  = {0, 'M', DEVICE_ID, 0, 0, 0, 0, 0, 0xFF};

uint8_t ecdata[16];   // Buffer for message with error correction data added
uint8_t eclen;        // Buffer lenght for the above buffer

uint8_t stsseq = 0;   // Message sequence number for status_msg
uint8_t msgseq = 0;   // Message sequence number for data_msg

volatile uint8_t i_type = 0;    // Interrupt type: 0 for flap movement, 1 for status heart beat

void sleep() {

    GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts

    PCMSK |= _BV(PCINT0);                   // Use PB0 as interrupt pin for detecting the Mailbox flap
    PCMSK |= _BV(PCINT2);                   // Use PB2 as interrupt pin for the status message keep alive

    ADCSRA &= ~_BV(ADEN);                   // ADC off
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement

    sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei();                                  // Enable interrupts
    sleep_cpu();                            // sleep

    cli();                                  // Disable interrupts
    PCMSK &= ~_BV(PCINT0);                  // Turn off PB0 as interrupt pin
    PCMSK &= ~_BV(PCINT2);                  // Turn off PB1 as interrupt pin

    sleep_disable();                        // Clear SE bit
    ADCSRA |= _BV(ADEN);                    // ADC on

    sei();                                  // Enable interrupts
    } // sleep

ISR(PCINT0_vect) {
    i_type = 1;
    // if ( GIFR & 0x40 )
    //       i_type = 0;
    // if ( GIFR & 0x20 )
    //       i_type = 1;
}

void send_boot_msg() {
  // Given the data, add Hamming EC data
  eclen = man.EC_encodeMessage( 4, boot_msg, ecdata );
  boot_msg[0] = (byte) eclen;
  eclen = man.EC_encodeMessage( 4, boot_msg, ecdata );

  //Serial.println( eclen );
  man.transmitArray( eclen, ecdata);
}

void send_msg(long vcc , uint8_t seq) {

  data_msg[3] = seq;

  // Store the voltage:
  data_msg[4] = (byte) vcc;
  data_msg[5] = (byte) vcc >> 8;
  data_msg[6] = (byte) vcc >> 16;
  data_msg[7] = (byte) vcc >> 24;

  data_msg[8] = 0xff;

  // Given the data, add Hamming EC data
  eclen = man.EC_encodeMessage( 9, data_msg, ecdata );
  data_msg[0] = (byte) eclen;
  eclen = man.EC_encodeMessage( 9, data_msg, ecdata );

  man.transmitArray( eclen, ecdata);
}

void send_status_msg( long vcc )  {
  data_msg[1] = 'S';
  send_msg( vcc , stsseq++ );
}

void send_flap_msg( long vcc ) {
  data_msg[1] = 'M';
  send_msg( vcc, msgseq++ );
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
/*  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
*/
  ADMUX = _BV(MUX3) | _BV(MUX2);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void setup()
{
//  pinMode(LED_PIN, OUTPUT);
//  digitalWrite(LED_PIN, ledstat);

// Setup interrupt pins
  pinMode(FLAP_IRQ,INPUT);
  pinMode(STAT_IRQ,INPUT);
  digitalWrite(FLAP_IRQ, HIGH);
  digitalWrite(STAT_IRQ, HIGH);

  man.workAround1MhzTinyCore(); //add this in order for transmitter to work with 1Mhz Attiny85/84
  man.setupTransmit(TX_PIN, MAN_600);
  send_boot_msg();
}

void loop()
{
  long VccBat = 0;

  //delay(1000);
  sleep();

  VccBat = readVcc();         // Read battery voltage.

  if ( i_type == 0 )          // If interrupt type is 0, then it was wake up from PCINT0
    send_flap_msg(VccBat);    // It seems that in this case the ISR is not executed.
  else {
    send_status_msg(VccBat);  // We where waked up, and the ISR was executed.
    i_type = 0;               // Reset the i_type.
  }
}
