/*
 * This sketch is used to put the HC-05 bluetooth module into AT commmand mode
 * and allow you to send and recv commands from the module.
 * Pin 9: wired to the enable pin of the module. This must be high for AT mode.
 * If when you start this sketch the module doesn't enter AT mode, power off the HC-05 
 * and on again.
 * Fast Red blink: Data mode
 * Single Red blink every 2 seconds: AT command mode
 * Double Red blink ever 2 seconds: HC-05 connected to another bluetooth module
 * 
 * AT Commands:
 * AT +ADDR?  - Address of device
 * AT +VERSION? - Firmware version
 * AT +NAME?  - Name of device
 * AT +ROLE?  - 0 slave, 1 master, 2 slave-loop
 * AT +UART?  - Param 1: Baude, Param 2: stop bit, Param 3: parity
 * AT +BIND?  - Shows what address it's peer is if any configured
 * AT +STATE? - Current state of device
 *
 * To Set HC-05 into Master mode:
 * AT +ROLE=1   (master)
 * Optionally to only allow connecting to a specific slave:
 * AT +CMODE=0   (fixed addr)
 * AT +BIND=<slave address>  (use commas between sections of address)
 */
#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {

Serial.begin(9600);

pinMode(12,OUTPUT); digitalWrite(12,HIGH);  // Wire this to the Enable pin on the HC-05. Needs pulled high for AT mode

Serial.println("Enter AT commands:");

mySerial.begin(38400);  //  Has to be rate that your HC-05 is configured to.

}

void loop()

{

if (mySerial.available())

Serial.write(mySerial.read());

if (Serial.available())

mySerial.write(Serial.read());

}
