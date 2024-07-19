/*-----TRANSMIT-----*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

/*-----( Declare Constants and Pin Numbers )-----*/
#define CE_PIN 9
#define CSN_PIN 10

#define POT_X A0
#define POT_Y A1
#define POT_A A2
#define POT_B A3


// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL;  // Define the transmit pipe


/*-----( Declare objects )-----*/
RF24 radio(CE_PIN, CSN_PIN);  // Create a Radio
/*-----( Declare Variables )-----*/
int pot[4];


void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(pipe);
}


void loop() {
  pot[0] = analogRead(POT_X);
  pot[1] = analogRead(POT_Y);
  pot[2] = analogRead(POT_A);
  pot[3] = analogRead(POT_B);

  radio.write(pot, sizeof(pot));
}