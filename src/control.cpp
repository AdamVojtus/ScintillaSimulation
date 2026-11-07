#include <Arduino.h>
#define TXD1 19
#define RXD1 21

#define START_PIN  18
#define STOP_PIN  19

int button_state = 0;
int button_state1 = 0; 

// treba poslat bity typu 'O' je to ocko len velke O

void openCommunicationChannel()
{
  Serial.println("CONNECTING TO INSTRUMENTS SYSTEM...");
  if (Serial1.available()) {
    String message = Serial1.readStringUntil('\n');
    Serial.println("Received: " + message);
  }
  else
  {
    stop();
  }
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
  analogSetAttenuation(ADC_11db);
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);
}

void loop()
{
    Serial.println('O');
}