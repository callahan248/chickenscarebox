/* Once powered on, calibrate the motion sensor and wait for a button to be pressed to arm the system
   Once armed, begin reading motion
   if motion is detected,  look for distance
   read sound duration
   convert duration to distance
   convert distance into inches or CM
   if distance is within a certain range, activate the system
   set armed to if statement that states "if ARMED, then disable events and enable sensors
   set !armed to enable events and disable sensors
   Green LED is solid when Armed
   Red LED on and green LED off for events
   When button is pressed, give 5 sec delay and then re-arm system
*/

#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>
#include <SSD1306AsciiSoftSpi.h>
#include <SSD1306AsciiSpi.h>
#include <SSD1306AsciiWire.h>
#include <SSD1306init.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"


#define I2C_ADDRESS 0x3C
#define RST_PIN -1


#define DUTY 100
#define FIVESEC 50      //50 X DUTY = 5000 (five Seconds)
#define THREE 3         //For the redLed blink @ 3X Duty or 300ms Rate

#define triggerPin 7
#define echoPin  6

long duration;
int distance;

int greenLed = 11;
int redLed = 13;
int armButton = 9;
int soundTrigger = 10;
int pneumaticTrigger = 8;
int motionSensor = 12;
int calibrationTime = 3; // set to 21 when complete

bool motionDetected = false, update = false, systemArmed = false, distanceRead = false;

uint8_t eventCount = 0, soundTriggerState = LOW, pneumaticTriggerState = LOW, greenLedState = HIGH, redLedState = LOW, blinkCount = 0;

uint32_t prevTime = 0;

SSD1306AsciiWire oled;
//----------------------------------------------------------------------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000L);

#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

  oled.setFont(Adafruit5x7);

  //oled.set1X();
  oled.clear();
  oled.println();
  oled.println();
  oled.println();
  oled.set2X();
  oled.println("Turning On");
  delay (2000);
  oled.clear();

  Serial.begin (9600);
  pinMode (greenLed, OUTPUT);
  pinMode (redLed, OUTPUT);
  pinMode (armButton, INPUT_PULLUP);
  pinMode (soundTrigger, OUTPUT);
  pinMode (pneumaticTrigger, OUTPUT);
  pinMode (motionSensor, INPUT);
  pinMode (triggerPin, OUTPUT);
  pinMode (echoPin, INPUT);
  digitalWrite(soundTrigger, soundTriggerState);
  digitalWrite(pneumaticTrigger, pneumaticTriggerState);
  digitalWrite(redLed, redLedState);

  //  give the sensor some time to calibrate
  oled.println();
  oled.println("Calibrating"); //give the sensor some time to calibrate
  for (int i = 0; i < calibrationTime; i++) {
    oled.set1X();
    oled.print(".");
    delay(1000);

  }
  oled.clear();
  oled.set2X();
  oled.println("   DONE! ");
  oled.println();
  oled.println();
  oled.set1X();
  oled.println("    Sensor Active ");
  oled.println();
  delay(5000);
  oled.clear();
  digitalWrite(greenLed, greenLedState);
}

void loop() {
  // put your main code here, to run repeatedly

  if (systemArmed) {
    CheckMotion ();                                           //Sub-routine Below
    if (motionDetected) {
      distanceRead = true;
    }
  }
  if (millis() - prevTime >= DUTY) {
    prevTime = millis();
    if (distanceRead) {
      CheckDistance ();
      if (distance <= 2000) {                                 //Set Distance Parameter
        systemArmed = false;
        eventCount = 0;
        blinkCount = 0;
        pneumaticTriggerState = HIGH;
        soundTriggerState = HIGH;
        digitalWrite (greenLed, LOW);
        update = true;
      }
      distanceRead = false;
    }


    if (!systemArmed) {                                       //system not armed == System Activated(Events enabled, Sensors disabled)

      eventCount++;
      blinkCount++;
      digitalWrite (greenLed, LOW);
      if (blinkCount >= THREE) {                              //Blink LED every 300mS
        digitalWrite(redLed, (redLedState = !redLedState)); //
        blinkCount = 0;
        if (digitalRead(armButton) == LOW) {
          systemArmed = true;

        }
      }
      if (systemArmed == true) {
        digitalWrite (greenLed, HIGH);
        digitalWrite (redLed, LOW);
      }
      else if (eventCount == FIVESEC) {
        pneumaticTriggerState = LOW;
        soundTriggerState = LOW;
        update = true;
      }
    }

    if (update) {
      digitalWrite (pneumaticTrigger, pneumaticTriggerState);
      digitalWrite (soundTrigger, soundTriggerState);
      update = true;

    }

  }
}


void CheckMotion () {
  motionDetected = digitalRead (motionSensor);

}
void CheckDistance () {
  digitalWrite (triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite (triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite (triggerPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = ((float)((0.01723 * duration) * 10) + 0.5);
  //distance = duration * 0.0133 / 2; // inches
  Serial.println(distance);
}



//Format for Text
//oled.set1X();

//oled.set2X();
//oled.println();
//oled.println();
//oled.println("   WORDS ");
//oled.println();
//oled.println();
//oled.clear();
