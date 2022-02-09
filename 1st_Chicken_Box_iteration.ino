const byte  ledPin = 13, pinBuzzer = 11, rPin = 10, bPin = 8, pirPin = 12; //LED=detect wehther or not it is armed or went off, Buzzer = Soundboard, R pin = Relay 1 pin, B pin =
uint16_t    distance;
bool        isHuman = false, update = false, armed = true,
            ultaRead = false;
uint8_t     eventCount = 0, rPinState = LOW, bPinState = LOW,
            ledState = HIGH, blinkCount = 0;
uint32_t prevTime = 0;

#define DUTY 100
#define MINUTE 200      //200 x DUTY = 20,000 or 20 seconds
#define SIXSEC 50       //50 x DUTY = 5000 (five seconds)
#define THREE 3         //For LED blink @ 3 x DUTY or 300mS rate

int buttonPin = 9; 

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(7, OUTPUT);           //TRIG
  pinMode (6, INPUT);           //ECHO
  pinMode(pinBuzzer, OUTPUT);  //
  pinMode(rPin, OUTPUT);        //Define start state
  pinMode(bPin, OUTPUT);
  pinMode(pirPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  digitalWrite(rPin, rPinState);
  digitalWrite(bPin, bPinState);
  digitalWrite(ledPin, ledState);
}

uint32_t duration(uint8_t triggerPin, uint8_t echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(triggerPin, LOW);
  return pulseIn(echoPin, HIGH);
}

void loop() {


  if (armed) {
    isHuman = digitalRead(pirPin);
    if (isHuman) {
      ultaRead = true;
    }
  }

  if (millis() - prevTime >= DUTY) {
    prevTime = millis();
    if (ultaRead) {
      //Distance is in "mm" and it's rounded up/down to nearest "mm"
      distance = ((float)((0.01723 * duration(7, 6)) * 10) + 0.5);
      //Take a single distance read to see if object is 2000mm() or closer
      if (distance <= 2000) {
        armed = false;        //Disarm sensors and enable events
        eventCount = 0;       //Zero event timer counter
        blinkCount = 0;       //Zero blink timer counter
        //                        tone(pinBuzzer);  //Buzzer on
        rPinState = HIGH;     //Relay 1 on
        update = true;        //Update Outputs
      }
      ultaRead = false;         //Cancel UltraSonic read after each sample
    }

    if (!armed) {                     //Events enabled, Sensors disabled
      eventCount++;                   //Plus 1 each duty cycle
      blinkCount++;
      if (blinkCount >= THREE) {  //Blink LED every 300mS
        digitalWrite(ledPin, (ledState = !ledState));
        blinkCount = 0;
        if (digitalRead(buttonPin) == LOW) {
          armed = true; 
        }                                     // IF Button is pressed, armed = true
      }
      if ((eventCount == MINUTE) && (armed == true)) { //25,000mS Arm sensors and LED ON solid

        digitalWrite(ledPin, HIGH);
      } else if (eventCount == SIXSEC) { //6000mS Buzzer and 2 x relays are turned off
        noTone(pinBuzzer);
        bPinState = LOW;
        rPinState = LOW;
        update = true;
      } else if (eventCount == THREE) {  //300mS delay for On state of bPin
        bPinState = HIGH;
        update = true;
      }
    }

    if (update) {                     //So we only write to the ouputs when needed
      digitalWrite(rPin, rPinState);
      digitalWrite(bPin, bPinState);
      update = false;
    }
  }
}
