const byte  ledPin = 13, relayOne = 10, relayTwo = 8, pirPin = 12, buttonPin = 9;

bool isMotion = false, update = false, armed = false, distanceRead = false;   //Come back and change "is motion" to true and see if that changes anything
     
uint8_t eventCount = 0, relayOneState = LOW, relayTwoState = LOW, ledState = HIGH, blinkCount = 0;
            
uint16_t distance;

uint32_t prevTime = 0;

int trigPin = 7;      //defines the trigger pin from Ultrasonic Sensor
int echoPin = 6;      //defines the echo pin from Ultrasonic Sensor

// #define MINUTE 200      //200 x DUTY = 20,000 or 20 seconds
#define DUTY 100
#define SIXSEC 60       //60 x DUTY = 6000 (Six Seconds)
#define THREE 3         //For LED blink @ 3 x DUTY or 300mS rate

int calibrationTime = 30;


void setup() {
  //Setup pins other than the Ultrasonic Sensor
  pinMode(ledPin, OUTPUT);                //Sets LED as output
  pinMode(relayOne, OUTPUT);              //sets relay one (Soundboard) as output
  pinMode(relayTwo, OUTPUT);              //sets relay two (Pneumatics) as output
  pinMode(buttonPin, INPUT_PULLUP);       // Uses onboard pullup resistor as button input
  digitalWrite(relayOne, relayOneState);  //writes relayOne to "LOW" as stated in the init.
  digitalWrite(relayTwo, relayTwoState);  //writes relayTwo to "LOW" as stated in the init.
  digitalWrite(ledPin, ledState);         //writes LED to "LOW" as stated in the init.

  pinMode(pirPin, INPUT);                 //sets Motion Sensor as input

  Serial.begin (9600);


  Serial.print("calibrating sensor "); //give the sensor some time to calibrate
  for (int i = 0; i < calibrationTime; i++) { //will count to 30 seconds before sensor is active
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" done");
  Serial.println("SENSOR ACTIVE");
  delay(50);
//Start Calibration of Ultrasonic Sensor
pinMode (trigPin, OUTPUT);               //Sets TRIGGER pin as output
pinMode (echoPin, INPUT);                //Sets ECHO pin as input
delay (1000);
}
//command for ititial ultrasonic sensor reading
uint32_t duration(uint8_t triggerPin, uint8_t echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(triggerPin, LOW);
  return pulseIn(echoPin, HIGH);
  Serial.println("Distance Calibrated");
  delay (1000); 
  Serial.println("System is armed");
}

void loop() {

  if (armed) {
    isMotion = digitalRead(pirPin);
    if (isMotion) {
      distanceRead = true;
    }
  }
  if (millis() - prevTime >= DUTY) {
    prevTime = millis();
    if (distanceRead) {   //Distance is in "mm" and it's rounded up/down to nearest "mm"
         distance = ((float)((0.01723 * duration(trigPin, echoPin)) * 10) + 0.5); //Take a single distance read to see if object is 2000mm() or closer

      if (distance <= 2000) {   // if distance is less than or = to 2000 ()
        armed = false;        //Disarm sensors and enable events
        eventCount = 0;       //Zero event timer counter
        blinkCount = 0;       //Zero blink timer counter
        relayOneState = HIGH;     //Relay 1 on            // This might be the issue*****
        update = true;        //Update Outputs
      }
      distanceRead = false;         //Cancel UltraSonic read after each sample
    }

    if (!armed) {                     //If NOT ARMED (Activated) ...Events enabled, Sensors disabled
      eventCount++;                   //Plus 1 each duty cycle
      blinkCount++;
      if (blinkCount >= THREE) {  //Blink LED every 300mS
        digitalWrite(ledPin, (ledState = !ledState)); //
        blinkCount = 0;
        if (digitalRead(buttonPin) == LOW) {
          armed = true;                       // IF Button is pressed, armed = true (this will be how we reset it)
        }                                     
      }
      if (armed == true) { // If armed = true, Arm both sensors and turn LED ON solid
        digitalWrite(ledPin, HIGH);
      } 
      else if (eventCount == SIXSEC) { //6000mS Buzzer and 2 x relays are turned off
        relayTwoState = LOW;
        relayOneState = LOW;
        update = true;
      } else if (eventCount == THREE) {  //300mS delay for On state of relayTwo
        relayTwoState = HIGH;    //here
        update = true;
      }
    }

    if (update) {                     //So we only write to the ouputs when needed
      digitalWrite(relayOne, relayOneState);
      digitalWrite(relayTwo, relayTwoState);
      update = false;
    }
  }
}






// Sub-routines:

//
//void systemArmed (){
//  if (armed) {
//    isMotion = digitalRead(pirPin);
//    if (isMotion) {
//      distanceRead = true;
//    }
//  }
//  if (millis() - prevTime >= DUTY) {
//    prevTime = millis();
//    if (distanceRead) {   //Distance is in "mm" and it's rounded up/down to nearest "mm"
//         distance = ((float)((0.01723 * duration(trigPin, echoPin)) * 10) + 0.5); //Take a single distance read to see if object is 2000mm() or closer
//
//      if (distance <= 2000) {   // if distance is less than or = to 2000 ()
//        armed = false;        //Disarm sensors and enable events
//        eventCount = 0;       //Zero event timer counter
//        blinkCount = 0;       //Zero blink timer counter
//        relayOneState = HIGH;     //Relay 1 on            // This might be the issue*****
//        update = true;        //Update Outputs
//      }
//      distanceRead = false;         //Cancel UltraSonic read after each sample
//    }
//  }

//void systemDisarmed () {
//    if (!armed) {                     //If NOT ARMED (Activated) ...Events enabled, Sensors disabled
//      eventCount++;                   //Plus 1 each duty cycle
//      blinkCount++;
//      if (blinkCount >= THREE) {  //Blink LED every 300mS
//        digitalWrite(ledPin, (ledState = !ledState)); //
//        blinkCount = 0;
//        if (digitalRead(buttonPin) == LOW) {
//          armed = true;                       // IF Button is pressed, armed = true (this will be how we reset it)
//        }                                     
//      }
//      if (armed == true) { // If armed = true, Arm both sensors and turn LED ON solid
//        digitalWrite(ledPin, HIGH);
//      } 
//      else if (eventCount == SIXSEC) { //6000mS Buzzer and 2 x relays are turned off
//        relayTwoState = LOW;
//        relayOneState = LOW;
//        update = true;
//      } else if (eventCount == THREE) {  //300mS delay for On state of relayTwo
//        relayTwoState = HIGH;
//        update = true;
//      }
//    }
//}
