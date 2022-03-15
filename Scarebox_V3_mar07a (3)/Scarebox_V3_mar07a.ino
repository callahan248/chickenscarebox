#include "arduino_secrets.h"

/*
  Sketch generated by the Arduino IoT Cloud Thing "Untitled"
  https://create.arduino.cc/cloud/things/d61532e0-90c1-4fdb-b368-1b1965d3f3e6

  Arduino IoT Cloud Variables description

  The following variables are automatically generated and updated when changes are made to the Thing

  bool armButton;
  bool pir_sensor;
  bool stopButton;
  bool triggerButton;

*/

#include "thingProperties.h"
#define pirPin  9

// ----------- Constants-----------
const int redLed = 2;
const int greenLed = 3;
const int yellowLed = 4;
const int solenoidPin = 5;
const int armedSound = 6;
const int disarmedSound = 7;
const int triggerSound = 8;
const int airSupplyPin = 10;

const int yellowLed_BlinkInterval = 500;
const int redLed_BlinkInterval = 250;

const int yellowBlinkDuration = 500;
const int redBlinkDuration = 250;
const int solenoidDuration = 5000;
const int soundDuration = 100;


//--------------Variables (Changing)-------------

int val = 0;

bool timerComplete;

byte yellowLedState = LOW;
byte redLedState = LOW;
byte greenLedState = LOW;
byte solenoidPinState = LOW;
byte armedsoundState = LOW;
byte disarmedSoundState = LOW;
byte triggerSoundState = LOW;


unsigned long currentMillis = 0;
unsigned long yellowLed_PreviousMillis = 0;
unsigned long redLed_PreviousMillis = 0;
unsigned long solenoid_PreviousMillis = 0;
unsigned long sound_PreviousMillis = 0;


// transition Events
typedef enum
{
  ARM_ON,   // arm button
  TRG_SIG,  // signal from either the PIR sensor, Gyro, or manual button
  ARM_OFF,  // disarm button
  TRG_BTN,  // manual trigger button
  STOP_BTN, // stop button
  TMR_DONE, // event timer
} TransitionEvent_e;

// State Names
typedef enum
{
  DISARMED,
  ARMED,
  TRIGGERED,
} state_e;

int state = DISARMED;

// functions
void TransitionFSM(TransitionEvent_e event);
void EnterState(void);
void DoInState(void);


void setup() {
  pinMode(yellowLed, OUTPUT); // 4
  pinMode(redLed, OUTPUT); // 2
  pinMode(greenLed, OUTPUT); // 3
  pinMode(solenoidPin, OUTPUT); // 5
  pinMode(armedSound, OUTPUT); // 6
  pinMode(disarmedSound, OUTPUT); // 7
  pinMode(triggerSound, OUTPUT); // 8
  pinMode(airSupplyPin, OUTPUT); // 10
  pinMode(pirPin, INPUT); // A5

  Serial.begin(9600);
  delay(1500);

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(4);
  ArduinoCloud.printDebugInfo();

  while (ArduinoCloud.connected() != 1) {
    ArduinoCloud.update();
    delay(500);
  }
}

void loop() {
  ArduinoCloud.update();
  currentMillis = millis();
  DoInState();
}

// These are the transition events
void TransitionFSM(TransitionEvent_e event)
{
  // initial "valid transition = not valid"
  int validTransition = 0;
  switch (state)
  {
    case DISARMED: // While in the disarmed state, allow either the "arm event" or "trigger button" event to transition FSM to "triggered"
      if (event == ARM_ON)
      {
        validTransition = 1;
        state = ARMED;
      }
      else if (event == TRG_BTN)
      {
        validTransition = 1;
        state = TRIGGERED;
      }
      break;

    case ARMED: // While in the armed state, allow either the "disarm event", "trigger signal" or "trigger button" to transition states
      if (event == ARM_OFF)
      {
        validTransition = 1;
        state = DISARMED;
      }
      else if (event == TRG_SIG)
      {
        validTransition = 1;
        state = TRIGGERED;
      }
      else if (event == TRG_BTN)
      {
        validTransition = 1;
        state = TRIGGERED;
      }

      break;

    case TRIGGERED:          // While in the triggered state, wait for the timer to complete. Then, the transition will take it back to the disarmed state
      if (event == STOP_BTN) // Change to TMR_DONE After testing
      {
        validTransition = 1;
        state = DISARMED;
      }

      else if (event == TMR_DONE)
      {
        validTransition = 1;
        state = DISARMED;
      }
      break;

    default:
      // shouldn't get here
      break;
  }
  if (validTransition == 1)
  { // if it's a valid transition, enter the correct state
    EnterState();
  }
}


void EnterState(void) // "enter state" = set the inputs, outputs, and timers for each of the states. This will be what runs in the loop while in the state
{
  switch (state)
  {
    case DISARMED:
      messageUpdate = "SYSTEM DISARMED";
      digitalWrite(yellowLed, HIGH);
      digitalWrite(redLed, LOW);
      digitalWrite(greenLed, HIGH);
      digitalWrite(solenoidPin, LOW);
      digitalWrite(disarmedSound, LOW);
      break;

    case ARMED:
      // set all outputs as appropriate
      messageUpdate = "SYSTEM ARMED";
      digitalWrite(yellowLed, LOW);
      digitalWrite(redLed, HIGH);
      digitalWrite(greenLed, LOW);
      digitalWrite(solenoidPin, LOW);
      digitalWrite(armedSound, LOW);
      break;

    case TRIGGERED:
      // set all outputs as appropriate
      messageUpdate = "SYSTEM TRIGGERED";
      digitalWrite(solenoidPin, LOW);
      digitalWrite(triggerSound, LOW);
      digitalWrite(yellowLed, LOW);
      digitalWrite(redLed, LOW);
      digitalWrite(greenLed, LOW);
      break;

    default:
      // shouldn't get here, but Ill add the disarmed variables for the sake of the arduino code
      messageUpdate = "SYSTEM DISARMED";
      digitalWrite(yellowLed, HIGH);
      digitalWrite(redLed, LOW);
      digitalWrite(greenLed, LOW);
      digitalWrite(solenoidPin, LOW);
      digitalWrite(disarmedSound, LOW);
      break;
  }
}

// "DoInState" calls the transition when the conditions are met
void DoInState(void)  {

  unsigned long currentMillis = millis();

  switch (state)
  {
    case DISARMED:

      BlinkYellowLed ();

      if (triggerButton == HIGH)
      {
        void TransitionFSM(TransitionEvent_e TRG_BTN);
      }
      break;

    case ARMED:
      //|| (Gy > 50 || Gy < -50))
      val = digitalRead(pirPin); // start checking motion detector

      if (val == HIGH)
      {
        void TransitionFSM(TransitionEvent_e TRG_SIG);
      }
      break;

    case TRIGGERED:
      //  updateTriggerSoundState ();

      BlinkRedLed ();
      SolenoidTrigger ();
      

      if (timerComplete == true)
      {
        void TransitionFSM(TransitionEvent_e TMR_DONE);
      }

      if (stopButton == HIGH)
      {
        void TransitionFSM(TransitionEvent_e STOP_BTN);
      }
      break;

    default:
      // shouldn't get here
      break;
  }
}




void BlinkYellowLed () {

  if (yellowLedState == LOW) {

    if (currentMillis - yellowLed_PreviousMillis >= yellowLed_BlinkInterval) {
      yellowLedState = HIGH;
      yellowLed_PreviousMillis += yellowLed_BlinkInterval;
    }
  }
  else {
    if (currentMillis - yellowLed_PreviousMillis >= yellowBlinkDuration) {
      yellowLedState = LOW;
      yellowLed_PreviousMillis += yellowBlinkDuration;
    }
  }
  digitalWrite(yellowLed, yellowLedState);
}

void BlinkRedLed () {
  if (redLedState == LOW) {

    if (currentMillis - redLed_PreviousMillis >= redLed_BlinkInterval) {
      redLedState = HIGH;
      redLed_PreviousMillis += redLed_BlinkInterval;
    }
  }
  else {
    if (currentMillis - redLed_PreviousMillis >= redBlinkDuration) {
      redLedState = LOW;
      redLed_PreviousMillis += redBlinkDuration;
    }
  }
  digitalWrite(redLed, redLedState);
}

void SolenoidTrigger () {

  unsigned long triggerStart = millis();
  if (triggerStart - solenoid_PreviousMillis >= solenoidDuration) {
    digitalWrite(solenoidPin, !digitalRead(solenoidPin));
    solenoid_PreviousMillis = currentMillis;
  }

  if (triggerStart >= solenoidDuration) {
    timerComplete = true;
  }
 }
 
void SoundTriggerActivate () {
  unsigned long soundStart = millis();
  if (soundStart - sound_PreviousMillis >= soundDuration) {
    digitalWrite(triggerSound, !digitalRead(triggerSound));

    sound_PreviousMillis += soundDuration;
  }
}

void onAirSupplyChange()  {
  if (airSupply == HIGH) {
    digitalWrite(airSupplyPin, HIGH);
  } else {
    digitalWrite(airSupplyPin, LOW);
  }
}

void onArmButtonChange()  {
  TransitionEvent_e event;
  if (armButton == true)
  {
    TransitionFSM(ARM_ON);
  }
  else
  {
    TransitionFSM(ARM_OFF);
  }
}

void onTriggerButtonChange()  {
  TransitionEvent_e event;
  if (triggerButton == true)
  {
    TransitionFSM(TRG_BTN);
  }
}

void onStopButtonChange()  {
  TransitionEvent_e event;
  if (stopButton == true)
  {
    TransitionFSM(STOP_BTN);
    (armButton == false);
  }
}

void onPirSensorChange()  {
  TransitionEvent_e event;
  val = digitalRead(pirPin);
  if (val == HIGH)
  {
    TransitionFSM(TRG_SIG);
  }
}

void onMessageUpdateChange()  {

}
