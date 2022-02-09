/* This Code will use a motion detector and ultrasonic distance detector to trigger 2 relays.
 *  The LED will be "HIGH" while its waiting for the event to be triggered
 *  The LED will blink on and off at 1 second intervals indefinately until I press the reset button. 
 *  Relay 1 will be the soundboard
 *  Relay 2 will be Pneumatics
 */

const int ledPin = 13;    // Sets pin 13 to + side of LED
const int buttonPin = 9;  // Sets pin 9 to button for reset

int relayOne = 10;  // Sets pin 10 to Relay 1 (Soundboard)
int relayTwo = 8;   // Sets pin 8 to Relay 2 (Pneumatics)

int pirPin = 12;    // Sets pin 12 to Motion Sensor 

//Details for Distance Sensor
#define trigPin  7    // Sets pin 7 to trigger pin of Ultrasonic Sensor (Output)
#define echoPin  6    // Sets pin 6 to Echo pin of Ultrasonic sensor (Input)
long duration;      // Takes time from trigger to echo and creates the duration 
int distance;       // Uses the duration/ speed of sound to calculate distance
long firstoff;      // The initial pulse in 
int distancefirst;  // takes the "firstoff" and multiplies by the speed of sound, divides it by 2 for (time traveled) to (time returned)

int menu = 0; //menu variable to pick each menu item 

void setup() {
//Begin Serial Monitor
Serial.begin(9600); //initializes serial monitor
//Set Inputs and outputs
pinMode(pirPin, INPUT); 
pinMode(buttonPin, INPUT);
pinMode(ledPin, OUTPUT);
pinMode(relayOne, OUTPUT);
pinMode(relayTwo, OUTPUT);
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT); 
delay(1000); 
//gets "first distance" to compare to 
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
firstoff = pulseIn(echoPin, HIGH);
distancefirst = firstoff*0.034/2; // cm

}

void loop() {
  //Displays the menu
 if (menu == 0){
  //Prints the menu to the serial port
    Serial.println("Select an option:");
    Serial.println("-----------------");
    Serial.println("1) Calibrate Distance");
    Serial.println("2) Arm The Chicken");
    Serial.println("3) Disarm The System");
    Serial.flush();
    
    while (!Serial.available()) {}  //Waits for an input on the serial device
    menu = Serial.parseInt();       //Takes the Serial input and looks for an integer
    Serial.flush();

  }
if (menu == 1) {
  
  }

if (menu == 2) {
  
  }

if (menu == 3) {
  
  }
}







//Calibrate distance of the tripwire.

void setDistance ()
{
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
//distance= duration*0.034/2; // cm
distance = duration * 0.0133 / 2; // in
Serial.println(distance);
delay(1000);
Serial.println(distancefirst);
  
}
  


//*
//if (distance <= distancefirst - 5)
//{
//Something Happens
