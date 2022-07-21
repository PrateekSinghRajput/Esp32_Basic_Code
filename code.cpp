//code 1

//Prateek
//www.prateks.in
//https://www.youtube.com/c/JustDoElectronics

void setup() {
  // put your setup code here.
  Serial.begin(115200);
  Serial.println("Hello World!");
}

void loop() {
  // put your main code here.

}


//Code 2
//Prateek
//www.prateks.in
//https://www.youtube.com/c/JustDoElectronics

void setup() {
  // put your setup code here.
  Serial.begin(115200);
  Serial.println("Hi i am from JustDoElectronics this message from (Void Setup)");
}

void loop() {
  // put your main code here.
  Serial.println("Hi i am from JustDoElectronics this message from (Void Loop)");
}


//Code 3

//Prateek
//www.prateks.in
//https://www.youtube.com/c/JustDoElectronics

#define BUTTON_PIN 33  // GIOP33 pin connected to button

int lastState = LOW;  
int currentState;     

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  currentState = digitalRead(BUTTON_PIN);

  if (lastState == HIGH && currentState == LOW)
    Serial.println("The button is pressed");
  else if (lastState == LOW && currentState == HIGH)
    Serial.println("The button is released");
  lastState = currentState;
}


//Code 4

//Prateek
//www.prateks.in
//https://www.youtube.com/c/JustDoElectronics

#define BUTTON_PIN       33   // GIOP33 pin connected to button
#define SHORT_PRESS_TIME 1000 // 1000 milliseconds
#define LONG_PRESS_TIME  1000 // 1000 milliseconds

// Variables will change:
int lastState = LOW;  
int currentState;  
unsigned long pressedTime  = 0;
unsigned long releasedTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  
  currentState = digitalRead(BUTTON_PIN);

  if (lastState == HIGH && currentState == LOW)       
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH) {
    releasedTime = millis();

    long pressDuration = releasedTime - pressedTime;

    if ( pressDuration < SHORT_PRESS_TIME )
      Serial.println("A short press is detected");

    if ( pressDuration > LONG_PRESS_TIME )
      Serial.println("A long press is detected");
  }

  lastState = currentState;
}

//Code 5


//Prateek
//www.prateks.in
//https://www.youtube.com/c/JustDoElectronics

#define BUTTON_PIN 33  // ESP32 pin GIOP33, which connected to button
#define LED_PIN    14  // ESP32 pin GIOP14, which connected to led

int button_state = 0;  

void setup() {

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {

  button_state = digitalRead(BUTTON_PIN);
  if (button_state == LOW)      
    digitalWrite(LED_PIN, HIGH); 
  else                          
    digitalWrite(LED_PIN, LOW); 
}


//Code 6


//Prateek
//www.prateks.in
//https://www.youtube.com/c/JustDoElectronics

#define BUTTON_PIN 33  // ESP32 pin GIOP22 connected to button's pin
#define LED_PIN  14  // ESP32 pin GIOP27 connected to relay's pin
#define RELAY_PIN  26  // ESP32 pin GIOP27 connected to relay's pin

void setup() {
  Serial.begin(9600);                
  pinMode(BUTTON_PIN, INPUT_PULLUP); 
   pinMode(LED_PIN, OUTPUT);         
  pinMode(RELAY_PIN, OUTPUT);       
}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN); // read new state

  if (buttonState == LOW) {
    Serial.println("The button is being pressed");
    digitalWrite(RELAY_PIN, HIGH); // turn on
     digitalWrite(LED_PIN, HIGH); // turn on
  }
  else if (buttonState == HIGH) {
    Serial.println("The button is unpressed");
    digitalWrite(RELAY_PIN, LOW);  // turn off
     digitalWrite(LED_PIN, LOW);  // turn off
  }
}

//Code 7

//Prateek
//www.prateks.in
//https://www.youtube.com/c/JustDoElectronics
#include <ezButton.h>

#define BUTTON_PIN 33 // ESP32 pin GIOP33 connected to button's pin
#define RELAY_PIN  26  // ESP32 pin GIOP26 connected to relay's pin
#define LED_PIN  14  // ESP32 pin GIOP14 connected to relay's pin

ezButton button(BUTTON_PIN);
int relay_state = LOW;
int led_state = LOW;

void setup() {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  button.setDebounceTime(50);
}

void loop() {
  button.loop(); // MUST call the loop() function first

  if (button.isPressed()) {
    Serial.println("The button is pressed");
    relay_state = !relay_state;
    digitalWrite(RELAY_PIN, relay_state);
    led_state = !led_state;
    digitalWrite(LED_PIN, led_state);
  
  }
}



//Code 8


//Prateek
//www.prateks.in
//https://www.youtube.com/c/JustDoElectronics

#define BUTTON_PIN 33 // ESP32 GIOP16 pin connected to button's pin
#define BUZZER_PIN 14 // ESP32 GIOP21 pin connected to Buzzer's pin

void setup() {
  Serial.begin(9600);                
  pinMode(BUTTON_PIN, INPUT_PULLUP); 
  pinMode(BUZZER_PIN, OUTPUT);    
}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == LOW) {
    Serial.println("The button is being pressed");
    digitalWrite(BUZZER_PIN, HIGH); 
  }
  else
  if (buttonState == HIGH) {
    Serial.println("The button is unpressed");
    digitalWrite(BUZZER_PIN, LOW);  
  }
}



//Code 9

//Prateek
//www.prateks.in
//https://www.youtube.com/c/JustDoElectronics

#include <ESP32Servo.h>
#include <ezButton.h>
#define BUTTON_PIN 33 // ESP32 pin GIOP21 connected to button's pin
#define SERVO_PIN  26 // ESP32 pin GIOP26 connected to servo motor's pin
ezButton button(BUTTON_PIN); 
Servo servo;                 
int angle = 0; 

void setup() {
  Serial.begin(9600);         
  button.setDebounceTime(50); 
  servo.attach(SERVO_PIN);   

  servo.write(angle);
}

void loop() {
  button.loop(); 

  if (button.isPressed()) {
    if (angle == 0)
      angle = 90;
    else if (angle == 90)
      angle = 0;
    Serial.print("The button is pressed => rotate servo to ");
    Serial.print(angle);
    Serial.println("°");
    servo.write(angle);
  }
}





//Code 10
//Esp32 Ultersonic Sensor Interfacing

//Prateek
//www.prateks.in
//https://www.youtube.com/c/JustDoElectronics

#define TRIG_PIN 33 // ESP32 pin GIOP33 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN 22 // ESP32 pin GIOP22 connected to Ultrasonic Sensor's ECHO pin

float duration_us, distance_cm;

void setup() {
  // begin serial port
  Serial.begin (9600);

  // configure the trigger pin to output mode
  pinMode(TRIG_PIN, OUTPUT);
  // configure the echo pin to input mode
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(ECHO_PIN, HIGH);

  // calculate the distance
  distance_cm = 0.017 * duration_us;

  // print the value to Serial Monitor
  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  delay(500);
}



//Code 11
//Esp32 LDR Interfacing

//Prateek
//www.prateks.in
//https://www.youtube.com/c/JustDoElectronics

#define LIGHT_SENSOR_PIN 33 // ESP32 pin GIOP33 (ADC0)

void setup() {

  Serial.begin(9600);
}

void loop() {
  int analogValue = analogRead(LIGHT_SENSOR_PIN);

  Serial.print("Analog Value = ");
  Serial.print(analogValue);  


  if (analogValue < 40) {
    Serial.println(" => Dark");
  } else if (analogValue < 2000) {
    Serial.println(" => Dim");
  } else if (analogValue < 2500) {
    Serial.println(" => Light");
  } else if (analogValue < 4000) {
    Serial.println(" => Bright");
  } else {
    Serial.println(" => Very bright");
  }

  delay(500);
}


//Code 12
//Esp32 Interfacing LM35 Sensor
//Prateek
//www.prateks.in
//https://www.youtube.com/c/JustDoElectronics

#define ADC_VREF_mV    3300.0 
#define ADC_RESOLUTION 4096.0
#define PIN_LM35       33 

void setup() {
  Serial.begin(9600);
}

void loop() {
  int adcVal = analogRead(PIN_LM35);
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  float tempC = milliVolt / 10;
  float tempF = tempC * 9 / 5 + 32;

  Serial.print("Temperature: ");
  Serial.print(tempC);   
  Serial.print("°C");
  Serial.print("  ~  "); 
  Serial.print(tempF);   
  Serial.println("°F");

  delay(500);
}


