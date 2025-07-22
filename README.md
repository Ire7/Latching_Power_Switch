# Latching_Power_Switch
# Project: Latching Power Switch System with Sensors (Arduino UNO)

## Overview:
This project includes two tasks:
Task 1: Build a latching power switch using a push button to toggle LEDs.
Task 2: Integrate a PIR motion sensor (digital) and a potentiometer (analog) with Arduino.
-------------------------------------------
## Task 1: Latching Power Switch

### Components:
Arduino UNO
Push button
10k ohm resistor
External LED
220 ohm resistor
Jumper wires
Breadboard

### Wiring:
Button: one pin to 5V, other pin to Pin 9 + 10k ohm to GND
External LED: long leg to Pin 12 through 220 ohm, short leg to GND
Internal LED: Pin 13 used by default

### Code:
// --- Pin Definitions ---
const int buttonPin = 9;            // Manual switch button
const int ledPin = 13;              // Built-in LED
const int externalLedPin = 12;      // External system indicator
const int digitalSensorPin = 7;     // Digital sensor (e.g. motion sensor)
const int analogSensorPin = A0;     // Analog sensor (e.g. potentiometer)

// --- State Variables ---
bool systemState = false;           // ON/OFF toggle
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

unsigned long systemOnTime = 0;
unsigned long autoOffDelay = 10000; // 10 seconds before auto OFF

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(externalLedPin, OUTPUT);
  pinMode(digitalSensorPin, INPUT);

  digitalWrite(ledPin, LOW);
  digitalWrite(externalLedPin, LOW);

  Serial.begin(9600);
  Serial.println("System Ready");
}

void loop() {
  // --- Manual Button Toggle ---
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != currentButtonState) {
      currentButtonState = reading;

      if (currentButtonState == LOW) {
        systemState = !systemState;
        if (systemState) {
          digitalWrite(ledPin, HIGH);
          digitalWrite(externalLedPin, HIGH);
          systemOnTime = millis();
          Serial.println("Manual Toggle: System ON");
        } else {
          digitalWrite(ledPin, LOW);
          digitalWrite(externalLedPin, LOW);
          Serial.println("Manual Toggle: System OFF");
        }
      }
    }
  }

  lastButtonState = reading;

  // --- Auto Power ON via Digital Sensor ---
  int digitalValue = digitalRead(digitalSensorPin);
  if (digitalValue == HIGH && !systemState) {
    systemState = true;
    digitalWrite(ledPin, HIGH);
    digitalWrite(externalLedPin, HIGH);
    systemOnTime = millis();
    Serial.println("Auto ON: Sensor Triggered");
  }

  // --- Read Analog Sensor ---
  int analogValue = analogRead(analogSensorPin);
  Serial.print("Analog Sensor Value: ");
  Serial.println(analogValue);

  // --- Auto Power OFF ---
  if (systemState && (millis() - systemOnTime > autoOffDelay)) {
    systemState = false;
    digitalWrite(ledPin, LOW);
    digitalWrite(externalLedPin, LOW);
    Serial.println("Auto OFF Triggered");
  }
}
### Result:
Button toggles LEDs ON/OFF.
Debounce logic prevents double triggers.

------------------------------------------------
# Task 2: Latching System with Button + External LED (Tinkercad)

## Objective:
Create a simple latching power switch using a push button. Pressing the button toggles the system ON or OFF. The system state is indicated by both the Arduino built-in LED and an external LED connected to digital pin 13.

## Components Used:
Arduino UNO
Breadboard
Push Button
220Ω Resistor
LED
Jumper Wires

##  Connections:
Button connected to digital pin 9 and GND
One leg of the button connected to 5V through a 220Ω resistor (pull-up logic)
External LED connected to digital pin 13 with series 220Ω resistor
GND of Arduino connected to the breadboard's GND rail

##  Code:
cpp
// Latching Power Switch with Button
const int buttonPin = 9;        // Button connected to pin 9
const int externalLedPin = 13;  // External LED connected to pin 13

bool systemState = false;
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);  
  pinMode(externalLedPin, OUTPUT);
  
  digitalWrite(externalLedPin, LOW);
  
  Serial.begin(9600);
  Serial.println("Latching Power Switch Ready");
  Serial.println("Press button to toggle power");
}

void loop() {
  int reading = digitalRead(buttonPin);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != currentButtonState) {
      currentButtonState = reading;
      
      if (currentButtonState == LOW) {
        systemState = !systemState;
        
        if (systemState) {
          digitalWrite(externalLedPin, HIGH);
          Serial.println("System ON");
        } else {
          digitalWrite(externalLedPin, LOW);
          Serial.println("System OFF");
        }
      }
    }
  }
  
  lastButtonState = reading;
}
```
## Output:
When the Arduino starts, it prints:
  Latching Power Switch Ready
    Press button to toggle power
  On every button press:
  The LED on pin 13 turns ON or OFF
    Serial Monitor displays: System ON or System OFF
    The system retains its state until the next button press
