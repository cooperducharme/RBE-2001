#include "rangefinder.h"

#define SPEED_OF_SOUND_CM_MICROSEC 0.034

/**
 * Method to setup the rangefinder pins
 * @params trigPin, echoPin: where its plugged in
 *
 * */
void Rangefinder::setup(int trigPin, int echoPin) {
  tPin = trigPin;
  ePin = echoPin;
  pinMode(tPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(ePin, INPUT);  // Sets the echoPin as an Input

  startTime = millis();
}

/**
 * Method to run in each iteration of the main loop to update distance
 *
 * */
void Rangefinder::loop() {
  if ((millis() - startTime) % 100 == 0) {
    updateDistance();
  }


}

/**
 * Method to force the class to get the distance now
 *
 * */
void Rangefinder::updateDistance() {
  // Sets the trigPin on HIGH state for 10 micro seconds

  digitalWrite(tPin, LOW);
  delayMicroseconds(2);
  digitalWrite(tPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(tPin, LOW);

  // measure the echo
  long duration = pulseIn(ePin, HIGH);
  float distance = duration * 0.034 / 2; // Speed of sound wave divided by 2
  //Serial.println(distance);
  //delay(500);
}

/**
 * Return the last distance value calculated in the loop
 * returns float distance that updates every 100 ms
 *
 * */
float Rangefinder::getDistanceCM() { return distance; }

void Rangefinder::isr() {
  switch (digitalRead(ePin)) {
  case HIGH:
    echoStart = micros();
    break;

  case LOW:
    distance = (micros() - echoStart) * SPEED_OF_SOUND_CM_MICROSEC / 2.0;
    //Serial.println("Distance updated: " + String(distance));
    break;
  }
}