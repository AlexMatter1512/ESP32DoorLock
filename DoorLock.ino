
#include "IRremote.h"
#include "ESP32Servo.h"

#include <WiFi.h>

#include <OTA.h>
#include <credentials.h> //where "mySSID" and "myPASSWORD" variables are stored.

//RGB led Pins
#define BLUE 32
#define GREEN 14
#define RED 33

//WiFi connection
int att = 0; //attempts to connect to wifi
IPAddress staticIP(192, 168, 68, 200);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 1, 1);

//infrared reciever
int IR_RECEIVE_PIN = 26;

//servo position in degrees
int pos = 0; 

//open/closed Lock and on/off LED button
int button = 12;

//led status
int stato; // 1=on 0=off

Servo myservo;
decode_results results; 

void setup() {
  
  Serial.begin(115200);
  delay(1000);

  //pins
    //led
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
    //button
  pinMode(button, INPUT_PULLUP);

  //WiFi setup
  if (WiFi.config(staticIP, gateway, subnet, dns, dns) == false) {
    Serial.println("Configuration failed.");
  }
  WiFi.begin(mySSID, myPASSWORD);
  
  //OTA 
  setupOTA("Lucchetto", mySSID, myPASSWORD);

  //checking wifi connection
  while ((WiFi.status() != WL_CONNECTED) && (att != 20)) {
    delay(500);
    Serial.println("Connecting to WiFi..");
    //led blinks blue at every attempt
    digitalWrite(BLUE, HIGH);
    delay(15);
    digitalWrite(BLUE, LOW);
    att++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to the WiFi network");
  } else {
    Serial.println("Not Connected to the WiFi network");
  }

  //infrared
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN); // Start the receiver

  //servo
  myservo.attach(13);  // attaches the servo on pin 13 to the servo object
  myservo.write(pos); //moves the servo to 0°(pos) position
}

void loop() {
  ArduinoOTA.handle();
  
  //infrared signal
  if (IrReceiver.decode()) {
    ArduinoOTA.handle();
    translateIR();
    Stato_Led();
    IrReceiver.resume(); // receive the next value
  }
  
  ButtonPressed();
  
}

void ButtonPressed(){
  //Button pressed
  while (digitalRead(button) == LOW) {
    delay (500);
    if (digitalRead(button) == LOW) {
      if (stato == 0) {
        stato = 1;
        //Lampeggio led blu
        digitalWrite(BLUE, HIGH);
        delay(15);
        digitalWrite(BLUE, LOW);
        Stato_Led();
        ArduinoOTA.handle();
      } else if (stato == 1) {
        stato = 0;
        digitalWrite(BLUE, HIGH);
        delay(15);
        digitalWrite(BLUE, LOW);
        Stato_Led();
        ArduinoOTA.handle();
      }
    } else {
      if (pos == 0) {
        myservo.write(180);
        pos = 180;
      } else {
        myservo.write(0);
        pos = 0;
      }
      Stato_Led();
    }
    delay(1000);
  }
}

void Stato_Led() {
ArduinoOTA.handle();
  if (stato == 1) {
    if (pos == 180) {
      analogWrite(GREEN, 0);
      analogWrite(RED, 5);
    } else {
      analogWrite(RED, 0);
      analogWrite(GREEN, 1);
    }
  } else {
    analogWrite(GREEN, 0);
    analogWrite(RED, 0);
  ArduinoOTA.handle();
  }
}

// takes action based on IR code received
void translateIR() {
  //ArduinoOTA.handle();
  switch (IrReceiver.decodedIRData.command) {

    case 0x7: Serial.println("Giù chiudi");
      myservo.write(180);
      pos = 180;
      break;
    case 0x9: Serial.println("Sù apri");
      myservo.write(0);
      pos = 0;
      break;

    default:
      Serial.println(" other button   ");
      Serial.println(results.value);
  }// End Case

  delay(100); // Do not get immediate repeat
ArduinoOTA.handle();

}
