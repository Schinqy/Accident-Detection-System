#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>


String accidentPhoneNumber = "+263717596521";
String firePhoneNumber = "+263717596521";

float lati;
float lngi;

int buzzer = 12;

TinyGPSPlus gps;
Adafruit_MPU6050 mpu;

// Tilt threshold

const float tiltThreshold = 60.0;
bool isTiltedLeft = false;
bool isTiltedRight = false;
bool isUpsideDown = false;


// Thresholds for accident detection
const float accelerationThreshold = 13.0; // Adjust this value based on your requirements
const float temperatureThreshold = 40.0; // Adjust this value based on your requirements

unsigned long lastSMSTime = 0;
const unsigned long smsInterval = 300000; // 5 minutes

void setup() {
  Serial.begin(9600);
   Serial1.begin(9600, SERIAL_8N1, 18, 19); // Use Hardware Serial 1 on pins 18 (RX) and 19 (TX)
  Serial2.begin(9600);
    delay(1000);
    pinMode(buzzer, OUTPUT);
  sendCommand("AT"); // Check if GSM module is responding

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println(TinyGPSPlus::libraryVersion());
  sendSMS(accidentPhoneNumber, "SYSTEM ON");
  digitalWrite(buzzer, HIGH);
  delay(500);
  digitalWrite(buzzer, LOW);
  delay(500);
  digitalWrite(buzzer, HIGH);
  delay(500);
  digitalWrite(buzzer, LOW);
  delay(500);
  
  delay(1000); // Give the sensors and SIM800L module some time to stabilize
}

void loop() {
  // Read accelerometer data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Read GPS data
  readGPS();

  // Calculate acceleration magnitude
  float acceleration = sqrt(pow(a.acceleration.x, 2) +
                            pow(a.acceleration.y, 2) +
                            pow(a.acceleration.z, 2));

  Serial.println("Acceleration: " + String(acceleration));
  
  //******************** Calculate tilt angles **************************
  float xAngle = atan2(a.acceleration.x, sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * (180 / PI);
float yAngle = atan2(a.acceleration.y, sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * (180 / PI);
float zAngle = atan2(sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2)), a.acceleration.z) * (180 / PI);

  //******************** set Flags **************************

isTiltedLeft = (xAngle > tiltThreshold);
isTiltedRight = (xAngle < -tiltThreshold);
isUpsideDown = (zAngle < -tiltThreshold);

  //******************* Read temperature ***********************
  float temperature;
  temperature = temp.temperature;
  Serial.println("Temp: " + String(temperature));

  // Check if any of the values exceed the thresholds
  if (acceleration > accelerationThreshold) {
     digitalWrite(buzzer, HIGH);
  delay(300);
  digitalWrite(buzzer, LOW);
  delay(300);
  digitalWrite(buzzer, HIGH);
  delay(300);
  digitalWrite(buzzer, LOW);
  delay(300);
  digitalWrite(buzzer, LOW);
  delay(300);
  digitalWrite(buzzer, HIGH);
  delay(300);
  digitalWrite(buzzer, LOW);
  delay(300);
    
    String accidentMsg = "Possible Accident! Temp:" + String(temperature, 2) + "*C. Speed: " + String(gps.speed.kmph(),2) + 
    ". http://maps.google.com/?ie=UTF8&hq=&ll=" + String(lati,6) + "," + String(lngi,6) + "&z=13";
   // http://maps.google.com/?ie=UTF8&hq=&ll=35.028028,-106.536655&z=13
    
 
    sendSMS(accidentPhoneNumber, accidentMsg);  
  }

  
    if ( temperature > temperatureThreshold){
      String fireMsg = "Vehicle likely on fire! Temp:" + String(temperature, 2) + "*C.";
  
  sendSMS(firePhoneNumber, fireMsg); 
       digitalWrite(buzzer, HIGH);
  delay(300);
  digitalWrite(buzzer, LOW);
  delay(300);
  digitalWrite(buzzer, HIGH);
  delay(300);
  digitalWrite(buzzer, LOW);
  delay(300);
  digitalWrite(buzzer, LOW);
  delay(300);
  digitalWrite(buzzer, HIGH);
  delay(300);
  digitalWrite(buzzer, LOW);
  delay(300);
    
    }

 if( isTiltedLeft || isTiltedRight || isUpsideDown)
 {
  String tiltMsg = "Vehicle tilted! Left:" + String(isTiltedLeft) + ",Right:" + String(isTiltedRight + ",UpsideDown:" + String(isUpsideDown) + ".");
  
  sendSMS(accidentPhoneNumber, tiltMsg); 
       digitalWrite(buzzer, HIGH);
    delay(300);
  digitalWrite(buzzer, LOW);
  delay(300);
  digitalWrite(buzzer, HIGH);
  delay(300);
  digitalWrite(buzzer, LOW);
  delay(300);
  digitalWrite(buzzer, LOW);
  delay(300);
  digitalWrite(buzzer, HIGH);
  delay(300);
  digitalWrite(buzzer, LOW);
  delay(300);
 }

  delay(1000); // Adjust the delay based on your requirements
}


//********************** Read GPS Data ************************
void readGPS() {
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
}

//********************** Display GPS Data ************************
void displayInfo() {
  Serial.print(F("Speed: "));
  if (gps.speed.isValid()) {
    Serial.print(gps.speed.kmph(), 6);
    Serial.print(F(","));
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
    lati = gps.location.lat();
    lngi = gps.location.lng();
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}


void sendCommand(const String& command)
{
  Serial1.println(command);
  delay(1000);

  if (Serial1.available())
  {
    String response = Serial1.readString();
    Serial.println(response);
  }
}

void sendSMS(const String& phoneNumber, const String& message)
{
  sendCommand("AT+CMGF=1"); // Set SMS mode to text
  delay(1000);

  sendCommand("AT+CMGS=\"" + phoneNumber + "\"");
  delay(1000);

  Serial1.print(message);
  delay(100);

  Serial1.write(26); // Ctrl+Z to send the message
  delay(5000);

  if (Serial1.available())
  {
    String response = Serial1.readString();
    Serial.println(response);
  }
}

void makeCall(const String& phoneNumber)
{
  sendCommand("ATD" + phoneNumber + ";");
  delay(1000);

  if (Serial1.available())
  {
    String response = Serial1.readString();
    Serial.println(response);
  }
}

void hangUpCall()
{
  sendCommand("ATH");
  delay(1000);

  if (Serial1.available())
  {
    String response = Serial1.readString();
    Serial.println(response);
  }
}
