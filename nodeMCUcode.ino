
//////////Please install all libraries used in your Arduino IDE for this code to work
////////////////
/////////Firebase and Wi-Fi//////////
#include "FirebaseESP8266.h"
#include <ESP8266WiFi.h>
#define FIREBASE_HOST "your firebase project name.firebaseio.com"
#define FIREBASE_AUTH "your firebase auth key"
#define WIFI_SSID "your wifi ssid"
#define WIFI_PASSWORD "your wifi password"
FirebaseData firebaseData;
FirebaseJson jsonTotal;
FirebaseJson jsonGet;
FirebaseJsonData jsonObj;
String jsonStr = "";
const long firebaseReadWritePeriod = 5000;
unsigned long firebasePreviousMillis = 0;
////////////////////////////////


//////////PIR motion sensor//////
#define motionSensor D0
char motionDetected[4] = "No";
///////////////////////////////

////////Engine and Door Relays/////////
#define engineRelay D4
#define doorRelay  D3
char engineState[4] = "Off";
char doorState[7] = "Closed";
//////////////////////////////////////

/////////Alcohol sensor//////////////
#define alcoholSensor A0
float alcoholLevel = 0;
////////////////////////////////////

/////////Alarm/////////////////////
#define alarm D8
const long alarmInterval = 3000;
unsigned long alarmPreviousMillis = 0;
int soundAlarm = 0;
////////////////////////////////////

/////////////Mpu////////////////////
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// Select SDA and SCL pins for I2C communication
const uint8_t scl = D1;
const uint8_t sda = D2;
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN D7

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t temperature;
double T;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

ICACHE_RAM_ATTR void dmpDataReady() {
  mpuInterrupt = true;
}
char towingDetected[4] = "No";
//////////////////////////////////

////////Sim808/////////////
#include <DFRobot_sim808.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
TinyGPS gps;
#define PIN_RX    D5
#define PIN_TX    D6
SoftwareSerial mySerial(PIN_RX, PIN_TX);
DFRobot_SIM808 sim808(&mySerial);
#define MESSAGE_LENGTH 160
#define PHONE_NUMBER "+2347055830564"
//#define PHONE_NUMBER "+2348109566601"
char message[MESSAGE_LENGTH];
int messageIndex = 0;
char phone[16];
char datetime[24];
char MESSAGE[300];
char latitude[12] = "";///9.052482
char longitude[12] = "";//7.455469
float flat, flon;
unsigned long age;
bool newData = false;
////////////////////////////////
const long SMSreadperiod = 15000;
unsigned long SMSPreviousMillis = 0;

/////////////LCD///////////////////
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F, 16, 2);
int toPrint = 1;
const long LCDperiod = 2000;
unsigned long LCDPreviousMillis = 0;
///////////////////////////////

void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
  GPS_GSM_setup();
  WiFiSetup();
  MPU_setup();
  lcd.init();
  lcd.backlight();
  pinMode(motionSensor, INPUT);
  pinMode (engineRelay, OUTPUT);
  pinMode (doorRelay, OUTPUT);
  pinMode(alcoholSensor, INPUT);
  pinMode(alarm, OUTPUT);
  lcd.setCursor(0, 0);
  lcd.print("System ready!");
  delay(1000);
}

void GPS_GSM_setup() {
  if (sim808.init()) {

    Serial.println("Sim808 initialized");
  } else {
    Serial.print("Sim808 init error\r\n");
  }

  //************* Turn on the GPS power************
  if (sim808.attachGPS()) {
    Serial.println("Open the GPS power success");
  }
  else {
    Serial.println("Open the GPS power failure");
  }
}

void MPU_setup() {
  Wire.begin(sda, scl);
  Wire.setClock(400000);
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void WiFiSetup() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  delay(3000);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  firebaseData.setBSSLBufferSize(1024, 1024);
  firebaseData.setResponseSize(1024);
  Serial.println("connecting");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("connected: ");
    Serial.println(WiFi.localIP());
    sendToFirebase();
  } else {
    Serial.println("Wi-Fi not connected");
  }
}

void loop() {
  getGPSData();
  loop2();
}

void loop2() {

  senseAlcohol();
  detectMotion();
  ESP.wdtFeed();
  if (mpu.testConnection()) {
    mpuLoop();
  }
  else {
    MPU_setup();
  }

  printValues();
  ESP.wdtFeed();
  soundAlarmFunc();
  readStates();
  respondtoSMS();
  ESP.wdtFeed();
  LCDprint();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to Wi-Fi");
    ESP.wdtFeed();
    receiveFromFirebase();
  }

  ESP.wdtFeed();
  Serial.println("");
}

void getGPSData() {

  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (mySerial.available())
    {
      char c = mySerial.read();
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("Latitude: ");
    Serial.println(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print("Longitude: ");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print("Satellites fixed to: ");
    dtostrf(flat, 6, 6, latitude);
    dtostrf(flon, 6, 6, longitude);
    Serial.println(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print("PREC: ");
    Serial.println(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }

}

void soundAlarmFunc() {
  if (soundAlarm == 1) {
    unsigned long currentMillis = millis();
    digitalWrite(alarm, HIGH);
    Serial.println("Alarm on");
    if (currentMillis - alarmPreviousMillis >= alarmInterval) {
      alarmPreviousMillis = currentMillis ;
      digitalWrite(alarm, LOW);
      Serial.println("Alarm off");
      soundAlarm = 0;
    }
  }
}

void senseAlcohol() {
  alcoholLevel = analogRead(alcoholSensor);
  alcoholLevel = (alcoholLevel / 1024) * 100;
  Serial.print("AlcoholLevel:");
  Serial.print("\t");
  Serial.print(alcoholLevel);
  Serial.print("\t");
  if (alcoholLevel > 80) {
    soundAlarm = 1;
    Serial.println("Alcohol level is high");
    digitalWrite(engineRelay, LOW);
    lcd.clear();
    lcd.print("Alcohol: High");
  } else {
    Serial.println("Alcohol level is low");
  }
}

void detectMotion() {
  if (digitalRead(motionSensor)) {
    soundAlarm = 1;
    strcpy(motionDetected, "Yes");
    Serial.println("Motion detected");
    lcd.clear();
    lcd.print("motion detected");
  } else {
    strcpy(motionDetected, "No");
    Serial.println("No motion ");
  }
}

void engineControl(String msg) {
  if (msg == "engineoff") {
    digitalWrite(engineRelay, LOW);
    strcpy(engineState, "Off");
    Serial.print("Engine is off");
    strcpy(MESSAGE, "Engine turned off");
    sendSMS();
  } else if (msg == "engineon") {
    digitalWrite(engineRelay, HIGH);
    strcpy(engineState, "On");
    Serial.println("Engine is on");
    strcpy(MESSAGE, "Engine turned on");
    sendSMS();
  }
}

void doorControl(String msg) {
  if (msg == "dooroff") {
    digitalWrite(doorRelay, LOW);
    strcpy(doorState, "Open");
    Serial.print("Door is open");
    strcpy(MESSAGE, "Door is open");
    sendSMS();
  } else if (msg == "dooron") {
    digitalWrite(doorRelay, HIGH);
    strcpy(doorState, "Closed");
    Serial.println("Door is locked");
    strcpy(MESSAGE, "Door is closed");
    sendSMS();
  }
}



void respondtoSMS() {

  unsigned long currentMillis = millis();
  if (currentMillis - SMSPreviousMillis >= SMSreadperiod) {
    SMSPreviousMillis = currentMillis;
    if (sim808.checkPowerUp()) {
      strcpy(message, "");
      strcpy(phone, "");
      messageIndex = sim808.isSMSunread();
      Serial.println(messageIndex);
      if (messageIndex > 0 && messageIndex < 255) {
        sim808.readSMS(messageIndex, message, MESSAGE_LENGTH, phone, datetime);
        sim808.deleteSMS(messageIndex);
        Serial.print("Message from: ");
        Serial.println(phone);
        Serial.print("Message is: ");
        Serial.println(message);
        lcd.clear();
        lcd.print("Message:");
        lcd.setCursor(8, 0);
        lcd.print(message);
        lcd.setCursor(0, 1);
        lcd.print(phone);
        String msg = message;
        String phoneNo = phone;
        if (phoneNo == PHONE_NUMBER) {
          if (msg == "Location") {
            sprintf(MESSAGE, "http://maps.google.com/maps?q=%s,%s", latitude, longitude);
            sendSMS();
          }
          if (msg == "engineon") {
            engineControl(msg);
          }
          if (msg == "engineoff") {
            engineControl(msg);
          }
          if (msg == "dooron") {
            doorControl(msg);
          }
          if (msg == "dooroff") {
            doorControl(msg);
          }
          if (msg == "alcohol") {
            alcoholLevel = analogRead(alcoholSensor);
            alcoholLevel = (alcoholLevel / 1024) * 100;
            if (alcoholLevel > 80) {
              sprintf(MESSAGE, "Alcohol level is high: %f percent", alcoholLevel);
              sendSMS();
            }
            else if (alcoholLevel < 80) {
              sprintf(MESSAGE, "Alcohol level is low: %f percent", alcoholLevel);
              sendSMS();
            }
          }
          if (msg == "capture") {
            if (WiFi.status() == WL_CONNECTED) {
              Firebase.setString (firebaseData, "omeizaalabi@gmail/Capture_image", "Yes");
            }
          }

        }
      }
    }
  }
}


void mpuLoop() {
  if (!dmpReady) {}
  else {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("AcceGyro data\t");
      Serial.print("Yaw\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\tPitch\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\tRoll\t");
      Serial.println(ypr[2] * 180 / M_PI);
      if (abs(ypr[1] * 180 / M_PI) > 30 || abs(ypr[2] * 180 / M_PI) > 30) {
        strcpy(towingDetected, "Yes");
        Serial.println("Towing action detected");
        lcd.clear();
        lcd.print("Towing detected");
      } else {
        strcpy(towingDetected, "No");
        Serial.println("No towing action detected");
      }

    }
    temperature = mpu.getTemperature();
    T = (double)temperature / 340 + 36.53;
    Serial.print("Temperature:");
    Serial.print("\t");
    Serial.print(T);
    if (T > 60) {
      soundAlarm = 1;
      Serial.println(" Temperature is high");
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("Temperature: High");
    } else {
      Serial.println(" Temperature is low");
    }
  }
}



void sendSMS() {
  sim808.sendSMS(PHONE_NUMBER, MESSAGE);
}

void readStates() {
  if (digitalRead(engineRelay)) {
    strcpy(engineState, "On");
  } else {
    strcpy(engineState, "Off");
  }

  if (digitalRead(engineRelay)) {
    strcpy(engineState, "On");
  } else {
    strcpy(engineState, "Off");
  }

}

void printValues() {
  Serial.print("Engine is: ");
  Serial.print("\t");
  Serial.println(engineState);
  Serial.print("Door is: ");
  Serial.print("\t");
  Serial.println(doorState);
}

void LCDprint() {
  unsigned long currentMillis = millis();
  if (currentMillis - LCDPreviousMillis >= LCDperiod) {
    LCDPreviousMillis = currentMillis;
    if (toPrint > 3) {
      toPrint = 1;
    }
    if (toPrint == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Lat: ");
      lcd.setCursor(6, 0);
      lcd.print(latitude);
      lcd.setCursor(0, 1);
      lcd.print("Long: ");
      lcd.setCursor(7, 1);
      lcd.print(longitude);
    }
    else if (toPrint == 2) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.setCursor(7, 0);
      lcd.print(T);
      lcd.setCursor(0, 1);
      lcd.print("Alcohol: ");
      lcd.setCursor(9, 1);
      lcd.print(alcoholLevel);
    }
    else if (toPrint == 3) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("EngLock: ");
      lcd.setCursor(9, 0);
      lcd.print(engineState);
      lcd.setCursor(0, 1);
      lcd.print("DoorLock: ");
      lcd.setCursor(10, 1);
      lcd.print(doorState);
    }
    toPrint ++;
  }
}


void receiveFromFirebase() {
  Firebase.setReadTimeout(firebaseData, 1000);
  if (Firebase.get(firebaseData, "omeizaalabi@gmail/vehicleConditions")) {
    jsonStr = firebaseData.jsonString();
    jsonGet.setJsonData(jsonStr);

    jsonGet.get(jsonObj, "Engine_state");
    if (jsonObj.stringValue == "Off") {
      digitalWrite(engineRelay, LOW);
      strcpy(engineState, "Off");
      Serial.println("Database says: Engine to be turned off ");
    } else if (jsonObj.stringValue == "On") {
      digitalWrite(engineRelay, HIGH);
      strcpy(engineState, "On");
      Serial.println("Database says: Engine to be turned on ");
    }

    jsonGet.get(jsonObj, "Door_state");
    if (jsonObj.stringValue == "Open") {
      digitalWrite(doorRelay, LOW);
      strcpy(doorState, "Open");
      Serial.println("Database says: Door to be open ");
    } else if (jsonObj.stringValue == "Closed") {
      digitalWrite(doorRelay, HIGH);
      strcpy(doorState, "Closed");
      Serial.println("Database says: Door to be closed ");
    }
    sendToFirebase();
  } else {

    Serial.println("Database read failed");
  }
}


void sendToFirebase() {
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
  unsigned long currentMillis = millis();
  if (currentMillis - firebasePreviousMillis >= firebaseReadWritePeriod) {
    firebasePreviousMillis = currentMillis;
    jsonTotal.set("Alcohol_value", String(alcoholLevel));
    jsonTotal.set("Door_state", String(doorState));
    jsonTotal.set("Engine's_Temperature", T);
    jsonTotal.set("Engine_state", String(engineState));
    jsonTotal.set("Location_lat", String(latitude));
    jsonTotal.set("Location_long", String(longitude));
    jsonTotal.set("MPU_data/yaw", ypr[0] * 180 / M_PI);
    jsonTotal.set("MPU_data/pitch", ypr[1] * 180 / M_PI);
    jsonTotal.set("MPU_data/roll", ypr[2] * 180 / M_PI);
    jsonTotal.set("Motion_detected", String(motionDetected));
    jsonTotal.set("Towing_detected", String(towingDetected));
    if ( Firebase.set(firebaseData, "omeizaalabi@gmail/vehicleConditions", jsonTotal)) {
      Serial.println("Database write successful");
    } else {
      Serial.println("Database write failed");
    }
  }
}
