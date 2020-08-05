///Please install all libraries needed in your Arduino ide for this code to work

/*
  ESP32-CAM Save a captured photo(Base64) to firebase.
  Author : ChungYi Fu (Kaohsiung, Taiwan)  2019-8-16 23:00
  https://www.facebook.com/francefu

  Arduino IDE Library
  Firebase ESP32 Client by Mobizt version 3.2.1

  ESP32-CAM How to save a captured photo to Firebase
  https://youtu.be/Hx7bdpev1ug

  How to set up Firebase
  https://iotdesignpro.com/projects/iot-controlled-led-using-firebase-database-and-esp32
*/

const char* ssid = "your wifi ssid";
const char* password = "your wifi password";


//https://console.firebase.google.com/project/xxxxxxxxxx/settings/serviceaccounts/databasesecrets
String FIREBASE_HOST = "your firebase project name.firebaseio.com";
String FIREBASE_AUTH = "your firebase auth key";
#include "FirebaseESP32.h"
FirebaseData firebaseData;
FirebaseJson json1;
FirebaseJsonData jsonData;
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "Base64.h"

#include "esp_camera.h"
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled

//CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

int LED_PIN = 4; //Built-in LED on ESp-32
String imagePart1;
String imagePart2;
String imagePart3;
String imagePart4;
String imagePart5;
String imagePart6;
String imagePart7;
String imagePart8;
String imagePart9;
String imagePart10;
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  pinMode(LED_PIN, OUTPUT); // Set the digital pin 33 as output for Build-in Led
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("ssid: " + (String)ssid);
  Serial.println("password: " + (String)password);

  WiFi.begin(ssid, password);

  long int StartTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if ((StartTime + 10000) < millis()) break;
  }
   Serial.println("Connected");
//  if (WiFi.status() == WL_CONNECTED) {
//    char* apssid = "ESP32-CAM";
//    char* appassword = "12345678";         //AP password require at least 8 characters.
//    Serial.println("");
//    Serial.print("Camera Ready! Use 'http://");
//    Serial.print(WiFi.localIP());
//    Serial.println("' to connect");
//    WiFi.softAP((WiFi.localIP().toString() + "_" + (String)apssid).c_str(), appassword);
//  }
//  else {
//    Serial.println("Connection failed");
//    return;
//  }

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  //drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  s->set_framesize(s, FRAMESIZE_VGA);  // VGA|CIF|QVGA|HQVGA|QQVGA   ( UXGA? SXGA? XGA? SVGA? )
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  Firebase.setMaxRetry(firebaseData, 3);
  Firebase.setMaxErrorQueue(firebaseData, 30);
  Firebase.enableClassicRequest(firebaseData, true);


  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "medium");

}


void loop() {
  if (Firebase.getString(firebaseData, "omeizaalabi@gmail/Capture_image")) {
    if (firebaseData.stringData() == "Yes") {
      Photo2Base64();
      if (Firebase.setString(firebaseData, "omeizaalabi@gmail/Photo_capture0", imagePart1)) {
        if (Firebase.setString(firebaseData, "omeizaalabi@gmail/Photo_capture1", imagePart2)) {
          if (Firebase.setString(firebaseData, "omeizaalabi@gmail/Photo_capture2", imagePart3)) {
            if (Firebase.setString(firebaseData, "omeizaalabi@gmail/Photo_capture3", imagePart4)) {
              if (Firebase.setString(firebaseData, "omeizaalabi@gmail/Photo_capture4", imagePart5)) {
                if (Firebase.setString(firebaseData, "omeizaalabi@gmail/Photo_capture5", imagePart6)) {
                  if (Firebase.setString(firebaseData, "omeizaalabi@gmail/Photo_capture6", imagePart7)) {
                    if (Firebase.setString(firebaseData, "omeizaalabi@gmail/Photo_capture7", imagePart8)) {
                      if (Firebase.setString(firebaseData, "omeizaalabi@gmail/Photo_capture8", imagePart9)) {
                        if (Firebase.setString(firebaseData, "omeizaalabi@gmail/Photo_capture9", imagePart10)) {
                          Firebase.setString (firebaseData, "omeizaalabi@gmail/Capture_image", "No");
                          Serial.println("Image sent successfully");
                        } else {
                          Serial.println("Image send failed");
                          Serial.println("REASON: " + firebaseData.errorReason());
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  delay(10000);
}

String Photo2Base64() {
  camera_fb_t * fb = NULL;
  digitalWrite(LED_PIN, HIGH);
  fb = esp_camera_fb_get();
  digitalWrite(LED_PIN, LOW);
  if (!fb) {
    Serial.println("Camera capture failed");
    return "";
  }else {
    Serial.println("Camera capture success");
    }

  String imageFile = "data:image/jpeg;base64,";
  char *input = (char *)fb->buf;
  char output[base64_enc_len(3)];
  for (int i = 0; i < fb->len; i++) {
    base64_encode(output, (input++), 3);
    if (i % 3 == 0) imageFile += urlencode(String(output));
  }
  unsigned long lengthOfbase64 = round(imageFile.length() / 10);
  imagePart1 = imageFile.substring(0, lengthOfbase64);
  imagePart2 = imageFile.substring(lengthOfbase64, lengthOfbase64 * 2);
  imagePart3 = imageFile.substring((lengthOfbase64 * 2), lengthOfbase64 * 3);
  imagePart4 = imageFile.substring((lengthOfbase64 * 3), lengthOfbase64 * 4);
  imagePart5 = imageFile.substring((lengthOfbase64 * 4), lengthOfbase64 * 5);
  imagePart6 = imageFile.substring((lengthOfbase64 * 5), lengthOfbase64 * 6);
  imagePart7 = imageFile.substring((lengthOfbase64 * 6), lengthOfbase64 * 7);
  imagePart8 = imageFile.substring((lengthOfbase64 * 7), lengthOfbase64 * 8);
  imagePart9 = imageFile.substring((lengthOfbase64 * 8), lengthOfbase64 * 9);
  imagePart10 = imageFile.substring((lengthOfbase64 * 9));
  esp_camera_fb_return(fb);
  return imageFile;

}

//https://github.com/zenmanenergy/ESP8266-Arduino-Examples/
String urlencode(String str)
{
  String encodedString = "";
  char c;
  char code0;
  char code1;
  char code2;
  for (int i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (c == ' ') {
      encodedString += '+';
    } else if (isalnum(c)) {
      encodedString += c;
    } else {
      code1 = (c & 0xf) + '0';
      if ((c & 0xf) > 9) {
        code1 = (c & 0xf) - 10 + 'A';
      }
      c = (c >> 4) & 0xf;
      code0 = c + '0';
      if (c > 9) {
        code0 = c - 10 + 'A';
      }
      code2 = '\0';
      encodedString += '%';
      encodedString += code0;
      encodedString += code1;
      //encodedString+=code2;
    }
    yield();
  }
  return encodedString;
}
