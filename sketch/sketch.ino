#include <ESP8266WiFi.h>

#include "I2Cdev.h"
#include "MPU6050.h"


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


#define LED_PIN 5

const char WiFiSSID[] = "led_dinosaur_poofer";
const char WiFiPSK[] = "poof4dino";

const int STILL_TIME_BEFORE_SLEEP = 10000;
const int STILL_TIME_AFTER_SLEEP_BEFORE_SLEEP = 1000;
const int CONTINUOUS_MOVEMENT_BEFORE_PING = 15000;

const int COOLDOWN_TIME = 45000;
const int COOLDOWN_BLINK_RATE = 500;

const int SLEEP_SECS = 1;
const bool SLEEP_MODE_ENABLED = false;

const int SAMPLE_RATE = 100;
const int MOVEMENT_THRESHOLD = 5000;

const char* HOSTNAME = "thing.local";
const int HTTP_PORT = 80;



MPU6050 accelgy_ro_;

int time_since_startup_ = 0;
int time_since_movement_ = 0;
int time_with_continuous_movement_ = 0;

bool has_had_movement_ = false;

int16_t ax_, ay_, az_;
int16_t gx_, gy_, gz_;
int16_t prev_gx_, prev_gy_, prev_gz_;
bool have_prev_ = false;

bool has_connected_wifi_ = false;

bool blink_state_ = false;

  
void setup() 
{
  initHardware(); // Setup input/output I/O pins
  initMTU();
  connectWiFi();

  digitalWrite(LED_PIN, LOW); // LED on to indicate connect success
}

void initHardware() {
    // initialize serial communication
    Serial.begin(9600);
    while (!Serial);

    pinMode(LED_PIN, OUTPUT); // Set LED as output
    digitalWrite(LED_PIN, HIGH); // LED off

    Serial.println("Hello World!");
}

void connectWiFi()
{
  byte ledStatus = LOW;
  Serial.println();
  Serial.println("Connecting to: " + String(WiFiSSID));

  WiFi.mode(WIFI_STA);

  WiFi.begin(WiFiSSID, WiFiPSK);

  while (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(LED_PIN, ledStatus); // Write LED high/low
    ledStatus = (ledStatus == HIGH) ? LOW : HIGH;

    delay(100);
  }
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  has_connected_wifi_ = true;   
}


void initMTU() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Serial.println("Using inbuilt");
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Serial.println("Using fastwire");
        Fastwire::setup(400, true);
    #endif

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgy_ro_.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    while( !accelgy_ro_.testConnection() ) {
      Serial.println("MPU6050 connection failed.");
      delay(100);
    }
    Serial.println("MPU6050 connection successful");
}

void contactServer() {
  if (!has_connected_wifi_) {
    connectWiFi(); // Connect to WiFi
  }

  WiFiClient client;
  if (!client.connect(HOSTNAME, HTTP_PORT)) {
    Serial.println("connection failed");
    return;
  }
  
  // This will send the request to the server
  client.print(String("GET /poof") + " HTTP/1.1\r\n" +
               "Host: " + HOSTNAME + "\r\n" + 
               "Connection: close\r\n\r\n");
  delay(10);
  
  // Read all the lines of the reply from server and print them to Serial
  while(client.available()){
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
  
  Serial.println();
  Serial.println("closing connection");
}

void loop() {
  delay(SAMPLE_RATE);
  time_since_startup_ += SAMPLE_RATE;
  time_since_movement_ += SAMPLE_RATE;
  
    // read raw accel/gy_ro measurements from device
    accelgy_ro_.getMotion6(&ax_, &ay_, &az_, &gx_, &gy_, &gz_);
    if (!have_prev_) {
      prev_gx_ = gx_;
      prev_gy_ = gy_;
      prev_gz_ = gz_;
      have_prev_ = true;
      return;
    }
    
    int delta = abs((gx_ - prev_gx_) + (gy_ - prev_gy_) + (gz_ - prev_gz_));

    // display_ tab-separated accel/gy_ro x/y/z values
    //Serial.print("a/g:\t");
//    Serial.print("Delta: " );
//    Serial.print(delta); Serial.print("\t");

    if (delta > MOVEMENT_THRESHOLD) {
      Serial.print(" MOVEMENT! ");
      if (time_since_movement_ == SAMPLE_RATE) {
        time_with_continuous_movement_ += SAMPLE_RATE;
        Serial.print(time_with_continuous_movement_); Serial.println("\t");
      }

      time_since_movement_ = 0;
      has_had_movement_ = true;

      // blink LED to indicate activity
      blink_state_ = !blink_state_;
      digitalWrite(LED_PIN, blink_state_);

      if (time_with_continuous_movement_ > CONTINUOUS_MOVEMENT_BEFORE_PING) {
        digitalWrite(LED_PIN, false);
        time_with_continuous_movement_ = 0;
        contactServer();

        int slept = 0;
        while (slept < COOLDOWN_TIME) {
          delay(COOLDOWN_BLINK_RATE);
          slept += COOLDOWN_BLINK_RATE;
          digitalWrite(LED_PIN, blink_state_);
          blink_state_ = !blink_state_;
        }
        digitalWrite(LED_PIN, LOW);
      }
    } else {
      digitalWrite(LED_PIN, true);
    }

    if (time_since_movement_ > 1000 && time_with_continuous_movement_ > 0) {
      Serial.print("MOVEMENT TIMEOUT!"); Serial.print("\t");
      time_with_continuous_movement_ = 0;
      Serial.println("");
    }
    //Serial.println("");
      
//        Serial.print(ax_); Serial.print("\t");
//        Serial.print(ay_); Serial.print("\t");
//        Serial.print(az_); Serial.print("\t");
//        Serial.print(gx_); Serial.print("\t");
//        Serial.print(gy_); Serial.print("\t");
//        Serial.print(gz_); Serial.print("\t");
  if (SLEEP_MODE_ENABLED && ((time_since_movement_ > STILL_TIME_BEFORE_SLEEP) ||
    (!has_had_movement_ && time_since_startup_ > STILL_TIME_AFTER_SLEEP_BEFORE_SLEEP))) {
    Serial.println("ESP8266 in sleep mode");
    ESP.deepSleep(SLEEP_SECS * 1000000);    
  }
  
}
