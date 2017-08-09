#include <ESP8266WiFi.h>

#include "I2Cdev.h"
#include "MPU6050.h"


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t prev_gx, prev_gy, prev_gz;
bool have_prev = false;


const char WiFiSSID[] = "geheb";
const char WiFiPSK[] = "kaffe2go";

const unsigned long postRate = 60000;
unsigned long lastPost = 0;

int sample_rate = 100;
int movement_threshold = 5000;
int time_since_startup = 0;
int time_since_movement = 0;
bool has_had_movement = false;

const int still_time_before_sleep = 10000;
const int still_time_after_sleep_before_sleep = 1000;
const int sleep_secs = 1;
const bool sleep_mode_enabled = false;

#define LED_PIN 5
bool blinkState = false;

void setup() 
{
  initHardware(); // Setup input/output I/O pins
  //connectWiFi(); // Connect to WiFi
  initMTU();
  digitalWrite(LED_PIN, LOW); // LED on to indicate connect success
}

void initHardware() {
    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)

    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    pinMode(LED_PIN, OUTPUT); // Set LED as output
    digitalWrite(LED_PIN, HIGH); // LED off

    Serial.println("Hello World!");
}

void connectWiFi()
{
  byte ledStatus = LOW;
  Serial.println();
  Serial.println("Connecting to: " + String(WiFiSSID));
  // Set WiFi mode to station (as opposed to AP or AP_STA)
  WiFi.mode(WIFI_STA);

  // WiFI.begin([ssid], [passkey]) initiates a WiFI connection
  // to the stated [ssid], using the [passkey] as a WPA, WPA2,
  // or WEP passphrase.
  WiFi.begin(WiFiSSID, WiFiPSK);

  // Use the WiFi.status() function to check if the ESP8266
  // is connected to a WiFi network.
  while (WiFi.status() != WL_CONNECTED)
  {
    // Blink the LED
    digitalWrite(LED_PIN, ledStatus); // Write LED high/low
    ledStatus = (ledStatus == HIGH) ? LOW : HIGH;

    // Delays allow the ESP8266 to perform critical tasks
    // defined outside of the sketch. These tasks include
    // setting up, and maintaining, a WiFi connection.
    delay(100);
    // Potentially infinite loops are generally dangerous.
    // Add delays -- allowing the processor to perform other
    // tasks -- wherever possible.
  }
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
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
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    while( !accelgyro.testConnection() ) {
      Serial.println("MPU6050 connection failed.");
      delay(100);
    }
    Serial.println("MPU6050 connection successful");
}


void loop() {
  //Serial.println("Loop!");
  delay(sample_rate);
  time_since_startup += sample_rate;
  time_since_movement += sample_rate;
  
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (!have_prev) {
      prev_gx = gx;
      prev_gy = gy;
      prev_gz = gz;
      have_prev = true;
      return;
    }
    
    int delta = abs((gx - prev_gx) + (gy - prev_gy) + (gz - prev_gz));

    // display tab-separated accel/gyro x/y/z values
    //Serial.print("a/g:\t");
    Serial.print("Delta: " );
    Serial.print(delta); Serial.print("\t");

    if (delta > movement_threshold) {
      Serial.print(" MOVEMENT! ");
      time_since_movement = 0;
      has_had_movement = true;
    }

    Serial.println("");
      
//        Serial.print(ax); Serial.print("\t");
//        Serial.print(ay); Serial.print("\t");
//        Serial.print(az); Serial.print("\t");
//        Serial.print(gx); Serial.print("\t");
//        Serial.print(gy); Serial.print("\t");
//        Serial.print(gz); Serial.print("\t");
  if (sleep_mode_enabled && ((time_since_movement > still_time_before_sleep) ||
    (!has_had_movement && time_since_startup > still_time_after_sleep_before_sleep))) {
    Serial.println("ESP8266 in sleep mode");
    ESP.deepSleep(sleep_secs * 1000000);    
  }
  
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
