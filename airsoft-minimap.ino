// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

#include <SPI.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>

#include <RH_RF69.h>

#include <TinyGPS++.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#include "MiniMap.h"

// Screen dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128 // Change this to 96 for 1.27" OLED.

// You can use any (4 or) 5 pins 
#define SCLK_PIN 24
#define MOSI_PIN 23
#define CS_PIN   17
#define DC_PIN   18
#define RST_PIN  19

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF
#define GRAY            0x8410

/************ Dispay Setup ***************/

Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN, RST_PIN);

/************ GPS Setup ***************/

#define GPSSerial Serial1

static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

/************ Mag Setup ***************/

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0


#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13


// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

/************ End Setup ***************/

uint8_t id = 1;

float heading = 0.0;
float declinationAngle = 0.1827;

Node other;

uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);

void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  /************ Dispay Init ***************/
  
  tft.begin();
  tft.fillScreen(BLACK);

  /************ GPS Init ***************/

  GPSSerial.begin(GPSBaud);

  /************ Mag Init ***************/

  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
  }
  
  /************ Radio Init ***************/

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED,LOW);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  /************ End Init ***************/

  tft.fillScreen(BLACK);

  uint32_t loop_start = millis();
  while(millis() < loop_start + 1000) {
    if (rf69.available())  { 
      // Should be a reply message for us now   
      if (rf69.recv(buf, &len)) {
        Msg msg = parseMsg(buf, 12);
        updateNode(other, msg);
        delay(500 - msg.ref_time);
        break;
      } else {
        Serial.println("Receive failed");
      }
    }
  }
}

void loop() {
  uint32_t loop_start = millis();
  int w = tft.width(), h = tft.height();

  Serial.print("Start: "); Serial.println(loop_start);

  readGPS();
  readHeading();

  tft.setCursor(0,0);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);

  digitalWrite(LED,HIGH);
  for(int i=0; i<4; i++){
    prepareMsg(buf, 1, id, millis()- loop_start, heading, gps.location.lat(), gps.location.lng());
    rf69.send(buf, 12);
    rf69.waitPacketSent();
    delay(loop_start + (i+1)*25 - millis());
  }
  digitalWrite(LED,LOW);

  Serial.print("Sent: "); Serial.println(millis());

  for(int i=0; i<4; i++){
    tft.fillRect(0,i*h/4,w,h/4,BLACK);
    if(i == 0){
      tft.print("Lat: "); tft.println(gps.location.lat(), 2);
      tft.print("Lon: "); tft.println(gps.location.lng(), 2); 
      tft.print("Hdg: "); tft.println((uint16_t)heading); 
    }
    // if(i == 1){
    //   tft.setCursor(0,32);
    //   tft.setTextColor(YELLOW);
    //   tft.print("Lat: "); tft.println(other.lat, 4);
    //   tft.print("Lon: "); tft.println(other.lng, 4); 
    //   tft.print("Heading: "); tft.println(other.hdg); 
    // }
  }

  drawCross(63, 63, 8, WHITE);
  tft.drawCircle(64, 64, 51, GRAY);
  tft.drawCircle(64, 64, 26, GRAY);

  if(gps.location.isValid()){
    int x, y, d;
    getRelativePos(500, 43.764052, -79.407043, x, y, d);

    drawCross(63+x, 63+y, 8, GREEN);
    tft.setCursor(63+x+4, 63+y+4);
    tft.setTextColor(GREEN);
    tft.print(d);
  } 

  if(gps.location.isValid() && loop_start - other.last_recv < 5000 && abs(other.lat) > 0.1 && abs(other.lng) > 0.1){
    int x, y, d;
    getRelativePos(500, other.lat, other.lng, x, y, d);

    drawCross(63+x, 63+y, 8, YELLOW);
    tft.setCursor(63+x+4, 63+y+4);
    tft.setTextColor(YELLOW);
    tft.print(d);
  }

  delay(loop_start + 500 - millis());
  Serial.print("Drawn: "); Serial.println(millis());

  while(millis() < loop_start + 1000) {
    if (rf69.available())  { 
      // Should be a reply message for us now   
      if (rf69.recv(buf, &len)) {
        Msg msg = parseMsg(buf, 12);
        updateNode(other, msg);
        loop_start = msg.recv_at - msg.ref_time - 500;
        Serial.print("Recv Lat: "); Serial.print(other.lat, 4);
        Serial.print(", Lon: "); Serial.println(other.lng, 4); 
      } else {
        Serial.println("Receive failed");
      }
    }
  }
}

void drawCross(int x, int y, int s, uint16_t color){
  tft.drawFastHLine(x-s+1, y, s*2, color);
  tft.drawFastHLine(x-s+1, y+1, s*2, color);
  tft.drawFastVLine(x, y-s+1, s*2, color);
  tft.drawFastVLine(x+1, y-s+1, s*2, color);
}

void getRelativePos(int max_d, float lat, float lng, int& x, int& y, int& d){
  float dis = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), lat, lng);
  float crs = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), lat, lng);

  x = (int)(sin(radians(crs)) * dis * 64 / max_d);
  y = -(int)(cos(radians(crs)) * dis * 64 / max_d);
  d = (int)dis;
}

void readGPS() {
  int l = GPSSerial.available();
  for(int i=0; i< l; i++){
    gps.encode(GPSSerial.read());
  }
  Serial.print("GPS Read bytes: "); Serial.println(l);
  Serial.print("Sat: "); Serial.print(gps.satellites.value());
  Serial.print(", HDop: "); Serial.print(gps.hdop.hdop(), 4);
  Serial.print(", Lat: "); Serial.print(gps.location.lat(), 4);
  Serial.print(", Lon: "); Serial.println(gps.location.lng(), 4); 
}

void readHeading() {
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  // Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  // Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  // Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  heading = heading * 180/M_PI; 
  
  Serial.print("Heading: "); Serial.println(heading);
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
