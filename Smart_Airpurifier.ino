/*
  PM2.5 Demo
  pm25-demo.ino
  Demonstrates operation of PM2.5 Particulate Matter Sensor
  ESP32 Serial Port (RX = 16, TX = 17)
  Derived from howtoelectronics.com - https://how2electronics.com/interfacing-pms5003-air-quality-sensor-arduino/
 
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
// Serial Port connections for PM2.5 Sensor
#define RXD2 2 // To sensor TXD
#define TXD2 0 // To sensor RXD

#define SDA 13 //Define SDA pins
#define SCL 14 //Define SCL pins

#define FAN 23
#define OZO 22

LiquidCrystal_I2C lcd(0x27,16,2);
 
void setup() {

pinMode(FAN, OUTPUT);
pinMode(OZO, OUTPUT);
 Wire.begin(SDA, SCL); // attach the IIC pin
 lcd.init(); // LCD driver initialization
 lcd.backlight(); // Turn on the backlight
 lcd.setCursor(0,0); // Move the cursor to row 0, column 0
 lcd.print("PM2.5 Reading: "); // The print content is displayed on the LCD
  // our debugging output
  Serial.begin(115200);
 
  // Set up UART connection
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
}
 
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
 
struct pms5003data data;
 
void loop() {
  if (readPMSdata(&Serial1)) {
    // reading data was successful!
    Serial.println();
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (standard)");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
    Serial.println("---------------------------------------");
    lcd.setCursor(0,1); // Move the cursor to row 1, column 0
    lcd.print(data.pm25_env); // The count is displayed every second

    if(data.pm25_env <= 8){
      lcd.setCursor(0,1); // Move the cursor to row 1, column 0
      lcd.print("Valid"); // The count is displayed every second
      digitalWrite(FAN, LOW);
      digitalWrite(OZO, LOW);
    }
    if(data.pm25_env > 8 && data.pm25_env <= 15){
      lcd.setCursor(0,1); // Move the cursor to row 1, column 0
      lcd.print("Moderate"); // The count is displayed every second
      digitalWrite(FAN, HIGH);
      digitalWrite(OZO, HIGH);
    }
    if(data.pm25_env > 15 && data.pm25_env <= 30){
      lcd.setCursor(0,1); // Move the cursor to row 1, column 0
      lcd.print("Bad"); // The count is displayed every second
      digitalWrite(FAN, HIGH);
      digitalWrite(OZO, HIGH);

    } 
    //lcd.print(millis() / 1000);
  
  }

  
  delay(1000);
}
 
boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
 
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
 
  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }
 
  /* debugging
    for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
    }
    Serial.println();
  */
 
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);
 
  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}