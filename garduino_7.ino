#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>

//1) libreria para el reloj
#include <stdio.h>
#include <DS1302.h>

//2) libreria para el sensor de temp & humedad
#include "DHT.h"
#define An1 A1
//constructor para el DHT11 en D8
DHT dht(An1, DHT11);

//3) libreria para LCD, conectado por un chip i2c

#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

//------------------------ajustes de LCD--------------------------

#define I2C_ADDR 0x3F  // Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

//------------------------constructor de LCD--------------------------
LiquidCrystal_I2C lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);

//------------------------ajustes de LCD--------------------------

// 13) sensor de co2
#include "SparkFun_SCD30_Arduino_Library.h" 
SCD30 airSensor;

//1) reloj DS1302
namespace {

// Set the appropriate digital I/O pin connections. These are the pin
// assignments for the Arduino as well for as the DS1302 chip. See the DS1302
// datasheet:
//
//   http://datasheets.maximintegrated.com/en/ds/DS1302.pdf
  const int kCePin   = 2;  // Chip Enable
  const int kIoPin   = 3;  // Input/Output
  const int kSclkPin = 4;  // Serial Clock

  // Create a DS1302 object.
  DS1302 rtc(kCePin, kIoPin, kSclkPin);

  String dayAsString(const Time::Day day) {
    switch (day) {
      case Time::kSunday: return "Domingo";
      case Time::kMonday: return "Lunes";
      case Time::kTuesday: return "Martes";
      case Time::kWednesday: return "Miercoles";
      case Time::kThursday: return "Jueves";
      case Time::kFriday: return "Viernes";
      case Time::kSaturday: return "Sabado";
    }
    return "(unknown day)";
  }

  void printTime() {
    // Get the current time and date from the chip.
    Time t = rtc.time();

    // Name the day of the week.
    const String day = dayAsString(t.day);

    // Format the time and date and insert into the temporary buffer.
    char buf1[25];
    char buf2[25];
    snprintf(buf1, sizeof(buf1), "%s %04d-%02d-%02d",
           day.c_str(),
           t.yr, t.mon, t.date);

    snprintf(buf2, sizeof(buf2), "%02d:%02d:%02d", t.hr, t.min, t.sec);
    // Print the formatted string to serial so we can see the time.
    Serial.println("1) fecha");
    Serial.print(buf1);
    Serial.print(" ");
    Serial.println(buf2);

    lcd.clear();
    lcd.print("1) fecha: ");
    lcd.setCursor(0, 1);
    lcd.print(buf1);
    lcd.setCursor(12, 3);
    lcd.print(buf2);
    delay(1000);
  }

  void printHora(){
    Time t = rtc.time();
    char buf2[25];
    snprintf(buf2, sizeof(buf2), "%02d:%02d:%02d", t.hr, t.min, t.sec);
    // Print the formatted string to serial so we can see the time.
    
    lcd.setCursor(12, 3);
    lcd.print(buf2);
  }
}  // namespace

//2) dht11
void dht11(){

    lcd.clear();
    volatile int temperatura = dht.readTemperature();
    volatile int humedad = dht.readHumidity();
    volatile int calor = dht.computeHeatIndex(temperatura, humedad);
    Serial.print("2) temperatura: ");
    Serial.print(temperatura);
    Serial.println("*C");
    lcd.print("2) temperatura: ");
    lcd.print(temperatura);
    lcd.print("*C");

    lcd.setCursor(0, 1);

    //2)
    Serial.print(F("3) humedad: "));
    lcd.print("3) humedad: ");
    Serial.print(humedad);
    Serial.println("%");
    lcd.print(humedad);
    lcd.print("%");

    //3)
    lcd.setCursor(0, 2);
    Serial.print("4) calor: ");
    Serial.print(calor);
    Serial.println("%");
    
    lcd.print("4) calor: ");
    Serial.print("4)calor: ");
    Serial.println(calor);
    lcd.print(calor);
    lcd.print("%");

    printHora();
  
    delay(1000);
}

// 3) ---------------------- humedad tierra -------------------------
uint8_t temp_tierra(){
  PINB = B00000000; // AY0
  double temp_soil = analogRead(PORTD);
  return temp_soil;
}

// 4) ---------------------- temperatura tierra -------------------------
uint8_t hum_tierra(){
  PINB = B00000001; // AY1
  double hum_soil = analogRead(PORTD);
  return hum_soil;
}

void temp_tierra_display(){
     lcd.clear();
     //3) temp de tierra AY0
     Serial.print(F("5) temp de tierra: "));
     lcd.print("5) temp de tierra: ");
     lcd.setCursor(0, 1);
     Serial.print(hum_tierra() / 10.24);
     Serial.println("%");
     lcd.print(hum_tierra() / 10.24);
     lcd.print("*C");

     printHora();
     
     delay(1000);
}

void hum_tierra_display(){
  
     //4) humedad de tierra AY1
     lcd.clear();
     Serial.print(F("6) humedad de tierra: "));
     lcd.print("6)humedad de tierra:");
     Serial.print(temp_tierra() / 10.24);
     Serial.println("%");
     lcd.setCursor(0, 1);
     lcd.print(temp_tierra() / 10.24);
     lcd.print("%");
     
     printHora();
     
     delay(1000);
}

// 5) ---------------------- AY2 oxigeno -------------------------
void oxigeno(){

    lcd.clear();
    PINB = B00000010; //AY2
    double oxigeno = analogRead(PORTD) / 10.24;
  
    Serial.print(F("7) oxigeno: "));
    lcd.print("7) oxigeno : ");
    Serial.print(oxigeno);
    Serial.println(F(" PPM"));

    lcd.setCursor(0, 1);
    lcd.print(oxigeno);
    lcd.print(" PPM");

    printHora();
    
    delay(1000);
}

// 6)-------------- sensor luz AY3 ------------------
void lumenes(){

    lcd.clear();
    PINB = B00000011;  //AY3
    double lumen = analogRead(PINB) / 10.24;
  
    Serial.print(F("8) lumenes: "));
    lcd.print("8) lumenes: ");
    Serial.print(lumen);
    Serial.println(F(" lm"));

    lcd.setCursor(0, 1);
    lcd.print(lumen);
    lcd.print(" lm");

    printHora();
    
    delay(1000);
}

// 7)-------------- sensor ultravioleta AY4 ------------------
void ultravioleta(){

    lcd.clear();
    PINB = B00000100; //AY4
    double uv = analogRead(PINB) / 10.24;

    Serial.print(F("9) ultravioleta: "));
    Serial.print(uv);
    Serial.println("%");

    lcd.print("9) ultravioleta: ");
    lcd.setCursor(0, 1);
    lcd.print(uv);
    lcd.print("%");

    printHora();
    
    delay(1000);
}
// 8)-------------- sensor barometro AY5 y 6 ------------------
void barometro(){
  
    lcd.clear();
    PINB = B00000110;
    double barometroSCL = analogRead(PINB) / 10.24;
    Serial.print(F("10) presion atm: "));
    Serial.print(barometroSCL);
    Serial.println(" ATM");

    lcd.print("10) presion atm: ");
    lcd.setCursor(0, 1);
    lcd.print(barometroSCL);
    lcd.print(" ATM");

    printHora();

    delay(1000);
}

// 9)-------------- sensor fuego AY7 ------------------
void fuego(){
  
    lcd.clear();
    PINB = B00000111;
    double fuego = analogRead(PINB) / 10.24;
    Serial.print(F("11) presencia fuego: "));
    Serial.print(fuego);
    Serial.println(" %");

    lcd.print("11) presencia fuego: ");
    lcd.setCursor(0, 1);
    lcd.print(fuego);
    lcd.print("%");

    printHora();

    delay(1000);
}

// 10) -----------------sensor co2, humedad y temperatura--------------------

void co2(){
  
    lcd.clear();
    if (airSensor.dataAvailable()){
      lcd.clear();
      Serial.print("co2(ppm):");
      Serial.print(airSensor.getCO2());
      lcd.print("12) co2: ");

      Serial.print(" temp(C):");
      Serial.print(airSensor.getTemperature(), 1);
      lcd.setCursor(0, 1);
      lcd.print(" temp(C): ");
      lcd.print(airSensor.getTemperature(), 1);

      Serial.print(" humidity(%):");
      Serial.print(airSensor.getHumidity(), 1);

      Serial.println();
    }
    else{
      Serial.println("No data");
    }
  delay(1000);
}


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  rtc.writeProtect(false);
  rtc.halt(false);
  //Time t(2018, 6, 19, 10, 32, 20, Time::kTuesday);
  //rtc.time(t);
  Wire.begin();
  airSensor.begin();
  
  lcd.begin (20, 4);
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
  lcd.setBacklight(HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 0; i < 5; i++){
    printTime();
  }
  
  for(int i = 0; i < 5; i++){
    dht11();
  }
  
  for(int i = 0; i < 5; i++){
    temp_tierra_display();
  }

  for(int i = 0; i < 5; i++){
    hum_tierra_display();
  }
  
  for(int i = 0; i < 5; i++){
    oxigeno();
  }

  for(int i = 0; i < 5; i++){
    lumenes();
  }
  
  for(int i = 0; i < 5; i++){
    ultravioleta();
  }

  for(int i = 0; i < 5; i++){
    barometro();
  }

  for(int i = 0; i < 5; i++){
    fuego();
  }
  
  for(int i = 0; i < 5; i++){
    co2();
  }
  
}
/*
 *  I2C device found at address 0x3F  LCD
    I2C device found at address 0x61  CO2
    I2C device found at address 0x77  BAROMETRo
 */
