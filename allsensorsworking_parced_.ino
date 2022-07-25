#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MCP9808.h>
#include "DFRobot_MICS.h"

/////////////////////
// loudness definitions 
#define COMMAND_GET_VALUE        0x05
#define COMMAND_NOTHING_NEW   0x99
const byte qwiicAddress = 0x38;     //Default Address
uint16_t ADC_VALUE=0;
////////////////////////

Adafruit_MCP9808  sensor;
// I2C
Adafruit_LIS3DH lis1 = Adafruit_LIS3DH();
Adafruit_LIS3DH lis2 = Adafruit_LIS3DH();
unsigned long t;

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif
#define CALIBRATION_TIME   0

#define TCAADDR 0x70
#define addr MICS_ADDRESS_0
DFRobot_MICS_I2C mics(&Wire, addr);
     
void tcaselect(uint8_t i) {
  if (i > 8) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission(); 
}

void setup(void) {
#ifndef ESP8266
  while (! Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Wire.begin(); 
  Serial.begin(115200);

    ///COOOL LOUD SENS TEST
  
  tcaselect(7);
  testForConnectivity();
  
  /////cooll aceeel test biiitch
    tcaselect(6);
    if (!lis1.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start 1");
    while (1);
  }

 
  lis1.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
  

  /// accel 2 test

    tcaselect(7);
    if (! lis2.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }

 
  lis2.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
 


  
  ////coool temp sensor cuuuuuhhhh
  tcaselect(4);
  if (!sensor.begin()) {
    Serial.println("Failed to find Adafruit MCP9808 chip");
    while (1) { delay(10); }
  }
  

  tcaselect(5);
  airQualtest();
 Serial.print("Time(s)/ ");
 Serial.print("x1/ ");
 Serial.print("y1/ ");
 Serial.print("z1/ ");
 Serial.print("x2/ ");
 Serial.print("y2/ ");
 Serial.print("z2/ ");
 Serial.print("temp(c)/ ");
 Serial.print("loadness(db)/ ");
 Serial.print("CO/ ");
 Serial.print("CH4/ ");
 Serial.print("C2H5OH/ ");
 Serial.print("H2/ ");
 Serial.print("NH3/ ");
 Serial.print("NO2/ ");
 Serial.println();
}

void loop() 
{

//current time
float t = ((float) millis())/1000.0;

Serial.print(t);

Serial.print(" ");
  
accel();

accel2();

tempsens();

loudsens();

airqual();

}
void testForConnectivity() {
   Wire.beginTransmission(qwiicAddress); // transmit to device
  //check here for an ACK from the slave
  if (Wire.endTransmission() != 0 ) {
    Serial.println("Check Connections. Slave not found.");
   
  }
  
}
void loudsens()
{
  tcaselect(6);
  Wire.beginTransmission(qwiicAddress);
  Wire.write(COMMAND_GET_VALUE); // command for status
  Wire.endTransmission();    // stop transmitting //this looks like it was essential.

  Wire.requestFrom(qwiicAddress, 2);    // request 1 bytes from slave device qwiicAddress
  
  if (Wire.available()) { // slave may send less than requested
    uint8_t ADC_VALUE_L = Wire.read(); 
    uint8_t ADC_VALUE_H = Wire.read();
    ADC_VALUE=ADC_VALUE_H;
    ADC_VALUE<<=8;
    ADC_VALUE|=ADC_VALUE_L;
    double dB = (ADC_VALUE + 83.2073) / 11.003;
    Serial.print(dB,DEC);
    Serial.print(" ");
  }
  else{
    Serial.print("N/A");
    Serial.print(" ");
  }
  uint16_t x=Wire.read(); 
}
void accel()
{
  tcaselect(6);
  lis1.read();      // get X Y and Z data at once
  sensors_event_t event;
  lis1.getEvent(&event);
  t = millis();
  Serial.print(event.acceleration.x);
  Serial.print(" ");
  Serial.print(event.acceleration.y);
  Serial.print(" ");
  Serial.print(event.acceleration.z);
  Serial.print(" ");
  //Serial.print(" m/s^2 ");
}
void accel2()
{
  tcaselect(7);
  lis2.read();      // get X Y and Z data at once
  sensors_event_t event2;
  lis2.getEvent(&event2);
  t = millis();
  Serial.print(event2.acceleration.x);
  Serial.print(" ");
  Serial.print(event2.acceleration.y);
  Serial.print(" ");
  Serial.print(event2.acceleration.z);
  Serial.print(" ");
  //Serial.print(" m/s^2 ");
 
}
void tempsens()
{
  tcaselect(4);
  sensors_event_t event;
  sensor.getEvent(&event);
  double c = event.temperature;
  Serial.print(c);
  Serial.print(" ");
}
void airQualtest()
{
  while(!Serial);
  while(!mics.begin()){
    Serial.println("NO Deivces !");
    delay(1000);
  } 

  /**
   * Gets the power mode of the sensor
   * The sensor is in sleep mode when power is on,so it needs to wake up the sensor. 
   * The data obtained in sleep mode is wrong
   */
  uint8_t mode = mics.getPowerState();
  if(mode == SLEEP_MODE){
    mics.wakeUpMode();

  }
int i = 180;
  /**
   * Do not touch the sensor probe when preheating the sensor.
   * Place the sensor in clean air.
   * The default calibration time is 3 minutes.
   */
  while(!mics.warmUpTime(CALIBRATION_TIME)){
    if(i==180)
      Serial.println("Please wait until the warm-up time is over!");
     Serial.println(i);
     delay(1000);
    i--;
  }
}
void airqual()
{
  tcaselect(5);
  float gasdata = mics.getGasData(CO);
  float gasdata2 = mics.getGasData(CH4);
  float gasdata3 = mics.getGasData(C2H5OH);
  float gasdata4 = mics.getGasData(H2);
  float gasdata5 = mics.getGasData(NH3);
  float gasdata6 = mics.getGasData(NO2);


  
  Serial.print(gasdata,1);
  Serial.print(" ");
  Serial.print(gasdata2,1);
  Serial.print(" ");
  Serial.print(gasdata3,1);
  Serial.print(" ");
  Serial.print(gasdata4,1);
  Serial.print(" ");
  Serial.print(gasdata5,1);
  Serial.print(" ");
  Serial.print(gasdata6,1);
  Serial.println("/");
  
  //mics.sleepMode();
}
