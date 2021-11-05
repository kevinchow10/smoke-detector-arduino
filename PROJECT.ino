#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"

// Microcontroller Project Smoke Detector - Group 6 & 7
//By: Kevin, John Winston, Athira Jasmine, Kaizen Lolo O

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  1000 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

// connection of the digital pins, digital pins 10-13 has been used for the shield
#define redLEDpin 2
#define greenLEDpin 3

int pirSensor = 4;           // the pin that the sensor is atteched to (PIR Sensor)
int ledPIR = 13;             // the pin that the LED is atteched to (PIR Sensor)
int state = LOW;             // by default, no motion detected (PIR Sensor)
int val = 0;                 // variable to store the sensor status (value) (PIR Sensor)
int gas135Value;             // declaration of gas sensor MQ135

// connection of the analog pins
//analog pin 4 and 5 has been used for i2c communication
#define BANDGAPREF 14            // special indicator that we want to measure the bandgap
#define aref_voltage 3.3         // we tie 3.3V to ARef and measure it with a multimeter!
#define bandgap_voltage 1.1      // this is not super guaranteed but its not -too- off

RTC_DS1307 RTC; // define the Real Time Clock object

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;



void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while(1);
}



void setup(void)
{
  Serial.begin(9600);
  Serial.println();
  
  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  pinMode(ledPIR, OUTPUT);      // initalize LED as an output (PIR Sensor)
  pinMode(pirSensor, INPUT);    // initialize sensor as an input (PIR Sensor)
  
  #if WAIT_TO_START
    Serial.println("Type any character to start");
    while (!Serial.available());
  #endif //WAIT_TO_START

  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);

  // connect to RTC
  Wire.begin();
  RTC.begin();  
  
  if (!RTC.isrunning()) {
    logfile.println("RTC failed");
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  #if ECHO_TO_SERIAL
    Serial.println("RTC failed");
  #endif  //ECHO_TO_SERIAL
  }

  logfile.println("millis,stamp,datetime,ppm,object detected,vcc");    
  
  #if ECHO_TO_SERIAL
    Serial.println("millis,stamp,datetime,ppm,object detected,vcc");
  #endif //ECHO_TO_SERIAL

  /* 
    // If you want to set the aref to something other than 5v
    analogReference(EXTERNAL);
  */ 
}



void loop(void)
{
  DateTime now = RTC.now();

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  
  digitalWrite(greenLEDpin, HIGH);
  
  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");    

  #if ECHO_TO_SERIAL
    Serial.print(m);         // milliseconds since start
    Serial.print(", ");  
  #endif // ECHO_TO_SERIAL

  // log time
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');

  #if ECHO_TO_SERIAL
    Serial.print(now.unixtime()); // seconds since 1/1/1970
    Serial.print(", ");
    Serial.print('"');
    Serial.print(now.year(), DEC);
    Serial.print("/");
    Serial.print(now.month(), DEC);
    Serial.print("/");
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(":");
    Serial.print(now.minute(), DEC);
    Serial.print(":");
    Serial.print(now.second(), DEC);
    Serial.print('"');
  #endif //ECHO_TO_SERIAL


  /* SENSOR START READING */

  //GAS MQ135
  gas135Value = analogRead(0);       // read analog input pin 0
  
  //PIR
  val = digitalRead(pirSensor);   // read sensor value
  
  if (val == HIGH) {           // check if the sensor is HIGH
    digitalWrite(ledPIR, HIGH);   // turn LED ON
    delay(100);                // delay 100 milliseconds 
    
    if (state == LOW) { 
      state = HIGH;       // update variable state to HIGH
      }
  } 

  else {
    digitalWrite(ledPIR, LOW); // turn LED OFF
    delay(200);             // delay 200 milliseconds 
      
    if (state == HIGH){
      state = LOW;       // update variable state to LOW
      }
  }


  /* SENSOR SHOWS THE OUTPUT AND STORE TO SD CARD */
  
  #if ECHO_TO_SERIAL
    Serial.print(", ");
    Serial.print(gas135Value, DEC);              
    Serial.print(" PPM");
    //delay(100); 
    logfile.print(",");    
    logfile.print(gas135Value, DEC);  
  
    if (state == HIGH) { 
        Serial.print(", Motion detected!");
        }
    
    else{ 
        Serial.print(", Motion stopped!");
        }
    
    logfile.print(", ");    
    logfile.print(state);
  
  #endif //ECHO_TO_SERIAL

  // Log the estimated 'VCC' voltage by measuring the internal 1.1v ref
  analogRead(BANDGAPREF); 
  delay(10);
  int refReading = analogRead(BANDGAPREF); 
  float supplyvoltage = (bandgap_voltage * 1024) / refReading; 
  
  logfile.print(", ");
  logfile.print(supplyvoltage);
  
  #if ECHO_TO_SERIAL
    Serial.print(", ");   
    Serial.print(supplyvoltage);
  #endif // ECHO_TO_SERIAL

  logfile.println();
  
  #if ECHO_TO_SERIAL
    Serial.println();
  #endif // ECHO_TO_SERIAL

  digitalWrite(greenLEDpin, LOW);

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  
  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);
  
}
