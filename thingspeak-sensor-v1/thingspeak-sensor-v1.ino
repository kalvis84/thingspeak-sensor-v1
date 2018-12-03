/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/

/**********************************DS18B20******************/
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 25  //d2
#define DALLAS_VCC_PIN 26 //d3

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
// DS18B20 water proof = { 0x28, 0xFF, 0x97, 0x59, 0x74, 0x15, 0x3, 0x65 }
//DeviceAddress insideThermometer = { 0x28, 0xFF, 0x97, 0x59, 0x74, 0x15, 0x3, 0x65 };
DeviceAddress insideThermometer;

float celsius = 0;
int celsiusOutOfRangeCount = 0;
/**********************************END DS18B20******************/

#include "WiFi.h"
#include "ThingSpeak.h"

/*
  *****************************************************************************************
  **** Visit https://www.thingspeak.com to sign up for a free account and create
  **** a channel.  The video tutorial http://community.thingspeak.com/tutorials/thingspeak-channels/ 
  **** has more information. You need to change this to your channel, and your write API key
  **** IF YOU SHARE YOUR CODE WITH OTHERS, MAKE SURE YOU REMOVE YOUR WRITE API KEY!!
  *****************************************************************************************/
unsigned long myChannelNumber = 495946;
const char * myWriteAPIKey = "VITQ0SUG7LGNEK6R";
WiFiClient  client;

float adcToVoltage = 6.87;
float batteryVoltage = 0;
uint8_t sendData_flag = 0;

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR uint8_t bootCount = 0;
RTC_DATA_ATTR uint8_t wifiConnectionFail = 0;
RTC_DATA_ATTR uint8_t bootsWithoutSendingData = 0;
RTC_DATA_ATTR float lastTemp = 0;



void goToDeepSleep(uint16_t seconds);

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void ds_setup(){
  // locate devices on the bus
//  Serial.print("Locating devices...");
  sensors.begin();
//  Serial.print("Found ");
  int deviceCount = sensors.getDeviceCount();
//  Serial.print(deviceCount, DEC);
//  Serial.println(" devices.");

  // report parasite power requirements
//  Serial.print("Parasite power is: "); 
//  if (sensors.isParasitePowerMode())    Serial.println("ON");
//  else    Serial.println("OFF");

//  if(insideThermometer == 0){
    if (!sensors.getAddress(insideThermometer, 0)){
      Serial.println("Unable to find address for Device 0");
    }
//  }  

  // show the addresses we found on the bus
//  Serial.print("Device 0 Address: ");
//  printAddress(insideThermometer);
//  Serial.println();

    // set the resolution to 10 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 10);
//  Serial.print("Device 0 Resolution: ");
  int resolution = sensors.getResolution(insideThermometer);
//  Serial.print(resolution, DEC);
//  Serial.println();  
}

/**************************DS18B20 print temp****************************/
void read_temperature(void){
  //  ToDo - take avg of 10.

  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
//  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
//  Serial.println("DONE");
  delay(50); //requestTemp... not waiting for required time? ToDo - adj. delay time.
  celsius = sensors.getTempC(insideThermometer);
  // check if value is correct. < -50mean there is no sensor or broken.
  if(celsius < -50 || celsius > 80) {
    celsius = 0;
    //if 10 broken values than reset.
    if(++celsiusOutOfRangeCount == 10){
//      blinkLEDBuildin(4, 300, 300);
//      HardwareResetESP();
    }
  }else celsiusOutOfRangeCount = 0;

  Serial.print("Temp C: ");
  Serial.println(celsius);
  
}

void smartConfigWiFi(void){
  //Init WiFi as Station, start SmartConfig
  WiFi.mode(WIFI_AP_STA);
  WiFi.beginSmartConfig();

  //Wait for SmartConfig packet from mobile
  Serial.println("Waiting for SmartConfig.");
  while (!WiFi.smartConfigDone()) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("SmartConfig received.");

  //Wait for WiFi to connect to AP
  Serial.println("Waiting for WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

void setupWiFi(void){
  int count = 0;
  int status = 0;
  
  Serial.print("Attempting WiFi connection");
//  status = WiFi.reconnect(); //or reconnect they work about the same
  status = WiFi.begin();
//  while (status != WL_CONNECTED) {
  while (status != WL_CONNECTED) {
    if (count++ >= 10) break;
    Serial.print(".");
    status = WiFi.status();
    delay (500);
  }
  if (status != WL_CONNECTED){
    if(wifiConnectionFail++ > 5){
      smartConfigWiFi ();
    }else{
      goToDeepSleep(60);
    }
  }
  wifiConnectionFail = 0;

  Serial.print("WiFi Connected.");

  Serial.print(" IP Address: ");
  Serial.println(WiFi.localIP());
}

void goToDeepSleep(uint16_t seconds)
{
  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(seconds * uS_TO_S_FACTOR);
//  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
//  " Seconds");

  /*
  Next we decide what all peripherals to shut down/keep on
  By default, ESP32 will automatically power down the peripherals
  not needed by the wakeup source, but if you want to be a poweruser
  this is for you. Read in detail at the API docs
  http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
  Left the line commented as an example of how to configure peripherals.
  The line below turns off all RTC peripherals in deep sleep.
  */
  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

  /*
  Now that we have setup a wake cause and if needed setup the
  peripherals state in deep sleep, we can now start going to
  deep sleep.
  In the case that no wake up sources were provided but deep
  sleep was started, it will sleep forever unless hardware
  reset occurs.
  */
  Serial.print("Going to sleep for: ");
  Serial.println(seconds);
  delay(10);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

// the setup function runs once when you press reset or power the board
void setup() {
  uint16_t sleepTime = 60;
  // Debug console
  Serial.begin(115200);
  delay(100);

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("");
  Serial.println("boot number: " + String(bootCount));
  Serial.println("wifiConnectionFail: " + String(wifiConnectionFail));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(DALLAS_VCC_PIN, OUTPUT);

  adcAttachPin(36);
  analogSetWidth(10);
  analogReadResolution(10);
  analogSetCycles(255);
//  analogSetSamples(255);
  
  digitalWrite(DALLAS_VCC_PIN, HIGH);  //turn on vcc for DS18B20 sensor.
  delay(10);
  ds_setup();
  read_temperature();
  digitalWrite(DALLAS_VCC_PIN, LOW);  //turn off vcc for DS18B20 sensor.

  batteryVoltage = (adcToVoltage * analogRead(A0)) / 1000;
  Serial.print("batteryVoltage: ");
  Serial.println(batteryVoltage);

  if(lastTemp == celsius){
    sendData_flag = 0;
    ++bootsWithoutSendingData;
  }
  else {
    sendData_flag = 1;
  }

  lastTemp = celsius;
  
  if(sendData_flag == 1 || bootsWithoutSendingData >=5){
    
    setupWiFi();
  
    ThingSpeak.begin(client);
    Serial.println("Ting Speak sensor ready."); 
    if(celsiusOutOfRangeCount)  ThingSpeak.setField(1,"");
    else ThingSpeak.setField(1,celsius);
    ThingSpeak.setField(2,batteryVoltage);
    ThingSpeak.setField(7,bootsWithoutSendingData);
    ThingSpeak.setField(8,bootCount);
  
    // Write the fields that you've set all at once.
    ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);  
  //  ThingSpeak.writeField(myChannelNumber, 1, celsius, myWriteAPIKey);  
    bootsWithoutSendingData = 0;
  }

   goToDeepSleep(sleepTime);
}

// the loop function runs over and over again forever
void loop() {

}
