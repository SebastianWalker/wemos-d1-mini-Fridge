#include <Arduino.h>

// used to get the reset reason
extern "C" {
#include <user_interface.h>
}
int rst_cause;

// Stuff for the WiFi manager 
#include "LittleFS.h"
#include "WiFiManager.h"
#include "webServer.h"
#include "updater.h"
#include "fetch.h"
#include "configManager.h"
#include "dashboard.h"
#include "timeSync.h"
#include <TZ.h>
#include "ESP8266HTTPClient.h"

#include "ESP8266mDNS.h"


// I2C sensors
	#include <Wire.h>  

	// BME280 
	#include <Adafruit_BME280.h>
	boolean bmeErr = true; // set to false once wire.begin is successful
	Adafruit_BME280 bme; // use I2C interface
	Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
	Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
	Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

  // MAX44009
	#include <Max44009.h>
	Max44009 luxSensor(0x4A);
	boolean luxErr = true; // set to false once connected in setup()

  // AHT10: Temperatur, Humidity
  #include <Adafruit_AHTX0.h>
  boolean ahtErr = true; // set to false once wire.begin is successful 
  Adafruit_AHTX0 aht;
  Adafruit_Sensor *aht_humidity , *aht_temp; // AHT and BME seems not to be implemented in the same way.. getXYZSensor will be done in Setup()


// Stuff for the DHT sensor
  #include "DHT.h"
  #define DHTPIN D7 
  #define DHTTYPE DHT11  
  DHT dht(DHTPIN, DHTTYPE);

/*
// HC-SR04 ultra sonic distance sensor
  #include <Ultrasonic.h>
  #define HCSR04_ECHO D6
  #define HCSR04_TRIG D7
  Ultrasonic HCSR04(HCSR04_TRIG, HCSR04_ECHO);
*/

// LDR input pin (ESP8266 only got one ADC on A0)
  #define LDR A0
  int LDRvalue = 0;

// HC-SR501 PIR
  #define PIR D5  // input pin
  static unsigned long pirTrippedTime = 0 ; // last time in millis() the PIR got tripped by motion
  boolean pirTripped = false; // is motion detected? true/false

// LEDs
  #define Heartbeat_LED D4 // D4 = GPIO2 = onchip LED
  #define Splunking_LED D8 // output pin for measurement and splunking indicator

// variable for timing
  static unsigned long msTickSplunk = 0;

// error counter for http connection -> reset esp after to many errors
  static int httpError = 0;
  

// Software restart 
void(* resetFunc) (void) = 0; //declare reset function @ address 0


/*
 * Returns a string of Localtime
 * in format "%Y-%m-%d %H:%M:%S"
 */
String getLocaltime(){
  time_t now = time(nullptr);
  struct tm * timeinfo;
  char timeStringBuff[20];
    
  time (&now);
  timeinfo = localtime(&now);
    
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", timeinfo);
  return timeStringBuff;
}

/*
 * Returns a string of time in UTC
 * in format "%Y-%m-%d %H:%M:%S"
 */
String getUTC(){
  time_t now = time(nullptr);
  struct tm * timeinfo;
  char timeStringBuff[20];
    
  time (&now);
  timeinfo = gmtime(&now);
    
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", timeinfo);
  return timeStringBuff;
}

/*
 * Returns epoch time as long
 */
long getEpoch(){
  time_t now = time(nullptr);
  return now;
}


/*
 * Post event data to splunk http event collector
 * 
 * PostData: 
 * a string of json formatted key:value pairs 
 * just everything between the curly braces of the event node
 */
void splunkpost(String PostData){

  // only send time string to splunk if it is synced, else omit it and let splunk use index time...
  String timeField = timeSync.isSynced() ? "\"time\" : \"" + String(getEpoch()) + "\" , " : "" ;

  String payload  = "{ \"host\": \"" + String(configManager.data.clientName) + "\", " 
                      "\"sourcetype\": \"" + String(configManager.data.sourcetype) + "\", " 
                      "\"index\": \"" + String(configManager.data.index) + "\", " 
                      + timeField +
                      "\"fields\" : {"
                                      "\"IP\" : \"" + String(WiFi.localIP().toString()) + "\" , "
                                      "\"UTC\" : \"" + String(getUTC()) + "\" , "
                                      "\"Localtime\" : \"" + String(getLocaltime()) + "\" , "
                                      "\"interval\" : \"" + String(configManager.data.updateSpeed/1000) + "\" "
                        "}, "
                        "\"event\"  : {" + PostData + "}"
                    "}";

  //Build the request
  WiFiClient client; // just to avoid deprecation error on http.begin(url)
  HTTPClient http;
  String splunkurl="http://"+ String(configManager.data.splunkindexer) +"/services/collector";
  String tokenValue="Splunk " + String(configManager.data.collectorToken);
  
  // fire at will!! 
  http.begin(client, splunkurl);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", tokenValue);

  if (!configManager.data.silenceSerial){
    Serial.print("splunking: ");
    Serial.print(payload);
  }


  String contentlength = String(payload.length());
  http.addHeader("Content-Length", contentlength );
  int httpResponseCode = http.POST(payload);
  if (!configManager.data.silenceSerial){
    http.writeToStream(&Serial);
    Serial.printf("HTTP: %d", httpResponseCode);  
    Serial.println();
  }
  http.end();

  // check for http error
  if (httpResponseCode > 200){
    httpError++;
    if (httpError>10){resetFunc();}
  }

  // send data to dashboard
  dash.data.httpResponse = httpResponseCode;
  dash.send();
}

/*
 * Triggers a software restart
 * resets the flag in config manager
 * send INFO msg to splunk
 */
void forceRestart(){
  // save false to config data.. else it would keep restarting...
  configManager.data.forceRestart = false;
  configManager.save();
  splunkpost("\"status\" : \"INFO\", \"msg\" : \"Executing Order 66\""); 
  resetFunc();
}

void checkPir(){ 
  pirTripped = digitalRead(PIR);
  if (pirTripped) pirTrippedTime = millis();
}

/*
 * callback function for configManager save to EEPROM
 */
void saveCallback(){
  // do stuff on save

  // set new hostname for MDNS on save
  // didnt find a way to get the change within the loop.. so change it here anyways
  MDNS.setHostname(configManager.data.clientName);
}
void setup()
{
  // enable light sleep = wifi/cpu off during delay calls
  wifi_set_sleep_type(LIGHT_SLEEP_T);

  rst_info *resetInfo;
  resetInfo = ESP.getResetInfoPtr();
  String rst_string = ESP.getResetReason(); // good for logging to splunk as plain text
  rst_cause = resetInfo->reason;        // good to use in code 
  switch (rst_cause){
    case REASON_DEFAULT_RST:      /* normal startup by power on */
      break;

    case REASON_WDT_RST:          /* hardware watch dog reset */
      break;

    case REASON_EXCEPTION_RST:    /* exception reset, GPIO status won???t change */
      break;

    case REASON_SOFT_WDT_RST:     /* software watch dog reset, GPIO status won???t change */
      break;

    case REASON_SOFT_RESTART:     /* software restart ,system_restart , GPIO status won???t change */
      break;

    case REASON_DEEP_SLEEP_AWAKE: /* wake up from deep-sleep */
      break;

    case REASON_EXT_SYS_RST:      /* external system reset */
      break;

    default:
      break;     
  }

  Serial.begin(115200);
  Serial.println(rst_string);

  LittleFS.begin(); GUI.begin(); configManager.begin(); dash.begin(1000);
  configManager.setConfigSaveCallback(saveCallback); // executed everytime a new config is saved

  // get the mac address as int -> hex -> use it as part of wifi name
  uint8_t macAddr[6];
  WiFi.macAddress(macAddr);
  //Serial.println(macAddr[5], HEX);Serial.println(macAddr[4], HEX);Serial.println(macAddr[3], HEX);Serial.println(macAddr[2], HEX);Serial.println(macAddr[1], HEX);Serial.println(macAddr[0], HEX);
  String macHex = String(macAddr[3], HEX) + String(macAddr[4], HEX) + String(macAddr[5], HEX);
  macHex.toUpperCase();

  // add the MAC to the Project name to avoid duplicate SSIDs
  char AP_Name[60]; // 60 should be more than enough...
  strcpy(AP_Name, configManager.data.projectName);
  strcat(AP_Name, " _ ");
  strcat(AP_Name, WiFi.macAddress().c_str());
  WiFiManager.begin(AP_Name); //192.168.4.1
  //WiFiManager.begin("newNetwork"); //192.168.4.1

  // if no client name is set (default=MAC_ADRESS) return the MAC instead
  if (String(configManager.data.clientName) == "MAC_ADDRESS"){
    WiFi.macAddress().toCharArray(configManager.data.clientName, 20);
    configManager.save();
  }

  // just for testing.. needs some fixfix
  MDNS.begin(configManager.data.clientName);

  // set mac to dashboard
  WiFi.macAddress().toCharArray(dash.data.macAddress, 20);

  // set the timezone
  timeSync.begin(configManager.data.sensorTimezone);

  // wait for connection
  timeSync.waitForSyncResult(5000);

  // set input/output pins
  pinMode(Heartbeat_LED, OUTPUT); // heartbeat LED
  digitalWrite(Heartbeat_LED, HIGH);
  pinMode(Splunking_LED, OUTPUT); // sending to splunk LED
  digitalWrite(Splunking_LED, LOW);
 
  pinMode(LDR, INPUT);
  pinMode(PIR, INPUT);


  // BME280
  if (bme.begin(0x76)){
    bmeErr = false; 
  }
  // BME280 END

  // MAX44009 lux sensor
  if (luxSensor.isConnected()){
    luxSensor.setAutomaticMode();
    luxErr = false;
  }
  // MAX44009 lux sensor END

  // AHT10
  if (aht.begin()){
    ahtErr = false; 
    aht_temp = aht.getTemperatureSensor();
    aht_humidity = aht.getHumiditySensor();
  }
  // AHT10 END

  // only report sensor status on hardware restart.. dont report it after deep-sleep wake
  // after deep-sleep wake silently go into measurement and send data only
  if (rst_cause != REASON_DEEP_SLEEP_AWAKE){
    String msgSensorStatus =  "\"msg\":\"restarted\", "
                              "\"rstReason\":\""  + String(rst_cause) + " - "  + String(rst_string) + "\", "
                              "\"BME280\":\""     + String(!bmeErr) + "\", "
                              "\"AHT10\":\""      + String(!ahtErr) + "\", "
                              "\"MAX44009\":\""   + String(!luxErr) + "\", "
                              "\"TimeSync\":\""   + String(timeSync.isSynced()) + "\"";

    splunkpost(msgSensorStatus); 
  }
}

void loop()
{
  // software interrupts.. dont touch next line!
  WiFiManager.loop();updater.loop();configManager.loop();dash.loop();MDNS.update();
  yield();

  if (WiFiManager.isCaptivePortal() && rst_cause != REASON_DEEP_SLEEP_AWAKE){
    digitalWrite(Heartbeat_LED, (millis() / 100) % 2);
    
    // fixfix suicide after to much time spent in captive portal

    return;
  }
  else{
    // toggle LED every second if heartbeat is activated in config
    digitalWrite(Heartbeat_LED, configManager.data.heartbeat ? ((millis() / 1000) % 2) : 1);  
  }

  // restart over web interface...
  if (configManager.data.forceRestart){forceRestart();}

  // read PIR state
  if (configManager.data.pirInstalled){checkPir();}

  if (millis() - msTickSplunk > configManager.data.updateSpeed || (rst_cause == REASON_DEEP_SLEEP_AWAKE && configManager.data.deepsleep > 0)){
    msTickSplunk = millis();

    digitalWrite(Splunking_LED, HIGH);
  
  // PIR
  // if last trigger time is inside the last update intervall then log motion as true, else the checkPIR function has reset the pirTripped to 0 anyways
    if (pirTrippedTime >= msTickSplunk-configManager.data.updateSpeed){pirTripped = 1;}

  // BME280
    sensors_event_t temp_event, pressure_event, humidity_event; // sensor event need to be declared for downstream code
    if (!bmeErr){ // only accessible if sensor is present.. else exception 28 is triggered => restart ESP
      bme_temp->getEvent(&temp_event);
      bme_pressure->getEvent(&pressure_event);
      bme_humidity->getEvent(&humidity_event);
    }
  // BME280 END

  // AHT10
    sensors_event_t aht10_humidity, aht10_temp; // sensor event need to be declared for downstream code
    if (!ahtErr){ // only accessible if sensor is present.. else exception 28 is triggered => restart ESP
      aht_humidity->getEvent(&aht10_humidity); 
      aht_temp->getEvent(&aht10_temp);
    }
  // AHT10 END

  // Reading DHT10 temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    float t = dht.readTemperature(); // Read temperature as Celsius (the default)
    bool dhtErr = (isnan(h) || isnan(t)) ? true : false;

  // only report these sensors if they are up and running... 
    String PIR_data = (configManager.data.pirInstalled) ? "\"PIR_State\": \"" + String(pirTripped) + "\" , " : "";
    String LDR_data = (configManager.data.ldrInstalled) ? "\"lightIndex\": \"" + String(map(analogRead(LDR), 0,1024,0,100)) + "\" , " : "";
  // DHT11 reporting NaN ?
    String DHT11_data = dhtErr ? "" : "\"DHT11_Temp\": \"" + String(t) + "\" , "
                                      "\"DHT11_Humidity\": \"" + String(h) + "\" , ";

  // BME280 found at setup on I2C ?
    String BME280_data = bmeErr ? "" :  "\"BME280_Temp\": \"" + String(temp_event.temperature) + "\" , "
                                        "\"BME280_Pressure\": \"" + String(pressure_event.pressure) + "\" , "
                                        "\"BME280_Humidity\": \"" + String(humidity_event.relative_humidity) + "\" , ";

  // MAX44009 found at setup on I2C ?
    String MAX44009_data = luxErr ? "" :  "\"MAX44009_lux\": \"" + String(luxSensor.getLux()) + "\" , "
                                          "\"MAX44009_IntegrationTime\": \"" + String(luxSensor.getIntegrationTime()/1000.0) + "\" , ";
 
  // AHT10 sometimes reports ZERO humidity.. thats bullshit.. so dont report anything
  // does not contain the last comma.. this will be set by uptime depending on deepsleep
    String AHT10_data = (ahtErr || aht10_humidity.relative_humidity == 0) ? "" :  "\"AHT10_Temp\": \"" + String(aht10_temp.temperature) + "\" , "
                                                                                  "\"AHT10_Humidity\": \"" + String(aht10_humidity.relative_humidity) + "\" ";
  
  // report unified sensor data
  // only name it Temperature instead of AHT10_Temp or BME280_Temp
  // take the best sensor first if available
  String unifiedSensor_data = ""; // does not contain the last comma.. this will be set by uptime depending on deepsleep
  if (!bmeErr){
    unifiedSensor_data = "\"Temperature\":\"" + String(temp_event.temperature) + "\", "
                         "\"Pressure\":\""    + String(pressure_event.pressure) + "\", "
                         "\"Humidity\":\""    + String(humidity_event.relative_humidity) + "\" ";

  }else if(!ahtErr || aht10_humidity.relative_humidity != 0){
    unifiedSensor_data = "\"Temperature\":\"" + String(aht10_temp.temperature) + "\", "
                         "\"Humidity\":\""    + String(aht10_humidity.relative_humidity) + "\" ";

  }else if (!dhtErr){
    unifiedSensor_data = "\"Temperature\":\"" + String(t) + "\", "
                         "\"Humidity\":\""    + String(h) + "\" ";

  }

  String singleSensor_data = DHT11_data + BME280_data + AHT10_data; // AHT10 does not contain the last comma.. this will be set by uptime depending on deepsleep

  String report = configManager.data.unifiedSensorData ? unifiedSensor_data : singleSensor_data;

  String uptime = (rst_cause == REASON_DEEP_SLEEP_AWAKE) ? "" : ", \"uptime\": \"" + String(millis()/1000) + "\" ";

  // build the event data string
    String eventData =    LDR_data
                        + PIR_data
                        + MAX44009_data 
                        + report 
                        + uptime;
                        //"\"HCSR04_Distance\": \"" + String(HCSR04.read()) + "\" , "
                        //"\"uptime\": \"" + String(millis()/1000) + "\" ";

  //send off the data
    splunkpost(eventData);

    digitalWrite(Splunking_LED, LOW);

  }

  // sleep a little in light sleep during a delay call.. hopefully reducing heat from the esp
  // fixfix maybe set it to the update intervall.. and just sleep between two updates
  // fixfix bad idea. web gui gets inresponsive at long sleep times
  delay(configManager.data.delay);

  // stay awake for 60s after hard reset to flash or change config
  // deepsleep config of ZERO will disable deepsleep 
  if (configManager.data.deepsleep != 0){
    if (millis() > 60000 && rst_cause != REASON_DEEP_SLEEP_AWAKE){
      Serial.println("60s after restart.. going to deepsleep");
      ESP.deepSleep(configManager.data.deepsleep * 1000000);
    }
    // if woken up from deep sleep .. go back to deep sleep
    if (rst_cause == REASON_DEEP_SLEEP_AWAKE){
      Serial.println("woke up, did work, going to sleep again");
      ESP.deepSleep(configManager.data.deepsleep * 1000000);
    } // from seconds in webconfig to us in function call = "* 1000000"
  }
}