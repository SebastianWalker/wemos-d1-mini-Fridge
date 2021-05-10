#include <Arduino.h>

// used to get the reset reason
extern "C" {
#include <user_interface.h>
}
enum rst_reason {
 REASON_DEFAULT_RST = 0,      /* normal startup by power on */
 REASON_WDT_RST = 1,          /* hardware watch dog reset */
 REASON_EXCEPTION_RST = 2,    /* exception reset, GPIO status won’t change */
 REASON_SOFT_WDT_RST   = 3,   /* software watch dog reset, GPIO status won’t change */
 REASON_SOFT_RESTART = 4,     /* software restart ,system_restart , GPIO status won’t change */
 REASON_DEEP_SLEEP_AWAKE = 5, /* wake up from deep-sleep */
 REASON_EXT_SYS_RST      = 6  /* external system reset */
};

// Stuff for the WiFi manager 
#include "LittleFS.h"
#include "WiFiManager.h"
#include "webServer.h"
#include "updater.h"
#include "fetch.h"
#include "configManager.h"
#include "timeSync.h"
#include <TZ.h>
#include "ESP8266HTTPClient.h"

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
  Adafruit_Sensor *aht_humidity, *aht_temp;

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

// Stuff for Splunk
  String eventData="";

// variable for timing
  static unsigned long msTickSplunk = 0;

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
  String splunkurl="http://"+ String(configManager.data.splunkindexer) +"/services/collector"; //removed :8088 due to DOCKER container port redirect.. port now lives in the splunkindexer variable
  String tokenValue="Splunk " + String(configManager.data.collectorToken);
  
  // fire at will!! 
  http.begin(client, splunkurl);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", tokenValue);

  Serial.print("splunking: ");
  Serial.print(payload);

  String contentlength = String(payload.length());
  http.addHeader("Content-Length", contentlength );
  int httpResponseCode = http.POST(payload);
  http.writeToStream(&Serial);
  Serial.printf("HTTP: %d", httpResponseCode);  
  Serial.println();
  http.end();
}

void checkPir(){ 
  pirTripped = digitalRead(PIR);
  if (pirTripped) pirTrippedTime = millis();
}

// Software restart 
void(* resetFunc) (void) = 0; //declare reset function @ address 0
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

void setup()
{
  rst_info *resetInfo;
  resetInfo = ESP.getResetInfoPtr();
  int rst_cause = resetInfo->reason;
  switch (rst_cause){
    case REASON_DEFAULT_RST:
      break;

    case REASON_WDT_RST:
      break;

    case REASON_EXCEPTION_RST:
      break;

    case REASON_SOFT_WDT_RST:
      break;

    case REASON_SOFT_RESTART:
      break;

    case REASON_DEEP_SLEEP_AWAKE:
      break;

    case REASON_EXT_SYS_RST:
      break;

    default:
      break;     
  }


  Serial.begin(115200);

  LittleFS.begin();
  GUI.begin();
  configManager.begin();

  // add the MAC to the Project name to avoid duplicate SSIDs
  char AP_Name[60]; // 60 should be more than enough...
  strcpy(AP_Name, configManager.data.projectName);
  strcat(AP_Name, " - ");
  strcat(AP_Name, WiFi.macAddress().c_str());
  WiFiManager.begin(AP_Name); //192.168.4.1

  // if no client name is set (default=MAC_ADRESS) return the MAC instead
  if (String(configManager.data.clientName) == "MAC_ADDRESS"){
    char mac[20]; // clientName is size 20...
    WiFi.macAddress().toCharArray(mac, 20);

    // set the mac char by char... no idea how to do it in one go
    for(int i=0; i<=20; i++){
      configManager.data.clientName[i] = mac[i];
    }

    configManager.save();
  }

  //Set the timezone
  timeSync.begin(configManager.data.sensorTimezone);

  //Wait for connection
  timeSync.waitForSyncResult(5000);

  if (!timeSync.isSynced()){
    splunkpost("\"status\" : \"ERROR\", \"msg\" : \"No time sync available.\"");
  }
  else{
    //time_t now = time(nullptr);
    //Serial.print(PSTR("Local time: ")); Serial.println(asctime(localtime(&now)));
    //Serial.print(PSTR("UTC: ")); Serial.println(asctime(gmtime(&now)));
  }

  // set input/output pins
  pinMode(Heartbeat_LED, OUTPUT); // heartbeat LED
  digitalWrite(Heartbeat_LED, HIGH);
  pinMode(Splunking_LED, OUTPUT); // sending to splunk LED
  digitalWrite(Splunking_LED, LOW);
 
  pinMode(LDR, INPUT);
  pinMode(PIR, INPUT);


  // BME280
  if (!bme.begin(0x76)){
    splunkpost("\"status\" : \"ERROR\", \"msg\" : \"BME280 not found.\""); 
  }
  else{
    bmeErr = false; 
  }
  // BME280 END

  // MAX44009 lux sensor
  if (!luxSensor.isConnected()){
    splunkpost("\"status\" : \"ERROR\", \"msg\" : \"MAX44009 not found.\""); 
  }
  else{
    luxSensor.setAutomaticMode();
    luxErr = false;
  }
  // MAX44009 lux sensor END

  // AHT10
  if (!aht.begin()){
    splunkpost("\"status\" : \"ERROR\", \"msg\" : \"AHT10 not found.\""); 
  }
  else{
    ahtErr = false; 
    aht_temp = aht.getTemperatureSensor();
    aht_humidity = aht.getHumiditySensor();
  }
  // AHT10 END
  String msg = "\"status\" : \"INFO\", \"msg\" : \"restarted " + String(rst_cause) + "\"";
  splunkpost(msg); 
  //splunkpost("\"status\" : \"INFO\", \"msg\" : \"restarted\""); 
}

void loop()
{
  //software interrupts.. dont touch next line!
  WiFiManager.loop();updater.loop();configManager.loop(); 

  if (WiFiManager.isCaptivePortal()){
    digitalWrite(Heartbeat_LED, (millis() / 100) % 2);
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

  if (millis() - msTickSplunk > configManager.data.updateSpeed){
    msTickSplunk = millis();

    digitalWrite(Splunking_LED, HIGH);

  // Read the light intensity
    LDRvalue = map(analogRead(LDR), 0,1024,0,100);
  
  // PIR
  // if last trigger time is inside the last update intervall then log motion as true, else the checkPIR function has reset the pirTripped to 0 anyways
    if (pirTrippedTime >= msTickSplunk-configManager.data.updateSpeed){pirTripped = 1;}

  // BME280
    sensors_event_t temp_event, pressure_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);
  // BME280 END


  // AHT10
    sensors_event_t aht10_humidity, aht10_temp;
    aht_humidity->getEvent(&aht10_humidity);
    aht_temp->getEvent(&aht10_temp);
  // AHT10 END


  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
    float t = dht.readTemperature();
  

  // only report these sensors if they are up and running... 
    String PIR_data = (configManager.data.pirInstalled) ? "\"PIR_State\": \"" + String(pirTripped) + "\" , " : "";

  // DHT11 reporting NaN ?
    String DHT11_data = (isnan(h) || isnan(t)) ? "" : "\"DHT11_Temp\": \"" + String(t) + "\" , "
                                                      "\"DHT11_Humidity\": \"" + String(h) + "\" , ";

  // BME280 found at setup on I2C ?
    String BME280_data = bmeErr ? "" :  "\"BME280_Temp\": \"" + String(temp_event.temperature) + "\" , "
                                        "\"BME280_Pressure\": \"" + String(pressure_event.pressure) + "\" , "
                                        "\"BME280_Humidity\": \"" + String(humidity_event.relative_humidity) + "\" , ";

  // MAX44009 found at setup on I2C ?
    String MAX44009_data = luxErr ? "" :  "\"MAX44009_lux\": \"" + String(luxSensor.getLux()) + "\" , "
                                          "\"MAX44009_IntegrationTime\": \"" + String(luxSensor.getIntegrationTime()/1000.0) + "\" , ";
 
  // AHT10 sometimes reports ZERO humidity.. thats bullshit.. so dont report anything
    String AHT10_data = (ahtErr || aht10_humidity.relative_humidity == 0) ? "" :  "\"AHT10_Temp\": \"" + String(aht10_temp.temperature) + "\" , "
                                       "\"AHT10_Humidity\": \"" + String(aht10_humidity.relative_humidity) + "\" , ";
  
  // build the event data string
    eventData =   "\"lightIndex\": \"" + String(LDRvalue) + "\" , "
                  + PIR_data
                  + DHT11_data 
                  + BME280_data
                  + AHT10_data  
                  + MAX44009_data + 
                  //"\"HCSR04_Distance\": \"" + String(HCSR04.read()) + "\" , "
                  "\"uptime\": \"" + String(millis()/1000) + "\" ";

  //send off the data
    splunkpost(eventData);

    digitalWrite(Splunking_LED, LOW);
  }
}