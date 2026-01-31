#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <PZEM004Tv30.h>

#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "esp_task_wdt.h"

// ======================================================
// WIFI + MQTT SETTINGS
// ======================================================
const char* ssid        = "Airtel_Thilagam Farm";
const char* password    = "Ganesh@1980";
const char* mqtt_server = "192.168.1.7";

EthernetClient ethClient;
PubSubClient client(ethClient);

// ======================================================
// PIN MAP
// ======================================================
#define RELAY_START   26
#define RELAY_STOP    27
#define FLOAT_PIN     32

#define UL_PWR_PIN    14
#define UL_RX         16
HardwareSerial ULSerial(2);

#define PZEM_RX       4
#define PZEM_TX       15
HardwareSerial PZEMSerial(1);

#define ADC_Y         34
#define ADC_B         35

#define POWER_LED     2
#define FAULT_LED     13

#define LCD_SDA       21
#define LCD_SCL       22

#define DHT_PIN       17
#define DS18B20_PIN   33

// ======================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;
bool rtcOK=false;

PZEM004Tv30 pzem(PZEMSerial,PZEM_RX,PZEM_TX);

DHT dht(DHT_PIN,DHT11);

OneWire oneWire(DS18B20_PIN);
DallasTemperature tankSensor(&oneWire);

// ======================================================
// TANK LIMITS
// ======================================================
#define DIST_FULL_CM     91.0
#define DIST_EMPTY_CM   427.0

#define MOTOR_STOP_DIST   30.0
#define MOTOR_START_DIST 200.0

#define ULTRA_MIN_VALID   20.0
#define ULTRA_MAX_VALID  500.0

// ======================================================
#define ULTRA_FAIL_GRACE_MS    (30UL*60UL*1000UL)
#define ULTRA_DEADBAND_CM      3.0

#define MIN_MOTOR_RUNTIME_MS   (5UL*60UL*1000UL)
#define MOTOR_RESTART_LOCK_MS  (2UL*60UL*1000UL)

#define AUTO_BOOT_SETTLE_MS    (3UL*60UL*1000UL)

// ======================================================
#define SENSOR_INTERVAL   2000UL
#define ULTRA_INTERVAL   10000UL
#define MQTT_INTERVAL     2000UL
#define LCD_INTERVAL      1500UL

#define WIFI_RETRY_MS     5000UL
#define DIST_FRESH_MS     (ULTRA_INTERVAL*2)

// ======================================================
#define MOTOR_CONFIRM_A       0.10
#define MOTOR_DRYRUN_A        0.10
#define MOTOR_OVERCURRENT_A  20.0

// ======================================================
// ZMPT
// ======================================================
float CAL_FACTOR_Y=0.183;
float CAL_FACTOR_B=0.180;

float Vy_filtered=0;
float Vb_filtered=0;

// ======================================================
bool autoMode=true;
bool autoReady=false;

bool motorCmd=false;
bool motorRunning=false;

bool hardFault=false;
bool softFault=false;

char hardReason[32]="NONE";
char softReason[32]="NONE";

bool floatLatched=false;

// Distance
float distCM=-1;
float lastStableDist=-1;
float tankPct=0;

bool distValid=false;
unsigned long distLastUpdate=0;

// Electrical
float Vr=0,Vy=0,Vb=0,Ir=0;

// Temps
float ambientTemp=0,humidity=0,tankTemp=0;

// Timers
unsigned long ultraFailStart=0;
unsigned long lastSensor=0,lastUltra=0,lastMQTT=0,lastLCD=0;

unsigned long motorStartTime=0;
unsigned long lastMotorStopTime=0;

unsigned long bootTime=0;

// Debounce
int startConfirmCount=0;
int stopConfirmCount=0;

// WiFi retry
unsigned long lastWiFiRetry=0;

// ✅ FIX-1 Phase Fail debounce counter
static int phaseFailCount=0;

// ======================================================
// MOTOR FSM
// ======================================================
enum MotorState {IDLE,STARTING,RUNNING,STOPPING};
MotorState motorState=IDLE;

unsigned long motorConfirmStart=0;
unsigned long motorStopConfirmStart=0;

#define STOP_CONFIRM_TIMEOUT_MS 5000
#define MANUAL_START_GAP_MS     5000   // ✅ FIX-4

// ======================================================
// WATCHDOG
// ======================================================
void setupWatchdog(){
  esp_task_wdt_config_t cfg={
    .timeout_ms=10000,
    .idle_core_mask=(1<<0),
    .trigger_panic=true
  };
  esp_task_wdt_init(&cfg);
  esp_task_wdt_add(NULL);
}

// ======================================================
void getTimestamp(char*out){
  if(!rtcOK){strcpy(out,"00:00:00");return;}
  DateTime now=rtc.now();
  sprintf(out,"%02d:%02d:%02d",
          now.hour(),now.minute(),now.second());
}

void debugLog(const char*level,const char*msg){
  char ts[16];
  getTimestamp(ts);

  char full[160];
  snprintf(full,sizeof(full),"[%s] [%s] %s",ts,level,msg);

  Serial.println(full);
  if(client.connected()) client.publish("farm/status/debug",full);
}

// ======================================================
// WiFi Manager
// ======================================================
void mqttConnect(){
  if(client.connected()) return;
  if(client.connect("ESP32_FARM_V9_4_2")){
    client.subscribe("farm/cmd/#");
    debugLog("INFO","MQTT Connected");
  }
}

// ======================================================
void pulseRelay(int pin){
  digitalWrite(pin,LOW);
  delay(250);
  digitalWrite(pin,HIGH);
}

// ======================================================
void tripHard(const char*reason){
  if(hardFault) return;

  hardFault=true;
  strncpy(hardReason,reason,sizeof(hardReason));

  digitalWrite(FAULT_LED,HIGH);

  motorCmd=false;
  motorRunning=false;
  motorState=IDLE;

  pulseRelay(RELAY_STOP);
  debugLog("FAULT",reason);
}

void setSoft(const char*reason){
  softFault=true;
  strncpy(softReason,reason,sizeof(softReason));
}

// ======================================================
// ✅ FIX-3 FLOAT ACTIVE LOW
// ======================================================
void checkFloat(){
  if(digitalRead(FLOAT_PIN)==HIGH && !floatLatched){
    floatLatched=true;
    tripHard("FLOAT OVERFLOW");
  }
}

// ======================================================
bool distFresh(){
  return distValid && (millis()-distLastUpdate<DIST_FRESH_MS);
}

// ======================================================
float readZMPT(int pin,float factor){

  const int samples=400;
  long sumRaw=0;

  for(int i=0;i<samples;i++){
    sumRaw+=analogRead(pin);
    delayMicroseconds(120);
  }

  float offset=sumRaw/(float)samples;

  float sumSq=0;
  for(int i=0;i<samples;i++){
    float raw=analogRead(pin);
    float centered=raw-offset;
    sumSq+=centered*centered;
    delayMicroseconds(120);
  }

  float rms=sqrt(sumSq/samples);
  return rms*factor;
}

// ======================================================
float readDistance(){

  float sum=0;
  int valid=0;

  digitalWrite(UL_PWR_PIN,HIGH);
  delay(200);

  unsigned long start=millis();

  while(valid<6 && millis()-start<2000){

    yield();
    esp_task_wdt_reset();

    if(ULSerial.available()>=4 && ULSerial.read()==0xFF){

      uint8_t h=ULSerial.read();
      uint8_t l=ULSerial.read();
      uint8_t c=ULSerial.read();

      if(((0xFF+h+l)&0xFF)==c){

        float d=((h<<8)|l)/10.0;

        if(d>=ULTRA_MIN_VALID && d<=ULTRA_MAX_VALID){
          sum+=d;
          valid++;
        }
      }
    }
  }

  digitalWrite(UL_PWR_PIN,LOW);

  if(valid==0) return -1;
  return sum/valid;
}

// ======================================================
void updateLCD(){

  lcd.clear();

  if(hardFault){
    lcd.setCursor(0,0);
    lcd.print("!! HARD FAULT !!");
    lcd.setCursor(0,1);
    lcd.print(hardReason);
    return;
  }

  lcd.setCursor(0,0);
  lcd.print(autoMode?"AUTO ":"MAN ");
  lcd.print(motorRunning?"ON ":"OFF");


  lcd.setCursor(13,0);
  lcd.print(client.connected()?"M:Y":"M:N");

  lcd.setCursor(0,1);
  lcd.print("T:");
  lcd.print((int)tankPct);
  lcd.print("% ");
  lcd.print((int)distCM);
  lcd.print("cm");
}

// ======================================================
void mqttPublish(){

  if(!client.connected()) return;
  char buf[20];

  sprintf(buf,"%.1f",tankPct);
  client.publish("farm/status/tank",buf,true);

  client.publish("farm/status/mode",autoMode?"AUTO":"MANUAL",true);
  client.publish("farm/status/motor_running",motorRunning?"1":"0",true);

  sprintf(buf,"%.1f",Vr);
  client.publish("farm/status/voltage/r",buf,true);

  sprintf(buf,"%.1f",Vy_filtered);
  client.publish("farm/status/voltage/y",buf,true);

  sprintf(buf,"%.1f",Vb_filtered);
  client.publish("farm/status/voltage/b",buf,true);

  sprintf(buf,"%.2f",Ir);
  client.publish("farm/status/current",buf,true);

  client.publish("farm/status/hard_fault",hardFault?"1":"0",true);
  client.publish("farm/status/hard_reason",hardReason,true);

  client.publish("farm/status/soft_fault",softFault?"1":"0",true);
  client.publish("farm/status/soft_reason",softReason,true);

  sprintf(buf,"%.1f",ambientTemp);
  client.publish("farm/status/ambient_temp",buf,true);

  sprintf(buf,"%.1f",humidity);
  client.publish("farm/status/humidity",buf,true);

  sprintf(buf,"%.1f",tankTemp);
  client.publish("farm/status/tank_temp",buf,true);
}

// ======================================================
void mqttCallback(char*topic,byte*payload,unsigned int length){

  char msg[32]={0};
  for(int i=0;i<length && i<30;i++){
    msg[i]=(char)payload[i];
  }

  if(strcmp(topic,"farm/cmd/reset")==0){

    hardFault=false;
    softFault=false;

    strcpy(hardReason,"NONE");
    strcpy(softReason,"NONE");

    floatLatched=false;
    phaseFailCount = 0;        // ✅ FIX

    digitalWrite(FAULT_LED,LOW);
    debugLog("INFO","RESET DONE");
  }

  if(strcmp(topic,"farm/cmd/restart")==0){
    debugLog("WARN","ESP RESTART...");
    delay(1000);
    ESP.restart();
  }

  if(strcmp(topic,"farm/cmd/mode")==0){
    autoMode=(strcmp(msg,"AUTO")==0);
  }

  if(strcmp(topic,"farm/cmd/motor")==0){

    if(strcmp(msg,"START")==0 && !hardFault){

      // ✅ FIX-4 Manual Start Lockout 5 sec
      if(!autoMode){
        if(millis()-lastMotorStopTime < MANUAL_START_GAP_MS){
          debugLog("WARN","START BLOCKED (MANUAL GAP)");
          return;
        }
      }

      motorCmd=true;
      motorState=STARTING;
      motorConfirmStart=millis();
      pulseRelay(RELAY_START);
    }

    if(strcmp(msg,"STOP")==0){

      motorCmd=false;
      motorState=STOPPING;
      motorStopConfirmStart=millis();
      pulseRelay(RELAY_STOP);
    }
  }
}

// ======================================================

bool ethLinkUp=false;
unsigned long lastEthCheck=0;
#define ETH_CHECK_MS 5000

void ethernetSupervisor(){
  if(millis()-lastEthCheck < ETH_CHECK_MS) return;
  lastEthCheck=millis();

  bool linkNow = (Ethernet.linkStatus()==LinkON);

  if(linkNow && !ethLinkUp){
    debugLog("INFO","ETH Link Restored → Reinit");
    // Re-apply static config
    byte mac2[] = {0xDE,0xAD,0xBE,0xEF,0x32,0x01};
    IPAddress ip2(192,168,1,50);
    IPAddress dns2(192,168,1,1);
    IPAddress gw2(192,168,1,1);
    IPAddress mask2(255,255,255,0);
    Ethernet.begin(mac2, ip2, dns2, gw2, mask2);

    ethLinkUp=true;
    client.disconnect(); // force clean MQTT reconnect
  }
  if(!linkNow && ethLinkUp){
    debugLog("WARN","ETH Link Lost!");
    ethLinkUp=false;
    client.disconnect();
  }
}

void setup(){

  Serial.begin(115200);

  pinMode(RELAY_START,OUTPUT);
  pinMode(RELAY_STOP,OUTPUT);
  digitalWrite(RELAY_START,HIGH);
  digitalWrite(RELAY_STOP,HIGH);

  pinMode(FLOAT_PIN,INPUT_PULLUP);

  pinMode(POWER_LED,OUTPUT);
  pinMode(FAULT_LED,OUTPUT);
  digitalWrite(POWER_LED,HIGH);
  digitalWrite(FAULT_LED,LOW);

  pinMode(UL_PWR_PIN,OUTPUT);
  digitalWrite(UL_PWR_PIN,LOW);

  analogReadResolution(12);

  ULSerial.begin(9600,SERIAL_8N1,UL_RX,-1);
  PZEMSerial.begin(9600,SERIAL_8N1,PZEM_RX,PZEM_TX);

  Wire.begin(LCD_SDA,LCD_SCL);
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0,0);
  lcd.print("FARM SYSTEM");
  lcd.setCursor(0,1);
  lcd.print("BOOTING...");
  delay(3000);

  if(rtc.begin()) rtcOK=true;

  dht.begin();
  tankSensor.begin();

  Ethernet.init(5);
  byte mac[] = {1️⃣ W5500 Ethernet Module (PRIMARY NETWORK — VSPI Default)
⚠ Must use default SPI for stable Ethernet.h
W5500 Pin	ESP32 GPIO	Notes
SCLK	GPIO18	SPI Clock ✅
MISO	GPIO19	SPI Data Out ✅
MOSI	GPIO23	SPI Data In ✅
SCS / CS	GPIO5	Chip Select ✅
RST	GPIO25	Reset pin (recommended) ✅
VCC	3.3V	Use 3.3V safest ✅
GND	GND	Common ground ✅
INT	Not used	Leave open
✅ Ethernet becomes truly industrial stable.
________________________________________
};
  IPAddress ip(192,168,1,50);
  IPAddress dns(192,168,1,1);
  IPAddress gw(192,168,1,1);
  IPAddress mask(255,255,255,0);
  Ethernet.begin(mac, ip, dns, gw, mask);
  debugLog("INFO","ETH Static IP 192.168.1.50 ✅");


  client.setServer(mqtt_server,1883);
  client.setCallback(mqttCallback);

  setupWatchdog();

  bootTime=millis();
  debugLog("INFO","V9.4.2 RELEASE READY ✅");
}

// ======================================================
void loop(){

  esp_task_wdt_reset();

  mqttConnect();
  ethernetSupervisor();
  client.loop();

  checkFloat();

  if(!autoReady && millis()-bootTime>AUTO_BOOT_SETTLE_MS){
    autoReady=true;
    debugLog("INFO","AUTO READY ✅");
  }

  // SENSOR UPDATE
  if(millis()-lastSensor>SENSOR_INTERVAL){

    lastSensor=millis();

    Vr=pzem.voltage();
    Ir=pzem.current();

    static bool alt=false;
    if(!alt){
      Vy=readZMPT(ADC_Y,CAL_FACTOR_Y);
      Vy_filtered=Vy_filtered*0.85+Vy*0.15;
    } else {
      Vb=readZMPT(ADC_B,CAL_FACTOR_B);
      Vb_filtered=Vb_filtered*0.85+Vb*0.15;
    }
    alt=!alt;

    ambientTemp=dht.readTemperature();
    humidity=dht.readHumidity();

    tankSensor.requestTemperatures();
    tankTemp=tankSensor.getTempCByIndex(0);

    // ✅ FIX-1 Phase Fail Debounce (AUTO ONLY)
    if(autoMode && autoReady){

      if(Vr<180 || Vy_filtered<180 || Vb_filtered<180){
        phaseFailCount++;
        if(phaseFailCount>=5) tripHard("PHASE FAIL");
      }
      else phaseFailCount=0;
    }

    // ✅ FIX-2 MANUAL DryRun Stop (Soft Only)
    if(motorState==RUNNING){

      if(Ir < MOTOR_DRYRUN_A){

        if(autoMode){
          tripHard("DRY RUN");
        }
        else{
          setSoft("DRY RUN STOP");
          motorCmd=false;
          motorState=STOPPING;
          motorStopConfirmStart=millis();
          pulseRelay(RELAY_STOP);
        }
      }

      if(Ir > MOTOR_OVERCURRENT_A){
        tripHard("OVERCURRENT");
      }
    }
  }

  // ULTRASONIC UPDATE
  if(millis()-lastUltra>ULTRA_INTERVAL){

    lastUltra=millis();
    float raw=readDistance();

    if(raw<0){

      if(ultraFailStart==0)
        ultraFailStart=millis();

      if(millis()-ultraFailStart > ULTRA_FAIL_GRACE_MS){

        if(autoMode) tripHard("ULTRASONIC FAIL");
        else setSoft("ULTRASONIC FAIL");
      }
      else setSoft("ULTRASONIC WAIT");
    }
    else{

      ultraFailStart=0;

      if(lastStableDist<0) lastStableDist=raw;

      distCM=(raw*0.3)+(lastStableDist*0.7);

      if(fabs(distCM-lastStableDist)<ULTRA_DEADBAND_CM)
        distCM=lastStableDist;

      lastStableDist=distCM;

      distValid=true;
      distLastUpdate=millis();

      tankPct=(DIST_EMPTY_CM-distCM)*100.0/
              (DIST_EMPTY_CM-DIST_FULL_CM);

      tankPct=constrain(tankPct,0,100);

      softFault=false;
      strcpy(softReason,"NONE");
    }
  }

  // FSM START CONFIRM
  if(motorState==STARTING){

    if(Ir>=MOTOR_CONFIRM_A){
      motorRunning=true;
      motorState=RUNNING;
      motorStartTime=millis();
    }
    else if(millis()-motorConfirmStart>8000){

      if(autoMode) tripHard("MOTOR NOT RUNNING");
      else setSoft("START FAIL");

      motorState=IDLE;
      motorCmd=false;
    }
  }

  // FSM STOP CONFIRM
  if(motorState==STOPPING){

    if(Ir<0.05){
      motorRunning=false;
      motorState=IDLE;
      lastMotorStopTime=millis();
    }
    else if(millis()-motorStopConfirmStart>STOP_CONFIRM_TIMEOUT_MS){
      tripHard("STOP FAILED");
    }
  }

  // AUTO START/STOP CONTROL
  if(autoMode && autoReady && !hardFault && distFresh()){

    if(motorState==IDLE && distCM>=MOTOR_START_DIST){

      if(millis()-lastMotorStopTime > MOTOR_RESTART_LOCK_MS){

        startConfirmCount++;
        if(startConfirmCount>=3){

          motorCmd=true;
          motorState=STARTING;
          motorConfirmStart=millis();
          pulseRelay(RELAY_START);

          startConfirmCount=0;
        }
      }
    }
    else startConfirmCount=0;

    if(motorState==RUNNING && distCM<=MOTOR_STOP_DIST){

      unsigned long runtime=millis()-motorStartTime;

      if(runtime>=MIN_MOTOR_RUNTIME_MS){

        stopConfirmCount++;
        if(stopConfirmCount>=3){

          motorCmd=false;
          motorState=STOPPING;
          motorStopConfirmStart=millis();
          pulseRelay(RELAY_STOP);

          stopConfirmCount=0;
        }
      }
    }
    else stopConfirmCount=0;
  }

  // LCD
  if(millis()-lastLCD>LCD_INTERVAL){
    lastLCD=millis();
    updateLCD();
  }

  // MQTT Publish
  if(millis()-lastMQTT>MQTT_INTERVAL){
    lastMQTT=millis();
    mqttPublish();
  }
}
