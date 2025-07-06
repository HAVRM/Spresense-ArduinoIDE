#include <GNSS.h>
#include <SDHCI.h>
#include <File.h>
#include "multiimu_set.h"

#define USE_GNSS_ADDON

#define STRING_BUFFER_SIZE  128

#ifdef USE_GNSS_ADDON
  static SpGnssAddon Gnss;
#else
  static SpGnss Gnss;
#endif

bool gleds[4]={LOW,LOW,LOW,LOW};
int gled[4]={PIN_LED0,PIN_LED1,PIN_LED2,PIN_LED3};

unsigned long premillis;
unsigned long saveCycle=10000;

SDClass SD;
File myFile;

#include "gnss_print.h" //Using "myFile"

void changeLEDs(){
  for(int i=0;i<4;i++){
    digitalWrite(gled[i],gleds[i]);
  }
}

void toggleLEDs(int led){
  for(int i=0;i<4;i++){
    if(gled[i]==led){
      gleds[i]=!gleds[i];
    }
  }
  changeLEDs();
}

void setLEDs(int led, bool state){
  for(int i=0;i<4;i++){
    if(gled[i]==led){
      gleds[i]=state;
    }
  }
  changeLEDs();
}


void setup() {
  for(int i=0;i<4;i++){
    pinMode(gled[i], OUTPUT);
  }
  changeLEDs(); //LED All Off

  int error_flag=0;
  Serial.begin(115200);

  sleep(3);

  int result=0;
  result=Gnss.begin();
  if(result!=0){
    Serial.println("Gnss begin error!!");
    error_flag=1;
  }else{

#ifndef USE_GNSS_ADDON
    Gnss.select(GPS);
    Gnss.select(QZ_L1CA);
    Gnss.select(QZ_L1S);
#endif

    result=Gnss.start();
    if(result!=0){
      Serial.println("Gnss start error!!");
      error_flag=1;
    }else{
      Serial.println("Gnss setup OK");
    }
  }
  if(SD.begin()){
    Serial.println("SD setup OK");
    SD.mkdir("gnss-imu/");
    myFile=SD.open("gnss-imu/test.txt", FILE_WRITE);
    myFile.println("AveTemp:<temp> AveGyro:<gyro>,<gx>,<gy>,<gz>");
    myFile.println("AveAccel:<accel>,<ax>,<ay>,<az> ISOdBAccel:<accel>,<ax>,<ay>,<az>");
    myFile.close();
  }else{
    Serial.println("No SD mounted");
    error_flag=1;
  }

  if(board_cxd5602pwbimu_initialize(5)<0){
    Serial.println("ERROR: Failed to initialize CXD5602PWBIMU.");
    error_flag=1;
  }else{
    Serial.println("board_cxd5602pwbimu_initialize: OK");
  }
  
  delay(500);

  if(error_flag ==1){
    setLEDs(PIN_LED3,HIGH);
    exit(0);
  }
  premillis=millis();
  Serial.println("End Setup");
}
void loop() {
  GnssPositionData2 PositionData;
  SpNavData NavData;

  if(Gnss.waitUpdate(2)){
#ifdef USE_GNSS_ADDON
    Gnss.getPositionData(&PositionData);
    setLEDs(PIN_LED1, ((PositionData.Data.receiver.date.year != 2000) && gleds[0]) || ((PositionData.Data.receiver.pos_dataexist != 0) && (PositionData.Data.receiver.fix_indicator != 0)));
    print_pos2(&PositionData);
#else
    Gnss.getNavData(&NavData);
    setLEDs(PIN_LED1, ((NavData.time.year != 1980) && gleds[0]) || (NavData.posDataExist && (NavData.posFixMode != FixInvalid)));
    print_pos(&NavData);
#endif
    toggleLEDs(PIN_LED0);
  }else{
    Serial.println("data not update");
  }
  int ret;
  int8_t msgid;

  unsigned long ntime=millis();
  if(ntime-premillis>=saveCycle){
    setLEDs(PIN_LED2, HIGH);
    Serial.println("Start IMU sampling");
    Serial.println("Writing to SD...");
    myFile=SD.open("gnss-imu/test.txt", FILE_WRITE);
    if(myFile){
#ifdef USE_GNSS_ADDON
      save_pos2(&PositionData);
#else
      save_pos(&NavData);
#endif
      getIMUDATA(0,NULL);
      setLEDs(PIN_LED3, packet.status);
      char StringBuffer[STRING_BUFFER_SIZE];
      snprintf(StringBuffer, STRING_BUFFER_SIZE, "AveTemp:%.6f ", packet.ave_temp);
      Serial.println(StringBuffer);
      myFile.print(StringBuffer);
      snprintf(StringBuffer, STRING_BUFFER_SIZE, "AveGyro:%.8f,%.8f,%.8f,%.8f", packet.ave_gyro, packet.ave_gx, packet.ave_gy, packet.ave_gz);
      Serial.println(StringBuffer);
      myFile.println(StringBuffer);
      snprintf(StringBuffer, STRING_BUFFER_SIZE, "AveAccel:%.8f,%.8f,%.8f,%.8f", packet.ave_accel, packet.ave_ax, packet.ave_ay, packet.ave_az);
      Serial.println(StringBuffer);
      myFile.print(StringBuffer);
      snprintf(StringBuffer, STRING_BUFFER_SIZE, "ISOdBAccel:%.8f,%.8f,%.8f,%.8f", packet.mdb_accel, packet.mdb_ax, packet.mdb_ay, packet.mdb_az);
      Serial.println(StringBuffer);
      myFile.println(StringBuffer);
      myFile.close();
      premillis=premillis+saveCycle;
    }else{
      Serial.println("File opening error");
      setLEDs(PIN_LED3, HIGH);
    }
    setLEDs(PIN_LED2, LOW);
  }
}
