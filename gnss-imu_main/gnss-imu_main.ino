#include <GNSS.h>
#include <SDHCI.h>
#include <File.h>
#include <MP.h>

#define STRING_BUFFER_SIZE  128

#define MEMSIZE (20*1024) //20KB

struct imuPacket{
  volatile int status; //0: ready, 1: busy, 2: done, -1: no update
  volatile float ave_temp;
  volatile float ave_gyro;
  volatile float ave_gx;
  volatile float ave_gy;
  volatile float ave_gz;
  volatile float ave_accel;
  volatile float ave_ax;
  volatile float ave_ay;
  volatile float ave_az;
  volatile float mdb_accel;
  volatile float mdb_ax;
  volatile float mdb_ay;
  volatile float mdb_az;
};

imuPacket *packet=NULL;

//static SpGnssAddon Gnss;
static SpGnss Gnss;

bool gleds[4]={LOW,LOW,LOW,LOW};
int gled[4]={PIN_LED0,PIN_LED1,PIN_LED2,PIN_LED3};

unsigned long premillis;
unsigned long saveCycle=10000;

SDClass SD;
File myFile;

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

  int error_flag=0;
  Serial.begin(115200);

  sleep(3);

  int result=0;
  result=Gnss.begin();
  if(result!=0){
    Serial.println("Gnss begin error!!");
    error_flag=1;
  }else{
    Gnss.select(GPS);
    Gnss.select(QZ_L1CA);
    Gnss.select(QZ_L1S);
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
  //uint8_t *addr = (uint8_t *)MP.AllocSharedMemory(MEMSIZE);
  if(MP.begin(2)<0){
    Serial.println("MP begin error!!");
    error_flag=1;
  }else{
    Serial.println("MP begin");
    MP.RecvTimeout(MP_RECV_POLLING); 
/*毎回ポインタを使って送信すると
 * arm_hardfault: Hard Fault escalation:
 * arm_hardfault: PANIC!!! Hard Fault!:
 * になる？
 */
    delay(500);
  }
  if(error_flag ==1){
    setLEDs(PIN_LED3,HIGH);
    exit(0);
  }
  premillis=millis();
  Serial.println("End Setup");
}

static void print_pos(SpNavData *pNavData)
{
  char StringBuffer[STRING_BUFFER_SIZE];

  /* print time */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%04d/%02d/%02d ", pNavData->time.year, pNavData->time.month, pNavData->time.day);
  Serial.print(StringBuffer);

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%02d:%02d:%02d.%06ld, ", pNavData->time.hour, pNavData->time.minute, pNavData->time.sec, pNavData->time.usec);
  Serial.print(StringBuffer);

  /* print satellites count */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "numSat:%2d, ", pNavData->numSatellites);
  Serial.print(StringBuffer);

  /* print position data */
  if (pNavData->posFixMode == FixInvalid)
  {
    Serial.print("No-Fix, ");
  }
  else
  {
    Serial.print("Fix, ");
  }
  if (pNavData->posDataExist == 0)
  {
    Serial.print("No Position");
  }
  else
  {
    Serial.print("Lat=");
    Serial.print(pNavData->latitude, 6);
    Serial.print(", Lon=");
    Serial.print(pNavData->longitude, 6);
    Serial.print(", Alt=");
    Serial.print(pNavData->altitude, 3);
  }

  Serial.println("");
}

void print_pos2(GnssPositionData2 *pNavData)
{
  char StringBuffer[STRING_BUFFER_SIZE];

  /* print time */
  Serial.print("NME:");
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%04d/%02d/%02d ", pNavData->Data.receiver.date.year, pNavData->Data.receiver.date.month, pNavData->Data.receiver.date.day);
  Serial.print(StringBuffer);

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%02d:%02d:%02d.%06ld, ", pNavData->Data.receiver.time.hour, pNavData->Data.receiver.time.minute, pNavData->Data.receiver.time.sec, pNavData->Data.receiver.time.usec);
  Serial.print(StringBuffer);

  /* print satellites count */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "numSat:%2d, ", pNavData->Data.svcount);
  Serial.print(StringBuffer);

  /* print position data */
  if (pNavData->Data.receiver.fix_indicator == 0)
  {
    Serial.print("No-Fix, ");
  }
  else
  {
    Serial.print("Fix, ");
  }
  if (pNavData->Data.receiver.pos_dataexist == 0)
  {
    Serial.print("No Position");
  }
  else
  {
    Serial.print("Lat=");
    Serial.print(pNavData->Data.receiver.latitude, 6);
    Serial.print(", Lon=");
    Serial.print(pNavData->Data.receiver.longitude, 6);
    Serial.print(", Alt=");
    Serial.print(pNavData->Data.receiver.altitude, 3);
  }

  Serial.println("");
}

static void save_pos(SpNavData *pNavData)
{
  char StringBuffer[STRING_BUFFER_SIZE];

  /* print time */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%04d/%02d/%02d ", pNavData->time.year, pNavData->time.month, pNavData->time.day);
  myFile.print(StringBuffer);

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%02d:%02d:%02d.%06ld, ", pNavData->time.hour, pNavData->time.minute, pNavData->time.sec, pNavData->time.usec);
  myFile.print(StringBuffer);

  /* print satellites count */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "numSat:%2d, ", pNavData->numSatellites);
  myFile.print(StringBuffer);

  /* print position data */
  if (pNavData->posFixMode == FixInvalid)
  {
    myFile.print("No-Fix, ");
  }
  else
  {
    myFile.print("Fix, ");
  }
  if (pNavData->posDataExist == 0)
  {
    myFile.print("No Position");
  }
  else
  {
    myFile.print("Lat=");
    myFile.print(pNavData->latitude, 6);
    myFile.print(", Lon=");
    myFile.print(pNavData->longitude, 6);
    myFile.print(", Alt=");
    myFile.print(pNavData->altitude, 3);
  }

  myFile.println("");
}

void save_pos2(GnssPositionData2 *pNavData)
{
  char StringBuffer[STRING_BUFFER_SIZE];

  /* print time */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%04d/%02d/%02d ", pNavData->Data.receiver.date.year, pNavData->Data.receiver.date.month, pNavData->Data.receiver.date.day);
  myFile.print(StringBuffer);

  snprintf(StringBuffer, STRING_BUFFER_SIZE, "%02d:%02d:%02d.%06ld, ", pNavData->Data.receiver.time.hour, pNavData->Data.receiver.time.minute, pNavData->Data.receiver.time.sec, pNavData->Data.receiver.time.usec);
  myFile.print(StringBuffer);

  /* print satellites count */
  snprintf(StringBuffer, STRING_BUFFER_SIZE, "numSat:%2d, ", pNavData->Data.svcount);
  myFile.print(StringBuffer);

  /* print position data */
  if (pNavData->Data.receiver.fix_indicator == 0)
  {
    myFile.print("No-Fix, ");
  }
  else
  {
    myFile.print("Fix, ");
  }
  if (pNavData->Data.receiver.pos_dataexist == 0)
  {
    myFile.print("No Position");
  }
  else
  {
    myFile.print("Lat=");
    myFile.print(pNavData->Data.receiver.latitude, 6);
    myFile.print(", Lon=");
    myFile.print(pNavData->Data.receiver.longitude, 6);
    myFile.print(", Alt=");
    myFile.print(pNavData->Data.receiver.altitude, 3);
  }

  myFile.println("");
}

void loop() {
  toggleLEDs(PIN_LED0);
//  GnssPositionData2 PositionData;
  SpNavData NavData;
  if(Gnss.waitUpdate(2)){
//    Gnss.getPositionData(&PositionData);
//    setLEDs(PIN_LED1, (PositionData.Data.receiver.pos_dataexist != 0) && (PositionData.Data.receiver.fix_indicator != 0));
//    print_pos2(&PositionData);
    Gnss.getNavData(&NavData);
    setLEDs(PIN_LED1, (NavData.posDataExist && (NavData.posFixMode != FixInvalid)));
    print_pos(&NavData);
    
  }else{
    Serial.println("data not update");
  }
  int ret;
  int8_t msgid;

  unsigned long ntime=millis();
  if(ntime-premillis>=saveCycle){
    setLEDs(PIN_LED2, HIGH);
    Serial.println("Sending start command");
    packet = (imuPacket *)MP.AllocSharedMemory(MEMSIZE);
    if(!packet){
      Serial.println("Out of memory");
      setLEDs(PIN_LED3,HIGH);
    }else{
      setLEDs(PIN_LED3,LOW);
      printf("SharedMemory Address=@%08lx\n", (uint32_t)packet);
      packet->status=0;
      MP.Send(11, packet, 2);
      Serial.println("Waiting IMU sampling...");
      while(1){
        delay(200);
          if(packet->status==2){
            break;
          }
      }
      Serial.println("Writing to SD...");
      myFile=SD.open("gnss-imu/test.txt", FILE_WRITE);
//      save_pos2(&PositionData);
      save_pos(&NavData);

      char StringBuffer[STRING_BUFFER_SIZE];
      snprintf(StringBuffer, STRING_BUFFER_SIZE, "AveTemp:%.6f ", packet->ave_temp);
      Serial.println(StringBuffer);
      myFile.print(StringBuffer);
      snprintf(StringBuffer, STRING_BUFFER_SIZE, "AveGyro:%.8f,%.8f,%.8f,%.8f", packet->ave_gyro, packet->ave_gx, packet->ave_gy, packet->ave_gz);
      Serial.println(StringBuffer);
      myFile.println(StringBuffer);
      snprintf(StringBuffer, STRING_BUFFER_SIZE, "AveAccel:%.8f,%.8f,%.8f,%.8f", packet->ave_accel, packet->ave_ax, packet->ave_ay, packet->ave_az);
      Serial.println(StringBuffer);
      myFile.print(StringBuffer);
      snprintf(StringBuffer, STRING_BUFFER_SIZE, "ISOdBAccel:%.8f,%.8f,%.8f,%.8f", packet->mdb_accel, packet->mdb_ax, packet->mdb_ay, packet->mdb_az);
      Serial.println(StringBuffer);
      myFile.println(StringBuffer);
      myFile.close();
    }
    MP.FreeSharedMemory(packet);
    setLEDs(PIN_LED2, LOW);
    premillis=premillis+saveCycle;
  }
}
