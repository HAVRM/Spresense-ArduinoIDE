#ifndef GNSS_PRINT_H_HAVRM_20250703
#define GNSS_PRINT_H_HAVRM_20250703

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
  if (pNavData->posFixMode == FixInvalid){
    Serial.print("No-Fix, ");
  }else{
    Serial.print("Fix   , ");
  }
  if (pNavData->posDataExist == 0){
    Serial.print("No Position: ");
  }else{
    Serial.print("Position   : ");
  }
  Serial.print("Lat=");
  Serial.print(pNavData->latitude, 6);
  Serial.print(", Lon=");
  Serial.print(pNavData->longitude, 6);
  Serial.print(", Alt=");
  Serial.print(pNavData->altitude, 3);

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
  if (pNavData->Data.receiver.fix_indicator == 0){
    Serial.print("No-Fix, ");
  }else{
    Serial.print("Fix   , ");
  }
  if (pNavData->Data.receiver.pos_dataexist == 0){
    Serial.print("No Position: ");
  }else{
    Serial.print("Position   : ");
  }
  Serial.print("Lat=");
  Serial.print(pNavData->Data.receiver.latitude, 6);
  Serial.print(", Lon=");
  Serial.print(pNavData->Data.receiver.longitude, 6);
  Serial.print(", Alt=");
  Serial.print(pNavData->Data.receiver.altitude, 3);

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
  if (pNavData->posFixMode == FixInvalid){
    myFile.print("No-Fix, ");
  }else{
    myFile.print("Fix   , ");
  }
  if (pNavData->posDataExist == 0){
    myFile.print("No Position: ");
  }else{
    myFile.print("Position   : ");
  }
  myFile.print("Lat=");
  myFile.print(pNavData->latitude, 6);
  myFile.print(", Lon=");
  myFile.print(pNavData->longitude, 6);
  myFile.print(", Alt=");
  myFile.print(pNavData->altitude, 3);

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
  if (pNavData->Data.receiver.fix_indicator == 0){
    myFile.print("No-Fix, ");
  }else{
    myFile.print("Fix   , ");
  }
  if (pNavData->Data.receiver.pos_dataexist == 0){
    myFile.print("No Position: ");
  }else{
    myFile.print("Position   : ");
  }
  myFile.print("Lat=");
  myFile.print(pNavData->Data.receiver.latitude, 6);
  myFile.print(", Lon=");
  myFile.print(pNavData->Data.receiver.longitude, 6);
  myFile.print(", Alt=");
  myFile.print(pNavData->Data.receiver.altitude, 3);

  myFile.println("");
}

#endif
