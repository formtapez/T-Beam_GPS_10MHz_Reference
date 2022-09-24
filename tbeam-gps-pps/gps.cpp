#include "gps.h"
#include "SparkFun_Ublox_Arduino_Library_Series_6_7.h"

HardwareSerial GPSSerial(1);
SFE_UBLOX_GPS myGNSS;

void gps::init()
{
  static boolean serial_ready = false;
  if (serial_ready) GPSSerial.updateBaudRate(GPS_BAUDRATE);
  else
  {
    GPSSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_TX, GPS_RX);
    GPSSerial.setRxBufferSize(2048);  // Default is 256
    GPSSerial.setTimeout(2);
    serial_ready = true;
  }

  // Drain any waiting garbage
  while (GPSSerial.read() != -1);

  // Check all the possible GPS bitrates to get in sync
  while (1)
  {
    GPSSerial.updateBaudRate(GPS_BAUDRATE);  // Try the desired speed first
    if (myGNSS.begin(GPSSerial))
    {
      Serial.println("GPS connected.");
      break;
    }

    Serial.println("Searching GPS @ 115200 Baud...");
    GPSSerial.updateBaudRate(115200);
    if (myGNSS.begin(GPSSerial))
    {
      Serial.println("GPS found at 115200 baud");
      myGNSS.setSerialRate(GPS_BAUDRATE);
      continue;
    }

    Serial.println("Searching GPS @ 9600 Baud...");
    GPSSerial.updateBaudRate(9600);
    if (myGNSS.begin(GPSSerial))
    {
      Serial.println("GPS found at 9600 baud");
      myGNSS.setSerialRate(GPS_BAUDRATE);
      continue;
    }

    Serial.println("Searching GPS @ 38400 Baud...");
    GPSSerial.updateBaudRate(38400);
    if (myGNSS.begin(GPSSerial))
    {
      Serial.println("GPS found at 38400 baud");
      myGNSS.setSerialRate(GPS_BAUDRATE);
      continue;
    }

    Serial.println("Searching GPS @ 57600 Baud...");
    GPSSerial.updateBaudRate(57600);
    if (myGNSS.begin(GPSSerial))
    {
      Serial.println("GPS found at 57600 baud");
      myGNSS.setSerialRate(GPS_BAUDRATE);
      continue;
    }

    Serial.println("Could not connect to GPS. Retrying all speeds...");
  }

  // Configure NMEA messages
  myGNSS.setUART1Output(COM_TYPE_NMEA);  // We do want NMEA
  myGNSS.setNavigationFrequency(1);  // Produce X solutions per second

  // On the Neo6M, these are all off by default anyway:
  myGNSS.disableNMEAMessage(UBX_NMEA_DTM, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GAQ, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GBQ, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GBS, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GLQ, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GNQ, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GNS, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GPQ, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GRS, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GST, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_TXT, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_VLW, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_UART1);

  // Disable these messages that are on after factory reset
  myGNSS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
    
  myGNSS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);   // For Speed
  myGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);   // For Time & Location & SV count

}

void gps::encode()
{       
    int data;
    unsigned long previousMillis = millis();

    while((previousMillis + 100) > millis())
    {
        while (GPSSerial.available() )
        {
            char data = GPSSerial.read();
            tGps.encode(data);
//            Serial.print(data);
        }
    }
//     Serial.println("");
}

void gps::passthrough()
{
  Serial.println("GPS Passthrough forever...");
  while (1) {
    if (GPSSerial.available()) Serial.write(GPSSerial.read());
    if (Serial.available()) GPSSerial.write(Serial.read());
  }
}

void gps::buildPacket(uint8_t txBuffer[9])
{
  LatitudeBinary = ((tGps.location.lat() + 90) / 180.0) * 16777215;
  LongitudeBinary = ((tGps.location.lng() + 180) / 360.0) * 16777215;
  
  sprintf(t, "Lat: %f", tGps.location.lat());
  Serial.println(t);
  
  sprintf(t, "Lng: %f", tGps.location.lng());
  Serial.println(t);
  
  txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  txBuffer[2] = LatitudeBinary & 0xFF;

  txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  txBuffer[5] = LongitudeBinary & 0xFF;

  altitudeGps = tGps.altitude.meters();
  txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
  txBuffer[7] = altitudeGps & 0xFF;

  hdopGps = tGps.hdop.value()/10;
  txBuffer[8] = hdopGps & 0xFF;
}

void gps::gdisplay(uint16_t txBuffer2[5])
{
  txBuffer2[0] = tGps.satellites.value();
  txBuffer2[1] = tGps.speed.kmph();
  txBuffer2[2] = tGps.course.deg();
  txBuffer2[3] = tGps.altitude.meters();
  txBuffer2[4] = tGps.hdop.value()/10;
}

void gps::getpos(double pos[2])
{
  pos[0] = tGps.location.lat();
  pos[1] = tGps.location.lng();
}

bool lostFix = true;

bool gps::checkGpsFix()
{
  encode();
  if (tGps.location.isValid() && 
      tGps.location.age() < 4000 &&
      tGps.hdop.isValid() &&
      tGps.hdop.value() <= 600 &&
      tGps.hdop.age() < 4000 &&
      tGps.altitude.isValid() && 
      tGps.altitude.age() < 4000 )
  {

    if (lostFix) Serial.println("Got valid GPS fix.");
    lostFix = false;
    return true;
  }
  else
  {
     if (!lostFix) Serial.println("Lost GPS fix.");
     lostFix  = true;
/*
     sprintf(t, "location valid: %i" , tGps.location.isValid());
     Serial.println(t);
     sprintf(t, "location age: %i" , tGps.location.age());
     Serial.println(t);
     sprintf(t, "hdop valid: %i" , tGps.hdop.isValid());
     Serial.println(t);
     sprintf(t, "hdop age: %i" , tGps.hdop.age());
     Serial.println(t);
     sprintf(t, "hdop: %i" , tGps.hdop.value());
     Serial.println(t);
     sprintf(t, "altitude valid: %i" , tGps.altitude.isValid());
     Serial.println(t);
     sprintf(t, "altitude age: %i" , tGps.altitude.age());
     Serial.println(t);
     sprintf(t, "sats: %i, valid: %i" , tGps.satellites.value(), tGps.satellites.isValid());
     Serial.println(t);
*/
    return false;
  }
}
