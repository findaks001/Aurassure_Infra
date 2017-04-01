#include "DHT.h"
#include <PROCESS9.h>
#include <Wire.h>
#include <Arduino.h>

#define DHTTYPE DHT22           // DHT 22  (AM2302), AM2321
#define DHTPIN A3


#define ss Serial

#include <TinyGPS.h>
TinyGPS gps;

#define alphasense 0x48
#define otherSensor 0x49

const byte channels[8] = {0x8C,0xCC,0x9C,0xDC,0xAC,0xEC,0xBC,0xFC};

// ADC IC connected to Alphasense Gas sensors
// Address A1=0, A0=0
const byte no2AuxilaryChannel = 0;
const byte no2WorkingChannel = 1;
const byte o3AuxilaryChannel = 2;
const byte o3WorkingChannel = 3;
const byte coAuxilaryChannel = 4;
const byte coWorkingChannel = 5;
const byte so2AuxilaryChannel = 6;
const byte so2WorkingChannel = 7;

const int no2WEZero = 284;
const int no2AEZero = 281;
const int coWEZero = 290;
const int coAEZero = 292;
const int so2WEZero = 273;
const int so2AEZero = 279;
const int o3WEZero = 388;
const int o3AEZero = 401;
const float no2Sensitivity = 0.229;
const float coSensitivity = 0.237;
const float so2Sensitivity = 0.330;
const float o3Sensitivity = 0.345;

// ADC IC connected to other sensors
// Address: A1=0, A0=1
const byte co2Channel = 0;
const byte noiseChannel = 1;
const byte uvChannel = 2;

int PM01Value = 0;        //define PM1.0 value of the air detector module
int PM2_5Value = 0;       //define PM2.5 value of the air detector module
int PM10Value = 0;       //define PM10 value of the air detector module

#define LENG 31
unsigned char buf[LENG]; //receive data from the air detector module

DHT dht(DHTPIN, DHTTYPE);

#define TRANSMISSION_INTERVAL 1*60
#define DELAY_INTERVAL 0 * 60   // 15 is number of minutes, 60 is multiplied to convert the minutes to second
unsigned long g_time = 0;    //  This number will overflow (go back to zero), after approximately 38 days.

#define SerialNeeded 1

float h, t , co2, co, so2, no2, o3, noise;
float flat, flon;
int n;

// Look up table for n from -30 to 50 for A4 type sensors
const PROGMEM float look_up_o3[] = {0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 2.87};
const PROGMEM float look_up_so2[] = {0.85, 0.85, 0.85, 0.85, 0.85, 1.15, 1.45, 1.75, 1.95};
const PROGMEM float look_up_no2[] = {1.18, 1.18, 1.18, 1.18, 1.18, 1.18, 1.18, 2.00, 2.70};
const PROGMEM float look_up_co[] = {1.40, 1.03, 0.85, 0.62, 0.30, 0.03, -0.25, -0.48, -0.80};

String data;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Serial1.begin(9600);
  Serial1.setTimeout(1500);    //set the Timeout to 1500ms, longer than the data transmission periodic time of the sensor
  Wire.begin();             // join i2c bus (address optional for master) 
  delay(1000);
  pinMode(9,OUTPUT);
  digitalWrite(9, HIGH);
  dht.begin();
  ISASerial.begin(9600);
  gsmSerial.begin(9600);
  DataPort.begin(9600);

  GSMModelStandard();
  timeInitialization();
  delay(2000);

  while (!delay_period());
}

void loop() {
  analogWrite(9,150);
  float battery_voltage = (analogRead(A1)*8.4/1023.0);
  Serial.print("Battery Voltage is:");
  Serial.println(battery_voltage);
  readSensors();
  delay(4000);
  data = stringFormation(); 

 // config_packet_aurassure(id, auth_key, interval, data);
  delay(1000);
  
  if (check_gprs_transmission_period()) {

    networkErrorHandling();
    transmit_gprs_data();
  }
}

void readSensors()
{
  readTemperatureHumidity();
  delay(1000);
  readTempCoefficient();
  delay(1000);
  readCo2();
  delay(1000);
  readNo2();
  delay(1000);
  readCo();
  delay(1000);
  readO3();
  delay(1000);
  readSo2();
  delay(1000);
  readDust();
  delay(1000);
  readnoise();
  delay(1000);
  readGPS();
  delay(1000);
}

String stringFormation() {
  String parameters = "{'temperature':'" + String(t) + "','humidity':'" + String(h) + "','co2':'" + String(co2) + "','no2':'" + String(no2) + "','o3':'" + String(o3) + "','co':'" + String(co) + "','so2':'" + String(so2) + "','pm1':'" + String(PM01Value) + "','pm2.5':'" + String(PM2_5Value) + "','pm10':'" + String(PM10Value) + "','noise':'" + String(noise) + "','latitude':'" + String(flat,6) + "','longitude':'" + String(flon,6) + "'}";

  //String parameters = "['" + String(t) + "','" + String(h) + "','" + String(no2) + "','" + String(o3) + "','" + String(co) + "','" + String(so2) + "','" + String(PM01Value) + "','" + String(PM2_5Value) + "','" + String(PM10Value) + "']";
  //  String packet = "packet={'i':'" + id + "','k':'" + key + "','data':" + data + "}" ;
  //  packet.replace("'", "\""); // JSON requires double quoted string to be valid.
#if SerialNeeded
  Serial.println(parameters);
#endif
  return parameters;
}

/*
  .  This will be called after certain intervals defined by the user
  .  It is used to transmit the data to the servers
*/
void transmit_gprs_data() {

  int connectionNo = 0;
  String gprs_packet_data = "";



  //http://api.aurassure.com/device/v1/log?i=c52841sr&e={"a":"abf1ea25","n":0,"da":[{"d":"04-02-2017","t":"20:09:18","p":["0","0","nan","nan","0.00","0.00","10.0","1.0","0.4","0"]}]}
   data = stringFormation();
   String id = "CSwBbeYZ";
   String auth_key = "SRi9aLrt";
   int interval = 0;
   GSMOBJ.time_info();
  String packet = config_packet_aurassure(id, auth_key, interval, data);

  String Ip = "api.aurassure.com";
  String Port = "80";
  String protocol = "http";
  String URL = "/device/v2/log?";
  ConnectionStatus = disconnected;
  gprs_send(packet, connectionNo, ConnectionStatus, protocol, Ip, Port, URL);

}

void readTemperatureHumidity() {
  h = dht.readHumidity();
#if SerialNeeded
  Serial.print("Humidity : ");
  Serial.println(h);
#endif
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();
#if SerialNeeded
  Serial.print("Temperature : ");
  Serial.println(t);
#endif
  if (isnan(h) || isnan(t)) {
    h = -1;
    t = -1;
#if SerialNeeded
    Serial.println("Failed to read from DHT sensor!");
#endif
    return;
  }
}

void readTempCoefficient()
{
  // read temperature and find out n
  n = t < 0 ? (t / 10) + 4 : (t / 10) + 3;
  // Serial.println(n);
}

void readCo2() {
  int sensorValue = readChannel(otherSensor,co2Channel);  

  // The analog signal is converted to a voltage 
  float voltage = sensorValue*(5000.0/4096); 
  if(voltage == 0)
  {
    #if SerialNeeded
    Serial.println("Fault");
    #endif
  }
  else if(voltage < 400)
  {
    #if SerialNeeded
    Serial.println("preheating");
    #endif
  }
  else
  {
    int voltage_diference=voltage-400;
    co2=voltage_diference*50.0/16.0;
    // Print Voltage
    #if SerialNeeded
    Serial.print("voltage:");
    Serial.print(voltage);
    Serial.println("mv");
    //Print CO2 concentration
    Serial.print(co2);
    Serial.println("ppm");
    #endif
  }
  delay(100); 
}

void readNo2() {
  double no2VoltW = readChannel(alphasense,no2WorkingChannel)*(6000.0/4096); // convert to mV
#if SerialNeeded
  Serial.print("Working Electrode of NO2 : ");
  Serial.println(no2VoltW);
#endif
  delay(100);
  double no2VoltA = readChannel(alphasense,no2AuxilaryChannel)*(6000.0/4096); // convert to mV
#if SerialNeeded
  Serial.print("Auxiliary Electrode of NO2 : ");
  Serial.println(no2VoltA);
#endif

  // Calulation of gas concentration [(Vwe-Vpcbwe)-n*(Vae-Vpcbae)]/sensitivity
  no2 = abs(((no2VoltW - no2WEZero) - ( pgm_read_float_near(look_up_no2 + n) * (no2VoltA - no2AEZero))) / no2Sensitivity);
  //float no2 = ((no2VoltW - 306) - (no2VoltA - 312)) / 0.232;
#if SerialNeeded
  Serial.print("NO2 : ");
  Serial.println(no2);
#endif
}

void readCo() {
  double coVoltW =  readChannel(alphasense,coWorkingChannel)*(6000.0/4096);; // convert to mV;
#if SerialNeeded
  Serial.print("working Electrode of CO : ");
  Serial.println(coVoltW);
#endif
  delay(100);
  double coVoltA = readChannel(alphasense,coAuxilaryChannel)*(6000.0/4096); // convert to mV
#if SerialNeeded
  Serial.print("Auxiliary Electrode of CO : ");
  Serial.println(coVoltA);
#endif
  co = abs(((coVoltW - coWEZero) - (pgm_read_float_near(look_up_co + n) * (coVoltA - coAEZero)))/ coSensitivity);
  //float co = ((coVoltW - 278) - (coVoltA - 270)) / 0.321;
  co = co / 1000;       // Convert to ppm
#if SerialNeeded
  Serial.print("CO : ");
  Serial.println(co);
#endif
}

void readO3() {

  double o3VoltW =  readChannel(alphasense,o3WorkingChannel)*(6000.0/4096); // convert to mV
#if SerialNeeded
  Serial.print("working Electrode of O3 : ");
  Serial.println(o3VoltW);
#endif
  delay(100);
  double o3VoltA = readChannel(alphasense,o3AuxilaryChannel)*(6000.0/4096); // convert to mV
#if SerialNeeded
  Serial.print("Auxiliary Electrode of O3  : ");
  Serial.println(o3VoltA);
#endif

  o3 = abs(((o3VoltW - o3WEZero) - (pgm_read_float_near(look_up_o3 + n) * (o3VoltA - o3AEZero))) / o3Sensitivity);
  //float O3 = ((O3VoltW - 360) - (O3VoltA - 340)) / 0.182;
#if SerialNeeded
  Serial.print("O3 : ");
  Serial.println(o3);
#endif

}

void readSo2() {

  double so2VoltW =  readChannel(alphasense,so2WorkingChannel)*(6000.0/4096); // convert to mV
#if SerialNeeded
  Serial.print("working Electrode of SO2 : ");
  Serial.println(so2VoltW);
#endif
  delay(100);
  double so2VoltA = readChannel(alphasense,so2AuxilaryChannel)*(6000.0/4096); // convert to mV
#if SerialNeeded
  Serial.print("Auxiliary Electrode of SO2 : ");
  Serial.println(so2VoltA);
#endif

  so2 = abs(((so2VoltW - so2WEZero) - (pgm_read_float_near(look_up_so2 + n) * (so2VoltA - so2AEZero))) / so2Sensitivity);
  //float so2 = ((so2VoltW - 278) - (so2VoltA - 284)) / 0.300;
#if SerialNeeded
  Serial.print("SO2 : ");
  Serial.println(so2);
#endif
}

void readDust() {

  if (Serial1.find(0x42)) {  //start to read when detect 0x42
    Serial1.readBytes(buf, LENG);

    if (buf[0] == 0x4d) {
      if (checkValue(buf, LENG)) {
        PM01Value = transmitPM01(buf); //count PM1.0 value of the air detector module
        PM2_5Value = transmitPM2_5(buf); //count PM2.5 value of the air detector module
        PM10Value = transmitPM10(buf); //count PM10 value of the air detector module
      }
    }
  }

  static unsigned long OledTimer = millis();
  if (millis() - OledTimer >= 1000)
  {
    OledTimer = millis();

#if SerialNeeded
    Serial.print("PM1.0: ");
    Serial.print(PM01Value);
    Serial.println("  ug/m3");

    Serial.print("PM2.5: ");
    Serial.print(PM2_5Value);
    Serial.println("  ug/m3");

    Serial.print("PM1 0: ");
    Serial.print(PM10Value);
    Serial.println("  ug/m3");
    Serial.println();
#endif
  }
}

char checkValue(unsigned char *thebuf, char leng)
{
  char receiveflag = 0;
  int receiveSum = 0;

  for (int i = 0; i < (leng - 2); i++) {
    receiveSum = receiveSum + thebuf[i];
  }
  receiveSum = receiveSum + 0x42;

  if (receiveSum == ((thebuf[leng - 2] << 8) + thebuf[leng - 1])) //check the serial data
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}

int transmitPM01(unsigned char *thebuf)
{
  int PM01Val;
  PM01Val = ((thebuf[3] << 8) + thebuf[4]); //count PM1.0 value of the air detector module
  return PM01Val;
}

//transmit PM Value to PC
int transmitPM2_5(unsigned char *thebuf)
{
  int PM2_5Val;
  PM2_5Val = ((thebuf[5] << 8) + thebuf[6]); //count PM2.5 value of the air detector module
  return PM2_5Val;
}

//transmit PM Value to PC
int transmitPM10(unsigned char *thebuf)
{
  int PM10Val;
  PM10Val = ((thebuf[7] << 8) + thebuf[8]); //count PM10 value of the air detector module
  return PM10Val;
}

void readnoise() {
  //Read voltage in milliVolts
  noise = readChannel(otherSensor,noiseChannel)*(3300/4096.0);
  
#if SerialNeeded
  Serial.print(F("Noise Sensor Analogue voltage : "));
  Serial.print(noise);
  Serial.println(F("mv"));
#endif

  /* Noise to DB Convertion
    .  -42 dBV/Pa = 0.00794 RMS V/Pa    .
    .  Noise DB = 20 log (Vout / 0.00794)
  */

  noise = 20 * log10(noise / 7.9433);
  if(noise == NAN) noise = -1;
#if SerialNeeded
  Serial.print(F("Noise Sensor Reading in dB : "));
  Serial.print(noise);
  Serial.println(F("dB"));
#endif
}

bool check_gprs_transmission_period() {

  // check if the time interval has reached then start transmission
  if (TRANSMISSION_INTERVAL < (millis() - g_time) / 1000) {
    g_time = millis();               // Store the Time for next check
    return  true;
  }
  else
    return false;
}


bool delay_period() {

  // check if the time interval has reached then start transmission
  if (DELAY_INTERVAL < (millis() - g_time) / 1000) {
    return  true;
  }
  else {
    return false;
  }
}

int readChannel(int I2C_address,byte channelNumber)
{
  int value;
  byte adval_high, adval_low;

  Wire.beginTransmission(I2C_address);
  Wire.write(channels[channelNumber]);
  Wire.endTransmission();
  delay(1);

  Wire.requestFrom(I2C_address, 2);
  
  while(Wire.available())
  {
    adval_high = Wire.read();
    adval_low = Wire.read();
  }

    value = getDecimalValue(adval_high, adval_low);

    return value;
}


int getDecimalValue(byte adval_high, byte adval_low)
{
  int dec = 0;
  dec = adval_high;
  dec = dec << 8;
  dec = dec + adval_low;

  return dec;
}


void readGPS() {
  //ss.begin(9600);
    smartdelay(1000);
    
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
  

  gps.f_get_position(&flat, &flon, &age);
  #if SerialNeeded
  Serial.print(flat,6);
  Serial.print("   ");
  Serial.println(flon,6);
  #endif
  //ss.end();
  delay(1000);  
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
