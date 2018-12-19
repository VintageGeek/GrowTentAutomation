/*
 * Project GrowTentAutomation
 * Description: Automation of watering and temp/humidity sensor recording
 * Author: Mike Morales
 * Date: 27Nov2017
 */

//#region - Global settings
// This #include statement was automatically added by the Particle IDE.
#include <NCD2Relay.h>
#include <I2CSoilMoistureSensor.h>

//Directives
#define NUMBER_OF_SOIL_SENSORS 2
#define HumiChipAddress 0x28  //https://github.com/ControlEverythingCommunity/HCPA-5V-U3/blob/master/README.md
#define i2CMuxer 0x74
#define NUMBER_OF_MUX_CHANNELS 5
#define MAX_MUX_CHANNEL_DEVICES 127
#define NUMBER_OF_RELAYS 2
#define LOOP_DELAY 15000

//global variables
byte soilSensorArray[NUMBER_OF_SOIL_SENSORS] {0x30,0x31};
String soilSensorReadingArray[NUMBER_OF_SOIL_SENSORS][4];
bool ic2MuxArray[NUMBER_OF_MUX_CHANNELS][MAX_MUX_CHANNEL_DEVICES] ;
NCD2Relay relay;
int relayState[NUMBER_OF_RELAYS];
bool relayEnabled[NUMBER_OF_RELAYS];
double tentTempCelsius = 0.0, tentTempFahrenheit = 0.0, tentHumidity = 0.0;
bool error;
String errorMessage;
String msg = "";
int ventTent=false;

//#endregion

//#region - Main program methods
void setup(){

  Particle.variable("VentTent", ventTent);
  Particle.publish("setup", "Starting setup...", 60, PRIVATE);

    //Init Wire for I2C Communication
    Wire.setSpeed(400000);
      if (!Wire.isEnabled()) {
          Wire.begin();
      }

    initMuxArray();
    initSoilMoistureSensors();
    initNCD2Relay();
    initHumiChip();
    delay(1000);
  Particle.publish("setup", "Finished setup", 60, PRIVATE);
}
void loop() {
  getHumiChipMeasures();
  getSoilSensorMeasures();
  getRelayStatus();
  PublishTentMeasurements();
  PublishSoilMeasurements();
  delay(LOOP_DELAY);
}
//#endregion

//#region - HumiChip methods
void initHumiChip(){
    // Start I2C transmission for module
  Wire.beginTransmission(HumiChipAddress);
  // Send start command
  Wire.write(0x80);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(300);
  Particle.publish("setup", "Humidity & Temp Sensor Started");
}
void getHumiChipMeasures(){
  unsigned int data[4];
  Wire.beginTransmission(HumiChipAddress);
  Wire.endTransmission();
  // Request 4 byte of data
  Wire.requestFrom(HumiChipAddress, 4);
  // Read 4 bytes of data
  // humidity msb, humidity lsb, cTemp msb, cTemp lsb
  if (Wire.available() == 4)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
  // Convert the data to 14-bits
  tentHumidity = (((data[0] & 0x3F) * 256) + data[1]) / 16384.0 * 100.0;
  tentTempCelsius = (((data[2] * 256) + (data[3] & 0xFC)) / 4) / 16384.0 * 165.0 - 40.0;
  tentTempFahrenheit = (tentTempCelsius * 1.8) + 32;
  //adjust temp and humidity based on calibration of 2 other thermometers/humidity sensors
  //tentTempFahrenheit+=.5;
  tentHumidity-=6.4;
  }
}
//#endregion

//#region - SoilSensor(s) methods
void initSoilMoistureSensors(){
      String address="";
      String version = "";

      for (size_t i = 1; i <= NUMBER_OF_SOIL_SENSORS; i++) {
         selectChannel(i);
         I2CSoilMoistureSensor soilSensor(soilSensorArray[i-1]);
         soilSensor.begin();
         delay(3000); // give some time to boot up
         msg = "I2C Soil Sensor "+ String(i) +" of "+ String(NUMBER_OF_SOIL_SENSORS) + ": ";
         address = String(soilSensor.getAddress(), HEX);
         version = String(soilSensor.getVersion());

        // publish sensors info
        Particle.publish("setup", msg + " Ch: " + i + " - Addr: " + address + " - FW: " + version, 60, PRIVATE);
        delay(1000);
      }
      selectChannel(0);  //reset mux - all channels off
}
void getSoilSensorMeasures() {
  for (size_t i = 0; i < NUMBER_OF_SOIL_SENSORS; i++) {
    selectChannel(i+1);
    I2CSoilMoistureSensor soilSensor(soilSensorArray[i]);
    soilSensor.begin();
    delay(3000); // give some time to boot up
    //    selectChannel(i+1);
    //  if (i>0){
    //  soilSensor.changeSensor(soilSensorArray[i]);
    //}
    GetSoilMoisture(&soilSensor, i);
    GetSoilTemperature(&soilSensor, i);
    GetSoilLight(&soilSensor, i);
  }
  selectChannel(0);
}
void GetSoilMoisture(I2CSoilMoistureSensor *currentSensor, int sensorNumber){
  int moisture=0;
    while (currentSensor->isBusy()) delay(50); // available since FW 2.3
    int soilMoistureRaw = currentSensor->getCapacitance();
    if (!IsValidSoilMoisture(soilMoistureRaw)){
        errorMessage="Soil moisture reading out of bounds: " + String(soilMoistureRaw);
        soilSensorReadingArray[sensorNumber][0]="ERROR";
        soilSensorReadingArray[sensorNumber][1]="ERROR";
        HandleError("GetMoisture");
        return ;
    }

    if (sensorNumber==0) {
       moisture = map(soilMoistureRaw,273,619,0,100);
    } else {
       moisture = map(soilMoistureRaw,273,646,0,100);

    }

    soilSensorReadingArray[sensorNumber][0]=String(soilMoistureRaw);
    soilSensorReadingArray[sensorNumber][1]=String(moisture);
}
void GetSoilTemperature(I2CSoilMoistureSensor *currentSensor, int sensorNumber){
    int temperature = currentSensor->getTemperature();
    if (!IsValidTemperature(temperature)){
        errorMessage="Temperature reading out of bounds: " + String(temperature);
        soilSensorReadingArray[sensorNumber][2]="ERROR";
        HandleError("GetTemperature");
        return;
    }
    float tempInF=(temperature/(float)10)*9/5+32;

     soilSensorReadingArray[sensorNumber][2]=String(tempInF);
}
void GetSoilLight(I2CSoilMoistureSensor *currentSensor, int sensorNumber){
    int light = currentSensor->getLight(true);
    soilSensorReadingArray[sensorNumber][3]=String(light);
}
bool IsValidSoilMoisture(int moistureReading){
    return ((moistureReading > 200) & (moistureReading <700));
}
bool IsValidTemperature(int temperatureReading){
    return ((temperatureReading>0) & (temperatureReading <500));
}
int numberOfSoilSensors(){
  int numberOfSensors=0;
  for (size_t i = 1; i < NUMBER_OF_MUX_CHANNELS; i++) {
    for (size_t j = 16; j < MAX_MUX_CHANNEL_DEVICES; j++) {
      if (ic2MuxArray[i][j]) {
        numberOfSensors++;
      }
    }
  }
  return numberOfSensors;
}
//#endregion

//#region - NCD2Relay methods
void initNCD2Relay(){
  //initiate the NCD2Relay

  relay.setAddress(1,0,0);
  getRelayStatus();
  Particle.variable("Humidifier", relayState[0]);
  Particle.variable("WaterPump", relayState[1]);
  relay.turnOffAllRelays();
  Particle.publish("setup", "Relay 1 was " + String(relayState[0]) + ". Relay 2 was " + String(relayState[1]) + ". All relays turned off (0)", 60, PRIVATE);

}
void getRelayStatus(){
  for (size_t i = 0; i < NUMBER_OF_RELAYS; i++) {
    relayState[i] = relay.readRelayStatus(i+1);
  }
}
//#endregion

//#region - TCA9546 IC2 Multiplexer
void initMuxArray(){
  for (size_t i = 0; i < NUMBER_OF_MUX_CHANNELS; i++) {
    for (size_t j = 0; j < MAX_MUX_CHANNEL_DEVICES; j++) {
      ic2MuxArray[i][j]=false;
    }
  }

  for (size_t i = 0; i < 5; i++) {
    selectChannel(i); //set mux to all channels disabled
    scanIc2Bus(i);  //scan bus initially to find main devices not on the mux
    delay(2000);
  }
  msg = "Qty of I2C Soil Sensors:  " + String(numberOfSoilSensors());
  Particle.publish("setup", msg , 60, PRIVATE);
}
void scanIc2Bus(int channel)
    {

      byte error, address;
      Serial.println("Looking for i2C devices...");
      for(address = 1; address < 127; address++ )
      {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {

          Serial.print("I2C device found at address 0x");
          if (address<16)
            Serial.print("0");
            if (ic2MuxArray[0][address]==false) {
              ic2MuxArray[channel][address]=true;
              Particle.publish("I2C","Device on Ch: " + String(channel)+" - Addr: " + String(address,HEX),60,PRIVATE);

            }

          Serial.print(address,HEX);
          Serial.println("  !");

        }
        else if (error==4)
        {
          Serial.print("Unknow error at address 0x");
            Particle.publish("ERROR","Unknow error at address 0x: " + String(address),60,PRIVATE);
          if (address<16)
            Serial.print("0");

          Serial.println(address,HEX);
        }

    }
        Serial.println("done\n");

    }
void selectChannel(uint8_t channel){
  if( channel >= 0 && channel < 5 ) {
    Wire.beginTransmission(i2CMuxer);
    switch(channel) {
      case 0:
        Wire.write(0x00);
        break;
      case 1:
        Wire.write(0x01);
        break;
      case 2:
        Wire.write(0x02);
        break;
      case 3:
        Wire.write(0x04);
        break;
      case 4:
        Wire.write(0x08);
        break;


        Serial.print("Channel ");
        Serial.print(channel);
        Serial.print(" selected.");

    }
    Wire.endTransmission();
  } else {
    Serial.print("TCA9546A ERROR - Wrong channel selected: ");
    Serial.print(channel);
    Serial.println(" (available channels 0 (none),1,2,3,4)");
  }
}
//#endregion

//#region - Utility methods
void HandleError(String functionalArea){
     Particle.publish("ERROR: " + functionalArea, errorMessage, 60, PRIVATE);
     relay.turnOffAllRelays();
     error=false;
     delay(20000);
}
void PublishTentMeasurements(){

    String jsonSensorData = "{\"TentTemperature\":{{TentTemperature}},\"TentHumidity\":{{TentHumidity}},\"RelayOne\":{{RelayOne}},\"RelayTwo\":{{RelayTwo}}}";

    jsonSensorData.replace("{{TentTemperature}}",String(tentTempFahrenheit));
    jsonSensorData.replace("{{TentHumidity}}",String(tentHumidity));
    jsonSensorData.replace("{{RelayOne}}",String(relayState[0]));
    jsonSensorData.replace("{{RelayTwo}}",String(relayState[1]));


    if(tentHumidity>65.0) {ventTent=true;}
    if(tentHumidity<50.0) {ventTent=false;}
    Particle.publish("Vent",String(ventTent),60,PRIVATE);
    Particle.publish("Tent", jsonSensorData, 60, PRIVATE);
}
void PublishSoilMeasurements(){

    for (size_t i = 0; i < NUMBER_OF_SOIL_SENSORS; i++) {

      String jsonSensorData = "{\"SoilMoistureRaw\":{{SoilMoistureRaw}},\"SoilMoisture\":{{SoilMoisture}},\"SoilTemperature\":{{SoilTemperature}},\"Light\":{{Light}}}";
      jsonSensorData.replace("{{SoilMoistureRaw}}",soilSensorReadingArray[i][0]);
      jsonSensorData.replace("{{SoilMoisture}}",soilSensorReadingArray[i][1]);
      jsonSensorData.replace("{{SoilTemperature}}",soilSensorReadingArray[i][2]);
      jsonSensorData.replace("{{Light}}",soilSensorReadingArray[i][3]);

      String sensor = String(i+1);
      Particle.publish("SoilSensor-"+sensor, jsonSensorData, 60, PRIVATE);
    }
}
//#endregion
