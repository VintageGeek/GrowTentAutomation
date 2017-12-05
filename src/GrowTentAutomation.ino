/*
 * Project GrowTentAutomation
 * Description: Automation of watering and temp/humidity sensor recording
 * Author: Mike Morales
 * Date: 27Nov2017
 */

//#region Global Variables
// This #include statement was automatically added by the Particle IDE.
#include <NCD2Relay.h>
#include <I2CSoilMoistureSensor.h>

//constants
#define NUMBER_OF_SOIL_SENSORS 2
#define AddrHumidTemp 0x28
#define i2CMuxer 0x74
#define NUMBER_OF_MUX_CHANNELS 5
#define MAX_MUX_CHANNEL_DEVICES 127
byte soilSensorArray[NUMBER_OF_SOIL_SENSORS] {0x30,0x31};
String soilSensorReadingArray[NUMBER_OF_SOIL_SENSORS][4];

bool addrFoundArray[127];
bool ic2MuxArray[NUMBER_OF_MUX_CHANNELS][MAX_MUX_CHANNEL_DEVICES] ;//Support 8 devices per mux channel
bool muxScanned = false;


//variables
NCD2Relay relay;
double cTemp = 0.0, fTemp = 0.0, humidity = 0.0;
bool error;
String errorMessage;
int sensorPumpTotalNumber=1;
int relayOneStatus=0;
int relayTwoStatus=0;
bool disableRelay1=false;
bool disableRelay2=false;
bool success=false;
bool humidityHitMax=false;
int lowHumidity=64;
int highHumidity = 67;
bool needToWater;
String msg = "";
//#endregion

//#region setup() runs once, when the device is first turned on.
void setup(){
  Particle.publish("setup", "Starting setup...", 60, PRIVATE);

  //Init Wire for I2C Communication
  Wire.setSpeed(400000);
    if (!Wire.isEnabled()) {
        Wire.begin();
    }

  initMuxArray();

  for (size_t i = 0; i < 5; i++) {
    selectChannel(i); //set mux to all channels disabled
    scanIc2Bus(i);  //scan bus initially to find main devices not on the mux
    delay(2000);
  }

  msg = "Qty of I2C Soil Sensors:  " + String(numberOfSoilSensors());
  Particle.publish("setup", msg , 60, PRIVATE);

  initializeSoilMoistureSensors();
  initializeNCD2Relay();
  initializeHumiTemp();
  delay(1000);
  Particle.publish("setup", "Finished setup", 60, PRIVATE);
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
void initMuxArray(){
  for (size_t i = 0; i < NUMBER_OF_MUX_CHANNELS; i++) {
    for (size_t j = 0; j < MAX_MUX_CHANNEL_DEVICES; j++) {
      ic2MuxArray[i][j]=false;
    }
  }
}
void initializeSoilMoistureSensors(){
      //figure out how many devices wehave in the mux




      String address="";
      String version = "";

      for (size_t i = 1; i <= NUMBER_OF_SOIL_SENSORS; i++) {
         selectChannel(i);
         I2CSoilMoistureSensor soilSensor(soilSensorArray[i-1]);
         soilSensor.begin();
         delay(3000); // give some time to boot up

         msg = "I2C Soil Sensor "+ String(i) +" of "+ String(NUMBER_OF_SOIL_SENSORS) + ": ";

         //if (i>1){
           //soilSensor.changeSensor(soilSensorArray[i-1]);
        // }
         address = String(soilSensor.getAddress(), HEX);
         version = String(soilSensor.getVersion());

        // publish sensors info
        Particle.publish("setup", msg + " Ch: " + i + " - Addr: " + address + " - FW: " + version, 60, PRIVATE);
        delay(1000);
      }
      selectChannel(0);
}
void initializeNCD2Relay(){
  //initiate the NCD2Relay
  relay.setAddress(1,0,0);
  int status1=relay.readRelayStatus(1);
  String state1="OFF";
  if (status1==1){state1="ON";}
   int status2=relay.readRelayStatus(2);
  String state2="OFF";
  if (status2==1){state2="ON";}
  Particle.variable("Humidifier", relayOneStatus);
  Particle.variable("WaterPump", relayTwoStatus);
  relay.turnOffAllRelays();
  Particle.publish("setup", "Relay 1 was " + state1 + ". Relay 2 was " + state2 + ". All relays turned off", 60, PRIVATE);
}
void initializeHumiTemp(){
    // Start I2C transmission for module
  Wire.beginTransmission(AddrHumidTemp);
  // Send start command
  Wire.write(0x80);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(300);
  Particle.publish("setup", "Humidity & Temp Sensor Started");
}


//#endregion

void loop() {
  error=false;
  errorMessage="";

  getHumiTempMeasures();
  getSoilSensorMeasures();

  relayOneStatus=relay.readRelayStatus(1);
  relayTwoStatus=relay.readRelayStatus(2);
  PublishTentMeasurements(fTemp, humidity, relayOneStatus, relayTwoStatus );
  PublishSoilMeasurements();
  delay(30000);

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
              Particle.publish("sensors","Found on Ch: " + String(channel)+" - Addr: " + String(address,HEX),60,PRIVATE);

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
void getHumiTempMeasures(){
  unsigned int data[4];

  // Start I2C transmission
  Wire.beginTransmission(AddrHumidTemp);
  // Stop I2C transmission
  Wire.endTransmission();
  // Request 4 byte of data
  Wire.requestFrom(AddrHumidTemp, 4);
  // Read 4 bytes of data
  // humidity msb, humidity lsb, cTemp msb, cTemp lsb
  if (Wire.available() == 4)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
  // Convert the data to 14-bits
  humidity = (((data[0] & 0x3F) * 256) + data[1]) / 16384.0 * 100.0;
  cTemp = (((data[2] * 256) + (data[3] & 0xFC)) / 4) / 16384.0 * 165.0 - 40.0;
  fTemp = (cTemp * 1.8) + 32;
  //adjust temp and humidity based on calibration of 2 other thermometers/humidity sensors
  fTemp+=1.3;
  humidity-=6.4;

  }
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
    GetTemperature(&soilSensor, i);
    GetLight(&soilSensor, i);
  }
  selectChannel(0);
}
void GetSoilMoisture(I2CSoilMoistureSensor *currentSensor, int sensorNumber){
    while (currentSensor->isBusy()) delay(50); // available since FW 2.3
    int soilMoistureRaw = currentSensor->getCapacitance();
    if (!IsValidSoilMoisture(soilMoistureRaw)){
        errorMessage="Soil moisture reading out of bounds: " + String(soilMoistureRaw);
        soilSensorReadingArray[sensorNumber][0]="ERROR";
        soilSensorReadingArray[sensorNumber][1]="ERROR";
        HandleError("GetMoisture");
        return ;
    }
    int moisture = map(soilMoistureRaw,277,600,0,100);
    soilSensorReadingArray[sensorNumber][0]=String(soilMoistureRaw);
    soilSensorReadingArray[sensorNumber][1]=String(moisture);
}
void GetTemperature(I2CSoilMoistureSensor *currentSensor, int sensorNumber){
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
void GetLight(I2CSoilMoistureSensor *currentSensor, int sensorNumber){
    int light = currentSensor->getLight(true);
    soilSensorReadingArray[sensorNumber][3]=String(light);
}
bool IsValidSoilMoisture(int moistureReading){
    return ((moistureReading > 200) & (moistureReading <700));
}
bool IsValidTemperature(int temperatureReading){
    return ((temperatureReading>0) & (temperatureReading <500));
}
void HandleError(String functionalArea){
     Particle.publish("ERROR: " + functionalArea, errorMessage, 60, PRIVATE);
     relay.turnOffAllRelays();
     error=false;
     delay(20000);
}
void PublishTentMeasurements(double tentTemperature, double tentHumidity, int relay1, int relay2){

    String jsonSensorData = "{\"TentTemperature\":{{TentTemperature}},\"TentHumidity\":{{TentHumidity}},\"RelayOne\":{{RelayOne}},\"RelayTwo\":{{RelayTwo}}}";

    jsonSensorData.replace("{{TentTemperature}}",String(tentTemperature));
    jsonSensorData.replace("{{TentHumidity}}",String(tentHumidity));
    jsonSensorData.replace("{{RelayOne}}",String(relay1));
    jsonSensorData.replace("{{RelayTwo}}",String(relay2));

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
void WaterPlant(int pumpNumber){

     //relay.turnOnRelay(1);
        delay(1000);
       // relay.turnOffRelay(1);
}
int HumidityNeeded(double humidity){

    int returnVal=0;

    if (humidity>highHumidity) { humidityHitMax=true; }
    if (humidity<lowHumidity) { humidityHitMax=false; }

    if ((humidity <= highHumidity & !humidityHitMax) | (humidity < lowHumidity)){
        returnVal= 1;
    }

    return returnVal;

}
bool PlantNeedsWater(int moistureReading){
    return moistureReading <=15;
  }
