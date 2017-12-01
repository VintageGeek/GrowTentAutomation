/*
 * Project GrowTentAutomation
 * Description: Automation of watering and temp/humidity sensor recording
 * Author: Mike Morales
 * Date: 27Nov2017
 */

// This #include statement was automatically added by the Particle IDE.
#include <NCD2Relay.h>
#include <I2CSoilMoistureSensor.h>

//constants
#define NUMBER_OF_SOIL_SENSORS 1
#define AddrHumidTemp 0x28
byte soilSensorArray[NUMBER_OF_SOIL_SENSORS] {0x30};
String soilSensorReadingArray[NUMBER_OF_SOIL_SENSORS][4];

//variables

I2CSoilMoistureSensor soilSensor(soilSensorArray[0]);//grab 1st soil sensor address
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

// setup() runs once, when the device is first turned on.
void setup()
{
  Particle.publish("setup", "Starting setup...", 60, PRIVATE);

  //Init Wire for I2C Communication
  Wire.setSpeed(400000);
    if (!Wire.isEnabled()) {
        Wire.begin();
    }

  initializeSoilMoistureSensors();
  initializeNCD2Relay();
  initializeHumiTemp();
  delay(1000);
  Particle.publish("setup", "Finished setup", 60, PRIVATE);
}
void initializeHumiTemp()
{
    // Start I2C transmission for module
  Wire.beginTransmission(AddrHumidTemp);
  // Send start command
  Wire.write(0x80);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(300);
  Particle.publish("setup", "Humidity & Temp Sensor Started");
}
void initializeNCD2Relay()
{
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
void initializeSoilMoistureSensors()
{
  //initiate the Soil Sensor object
      soilSensor.begin();
      delay(3000); // give some time to boot up

      msg = "I2C Soil Moisture Sensor begin() done.  Total sensors:  " + String(NUMBER_OF_SOIL_SENSORS);
      Particle.publish("setup", msg , 60, PRIVATE);
      String address="";
      String version = "";
      for (size_t i = 0; i < NUMBER_OF_SOIL_SENSORS; i++) {

         msg = "Validating I2C Soil Moisture Sensor "+ String(i+1) +" of "+ String(NUMBER_OF_SOIL_SENSORS) + ": ";

         if (i>0){
           soilSensor.changeSensor(soilSensorArray[i]);
         }
         address = String(soilSensor.getAddress(), HEX);
         version = String(soilSensor.getVersion());

        // publish sensors info
        Particle.publish("setup", msg + " Address: " + address + " - Firmware: " + version, 60, PRIVATE);
        delay(1000);
      }
}
void getHumiTempMeasures()
{
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
void getSoilSensorMeasures()
{
  for (size_t i = 0; i < NUMBER_OF_SOIL_SENSORS; i++) {
    if (i>0){
      soilSensor.changeSensor(soilSensorArray[i]);
    }
    GetSoilMoisture(&soilSensor, i);
    GetTemperature(&soilSensor, i);
    GetLight(&soilSensor, i);
  }
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
    int moisture = map(soilMoistureRaw,266,600,0,100);
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

/*



    //-----------------------------------------------
    error=false;
    errorMessage="";

    relay.setAddress(1,0,0);

    relayOneStatus=relay.readRelayStatus(1);
    relayTwoStatus=relay.readRelayStatus(2);

    //Process all sensors
        for (int sensorPumpNumber=1; sensorPumpNumber <= sensorPumpTotalNumber; sensorPumpNumber++)
        {

            //First we gather our sensor readings
            I2CSoilMoistureSensor* currentSensor = GetSensorReference(sensorPumpNumber);

           int moisture=GetSoilMoisture(currentSensor);
           if(error) HandleError("GetMoisture");

            float temp=GetTemperature(currentSensor);
            if(error) HandleError("GetTemperature");

           int light=0;//GetLight(currentSensor);
          // if(error) HandleError("GetLight");



            //Do we need humidity?

        if (!disableRelay1){
            switch (HumidityNeeded(humidity))
            {
                case 1:

                       relay.turnOnRelay(1);

                    break;
                case 0:

                       relay.turnOffRelay(1);

                    break;
            }

        }
            if(error) {
                HandleError("HumidifyTent");
                return;
            }

            //Do we need to water plant?  If so, do it.
            if (PlantNeedsWater(moisture))
            {
                //while (ContinueWatering(45)){
                    WaterPlant(sensorPumpNumber);
                    if(error) {
                        HandleError("WaterPlant");
                    return;
                    }
                //}
           }


    relayOneStatus=relay.readRelayStatus(1);
    relayTwoStatus=relay.readRelayStatus(2);
                   //Let's publish our readings
            PublishSoilMeasurements(sensorPumpNumber, moisture, temp, light, fTemp, humidity, relayOneStatus, relayTwoStatus );




        }


    delay(30000);
}

I2CSoilMoistureSensor* GetSensorReference(int sensorNumber){


    I2CSoilMoistureSensor *sensor;
    switch (sensorNumber)
    {
        case 1:
            sensor = &sensor1;
            break;
        case 2:
            sensor = &sensor2;
            break;
        default:
            error=true;
            errorMessage="Invalid sensorNumber passed: " + String(sensorNumber);

            // if nothing else matches, do the
            // default (which is optional)
    }



    return sensor;
}

bool IsRelayOn(int state)
{
    if (state==1)
        return true;
    else
        return false;
}



float GetTemperature(I2CSoilMoistureSensor *currentSensor){
    int temperature = currentSensor->getTemperature();
    if (!IsValidTemperature(temperature)){
        error=true;
        errorMessage="Temperature reading out of bounds: " + String(temperature);
        return 0.00;
    }
    float tempInF=(temperature/(float)10)*9/5+32;
    return tempInF;
}

int GetLight(I2CSoilMoistureSensor *currentSensor){
    int light = 0;// currentSensor->getLight(false);
    return light;
}



bool PlantNeedsWater(int moistureReading){
    return moistureReading <=15;}

bool ContinueWatering(int moistureReading){
    return moistureReading <75;}

int HumidityNeeded(double humidity){

    int returnVal=0;

    if (humidity>highHumidity) { humidityHitMax=true; }
    if (humidity<lowHumidity) { humidityHitMax=false; }

    if ((humidity <= highHumidity & !humidityHitMax) | (humidity < lowHumidity)){
        returnVal= 1;
    }

    return returnVal;

}

void WaterPlant(int pumpNumber){

     //relay.turnOnRelay(1);
        delay(1000);
       // relay.turnOffRelay(1);
}

void HumidifierAction(int turnOn, bool disableRelay1, NCD2Relay *relay){



}






void HandleError(String functionalArea){
     Particle.publish("ERROR: " + functionalArea, errorMessage, 60, PRIVATE);
     relay.turnOffAllRelays();
     delay(20000);
}
*/
}
