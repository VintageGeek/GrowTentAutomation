/*
 * Project GrowTentAutomation
 * Description: Automation of watering and temp/humidity sensor recording
 * Author: Mike Morales
 * Date: 27-Nov-2017
 */

// This #include statement was automatically added by the Particle IDE.
#include <NCD2Relay.h>
#include <I2CSoilMoistureSensor.h>

#define AddrHumidTemp 0x28
#define SoilSensor1Addr 0x30

I2CSoilMoistureSensor sensor1(SoilSensor1Addr);
I2CSoilMoistureSensor sensor2;
NCD2Relay relay;


double cTemp = 0.0, fTemp = 0.0, humidity = 0.0;
int soilMoistureRaw=0;

bool error;
String errorMessage;
int sensorPumpTotalNumber;
int relayOneStatus;
int relayTwoStatus;
bool disableRelay1;
bool disableRelay2;
bool success;
bool humidityHitMax=true;

int lowHumidity=64;
int highHumidity = 67;

// setup() runs once, when the device is first turned on.
void setup() {
    sensorPumpTotalNumber=1;
    relayOneStatus=0;
    relayTwoStatus=0;
    disableRelay1=false;
    disableRelay2=false;
    success = false;
    humidityHitMax=false;



    relay.setAddress(1,0,0);

    Wire.setSpeed(400000);
    if (!Wire.isEnabled()) {
        Wire.begin();
    }

    Particle.publish("setup", "Start setup", 60, PRIVATE);
    sensor1.begin(); // reset sensor
    Particle.publish("setup", "sensor1.begin() done", 60, PRIVATE);
    delay(2000); // give some time to boot up


    String address = String(sensor1.getAddress(), HEX);
    String version = String(sensor1.getVersion(), HEX);

    // publish sensors info
    Particle.publish("setup", "I2C Soil Moisture Sensor Address: " + address, 60, PRIVATE);
    Particle.publish("setup", "Sensor Firmware version: " + version, 60, PRIVATE);

    Particle.variable("Humidifier", relayOneStatus);
    Particle.variable("WaterPump", relayTwoStatus);

    int status1=relay.readRelayStatus(1);
    String state1="OFF";
    if (status1==1){state1="ON";}
     int status2=relay.readRelayStatus(2);
    String state2="OFF";
    if (status2==1){state2="ON";}

    Particle.publish("setup", "Relay 1 is " + state1);
    Particle.publish("setup", "Relay 2 is " + state2);
    relay.turnOffAllRelays();

      // Start I2C transmission
  Wire.beginTransmission(AddrHumidTemp);
  // Send start command
  Wire.write(0x80);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(300);
    Particle.publish("setup", "Humidity & Temp Sensor Started");
}

void loop() {

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

      humidity -= 3.0; // adjust to match external monitor
      fTemp -= .3;

      }


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

int GetSoilMoisture(I2CSoilMoistureSensor *currentSensor){
    while (currentSensor->isBusy()) delay(50); // available since FW 2.3
    soilMoistureRaw = currentSensor->getCapacitance();
    if (!IsValidSoilMoisture(soilMoistureRaw)){
        error=true;
        errorMessage="Soil moisture reading out of bounds: " + String(soilMoistureRaw);
        return -1;
    }
    int moisture = map(soilMoistureRaw,266,600,0,100);
    return moisture;
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

bool IsValidSoilMoisture(int moistureReading){
    return ((moistureReading > 200) & (moistureReading <700));
}

bool IsValidTemperature(int temperatureReading){
    return ((temperatureReading>0) & (temperatureReading <500));
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




void PublishSoilMeasurements(int sensorNumber, int soilMoisture, float soilTemperature, int light, double tentTemperature, double tentHumidity, int relay1, int relay2){

    String jsonSensorData = "{\"SensorNumber\":{{SensorNumber}},\"SoilMoistureRaw\":{{SoilMoistureRaw}},\"SoilMoisture\":{{SoilMoisture}},\"SoilTemperature\":{{SoilTemperature}},\"Light\":{{Light}},\"TentTemperature\":{{TentTemperature}},\"TentHumidity\":{{TentHumidity}},\"RelayOne\":{{RelayOne}},\"RelayTwo\":{{RelayTwo}}}";
    jsonSensorData.replace("{{SensorNumber}}",String(sensorNumber));
    jsonSensorData.replace("{{SoilMoisture}}",String(soilMoisture));
    jsonSensorData.replace("{{SoilMoistureRaw}}",String(soilMoistureRaw));
    jsonSensorData.replace("{{SoilTemperature}}",String(soilTemperature));
    jsonSensorData.replace("{{Light}}",String(light));
    jsonSensorData.replace("{{TentTemperature}}",String(tentTemperature));
    jsonSensorData.replace("{{TentHumidity}}",String(tentHumidity));
    jsonSensorData.replace("{{RelayOne}}",String(relay1));
    jsonSensorData.replace("{{RelayTwo}}",String(relay2));

    String sensor = String(sensorNumber);
    Particle.publish("GrowData-"+sensor, jsonSensorData, 60, PRIVATE);
   // Particle.publish("SoilMoisture-"+sensor, String(moisture), 60, PRIVATE);
    //delay(20000);
    //Particle.publish("SoilTemperature-"+sensor, String(temperature), 60, PRIVATE);
    //delay(20000);
    //Particle.publish("SoilLight-"+sensor, String(light), 60, PRIVATE);
}

void HandleError(String functionalArea){
     Particle.publish("ERROR: " + functionalArea, errorMessage, 60, PRIVATE);
     relay.turnOffAllRelays();
     delay(20000);
}
