

// Debug Section Turn off DEBUG for prodution to save battery
#define DEBUG  // Controls print statements. Comment out for production
#define DEBUG_BAT  // Controls LED blinks for battery operation debug. Comment out for production
//#define MY_DEBUG  //Controls logging of protocol messages. Comment out for production
//#define MY_DEBUG_VERBOSE_RFM69 //Controls logging  of radio messages. Comment out for production

// Capabilities Section 
//#define LOWPOWER_SLEEP // enables lowpower sleep. Don't use if you need to listen for radio messages
#define ENABLE_MYSENSORS // Enables MySensors radio communication with the gateway
#define ENABLE_BME280 // Enables BME280 sensor [not complete, must keep enabled]
#define ENABLE_POWER_MON // Power and Solar monitoring
#define ENABLE_WX_RACK // Wind speed and rain sensors

#ifdef ENABLE_MYSENSORS
  // Configure RFM69 Radio
  #define MY_RADIO_RFM69
  #define MY_RFM69_NEW_DRIVER
  #define MY_IS_RFM69HW
  #define MY_RFM69_FREQUENCY RFM69_433MHZ
  //#define MY_ENCRYPTION_SIMPLE_PASSWD "some16characters"

  // Configure hardware pins
  #define MY_RFM69_IRQ_PIN 9
  #define MY_RFM69_IRQ_NUM 9
  #define MY_RFM69_CS_PIN A2

  // Configure MySensors Node and its children
  // Parent is always my gateway.  No need to search
  #define MY_PARENT_NODE_ID 0
  #define MY_PARENT_NODE_IS_STATIC
  #define MY_NODE_ID 72 // Manually assigned node ID for this device.  AUTO not supported if using MQTT
  #define W_DIRECTION_CHILD 1
  #define WIND_CHILD 2
  #define W_GUST_CHILD 3
  // #define W_DIRECTION_2M_CHILD 4
  // #define WIND_2M_CHILD 5
  // #define W_GUST_2M_CHILD 6
  // #define W_DIRECTION_10M_CHILD 7
  // #define WIND_10M_CHILD 8
  // #define W_GUST_10M_CHILD 9
  #define BARO_CHILD 10
  #define TEMP_CHILD 11
  #define HUM_CHILD 12
  #define VOLT_CHILD 13
  #define LIPO_BUS_VOLT_CHILD 14
  #define LIPO_SHUNT_VOLT_CHILD 15
  #define LIPO_LOAD_VOLT_CHILD 16
  #define LIPO_CURR_CHILD 17
  #define LIPO_CHARGING_CHILD 18
  #define SOLAR_BUS_VOLT_CHILD 19
  #define SOLAR_SHUNT_VOLT_CHILD 20
  #define SOLAR_LOAD_VOLT_CHILD 21
  #define SOLAR_CURR_CHILD 22
  #define SOLAR_ACTIVE_CHILD 23
  #define OUTPUT_BUS_VOLT_CHILD 24
  #define OUTPUT_SHUNT_VOLT_CHILD 25
  #define OUTPUT_LOAD_VOLT_CHILD 26
  #define OUTPUT_CURR_CHILD 27
  #define OUTPUT_ACTIVE_CHILD 28
  #define RAIN_CHILD 29
  #define RAINDAILY_CHILD 30
#endif

#define BATTERY_PIN A5
#define LED_PIN LED_BUILTIN

#ifdef ENABLE_WX_RACK
  #include "SDL_Weather_80422.h"
  //For SDL_Weather_80422 library
  #define pinLED     13   // LED connected to digital pin 13
  #define pinAnem    0  // Anenometer connected to pin 18 - Int 5 - Mega   / Uno pin 2
  #define pinRain    1  // Anenometer connected to pin 2 - Int 0 - Mega   / Uno Pin 3 
  #define intAnem    0  // int 0 (check for Uno)
  #define intRain    1  // int 1
#endif


// All defines must be before any library inclusions (except for SDL_Weather_80422.h)

#include <SPI.h>
#ifdef ENABLE_MYSENSORS
  #include <MySensors.h>
#endif
#include <Wire.h>
#ifdef ENABLE_BME280
  #include <SparkFunBME280.h>
#endif
#ifdef LOWPOWER_SLEEP
  #include <ArduinoLowPower.h>
#endif

#ifdef ENABLE_POWER_MON
  // Battery and Solar Cell management
  #include "SDL_Arduino_INA3221.h"
#endif

#include <Time.h>
#include "elapsedMillis.h"

// Define variables to use later
bool initialValueSent = false;
volatile bool awakeFlag = false; 
#ifdef DEBUG_BAT
  unsigned long sleepTime = 5000; // Update every 5 seconds
#else
  unsigned long sleepTime = 300000; // Update every 5 minutes
#endif

float lastTemperature = 0;
float lastPressure = 0;
float lastHumidity = 0;
float lastBattery = 0;

float lastBusvoltage1 = 0;
float lastShuntvoltage1 = 0;
float lastLoadvoltage1 = 0;
float lastCurrent_mA1 = 0;
bool lastLipoCharging = 0;

float lastBusvoltage2 = 0;
float lastShuntvoltage2 = 0;
float lastLoadvoltage2 = 0;
float lastCurrent_mA2 = 0;
bool lastSolarActive = 0;

float lastBusvoltage3 = 0;
float lastShuntvoltage3 = 0;
float lastLoadvoltage3 = 0;
float lastCurrent_mA3 = 0;
bool lastOutputActive = 0;

#ifdef ENABLE_MYSENSORS
  // Only use one variable type per child
  // Customize HA to display icon and units
  MyMessage tempMsg(TEMP_CHILD, V_TEMP);
  MyMessage humMsg(HUM_CHILD, V_HUM);
  MyMessage pressureMsg(BARO_CHILD, V_PRESSURE);
  MyMessage voltageMsg(VOLT_CHILD, V_VOLTAGE);
  MyMessage lipoBusVoltMsg(LIPO_BUS_VOLT_CHILD, V_VOLTAGE); // S_MULTIMETER
  MyMessage lipoShuntVoltMsg(LIPO_SHUNT_VOLT_CHILD, V_VOLTAGE);
  MyMessage lipoLoadVoltMsg(LIPO_LOAD_VOLT_CHILD, V_VOLTAGE);
  MyMessage lipoCurrentMsg(LIPO_CURR_CHILD, V_CURRENT); // S_MULTIMETER
  MyMessage lipoChargingMsg(LIPO_CHARGING_CHILD, V_STATUS); // S_BINARY
  MyMessage solarBusVoltMsg(SOLAR_BUS_VOLT_CHILD, V_VOLTAGE);
  MyMessage solarShuntVoltMsg(SOLAR_SHUNT_VOLT_CHILD, V_VOLTAGE);
  MyMessage solarLoadVoltMsg(SOLAR_LOAD_VOLT_CHILD, V_VOLTAGE);
  MyMessage solarCurrentMsg(SOLAR_CURR_CHILD, V_CURRENT);
  MyMessage solarActivegMsg(SOLAR_ACTIVE_CHILD, V_STATUS); // S_BINARY
  MyMessage outputBusVoltMsg(OUTPUT_BUS_VOLT_CHILD, V_VOLTAGE);
  MyMessage outputShuntVoltMsg(OUTPUT_SHUNT_VOLT_CHILD, V_VOLTAGE);
  MyMessage outputLoadVoltMsg(OUTPUT_LOAD_VOLT_CHILD, V_VOLTAGE);
  MyMessage outputCurrentMsg(OUTPUT_CURR_CHILD, V_CURRENT);
  MyMessage outputActiveMsg(OUTPUT_ACTIVE_CHILD, V_STATUS); // S_BINARY
  MyMessage winddDirMsg(W_DIRECTION_CHILD, V_DIRECTION);
  MyMessage windMsg(WIND_CHILD, V_WIND);
  MyMessage WindGustMsg(W_GUST_CHILD, V_GUST);
  MyMessage rainMsg(RAIN_CHILD, V_RAIN);
  MyMessage rainDailyMsg(RAINDAILY_CHILD, V_RAIN);
#endif

#ifdef ENABLE_BME280
  BME280 myBME280; //Global sensor object
#endif

  elapsedMillis timeElapsed; //declare global if you don't want it reset every time loop runs
  elapsedMillis rainTime; //declare global if you don't want it reset every time loop runs
  elapsedMillis rainTimeElapsed; //declare global if you don't want it reset every time loop runs

#ifdef ENABLE_WX_RACK
  // Example: SDL_Weather_80422(int pinAnem, int pinRain, int intAnem, int intRain, int ADChannel, int ADMode);
  SDL_Weather_80422 weatherStation(pinAnem, pinRain, intAnem, intRain, A0, SDL_MODE_INTERNAL_AD);
  float currentWindSpeed;
  float currentWindGust;
  float rainTotal;
  bool rainingNow;

#endif

#ifdef ENABLE_POWER_MON
  // Battery and Solar Cell management
  SDL_Arduino_INA3221 ina3221;
  // the three channels of the INA3221 named for SunAirPlus Solar Power Controller channels (www.switchdoc.com)
  #define LIPO_BATTERY_CHANNEL 1
  #define SOLAR_CELL_CHANNEL 2
  #define OUTPUT_CHANNEL 3
  bool lipoCharging;
  bool solarActive;
  bool outputActive;
#endif

//replace Serial with SerialUSB
#if defined (MOTEINO_M0)
  #if defined(SERIAL_PORT_USBVIRTUAL)
    #define Serial SERIAL_PORT_USBVIRTUAL // Required for Serial on Zero based boards
  #endif
#endif

void setup()
{
  /*
    Used for RFM69 CS (NSS)
    I don't think that the MySensors.h anticipates using analog
    input as an output.
    The radio does not work without the following pinMode statement.
  */
  pinMode(A2, OUTPUT);  //Used for RFM69 CS (NSS)
  //Setup ADC. Arduino default is 10 bits
  //analogReadResolution(12);

  #ifdef LOWPOWER_SLEEP
    LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, awake, CHANGE);
  #endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  //SerialUSB does not work when sleeping the Moteino M0 board.
  // Blinking LED was used in debug mode.
#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("In setup function.");
#endif
  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!

#ifdef ENABLE_BME280
  myBME280.beginI2C();
  // Use same settings as Micro Python version
  myBME280.settings.filter = 3;
  myBME280.settings.tempOverSample = 8;
  myBME280.settings.pressOverSample = 4;
  myBME280.settings.humidOverSample = 2;

  myBME280.setMode(MODE_SLEEP); //Sleep for now
#endif

#ifdef ENABLE_WX_RACK
  weatherStation.setWindMode(SDL_MODE_SAMPLE, 5.0);
  //weatherStation.setWindMode(SDL_MODE_DELAY, 5.0);
  rainTotal = 0.0;
  timeElapsed = 0;
  rainTimeElapsed = 0;
  rainingNow = 0;
  
#endif

#ifdef ENABLE_POWER_MON
  // Battery and Solar Cell management
  #ifdef DEBUG
    Serial.println("SDA_Arduino_INA3221_Test");
    Serial.println("Measuring voltage and current with ina3221 ...");
    
  #endif

  ina3221.begin();  

  #ifdef DEBUG
    Serial.print("Manufactures ID=0x");
    int MID;
    MID = ina3221.getManufID();
    Serial.println(MID,HEX);
  #endif
#endif
}

#ifdef ENABLE_MYSENSORS
  void presentation()
  {
  #ifdef DEBUG
    Serial.println("Sending Presentation Info");
  #endif
    sendSketchInfo("test72", "1.0");   //Max 25 characters.  Optional to do.
    // Use one child per measurement for consistent HA entity naming.
    present(TEMP_CHILD, S_TEMP);
    present(HUM_CHILD, S_HUM);
    present(BARO_CHILD, S_BARO);
    present(VOLT_CHILD, S_MULTIMETER);
    present(LIPO_BUS_VOLT_CHILD, S_MULTIMETER);
    present(LIPO_SHUNT_VOLT_CHILD, S_MULTIMETER);
    present(LIPO_LOAD_VOLT_CHILD, S_MULTIMETER);
    present(LIPO_CURR_CHILD, S_MULTIMETER);
    present(LIPO_CHARGING_CHILD, S_BINARY);
    present(SOLAR_BUS_VOLT_CHILD, S_MULTIMETER);
    present(SOLAR_SHUNT_VOLT_CHILD, S_MULTIMETER);
    present(SOLAR_LOAD_VOLT_CHILD, S_MULTIMETER);
    present(SOLAR_CURR_CHILD, S_MULTIMETER);
    present(SOLAR_ACTIVE_CHILD, S_BINARY);
    present(OUTPUT_BUS_VOLT_CHILD, S_MULTIMETER);
    present(OUTPUT_SHUNT_VOLT_CHILD, S_MULTIMETER);
    present(OUTPUT_LOAD_VOLT_CHILD, S_MULTIMETER);
    present(OUTPUT_CURR_CHILD, S_MULTIMETER);
    present(OUTPUT_ACTIVE_CHILD, S_BINARY);
    present(W_DIRECTION_CHILD, S_WIND, "Wind Direction " );
    present(WIND_CHILD, S_WIND, "Wind Speed " );
    present(W_GUST_CHILD, S_WIND, "Wind Gust " );
    present(RAIN_CHILD, S_RAIN, "Rain in Inches " );
    present(RAINDAILY_CHILD, S_RAIN, "Daily Rain Inches " );
  }
#endif

void loop()
{
  #ifdef ENABLE_WX_RACK
    currentWindSpeed = weatherStation.current_wind_speed()/1.6;
  #endif

  if (timeElapsed > 1000)
  {

    #ifdef ENABLE_BME280
      myBME280.setMode(MODE_FORCED); //Wake up sensor and take reading
      #ifdef DEBUG
      long startTime = millis();
      #endif
      while (myBME280.isMeasuring() == false) ; //Wait for sensor to start measurment
      while (myBME280.isMeasuring() == true) ; //Hang out while sensor completes the reading
      #ifdef DEBUG
      long endTime = millis();

      //Sensor is back from sleep so we can now get the data

      Serial.print(" Measure time(ms): ");
      Serial.println(endTime - startTime);
      #endif
      float temperature = myBME280.readTempC();
      float humidity = myBME280.readFloatHumidity();
      float pressure = myBME280.readFloatPressure() / 100.0;
      float battery = analogRead(BATTERY_PIN);
    #endif

    #ifdef ENABLE_WX_RACK
      currentWindSpeed = weatherStation.current_wind_speed()/1.6;
      currentWindGust = weatherStation.get_wind_gust()/1.6;
      
      float oldRain = rainTotal;
      rainTotal = rainTotal + weatherStation.get_current_rain_total() * 0.03937;
      if (oldRain < rainTotal)
      {
        rainingNow = 1;
        #ifdef DEBUG
        Serial.println( F("It is Raining"));
        #endif   
      } else if (rainTimeElapsed > 300000)
        {
          rainingNow = 0;
          rainTimeElapsed = 0;
        }

      timeElapsed = 0;       // reset the counter to 0 so the counting starts over...
      
      #ifdef DEBUG
      Serial.print(F(" currentWindSpeed:          ")); Serial.println(currentWindSpeed);
      Serial.print(F(" currentWindGust:           ")); Serial.println(currentWindGust);
      Serial.print(F(" Wind Direction:            ")); Serial.println(weatherStation.current_wind_direction());
      Serial.print(F(" Cumulative Rain:           ")); Serial.println(rainTotal);
      Serial.print(F(" Raining Now:               ")); Serial.println(rainingNow);
      Serial.println("");
      #endif
    
    
    #endif

    //Reads approx 620 per volt, 4095/3.3/2
    battery = battery / 625.0; //My calibration
    



    #ifdef ENABLE_POWER_MON
    // Battery and Solar Cell management
    #ifdef DEBUG
    Serial.println("------------------------------");
    #endif

    float shuntvoltage1 = 0;
    float busvoltage1 = 0;
    float current_mA1 = 0;
    float loadvoltage1 = 0;
    bool  lipoCharging = 0;

    busvoltage1 = ina3221.getBusVoltage_V(LIPO_BATTERY_CHANNEL);
    shuntvoltage1 = ina3221.getShuntVoltage_mV(LIPO_BATTERY_CHANNEL);
    current_mA1 = -ina3221.getCurrent_mA(LIPO_BATTERY_CHANNEL);  // minus is to get the "sense" right.   - means the battery is charging, + that it is discharging
    loadvoltage1 = busvoltage1 + (shuntvoltage1 / 1000);
    if (current_mA1 < 0)
      {
        lipoCharging = 1;
      } else
      {
        lipoCharging = 0;
      }
      
    
    #ifdef DEBUG
    Serial.print("LIPO_Battery Bus Voltage:   "); Serial.print(busvoltage1); Serial.println(" V");
    Serial.print("LIPO_Battery Shunt Voltage: "); Serial.print(shuntvoltage1); Serial.println(" mV");
    Serial.print("LIPO_Battery Load Voltage:  "); Serial.print(loadvoltage1); Serial.println(" V");
    Serial.print("LIPO_Battery Current 1:     "); Serial.print(current_mA1); Serial.println(" mA");
    Serial.print("LIPO Charging:              "); Serial.println(lipoCharging); 
    Serial.println("");
    #endif

    float shuntvoltage2 = 0;
    float busvoltage2 = 0;
    float current_mA2 = 0;
    float loadvoltage2 = 0;
    bool  solarActive =0;

    
    busvoltage2 = ina3221.getBusVoltage_V(SOLAR_CELL_CHANNEL);
    shuntvoltage2 = ina3221.getShuntVoltage_mV(SOLAR_CELL_CHANNEL);
    current_mA2 = -ina3221.getCurrent_mA(SOLAR_CELL_CHANNEL);
    loadvoltage2 = busvoltage2 + (shuntvoltage2 / 1000);
    if (current_mA2 < 0)
    {
    solarActive = 1;
    } else
    {
      solarActive = 0;
    }
    
    #ifdef DEBUG
    Serial.print("Solar Cell Bus Voltage 2:   "); Serial.print(busvoltage2); Serial.println(" V");
    Serial.print("Solar Cell Shunt Voltage 2: "); Serial.print(shuntvoltage2); Serial.println(" mV");
    Serial.print("Solar Cell Load Voltage 2:  "); Serial.print(loadvoltage2); Serial.println(" V");
    Serial.print("Solar Cell Current 2:       "); Serial.print(current_mA2); Serial.println(" mA");
    Serial.print("Solar Active:               "); Serial.println(solarActive);
    Serial.println("");
    #endif

    float shuntvoltage3 = 0;
    float busvoltage3 = 0;
    float current_mA3 = 0;
    float loadvoltage3 = 0;
    bool  outputActive = 0;

    busvoltage3 = ina3221.getBusVoltage_V(OUTPUT_CHANNEL);
    shuntvoltage3 = ina3221.getShuntVoltage_mV(OUTPUT_CHANNEL);
    current_mA3 = ina3221.getCurrent_mA(OUTPUT_CHANNEL);
    loadvoltage3 = busvoltage3 + (shuntvoltage3 / 1000);
    if (current_mA3 < 0)
    {
    outputActive = 1;
    } else
    {
      outputActive = 0;
    }

    #ifdef DEBUG 
    Serial.print("Output Bus Voltage 3:      "); Serial.print(busvoltage3); Serial.println(" V");
    Serial.print("Output Shunt Voltage 3:    "); Serial.print(shuntvoltage3); Serial.println(" mV");
    Serial.print("Output Load Voltage 3:     "); Serial.print(loadvoltage3); Serial.println(" V");
    Serial.print("Output Current 3:          "); Serial.print(current_mA3); Serial.println(" mA");
    Serial.print("Output Active:             "); Serial.println(outputActive); 
    Serial.println("");
    #endif
  #endif

  #ifdef DEBUG
    #ifdef ENABLE_BME280
      Serial.print(" Temp: ");
      Serial.print(temperature);
      Serial.print(" Humidity: ");
      Serial.print(humidity);
      Serial.print(" Pressure: ");
      Serial.print(pressure);
    #endif
    Serial.print(" Battery Voltage ");
    Serial.println(battery);

    #ifdef ENABLE_WX_RACK
      Serial.print("rain_total=");
      Serial.print(rainTotal);
      Serial.print(""" wind_speed=");
      Serial.print(currentWindSpeed);
      Serial.print("MPH wind_gust=");
      Serial.print(currentWindGust);
      Serial.print("MPH wind_direction=");
      Serial.println(weatherStation.current_wind_direction());
    #endif

  #endif
    // Only attempt to send if link is up else sleep again.
    #ifdef ENABLE_MYSENSORS
    if (transportCheckUplink(true))
    {
      #ifdef DEBUG
        Serial.println("Uplink OK ");
        send(tempMsg.set(temperature, 1));
      #endif
      #ifdef DEBUG_BAT
        #ifdef ENABLE_BME280
          send(tempMsg.set(temperature, 1));
          send(pressureMsg.set(pressure, 1));
          send(lipoBusVoltMsg.set(busvoltage1, 2));
          send(lipoShuntVoltMsg.set(shuntvoltage1, 1));
          send(lipoLoadVoltMsg.set(loadvoltage1, 1));
          send(lipoCurrentMsg.set(current_mA1, 1));
          send(lipoChargingMsg.set(lipoCharging, 1));
          send(solarBusVoltMsg.set(busvoltage2, 1));
          send(solarShuntVoltMsg.set(shuntvoltage2, 1));
          send(solarLoadVoltMsg.set(loadvoltage2, 1));
          send(solarCurrentMsg.set(current_mA2, 1));
          send(solarActivegMsg.set(solarActive,  1));
          send(outputBusVoltMsg.set(busvoltage3, 1));
          send(outputShuntVoltMsg.set(shuntvoltage3, 1));
          send(outputLoadVoltMsg.set(loadvoltage3, 1));
          send(outputCurrentMsg.set(current_mA3, 1));
          send(outputActiveMsg.set(outputActive, 1));
          send(winddDirMsg.set(shuntvoltage3, 1));
          send(windMsg.set(currentWindSpeed, 1));
          send(WindGustMsg.set(currentWindGust, 1));
          send(rainMsg.set(rainTotal, 2)); // needs to be configured for dail total
          send(rainDailyMsg.set(rainTotal, 2)); // needs to be configured for dail total
        #endif
        send(voltageMsg.set(battery, 2));
      #endif   
      // To Save power only send message on changed values
      #ifdef ENABLE_BME280
        if (abs(temperature - lastTemperature) > 0.1)
        {
        send(tempMsg.set(temperature, 1));
        lastTemperature = temperature;
        }
        if (abs(humidity - lastHumidity) > 1.0)
        {
        send(humMsg.set(humidity, 2));
        lastHumidity = humidity;
        }

        if (abs(pressure - lastPressure) > 1.0)
        {
        send(pressureMsg.set(pressure, 1));
        lastPressure = pressure;
        }
        if (abs(battery - lastBattery) > 0.1)
        {
        send(voltageMsg.set(battery, 2));
        lastBattery = battery;
        }
      #endif

      #ifdef ENABLE_POWER_MON
      // LIPO Channel
        if (abs(busvoltage1 - lastBusvoltage1) > 0.1)
        {
        send(lipoBusVoltMsg.set(busvoltage1, 1));
        lastBusvoltage1 = busvoltage1;
        }
        if (abs(shuntvoltage1 - lastShuntvoltage1) > 0.1)
        {
        send(lipoShuntVoltMsg.set(shuntvoltage1, 1));
        lastShuntvoltage1 = shuntvoltage1;
        }
        if (abs(loadvoltage1 - lastLoadvoltage1) > 0.1)
        {
        send(lipoLoadVoltMsg.set(loadvoltage1, 1));
        lastLoadvoltage1 = loadvoltage1;
        }
        if (abs(current_mA1 - lastCurrent_mA1) > 0.1)
        {
        send(lipoCurrentMsg.set(current_mA1, 1));
        lastCurrent_mA1 = current_mA1;
        }
        if (abs(lipoCharging - lastLipoCharging) > 0.1)
        {
        send(lipoChargingMsg.set(lipoCharging, 1));
        lastLipoCharging = lipoCharging;
        }

      // Solar Channel
        if (abs(busvoltage2 - lastBusvoltage2) > 0.1)
        {
        send(solarBusVoltMsg.set(busvoltage2, 1));
        lastBusvoltage2 = busvoltage2;
        }
        if (abs(shuntvoltage2 - lastShuntvoltage2) > 0.1)
        {
        send(solarShuntVoltMsg.set(shuntvoltage2, 1));
        lastShuntvoltage2 = shuntvoltage2;
        }
        if (abs(loadvoltage2 - lastLoadvoltage2) > 0.1)
        {
        send(solarLoadVoltMsg.set(loadvoltage2, 1));
        lastLoadvoltage2 = loadvoltage2;
        }
        if (abs(current_mA2 - lastCurrent_mA2) > 0.1)
        {
        send(solarCurrentMsg.set(current_mA2, 1));
        lastCurrent_mA2 = current_mA2;
        }
        if (abs(solarActive - lastSolarActive) > 0.1)
        {
        send(solarActivegMsg.set(solarActive, 1));
        lastSolarActive = solarActive;
        }

      // Output Channel
        if (abs(busvoltage3 - lastBusvoltage3) > 0.1)
        {
        send(outputBusVoltMsg.set(busvoltage3, 1));
        lastBusvoltage3 = busvoltage3;
        }
        if (abs(shuntvoltage3 - lastShuntvoltage3) > 0.1)
        {
        send(outputShuntVoltMsg.set(shuntvoltage3, 1));
        lastShuntvoltage3 = shuntvoltage3;
        }
        if (abs(loadvoltage3 - lastLoadvoltage3) > 0.1)
        {
        send(outputLoadVoltMsg.set(loadvoltage3, 1));
        lastLoadvoltage3 = loadvoltage3;
        }
        if (abs(current_mA3 - lastCurrent_mA3) > 0.1)
        {
        send(outputCurrentMsg.set(current_mA3, 1));
        lastCurrent_mA3 = current_mA3;
        }
        if (abs(outputActive - lastOutputActive) > 0.1)
        {
        send(outputActiveMsg.set(outputActive, 1));
        lastOutputActive = outputActive;
        }
      #endif

    } else {
      #ifdef DEBUG_BAT
        //blink();
      #endif   
    #ifdef DEBUG
        Serial.println("Uplink Down ");
      #endif
    }
    #endif

  #ifdef DEBUG
    endTime = millis();
    Serial.print(" Total time(ms): ");
    Serial.println(endTime - startTime);
  #endif

    //Read current mode of radio
    // byte rfmOpMode = readRegister(0x01);

  #ifdef LOWPOWER_SLEEP

    // Sleeping Radio
    writeRegister(0x01, 0x00);
    // Arduino Sleep
  #ifdef DEBUG
    wait(5000); //Use this to debug radio sleep only. Time is in mS
  #else
    wait(10); //Some settle time before sleeping
    LowPower.deepSleep(sleepTime);
    wait(10);
  #endif
    // Restoring Radio to Listen Mode
    writeRegister(0x01, 0x10);
    
    if (awakeFlag) // Flash LED twice when waking up from sleep.
    {
      #ifdef DEBUG_BAT
        blink2();
      #endif
      awakeFlag = false;
      
    }
    #endif
  }
  
  // End of Loop
}

void awake()
{
  awakeFlag = true;
  // This function will be called once on device wakeup
  // You can do some little operations here (like changing variables which will be used in the loop)
  // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
}

//Read from RFM69HW register.  Not used
/*
byte readRegister(byte dataToSend)
{
  byte result;   // result to return
  // take the chip select low to select the device:
  digitalWrite(MY_RFM69_CS_PIN, LOW);
  // send the device the register you want to read:
  SPI.transfer(dataToSend);
  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);
  // take the chip select high to de-select:
  digitalWrite(MY_RFM69_CS_PIN, HIGH);
  // return the result:
  return (result);
}
*/
 
 #ifdef LOWPOWER_SLEEP
//Sends a write command to RFM69HW
void writeRegister(byte thisRegister, byte thisValue)
{
  // now combine the register address and the write command into one byte:
  byte dataToSend = thisRegister | 0x80;
  // take the chip select low to select the device:
  digitalWrite(MY_RFM69_CS_PIN, LOW);
  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(thisValue);  //Send value to record into register
  // take the chip select high to de-select:
  digitalWrite(MY_RFM69_CS_PIN, HIGH);
}


void blink2()
{
  digitalWrite(LED_PIN, HIGH);
  
  (100);
  digitalWrite(LED_PIN, LOW);
  wait(1000);
  digitalWrite(LED_PIN, HIGH);
  wait(100);
  digitalWrite(LED_PIN, LOW);
}
void blink()
{
  digitalWrite(LED_PIN, HIGH);
  wait(100);
  digitalWrite(LED_PIN, LOW);
}
#endif